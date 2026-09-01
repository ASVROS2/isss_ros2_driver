#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "isss_ros2_driver/isss_sonar.hpp"
#include "isss_ros2_driver/msg/isss_status.hpp"
#include "isss_ros2_driver/srv/configure_sonar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "smarc_msgs/msg/isss.hpp"
#include "std_srvs/srv/trigger.hpp"

class ISSSonarNode : public rclcpp::Node
{
public:
  ISSSonarNode()
  : Node("isss_ros2_driver_node")
  {
    declare_parameters();
    load_parameters();
    sonar_ = std::make_unique<isss_ros2_driver::ISSSonar>(sonar_config_);

    isss_pub_ = create_publisher<smarc_msgs::msg::ISSS>("isss/raw", 10);
    status_pub_ = create_publisher<isss_ros2_driver::msg::ISSSStatus>("isss/status", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(1));

    configure_service_ = create_service<isss_ros2_driver::srv::ConfigureSonar>(
      "~/configure_sonar",
      std::bind(
        &ISSSonarNode::handle_configure_sonar, this,
        std::placeholders::_1, std::placeholders::_2));

    start_logging_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start_logging",
      std::bind(
        &ISSSonarNode::handle_start_logging, this,
        std::placeholders::_1, std::placeholders::_2));

    stop_logging_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop_logging",
      std::bind(
        &ISSSonarNode::handle_stop_logging, this,
        std::placeholders::_1, std::placeholders::_2));

    status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ISSSonarNode::publish_status, this));

    if (!connect_and_setup()) {
      RCLCPP_WARN(
        get_logger(),
        "Initial sonar connection failed; read loop will attempt reconnect.");
    } else if (!save_path_.empty()) {
      open_log_file();
    }

    read_thread_ = std::thread(&ISSSonarNode::read_loop, this);
  }

  ~ISSSonarNode() override
  {
    shutdown_requested_.store(true);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    close_log_file();
    if (sonar_) {
      sonar_->disconnect();
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("ip", "192.168.2.62");
    declare_parameter<int>("port", 57349);
    declare_parameter<double>("range", 20.0);
    declare_parameter<int>("num_samples", isss_ros2_driver::kDefaultSamples);
    declare_parameter<int>("num_phases", 2);
    declare_parameter<std::string>("save_path", "");
    declare_parameter<std::string>("frame_id", "isss_link");
    declare_parameter<double>("sound_velocity", 1440.9);
    declare_parameter<double>("carrier_frequency", 12500000.0);
    declare_parameter<double>("baseline", 0.0032);
    declare_parameter<int>("recv_timeout_sec", 2);
    declare_parameter<double>("device_time_offset_sec", 0.0);
    declare_parameter<bool>("use_device_time", true);
    declare_parameter<double>("reconnect_interval_sec", 5.0);
  }

  void load_parameters()
  {
    get_parameter("ip", ip_);
    get_parameter("port", port_);
    get_parameter("range", range_);
    get_parameter("num_samples", num_samples_);
    get_parameter("num_phases", num_phases_);
    get_parameter("save_path", save_path_);
    get_parameter("frame_id", frame_id_);
    get_parameter("sound_velocity", sound_velocity_);
    get_parameter("carrier_frequency", carrier_frequency_);
    get_parameter("baseline", baseline_);
    get_parameter("recv_timeout_sec", recv_timeout_sec_);
    get_parameter("device_time_offset_sec", device_time_offset_sec_);
    get_parameter("use_device_time", use_device_time_);
    get_parameter("reconnect_interval_sec", reconnect_interval_sec_);

    if (range_ <= 0.0) {
      throw std::invalid_argument("Parameter 'range' must be > 0");
    }
    if (num_samples_ <= 0) {
      throw std::invalid_argument("Parameter 'num_samples' must be > 0");
    }
    if (num_phases_ < 0) {
      throw std::invalid_argument("Parameter 'num_phases' must be >= 0");
    }
    if (port_ <= 0 || port_ > 65535) {
      throw std::invalid_argument("Parameter 'port' must be in [1, 65535]");
    }

    sonar_config_.ip = ip_;
    sonar_config_.port = port_;
    sonar_config_.range = range_;
    sonar_config_.num_samples = static_cast<size_t>(num_samples_);
    sonar_config_.num_phases = num_phases_;
    sonar_config_.sound_velocity = static_cast<float>(sound_velocity_);
    sonar_config_.carrier_frequency = static_cast<float>(carrier_frequency_);
    sonar_config_.baseline = static_cast<float>(baseline_);
    sonar_config_.recv_timeout_sec = recv_timeout_sec_;
  }

  bool connect_and_setup()
  {
    sonar_->config() = sonar_config_;
    if (!sonar_->connect()) {
      set_last_error("Failed to connect to sonar at " + ip_ + ":" + std::to_string(port_));
      set_connected(false);
      return false;
    }

    if (!sonar_->setup_sonar()) {
      set_last_error("Failed to configure sonar registers");
      sonar_->disconnect();
      set_connected(false);
      return false;
    }

    set_connected(true);
    set_last_error("");
    RCLCPP_INFO(get_logger(), "Connected to sonar at %s:%d", ip_.c_str(), port_);
    return true;
  }

  void read_loop()
  {
    std::vector<uint8_t> packet;
    packet.reserve(static_cast<size_t>(sonar_->packet_size()));

    while (rclcpp::ok() && !shutdown_requested_.load()) {
      if (!is_connected()) {
        if (!attempt_reconnect()) {
          continue;
        }
      }

      if (!sonar_->read_packet(packet)) {
        set_connected(false);
        set_last_error("Receive error or timeout from sonar");
        sonar_->disconnect();
        continue;
      }

      update_ping_rate();
      publish_packet(packet);
    }
  }

  bool attempt_reconnect()
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Sonar disconnected; retrying in %.1f s", reconnect_interval_sec_);

    const auto sleep_step = std::chrono::milliseconds(100);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(reconnect_interval_sec_);

    while (rclcpp::ok() && !shutdown_requested_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(sleep_step);
    }

    return connect_and_setup();
  }

  void publish_packet(const std::vector<uint8_t> & packet)
  {
    isss_ros2_driver::PacketHeader header{};
    if (!isss_ros2_driver::ISSSonar::parse_header(packet.data(), header)) {
      RCLCPP_WARN(get_logger(), "Received packet with invalid header sync bytes");
      return;
    }

    if (header.type != isss_ros2_driver::kPacketTypeIsss) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unexpected packet type 0x%02X (expected 0xE1)", header.type);
    }

    isss_ros2_driver::SampleData samples;
    isss_ros2_driver::ISSSonar::parse_payload(
      packet.data() + isss_ros2_driver::kHeaderSize,
      sonar_config_.num_samples,
      sonar_config_,
      samples);

    smarc_msgs::msg::ISSS msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = compute_stamp(header.ping_time);
    msg.type = header.type;
    msg.time = header.ping_time;
    msg.frequency_id = header.frequency_id;
    msg.frequency = static_cast<float>(carrier_frequency_);
    msg.decimation = header.decimation;
    msg.max_duration = static_cast<float>(range_ / static_cast<double>(num_samples_));
    msg.baseline = static_cast<float>(baseline_);
    msg.sound_velocity = static_cast<float>(sound_velocity_);
    msg.port_channel = samples.port_rss;
    msg.starboard_channel = samples.starboard_rss;
    msg.port_channel_phase_difference = samples.port_phase_diff;
    msg.starboard_channel_phase_difference = samples.starboard_phase_diff;
    msg.port_channel_bearing = samples.port_bearing;
    msg.starboard_channel_bearing = samples.starboard_bearing;
    msg.extra_channel.assign(packet.begin(), packet.begin() + isss_ros2_driver::kHeaderSize);

    write_log_packet(packet);
    isss_pub_->publish(msg);
  }

  rclcpp::Time compute_stamp(uint32_t ping_time_100us) const
  {
    if (!use_device_time_) {
      return now();
    }

    const double device_seconds =
      static_cast<double>(ping_time_100us) * 1e-4 + device_time_offset_sec_;
    return rclcpp::Time(static_cast<int64_t>(device_seconds * 1e9));
  }

  void write_log_packet(const std::vector<uint8_t> & packet)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!log_file_.is_open()) {
      return;
    }
    log_file_.write(
      reinterpret_cast<const char *>(packet.data()),
      static_cast<std::streamsize>(packet.size()));
  }

  bool open_log_file()
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (save_path_.empty()) {
      return false;
    }

    close_log_file_unsafe();

    std::string path = save_path_;
    if (path.back() != '/') {
      path += '/';
    }
    const std::string filename = path + std::to_string(now().seconds()) + ".bin";
    log_file_.open(filename, std::ios::binary);
    if (!log_file_.is_open()) {
      set_last_error("Failed to open log file: " + filename);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Saving sonar data to: %s", filename.c_str());
    logging_enabled_.store(true);
    return true;
  }

  void close_log_file()
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    close_log_file_unsafe();
  }

  void close_log_file_unsafe()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
    logging_enabled_.store(false);
  }

  void handle_configure_sonar(
    const std::shared_ptr<isss_ros2_driver::srv::ConfigureSonar::Request> request,
    std::shared_ptr<isss_ros2_driver::srv::ConfigureSonar::Response> response)
  {
    if (request->range <= 0.0 || request->num_samples <= 0 || request->num_phases < 0) {
      response->success = false;
      response->message = "Invalid configuration values";
      return;
    }

    range_ = request->range;
    num_samples_ = request->num_samples;
    num_phases_ = request->num_phases;
    sonar_config_.range = range_;
    sonar_config_.num_samples = static_cast<size_t>(num_samples_);
    sonar_config_.num_phases = num_phases_;

    if (!is_connected()) {
      response->success = false;
      response->message = "Sonar is not connected";
      return;
    }

    if (!sonar_->configure(range_, static_cast<size_t>(num_samples_), num_phases_)) {
      response->success = false;
      response->message = "Failed to write sonar configuration registers";
      set_last_error(response->message);
      return;
    }

    response->success = true;
    response->message = "Sonar configured successfully";
    RCLCPP_INFO(
      get_logger(),
      "Sonar reconfigured: range=%.1f, num_samples=%d, num_phases=%d",
      range_, num_samples_, num_phases_);
  }

  void handle_start_logging(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (save_path_.empty()) {
      response->success = false;
      response->message = "save_path parameter is empty; set it to enable logging";
      return;
    }

    response->success = open_log_file();
    response->message = response->success ? "Logging started" : last_error_;
  }

  void handle_stop_logging(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    close_log_file();
    response->success = true;
    response->message = "Logging stopped";
  }

  void publish_status()
  {
    isss_ros2_driver::msg::ISSSStatus status;
    status.header.stamp = now();
    status.header.frame_id = frame_id_;
    status.connected = is_connected();
    status.ping_rate_hz = get_ping_rate_hz();
    status.last_error = last_error_;
    status.range = range_;
    status.num_samples = num_samples_;
    status.num_phases = num_phases_;
    status.decimation = static_cast<uint16_t>(sonar_->compute_decimation());
    status_pub_->publish(status);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = status.header.stamp;

    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "isss_ros2_driver";
    diag.hardware_id = ip_;
    diag.level = status.connected ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag.message = status.connected ? "Streaming" : status.last_error;

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "ping_rate_hz";
    kv.value = std::to_string(status.ping_rate_hz);
    diag.values.push_back(kv);

    kv.key = "range_m";
    kv.value = std::to_string(status.range);
    diag.values.push_back(kv);

    kv.key = "num_samples";
    kv.value = std::to_string(status.num_samples);
    diag.values.push_back(kv);

    kv.key = "decimation";
    kv.value = std::to_string(status.decimation);
    diag.values.push_back(kv);

    diagnostics.status.push_back(diag);
    diagnostics_pub_->publish(diagnostics);
  }

  void update_ping_rate()
  {
    const auto now_time = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++ping_count_window_;
    const double elapsed = std::chrono::duration<double>(now_time - ping_window_start_).count();
    if (elapsed >= 1.0) {
      ping_rate_hz_ = static_cast<float>(ping_count_window_ / elapsed);
      ping_count_window_ = 0;
      ping_window_start_ = now_time;
    }
  }

  float get_ping_rate_hz()
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return ping_rate_hz_;
  }

  void set_connected(bool value)
  {
    connected_.store(value);
  }

  bool is_connected()
  {
    return connected_.load();
  }

  void set_last_error(const std::string & error)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    last_error_ = error;
  }

  isss_ros2_driver::SonarConfig sonar_config_;
  std::unique_ptr<isss_ros2_driver::ISSSonar> sonar_;

  rclcpp::Publisher<smarc_msgs::msg::ISSS>::SharedPtr isss_pub_;
  rclcpp::Publisher<isss_ros2_driver::msg::ISSSStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<isss_ros2_driver::srv::ConfigureSonar>::SharedPtr configure_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_logging_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_logging_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string ip_;
  int port_;
  double range_;
  int num_samples_;
  int num_phases_;
  std::string save_path_;
  std::string frame_id_;
  double sound_velocity_;
  double carrier_frequency_;
  double baseline_;
  int recv_timeout_sec_;
  double device_time_offset_sec_;
  bool use_device_time_;
  double reconnect_interval_sec_;

  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> logging_enabled_{false};

  std::thread read_thread_;
  std::ofstream log_file_;
  std::mutex log_mutex_;
  std::mutex stats_mutex_;
  std::string last_error_;

  size_t ping_count_window_{0};
  float ping_rate_hz_{0.0f};
  std::chrono::steady_clock::time_point ping_window_start_{std::chrono::steady_clock::now()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ISSSonarNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("isss_ros2_driver_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
