#ifndef ISSS_ROS2_DRIVER__ISSS_SONAR_HPP_
#define ISSS_ROS2_DRIVER__ISSS_SONAR_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace isss_ros2_driver
{

constexpr int kHeaderSize = 12;
constexpr int kDefaultSamples = 1000;
constexpr int kBytesPerSample = 6;
constexpr uint8_t kPacketTypeIsss = 0xE1;
constexpr uint8_t kFreqIdDefault = 0x01;

struct SonarConfig
{
  std::string ip = "192.168.2.62";
  int port = 57349;
  double range = 20.0;
  size_t num_samples = kDefaultSamples;
  int num_phases = 2;
  float sound_velocity = 1440.9f;
  float carrier_frequency = 12.5e6f;
  float baseline = 3.2e-3f;
  int recv_timeout_sec = 2;
};

struct PacketHeader
{
  uint8_t sop1 = 0;
  uint8_t sop2 = 0;
  uint16_t packet_size = 0;
  uint8_t type = 0;
  uint32_t ping_time = 0;
  uint8_t frequency_id = 0;
  uint16_t decimation = 0;
};

struct SampleData
{
  std::vector<uint8_t> port_rss;
  std::vector<uint8_t> starboard_rss;
  std::vector<float> port_phase_diff;
  std::vector<float> starboard_phase_diff;
  std::vector<float> port_bearing;
  std::vector<float> starboard_bearing;
};

class ISSSonar
{
public:
  explicit ISSSonar(SonarConfig config);
  ~ISSSonar();

  ISSSonar(const ISSSonar &) = delete;
  ISSSonar & operator=(const ISSSonar &) = delete;

  bool connect();
  void disconnect();
  bool is_connected() const;

  bool setup_sonar();
  bool configure(double range, size_t num_samples, int num_phases);

  bool read_packet(std::vector<uint8_t> & packet_out);

  static bool parse_header(const uint8_t * header, PacketHeader & out);
  static bool parse_payload(
    const uint8_t * payload,
    size_t num_samples,
    const SonarConfig & config,
    SampleData & out);

  int compute_decimation() const;
  int data_size() const;
  int packet_size() const;

  SonarConfig & config();
  const SonarConfig & config() const;

private:
  bool set_reg_v4(int reg, int value);

  SonarConfig config_;
  int sock_ = -1;
};

}  // namespace isss_ros2_driver

#endif  // ISSS_ROS2_DRIVER__ISSS_SONAR_HPP_
