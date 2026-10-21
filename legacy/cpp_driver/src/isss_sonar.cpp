#include "isss_ros2_driver/isss_sonar.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <cstring>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace isss_ros2_driver
{

namespace
{

constexpr int32_t kMaxPositive = 32768;
constexpr int32_t kFullRange = 65536;

float clamp_asin_input(float value)
{
  if (value > 1.0f) {
    return 1.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value;
}

float phase_to_radians(float raw_phase)
{
  return (raw_phase / 512.0f) * (static_cast<float>(M_PI) / 180.0f);
}

float compute_bearing(float raw_phase, const SonarConfig & config)
{
  const float asin_input = clamp_asin_input(
    (raw_phase * config.sound_velocity) /
    (config.carrier_frequency * config.baseline));
  return std::asin(asin_input);
}

float decode_phase(uint8_t high, uint8_t low)
{
  float phase = static_cast<float>((static_cast<uint16_t>(high) << 8) | low);
  if (phase > kMaxPositive) {
    phase -= kFullRange;
  }
  return phase;
}

}  // namespace

ISSSonar::ISSSonar(SonarConfig config)
: config_(std::move(config))
{
}

ISSSonar::~ISSSonar()
{
  disconnect();
}

SonarConfig & ISSSonar::config()
{
  return config_;
}

const SonarConfig & ISSSonar::config() const
{
  return config_;
}

int ISSSonar::compute_decimation() const
{
  return static_cast<int>(
    static_cast<float>(config_.range) / static_cast<float>(config_.num_samples) /
    750.0f * (12.5e6f / 3.0f));
}

int ISSSonar::data_size() const
{
  return static_cast<int>(config_.num_samples * kBytesPerSample);
}

int ISSSonar::packet_size() const
{
  return kHeaderSize + data_size();
}

bool ISSSonar::connect()
{
  if (is_connected()) {
    return true;
  }

  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) {
    return false;
  }

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(static_cast<uint16_t>(config_.port));
  if (inet_pton(AF_INET, config_.ip.c_str(), &server_addr.sin_addr) != 1) {
    disconnect();
    return false;
  }

  if (::connect(sock_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
    disconnect();
    return false;
  }

  timeval tv{};
  tv.tv_sec = config_.recv_timeout_sec;
  tv.tv_usec = 0;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return true;
}

void ISSSonar::disconnect()
{
  if (sock_ >= 0) {
    close(sock_);
    sock_ = -1;
  }
}

bool ISSSonar::is_connected() const
{
  return sock_ >= 0;
}

bool ISSSonar::set_reg_v4(int reg, int value)
{
  if (!is_connected()) {
    return false;
  }

  uint8_t tx_buff[4];
  tx_buff[0] = 0xFE;
  tx_buff[1] = static_cast<uint8_t>(reg & 0xFF);
  tx_buff[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  tx_buff[3] = static_cast<uint8_t>(value & 0xFF);

  const ssize_t sent = send(sock_, tx_buff, sizeof(tx_buff), 0);
  return sent == static_cast<ssize_t>(sizeof(tx_buff));
}

bool ISSSonar::setup_sonar()
{
  if (!is_connected()) {
    return false;
  }

  const int decimation = compute_decimation();
  return set_reg_v4(0x10, decimation) &&
         set_reg_v4(0x11, static_cast<int>(config_.num_samples)) &&
         set_reg_v4(0x12, config_.num_phases);
}

bool ISSSonar::configure(double range, size_t num_samples, int num_phases)
{
  config_.range = range;
  config_.num_samples = num_samples;
  config_.num_phases = num_phases;
  return setup_sonar();
}

bool ISSSonar::parse_header(const uint8_t * header, PacketHeader & out)
{
  out.sop1 = header[0];
  out.sop2 = header[1];
  out.packet_size = static_cast<uint16_t>((header[2] << 8) | header[3]);
  out.type = header[4];
  out.ping_time = static_cast<uint32_t>(
    (static_cast<uint32_t>(header[5]) << 24) |
    (static_cast<uint32_t>(header[6]) << 16) |
    (static_cast<uint32_t>(header[7]) << 8) |
    static_cast<uint32_t>(header[8]));
  out.frequency_id = header[9];
  out.decimation = static_cast<uint16_t>((header[10] << 8) | header[11]);
  return out.sop1 == 0xFE && out.sop2 == 0x05;
}

bool ISSSonar::parse_payload(
  const uint8_t * payload,
  size_t num_samples,
  const SonarConfig & config,
  SampleData & out)
{
  out.port_rss.resize(num_samples);
  out.starboard_rss.resize(num_samples);
  out.port_phase_diff.resize(num_samples);
  out.starboard_phase_diff.resize(num_samples);
  out.port_bearing.resize(num_samples);
  out.starboard_bearing.resize(num_samples);

  for (size_t i = 0; i < num_samples; ++i) {
    const size_t offset = i * kBytesPerSample;

    const float port_raw = decode_phase(payload[offset], payload[offset + 1]);
    out.port_rss[i] = payload[offset + 2];
    out.port_phase_diff[i] = phase_to_radians(port_raw);
    out.port_bearing[i] = compute_bearing(port_raw, config);

    const float starboard_raw = decode_phase(payload[offset + 3], payload[offset + 4]);
    out.starboard_rss[i] = payload[offset + 5];
    out.starboard_phase_diff[i] = phase_to_radians(starboard_raw);
    out.starboard_bearing[i] = compute_bearing(starboard_raw, config);
  }

  return true;
}

bool ISSSonar::read_packet(std::vector<uint8_t> & packet_out)
{
  if (!is_connected()) {
    return false;
  }

  const int data_size = this->data_size();
  const int packet_size = this->packet_size();

  packet_out.assign(packet_size, 0);
  uint8_t header_buffer[kHeaderSize]{};
  bool header_found = false;

  while (!header_found) {
    uint8_t byte = 0;
    const ssize_t bytes_received = recv(sock_, &byte, 1, 0);
    if (bytes_received <= 0) {
      return false;
    }

    memmove(header_buffer, header_buffer + 1, kHeaderSize - 1);
    header_buffer[kHeaderSize - 1] = byte;

    if (header_buffer[0] == 0xFE &&
      header_buffer[1] == 0x05 &&
      header_buffer[4] == kPacketTypeIsss &&
      header_buffer[9] == kFreqIdDefault)
    {
      header_found = true;
      std::memcpy(packet_out.data(), header_buffer, kHeaderSize);
    }
  }

  int total_bytes_read = 0;
  while (total_bytes_read < data_size) {
    const ssize_t bytes_received = recv(
      sock_,
      reinterpret_cast<char *>(packet_out.data() + kHeaderSize + total_bytes_read),
      data_size - total_bytes_read,
      0);
    if (bytes_received <= 0) {
      return false;
    }
    total_bytes_read += static_cast<int>(bytes_received);
  }

  return total_bytes_read == data_size;
}

}  // namespace isss_ros2_driver
