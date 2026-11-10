# Driver health and the sonar configuration in effect, published at 1 Hz.
std_msgs/Header header
bool connected
float32 ping_rate_hz   # pings received per second
string last_error      # empty when healthy
float64 range          # configured range [m]
int32 num_samples      # range bins per channel
int32 num_phases       # phase register (0x12); not in isss_msgs/ISSS, so record this topic too
uint16 decimation      # decimation register (0x10)
