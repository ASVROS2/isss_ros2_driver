# One ping from the DeepVision interferometric sidescan sonar (ISSS).
# Every per-sample array has one entry per range bin, nearest first.
# Protocol and phase model: docs/protocol.md in the isss_ros2_driver repository.

uint8 PACKET_TYPE_ISSS = 225          # 0xE1
int32 PHASE_COUNTS_PER_CYCLE = 20480  # phase counts in one electrical cycle (2 pi)

# stamp: device ping time (+ offset) or receive time, see the driver's use_device_time
# frame_id: the sensor frame; depression angles are measured from its horizontal plane
std_msgs/Header header

# From the packet header
uint8 packet_type       # PACKET_TYPE_ISSS
uint32 ping_time        # device ping counter [100 us]; wraps after ~4.97 days
uint8 frequency_id
uint16 decimation       # sample interval = decimation * 3 / 12.5 MHz

# Configuration in effect for this ping
float32 range                  # configured range [m]
float32 sample_interval        # time between range bins [s], from decimation
float32 sound_velocity         # [m/s]; slant range of bin i is about i * sample_interval * sound_velocity / 2
float32 counts_per_unit_sine   # K in the phase model below
float32 boresight_depression   # beta0 in the phase model below [rad]

# Echo intensity (RSS)
uint8[] port_intensity
uint8[] starboard_intensity

# Interferometric phase difference between the two receivers of each side,
# as signed hardware counts. Values normally lie in [-10240, 10240).
int16[] port_phase_counts
int16[] starboard_phase_counts

# Depression angle of the return below the horizontal of frame_id, toward that
# side [rad], from counts = K * sin(beta0 - depression). NaN where |counts| > K.
# Returns more than asin(10240 / K) from beta0 (about 42 deg) alias; nadir does.
float32[] port_depression
float32[] starboard_depression
