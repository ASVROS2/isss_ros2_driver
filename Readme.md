# isss_ros2_driver

ROS 2 driver for the **DeepVision Interferometric Side Scan Sonar (ISSS)**. The driver connects over TCP, configures the sonar via its register protocol, streams interferometric sidescan data, and publishes `smarc_msgs/ISSS` messages.

Manufacturer reference implementation: [`../isss_processing/ISSSReader_ubuntu.cpp`](../isss_processing/ISSSReader_ubuntu.cpp)

## Prerequisites

- ROS 2 (Humble or newer recommended)
- [`smarc_msgs`](https://github.com/smarc-project/smarc_msgs) in your workspace
- Network access to the sonar (default IP `192.168.2.62`, port `57349`)

Configure the host machine with a static IP on the same subnet as the sonar.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select isss_ros2_driver
source install/setup.bash
```

## Run

Launch with default parameters from YAML:

```bash
ros2 launch isss_ros2_driver isss_driver.launch.py
```

Topics are published as relative names under the node namespace:

| Topic (relative) | Default fully-qualified name |
|---|---|
| `isss/raw` | `/isss/raw` |
| `isss/status` | `/isss/status` |

To namespace the driver (e.g. mount on a vehicle or payload):

```bash
ros2 launch isss_ros2_driver isss_driver.launch.py namespace:=isss_stick
```

This publishes `/isss_stick/isss/raw` and `/isss_stick/isss/status`. Services are resolved under the same namespace, e.g. `/isss_stick/isss_ros2_driver_node/configure_sonar`.

Override IP, range, or log directory at launch:

```bash
ros2 launch isss_ros2_driver isss_driver.launch.py namespace:=vehicle/payload ip:=192.168.2.62 range:=30.0 save_path:=/tmp/isss_logs/
```

Run the node directly (optionally with a namespace):

```bash
ros2 run isss_ros2_driver isss_ros2_driver_node --ros-args \
  --params-file $(ros2 pkg prefix isss_ros2_driver)/share/isss_ros2_driver/config/isss_driver.yaml \
  -r __ns:=/isss_stick
```

## ROS interface

### Published topics

All topic names are **relative** to the node namespace. With no namespace, defaults are shown below.

| Topic (relative) | Default FQN | Type | Description |
|---|---|---|---|
| `isss/raw` | `/isss/raw` | `smarc_msgs/ISSS` | Interferometric sidescan ping data |
| `isss/status` | `/isss/status` | `isss_ros2_driver/ISSSStatus` | Connection state, ping rate, configuration |
| `/diagnostics` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health (global diagnostics bus) |

### Services

All services use private names (`~/...`) and resolve under the node namespace.

| Service (relative) | Type | Description |
|---|---|---|
| `~/configure_sonar` | `isss_ros2_driver/ConfigureSonar` | Runtime range / sample / phase configuration |
| `~/start_logging` | `std_srvs/Trigger` | Start writing raw `.bin` packets to `save_path` |
| `~/stop_logging` | `std_srvs/Trigger` | Stop binary logging |

Example reconfiguration (default namespace):

```bash
ros2 service call /isss_ros2_driver_node/configure_sonar isss_ros2_driver/srv/ConfigureSonar "{range: 30.0, num_samples: 1000, num_phases: 2}"
```

With `namespace:=isss_stick`:

```bash
ros2 service call /isss_stick/isss_ros2_driver_node/configure_sonar isss_ros2_driver/srv/ConfigureSonar "{range: 30.0, num_samples: 1000, num_phases: 2}"
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ip` | string | `192.168.2.62` | Sonar IP address |
| `port` | int | `57349` | Sonar TCP port (`0xE005`) |
| `range` | double | `20.0` | Sonar range in metres |
| `num_samples` | int | `1000` | Samples per channel (register `0x11`) |
| `num_phases` | int | `2` | Number of phases (register `0x12`, `0` = off) |
| `save_path` | string | `""` | Directory for `.bin` logs; empty disables logging |
| `frame_id` | string | `isss_link` | TF frame for published messages |
| `sound_velocity` | double | `1440.9` | Sound speed (m/s) for bearing calculation |
| `carrier_frequency` | double | `12500000.0` | Carrier frequency (Hz) |
| `baseline` | double | `0.0032` | Sensor baseline (m) |
| `use_device_time` | bool | `true` | Use device ping counter for message stamp |
| `device_time_offset_sec` | double | `0.0` | Offset added to device time when stamping |
| `reconnect_interval_sec` | double | `5.0` | Delay between reconnect attempts |
| `recv_timeout_sec` | int | `2` | Socket receive timeout |

Launch argument `namespace` (empty by default) prefixes all relative topics and services. See [Run](#run) above.

Parameters are defined in [`config/isss_driver.yaml`](config/isss_driver.yaml).

## Message fields (`smarc_msgs/ISSS`)

The driver populates all primary fields from the packet header and payload:

- `type`, `time`, `frequency_id`, `decimation` — from 12-byte packet header
- `frequency`, `baseline`, `sound_velocity` — from ROS parameters
- `port_channel`, `starboard_channel` — RSS intensity per sample
- `port_channel_phase_difference`, `starboard_channel_phase_difference` — radians
- `port_channel_bearing`, `starboard_channel_bearing` — computed via interferometry
- `extra_channel` — raw 12-byte header (backward compatibility)

## Protocol reference

### Register map (documented)

| Register | Address | Description |
|---|---|---|
| Decimation | `0x10` | Sample decimation factor |
| Num samples | `0x11` | Samples per channel (default 1000) |
| Num phases | `0x12` | Phase count (default 2; 0 = off) |

Register write packet: `[0xFE, reg, value_hi, value_lo]`

Decimation formula:

```
decimation = int(range / num_samples / 750 * (12.5e6 / 3))
```

### Data packet format

**Header (12 bytes)**

| Byte | Field |
|---|---|
| 0 | `0xFE` — start of packet |
| 1 | `0x05` — start of packet |
| 2–3 | Packet size |
| 4 | Type (`0xE1` = ISSS, `0xE2` = 8-bit sonar) |
| 5–8 | Ping time (100 µs units) |
| 9 | Frequency ID (`0x01`) |
| 10–11 | Decimation |

**Payload (1000 × 6 bytes)**

Each sample: port phase [15:8], port phase [7:0], port RSS, starboard phase [15:8], starboard phase [7:0], starboard RSS.

Total packet size: 6012 bytes.

### Not implemented (requires manufacturer documentation)

- Channel registers at `0x20` / `0x30`
- 8-bit packet mode (`0xE2`)

## Binary log visualization

If `save_path` is set, raw packets are written as timestamped `.bin` files. Plot them with:

```bash
ros2 run isss_ros2_driver plot_isss.py /path/to/log.bin 500
```

## Architecture

```
DeepVision ISSS ──TCP:57349──► isss_sonar (hardware layer)
                                    │
                                    ▼
                           isss_ros2_driver_node
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
              {ns}/isss/raw    {ns}/isss/status  /diagnostics
              smarc_msgs/ISSS     ISSSStatus
```

The hardware layer (`isss_sonar.hpp/cpp`) mirrors the manufacturer reference class and has no ROS dependencies.

## License

MIT
