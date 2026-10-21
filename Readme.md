# isss_ros2_driver

Driver for the **DeepVision Interferometric Side Scan Sonar (ISSS)**, split so the
sensor can be developed and bench-tested without ROS:

| Directory | What | Build |
|---|---|---|
| [`isss_core/`](isss_core) | Python package `isss`: protocol, TCP client, driver loop (reconnect, logging, statistics), log reader, simulator, command-line tools. **No ROS dependency.** | `uv` on any machine; also a colcon `ament_python` package |
| [`isss_ros2_driver/`](isss_ros2_driver) | Thin rclpy node around `isss.SonarDriver`: parameters, `smarc_msgs/ISSS`, status, services | colcon |
| [`isss_ros2_driver_interfaces/`](isss_ros2_driver_interfaces) | `ISSSStatus.msg`, `ConfigureSonar.srv` | colcon |
| [`docs/protocol.md`](docs/protocol.md) | The protocol, what has been checked against data, and open questions | |
| `isss_data/` | Three raw logs from the stick (Aug 2025) | |

All sensor behaviour lives in `isss_core`; the ROS node only translates. A change
to how the sonar is handled goes in `isss_core` with a test, and the node picks it
up.

## Bench work (no ROS)

```bash
cd isss_core
uv sync
uv run pytest                                   # protocol, golden data, driver vs simulator
uv run isss-sim ../isss_data/1755610748.553636.bin   # pretend to be the sonar on 127.0.0.1:57349
uv run isss-stream --ip 127.0.0.1 --count 5     # in another shell; or --ip 192.168.2.62 for the real one
uv run isss-plot ../isss_data/1755610748.553636.bin 100
```

See [`isss_core/README.md`](isss_core/README.md) for the Python API.

## ROS 2

Needs [`smarc_msgs`](https://github.com/ASVROS2/smarc_msgs) (with `ISSS.msg`) in the workspace.

```bash
cd ~/ros2_ws/src && git clone https://github.com/ASVROS2/isss_ros2_driver.git
cd ~/ros2_ws && colcon build --packages-up-to isss_ros2_driver
source install/setup.bash
ros2 launch isss_ros2_driver isss_driver.launch.py
```

Test the node against the simulator (no sonar needed):

```bash
colcon test --packages-select isss_core isss_ros2_driver && colcon test-result --verbose
```

### Launch arguments

```bash
ros2 launch isss_ros2_driver isss_driver.launch.py namespace:=isss_stick ip:=192.168.2.62 range:=30.0 save_path:=/tmp/isss_logs/
```

`namespace` prefixes all relative topics and services; `ip`, `range` and
`save_path` override the YAML. `config_file` swaps the whole YAML.

### Published topics

| Topic (relative) | Default FQN | Type | Description |
|---|---|---|---|
| `isss/raw` | `/isss/raw` | `smarc_msgs/ISSS` | Interferometric sidescan ping data |
| `isss/status` | `/isss/status` | `isss_ros2_driver_interfaces/ISSSStatus` | Connection state, ping rate, configuration (1 Hz) |
| `/diagnostics` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health |

### Services

| Service | Type | Description |
|---|---|---|
| `~/configure_sonar` | `isss_ros2_driver_interfaces/ConfigureSonar` | Runtime range / sample / phase configuration |
| `~/start_logging` | `std_srvs/Trigger` | Start a new raw `.bin` log in `save_path` |
| `~/stop_logging` | `std_srvs/Trigger` | Stop logging |

```bash
ros2 service call /isss_ros2_driver_node/configure_sonar isss_ros2_driver_interfaces/srv/ConfigureSonar "{range: 30.0, num_samples: 1000, num_phases: 2}"
```

### Parameters

Defined in [`isss_ros2_driver/config/isss_driver.yaml`](isss_ros2_driver/config/isss_driver.yaml).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ip` | string | `192.168.2.62` | Sonar IP address |
| `port` | int | `57349` | Sonar TCP port (`0xE005`) |
| `range` | double | `20.0` | Range in metres |
| `num_samples` | int | `1000` | Samples per channel |
| `num_phases` | int | `2` | Number of phases (`0` = off) |
| `save_path` | string | `""` | Directory for `.bin` logs; empty disables logging |
| `frame_id` | string | `isss_link` | Frame for published messages |
| `sound_velocity` | double | `1440.9` | Sound speed (m/s) for bearing |
| `carrier_frequency` | double | `12500000.0` | Used for bearing; see open question 1 in the protocol doc |
| `baseline` | double | `0.0032` | Receiver baseline (m) |
| `use_device_time` | bool | `true` | Stamp messages with the device ping counter |
| `device_time_offset_sec` | double | `0.0` | Added to device time when stamping |
| `reconnect_interval_sec` | double | `5.0` | Delay between reconnect attempts |
| `recv_timeout_sec` | int | `2` | Socket receive timeout |

### Message fields (`smarc_msgs/ISSS`)

- `type`, `time`, `frequency_id`, `decimation`: from the packet header
- `frequency`, `baseline`, `sound_velocity`: from parameters
- `port_channel`, `starboard_channel`: RSS intensity per sample
- `port_channel_phase_difference`, `starboard_channel_phase_difference`: radians
- `port_channel_bearing`, `starboard_channel_bearing`: **known to be wrong**, see [`docs/protocol.md`](docs/protocol.md#open-questions)
- `extra_channel`: the raw 12-byte header

## Migrating from the C++ driver (≤ 0.1.0)

Topic names, parameters, services and the `.bin` log format are unchanged. Two
things differ:

- The message and service types moved from `isss_ros2_driver` to
  `isss_ros2_driver_interfaces` (e.g. `isss_ros2_driver_interfaces/srv/ConfigureSonar`).
- `ros2 run isss_ros2_driver plot_isss.py` is replaced by `isss-plot` from `isss_core`.

With `save_path` set, logging now starts at launch even if the first connection
attempt fails (the C++ node only started it after a successful first connect).

## License

MIT
