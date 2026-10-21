# isss_ros2_driver

Driver for the **DeepVision Interferometric Side Scan Sonar (ISSS)**, split so the
sensor can be developed and bench-tested without ROS:

| Directory | What | Build |
|---|---|---|
| [`isss_core/`](isss_core) | Python package `isss`: protocol, TCP client, driver loop (reconnect, logging, statistics), log reader, simulator, diagnostic report and viewer, command-line tools. **No ROS dependency.** | `uv` on any machine; also a colcon `ament_python` package |
| [`isss_ros2_driver/`](isss_ros2_driver) | Thin rclpy node around `isss.SonarDriver`: parameters, `isss_msgs/ISSS`, status, services | colcon |
| [`isss_msgs/`](isss_msgs) | `ISSS.msg` (one ping), `ISSSStatus.msg`, `ConfigureSonar.srv` | colcon |
| [`docs/protocol.md`](docs/protocol.md) | The protocol, the phase-to-depression model, what has been checked against data, and open questions | |
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
uv run isss-report ../isss_data/1755610748.553636.bin   # figures + health checks
uv run isss-view ../isss_data/1755610748.553636.bin     # interactive; or --ip for a live sonar
```

See [`isss_core/README.md`](isss_core/README.md) for the Python API.

## ROS 2

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
| `isss/raw` | `/isss/raw` | `isss_msgs/ISSS` | One ping: intensity, phase counts, depression angles |
| `isss/status` | `/isss/status` | `isss_msgs/ISSSStatus` | Connection state, ping rate, configuration (1 Hz) |
| `/diagnostics` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health |

### Services

| Service | Type | Description |
|---|---|---|
| `~/configure_sonar` | `isss_msgs/ConfigureSonar` | Runtime range / sample / phase configuration |
| `~/start_logging` | `std_srvs/Trigger` | Start a new raw `.bin` log in `save_path` |
| `~/stop_logging` | `std_srvs/Trigger` | Stop logging |

```bash
ros2 service call /isss_ros2_driver_node/configure_sonar isss_msgs/srv/ConfigureSonar "{range: 30.0, num_samples: 1000, num_phases: 2}"
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
| `sound_velocity` | double | `1440.9` | Sound speed (m/s), reported in the message for slant range |
| `counts_per_unit_sine` | double | `15218.0` | Interferometer scale K; see [phase model](docs/protocol.md#phase-and-depression) |
| `boresight_depression_deg` | double | `44.5` | Depression of zero phase (includes the mount tilt) |
| `use_device_time` | bool | `true` | Stamp messages with the device ping counter |
| `device_time_offset_sec` | double | `0.0` | Added to device time when stamping |
| `reconnect_interval_sec` | double | `5.0` | Delay between reconnect attempts |
| `recv_timeout_sec` | int | `2` | Socket receive timeout |

### Message (`isss_msgs/ISSS`)

See [`isss_msgs/msg/ISSS.msg`](isss_msgs/msg/ISSS.msg). Per ping: the packet header
fields, the configuration in effect (range, sample interval, sound speed, and
the phase-model K and β0), and per range bin and side the intensity, the signed
phase counts, and the depression angle below the horizontal of `frame_id`
(NaN where the model has no solution). Depression aliases for returns more than
about 42° from the boresight, nadir included; see the protocol doc.

## Migrating from the C++ driver (≤ 0.1.0)

Topic names, services and the `.bin` log format are unchanged. What differs:

- `isss/raw` publishes `isss_msgs/ISSS` instead of `smarc_msgs/ISSS`, so the
  driver no longer needs `smarc_msgs`. Field changes: `port_channel` →
  `port_intensity`; the phase is sent as signed counts (`port_phase_counts`)
  instead of `counts / 512 · π / 180` radians; the broken `*_bearing` is
  replaced by `*_depression` from the calibrated phase model; `type`/`time` →
  `packet_type`/`ping_time`; `extra_channel`, `gain`, `frequency`, `baseline`
  and `max_duration` are gone, `sample_interval` and the model parameters are new.
  Subscribers of the old type (e.g. `ASVROS2/post_process_smarc`) need updating.
- Parameters `carrier_frequency` and `baseline` are replaced by
  `counts_per_unit_sine` and `boresight_depression_deg`.
- The status message and service moved to `isss_msgs`
  (e.g. `isss_msgs/srv/ConfigureSonar`).
- `ros2 run isss_ros2_driver plot_isss.py` is replaced by `isss-report` and `isss-view` from `isss_core`.

With `save_path` set, logging now starts at launch even if the first connection
attempt fails (the C++ node only started it after a successful first connect).

## License

MIT
