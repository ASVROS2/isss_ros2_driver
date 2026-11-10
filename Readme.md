# isss_ros2_driver

Driver for the **DeepVision Interferometric Side Scan Sonar (ISSS)**, split so the
sensor can be developed and bench-tested without ROS:

| Directory | What | Build |
|---|---|---|
| [`isss_core/`](isss_core) | Python package `isss`: protocol, TCP client, driver loop (reconnect, logging, statistics), log reader, simulator, diagnostic report and viewer, command-line tools. **No ROS dependency.** | `uv` on any machine; also a colcon `ament_python` package |
| [`isss_proc/`](isss_proc) | Python package `isss_proc`: processing. Filter stages (intensity, phase bias states, first surface, MPAA) over a log or live pings, `isss-proc`, `isss-proc-view`. **No ROS dependency.** Structure and plan: [`docs/processing.md`](docs/processing.md) | `uv`; also colcon `ament_python` |
| [`isss_ros2_driver/`](isss_ros2_driver) | Thin rclpy node around `isss.SonarDriver`: parameters, `isss_msgs/ISSS`, status, services; `isss_cloud_node` (filtered clouds) and `isss_ros_view` (live viewer of the topic) | colcon |
| [`isss_msgs/`](isss_msgs) | `ISSS.msg` (one ping), `ISSSStatus.msg`, `ConfigureSonar.srv` | colcon |
| [`docs/protocol.md`](docs/protocol.md) | The protocol, log files, the phase-to-depression model, what has been checked against data, and open questions | |
| [`docs/bench-testing.md`](docs/bench-testing.md) | How to bench-test a sonar with `isss-bench`, and what to check by hand | |
| [`docs/netcdf-format.md`](docs/netcdf-format.md) | The NetCDF4 export: layout, where files come from, how time is set | |
| [`docs/processing.md`](docs/processing.md) | How processing is structured, which recordings are read, the plan | |
| `isss_data/` | Three raw logs from the stick (Aug 2025) | |
| [`legacy/`](legacy) | The previous C++ driver, kept for reference; not built | |

All sensor behaviour lives in `isss_core`; the ROS node only translates. A change
to how the sonar is handled goes in `isss_core` with a test, and the node picks it
up.

## Connecting the sonar

| | |
|---|---|
| Sonar IP | **`192.168.2.62`** |
| Port | `57349`, TCP |
| Subnet | `192.168.2.0/24` (netmask `255.255.255.0`) |

The sonar sits at a fixed address, so the host has to come to it. Give the
network port the sonar is plugged into a **static** address on `192.168.2.0/24`
that is not `.62`, for example `192.168.2.10/24`, with no gateway. A port that
already carries another network (a vessel LAN, say) needs its own subnet, so use
a separate port or a USB Ethernet adapter. Example with NetworkManager:

```bash
nmcli connection add type ethernet ifname <iface> con-name isss ipv4.method manual ipv4.addresses 192.168.2.10/24
nmcli connection up isss
ping 192.168.2.62
```

Wait about a minute after power-on: ping can answer before TCP connections do.
`isss-stream`, `isss-bench` and the node's `ip` parameter default to this
address; pass another one only if your unit has been reconfigured. The viewers
go live only when given an address, so type it there: `isss-view --ip 192.168.2.62`. More in [`docs/bench-testing.md`](docs/bench-testing.md)
and [`docs/protocol.md`](docs/protocol.md).

## Bench work (no ROS)

```bash
cd isss_core
uv sync
uv run pytest                                   # protocol, golden data, driver vs simulator
uv run isss-sim ../isss_data/1755610748.553636.bin   # pretend to be the sonar on 127.0.0.1:57349
uv run isss-stream --ip 127.0.0.1 --count 5     # in another shell; or --ip 192.168.2.62 for the real one
uv run isss-report ../isss_data/1755610748.553636.bin   # figures + health checks
uv run isss-view ../isss_data/1755610748.553636.bin     # interactive; or --ip for a live sonar
uv run isss-export ../isss_data/1755610748.553636.bin   # NetCDF4 for analysis, see docs/netcdf-format.md
uv run isss-export rosbag2_2026_11_09-*/                # a bag of isss/raw (MCAP or sqlite3) -> .nc
uv run isss-bench --ip 192.168.2.62                     # bench test of a connected sonar, see docs/bench-testing.md

cd ../isss_proc && uv sync
uv run isss-proc-view ../isss_data/1755610748.553636.bin  # the viewer with the processing filters (or --ip)
```

See [`isss_core/README.md`](isss_core/README.md) for the Python API.

## ROS 2

```bash
cd ~/ros2_ws/src && git clone https://github.com/ASVROS2/isss_ros2_driver.git
cd ~/ros2_ws && rosdep install --from-paths src/isss_ros2_driver --ignore-src -y   # pulls python3-h5py
colcon build --packages-up-to isss_ros2_driver
source install/setup.bash
ros2 launch isss_ros2_driver isss_driver.launch.py
```

Test the node against the simulator (no sonar needed):

```bash
colcon test --packages-select isss_core isss_proc isss_ros2_driver && colcon test-result --verbose
```

### Launch arguments

```bash
ros2 launch isss_ros2_driver isss_driver.launch.py namespace:=isss_stick ip:=192.168.2.62 range:=30.0 save_path:=/tmp/isss_logs/
```

`namespace` prefixes all relative topics and services; `ip`, `range` and
`save_path` override the YAML. `config_file` swaps the whole YAML.
`cloud:=true` also starts the processing node (`isss_cloud_node`, below), with
the parameters of `cloud_config_file`.

[`launch/isss.launch.yaml`](isss_ros2_driver/launch/isss.launch.yaml) does the
same in YAML, with every parameter of both nodes written inline, as a template
to copy into a vehicle bringup. Arguments: `namespace`, `ip`, `range`,
`save_path`, `frame_id`, `cloud` (default `true`), and the mount:
`parent_frame` with `x y z` [m] and `roll pitch yaw` [rad]. With a
`parent_frame`, a `static_transform_publisher` places `frame_id` on it; empty
(the default) publishes no transform, for vehicles whose description already does.

```bash
ros2 launch isss_ros2_driver isss.launch.yaml namespace:=isss_stick save_path:=/tmp/isss_logs/
ros2 launch isss_ros2_driver isss.launch.yaml parent_frame:=base_link x:=0.5 z:=-0.3
```

### Published topics

| Topic (relative) | Default FQN | Type | Description |
|---|---|---|---|
| `isss/raw` | `/isss/raw` | `isss_msgs/ISSS` | One ping: intensity, phase counts, depression angles |
| `isss/status` | `/isss/status` | `isss_msgs/ISSSStatus` | Connection state, ping rate, configuration (1 Hz) |
| `/diagnostics` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Driver health |
| `isss/cloud` | `/isss/cloud` | `sensor_msgs/PointCloud2` | From `isss_cloud_node`: the returns of one ping the filters keep, in the ping's `frame_id` and stamp |

### Recording for offline work

`isss/raw` is unfiltered and carries everything needed to rebuild the data:
each message turns back into the sonar's packet byte for byte, except header
bytes 2–3, which are constant (6007) and of unknown meaning
([protocol.md](docs/protocol.md#open-questions)). Phase is the signed hardware
counts, with nothing lost. Every message also holds the configuration it was
taken with (range, sample interval, sound speed, K, boresight, phase signs).
`num_phases` is only in `isss/status`, and the mount only in `/tf_static`, so
record those too:

```bash
ros2 bag record /isss/raw /isss/status /tf_static
isss-export rosbag2_*/          # -> .nc, calibration from the messages
```

`header.stamp` is the host's receive time and `ping_time` the sonar's counter;
`isss-export` fits one to the other. With `save_path` set, the driver also
writes the `.bin` logs, the lossless record of the packets themselves.

### Services

Names are relative, so they sit under the launch `namespace` next to the topics (`/<namespace>/isss/configure_sonar`).

| Service | Type | Description |
|---|---|---|
| `isss/configure_sonar` | `isss_msgs/ConfigureSonar` | Runtime range / sample / phase configuration |
| `isss/start_logging` | `std_srvs/Trigger` | Start a new raw `.bin` log in `save_path` |
| `isss/stop_logging` | `std_srvs/Trigger` | Stop logging |

```bash
ros2 service call /isss/configure_sonar isss_msgs/srv/ConfigureSonar "{range: 30.0, num_samples: 1000, num_phases: 2}"
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
| `save_path` | string | `""` | Directory for `.bin` logs (each with a `.json` sidecar); empty disables logging |
| `export_netcdf` | bool | `true` | Also write `<log>.nc` ([format](docs/netcdf-format.md)) when each log closes; needs `h5netcdf` (pip) |
| `frame_id` | string | `isss_link` | Frame for published messages |
| `sound_velocity` | double | `1440.9` | Sound speed (m/s), reported in the message for slant range |
| `counts_per_unit_sine` | double | `15218.0` | Interferometer scale K; see [phase model](docs/protocol.md#phase-and-depression) |
| `boresight_depression_deg` | double | `44.5` | Depression of zero phase (includes the mount tilt) |
| `port_phase_sign` | int | `1` | `-1` if the port transducer is mounted upside down; see [phase model](docs/protocol.md#phase-and-depression) |
| `starboard_phase_sign` | int | `1` | The same for starboard |
| `use_device_time` | bool | `false` | `true`: stamp with the device ping counter (uptime since the sonar booted, not wall time). `false`: receive time on the host; the device counter stays in `ping_time`. Bag exports assume `false` |
| `device_time_offset_sec` | double | `0.0` | Added to device time when stamping |
| `reconnect_interval_sec` | double | `5.0` | Delay between reconnect attempts |
| `recv_timeout_sec` | int | `2` | Socket receive timeout |

### Processing node (`isss_cloud_node`)

Subscribes to `isss/raw` and runs the filter chain of [`isss_proc`](isss_proc)
(the one `isss-proc` uses) on the pings, with the calibration each message
carries. Pings are processed `block` at a time, each with up to `context`
pings before it, and published as one `PointCloud2` per ping on `isss/cloud`:
fields `x` (0: no motion), `y` (across-track, port negative), `z` (up),
`intensity`, `range` (slant, m) as float32 and `side` (0 port, 1 starboard) as
uint8. TF places each cloud with the ping's stamp. A block waits for `block`
pings, so the clouds lag by up to that many pings.

```bash
ros2 run isss_ros2_driver isss_cloud_node --ros-args -p mpaa:=true
```

Defined in [`isss_ros2_driver/config/isss_cloud.yaml`](isss_ros2_driver/config/isss_cloud.yaml);
the filters are described in [`isss_proc`](isss_proc/README.md#stages).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `block` | int | `5` | Pings processed at a time (the chain takes ~50 ms per 200-ping window) |
| `context` | int | `200` | Pings of history each block is processed with |
| `min_range` | double | `1.0` | Drop bins nearer than this [m] |
| `max_range` | double | `0.0` | Drop bins farther than this [m]; `0` = no limit |
| `min_excess` | double | `30.0` | Intensity above the water column to keep a return; `0` disables the stage |
| `bias_states` | bool | `true` | Drop phase bias states |
| `first_surface` | bool | `true` | Drop returns behind the first surface |
| `mpaa` | bool | `false` | Add the most-probable-angle filter at the end |
| `mpaa_pings` | int | `1` | Pings pooled per MPAA bin |
| `mpaa_band` | double | `3.0` | Keep returns this close to the bin's mode [deg] |

### Utility: live viewer (`isss_ros_view`)

The `isss-view` window on a running driver's `isss/raw` topic. It is a
subscriber, so it runs next to the driver and never opens its own connection to
the sonar (`isss-view --ip` does, and must not run while the driver is
connected). The phase model comes from the messages; keys and panels are those
of `isss-view` ([`isss_core`](isss_core/README.md#diagnose-a-log-or-a-live-sonar)).
It is a diagnostic tool, not part of the launch.

```bash
ros2 run isss_ros2_driver isss_ros_view                                 # /isss/raw
ros2 run isss_ros2_driver isss_ros_view --filters --min-excess 20       # with the isss-proc-view filter overlay
ros2 run isss_ros2_driver isss_ros_view --ros-args -r __ns:=/isss_stick  # a namespaced driver
```

| Option | Default | Description |
|---|---|---|
| `--topic` | `isss/raw` | ISSS topic; relative names follow the node namespace |
| `--keep` | `600` | Pings kept in the waterfall |
| `--filters` | off | Overlay the processing filters, as `isss-proc-view` does; the filter options (`--min-excess`, `--mpaa`, ...) are those of `isss-proc-view`, listed by `--help` |
| `--trail` | `50` | With `--filters`: pings shown in the profile |

The three viewers share one window and live loop (`isss.viz.run_live`) and
differ only in where pings come from:

| Command | Source | Use |
|---|---|---|
| `isss-view <log>` / `isss-proc-view <log>` | `.bin`, `.nc` or a bag of `isss/raw`; old-driver bags: `isss-view-legacy` (`isss-proc-view` takes both) | Offline |
| `isss-view --ip` / `isss-proc-view --ip` | TCP to the sonar (`isss.diagnostics.SonarSource`) | Bench, **without** the ROS driver |
| `isss_ros_view` | `isss/raw` topic (`isss_ros2_driver.view_node.TopicSource`) | With the ROS driver running |

If it keeps printing `Still nothing on /isss/raw`, the viewer and driver are on
different middlewares or domains (`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`).

### Message (`isss_msgs/ISSS`)

See [`isss_msgs/msg/ISSS.msg`](isss_msgs/msg/ISSS.msg). Per ping: the packet header
fields, the configuration in effect (range, sample interval, sound speed, and
the phase-model K, β0 and per-side phase sign), and per range bin and side the intensity, the signed
phase counts, and the depression angle below the horizontal of `frame_id`
(NaN where the model has no solution). Depression aliases for returns more than
about 42° from the boresight, nadir included; see the protocol doc.

## Migrating from the C++ driver (≤ 0.1.0)

The C++ source is kept in [`legacy/cpp_driver/`](legacy/cpp_driver) and under the tag
`cpp-driver-0.1.0`. Topic names, services and the `.bin` log format are unchanged. What differs:

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
- Services moved from `<namespace>/isss_ros2_driver_node/<name>` to `<namespace>/isss/<name>`,
  beside the topics.
- `ros2 run isss_ros2_driver plot_isss.py` is replaced by `isss-report` and `isss-view` from `isss_core`.
- Bags of the old driver open with `isss-view-legacy` and `isss-export-legacy`;
  `isss-view` and `isss-export` read this driver's `.bin` logs and `isss/raw` bags.

With `save_path` set, logging now starts at launch even if the first connection
attempt fails (the C++ node only started it after a successful first connect).

## License

MIT
