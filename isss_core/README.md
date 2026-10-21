# isss_core

Python package `isss` for the **DeepVision Interferometric Side Scan Sonar (ISSS)**, with no ROS dependency. It is the single implementation of the sensor: the ROS 2 node in [`../isss_ros2_driver`](../isss_ros2_driver) is a thin wrapper around `isss.SonarDriver`. The protocol is described in [`../docs/protocol.md`](../docs/protocol.md).

## Features

- TCP connection and register-based sonar configuration
- Streaming and decoding of interferometric sidescan packets
- Phase difference and bearing computation (interferometry)
- `SonarDriver`: read thread with reconnect, raw `.bin` logging and ping-rate statistics
- `SimulatedSonar` / `isss-sim`: a fake sonar on TCP that replays logs or makes synthetic pings
- Binary log file reading and plotting utilities

## Install

Requires [uv](https://docs.astral.sh/uv/).

```bash
cd isss_core
uv sync
```

This creates a virtual environment, installs the package in editable mode, and pulls in dev dependencies (pytest, matplotlib, numpy).

To include only the plotting extra without dev tools:

```bash
uv sync --no-default-groups --extra plot
```

Run commands inside the project environment:

```bash
uv run pytest
uv run isss-stream --ip 192.168.2.62 --count 5
uv run isss-plot /path/to/log.bin 500
```

## Quick start

### Stream live pings

```python
from isss import ISSSonar, SonarConfig

config = SonarConfig(ip="192.168.2.62", range_m=20.0, num_samples=1000)

with ISSSonar(config) as sonar:
    samples, raw_packet = sonar.read_ping()
    print(f"Received {len(raw_packet)} bytes, port RSS max={max(samples.port_rss)}")
```

Or from the command line (requires hardware on the network):

```bash
isss-stream --ip 192.168.2.62 --range 20 --count 5
```

### Read a binary log

```python
from isss.log import iter_packets_from_log

for samples, packet in iter_packets_from_log("log.bin"):
    print(max(samples.port_rss), max(samples.starboard_rss))
```

### Plot a log file

```bash
isss-plot /path/to/log.bin 500
```

### Run the driver loop

```python
from isss import SonarConfig, SonarDriver

def on_ping(ping):
    print(ping.header.ping_time_sec, max(ping.samples.port_rss))

with SonarDriver(SonarConfig(ip="192.168.2.62"), on_ping, log_dir="logs/") as driver:
    ...  # pings arrive on a background thread; driver.status() for health
```

### Bench without hardware

```bash
uv run isss-sim ../isss_data/1755610748.553636.bin   # or no argument for synthetic pings
uv run isss-stream --ip 127.0.0.1 --count 5
```

The simulator records register writes and paces pings like the device (one per
two-way travel time). It cannot tell you how the real device behaves where that
is unknown; see the open questions in the protocol doc.

## Tests

`uv run pytest` runs everything, without hardware:

- `test_protocol.py`: header and payload decoding on hand-made packets
- `test_golden.py`: decoding of 5 real pings pinned to a snapshot (bearing excluded)
- `test_driver.py`: connect, register writes, reconfigure, reconnect, logging, pacing against the simulator
- `test_packaging.py`: `pyproject.toml` (uv) and `setup.py`/`package.xml` (colcon) stay in step

## Project layout

```
src/isss/
  config.py     SonarConfig dataclass
  protocol.py   Packet parsing and constants
  sonar.py      ISSSonar TCP client
  driver.py     SonarDriver: read loop, reconnect, logging, status
  log.py        Binary log reading and writing
  sim.py        SimulatedSonar and the isss-sim command
  cli.py        isss-stream / isss-plot commands
tests/data/     5 real pings and their decoded snapshot
setup.py, package.xml, resource/   colcon (ament_python) packaging
```

## Related projects

- [`julRusVal/isss_processing`](https://github.com/julRusVal/isss_processing): manufacturer reference reader, analysis scripts, July 2024 logs
- [`ASVROS2/post_process_smarc`](https://github.com/ASVROS2/post_process_smarc): ROS post-processing and visualisation of `smarc_msgs/ISSS`

## License

MIT
