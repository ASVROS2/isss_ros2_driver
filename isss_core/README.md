# isss_core

Python package `isss` for the **DeepVision Interferometric Side Scan Sonar (ISSS)**, with no ROS dependency. It is the single implementation of the sensor: the ROS 2 node in [`../isss_ros2_driver`](../isss_ros2_driver) is a thin wrapper around `isss.SonarDriver`. The protocol is described in [`../docs/protocol.md`](../docs/protocol.md).

## Features

- TCP connection and register-based sonar configuration
- Streaming and decoding of interferometric sidescan packets
- Phase decoding and the calibrated phase-to-depression model (interferometry)
- `SonarDriver`: read thread with reconnect, raw `.bin` logging and ping-rate statistics
- `SimulatedSonar` / `isss-sim`: a fake sonar on TCP that replays logs or makes synthetic pings
- Binary log reading and writing, with a JSON sidecar per log (`isss/sidecar.py`)
- `isss-bench`: bench test of a connected sonar ([../docs/bench-testing.md](../docs/bench-testing.md))
- Diagnostics: `isss-report` (figures and health checks for a log) and `isss-view` (interactive viewer for a log or a live sonar)

## Install

Requires [uv](https://docs.astral.sh/uv/).

```bash
cd isss_core
uv sync
```

This creates a virtual environment, installs the package in editable mode, and pulls in dev dependencies (pytest, matplotlib, numpy).

To include only the diagnostics extra (numpy, matplotlib) without dev tools:

```bash
uv sync --no-default-groups --extra viz
```

Run commands inside the project environment:

```bash
uv run pytest
uv run isss-stream --ip 192.168.2.62 --count 5
uv run isss-report /path/to/log.bin
uv run isss-view /path/to/log.bin
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

### Diagnose a log or a live sonar

```bash
isss-report log.bin            # figures + health checks in log_report/ (README.md, summary.json, PNGs)
isss-view log.bin              # interactive: waterfall + selected ping
isss-view --ip 192.168.2.62    # the same, live (or --ip 127.0.0.1 with isss-sim running)
```

The health checks warn about what we have seen go wrong: ping interval not
matching the two-way travel time for the configured range, gaps in the ping
counter, phase outside the ±10240 encoding the depression model assumes (as in
the July 2024 logs), almost no echoes (out of the water), strong returns with no
depression solution, and saturation.

The figures:

| Figure | What to look for |
|---|---|
| Waterfalls (intensity, phase) | Port mirrored left, starboard right. Seabed and targets; phase should be smooth where returns are strong. |
| Sensor-frame profile | Strong returns of all pings placed by range and depression, no motion applied. **A flat seabed is a horizontal band**; tilt or curvature points at the phase model or the mount. Dashed: boresight; dotted: the ±42° unambiguous sector, outside which returns alias (nadir does). |
| Phase vs range | Strong-return phase density per side; a geometric return traces a curve, a stuck channel a flat band. |
| Phase distributions | Weak samples collapse to a few fixed values ("bias states"); strong ones should not. |
| Ping interval | Flat at the expected value; spikes are dropped pings. |

In `isss-view`: click the waterfall or drag the slider to pick a ping, ←/→
step by one, ↑/↓ by ten, `p` toggles intensity/phase, `f` follows the newest
ping when live. All tools take `--k`, `--boresight` and `--sound-velocity` to try
another calibration.

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
- `test_golden.py`: decoding of 5 real pings pinned to a snapshot
- `test_protocol.py` also covers the phase model: cycle, round trip, aliasing, and agreement with sonar3d_reference
- `test_driver.py`: connect, register writes, reconfigure, reconnect, logging, pacing against the simulator
- `test_diagnostics.py`, `test_viz.py`: the vectorised decoder matches the reference one; the checks flag shifted phase, gaps, out-of-water logs; a flat bottom comes out flat; figures, viewer and live buffer work headless
- `test_sidecar.py`: sidecars describe their logs, segments on reconfiguration, older logs still load
- `test_bench.py`: the bench passes on the simulator and catches a device that ignores a register
- `test_packaging.py`: `pyproject.toml` (uv) and `setup.py`/`package.xml` (colcon) stay in step

## Project layout

```
src/isss/
  config.py     SonarConfig dataclass
  protocol.py   Packet parsing and constants
  sonar.py      ISSSonar TCP client
  driver.py     SonarDriver: read loop, reconnect, logging, status
  log.py        Binary log reading and writing
  sidecar.py    The JSON sidecar next to each log
  bench.py      isss-bench
  sim.py        SimulatedSonar and the isss-sim command
  diagnostics.py  numpy arrays of pings, health checks, sensor-frame points
  viz.py        report figures and the interactive viewer (matplotlib)
  cli.py        isss-stream / isss-report / isss-view commands
tests/data/     5 real pings and their decoded snapshot
setup.py, package.xml, resource/   colcon (ament_python) packaging
```

## Related projects

- [`julRusVal/isss_processing`](https://github.com/julRusVal/isss_processing): manufacturer reference reader, analysis scripts, July 2024 logs
- [`ASVROS2/post_process_smarc`](https://github.com/ASVROS2/post_process_smarc): ROS post-processing and visualisation of `smarc_msgs/ISSS`

## License

MIT
