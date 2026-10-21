# isss_python_interface

Python library for interfacing with the **DeepVision Interferometric Side Scan Sonar (ISSS)** over TCP. This package mirrors the hardware layer from [`isss_ros2_driver`](../isss_ros2_driver) and the manufacturer reference in [`isss_processing`](../isss_processing), without ROS dependencies.

## Features

- TCP connection and register-based sonar configuration
- Streaming and decoding of interferometric sidescan packets
- Phase difference and bearing computation (interferometry)
- Binary log file reading and plotting utilities

## Install

Requires [uv](https://docs.astral.sh/uv/).

```bash
cd isss_python_interface
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

## Protocol reference

### Connection

| Parameter | Default |
|---|---|
| IP | `192.168.2.62` |
| Port | `57349` (`0xE005`) |

Configure the host with a static IP on the same subnet as the sonar.

### Register map

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

**Payload (num_samples × 6 bytes)**

Each sample: port phase [15:8], port phase [7:0], port RSS, starboard phase [15:8], starboard phase [7:0], starboard RSS.

Default packet size: 6012 bytes (1000 samples).

## Project layout

```
src/isss/
  config.py     SonarConfig dataclass
  protocol.py   Packet parsing and constants
  sonar.py      ISSSonar TCP client
  log.py        Binary log utilities
  cli.py        isss-stream / isss-plot commands
```

## Related projects

- [`isss_ros2_driver`](../isss_ros2_driver) — ROS 2 driver publishing `smarc_msgs/ISSS`
- [`isss_processing`](../isss_processing) — Manufacturer reference and analysis scripts

## License

MIT
