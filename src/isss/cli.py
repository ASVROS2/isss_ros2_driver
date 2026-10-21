"""Command-line tools for streaming and plotting ISSS data."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main_stream(argv: list[str] | None = None) -> None:
    """Connect to the sonar and print ping metadata."""
    from isss import ISSSonar, SonarConfig

    parser = argparse.ArgumentParser(description="Stream pings from a DeepVision ISSS")
    parser.add_argument("--ip", default="192.168.2.62", help="Sonar IP address")
    parser.add_argument("--port", type=int, default=57349, help="Sonar TCP port")
    parser.add_argument("--range", type=float, default=20.0, dest="range_m", help="Range in metres")
    parser.add_argument("--samples", type=int, default=1000, help="Samples per channel")
    parser.add_argument("--count", type=int, default=10, help="Number of pings to print")
    args = parser.parse_args(argv)

    config = SonarConfig(
        ip=args.ip,
        port=args.port,
        range_m=args.range_m,
        num_samples=args.samples,
    )

    try:
        with ISSSonar(config) as sonar:
            for i, (samples, packet) in enumerate(sonar.stream_pings()):
                if i >= args.count:
                    break
                print(
                    f"ping {i}: {len(packet)} bytes, "
                    f"port RSS max={max(samples.port_rss)}, "
                    f"stbd RSS max={max(samples.starboard_rss)}"
                )
    except (ConnectionError, OSError, TimeoutError) as exc:
        print(f"Connection failed: {exc}", file=sys.stderr)
        sys.exit(1)


def main_plot(argv: list[str] | None = None) -> None:
    """Plot phase and intensity from a recorded .bin log."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Install plot extras: uv sync --extra plot", file=sys.stderr)
        sys.exit(1)

    from isss.log import find_packet_offsets
    from isss.protocol import BYTES_PER_SAMPLE, HEADER_SIZE

    parser = argparse.ArgumentParser(description="Plot ISSS binary log data")
    parser.add_argument("file", type=Path, help="Path to .bin log file")
    parser.add_argument(
        "num_pings",
        nargs="?",
        type=int,
        default=1000,
        help="Number of pings to plot (default: 1000)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=1000,
        help="Samples per channel (default: 1000)",
    )
    args = parser.parse_args(argv)

    data = args.file.read_bytes()
    num_samples = args.samples
    bytes_per_ping = HEADER_SIZE + BYTES_PER_SAMPLE * num_samples
    total_pings = len(data) // bytes_per_ping
    nbr_pings = min(args.num_pings, total_pings)

    print(f"total pings: {total_pings}")
    print(f"plotting: {nbr_pings}")

    index_list = find_packet_offsets(
        data,
        num_samples=num_samples,
        max_offset=bytes_per_ping * nbr_pings,
    )
    if not index_list:
        print("No valid ISSS packets found", file=sys.stderr)
        sys.exit(1)

    print(f"found packets: {len(index_list)}")

    sss_port = []
    sss_stbd = []
    phase_port = []
    phase_stbd = []

    for i in index_list[:nbr_pings]:
        sonar_data = data[i + HEADER_SIZE : i + HEADER_SIZE + BYTES_PER_SAMPLE * num_samples]
        sss_port.append(list(sonar_data[2::6]))
        sss_stbd.append(list(sonar_data[5::6]))
        phase_port.append(256 * sonar_data[0] + sonar_data[1])
        phase_stbd.append(256 * sonar_data[3] + sonar_data[4])

    sss_port_arr = np.array(sss_port)
    sss_stbd_arr = np.array(sss_stbd)
    phase_port_arr = np.array(phase_port)
    phase_stbd_arr = np.array(phase_stbd)

    idx = min(10, len(sss_port_arr) - 1)

    fig, (int_port, int_stbd) = plt.subplots(1, 2)
    int_port.imshow(sss_port_arr, cmap="gray", vmax=sss_port_arr.max())
    int_port.set_title("Port intensity")
    int_stbd.imshow(sss_stbd_arr, cmap="gray", vmax=sss_stbd_arr.max())
    int_stbd.set_title("Starboard intensity")

    fig, (ax_phase_port, ax_phase_stbd) = plt.subplots(1, 2, sharey=True)
    ax_phase_port.scatter(np.arange(num_samples), phase_port_arr[idx], s=1)
    ax_phase_port.set_title("Port phase")
    ax_phase_stbd.scatter(np.arange(num_samples), phase_stbd_arr[idx], s=1)
    ax_phase_stbd.set_title("Starboard phase")

    plt.show()
