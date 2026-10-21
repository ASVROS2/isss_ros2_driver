"""Command-line tools: stream pings, write a diagnostic report, view logs or a live sonar."""

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


def _viz_imports():
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except ImportError:
        print("The diagnostic tools need numpy and matplotlib: uv sync --extra viz", file=sys.stderr)
        sys.exit(1)


def _calibration_args(parser: argparse.ArgumentParser) -> None:
    from isss.config import SonarConfig

    d = SonarConfig()
    parser.add_argument("--samples", type=int, default=None,
                        help="Samples per channel (default: from the log's sidecar, else 1000; live: 1000)")
    parser.add_argument("--segment", type=int, default=None, help="Segment of a log whose sample count changed")
    parser.add_argument("--sound-velocity", type=float, default=d.sound_velocity, help="m/s, for slant range")
    parser.add_argument("--k", type=float, default=d.counts_per_unit_sine, help="Phase model K (counts per unit sine)")
    parser.add_argument("--boresight", type=float, default=d.boresight_depression_deg, help="Phase model boresight depression [deg]")
    parser.add_argument("--flip-port", action="store_true",
                        help="Port transducer mounted upside down: mirror its returns about the boresight")
    parser.add_argument("--flip-starboard", action="store_true", help="The same for starboard")


def d_samples() -> int:
    from isss.config import SonarConfig

    return SonarConfig().num_samples


def _config_from(args: argparse.Namespace, **extra):
    from isss.config import SonarConfig

    return SonarConfig(num_samples=args.samples or d_samples(), sound_velocity=args.sound_velocity,
                       counts_per_unit_sine=args.k, boresight_depression_deg=args.boresight,
                       port_phase_sign=-1 if args.flip_port else 1,
                       starboard_phase_sign=-1 if args.flip_starboard else 1, **extra)


def _load_arrays(path: Path, args: argparse.Namespace):
    """Pings of a .bin log, an exported .nc, or a rosbag2 bag of the old driver."""
    from collections import Counter

    from isss import netcdf
    from isss.diagnostics import decode_packets, load_log

    if path.suffix == ".nc":
        try:
            return netcdf.read_netcdf(path)
        except ImportError:
            raise ValueError("reading .nc files needs the export extra: uv sync --extra export")
    if netcdf.is_bag(path):
        packets = [p for _, p in netcdf.bag_packets(path, getattr(args, "topic", None))]
        if not packets:
            raise ValueError(f"{path} has no ISSS pings")
        size = Counter(len(p) for p in packets).most_common(1)[0][0]
        return decode_packets([p for p in packets if len(p) == size], (size - 12) // 6)
    return load_log(path, num_samples=args.samples, segment=args.segment)


def main_export(argv: list[str] | None = None) -> None:
    """Export .bin logs or rosbag2 bags of the old driver to NetCDF4."""
    parser = argparse.ArgumentParser(
        description="Export ISSS pings to NetCDF4 (one self-describing .nc per log; open with xarray). "
        "Inputs are .bin logs (with their .json sidecar if present) or rosbag2 directories "
        "recorded by the old C++ driver (smarc_msgs/ISSS).")
    parser.add_argument("inputs", nargs="+", type=Path, help=".bin logs or rosbag2 bag directories")
    parser.add_argument("-o", "--out", type=Path, default=None,
                        help="Output .nc (one input) or directory (default: next to each input)")
    parser.add_argument("--topic", default=None, help="ISSS topic in a bag (default: the only smarc_msgs/ISSS one)")
    _calibration_args(parser)
    args = parser.parse_args(argv)
    try:
        from isss import netcdf
    except ImportError:
        print("isss-export needs the export extra (h5netcdf, h5py, numpy): uv sync --extra export", file=sys.stderr)
        sys.exit(1)

    config = _config_from(args)
    failed = False
    for path in args.inputs:
        out = args.out
        if out is not None and (len(args.inputs) > 1 or out.is_dir()):
            out = out / (path.name + ".nc" if netcdf.is_bag(path) else path.with_suffix(".nc").name)
        try:
            if netcdf.is_bag(path):
                written = netcdf.export_bag(path, out, config, topic=args.topic)
            else:
                written = netcdf.export_bin(path, out, config)
        except ImportError:
            print("isss-export needs the export extra (h5netcdf, h5py, numpy): uv sync --extra export", file=sys.stderr)
            sys.exit(1)
        except (OSError, ValueError) as exc:
            print(f"{path}: {exc}", file=sys.stderr)
            failed = True
            continue
        for p in written:
            print(p)
        if not written:
            print(f"{path}: no pings", file=sys.stderr)
    sys.exit(1 if failed else 0)


def main_report(argv: list[str] | None = None) -> None:
    """Write diagnostic figures and health checks for a recorded .bin log."""
    _viz_imports()
    import json

    import matplotlib

    matplotlib.use("Agg")
    from isss.diagnostics import health
    from isss.viz import report_figures

    parser = argparse.ArgumentParser(description="Diagnostic report for an ISSS .bin log")
    parser.add_argument("file", type=Path, help=".bin log, exported .nc, or rosbag2 bag of the old driver")
    parser.add_argument("-o", "--out", type=Path, help="Output directory (default: <log>_report/)")
    _calibration_args(parser)
    args = parser.parse_args(argv)

    config = _config_from(args)
    try:
        arrays = _load_arrays(args.file, args)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
    if arrays.n_pings == 0:
        print(f"No complete {arrays.num_samples}-sample packets in {args.file}", file=sys.stderr)
        sys.exit(1)
    out = args.out or args.file.with_name(args.file.stem + "_report")
    out.mkdir(parents=True, exist_ok=True)

    report = health(arrays, config)
    for stem, fig in report_figures(arrays, config).items():
        fig.savefig(out / f"{stem}.png", dpi=110)
    from isss.sidecar import read_sidecar

    summary = {"log": str(args.file), "sidecar": read_sidecar(args.file), **report.to_dict(), "config": {
        "sound_velocity": config.sound_velocity,
        "counts_per_unit_sine": config.counts_per_unit_sine,
        "boresight_depression_deg": config.boresight_depression_deg,
        "port_phase_sign": config.port_phase_sign,
        "starboard_phase_sign": config.starboard_phase_sign,
    }}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (out / "README.md").write_text(_report_markdown(args.file, report))

    print(f"{arrays.n_pings} pings, {report.worst.upper()}")
    for c in report.checks:
        print(f"  {c.status.upper():4}  {c.name:22} {c.value}" + (f"  ({c.detail})" if c.detail and c.status == "warn" else ""))
    print(f"Report: {out}/README.md")


def _report_markdown(log: Path, report) -> str:
    rows = "\n".join(f"| {c.status.upper()} | {c.name} | {c.value} | {c.detail} |" for c in report.checks)
    figures = "\n\n".join(f"### {title}\n\n![{title}]({stem}.png)" for stem, title in (
        ("waterfalls", "Waterfalls"),
        ("sensor_profile", "Sensor-frame profile (a flat seabed is a horizontal band)"),
        ("phase_vs_range", "Phase against range"),
        ("phase_histograms", "Phase distributions (weak samples show bias states)"),
        ("timing", "Ping interval"),
    ))
    return (f"# ISSS diagnostic report: `{log.name}`\n\n{report.n_pings} pings of {report.num_samples} samples. "
            f"Overall: **{report.worst.upper()}**.\n\n| Status | Check | Value | Detail |\n|---|---|---|---|\n"
            f"{rows}\n\n{figures}\n")


def main_view(argv: list[str] | None = None) -> None:
    """Interactive viewer for a .bin log, or a live sonar (``--ip``)."""
    _viz_imports()
    parser = argparse.ArgumentParser(
        description="View an ISSS log or a live sonar. Click the waterfall or use the slider to pick a ping; "
        "arrow keys step, p toggles phase, f follows the newest ping (live), "
        "1/2 flip the port/starboard angle (an upside-down transducer).")
    parser.add_argument("file", nargs="?", type=Path,
                        help=".bin log, exported .nc, or rosbag2 bag of the old driver")
    parser.add_argument("--ip", help="Connect to a sonar (or isss-sim) instead of reading a file")
    parser.add_argument("--port", type=int, default=57349)
    parser.add_argument("--range", type=float, default=20.0, dest="range_m", help="Range to configure (live)")
    parser.add_argument("--keep", type=int, default=600, help="Pings to keep in the live waterfall")
    _calibration_args(parser)
    args = parser.parse_args(argv)
    if (args.file is None) == (args.ip is None):
        parser.error("give either a log file or --ip")

    import matplotlib.pyplot as plt

    from isss.viz import PingViewer

    if args.file is not None:
        config = _config_from(args)
        try:
            arrays = _load_arrays(args.file, args)
        except ValueError as exc:
            parser.error(str(exc))
        viewer = PingViewer(arrays, config, title=args.file.name)
        plt.show()
        return

    from isss.diagnostics import PacketBuffer
    from isss.driver import SonarDriver

    config = _config_from(args, ip=args.ip, port=args.port, range_m=args.range_m)
    buffer = PacketBuffer(args.keep)
    driver = SonarDriver(config, lambda ping: buffer.add(ping.raw))
    viewer = PingViewer(buffer.arrays(), config, title=f"{args.ip}:{args.port}", live=True)

    def refresh() -> None:
        if len(buffer):
            viewer.set_arrays(buffer.arrays())

    timer = viewer.fig.canvas.new_timer(interval=250)
    timer.add_callback(refresh)
    timer.start()
    driver.start()
    try:
        plt.show()
    finally:
        driver.stop()
