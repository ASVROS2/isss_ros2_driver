"""Command-line tools: stream pings, write a diagnostic report, view or export recordings or a live sonar.

``isss-view`` and ``isss-export`` read what this driver records: ``.bin`` logs,
exported ``.nc`` files and rosbag2 bags of ``isss_msgs/ISSS`` (MCAP or sqlite3).
``isss-view-legacy`` and ``isss-export-legacy`` read bags of the old C++ driver
(``smarc_msgs/ISSS``).
"""

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


def calibration_args(parser: argparse.ArgumentParser) -> None:
    """Options for reading logs and for the phase model; ``config_from`` turns them into a ``SonarConfig``.

    Unset options keep the recording's calibration (a bag of this driver carries
    it in every message), else the ``SonarConfig`` defaults.
    """
    from isss.config import SonarConfig

    d = SonarConfig()
    parser.add_argument("--samples", type=int, default=None,
                        help="Samples per channel (default: from the log's sidecar, else 1000; live: 1000)")
    parser.add_argument("--segment", type=int, default=None, help="Segment of a log whose sample count changed")
    parser.add_argument("--sound-velocity", type=float, default=None,
                        help=f"m/s, for slant range (default: from the recording, else {d.sound_velocity})")
    parser.add_argument("--k", type=float, default=None,
                        help=f"Phase model K, counts per unit sine (default: from the recording, else {d.counts_per_unit_sine})")
    parser.add_argument("--boresight", type=float, default=None,
                        help=f"Phase model boresight depression [deg] (default: from the recording, else {d.boresight_depression_deg})")
    parser.add_argument("--flip-port", action="store_true",
                        help="Port transducer mounted upside down: mirror its returns about the boresight "
                        "(relative to the recording's sign)")
    parser.add_argument("--flip-starboard", action="store_true", help="The same for starboard")


def d_samples() -> int:
    from isss.config import SonarConfig

    return SonarConfig().num_samples


def config_from(args: argparse.Namespace, base=None, **extra):
    """The ``SonarConfig`` of the calibration options, on top of ``base`` (a recording's) or the defaults."""
    import dataclasses

    from isss.config import SonarConfig

    base = base or SonarConfig()
    changes = {
        "num_samples": args.samples or base.num_samples,
        "port_phase_sign": -base.port_phase_sign if args.flip_port else base.port_phase_sign,
        "starboard_phase_sign": -base.starboard_phase_sign if args.flip_starboard else base.starboard_phase_sign,
    }
    for option, field in (("sound_velocity", "sound_velocity"), ("k", "counts_per_unit_sine"),
                          ("boresight", "boresight_depression_deg")):
        if getattr(args, option) is not None:
            changes[field] = getattr(args, option)
    return dataclasses.replace(base, **changes, **extra)


def load_arrays(path: Path, args: argparse.Namespace):
    """Pings of a .bin log, an exported .nc, or a rosbag2 bag of either driver (``isss.sources``)."""
    return load_recording(path, args)[0]


def load_recording(path: Path, args: argparse.Namespace):
    """``(pings, config)`` of a recording: the calibration options on top of what it carries."""
    from isss.sources import load_recording as load

    arrays, recorded = load(path, num_samples=args.samples, segment=args.segment, topic=getattr(args, "topic", None))
    return arrays, config_from(args, base=recorded)


def _check_input(path: Path, legacy: bool) -> str | None:
    """Why ``path`` belongs to the other command (current vs legacy), else None."""
    from isss.ros2bag import bag_kind, is_bag

    if not path.exists():
        return "no such file or directory"
    kind = bag_kind(path) if is_bag(path) else None
    if legacy and kind != "legacy":
        return ("not a bag of the old C++ driver (smarc_msgs/ISSS); "
                "this driver's recordings open with the command without -legacy")
    if not legacy and kind == "legacy":
        return "a bag of the old C++ driver (smarc_msgs/ISSS): use the -legacy command"
    return None


def main_export(argv: list[str] | None = None, *, legacy: bool = False) -> None:
    """Export .bin logs or rosbag2 bags of this driver (or, ``legacy``, of the old driver) to NetCDF4."""
    if legacy:
        inputs = "rosbag2 directories recorded by the old C++ driver (smarc_msgs/ISSS, sqlite3)"
        msg_type = "smarc_msgs/ISSS"
    else:
        inputs = ("this driver's .bin logs (with their .json sidecar if present) and rosbag2 bags of isss/raw "
                  "(isss_msgs/ISSS, MCAP or sqlite3; calibration from the messages, time from header.stamp "
                  "fitted to the device ping counter). Bags of the old C++ driver: isss-export-legacy")
        msg_type = "isss_msgs/ISSS"
    parser = argparse.ArgumentParser(
        prog="isss-export-legacy" if legacy else "isss-export",
        description=f"Export ISSS pings to NetCDF4 (one self-describing .nc per input; open with xarray). "
        f"Inputs are {inputs}.")
    parser.add_argument("inputs", nargs="+", type=Path,
                        help="rosbag2 bag directories" if legacy else ".bin logs, rosbag2 bag directories or .mcap/.db3 files")
    parser.add_argument("-o", "--out", type=Path, default=None,
                        help="Output .nc (one input) or directory (default: next to each input)")
    parser.add_argument("--topic", default=None, help=f"ISSS topic in a bag (default: the only {msg_type} one)")
    calibration_args(parser)
    args = parser.parse_args(argv)
    try:
        from isss import netcdf
    except ImportError:
        print("isss-export needs the export extra (h5netcdf, h5py, numpy): uv sync --extra export", file=sys.stderr)
        sys.exit(1)

    config = config_from(args)
    failed = False
    for path in args.inputs:
        out = args.out
        if out is not None and (len(args.inputs) > 1 or out.is_dir()):
            out = out / (path.name + ".nc" if netcdf.is_bag(path) and path.is_dir() else path.with_suffix(".nc").name)
        try:
            problem = _check_input(path, legacy)
            if problem:
                raise ValueError(problem)
            if legacy:
                written = netcdf.export_bag(path, out, config, topic=args.topic)
            elif netcdf.is_bag(path):
                written = netcdf.export_isss_bag(path, out, topic=args.topic,
                                                 adjust=lambda recorded: config_from(args, base=recorded))
            else:
                written = netcdf.export_bin(path, out, adjust=lambda recorded: config_from(args, base=recorded))
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


def main_export_legacy(argv: list[str] | None = None) -> None:
    """``isss-export`` for rosbag2 bags of the old C++ driver (``smarc_msgs/ISSS``)."""
    main_export(argv, legacy=True)


def main_report(argv: list[str] | None = None) -> None:
    """Write diagnostic figures and health checks for a recorded .bin log."""
    _viz_imports()
    import json

    import matplotlib

    matplotlib.use("Agg")
    from isss.diagnostics import health
    from isss.viz import report_figures

    parser = argparse.ArgumentParser(description="Diagnostic report for an ISSS recording: .bin log, exported .nc, or rosbag2 bag of either driver")
    parser.add_argument("file", type=Path, help=".bin log, exported .nc, or rosbag2 bag of either driver")
    parser.add_argument("--topic", default=None, help="ISSS topic in a bag with more than one")
    parser.add_argument("-o", "--out", type=Path, help="Output directory (default: <log>_report/)")
    calibration_args(parser)
    args = parser.parse_args(argv)

    try:
        arrays, config = load_recording(args.file, args)
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


def view_args(parser: argparse.ArgumentParser) -> None:
    """The viewer's source options: a file, or ``--ip`` for a live sonar."""
    parser.add_argument("file", nargs="?", type=Path,
                        help=".bin log, exported .nc, or rosbag2 bag (directory, .mcap or .db3)")
    parser.add_argument("--topic", default=None, help="ISSS topic in a bag with more than one")
    parser.add_argument("--ip", help="Connect to a sonar (or isss-sim) instead of reading a file")
    parser.add_argument("--port", type=int, default=57349)
    parser.add_argument("--range", type=float, default=20.0, dest="range_m", help="Range to configure (live)")
    parser.add_argument("--keep", type=int, default=600, help="Pings to keep in the live waterfall")
    calibration_args(parser)


_VIEW_KEYS = ("Click the waterfall or use the slider to pick a ping; arrow keys step, p toggles phase, "
              "f follows the newest ping (live), 1/2 flip the port/starboard angle (an upside-down transducer).")


def main_view(argv: list[str] | None = None) -> None:
    """Interactive viewer for this driver's recordings (.bin, .nc, isss_msgs/ISSS bags) or a live sonar."""
    parser = argparse.ArgumentParser(
        prog="isss-view",
        description="View an ISSS recording or a live sonar: a .bin log, an exported .nc, a rosbag2 bag of "
        "isss/raw (MCAP or sqlite3; calibration from the messages), or --ip. Bags of the old C++ driver: "
        "isss-view-legacy. " + _VIEW_KEYS)
    view_args(parser)
    args = parser.parse_args(argv)
    if args.file is not None and (problem := _check_input(args.file, legacy=False)):
        parser.error(f"{args.file}: {problem}")
    run_viewer(parser, args)


def main_view_legacy(argv: list[str] | None = None) -> None:
    """Interactive viewer for rosbag2 bags of the old C++ driver (``smarc_msgs/ISSS``)."""
    from isss.config import SonarConfig

    d = SonarConfig()
    parser = argparse.ArgumentParser(
        prog="isss-view-legacy",
        description="View a rosbag2 bag of the old C++ driver (smarc_msgs/ISSS, sqlite3). Those messages do not "
        f"carry the calibration: the defaults (K {d.counts_per_unit_sine}, boresight {d.boresight_depression_deg} "
        "deg) apply unless given. " + _VIEW_KEYS)
    parser.add_argument("file", type=Path, help="rosbag2 bag directory or .db3 of the old driver")
    parser.add_argument("--topic", default=None, help="ISSS topic in a bag with more than one")
    calibration_args(parser)
    args = parser.parse_args(argv)
    if problem := _check_input(args.file, legacy=True):
        parser.error(f"{args.file}: {problem}")
    args.ip = None
    run_viewer(parser, args)


def run_viewer(parser: argparse.ArgumentParser, args: argparse.Namespace, **viewer_kwargs) -> None:
    """Open ``PingViewer`` on the file or live sonar in ``args`` (from ``view_args``).

    ``viewer_kwargs`` go to ``PingViewer``: ``isss_proc`` passes its filters as ``keep``.
    """
    _viz_imports()
    if (args.file is None) == (args.ip is None):
        parser.error("give either a log file or --ip")

    import matplotlib.pyplot as plt

    from isss.viz import PingViewer

    if args.file is not None:
        try:
            arrays, config = load_recording(args.file, args)
        except ValueError as exc:
            parser.error(str(exc))
        viewer = PingViewer(arrays, config, title=args.file.name, **viewer_kwargs)
        plt.show()
        return

    from isss.diagnostics import SonarSource
    from isss.viz import run_live

    config = config_from(args, ip=args.ip, port=args.port, range_m=args.range_m)
    run_live(SonarSource(config, args.keep), title=f"{args.ip}:{args.port}", **viewer_kwargs)
