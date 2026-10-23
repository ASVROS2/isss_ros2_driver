"""Command-line tools: ``isss-proc`` (filter a recording), ``isss-proc-view`` (the viewer, filtered)
and ``isss-proc-crosstalk`` (measure and show the cross talk between the sides)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isss.cli import calibration_args, config_from, load_arrays, run_viewer, view_args


def filter_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("filters (defaults: the chain measured on the Beckholmen dock mesh)")
    g.add_argument("--min-range", type=float, default=1.0, help="Drop bins nearer than this [m]")
    g.add_argument("--max-range", type=float, default=None, help="Drop bins farther than this [m]")
    g.add_argument("--min-excess", type=float, default=30.0,
                   help="Intensity above the water column to keep a return; 0 disables")
    g.add_argument("--crosstalk", action="store_true",
                   help="Drop samples explained by the other side's leak (parked: today's curve flags port's nadir)")
    g.add_argument("--crosstalk-margin", type=float, default=20.0,
                   help="Flag samples within this of the expected leak [intensity counts]")
    g.add_argument("--no-bias-states", action="store_true", help="Keep phase bias states")
    g.add_argument("--no-first-surface", action="store_true", help="Keep returns behind the first surface")
    g.add_argument("--mpaa", action="store_true", help="Add the most-probable-angle filter at the end")
    g.add_argument("--mpaa-pings", type=int, default=1, help="Pings pooled per MPAA bin")
    g.add_argument("--mpaa-band", type=float, default=3.0, help="Keep returns this close to the bin's mode [deg]")
    g.add_argument("--context", type=int, default=200,
                   help="Pings of history each ping is processed with, as live")


def pipeline_from(args: argparse.Namespace):
    from isss_proc.filters import BiasStates, CrossTalk, FirstSurface, IntensityExcess, MostProbableAngle, RangeGate
    from isss_proc.pipeline import Pipeline

    stages = [RangeGate(args.min_range, args.max_range)]
    if args.min_excess > 0:
        stages.append(IntensityExcess(args.min_excess))
    if args.crosstalk:
        stages.append(CrossTalk(margin=args.crosstalk_margin))
    if not args.no_bias_states:
        stages.append(BiasStates())
    if not args.no_first_surface:
        stages.append(FirstSurface())
    if args.mpaa:
        stages.append(MostProbableAngle(pings=args.mpaa_pings, band_deg=args.mpaa_band))
    return Pipeline(stages)


def main_view(argv: list[str] | None = None) -> None:
    """``isss-view`` with the filters: the profile shows the kept returns of the last pings."""
    from isss_proc.pipeline import viewer_keep

    parser = argparse.ArgumentParser(
        description="View an ISSS log or a live sonar with the processing filters applied. "
        "Keys as in isss-view; the profile shows what the filters keep (grey: strong returns they drop).")
    view_args(parser)
    filter_args(parser)
    parser.add_argument("--trail", type=int, default=50, help="Pings shown in the profile")
    parser.add_argument("--clean-crosstalk", action="store_true",
                        help="Show the intensity with cross talk set to the water-column level (files only; "
                        "parked: today's curve removes port's nadir)")
    args = parser.parse_args(argv)
    keep = viewer_keep(pipeline_from(args), args.context)
    if not args.clean_crosstalk:
        run_viewer(parser, args, keep=keep, trail=args.trail)
        return
    if args.file is None:
        parser.error("--clean-crosstalk needs a file")

    import matplotlib.pyplot as plt

    from isss.viz import PingViewer
    from isss_proc.filters import CrossTalk

    config = config_from(args)
    try:
        arrays = load_arrays(args.file, args)
    except ValueError as exc:
        parser.error(str(exc))
    arrays = cleaned(arrays, config, CrossTalk(margin=args.crosstalk_margin))
    PingViewer(arrays, config, title=f"{args.file.name} (cross talk removed)", keep=keep, trail=args.trail)
    plt.show()


def cleaned(arrays, config, stage, block: int = 2000):
    """``arrays`` with the samples ``stage`` (a ``CrossTalk``) flags set to the water-column level."""
    import numpy as np

    from isss.diagnostics import PingArrays
    from isss_proc.filters import clean_intensity
    from isss_proc.swath import Swath

    parts = []
    for start in range(0, arrays.n_pings, block):
        swath = Swath(arrays.select(slice(start, start + block)), config)
        parts.append(clean_intensity(swath, stage.flags(swath), window_m=stage.window_m, smooth=stage.smooth).intensity)
    intensity = np.concatenate(parts) if parts else arrays.intensity
    return PingArrays(arrays.ping_time, arrays.decimation, arrays.size_field, intensity, arrays.counts)


def main_process(argv: list[str] | None = None) -> None:
    """Filter a recording and report what each stage dropped; optionally save the cloud."""
    import numpy as np

    from isss_proc.filters import CrossTalk
    from isss_proc.pipeline import Stream
    from isss_proc.swath import Swath

    parser = argparse.ArgumentParser(description="Filter an ISSS recording (.bin, .nc or old-driver bag)")
    parser.add_argument("file", type=Path)
    parser.add_argument("-o", "--out", type=Path, help="Save the kept returns (sensor frame) to this .npz")
    parser.add_argument("--labels", type=Path,
                        help="Save per-sample labels to this .npz: which stage dropped each sample "
                        "(rejected_by), and the cross-talk score")
    parser.add_argument("--block", type=int, default=500, help="Pings processed at a time")
    calibration_args(parser)
    filter_args(parser)
    args = parser.parse_args(argv)

    config = config_from(args)
    try:
        arrays = load_arrays(args.file, args)
    except ValueError as exc:
        parser.error(str(exc))
    if arrays.n_pings == 0:
        print(f"No pings in {args.file}", file=sys.stderr)
        sys.exit(1)

    stream = Stream(config, pipeline_from(args), context=args.context)
    totals: dict[str, dict[str, int]] = {}
    clouds, labels = [], []
    crosstalk = next((s for s in stream.pipeline.stages if isinstance(s, CrossTalk)),
                     CrossTalk(margin=args.crosstalk_margin))
    for start in range(0, arrays.n_pings, args.block):
        result = stream.push(arrays.select(slice(start, start + args.block)))
        for side, row in result.summary().items():
            acc = totals.setdefault(side, dict.fromkeys(row, 0))
            for key, value in row.items():
                acc[key] += value
        if args.out:
            pts = result.points()
            pts["ping"] = pts["ping"] + start
            clouds.append(pts)
        if args.labels:
            labels.append((result.rejected_by, crosstalk.score(result.swath)))

    print(f"{arrays.n_pings} pings of {arrays.num_samples} samples; stages: {', '.join(stream.pipeline.names)}")
    for side, row in totals.items():
        n = max(row["samples"], 1)
        print(f"  {side:9}" + "  ".join(f"{k} {v / n:6.1%}" for k, v in row.items() if k != "samples"))
    if args.out:
        cloud = {k: np.concatenate([c[k] for c in clouds]) for k in clouds[0]}
        np.savez_compressed(args.out, **cloud)
        print(f"{cloud['y'].size} points -> {args.out}")
    if args.labels:
        np.savez_compressed(
            args.labels,
            rejected_by=np.concatenate([r for r, _ in labels]),
            stages=np.array(["kept", *stream.pipeline.names]),
            crosstalk_score=np.concatenate([c for _, c in labels]),
            ping_time=arrays.ping_time,
            slant_range=Swath(arrays.select(slice(0, 1)), config).slant_range,
        )
        print(f"labels for {arrays.n_pings} pings -> {args.labels} (rejected_by: 0 kept, k stage k, 255 no solution)")


def main_crosstalk(argv: list[str] | None = None) -> None:
    """Measure the leak between the sides, and show what ``CrossTalk`` flags."""
    import numpy as np

    from isss_proc.filters import SIDES, CrossTalk, estimate_leak
    from isss_proc.swath import Swath

    parser = argparse.ArgumentParser(
        description="Cross talk between the ISSS sides: the leak curve measured in each side's water column, "
        "the share of samples CrossTalk flags, and a figure to check them.")
    parser.add_argument("file", type=Path)
    parser.add_argument("-o", "--out", type=Path, help="Save the figure here (default: show it)")
    parser.add_argument("--pings", default=None, help="Pings shown in the waterfall, A:B (default: 600 mid-file)")
    parser.add_argument("--max-pings", type=int, default=8000,
                        help="Measure the leak on at most this many pings, spread over the file")
    parser.add_argument("--estimate", action="store_true", help="Flag with the measured curves, not the defaults (check the curves first)")
    parser.add_argument("--margin", type=float, default=20.0, help="CrossTalk margin [intensity counts]")
    calibration_args(parser)
    args = parser.parse_args(argv)

    config = config_from(args)
    try:
        arrays = load_arrays(args.file, args)
    except ValueError as exc:
        parser.error(str(exc))
    if arrays.n_pings == 0:
        print(f"No pings in {args.file}", file=sys.stderr)
        sys.exit(1)

    sample = Swath(arrays.select(slice(None, None, max(1, -(-arrays.n_pings // args.max_pings)))), config)
    est = estimate_leak(sample)
    stage = CrossTalk(margin=args.margin)
    if args.estimate:
        stage.leak = tuple(e["curve"] or d for e, d in zip(est.values(), stage.leak))
    print(f"{args.file.name}: {arrays.n_pings} pings, leak measured on {sample.n_pings}")
    for side, name in enumerate(SIDES):
        other = SIDES[1 - side]
        print(f"  into {name}, in its water column, by {other} intensity: share leaking, leak above the water column")
        print("    " + "  ".join(f"{lo:.0f}:{share:4.0%} {e:3.0f}" for lo, share, e, n in est[name]["table"]
                                 if n >= 100 and lo >= 150))
        curve = stage.leak[side]
        print(f"    curve used: {', '.join(f'{x:.0f}->{y:.0f}' for x, y in curve) if curve else 'none'}")

    flags, score = stage.evaluate(sample)
    rng = sample.slant_range
    bands = np.linspace(0.0, rng[-1] + 1e-9, 5)
    for side, name in enumerate(SIDES):
        per_band = [flags[:, (rng >= a) & (rng < b), side].mean() for a, b in zip(bands[:-1], bands[1:])]
        print(f"  flagged {name:9} {flags[:, :, side].mean():6.2%}   by range "
              + "  ".join(f"{a:.0f}-{b:.0f} m {f:6.2%}" for a, b, f in zip(bands[:-1], bands[1:], per_band)))

    import matplotlib

    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if args.pings:
        a, b = (int(v) if v else None for v in args.pings.split(":"))
        shown = slice(a, b)
    else:
        mid = arrays.n_pings // 2
        shown = slice(max(0, mid - 300), mid + 300)
    seg = Swath(arrays.select(shown), config)
    seg_flags = stage.flags(seg)
    clean = cleaned(seg.arrays, config, stage)
    p0 = shown.indices(arrays.n_pings)[0]
    extent = [p0, p0 + seg.n_pings, 0, rng[-1]]

    fig, ax = plt.subplots(2, 3, figsize=(17, 9))
    ax[0, 0].imshow(seg.intensity[:, :, 1].T, aspect="auto", origin="lower", cmap="gray", extent=extent)
    ax[0, 0].set_title("starboard intensity")
    ax[0, 1].imshow(seg.intensity[:, :, 0].T, aspect="auto", origin="lower", cmap="gray", extent=extent)
    ax[0, 1].imshow(np.ma.masked_where(~seg_flags[:, :, 0].T, seg_flags[:, :, 0].T), aspect="auto", origin="lower",
                    cmap="autumn", alpha=0.6, extent=extent)
    ax[0, 1].set_title("port intensity, flagged in colour")
    ax[0, 2].imshow(clean.intensity[:, :, 0].T, aspect="auto", origin="lower", cmap="gray", extent=extent)
    ax[0, 2].set_title("port intensity, cross talk set to the water column")
    for a in ax[0]:
        a.set_xlabel("ping")
        a.set_ylabel("slant range [m]")

    for side, name in enumerate(SIDES):
        t = [r for r in est[name]["table"] if r[3] >= 100 and r[1] >= 0.2]
        if t:
            ax[1, 0].plot([r[0] + 5 for r in t], [r[2] for r in t], "o-", label=f"into {name}, measured (share >= 20 %)")
        if stage.leak[side]:
            x, y = np.asarray(stage.leak[side]).T
            ax[1, 0].plot(x, y, "k--" if side == 0 else "k:", label=f"into {name}, used")
    ax[1, 0].axhline(stage.leak_min, color="grey", lw=0.5)
    ax[1, 0].set_xlabel("other side's intensity (smoothed)")
    ax[1, 0].set_ylabel("leak above this side's water column")
    ax[1, 0].set_title("leak curve")
    ax[1, 0].legend(fontsize=8)

    smooth, base = stage.levels(sample)
    h, _, _ = np.histogram2d(smooth[:, :, 1].ravel(), (smooth[:, :, 0] - base[:, None, 0]).ravel(),
                             bins=[128, 100], range=[[0, 256], [-40, 160]])
    ax[1, 1].imshow(np.log1p(h).T, origin="lower", extent=[0, 256, -40, 160], aspect="auto")
    if stage.leak[0]:
        x, y = np.asarray(stage.leak[0]).T
        ax[1, 1].plot(x, y, "w--", lw=1)
        ax[1, 1].plot(x, y + stage.margin, "w:", lw=1)
        ax[1, 1].plot(x, y - stage.margin, "w:", lw=1)
    ax[1, 1].set_xlabel("starboard intensity (smoothed)")
    ax[1, 1].set_ylabel("port excess")
    ax[1, 1].set_title("all samples: leak \u00b1 margin")

    counts = sample.counts[:, :, 0]
    quiet = np.isnan(score[:, :, 0]) & (smooth[:, :, 0] - base[:, None, 0] < 5)
    edges = np.linspace(-10240, 10240, 129)
    for sel, label in ((flags[:, :, 0], "flagged"), (quiet, "noise (no leak, no return)")):
        if sel.any():
            ax[1, 2].hist(counts[sel], edges, density=True, histtype="step", label=label)
    ax[1, 2].set_xlabel("port phase counts")
    ax[1, 2].set_title("port phase")
    ax[1, 2].legend(fontsize=8)
    fig.suptitle(args.file.name)
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=90)
        print(f"figure -> {args.out}")
    else:
        plt.show()
