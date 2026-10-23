"""Filter stages: each says which samples of a swath to keep.

A stage is called as ``stage(swath, keep, info)``. ``keep`` (n, samples, 2)
holds the samples that survived the stages before it; the stage returns its
own bool mask of the same shape, and only its verdict on samples in ``keep``
counts. Stages may record what they found in the ``info`` dict. They see one
``Swath`` at a time and keep no state between calls, so the same stage serves
a whole log at once or a rolling window of live pings (``isss_proc.pipeline``).

``RangeGate``, ``IntensityExcess``, ``BiasStates`` and ``FirstSurface`` are
the filters measured against the Beckholmen dock mesh in sonar3d_reference
(``experimental/isss/mesh_check.py``), with its defaults. ``MostProbableAngle``
is the MPAA of the 2024 post-processing scripts, on calibrated depressions.
``CrossTalk`` drops one side's echo leaking into the other; it is new and
not yet measured against the mesh, so it is not in ``default_stages()``.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from typing import Any, ClassVar, Optional

import numpy as np

from isss.diagnostics import PingArrays
from isss.protocol import PHASE_COUNTS_PER_CYCLE
from isss_proc.swath import Swath

SIDES = ("port", "starboard")
HALF_CYCLE = PHASE_COUNTS_PER_CYCLE / 2.0


@dataclass
class RangeGate:
    """Drop bins nearer than ``min_m`` (transmit ring-down) or farther than ``max_m``."""

    min_m: float = 1.0
    max_m: Optional[float] = None
    name: ClassVar[str] = "range"

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        r = swath.slant_range
        m = r >= self.min_m
        if self.max_m is not None:
            m &= r <= self.max_m
        return np.broadcast_to(m[None, :, None], keep.shape)


@dataclass
class IntensityExcess:
    """Keep returns whose smoothed intensity stands ``min_excess`` above the ping's water column.

    The water-column level is the median, per ping and side, of the intensity
    smoothed over ``smooth`` bins, between ``window_m`` of slant range. It
    assumes no return that close; with the sonar near a surface, move the window.
    On the dock, an excess of 30 kept about half of the starboard returns,
    67-69 % of them within 0.25 m of the mesh; 50-60 kept 30-40 % at 77-81 %.
    """

    min_excess: float = 30.0
    window_m: tuple[float, float] = (1.0, 2.5)
    smooth: int = 5
    name: ClassVar[str] = "intensity"

    def levels(self, swath: Swath) -> tuple[np.ndarray, np.ndarray]:
        """The intensity smoothed over ``smooth`` bins (n, samples, 2), and the water-column level (n, 2)."""
        from scipy.ndimage import uniform_filter1d

        smooth = uniform_filter1d(swath.intensity.astype(np.float64), self.smooth, axis=1)
        a = min(int(self.window_m[0] / swath.bin_size), swath.num_samples - 1)
        b = min(max(a + 5, int(self.window_m[1] / swath.bin_size)), swath.num_samples)
        return smooth, np.median(smooth[:, a:b, :], axis=1)

    def excess(self, swath: Swath) -> np.ndarray:
        """(n, samples, 2) smoothed intensity minus the water-column level."""
        smooth, base = self.levels(swath)
        return smooth - base[:, None, :]

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        return self.excess(swath) > self.min_excess


@dataclass
class CrossTalk:
    """Drop samples that are the other side's echo leaking in (cross talk between the channels).

    When one side is very strong, the other side can show it at the same ping
    and range bin. The leak is all or nothing: where it happens, it raises
    this side by a set amount above its water column, which depends on the
    other side's intensity; elsewhere it is absent. ``leak`` holds that amount
    per receiving side (into port, into starboard) as (other side's intensity,
    excess) points, interpolated, with no leak below the first point; ``None``
    for a side means no cross talk there. A sample is flagged where the
    expected leak is at least ``leak_min`` and the sample's excess lies within
    ``margin`` of it. A real return stronger than the leak is spared, and so is
    a sample well below it (no leak at that bin). Intensities are smoothed, and
    the water column taken, as in ``IntensityExcess``.

    **Parked, with an open issue: it obscures the nadir.** The default curve
    into port is the median of ``estimate_leak`` over the five Beckholmen dock
    bags of 14 Dec 2025: about 57 at a starboard intensity of 195, 85 at 235.
    But what it measured is port's own first bottom return. Both sides see the
    seabed at the same range, and port reads about 29 counts below starboard
    there. So the stage flags port's nadir line, and ``clean_intensity`` wipes
    it out. Away from the nadir, port stays at its water column under a
    saturated starboard object, and port's shapes that look like starboard's
    correlate at 0.05, so no same-bin leak is confirmed in those bags. Measure
    a real leak away from the nadir, then replace ``DEFAULT_LEAK``, before
    using this stage (docs/processing.md). Into starboard: off. ``estimate=True``
    measures the curves on the swath, with the same caveat.

    ``score`` is the sample's excess minus the expected leak, NaN where the
    expected leak is below ``leak_min``: within ``margin`` of 0 means the leak explains it.
    """

    DEFAULT_LEAK: ClassVar[tuple] = (
        ((195.0, 57.0), (205.0, 66.0), (215.0, 73.0), (225.0, 80.0), (235.0, 85.0)),
        None,
    )

    leak: tuple = DEFAULT_LEAK
    leak_min: float = 20.0
    margin: float = 20.0
    estimate: bool = False
    window_m: tuple[float, float] = (1.0, 2.5)
    smooth: int = 5
    name: ClassVar[str] = "crosstalk"

    def levels(self, swath: Swath) -> tuple[np.ndarray, np.ndarray]:
        return IntensityExcess(window_m=self.window_m, smooth=self.smooth).levels(swath)

    def curves(self, swath: Swath, levels: Optional[tuple[np.ndarray, np.ndarray]] = None) -> tuple:
        """The leak curve used for each receiving side on this swath."""
        if not self.estimate:
            return tuple(self.leak)
        est = estimate_leak(swath, levels=levels or self.levels(swath))
        return tuple(e["curve"] or d for e, d in zip(est.values(), self.leak))

    def expected(self, swath: Swath, levels: Optional[tuple[np.ndarray, np.ndarray]] = None) -> np.ndarray:
        """(n, samples, 2) the excess the other side's leak raises on each side, where it happens."""
        smooth, _ = levels = levels or self.levels(swath)
        out = np.zeros(smooth.shape)
        for side, curve in enumerate(self.curves(swath, levels)):
            if curve:
                x, y = np.asarray(curve, dtype=np.float64).T
                out[:, :, side] = np.interp(smooth[:, :, 1 - side], x, y, left=0.0)
        return out

    def evaluate(self, swath: Swath) -> tuple[np.ndarray, np.ndarray]:
        """(flags, score): which samples are cross talk, and their excess minus the expected leak."""
        smooth, base = levels = self.levels(swath)
        leak = self.expected(swath, levels)
        score = np.where(leak >= self.leak_min, smooth - base[:, None, :] - leak, np.nan)
        return np.abs(score) <= self.margin, score.astype(np.float32)

    def flags(self, swath: Swath) -> np.ndarray:
        return self.evaluate(swath)[0]

    def score(self, swath: Swath) -> np.ndarray:
        return self.evaluate(swath)[1]

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        flags = self.flags(swath)
        info["crosstalk"] = {name: float(flags[:, :, side].mean()) for side, name in enumerate(SIDES)}
        return ~flags


def estimate_leak(swath: Swath, *, levels: Optional[tuple[np.ndarray, np.ndarray]] = None,
                  present: float = 30.0, min_share: float = 0.4, bottom_excess: float = 40.0,
                  bottom_bins: int = 11, guard: int = 8, level_step: float = 10.0,
                  min_samples: int = 100) -> dict[str, dict[str, Any]]:
    """Per receiving side, the excess the other side's leak raises there, against the other side's intensity.

    Measured where this side should see only water: from 1 m out to ``guard``
    bins before its first bottom return (the first ``bottom_bins`` bins in a row
    at least ``bottom_excess`` above the water column). There, at a given
    intensity of the other side, this side is either at its water column (no
    leak) or raised by the leak. Samples at least ``present`` above the water
    column count as leaking, and their median is the leak. ``table`` has rows
    (level, share leaking, leak, samples) per ``level_step``. ``curve`` holds
    the rows with at least ``min_samples`` and a share of at least
    ``min_share``, as (level + step / 2, leak) points for ``CrossTalk.leak``,
    or None if there are none.

    The window ends at this side's first *sustained* return, so a narrow nadir
    line still counts as water. Where both sides return at the floor range,
    this side's own nadir is then measured as a leak. That is what happened on
    the Beckholmen bags (see ``CrossTalk``). Fix this before trusting the curve.
    """
    from scipy.ndimage import uniform_filter1d

    smooth, base = levels if levels is not None else IntensityExcess().levels(swath)
    excess = smooth - base[:, None, :]
    start = min(int(1.0 / swath.bin_size), swath.num_samples)
    above = uniform_filter1d((excess >= bottom_excess).astype(np.float64), bottom_bins, axis=1) > 1 - 1e-9
    above[:, :start] = False
    bottom = np.where(above.any(axis=1), above.argmax(axis=1) - bottom_bins // 2, swath.num_samples)
    idx = np.arange(swath.num_samples)[None, :]
    out = {}
    for side, name in enumerate(SIDES):
        water = (idx >= start) & (idx < bottom[:, side, None] - guard)
        other, this = smooth[:, :, 1 - side][water], excess[:, :, side][water]
        table = []
        for lo in np.arange(0.0, 256.0, level_step):
            m = (other >= lo) & (other < lo + level_step)
            if m.any():
                up = this[m][this[m] >= present]
                table.append((float(lo), float(up.size / m.sum()), float(np.median(up)) if up.size else math.nan,
                              int(m.sum())))
        curve = tuple((lo + level_step / 2, e) for lo, share, e, n in table if n >= min_samples and share >= min_share)
        out[name] = {"table": table, "curve": curve or None, "water_samples": int(water.sum())}
    return out


def clean_intensity(swath: Swath, flags: np.ndarray, *, window_m: tuple[float, float] = (1.0, 2.5),
                    smooth: int = 5) -> PingArrays:
    """A copy of the pings with the flagged samples set to their ping's water-column level."""
    _, base = IntensityExcess(window_m=window_m, smooth=smooth).levels(swath)
    a = swath.arrays
    fill = np.broadcast_to(np.round(base).astype(np.uint8)[:, None, :], a.intensity.shape)
    return PingArrays(a.ping_time, a.decimation, a.size_field, np.where(flags, fill, a.intensity), a.counts)


@dataclass
class BiasStates:
    """Drop phase values that noise collapses to ("bias states"), and the wrap edge.

    In the water column the phase does not spread evenly: it sits on a few
    narrow values, which weak returns share whatever the geometry, so they read
    as fixed bearings. The states are the peaks of the water-column phase
    histogram (``window_m``, all pings of the swath). A state is rejected only
    if it also stands out among the returns still kept: density in its band
    over the density in its two flanks at least ``min_return_ratio``. A state
    where real returns are dense anyway (at the boresight) is spared. Counts
    within ``edge_margin`` of +-10240 are dropped as well.
    """

    window_m: tuple[float, float] = (1.0, 2.5)
    max_states: int = 3
    bin_counts: float = 80.0
    min_relative_height: float = 0.2
    min_return_ratio: float = 1.3
    width: float = 1.0
    edge_margin: float = 150.0
    min_samples: int = 1000
    name: ClassVar[str] = "bias_states"

    def states(self, water_column: np.ndarray) -> list[dict[str, float]]:
        """Up to ``max_states`` peaks of the water-column histogram: centre, half width, share."""
        from scipy.ndimage import gaussian_filter1d
        from scipy.signal import find_peaks, peak_widths

        wc = water_column[np.isfinite(water_column)]
        if wc.size < self.min_samples:
            return []
        edges = np.arange(-HALF_CYCLE, HALF_CYCLE + self.bin_counts, self.bin_counts)
        h, _ = np.histogram(wc, bins=edges)
        hs = gaussian_filter1d(h.astype(np.float64), 1.0)
        peaks, props = find_peaks(hs, height=self.min_relative_height * hs.max(), prominence=0.1 * hs.max())
        if peaks.size == 0:
            return []
        peaks = peaks[np.argsort(props["peak_heights"])[::-1][: self.max_states]]
        widths = peak_widths(hs, peaks, rel_height=0.5)[0] * self.bin_counts
        out = []
        for c, w in zip(edges[peaks] + self.bin_counts / 2.0, widths):
            out.append({"center_counts": float(c), "half_width_counts": float(w / 2.0),
                        "water_column_share": float(np.mean(np.abs(wc - c) <= w / 2.0))})
        return out

    @staticmethod
    def return_peak_ratio(state: dict[str, float], returns: np.ndarray) -> float:
        """Density of returns in the state's band over the mean density in its two flanks."""
        hw, centre = state["half_width_counts"], state["center_counts"]
        if returns.size == 0:
            return 0.0
        band = np.mean(np.abs(returns - centre) <= hw)
        flanks = [np.mean(np.abs(returns - f) <= hw) for f in (centre - 3.0 * hw, centre + 3.0 * hw)
                  if abs(f) + hw < HALF_CYCLE]
        return float(band / np.mean(flanks)) if flanks and np.mean(flanks) > 0 else math.inf

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        r = swath.slant_range
        in_window = (r >= self.window_m[0]) & (r <= self.window_m[1])
        counts = swath.counts.astype(np.float64)
        mask = np.abs(counts) < HALF_CYCLE - self.edge_margin
        found = {}
        for side, name in enumerate(SIDES):
            c = counts[:, :, side]
            states = self.states(c[:, in_window].ravel())
            returns = c[keep[:, :, side]]
            for st in states:
                st["return_peak_ratio"] = self.return_peak_ratio(st, returns)
                st["rejected"] = st["return_peak_ratio"] >= self.min_return_ratio
                if st["rejected"]:
                    mask[:, :, side] &= np.abs(c - st["center_counts"]) > self.width * st["half_width_counts"]
            found[name] = states
        info["bias_states"] = found
        return mask


@dataclass
class FirstSurface:
    """Drop returns behind the first surface seen at the same angle (multipath, wrapped phase).

    Per side, kept returns are grouped by ``ping_block`` pings and by
    depression bins of ``angle_bin_deg``. A group's first surface is the
    nearest range where ``cluster_n`` returns lie within ``cluster_m``; returns
    more than ``margin_m`` beyond it are dropped. Groups with no such cluster
    are kept whole.

    With ``bin_extent`` (the default) the margin also covers the range a flat
    seabed spans within one angle bin, ``r * bin / tan(depression)`` (the
    depression floored at ``min_depression_deg``). Without it, as in
    sonar3d_reference, a seabed at shallow grazing angles comes out striped:
    at 10 m range and 29 deg a 2 deg bin spans 0.64 m, and the far part of
    every bin is dropped. Multipath ghosts arrive much later and steeper, where
    the extra margin is small. The dock-mesh numbers were measured without it.
    """

    margin_m: float = 0.5
    angle_bin_deg: float = 2.0
    cluster_m: float = 0.3
    cluster_n: int = 3
    ping_block: int = 1
    bin_extent: bool = True
    min_depression_deg: float = 5.0
    name: ClassVar[str] = "first_surface"

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        mask = np.ones(keep.shape, dtype=bool)
        deg = np.degrees(swath.depression)
        for side in range(2):
            ping, sample = np.nonzero(keep[:, :, side])
            mask[ping, sample, side] = self.mask(ping, swath.slant_range[sample], deg[ping, sample, side])
        return mask

    def mask(self, ping: np.ndarray, rng: np.ndarray, depression_deg: np.ndarray) -> np.ndarray:
        """For returns given as (ping, range, depression) triples, which to keep."""
        keep = np.ones(ping.size, dtype=bool)
        k = self.cluster_n - 1
        if ping.size <= k:
            return keep
        p = np.asarray(ping) // max(1, int(self.ping_block))
        a = np.floor(depression_deg / self.angle_bin_deg).astype(np.int64)
        order = np.lexsort((rng, a, p))
        p, a, r = p[order], a[order], rng[order]
        group = np.cumsum(np.r_[True, (p[1:] != p[:-1]) | (a[1:] != a[:-1])]) - 1
        start = np.nonzero((group[k:] == group[:-k]) & (r[k:] - r[:-k] < self.cluster_m))[0]
        first = np.full(group[-1] + 1, np.inf)
        np.minimum.at(first, group[start], r[start])
        limit = first[group] + self.margin_m
        if self.bin_extent:
            centre = np.maximum(np.abs((a + 0.5) * self.angle_bin_deg), self.min_depression_deg)
            limit = limit + first[group] * math.radians(self.angle_bin_deg) / np.tan(np.radians(centre))
        keep[order] = r <= limit
        return keep


@dataclass
class MostProbableAngle:
    """MPAA: keep returns near the most probable depression of their range bin.

    Per side, range bins of ``bin_m`` pooled over ``pings`` consecutive pings.
    A bin is trusted if at least ``min_fraction`` of its samples are still kept
    and their spread (standard deviation after dropping 1.5 IQR outliers) is
    at most ``max_spread_deg``; then the samples within ``band_deg`` of the
    mode of its ``angle_bin_deg`` histogram are kept, and the rest dropped.
    Untrusted bins are dropped whole. A bin assumes one angle, so where the
    depression changes faster than ``max_spread_deg`` across ``bin_m`` (a flat
    seabed close to nadir: about 18 deg/m at 60 deg and 5.5 m) the bin is
    dropped; smaller bins reach closer.

    From ``mpaa.py`` of the 2024 post-processing scripts, with these changes:
    it works on calibrated depressions, so no phase-model constants; the
    spread limit is absolute in degrees (theirs was relative to the mode, in
    an angle convention that differed between sides); range bins are the
    stated size, and the last partial bin is gated too; ``pings`` replaces
    their along-track median, which had a window of 1. Their nadir cone and
    DVL-altitude gates are left out: aliasing hides nadir from a depression
    gate, and altitude is not in the sonar data.
    """

    bin_m: float = 0.5
    pings: int = 1
    angle_bin_deg: float = 1.0
    band_deg: float = 3.0
    min_fraction: float = 0.25
    max_spread_deg: float = 2.0
    name: ClassVar[str] = "mpaa"

    def __call__(self, swath: Swath, keep: np.ndarray, info: dict[str, Any]) -> np.ndarray:
        n, ns = keep.shape[:2]
        bs = max(1, round(self.bin_m / swath.bin_size))
        pg = max(1, int(self.pings))
        nb, ng = -(-ns // bs), -(-n // pg)
        deg = np.full((ng * pg, nb * bs, 2), np.nan)
        deg[:n, :ns] = np.where(keep, np.degrees(swath.depression), np.nan)
        real = np.zeros(deg.shape, dtype=bool)
        real[:n, :ns] = True

        def grouped(x: np.ndarray) -> np.ndarray:  # -> (ping group, range bin, side, members)
            return x.reshape(ng, pg, nb, bs, 2).transpose(0, 2, 4, 1, 3).reshape(ng, nb, 2, pg * bs)

        g = grouped(deg)
        finite = np.isfinite(g)
        fraction = finite.sum(-1) / np.maximum(grouped(real).sum(-1), 1)

        lo, n_bins = -90.0, int(math.ceil(270.0 / self.angle_bin_deg))
        idx = np.clip(np.floor((g[finite] - lo) / self.angle_bin_deg).astype(np.int64), 0, n_bins - 1)
        owner = np.broadcast_to(np.arange(fraction.size).reshape(fraction.shape)[..., None], g.shape)[finite]
        hist = np.bincount(owner * n_bins + idx, minlength=fraction.size * n_bins)
        mode = lo + (hist.reshape(*fraction.shape, n_bins).argmax(-1) + 0.5) * self.angle_bin_deg

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN groups
            q1, q3 = _nan_quartiles(g, finite.sum(-1))
            iqr = q3 - q1
            inlier = (g >= (q1 - 1.5 * iqr)[..., None]) & (g <= (q3 + 1.5 * iqr)[..., None])
            spread = np.nanstd(np.where(inlier, g, np.nan), axis=-1)
        trusted = (fraction >= self.min_fraction) & (spread <= self.max_spread_deg)
        near = np.abs(g - mode[..., None]) <= self.band_deg
        kept = (trusted[..., None] & near).reshape(ng, nb, 2, pg, bs).transpose(0, 3, 1, 4, 2)
        info["mpaa"] = {"bin_samples": bs, "trusted_bins": float(trusted[fraction > 0].mean()) if (fraction > 0).any() else 0.0}
        return kept.reshape(ng * pg, nb * bs, 2)[:n, :ns]


def _nan_quartiles(x: np.ndarray, n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``np.nanpercentile(x, [25, 75], axis=-1)`` (linear), for ``n`` finite values per row; much faster."""
    s = np.sort(x, axis=-1)  # NaN last
    out = []
    for q in (0.25, 0.75):
        pos = np.maximum(n - 1, 0) * q
        lo = np.floor(pos).astype(np.int64)
        hi = np.minimum(lo + 1, np.maximum(n - 1, 0))
        a = np.take_along_axis(s, lo[..., None], -1)[..., 0]
        b = np.take_along_axis(s, hi[..., None], -1)[..., 0]
        out.append(np.where(n > 0, a + (b - a) * (pos - lo), np.nan))
    return out[0], out[1]


def default_stages() -> list:
    """The chain validated on the dock mesh: range, intensity, bias states, first surface."""
    return [RangeGate(), IntensityExcess(), BiasStates(), FirstSurface()]
