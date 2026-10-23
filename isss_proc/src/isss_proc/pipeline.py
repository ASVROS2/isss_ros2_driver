"""Run filter stages over pings: a whole log at once, or live pings as they arrive.

``Pipeline.run`` takes a ``Swath`` and returns a ``Result``: which samples
survive, which stage dropped each of the others, and the sensor-frame cloud.
``Stream`` keeps a rolling window of live pings and runs the same pipeline on
it, so stages that look at neighbouring pings (bias states, MPAA pooling)
see the recent past. ``viewer_keep`` does the same for ``isss.viz.PingViewer``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

import numpy as np

from isss.config import SonarConfig
from isss.diagnostics import PingArrays, sensor_frame_points
from isss_proc.filters import SIDES, default_stages
from isss_proc.swath import Swath

KEPT = 0
NO_SOLUTION = 255  # the phase model has no depression for these counts

Stage = Callable[[Swath, np.ndarray, dict], np.ndarray]


@dataclass
class Result:
    """A processed swath.

    ``keep`` (n, samples, 2) is True for samples every stage kept.
    ``rejected_by`` is 0 for those, ``k`` for samples stage ``k`` (from 1, in
    ``stages`` order) dropped first, and 255 where the phase model has no
    solution. ``info`` holds what the stages recorded (e.g. the bias states).
    """

    swath: Swath
    keep: np.ndarray
    rejected_by: np.ndarray
    stages: list[str]
    info: dict[str, Any] = field(default_factory=dict)

    def select(self, pings: slice) -> Result:
        return Result(self.swath.select(pings), self.keep[pings], self.rejected_by[pings], self.stages, self.info)

    def summary(self) -> dict[str, dict[str, int]]:
        """Per side: samples, those without a solution, those each stage dropped, those kept."""
        out = {}
        for side, name in enumerate(SIDES):
            codes = np.bincount(self.rejected_by[:, :, side].ravel(), minlength=256)
            out[name] = {"samples": int(codes.sum()), "no_solution": int(codes[NO_SOLUTION]),
                         **{stage: int(codes[k]) for k, stage in enumerate(self.stages, 1)},
                         "kept": int(codes[KEPT])}
        return out

    def points(self) -> dict[str, np.ndarray]:
        """Kept returns in the sensor frame: ``y`` across-track (port negative), ``z`` up, no motion.

        Also ``intensity``, ``side``, ``ping`` (within this swath), ``sample``,
        ``range`` [m] and ``depression`` [rad] per point.
        """
        pts = sensor_frame_points(self.swath.arrays, self.swath.config, keep=self.keep)
        pts["range"] = self.swath.slant_range[pts["sample"]]
        pts["depression"] = self.swath.depression[pts["ping"], pts["sample"], pts["side"]]
        return pts


class Pipeline:
    """Filter stages applied in order, each to the samples the ones before it kept."""

    def __init__(self, stages: Optional[Sequence[Stage]] = None) -> None:
        self.stages = list(default_stages() if stages is None else stages)

    @property
    def names(self) -> list[str]:
        return [getattr(s, "name", type(s).__name__) for s in self.stages]

    def run(self, swath: Swath) -> Result:
        keep = np.isfinite(swath.depression)
        rejected_by = np.where(keep, KEPT, NO_SOLUTION).astype(np.uint8)
        info: dict[str, Any] = {}
        for k, stage in enumerate(self.stages, 1):
            if not keep.any():
                break
            verdict = np.asarray(stage(swath, keep, info), dtype=bool)
            rejected_by[keep & ~verdict] = k
            keep = keep & verdict
        return Result(swath, keep, rejected_by, self.names, info)

    def __call__(self, arrays: PingArrays, config: SonarConfig) -> Result:
        return self.run(Swath(arrays, config))


class Stream:
    """Online processing: push pings as they arrive, each is processed with up to ``context`` before it.

    The window restarts when the sample count or decimation changes. Results
    agree with a batch run over the same window; over a whole log, stages that
    fit something to all pings (bias states) can differ slightly.
    """

    def __init__(self, config: SonarConfig, pipeline: Optional[Pipeline] = None, context: int = 200) -> None:
        self.config = config
        self.pipeline = pipeline or Pipeline()
        self.context = max(1, context)
        self.window: Optional[PingArrays] = None

    def push(self, arrays: PingArrays) -> Result:
        """Add one or more pings; returns the result for those pings."""
        w = self.window
        if w is None or w.num_samples != arrays.num_samples or (
                arrays.n_pings and w.n_pings and arrays.decimation[0] != w.decimation[-1]):
            w = arrays
        else:
            w = concat(w, arrays)
        self.window = w.select(slice(max(0, w.n_pings - self.context - arrays.n_pings), None))
        result = self.pipeline(self.window, self.config)
        return result.select(slice(self.window.n_pings - arrays.n_pings, None))

    def push_packet(self, packet: bytes) -> Result:
        """Add one raw packet (header + payload), as the driver's ``on_ping`` gets it in ``ping.raw``."""
        from isss.diagnostics import decode_packets
        from isss.protocol import BYTES_PER_SAMPLE, HEADER_SIZE

        return self.push(decode_packets([packet], (len(packet) - HEADER_SIZE) // BYTES_PER_SAMPLE))


def concat(a: PingArrays, b: PingArrays) -> PingArrays:
    return PingArrays(*(np.concatenate([getattr(a, f), getattr(b, f)])
                        for f in ("ping_time", "decimation", "size_field", "intensity", "counts")))


def viewer_keep(pipeline: Optional[Pipeline] = None, context: int = 200):
    """A ``keep`` for ``isss.viz.PingViewer``: each ping filtered as it would be live.

    The pings asked for are processed together with up to ``context`` pings
    before them, as ``Stream`` would have.
    """
    pipeline = pipeline or Pipeline()

    def keep(arrays: PingArrays, config: SonarConfig, pings: slice) -> np.ndarray:
        start, stop, _ = pings.indices(arrays.n_pings)
        lo = max(0, start - context)
        return pipeline(arrays.select(slice(lo, stop)), config).keep[start - lo:]

    return keep
