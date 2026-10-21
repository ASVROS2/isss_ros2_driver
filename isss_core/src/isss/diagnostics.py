"""Array views of ISSS pings and health checks, for the diagnostic tools.

Needs numpy (``uv sync`` installs it; otherwise the ``viz`` extra). Decoding here
is vectorised for whole logs, and ``tests/test_diagnostics.py`` checks that it
agrees with the reference decoder in ``isss.protocol`` sample for sample.
"""

from __future__ import annotations

import collections
import math
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np

from isss.config import SonarConfig
from isss.log import find_packet_offsets
from isss.protocol import (
    BYTES_PER_SAMPLE,
    DEFAULT_SAMPLES,
    HEADER_SIZE,
    PHASE_COUNTS_PER_CYCLE,
    expected_ping_interval,
    packet_size,
    range_from_decimation,
    sample_interval,
)

PORT, STARBOARD = 0, 1
SIDES = ("port", "starboard")
HALF_CYCLE = PHASE_COUNTS_PER_CYCLE // 2


@dataclass
class PingArrays:
    """A stack of pings: per-ping header values and per-sample data, side 0 = port, 1 = starboard."""

    ping_time: np.ndarray  # (n,) device counter [100 us]
    decimation: np.ndarray  # (n,)
    size_field: np.ndarray  # (n,) header bytes 2-3
    intensity: np.ndarray  # (n, samples, 2) uint8
    counts: np.ndarray  # (n, samples, 2) int16, signed phase counts

    @property
    def n_pings(self) -> int:
        return int(self.intensity.shape[0])

    @property
    def num_samples(self) -> int:
        return int(self.intensity.shape[1])

    def select(self, pings: slice) -> PingArrays:
        """A view of some of the pings."""
        return PingArrays(self.ping_time[pings], self.decimation[pings], self.size_field[pings],
                          self.intensity[pings], self.counts[pings])

    def depression(self, config: SonarConfig) -> np.ndarray:
        """Depression angle [rad] per sample and side; NaN where ``|counts| > K``."""
        s = self.counts.astype(np.float64) / config.counts_per_unit_sine
        with np.errstate(invalid="ignore"):
            alpha = np.arcsin(np.where(np.abs(s) <= 1.0, s, np.nan))
        return math.radians(config.boresight_depression_deg) - alpha

    def slant_range(self, config: SonarConfig) -> np.ndarray:
        """Slant range [m] of each bin, from the most common decimation in the stack."""
        if self.n_pings == 0:
            return np.zeros(self.num_samples)
        dec = int(np.bincount(self.decimation.astype(np.int64)).argmax())
        return np.arange(self.num_samples) * sample_interval(dec) * config.sound_velocity / 2.0


def decode_packets(packets: Sequence[bytes], num_samples: int = DEFAULT_SAMPLES) -> PingArrays:
    """Decode raw packets (header + payload) of one sample count into arrays."""
    size = packet_size(num_samples)
    whole = [p for p in packets if len(p) >= size]
    raw = np.frombuffer(b"".join(p[:size] for p in whole), dtype=np.uint8).reshape(len(whole), size)
    head = raw[:, :HEADER_SIZE].astype(np.uint32)
    payload = raw[:, HEADER_SIZE:].reshape(len(whole), num_samples, BYTES_PER_SAMPLE)

    def phase(hi: int, lo: int) -> np.ndarray:
        # Same sign rule as isss.protocol._decode_phase: values above 32768 wrap negative
        value = (payload[:, :, hi].astype(np.int32) << 8) | payload[:, :, lo]
        return np.where(value > 32768, value - 65536, value)

    return PingArrays(
        ping_time=(head[:, 5] << 24) | (head[:, 6] << 16) | (head[:, 7] << 8) | head[:, 8],
        decimation=(head[:, 10] << 8) | head[:, 11],
        size_field=(head[:, 2] << 8) | head[:, 3],
        intensity=np.stack([payload[:, :, 2], payload[:, :, 5]], axis=-1),
        counts=np.stack([phase(0, 1), phase(3, 4)], axis=-1).astype(np.int16),
    )


class PacketBuffer:
    """The newest ``maxlen`` raw packets from a live driver, thread-safe.

    ``add`` is the driver's ``on_ping`` callback target; ``arrays`` decodes the
    buffer. After a reconfiguration only packets of the newest size are kept.
    """

    def __init__(self, maxlen: int = 600) -> None:
        self._packets: collections.deque[bytes] = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, packet: bytes) -> None:
        with self._lock:
            if self._packets and len(packet) != len(self._packets[-1]):
                self._packets.clear()
            self._packets.append(packet)

    def __len__(self) -> int:
        with self._lock:
            return len(self._packets)

    def arrays(self) -> PingArrays:
        with self._lock:
            packets = list(self._packets)
        samples = (len(packets[-1]) - HEADER_SIZE) // BYTES_PER_SAMPLE if packets else DEFAULT_SAMPLES
        return decode_packets(packets, samples)


def load_log(path: str | Path, *, num_samples: int = DEFAULT_SAMPLES) -> PingArrays:
    """Decode every complete packet in a ``.bin`` log."""
    data = Path(path).read_bytes()
    size = packet_size(num_samples)
    offsets = [o for o in find_packet_offsets(data, num_samples=num_samples) if o + size <= len(data)]
    return decode_packets([data[o : o + size] for o in offsets], num_samples)


# Returns


def intensity_baseline(arrays: PingArrays, percentile: float = 20.0) -> np.ndarray:
    """Per ping and side, the background intensity level, shape (n, 2)."""
    return np.percentile(arrays.intensity, percentile, axis=1)


def strong_mask(arrays: PingArrays, min_excess: float = 25.0) -> np.ndarray:
    """Samples whose intensity is at least ``min_excess`` above their ping's background."""
    return arrays.intensity > intensity_baseline(arrays)[:, None, :] + min_excess


def detect_bottom(
    arrays: PingArrays,
    *,
    min_excess: float = 20.0,
    skip_fraction: float = 0.05,
    smooth: int = 15,
) -> np.ndarray:
    """First sustained return per ping and side, -1 where none; shape (n, 2).

    A marker for the viewers, not a bottom tracker: near-field echoes (the
    mount, the surface) count too. The first ``skip_fraction`` of bins is
    skipped, since the transmit ring-down sits there.
    """
    n, ns = arrays.n_pings, arrays.num_samples
    if n == 0:
        return np.zeros((0, 2), dtype=int)
    kernel = np.ones(smooth) / smooth
    x = arrays.intensity.astype(np.float64)
    smoothed = np.apply_along_axis(lambda row: np.convolve(row, kernel, mode="same"), 1, x)
    start = int(ns * skip_fraction)
    above = smoothed[:, start:, :] > intensity_baseline(arrays)[:, None, :] + min_excess
    found = above.any(axis=1)
    return np.where(found, above.argmax(axis=1) + start, -1)


def sensor_frame_points(
    arrays: PingArrays,
    config: SonarConfig,
    *,
    min_excess: float = 25.0,
    ping_slice: slice = slice(None),
) -> dict[str, np.ndarray]:
    """Strong returns in the sensor frame: across-track ``y`` (port negative) and ``z`` (up).

    No motion is applied, so pings are stacked on top of each other. Over a flat
    seabed the points should form a horizontal line at minus the altitude.
    """
    sub = arrays.select(ping_slice)
    r = sub.slant_range(config)[None, :, None]
    beta = sub.depression(config)
    keep = strong_mask(sub, min_excess) & np.isfinite(beta)
    side_sign = np.array([-1.0, 1.0])[None, None, :]
    y = side_sign * r * np.cos(beta)
    z = -r * np.sin(beta)
    side = np.broadcast_to(np.arange(2)[None, None, :], beta.shape)
    return {
        "y": y[keep],
        "z": z[keep],
        "intensity": sub.intensity[keep],
        "side": side[keep],
    }


# Health checks


@dataclass
class Check:
    name: str
    status: str  # "ok" | "warn" | "info"
    value: str
    detail: str = ""


@dataclass
class HealthReport:
    n_pings: int
    num_samples: int
    checks: list[Check] = field(default_factory=list)

    @property
    def worst(self) -> str:
        return "warn" if any(c.status == "warn" for c in self.checks) else "ok"

    def to_dict(self) -> dict:
        return {"n_pings": self.n_pings, "num_samples": self.num_samples, "worst": self.worst,
                "checks": [asdict(c) for c in self.checks]}


def health(arrays: PingArrays, config: SonarConfig) -> HealthReport:
    """Checks that flag a sonar or log that is not behaving like the ones we know."""
    report = HealthReport(arrays.n_pings, arrays.num_samples)
    add = report.checks.append
    if arrays.n_pings < 2:
        add(Check("pings", "warn", str(arrays.n_pings), "Too few pings to check anything."))
        return report

    decimations = np.unique(arrays.decimation)
    dec = int(np.bincount(arrays.decimation.astype(np.int64)).argmax())
    range_m = range_from_decimation(dec, arrays.num_samples)
    add(Check(
        "configuration",
        "ok" if decimations.size == 1 else "warn",
        f"decimation {dec}, {arrays.num_samples} samples = {range_m:.1f} m",
        "" if decimations.size == 1 else f"decimation changes within the log: {decimations.tolist()}",
    ))

    dt = np.diff(arrays.ping_time.astype(np.int64)) * 1e-4
    expected = expected_ping_interval(range_m)
    median = float(np.median(dt))
    add(Check(
        "ping interval",
        "ok" if abs(median - expected) <= 0.15 * expected else "warn",
        f"{median * 1e3:.1f} ms",
        f"expected about {expected * 1e3:.1f} ms (two-way travel time for {range_m:.1f} m)",
    ))
    gaps = dt > 1.5 * median
    backwards = dt < 0
    add(Check(
        "gaps",
        "ok" if not gaps.any() and not backwards.any() else "warn",
        f"{int(gaps.sum())} gaps, {int(backwards.sum())} backwards",
        f"longest interval {dt.max() * 1e3:.0f} ms; about {int(np.round(dt[gaps] / median - 1).sum())} pings missing"
        if gaps.any() else "",
    ))

    out_of_cycle = float(np.mean(np.abs(arrays.counts.astype(np.int32)) > HALF_CYCLE))
    add(Check(
        "phase encoding",
        "ok" if out_of_cycle < 0.01 else "warn",
        f"{out_of_cycle:.2%} of counts outside ±{HALF_CYCLE}",
        "" if out_of_cycle < 0.01 else
        "Phase does not look like the signed ±10240 encoding the depression model assumes "
        "(the July 2024 logs look like this).",
    ))

    strong = strong_mask(arrays)
    beta = arrays.depression(config)
    sat = arrays.intensity == 255
    for side, name in enumerate(SIDES):
        echo = float(strong[:, :, side].mean())
        add(Check(
            f"{name} echoes",
            "ok" if echo >= 0.05 else "warn",
            f"{echo:.1%} of samples well above background",
            "" if echo >= 0.05 else "Almost no echoes; the sonar may be out of the water.",
        ))
        n_strong = int(strong[:, :, side].sum())
        nan_frac = float(np.mean(np.isnan(beta[:, :, side][strong[:, :, side]]))) if n_strong else 0.0
        add(Check(
            f"{name} depression",
            "ok" if nan_frac < 0.05 else "warn",
            f"{nan_frac:.1%} of strong returns have no solution",
            f"{n_strong} strong returns; |counts| > K = {config.counts_per_unit_sine:.0f} has no solution",
        ))
        add(Check(
            f"{name} saturation",
            "ok" if sat[:, :, side].mean() < 0.01 else "warn",
            f"{sat[:, :, side].mean():.2%} of samples at 255",
        ))

    size_fields = np.unique(arrays.size_field)
    add(Check("header size field", "info", ", ".join(str(v) for v in size_fields[:5]),
              "Always 6007 in the logs seen so far; not a packet size."))
    return report


def phase_histograms(
    arrays: PingArrays, *, bins: int = 256, min_excess: float = 25.0
) -> dict[str, np.ndarray]:
    """Counts histograms per side for weak and strong samples over the full int16 range."""
    edges = np.linspace(-32768, 32768, bins + 1)
    strong = strong_mask(arrays, min_excess)
    out: dict[str, np.ndarray] = {"edges": edges}
    for side, name in enumerate(SIDES):
        c = arrays.counts[:, :, side]
        out[f"{name}_weak"] = np.histogram(c[~strong[:, :, side]], edges)[0]
        out[f"{name}_strong"] = np.histogram(c[strong[:, :, side]], edges)[0]
    return out

