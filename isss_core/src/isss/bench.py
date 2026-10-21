"""Bench test of a connected ISSS: everything that does not need echoes.

Out of the water the sonar's intensity and phase mean little, but its protocol,
configuration, timing and our driver can all be checked. ``run_bench`` talks to
the device step by step and records what it sees; ``isss-bench`` runs it and
writes a report. Run it against ``isss-sim`` to see what a pass looks like.

Several steps answer questions the protocol doc leaves open (does it stream
before configuration, does it keep its configuration across connections, how
fast does its clock run). Those are reported as ``info``: there is no wrong answer.
"""

from __future__ import annotations

import json
import socket
import statistics
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

from isss.config import SonarConfig
from isss.driver import SonarDriver
from isss.protocol import (
    FREQ_ID_DEFAULT,
    HEADER_SIZE,
    PACKET_TYPE_ISSS,
    SOP1,
    SOP2,
    compute_decimation,
    expected_ping_interval,
    packet_size,
)
from isss.sidecar import read_sidecar
from isss.sonar import ISSSonar

PASS, FAIL, INFO, SKIP = "pass", "fail", "info", "skip"
PACING_TOLERANCE = 0.15  # relative, on the ping interval


@dataclass
class StepResult:
    name: str
    status: str
    summary: str
    data: dict = field(default_factory=dict)


@dataclass
class BenchReport:
    sonar: str
    started_utc: str
    steps: list[StepResult] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(s.status != FAIL for s in self.steps)

    def to_dict(self) -> dict:
        return {"sonar": self.sonar, "started_utc": self.started_utc, "passed": self.passed,
                "steps": [asdict(s) for s in self.steps]}

    def markdown(self) -> str:
        rows = "\n".join(f"| {s.status.upper()} | {s.name} | {s.summary} |" for s in self.steps)
        return (f"# ISSS bench test: {self.sonar}\n\nStarted {self.started_utc}. "
                f"Overall: **{'PASS' if self.passed else 'FAIL'}**.\n\n"
                f"| Status | Step | Result |\n|---|---|---|\n{rows}\n\n"
                "Measurements for each step are in `bench.json`.\n")


# Reading the raw stream


@dataclass
class Probe:
    """What arrived on the socket over a period, framed without assuming a sample count."""

    seconds: float
    n_bytes: int
    spacing: int | None  # most common distance between headers = packet size
    packets: list[dict]  # framed packets: offset, ping_time, decimation, size_field
    resyncs: int  # times framing at ``spacing`` broke and had to search again

    @property
    def num_samples(self) -> int | None:
        if self.spacing is None or (self.spacing - HEADER_SIZE) % 6:
            return None
        return (self.spacing - HEADER_SIZE) // 6

    @property
    def decimations(self) -> list[int]:
        return sorted({p["decimation"] for p in self.packets})

    @property
    def ping_interval(self) -> float | None:
        t = [p["ping_time"] for p in self.packets]
        dt = [(b - a) * 1e-4 for a, b in zip(t, t[1:]) if b > a]
        return statistics.median(dt) if dt else None

    def summary(self) -> dict:
        return {"bytes": self.n_bytes, "seconds": round(self.seconds, 3), "packets": len(self.packets),
                "spacing": self.spacing, "num_samples": self.num_samples, "decimations": self.decimations,
                "ping_interval_ms": None if self.ping_interval is None else round(self.ping_interval * 1e3, 3),
                "resyncs": self.resyncs,
                "size_fields": sorted({p["size_field"] for p in self.packets})}


def _is_header(buf: bytes | bytearray, i: int) -> bool:
    return (i + HEADER_SIZE <= len(buf) and buf[i] == SOP1 and buf[i + 1] == SOP2
            and buf[i + 4] == PACKET_TYPE_ISSS and buf[i + 9] == FREQ_ID_DEFAULT)


def _header_candidates(buf: bytes) -> list[int]:
    out, i = [], buf.find(b"\xfe\x05")
    while i >= 0:
        if _is_header(buf, i):
            out.append(i)
        i = buf.find(b"\xfe\x05", i + 1)
    return out


def frame(buf: bytes, seconds: float = 0.0) -> Probe:
    """Find the packet size from header spacing, then walk the stream at that size."""
    cands = _header_candidates(buf)
    diffs = [b - a for a, b in zip(cands, cands[1:])]
    spacing = statistics.mode(diffs) if diffs else None
    packets, resyncs = [], 0
    if spacing:
        i = cands[0]
        while i + spacing <= len(buf):
            if _is_header(buf, i):
                packets.append({
                    "offset": i,
                    "ping_time": int.from_bytes(buf[i + 5 : i + 9], "big"),
                    "decimation": int.from_bytes(buf[i + 10 : i + 12], "big"),
                    "size_field": int.from_bytes(buf[i + 2 : i + 4], "big"),
                })
                i += spacing
            else:
                resyncs += 1
                later = [c for c in cands if c > i]
                if not later:
                    break
                i = later[0]
    return Probe(seconds, len(buf), spacing, packets, resyncs)


def read_for(sonar: ISSSonar, seconds: float) -> bytes:
    """Raw bytes received over ``seconds`` (stops early if the connection closes)."""
    buf = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            chunk = sonar.recv_raw()
        except (TimeoutError, socket.timeout):
            continue
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def probe(sonar: ISSSonar, seconds: float, *, settle: float = 0.0) -> Probe:
    """Discard ``settle`` seconds (packets in flight from before a change), then frame ``seconds``."""
    if settle:
        read_for(sonar, settle)
    start = time.monotonic()
    buf = read_for(sonar, seconds)
    return frame(buf, time.monotonic() - start)


# The bench


class Bench:
    def __init__(
        self,
        config: SonarConfig,
        out_dir: Path,
        *,
        scale: float = 1.0,
        ranges: tuple[float, ...] = (10.0, 20.0, 40.0),
        samples: tuple[int, ...] = (500, 1000, 2000),
        log: Callable[[str], None] = print,
    ) -> None:
        self.base = config
        self.out_dir = Path(out_dir)
        self.scale = scale
        self.ranges = ranges
        self.samples = samples
        self.log = log

    def _config(self, **changes) -> SonarConfig:
        values = asdict(self.base)
        values.update(changes)
        return SonarConfig(**values)

    def _sonar(self, **changes) -> ISSSonar:
        sonar = ISSSonar(self._config(recv_timeout_sec=0.5, **changes))
        sonar.connect()
        return sonar

    def _t(self, seconds: float) -> float:
        return max(0.3, seconds * self.scale)

    def run(self) -> BenchReport:
        from isss.sidecar import utc_now

        report = BenchReport(f"{self.base.ip}:{self.base.port}", utc_now())
        steps = [
            ("connect", self.step_connect),
            ("stream on connect", self.step_stream_on_connect),
            ("baseline configuration", self.step_baseline),
            ("range sweep", self.step_range_sweep),
            ("sample count sweep", self.step_sample_sweep),
            ("phase setting", self.step_phase_setting),
            ("configuration across connections", self.step_persistence),
            ("device clock", self.step_clock),
            ("driver and logging", self.step_driver_logging),
            ("driver reconnect", self.step_reconnect),
        ]
        for name, step in steps:
            self.log(f"… {name}")
            try:
                result = step()
            except Exception as exc:  # report and keep going: later steps may still work
                result = StepResult(name, FAIL, f"error: {exc!r}")
            result.name = name
            report.steps.append(result)
            self.log(f"  {result.status.upper():4} {result.summary}")
            if name == "connect" and result.status == FAIL:
                self.log("  cannot connect; stopping")
                break
        self._restore()
        return report

    def _restore(self) -> None:
        try:
            sonar = self._sonar()
            sonar.setup_sonar()
            sonar.disconnect()
        except OSError:
            pass

    # Steps

    def step_connect(self) -> StepResult:
        start = time.monotonic()
        sonar = ISSSonar(self._config(recv_timeout_sec=0.5))
        sonar.connect()
        ms = (time.monotonic() - start) * 1e3
        sonar.disconnect()
        return StepResult("", PASS, f"TCP connect in {ms:.1f} ms", {"connect_ms": ms})

    def step_stream_on_connect(self) -> StepResult:
        sonar = self._sonar()
        try:
            p = probe(sonar, self._t(2.0))
        finally:
            sonar.disconnect()
        if not p.packets:
            return StepResult("", INFO, f"no packets before configuration ({p.n_bytes} bytes)", p.summary())
        return StepResult("", INFO, (f"streams on connect: {len(p.packets)} packets, {p.num_samples} samples, "
                                     f"decimation {p.decimations}"), p.summary())

    def _check_config(self, sonar: ISSSonar, range_m: float, num_samples: int) -> tuple[bool, list[str], dict]:
        sonar.config.range_m, sonar.config.num_samples = range_m, num_samples
        writes = sonar.setup_sonar()
        p = probe(sonar, self._t(3.0), settle=self._t(0.6))
        expected_dec = compute_decimation(range_m, num_samples)
        expected_dt = expected_ping_interval(range_m)
        problems = []
        if p.spacing != packet_size(num_samples):
            problems.append(f"packet size {p.spacing}, expected {packet_size(num_samples)}")
        if p.decimations != [expected_dec]:
            problems.append(f"header decimation {p.decimations}, expected {expected_dec}")
        if p.ping_interval is None:
            problems.append("no ping interval")
        elif abs(p.ping_interval - expected_dt) > PACING_TOLERANCE * expected_dt:
            problems.append(f"ping interval {p.ping_interval * 1e3:.1f} ms, expected ~{expected_dt * 1e3:.1f}")
        if p.resyncs:
            problems.append(f"{p.resyncs} framing errors")
        data = {"range_m": range_m, "num_samples": num_samples, "register_writes": writes,
                "expected_decimation": expected_dec, "expected_interval_ms": round(expected_dt * 1e3, 3),
                **p.summary()}
        return not problems, problems, data

    def step_baseline(self) -> StepResult:
        sonar = self._sonar()
        try:
            ok, problems, data = self._check_config(sonar, self.base.range_m, self.base.num_samples)
        finally:
            sonar.disconnect()
        if ok:
            return StepResult("", PASS, (f"{data['packets']} packets of {data['spacing']} bytes, decimation "
                                         f"{data['decimations'][0]}, {data['ping_interval_ms']:.1f} ms apart"), data)
        return StepResult("", FAIL, "; ".join(problems), data)

    def step_range_sweep(self) -> StepResult:
        sonar = self._sonar()
        rows = []
        try:
            for r in self.ranges:
                ok, problems, data = self._check_config(sonar, r, self.base.num_samples)
                rows.append({"ok": ok, "problems": problems, **data})
        finally:
            sonar.disconnect()
        fit = _line([(row["range_m"], row["ping_interval_ms"]) for row in rows if row["ping_interval_ms"]])
        extra = ""
        if fit:
            slope, intercept = fit
            extra = f"; interval = {slope:.3f} ms/m · range + {intercept:.2f} ms (≈ {2000 / slope:.0f} m/s)"
        bad = [f"{row['range_m']:g} m: {'; '.join(row['problems'])}" for row in rows if not row["ok"]]
        status = PASS if not bad else FAIL
        summary = ("all ranges as expected" if not bad else " | ".join(bad)) + extra
        return StepResult("", status, summary, {"ranges": rows, "fit_ms_per_m_and_offset_ms": fit})

    def step_sample_sweep(self) -> StepResult:
        sonar = self._sonar()
        rows = []
        try:
            for n in self.samples:
                ok, problems, data = self._check_config(sonar, self.base.range_m, n)
                rows.append({"ok": ok, "problems": problems, **data})
        finally:
            sonar.disconnect()
        bad = [f"{row['num_samples']}: {'; '.join(row['problems'])}" for row in rows if not row["ok"]]
        summary = ("packet size and decimation follow the sample count" if not bad else " | ".join(bad))
        return StepResult("", PASS if not bad else FAIL, summary, {"samples": rows})

    def step_phase_setting(self) -> StepResult:
        sonar = self._sonar()
        rows = {}
        try:
            for phases in (0, self.base.num_phases or 2):
                sonar.config.num_phases = phases
                sonar.setup_sonar()
                sonar.read_packet()  # resynchronise after the writes
                read_for(sonar, self._t(0.4))
                counts = []
                for _ in range(10):
                    raw = sonar.read_packet()
                    payload = raw[HEADER_SIZE:]
                    counts += [int.from_bytes(payload[i : i + 2], "big", signed=True) for i in range(0, len(payload), 6)]
                rows[phases] = {"zero_fraction": round(sum(c == 0 for c in counts) / len(counts), 4),
                                "distinct_values": len(set(counts)),
                                "min": min(counts), "max": max(counts)}
        finally:
            sonar.config.num_phases = self.base.num_phases
            sonar.disconnect()
        summary = "; ".join(f"phases={k}: {v['distinct_values']} distinct values, {v['zero_fraction']:.0%} zero"
                            for k, v in rows.items())
        return StepResult("", INFO, summary, {"by_num_phases": rows})

    def step_persistence(self) -> StepResult:
        other_range = 30.0 if self.base.range_m != 30.0 else 25.0
        sonar = self._sonar(range_m=other_range)
        try:
            sonar.setup_sonar()
            probe(sonar, self._t(0.5))
        finally:
            sonar.disconnect()
        sonar = self._sonar()
        try:
            p = probe(sonar, self._t(1.5))
        finally:
            sonar.disconnect()
        written = compute_decimation(other_range, self.base.num_samples)
        default = compute_decimation(self.base.range_m, self.base.num_samples)
        if not p.packets:
            verdict = "no packets without configuration; cannot tell"
        elif p.decimations == [written]:
            verdict = f"kept: still decimation {written} ({other_range:g} m) after reconnecting"
        else:
            verdict = f"not kept: decimation {p.decimations} after reconnecting (written {written}, baseline {default})"
        return StepResult("", INFO, verdict, {"written_decimation": written, **p.summary()})

    def step_clock(self) -> StepResult:
        sonar = self._sonar()
        host, device = [], []
        try:
            sonar.setup_sonar()
            read_for(sonar, self._t(0.6))
            deadline = time.monotonic() + self._t(20.0)
            while time.monotonic() < deadline:
                packet = sonar.read_packet()
                host.append(time.monotonic())
                device.append(int.from_bytes(packet[5:9], "big") * 1e-4)
        finally:
            sonar.disconnect()
        if len(device) < 10:
            return StepResult("", FAIL, f"only {len(device)} packets in {self._t(20.0):.0f} s")
        t0h, t0d = host[0], device[0]
        fit = _line([(h - t0h, d - t0d) for h, d in zip(host, device)])
        slope, intercept = fit
        residual_ms = sorted(abs((d - t0d) - (slope * (h - t0h) + intercept)) * 1e3 for h, d in zip(host, device))
        dt = [b - a for a, b in zip(device, device[1:])]
        median_dt = statistics.median(dt)
        gaps = sum(1 for x in dt if x > 1.5 * median_dt)
        backwards = sum(1 for x in dt if x <= 0)
        data = {
            "packets": len(device),
            "seconds": round(host[-1] - host[0], 3),
            "drift_ppm": round((slope - 1.0) * 1e6, 1),
            "arrival_jitter_ms": {"p50": round(residual_ms[len(residual_ms) // 2], 3),
                                   "p95": round(residual_ms[int(len(residual_ms) * 0.95)], 3),
                                   "max": round(residual_ms[-1], 3)},
            "gaps": gaps,
            "backwards": backwards,
            "device_time_at_start_s": round(t0d, 4),
            "device_uptime_h_if_counter_starts_at_power_on": round(t0d / 3600, 3),
        }
        status = PASS if gaps == 0 and backwards == 0 else FAIL
        summary = (f"device clock {data['drift_ppm']:+.0f} ppm against host over {data['seconds']:.0f} s; "
                   f"arrival jitter p95 {data['arrival_jitter_ms']['p95']:.1f} ms; {gaps} gaps; "
                   f"counter at {t0d:.1f} s ({data['device_uptime_h_if_counter_starts_at_power_on']:.2f} h)")
        return StepResult("", status, summary, data)

    def step_driver_logging(self) -> StepResult:
        log_dir = self.out_dir / "logs"
        received = []
        driver = SonarDriver(self._config(recv_timeout_sec=1.0), received.append, log_dir=log_dir)
        driver.start()
        try:
            time.sleep(self._t(5.0))
            status = driver.status()
        finally:
            driver.stop()
        if not status.log_path:
            return StepResult("", FAIL, "driver did not open a log")
        path = Path(status.log_path)
        meta = read_sidecar(path)
        problems = []
        if meta is None:
            problems.append("no sidecar")
        else:
            if meta.get("closed_utc") is None:
                problems.append("sidecar not closed")
            if meta["packets"] != len(received):
                problems.append(f"sidecar counts {meta['packets']} packets, driver delivered {len(received)}")
            if meta["bytes"] != path.stat().st_size:
                problems.append("sidecar byte count does not match the file")
        data = {"log": str(path), "pings": len(received), "ping_rate_hz": round(status.ping_rate_hz, 2),
                "sidecar": meta}
        try:
            from isss.diagnostics import health, load_log

            arrays = load_log(path)
            if arrays.n_pings != len(received):
                problems.append(f"log decodes to {arrays.n_pings} pings, driver delivered {len(received)}")
            data["health"] = health(arrays, self.base).to_dict()
        except ImportError:
            data["health"] = "numpy not installed"
        if problems:
            return StepResult("", FAIL, "; ".join(problems), data)
        return StepResult("", PASS, f"{len(received)} pings at {status.ping_rate_hz:.1f} Hz; log and sidecar agree: {path.name}", data)

    def step_reconnect(self) -> StepResult:
        received = []
        config = self._config(recv_timeout_sec=1.0, reconnect_interval_sec=0.5)
        driver = SonarDriver(config, received.append)
        driver.start()
        try:
            time.sleep(self._t(1.0))
            before = len(received)
            dropped_at = time.monotonic()
            driver._sonar.disconnect()  # as if the link dropped
            deadline = dropped_at + config.reconnect_interval_sec + 5.0
            while time.monotonic() < deadline and len(received) <= before + 5:
                time.sleep(0.05)
            recovered = time.monotonic() - dropped_at
            after = len(received) - before
        finally:
            driver.stop()
        if after > 5:
            return StepResult("", PASS, f"pings resumed {recovered:.1f} s after the connection dropped",
                              {"recovery_s": round(recovered, 2)})
        return StepResult("", FAIL, f"no pings within {recovered:.1f} s after the connection dropped")


def _line(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    """Least-squares slope and intercept, or None with fewer than two distinct x."""
    if len({x for x, _ in points}) < 2:
        return None
    n = len(points)
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    sxx = sum((x - mx) ** 2 for x, _ in points)
    sxy = sum((x - mx) * (y - my) for x, y in points)
    slope = sxy / sxx
    return slope, my - slope * mx


def run_bench(config: SonarConfig, out_dir: str | Path, **kwargs) -> BenchReport:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    report = Bench(config, out, **kwargs).run()
    (out / "bench.json").write_text(json.dumps(report.to_dict(), indent=2, default=str) + "\n")
    (out / "README.md").write_text(report.markdown())
    return report


def main(argv: list[str] | None = None) -> None:
    """``isss-bench``: run the bench test against a sonar or ``isss-sim``."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Bench-test an ISSS (no echoes needed). Takes about two minutes.")
    parser.add_argument("--ip", default="192.168.2.62")
    parser.add_argument("--port", type=int, default=57349)
    parser.add_argument("-o", "--out", type=Path, default=None, help="Output directory (default: bench_<time>/)")
    parser.add_argument("--quick", action="store_true", help="Shorter measurements (about 30 s)")
    parser.add_argument("--ranges", default="10,20,40", help="Ranges for the sweep [m]")
    parser.add_argument("--samples", default="500,1000,2000", help="Sample counts for the sweep")
    args = parser.parse_args(argv)

    out = args.out or Path(f"bench_{time.strftime('%Y%m%d_%H%M%S')}")
    report = run_bench(
        SonarConfig(ip=args.ip, port=args.port), out,
        scale=0.3 if args.quick else 1.0,
        ranges=tuple(float(x) for x in args.ranges.split(",")),
        samples=tuple(int(x) for x in args.samples.split(",")),
    )
    print(f"\n{'PASS' if report.passed else 'FAIL'}. Report: {out}/README.md")
    sys.exit(0 if report.passed else 1)
