"""The bench test passes on the simulator and catches a device that misbehaves."""

from isss import SonarConfig
from isss.bench import FAIL, INFO, Bench, frame, run_bench
from isss.sim import REG_NUM_SAMPLES, SimulatedSonar, synthetic_packet


def _bench(sim, tmp_path):
    return run_bench(SonarConfig(ip="127.0.0.1", port=sim.port), tmp_path, scale=0.1,
                     ranges=(10.0, 20.0), samples=(500, 1000), log=lambda _msg: None)


def _steps(report):
    return {s.name: s for s in report.steps}


def test_bench_passes_on_simulator(tmp_path):
    with SimulatedSonar() as sim:
        report = _bench(sim, tmp_path)
    steps = _steps(report)
    assert report.passed, [(s.name, s.summary) for s in report.steps if s.status == FAIL]
    assert steps["stream on connect"].status == INFO
    assert "kept" in steps["configuration across connections"].summary
    assert steps["driver and logging"].data["sidecar"]["packets"] > 0
    assert (tmp_path / "README.md").exists() and (tmp_path / "bench.json").exists()


class IgnoresSampleCount(SimulatedSonar):
    """A faulty device: accepts the sample-count write but keeps sending 1000-sample pings."""

    def apply_register(self, reg, value):
        if reg != REG_NUM_SAMPLES:
            super().apply_register(reg, value)


def test_bench_catches_ignored_register(tmp_path):
    with IgnoresSampleCount() as sim:
        bench = Bench(SonarConfig(ip="127.0.0.1", port=sim.port), tmp_path, scale=0.1,
                      samples=(500, 1000), log=lambda _msg: None)
        result = bench.step_sample_sweep()
    assert result.status == FAIL
    assert "packet size 6012, expected 3012" in result.summary


def test_framing_infers_packet_size_without_being_told():
    stream = b"\x00" * 7 + b"".join(synthetic_packet(num_samples=750, ping_time=i * 300) for i in range(6))
    p = frame(stream)
    assert p.spacing == 12 + 6 * 750 and p.num_samples == 750
    assert len(p.packets) == 6
    assert p.resyncs == 0
