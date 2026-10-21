"""End-to-end driver behaviour against the simulated sonar (no hardware needed)."""

import threading
import time

import pytest

from isss import NotConnectedError, SonarConfig, SonarDriver
from isss.log import find_packet_offsets
from isss.protocol import compute_decimation, parse_header
from isss.sidecar import read_sidecar
from isss.sim import (
    REG_DECIMATION,
    REG_NUM_PHASES,
    REG_NUM_SAMPLES,
    SimulatedSonar,
    packets_from_log,
    synthetic_packet,
)
from pathlib import Path

LOG = Path(__file__).parent / "data" / "stick_2025-08-19_5pings.bin"


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _config(port: int, **kwargs) -> SonarConfig:
    return SonarConfig(ip="127.0.0.1", port=port, recv_timeout_sec=1.0, reconnect_interval_sec=0.1, **kwargs)


class Collector:
    def __init__(self):
        self.pings = []
        self.lock = threading.Lock()

    def __call__(self, ping):
        with self.lock:
            self.pings.append(ping)

    def __len__(self):
        with self.lock:
            return len(self.pings)


def test_connect_writes_registers_and_streams():
    with SimulatedSonar(rate_hz=50) as sim:
        pings = Collector()
        with SonarDriver(_config(sim.port, range_m=30.0), pings) as driver:
            assert _wait_for(lambda: len(pings) >= 3)
            assert driver.status().connected
        assert sim.register_writes[:3] == [
            (REG_DECIMATION, compute_decimation(30.0, 1000)),
            (REG_NUM_SAMPLES, 1000),
            (REG_NUM_PHASES, 2),
        ]
        first = pings.pings[0]
        assert first.header.is_valid
        assert len(first.samples.port_rss) == 1000


def test_replays_recorded_log():
    with SimulatedSonar(packets_from_log(LOG), rate_hz=50) as sim:
        pings = Collector()
        with SonarDriver(_config(sim.port), pings):
            assert _wait_for(lambda: len(pings) >= 5)
        times = [p.header.ping_time for p in pings.pings[:5]]
        assert times == sorted(times)


def test_reconfigure_changes_packet_size():
    with SimulatedSonar(rate_hz=50) as sim:
        pings = Collector()
        with SonarDriver(_config(sim.port), pings) as driver:
            assert _wait_for(lambda: len(pings) >= 1)
            driver.configure(num_samples=500)
            assert _wait_for(lambda: sim.registers[REG_NUM_SAMPLES] == 500)
            assert _wait_for(lambda: len(pings.pings[-1].samples.port_rss) == 500)
            assert driver.status().decimation == compute_decimation(20.0, 500)


def test_reconnects_after_connection_drop():
    with SimulatedSonar(rate_hz=50) as sim:
        pings = Collector()
        with SonarDriver(_config(sim.port), pings) as driver:
            assert _wait_for(lambda: len(pings) >= 2)
            sim.drop_client()
            assert _wait_for(lambda: not driver.status().connected)
            assert driver.status().last_error
            before = len(pings)
            assert _wait_for(lambda: len(pings) > before + 2)
            assert driver.status().connected
            assert driver.status().last_error == ""


def test_starts_without_sonar_and_rejects_configure():
    with SimulatedSonar() as sim:
        port = sim.port
    driver = SonarDriver(_config(port))
    driver.start()
    try:
        assert not driver.status().connected
        with pytest.raises(NotConnectedError):
            driver.configure(range_m=40.0)
        assert driver.config.range_m == 40.0  # kept for the next reconnect
        with pytest.raises(ValueError):
            driver.configure(range_m=-1.0)
    finally:
        driver.stop()


def test_logs_raw_packets(tmp_path):
    with SimulatedSonar(rate_hz=50) as sim:
        with SonarDriver(_config(sim.port), log_dir=tmp_path) as driver:
            assert _wait_for(lambda: driver.status().logging)
            path = Path(driver.status().log_path)
            assert _wait_for(lambda: path.stat().st_size >= 3 * 6012)
    header, expected = path.read_bytes()[:12], synthetic_packet()[:12]
    assert header[:5] + header[9:] == expected[:5] + expected[9:]  # bytes 5-8 are the ping time


def test_default_pacing_follows_range():
    """Logs show one ping per two-way travel time; the simulator reproduces it."""
    with SimulatedSonar() as sim:
        pings = Collector()
        with SonarDriver(_config(sim.port, range_m=10.0), pings):
            assert _wait_for(lambda: len(pings) >= 20)
            start = len(pings)
            time.sleep(1.0)
            rate = len(pings) - start
    assert 45 <= rate <= 85  # 2 * 10 m / 1500 m/s = 13.3 ms -> ~75 Hz


def test_synthetic_ping_decodes_to_flat_bottom():
    """The simulator's seabed comes back at depression asin(bottom / bin) wherever it is unambiguous."""
    import math

    from isss.protocol import parse_packet

    config = SonarConfig()
    bottom = 200
    ping = parse_packet(synthetic_packet(bottom_sample=bottom), config)
    beta0 = math.radians(config.boresight_depression_deg)
    limit = math.asin(10240 / config.counts_per_unit_sine)
    checked = 0
    for i in range(bottom, 1000):
        expected = math.asin(bottom / i)
        if abs(beta0 - expected) < limit - 0.01:
            assert ping.samples.starboard_depression[i] == pytest.approx(expected, abs=2e-4)
            checked += 1
    assert checked > 700


class SlowToApply(SimulatedSonar):
    """Keeps streaming at the old setting for a while after each register write."""

    def apply_register(self, reg, value):  # called under the simulator's lock, which pauses streaming
        threading.Timer(0.1, self._apply_later, (reg, value)).start()

    def _apply_later(self, reg, value):
        with self._lock:
            super().apply_register(reg, value)


def _logged_decimations(path, num_samples=1000):
    data = path.read_bytes()
    offsets = find_packet_offsets(data, num_samples=num_samples)
    return offsets, [parse_header(data[o:o + 12]).decimation for o in offsets]


def test_drops_pings_at_the_power_on_configuration(tmp_path):
    pings = Collector()
    with SlowToApply(rate_hz=100) as sim:
        with SonarDriver(_config(sim.port), pings, log_dir=tmp_path) as driver:
            assert _wait_for(lambda: len(pings) >= 10)
            path = Path(driver.status().log_path)
            status = driver.status()
    assert status.dropped_pings >= 1 and status.last_error == ""
    assert {p.header.decimation for p in pings.pings} == {compute_decimation(20.0, 1000)}
    offsets, decimations = _logged_decimations(path)
    assert set(decimations) == {compute_decimation(20.0, 1000)}
    assert read_sidecar(path)["packets"] == len(offsets)


def test_stale_pings_at_another_sample_count_do_not_desync(tmp_path):
    pings = Collector()
    with SlowToApply(rate_hz=100) as sim:  # powers on at 1000 samples; the driver wants 500
        with SonarDriver(_config(sim.port, num_samples=500), pings, log_dir=tmp_path) as driver:
            assert _wait_for(lambda: len(pings) >= 10)
            path = Path(driver.status().log_path)
            status = driver.status()
    assert status.dropped_pings >= 1 and status.last_error == ""
    assert {len(p.raw) for p in pings.pings} == {12 + 6 * 500}
    offsets, decimations = _logged_decimations(path, num_samples=500)
    assert set(decimations) == {compute_decimation(20.0, 500)}
    assert len(offsets) * (12 + 6 * 500) == path.stat().st_size


def test_reconfigure_starts_the_new_segment_at_the_first_new_ping(tmp_path):
    with SlowToApply(rate_hz=100) as sim:
        with SonarDriver(_config(sim.port), log_dir=tmp_path) as driver:
            path = Path(driver.status().log_path)
            assert _wait_for(lambda: path.stat().st_size >= 5 * 6012)
            driver.configure(range_m=40.0)
            assert _wait_for(lambda: compute_decimation(40.0, 1000) in _logged_decimations(path)[1])
            assert _wait_for(lambda: driver.status().dropped_pings >= 1)
    offsets, decimations = _logged_decimations(path)
    first_new = decimations.index(compute_decimation(40.0, 1000))
    assert set(decimations[first_new:]) == {compute_decimation(40.0, 1000)}
    segments = read_sidecar(path)["segments"]
    assert [seg["decimation"] for seg in segments] == [111, 222]
    assert segments[1]["byte_offset"] == offsets[first_new]
