"""End-to-end driver behaviour against the simulated sonar (no hardware needed)."""

import threading
import time

import pytest

from isss import NotConnectedError, SonarConfig, SonarDriver
from isss.protocol import compute_decimation
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
    assert path.read_bytes()[:12] == synthetic_packet(ping_time=0)[:12]


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
