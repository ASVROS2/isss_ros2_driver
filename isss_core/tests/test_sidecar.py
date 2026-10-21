"""Every .bin log gets a sidecar that says how to read it."""

import json
import time
from pathlib import Path

import pytest

from isss import SonarConfig, SonarDriver
from isss.protocol import compute_decimation
from isss.sidecar import FORMAT, read_sidecar, sidecar_path
from isss.sim import SimulatedSonar


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _run(tmp_path, reconfigure=None, n=5):
    with SimulatedSonar(rate_hz=100) as sim:
        config = SonarConfig(ip="127.0.0.1", port=sim.port, recv_timeout_sec=1.0, reconnect_interval_sec=0.1)
        with SonarDriver(config, log_dir=tmp_path) as driver:
            path = Path(driver.status().log_path)
            assert _wait_for(lambda: read_sidecar(path)["first_packet"] is not None)
            time.sleep(0.1)
            if reconfigure:
                driver.configure(**reconfigure)
                assert _wait_for(lambda: sim.registers[0x11] == reconfigure.get("num_samples", 1000))
                time.sleep(0.2)
    return path, read_sidecar(path)


def test_sidecar_describes_the_log(tmp_path):
    path, meta = _run(tmp_path)
    assert meta["format"] == FORMAT and meta["bin"] == path.name
    assert meta["segments"] == [pytest.approx(meta["segments"][0])]
    seg = meta["segments"][0]
    assert (seg["byte_offset"], seg["num_samples"], seg["range_m"]) == (0, 1000, 20.0)
    assert seg["decimation"] == compute_decimation(20.0, 1000)
    assert [(w["register"], w["value"]) for w in meta["register_writes"]] == [(0x10, 111), (0x11, 1000), (0x12, 2)]
    assert meta["calibration"]["counts_per_unit_sine"] == 15218.0
    assert meta["packets"] * 6012 == meta["bytes"] == path.stat().st_size
    assert meta["first_packet"]["ping_time"] <= meta["last_packet"]["ping_time"]
    assert meta["closed_utc"] is not None
    assert sidecar_path(path).suffix == ".json"
    json.loads(sidecar_path(path).read_text())  # plain JSON


def test_reconfiguration_starts_a_segment(tmp_path):
    np = pytest.importorskip("numpy")
    from isss.diagnostics import load_log

    path, meta = _run(tmp_path, reconfigure={"num_samples": 500})
    assert [s["num_samples"] for s in meta["segments"]] == [1000, 500]
    assert meta["segments"][1]["byte_offset"] > 0
    assert [(w["register"], w["value"]) for w in meta["register_writes"][3:]] == [
        (0x10, compute_decimation(20.0, 500)), (0x11, 500), (0x12, 2)]
    with pytest.raises(ValueError, match="segment"):
        load_log(path)
    first, second = load_log(path, segment=0), load_log(path, segment=1)
    assert first.num_samples == 1000 and first.n_pings > 0
    assert second.num_samples == 500 and second.n_pings > 0
    assert np.all(np.diff(np.concatenate([first.ping_time, second.ping_time]).astype(np.int64)) > 0)


def test_logs_without_sidecar_still_load():
    pytest.importorskip("numpy")
    from isss.diagnostics import load_log

    arrays = load_log(Path(__file__).parent / "data" / "stick_2025-08-19_5pings.bin")
    assert arrays.n_pings == 5 and arrays.num_samples == 1000
