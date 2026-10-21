"""NetCDF export of .bin logs, old-driver bags, and the driver's export on close."""

import json
import sqlite3
import struct
import time
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("h5netcdf")
xr = pytest.importorskip("xarray")

from isss import SonarConfig, SonarDriver  # noqa: E402
from isss.bag import LEGACY_TYPE, legacy_message_to_packet  # noqa: E402
from isss.cli import main_export  # noqa: E402
from isss.diagnostics import decode_packets, load_log  # noqa: E402
from isss.netcdf import export_bag, export_bin, read_netcdf  # noqa: E402
from isss.sidecar import read_sidecar  # noqa: E402
from isss.protocol import compute_decimation  # noqa: E402
from isss.sim import REG_DECIMATION, SimulatedSonar, synthetic_packet  # noqa: E402

LOG = Path(__file__).parent / "data" / "stick_2025-08-19_5pings.bin"


def _packets(n=20, decimation=111, num_samples=1000):
    return [synthetic_packet(ping_time=1000 + i * 268, decimation=decimation, num_samples=num_samples,
                             bottom_sample=num_samples // 4) for i in range(n)]


def test_bin_round_trip(tmp_path):
    [path] = export_bin(LOG, tmp_path / "log.nc")
    ds = xr.open_dataset(path)
    arrays = load_log(LOG)
    assert dict(ds.sizes) == {"ping": 5, "sample": 1000, "side": 2}
    np.testing.assert_array_equal(ds.intensity.values, arrays.intensity)
    np.testing.assert_array_equal(ds.phase_counts.values, arrays.counts)
    np.testing.assert_array_equal(ds.ping_time.values, arrays.ping_time)
    np.testing.assert_allclose(ds.slant_range.values, arrays.slant_range(SonarConfig()), rtol=1e-6)
    np.testing.assert_allclose(ds.depression.values, arrays.depression(SonarConfig()), rtol=1e-6)
    assert ds.attrs["isss_format"] == "isss-netcdf" and ds.attrs["counts_per_unit_sine"] == 15218.0
    assert ds.phase_counts.attrs["counts_per_cycle"] == 20480
    # Without sidecar or a time in the file name, time is unknown rather than made up
    assert np.isnat(ds.time.values).all()
    back = read_netcdf(path)
    np.testing.assert_array_equal(back.counts, arrays.counts)


def test_time_from_the_file_name_without_a_sidecar(tmp_path):
    log = tmp_path / "1765708429.500000.bin"
    log.write_bytes(b"".join(_packets(3)))
    ds = xr.open_dataset(export_bin(log)[0])
    seconds = ds.time.values.astype("datetime64[ns]").astype(np.int64) * 1e-9
    np.testing.assert_allclose(seconds, 1765708429.5 + np.arange(3) * 0.0268, atol=1e-6)
    assert "file name" in ds.attrs["time_source"]


def test_calibration_is_recorded(tmp_path):
    config = SonarConfig(port_phase_sign=-1, boresight_depression_deg=40.0)
    ds = xr.open_dataset(export_bin(LOG, tmp_path / "log.nc", config)[0])
    assert ds.attrs["port_phase_sign"] == -1 and ds.attrs["boresight_depression_deg"] == 40.0
    np.testing.assert_allclose(ds.depression.values, load_log(LOG).depression(config), rtol=1e-6)


def test_decimation_change_splits_the_file(tmp_path):
    log = tmp_path / "1765708429.000000.bin"
    log.write_bytes(b"".join(_packets(4, decimation=111) + _packets(3, decimation=222)))
    paths = export_bin(log)
    assert [p.name for p in paths] == [f"{log.stem}.nc", f"{log.stem}_2.nc"]
    a, b = (xr.open_dataset(p) for p in paths)
    assert a.sizes["ping"] == 4 and b.sizes["ping"] == 3
    assert int(b.decimation[0]) == 222 and b.slant_range.values[-1] > a.slant_range.values[-1]
    assert not list(tmp_path.glob("*.tmp"))


# Bags


def _legacy_cdr(packet: bytes, stamp_ns: int) -> bytes:
    """Serialize a packet as the old C++ driver published it (smarc_msgs/ISSS, little-endian CDR)."""
    n = (len(packet) - 12) // 6
    payload = np.frombuffer(packet[12:], dtype=np.uint8).reshape(n, 6)
    words = [(payload[:, hi].astype(np.uint16) << 8) | payload[:, hi + 1] for hi in (0, 3)]
    out = bytearray(b"\x00\x01\x00\x00")

    def put(fmt, *values):
        size = struct.calcsize("<" + fmt[0])
        out.extend(b"\x00" * (-(len(out) - 4) % size))
        out.extend(struct.pack("<" + fmt, *values))

    def seq(arr):
        put("I", arr.size)
        out.extend(b"\x00" * (-(len(out) - 4) % arr.itemsize))
        out.extend(arr.tobytes())

    put("i", stamp_ns // 1_000_000_000)
    put("I", stamp_ns % 1_000_000_000)
    put("I", 10)
    out.extend(b"isss_link\x00")
    for fmt, value in (("B", 0), ("I", 0), ("B", 0), ("f", 0.0), ("h", 0), ("H", 0), ("f", 0.02), ("f", 0.0), ("f", 0.0)):
        put(fmt, value)
    seq(payload[:, 2].copy())
    seq(payload[:, 5].copy())
    for w in words:
        seq((w.astype(np.float32) / np.float32(512.0)).astype("<f4"))
    for _ in range(2):
        seq(np.zeros(n, "<f4"))
    seq(np.frombuffer(packet[:12], dtype=np.uint8).copy())
    return bytes(out)


def _make_bag(directory: Path, messages, topic="/isss_stick/isss/raw"):
    directory.mkdir()
    con = sqlite3.connect(directory / f"{directory.name}_0.db3")
    con.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, "
                "offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES (1, ?, ?, 'cdr', '')", (topic, LEGACY_TYPE))
    con.execute("INSERT INTO topics VALUES (2, '/imu/data', 'sensor_msgs/msg/Imu', 'cdr', '')")
    for stamp, packet in messages:  # record time an hour off the stamps, as in the Beckholmen bags
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)",
                    (stamp + 3_600_000_000_000, _legacy_cdr(packet, stamp)))
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (2, 0, x'00')")
    con.commit()
    con.close()
    return directory


def test_legacy_message_rebuilds_the_packet():
    packet = _packets(1)[0]
    # Negative phase counts exercise the unsigned / 512 encoding
    assert (decode_packets([packet]).counts < 0).any()
    assert legacy_message_to_packet(_legacy_cdr(packet, 123_456_789_012)) == (123_456_789_012, packet)


def test_bag_export_sorts_by_header_stamp(tmp_path):
    packets = _packets(6)
    stamps = [1_765_708_429_000_000_000 + i * 26_800_000 for i in range(6)]
    order = [3, 4, 5, 0, 1, 2]  # recorded out of order, as after the Beckholmen clock step
    bag = _make_bag(tmp_path / "rosbag2_test", [(stamps[i], packets[i]) for i in order])
    [path] = export_bag(bag)
    assert path == tmp_path / "rosbag2_test.nc"
    ds = xr.open_dataset(path)
    expected = decode_packets(packets)
    np.testing.assert_array_equal(ds.phase_counts.values, expected.counts)
    np.testing.assert_array_equal(ds.intensity.values, expected.intensity)
    np.testing.assert_allclose(ds.time.values.astype("datetime64[ns]").astype(np.int64), stamps)


def test_export_command(tmp_path, capsys):
    bag = _make_bag(tmp_path / "rosbag2_cli", [(10**18 + i, p) for i, p in enumerate(_packets(3))])
    out = tmp_path / "out"
    out.mkdir()
    with pytest.raises(SystemExit) as exit_:
        main_export([str(bag), str(LOG), "-o", str(out), "--flip-port"])
    assert exit_.value.code == 0
    assert sorted(p.name for p in out.iterdir()) == ["rosbag2_cli.nc", "stick_2025-08-19_5pings.nc"]
    assert xr.open_dataset(out / "rosbag2_cli.nc").attrs["port_phase_sign"] == -1


# Driver


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_driver_exports_each_log_when_it_closes(tmp_path):
    with SimulatedSonar(rate_hz=100) as sim:
        sim.registers[REG_DECIMATION] = compute_decimation(20.0, 1000)  # skip the power-on pings
        config = SonarConfig(ip="127.0.0.1", port=sim.port, recv_timeout_sec=1.0, reconnect_interval_sec=0.1)
        with SonarDriver(config, log_dir=tmp_path) as driver:
            first = Path(driver.status().log_path)
            assert _wait_for(lambda: first.stat().st_size >= 5 * 6012)
            driver.start_logging()  # rotate: the first log is exported in the background
            second = Path(driver.status().log_path)
            assert _wait_for(lambda: second.stat().st_size >= 5 * 6012)
    for log in (first, second):
        ds = xr.open_dataset(log.with_suffix(".nc"))
        meta = read_sidecar(log)
        assert ds.sizes["ping"] == meta["packets"]
        assert json.loads(ds.attrs["sidecar"])["bin"] == log.name
        seconds = ds.time.values.astype("datetime64[ns]").astype(np.int64) * 1e-9
        assert abs(seconds[0] - meta["first_packet"]["unix"]) < 1e-3
        assert abs(seconds[-1] - meta["last_packet"]["unix"]) < 0.05


def test_driver_export_can_be_disabled(tmp_path):
    with SimulatedSonar(rate_hz=100) as sim:
        config = SonarConfig(ip="127.0.0.1", port=sim.port, recv_timeout_sec=1.0, reconnect_interval_sec=0.1)
        with SonarDriver(config, log_dir=tmp_path, export_netcdf=False) as driver:
            log = Path(driver.status().log_path)
            assert _wait_for(lambda: log.stat().st_size >= 2 * 6012)
    assert not list(tmp_path.glob("*.nc"))
