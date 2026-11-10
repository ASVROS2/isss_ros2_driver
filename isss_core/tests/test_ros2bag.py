"""Bags of this driver (isss_msgs/ISSS, MCAP and sqlite3): reading, the clock fit, export and the commands.

The bags are written here byte by byte, so no ROS install is needed; the reader
was also checked against rosbag2_py on bags recorded from the driver.
"""

import math
import sqlite3
import struct
from pathlib import Path

import numpy as np
import pytest

from isss.config import SonarConfig
from isss.diagnostics import decode_packets
from isss.protocol import parse_packet
from isss.ros2bag import ISSS_TYPE, bag_kind, fit_clock, parse_isss, read_pings, topic_types
from isss.sim import synthetic_packet
from isss.sources import load_recording

NUM_SAMPLES = 300
T0 = 1_794_238_800.0  # Thor time of the first ping [s]
DEVICE0 = 123_456_789  # device counter at the first ping [100 us]


def _packets(n, decimation=111):
    return [synthetic_packet(num_samples=NUM_SAMPLES, ping_time=(DEVICE0 + 268 * i) % 2**32, decimation=decimation,
                             bottom_sample=60 + i % 20) for i in range(n)]


class _Cdr:
    def __init__(self):
        self.buf = bytearray(b"\x00\x01\x00\x00")

    def put(self, fmt, value):
        size = struct.calcsize(fmt)
        self.buf += b"\x00" * (-(len(self.buf) - 4) % size)
        self.buf += struct.pack("<" + fmt, value)

    def string(self, text):
        self.put("I", len(text) + 1)
        self.buf += text.encode() + b"\x00"

    def seq(self, values, dtype):
        values = np.asarray(values, dtype=dtype)
        self.put("I", values.size)
        self.buf += b"\x00" * (-(len(self.buf) - 4) % values.itemsize)
        self.buf += values.tobytes()


def _message(packet, stamp_ns, k=15218.0, beta0_deg=44.5, signs=(1, 1), sound_velocity=1440.9, range_m=20.0):
    """A serialized isss_msgs/ISSS, field by field as in ISSS.msg."""
    ping = parse_packet(packet, SonarConfig(num_samples=NUM_SAMPLES))
    s, h = ping.samples, ping.header
    c = _Cdr()
    c.put("i", stamp_ns // 10**9)
    c.put("I", stamp_ns % 10**9)
    c.string("isss_link")
    c.put("B", 0xE1)
    c.put("I", h.ping_time)
    c.put("B", 0)
    c.put("H", h.decimation)
    for v in (range_m, h.decimation * 3 / 12.5e6, sound_velocity, k, math.radians(beta0_deg)):
        c.put("f", v)
    c.put("b", signs[0])
    c.put("b", signs[1])
    c.seq(s.port_rss, np.uint8)
    c.seq(s.starboard_rss, np.uint8)
    c.seq(s.port_phase_counts, np.int16)
    c.seq(s.starboard_phase_counts, np.int16)
    c.seq(np.zeros(NUM_SAMPLES), np.float32)
    c.seq(np.zeros(NUM_SAMPLES), np.float32)
    return bytes(c.buf)


def _sqlite_bag(directory: Path, messages, topic="/isss/raw"):
    directory.mkdir()
    con = sqlite3.connect(directory / f"{directory.name}_0.db3")
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, "
                "offered_qos_profiles TEXT, type_description_hash TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    con.execute("INSERT INTO topics VALUES (1, ?, ?, 'cdr', '', '')", (topic, ISSS_TYPE))
    con.execute("INSERT INTO topics VALUES (2, '/isss/status', 'isss_msgs/msg/ISSSStatus', 'cdr', '', '')")
    for data in messages:
        con.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, 0, ?)", (data,))
    con.commit()
    con.close()
    return directory


def _mcap_bag(directory: Path, messages, compression="", summary=True):
    def string(text):
        return struct.pack("<I", len(text)) + text.encode()

    def record(op, body):
        return struct.pack("<BQ", op, len(body)) + body

    schema = record(0x03, struct.pack("<H", 1) + string(ISSS_TYPE) + string("ros2msg") + struct.pack("<I", 0))
    status_schema = record(0x03, struct.pack("<H", 2) + string("isss_msgs/msg/ISSSStatus") + string("ros2msg")
                           + struct.pack("<I", 0))
    channel = record(0x04, struct.pack("<HH", 1, 1) + string("/isss/raw") + string("cdr") + struct.pack("<I", 0))
    status = record(0x04, struct.pack("<HH", 2, 2) + string("/isss/status") + string("cdr") + struct.pack("<I", 0))
    records = schema + channel + status_schema + status + b"".join(
        record(0x05, struct.pack("<HIQQ", 1, i, 0, 0) + data) for i, data in enumerate(messages))
    payload = records
    if compression == "lz4":
        import lz4.frame

        payload = lz4.frame.compress(records)
    chunk = record(0x06, struct.pack("<QQQI", 0, 0, len(records), 0) + string(compression)
                   + struct.pack("<Q", len(payload)) + payload)
    head = b"\x89MCAP0\r\n" + record(0x01, string("ros2") + string("test"))
    data = head + chunk + record(0x0F, struct.pack("<I", 0))
    summary_start = len(data) if summary else 0
    if summary:
        data += schema + channel + status_schema + status
    data += record(0x02, struct.pack("<QQI", summary_start, 0, 0)) + b"\x89MCAP0\r\n"
    directory.mkdir()
    (directory / f"{directory.name}_0.mcap").write_bytes(data)
    return directory


def _stamps(n, jitter_s=0.015, seed=0):
    """Receive stamps [ns]: device spacing with a 4 ppm drift, plus up to jitter_s of delay."""
    rng = np.random.default_rng(seed)
    device = np.arange(n) * 268 * 1e-4
    return T0 + device * (1 + 4e-6) + rng.uniform(0, jitter_s, n), T0 + device * (1 + 4e-6)


@pytest.mark.parametrize("storage", ["mcap", "mcap-no-summary", "sqlite3"])
def test_bags_read_back_in_stamp_order(tmp_path, storage):
    packets = _packets(6)
    stamps = [int(T0 * 1e9) + i * 26_800_000 for i in range(6)]
    order = [3, 4, 5, 0, 1, 2]
    messages = [_message(packets[i], stamps[i]) for i in order]
    if storage == "sqlite3":
        bag = _sqlite_bag(tmp_path / "bag", messages)
    else:
        bag = _mcap_bag(tmp_path / "bag", messages, summary=storage == "mcap")
    assert topic_types(bag) == {"/isss/raw": ISSS_TYPE, "/isss/status": "isss_msgs/msg/ISSSStatus"}
    assert bag_kind(bag) == "isss"
    pings = read_pings(bag)
    assert [p.stamp_ns for p in pings] == stamps
    expected = decode_packets(packets, NUM_SAMPLES)
    arrays, config = load_recording(bag)
    for name in ("ping_time", "decimation", "intensity", "counts"):
        np.testing.assert_array_equal(getattr(arrays, name), getattr(expected, name), err_msg=name)
    assert config.counts_per_unit_sine == 15218.0 and config.sound_velocity == 1440.9
    assert config.boresight_depression_deg == pytest.approx(44.5, abs=1e-5) and config.num_samples == NUM_SAMPLES


def test_lz4_chunks(tmp_path):
    pytest.importorskip("lz4")
    packets = _packets(3)
    bag = _mcap_bag(tmp_path / "bag", [_message(p, int(T0 * 1e9) + i) for i, p in enumerate(packets)],
                    compression="lz4")
    assert len(read_pings(bag)) == 3


def test_message_fields():
    packet = _packets(1)[0]
    ping = parse_isss(_message(packet, 1_794_238_800_123_456_789, k=14000.0, signs=(-1, 1), range_m=30.0))
    assert ping.stamp_ns == 1_794_238_800_123_456_789 and ping.ping_time == DEVICE0 and ping.decimation == 111
    assert (ping.range, ping.counts_per_unit_sine, ping.port_phase_sign, ping.starboard_phase_sign) == (
        30.0, 14000.0, -1, 1)
    assert ping.config().port_phase_sign == -1 and ping.config().range_m == 30.0


def test_clock_fit_removes_receive_jitter_and_follows_drift():
    received, true = _stamps(5000)  # 134 s
    pt = (DEVICE0 + 268 * np.arange(5000)) % 2**32
    fitted = fit_clock(received, pt)
    # A constant mean latency stays in (it is part of when Thor saw the ping); the jitter goes
    residual = fitted - true
    assert np.ptp(residual) < 1e-3 and abs(residual.mean() - 0.0075) < 1e-3


def test_clock_fit_handles_a_reboot_and_a_counter_wrap():
    received, _ = _stamps(2000, jitter_s=0.0)
    pt = (2**32 - 268 * 500 + 268 * np.arange(2000)) % 2**32  # wraps after 500 pings
    pt[1200:] = 5 + 268 * np.arange(800)  # power cycle: the counter starts over
    received[1200:] += 30.0  # 30 s off while it booted
    fitted = fit_clock(received, pt)
    np.testing.assert_allclose(fitted, received, atol=1e-6)


def test_export_takes_calibration_and_splits_where_it_changes(tmp_path):
    pytest.importorskip("h5netcdf")
    xr = pytest.importorskip("xarray")
    from isss.netcdf import export_isss_bag

    packets = _packets(8)
    received, true = _stamps(8)
    messages = [_message(p, int(round(t * 1e9)), k=14000.0 if i < 5 else 15000.0, signs=(-1, 1))
                for i, (p, t) in enumerate(zip(packets, received))]
    bag = _mcap_bag(tmp_path / "rosbag2_x", messages)
    paths = export_isss_bag(bag)
    assert [p.name for p in paths] == ["rosbag2_x.nc", "rosbag2_x_2.nc"]
    first, second = (xr.open_dataset(p) for p in paths)
    assert first.attrs["counts_per_unit_sine"] == 14000.0 and second.attrs["counts_per_unit_sine"] == 15000.0
    assert first.attrs["port_phase_sign"] == -1 and "header.stamp" in first.attrs["time_source"]
    expected = decode_packets(packets, NUM_SAMPLES)
    np.testing.assert_array_equal(np.concatenate([first.phase_counts.values, second.phase_counts.values]),
                                  expected.counts)
    np.testing.assert_allclose(first.depression.values, expected.select(slice(0, 5)).depression(
        SonarConfig(counts_per_unit_sine=14000.0, boresight_depression_deg=44.5, port_phase_sign=-1)), rtol=1e-6, atol=1e-6)
    seconds = first.time.values.astype("datetime64[ns]").astype(np.int64) * 1e-9
    np.testing.assert_allclose(seconds, received[:5], atol=0.02)


def test_commands_take_new_bags_and_options_override_the_messages(tmp_path, capsys):
    pytest.importorskip("h5netcdf")
    xr = pytest.importorskip("xarray")
    from isss.cli import _check_input, main_export

    bag = _sqlite_bag(tmp_path / "rosbag2_y", [_message(p, int(T0 * 1e9) + i) for i, p in enumerate(_packets(3))])
    with pytest.raises(SystemExit) as exit_:
        main_export([str(bag), "--k", "13000", "--flip-starboard"])
    assert exit_.value.code == 0
    ds = xr.open_dataset(tmp_path / "rosbag2_y.nc")
    assert ds.attrs["counts_per_unit_sine"] == 13000.0 and ds.attrs["starboard_phase_sign"] == -1
    assert ds.attrs["sound_velocity"] == pytest.approx(1440.9)  # not given: from the messages
    assert _check_input(bag, legacy=False) is None
    assert "not a bag of the old" in _check_input(bag, legacy=True)
    capsys.readouterr()
