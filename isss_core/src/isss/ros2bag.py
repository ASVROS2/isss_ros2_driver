"""Pings from rosbag2 bags of this driver (``isss_msgs/ISSS`` on ``isss/raw``), without ROS.

Reads both storage formats Jazzy writes: MCAP (the default) and sqlite3.
Chunks compressed with lz4 need the ``lz4`` module, zstd ones ``zstandard``;
``ros2 bag record`` leaves chunks uncompressed unless told otherwise.

Each message carries its own configuration (range, sound speed, phase model),
so a bag needs no calibration options. ``header.stamp`` is the host's receive
time (with the driver's default ``use_device_time: false``; ``fit_clock`` assumes it) and ``ping_time`` the
sonar's counter; ``fit_clock`` combines the two into one time per ping.

The driver's ``isss/settings`` and the cloud node's ``isss/cloud_settings``
(JSON on latched ``std_msgs/String`` topics) hold every other setting;
``read_settings`` returns them.

Bags of the old C++ driver (``smarc_msgs/ISSS``) are read by ``isss.bag``;
``bag_kind`` tells the two apart.
"""

from __future__ import annotations

import json
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np

from isss.bag import LEGACY_TYPE, CdrReader
from isss.config import SonarConfig

ISSS_TYPE = "isss_msgs/msg/ISSS"
STRING_TYPE = "std_msgs/msg/String"
SETTINGS_FORMAT = "isss-ros-settings"
_WRAP = 2**32


@dataclass
class BagPing:
    """One ``isss_msgs/ISSS`` message; the depression arrays are left out (they follow from the counts)."""

    stamp_ns: int
    ping_time: int
    decimation: int
    range: float
    sound_velocity: float
    counts_per_unit_sine: float
    boresight_depression: float  # rad
    port_phase_sign: int
    starboard_phase_sign: int
    port_intensity: np.ndarray
    starboard_intensity: np.ndarray
    port_phase_counts: np.ndarray
    starboard_phase_counts: np.ndarray

    @property
    def num_samples(self) -> int:
        return int(self.port_intensity.size)

    @property
    def config_key(self) -> tuple:
        """What pings must share to sit in one file: everything that changes how samples are read."""
        return (self.num_samples, self.range, self.sound_velocity, self.counts_per_unit_sine,
                self.boresight_depression, self.port_phase_sign, self.starboard_phase_sign)

    def config(self) -> SonarConfig:
        from isss.sources import config_from_message_fields

        return config_from_message_fields(
            range=self.range, num_samples=self.num_samples, sound_velocity=self.sound_velocity,
            counts_per_unit_sine=self.counts_per_unit_sine, boresight_depression=self.boresight_depression,
            port_phase_sign=self.port_phase_sign, starboard_phase_sign=self.starboard_phase_sign)


def parse_isss(cdr: bytes) -> BagPing:
    """One serialized ``isss_msgs/ISSS`` (see ``isss_msgs/msg/ISSS.msg``)."""
    r = CdrReader(cdr)
    sec, nanosec = r.scalar("i"), r.scalar("I")
    r.string()  # frame_id
    r.scalar("B")  # packet_type
    ping_time = r.scalar("I")
    r.scalar("B")  # frequency_id
    decimation = r.scalar("H")
    # float32 in the message: shortest decimal that round-trips, so 1440.9 stays 1440.9
    rng, _sample_interval, sound_velocity, k, beta0 = [float(str(np.float32(r.scalar("f")))) for _ in range(5)]
    port_sign, starboard_sign = r.scalar("b"), r.scalar("b")
    port_i, starboard_i = r.sequence("<u1"), r.sequence("<u1")
    port_c, starboard_c = r.sequence("<i2"), r.sequence("<i2")
    if not port_i.size == starboard_i.size == port_c.size == starboard_c.size:
        raise ValueError("ISSS message arrays differ in length")
    return BagPing(sec * 1_000_000_000 + nanosec, ping_time, decimation, rng, sound_velocity, k, beta0,
                   port_sign, starboard_sign, port_i, starboard_i, port_c, starboard_c)


# Bags


def storage_files(bag: str | Path) -> list[Path]:
    """The ``.mcap`` or ``.db3`` files of a rosbag2 directory (or one such file), in split order."""
    bag = Path(bag)
    if bag.is_file():
        if bag.suffix not in (".mcap", ".db3"):
            raise ValueError(f"{bag} is not a rosbag2 file (.mcap or .db3)")
        return [bag]
    files = sorted([*bag.glob("*.mcap"), *bag.glob("*.db3")], key=lambda p: (len(p.name), p.name))
    if not files:
        raise ValueError(f"{bag} has no .mcap or .db3 files")
    return files


def is_bag(path: str | Path) -> bool:
    """A rosbag2 directory or storage file, of either driver."""
    path = Path(path)
    if path.is_dir():
        return any(path.glob("*.mcap")) or any(path.glob("*.db3"))
    return path.suffix in (".mcap", ".db3")


def topic_types(bag: str | Path) -> dict[str, str]:
    """``{topic: message type}`` of a bag."""
    types: dict[str, str] = {}
    for path in storage_files(bag):
        if path.suffix == ".db3":
            with _connect(path) as con:
                types.update(con.execute("SELECT name, type FROM topics"))
        else:
            with _Mcap(path) as mcap:
                types.update(mcap.topic_types())
    return types


def bag_kind(bag: str | Path) -> Optional[str]:
    """``"isss"`` for a bag of this driver, ``"legacy"`` for one of the old C++ driver, else None."""
    types = set(topic_types(bag).values())
    if ISSS_TYPE in types:
        return "isss"
    if LEGACY_TYPE in types:
        return "legacy"
    return None


def isss_topics(bag: str | Path) -> list[str]:
    return sorted(t for t, ty in topic_types(bag).items() if ty == ISSS_TYPE)


def iter_pings(bag: str | Path, topic: str | None = None) -> Iterator[BagPing]:
    """The ``isss_msgs/ISSS`` messages of a bag, in record order."""
    if topic is None:
        topics = isss_topics(bag)
        if len(topics) != 1:
            raise ValueError(f"{bag}: expected one {ISSS_TYPE} topic, found {topics or 'none'}; pass topic=")
        topic = topics[0]
    for data in _messages(bag, topic, ISSS_TYPE):
        yield parse_isss(data)


def read_settings(bag: str | Path) -> dict[str, list[dict]]:
    """``{topic: [settings, ...]}`` of the bag's settings topics, in record order.

    A settings message is JSON on a ``std_msgs/String`` topic with
    ``"format": "isss-ros-settings"``; the driver publishes one at start and on
    every change, the cloud node one at start.
    """
    found: dict[str, list[dict]] = {}
    for topic, msg_type in sorted(topic_types(bag).items()):
        if msg_type != STRING_TYPE:
            continue
        for data in _messages(bag, topic, STRING_TYPE):
            r = CdrReader(data)
            try:
                settings = json.loads(r.string())
            except ValueError:
                break  # some other String topic
            if not isinstance(settings, dict) or settings.get("format") != SETTINGS_FORMAT:
                break
            found.setdefault(topic, []).append(settings)
    return found


def _messages(bag: str | Path, topic: str, msg_type: str) -> Iterator[bytes]:
    for path in storage_files(bag):
        if path.suffix == ".db3":
            with _connect(path) as con:
                row = con.execute("SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
                if row is None:
                    continue
                if row[1] != msg_type:
                    raise ValueError(f"{topic} is {row[1]}, not {msg_type}")
                for (data,) in con.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY id", (row[0],)):
                    yield data
        else:
            with _Mcap(path) as mcap:
                yield from mcap.messages(topic, msg_type)


def read_pings(bag: str | Path, topic: str | None = None) -> list[BagPing]:
    """All ISSS pings of a bag, in ``header.stamp`` order."""
    return sorted(iter_pings(bag, topic), key=lambda p: p.stamp_ns)


def to_arrays(pings: list[BagPing]):
    """``PingArrays`` of pings with one sample count."""
    from isss.sources import pings_from_message_fields

    return pings_from_message_fields(
        [p.ping_time for p in pings], [p.decimation for p in pings],
        [p.port_intensity for p in pings], [p.starboard_intensity for p in pings],
        [p.port_phase_counts for p in pings], [p.starboard_phase_counts for p in pings])


def fit_clock(stamp_s: np.ndarray, ping_time: np.ndarray) -> np.ndarray:
    """One UTC time per ping from the receive stamps and the device counter, in stamp order.

    The sonar's counter is steady but starts at an arbitrary value each power
    cycle and drifts by a few ppm; the receive stamps are on Thor's clock but
    jitter by the network delay. Per power cycle (a break where the two
    disagree by more than a second), the stamps are fitted with a line in the
    counter: the result keeps the stamps' clock and the counter's spacing.
    Runs shorter than 10 s get the counter's nominal rate and the median offset.
    """
    stamp_s = np.asarray(stamp_s, dtype=np.float64)
    pt = np.asarray(ping_time, dtype=np.int64)
    out = np.empty_like(stamp_s)
    if stamp_s.size == 0:
        return out
    step = ((np.diff(pt) % _WRAP) * 1e-4) if pt.size > 1 else np.zeros(0)
    cuts = np.flatnonzero(np.abs(step - np.diff(stamp_s)) > 1.0) + 1
    for start, end in zip([0, *cuts.tolist()], [*cuts.tolist(), stamp_s.size]):
        device = np.concatenate([[0.0], np.cumsum(step[start : end - 1])])
        stamps = stamp_s[start:end]
        rate = 1.0
        if device[-1] > 10.0:
            fitted = np.polyfit(device, stamps - stamps[0], 1)[0]
            if abs(fitted - 1.0) < 1e-3:
                rate = fitted
        offset = np.median(stamps - device * rate)
        out[start:end] = offset + device * rate
    return out


CLOCK_SOURCE = ("header.stamp (receive time on the recording computer) fitted with a line in the device "
                "ping counter, per power cycle of the sonar: the computer's clock, the sonar's ping spacing")


# Storage


def _connect(path: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{path}?mode=ro", uri=True)


_MAGIC = b"\x89MCAP0\r\n"
_SCHEMA, _CHANNEL, _MESSAGE, _CHUNK, _FOOTER = 0x03, 0x04, 0x05, 0x06, 0x02


class _Mcap:
    """The few MCAP records a rosbag2 file needs: schemas, channels, messages, chunks (https://mcap.dev/spec)."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._fh = path.open("rb")
        self.data = self._fh.read()
        if self.data[:8] != _MAGIC:
            raise ValueError(f"{path} is not an MCAP file")

    def __enter__(self) -> "_Mcap":
        return self

    def __exit__(self, *exc) -> None:
        self._fh.close()

    def topic_types(self) -> dict[str, str]:
        schemas: dict[int, str] = {}
        topics: dict[str, str] = {}
        summary = self._summary()
        records = self._records(summary, into_chunks=False) if summary is not None else self._records(
            self._body(), into_chunks=True)
        for op, body in records:
            if op == _SCHEMA:
                sid, name = struct.unpack_from("<H", body)[0], _string(body, 2)[0]
                schemas[sid] = name
            elif op == _CHANNEL:
                cid, sid = struct.unpack_from("<HH", body)
                topics[_string(body, 4)[0]] = sid
        return {t: schemas.get(sid, "") for t, sid in topics.items()}

    def messages(self, topic: str, msg_type: str) -> Iterator[bytes]:
        schemas: dict[int, str] = {}
        wanted: set[int] = set()
        for op, body in self._records(self._body(), into_chunks=True):
            if op == _SCHEMA:
                schemas[struct.unpack_from("<H", body)[0]] = _string(body, 2)[0]
            elif op == _CHANNEL:
                cid, sid = struct.unpack_from("<HH", body)
                if _string(body, 4)[0] == topic:
                    if schemas.get(sid, msg_type) != msg_type:
                        raise ValueError(f"{topic} is {schemas[sid]}, not {msg_type}")
                    wanted.add(cid)
            elif op == _MESSAGE and struct.unpack_from("<H", body)[0] in wanted:
                yield bytes(body[22:])

    def _body(self) -> memoryview:
        return memoryview(self.data)[8:]

    def _summary(self) -> Optional[memoryview]:
        """The summary section (schemas and channels without reading chunks), if the file has one."""
        end = len(self.data) - 8
        footer = end - (1 + 8 + 8 + 8 + 4)
        if footer < 8 or self.data[end:] != _MAGIC or self.data[footer] != _FOOTER:
            return None
        start = struct.unpack_from("<Q", self.data, footer + 9)[0]
        return memoryview(self.data)[start:footer] if start else None

    def _records(self, buf: memoryview, *, into_chunks: bool) -> Iterator[tuple[int, memoryview]]:
        pos = 0
        while pos + 9 <= len(buf):
            op, length = struct.unpack_from("<BQ", buf, pos)
            body = buf[pos + 9 : pos + 9 + length]
            pos += 9 + length
            if op == _FOOTER:
                return
            if op == _CHUNK:
                if into_chunks:
                    yield from self._records(memoryview(self._chunk(body)), into_chunks=False)
                continue
            yield op, body

    def _chunk(self, body: memoryview) -> bytes:
        size = struct.unpack_from("<Q", body, 16)[0]
        compression, at = _string(body, 28)
        n = struct.unpack_from("<Q", body, at)[0]
        records = bytes(body[at + 8 : at + 8 + n])
        if compression == "":
            return records
        if compression == "lz4":
            try:
                import lz4.frame
            except ImportError:
                raise ValueError(f"{self.path} has lz4 chunks: install lz4, or "
                                 "ros2 bag convert it to uncompressed") from None
            return lz4.frame.decompress(records)
        if compression == "zstd":
            try:
                import zstandard
            except ImportError:
                raise ValueError(f"{self.path} has zstd chunks: install zstandard, or "
                                 "ros2 bag convert it to uncompressed") from None
            return zstandard.ZstdDecompressor().decompress(records, max_output_size=size)
        raise ValueError(f"{self.path}: unknown chunk compression {compression!r}")


def _string(buf, at: int) -> tuple[str, int]:
    """An MCAP string at ``at`` and the offset after it."""
    n = struct.unpack_from("<I", buf, at)[0]
    return bytes(buf[at + 4 : at + 4 + n]).decode(), at + 4 + n
