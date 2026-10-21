"""Raw ISSS packets from rosbag2 recordings of the old C++ driver (``smarc_msgs/ISSS``).

Those bags (e.g. Beckholmen, 2025-12-14) predate ``.bin`` logging alongside the
bag, but each message keeps enough to rebuild the packet the sonar sent:

- ``extra_channel`` is the 12-byte packet header (``0xFE 0x05``, size, type,
  ping time, frequency id, decimation). The message's own ``type``, ``time``
  and ``decimation`` fields are left at 0.
- ``port_channel`` / ``starboard_channel`` are the RSS bytes.
- ``*_phase_difference`` is the unsigned 16-bit phase word divided by 512,
  exact in float32, so ``round(value * 512)`` recovers it.

The bag is read with ``sqlite3`` and a fixed CDR layout, so no ROS install or
bag library is needed. Only sqlite3 storage is supported.
"""

from __future__ import annotations

import sqlite3
import struct
from pathlib import Path
from typing import Iterator

import numpy as np

from isss.protocol import HEADER_SIZE, SOP1, SOP2

LEGACY_TYPE = "smarc_msgs/msg/ISSS"


def bag_files(bag: str | Path) -> list[Path]:
    """The ``.db3`` files of a rosbag2 directory (or a single ``.db3``), in split order."""
    bag = Path(bag)
    if bag.is_file():
        return [bag]
    files = sorted(bag.glob("*.db3"), key=lambda p: (len(p.name), p.name))
    if not files:
        raise ValueError(f"{bag} has no .db3 files (only sqlite3 bags are supported)")
    return files


def isss_topics(bag: str | Path) -> list[str]:
    """Topics of ``smarc_msgs/ISSS`` messages in a bag."""
    topics: set[str] = set()
    for path in bag_files(bag):
        with _connect(path) as con:
            topics.update(name for name, in con.execute("SELECT name FROM topics WHERE type = ?", (LEGACY_TYPE,)))
    return sorted(topics)


def iter_bag_packets(bag: str | Path, topic: str | None = None) -> Iterator[tuple[int, bytes]]:
    """``(header stamp [ns], raw packet)`` for each ISSS message, in record order.

    Record order is not always time order (a recording clock that stepped
    leaves messages out of order); sort by the stamp if that matters.
    """
    if topic is None:
        topics = isss_topics(bag)
        if len(topics) != 1:
            raise ValueError(f"{bag}: expected one {LEGACY_TYPE} topic, found {topics or 'none'}; pass topic=")
        topic = topics[0]
    for path in bag_files(bag):
        with _connect(path) as con:
            row = con.execute("SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
            if row is None:
                continue
            if row[1] != LEGACY_TYPE:
                raise ValueError(f"{topic} is {row[1]}, not {LEGACY_TYPE}")
            for (data,) in con.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY id", (row[0],)):
                yield legacy_message_to_packet(data)


def legacy_message_to_packet(cdr: bytes) -> tuple[int, bytes]:
    """Rebuild ``(header stamp [ns], raw packet)`` from one serialized ``smarc_msgs/ISSS``."""
    r = _CdrReader(cdr)
    sec, nanosec = r.scalar("i"), r.scalar("I")
    r.string()  # frame_id
    for fmt in ("B", "I", "B", "f", "h", "H", "f", "f", "f"):
        r.scalar(fmt)  # type, time, frequency_id, frequency, gain, decimation, max_duration, baseline, c
    port_rss, starboard_rss = r.sequence("<u1"), r.sequence("<u1")
    port_phase, starboard_phase = r.sequence("<f4"), r.sequence("<f4")
    r.sequence("<f4"), r.sequence("<f4")  # bearings, not meaningful
    header = r.sequence("<u1")
    if header.size < HEADER_SIZE or header[0] != SOP1 or header[1] != SOP2:
        raise ValueError("extra_channel does not hold an ISSS packet header")
    n = port_rss.size
    if not starboard_rss.size == port_phase.size == starboard_phase.size == n:
        raise ValueError("ISSS message channels differ in length")
    words = []
    for phase in (port_phase, starboard_phase):
        word = np.round(phase.astype(np.float64) * 512.0)
        if not ((word >= 0) & (word < 65536)).all():
            raise ValueError("phase is not in the legacy raw / 512 encoding")
        words.append(word.astype(np.uint16))
    payload = np.empty((n, 6), dtype=np.uint8)
    payload[:, 0], payload[:, 1], payload[:, 2] = words[0] >> 8, words[0] & 0xFF, port_rss
    payload[:, 3], payload[:, 4], payload[:, 5] = words[1] >> 8, words[1] & 0xFF, starboard_rss
    return sec * 1_000_000_000 + nanosec, header[:HEADER_SIZE].tobytes() + payload.tobytes()


def _connect(path: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{path}?mode=ro", uri=True)


class _CdrReader:
    """Little-endian CDR; alignment is relative to the end of the 4-byte encapsulation header."""

    def __init__(self, data: bytes) -> None:
        if data[:2] != b"\x00\x01":
            raise ValueError("only little-endian CDR is supported")
        self.data, self.offset = data, 4

    def _align(self, n: int) -> None:
        self.offset += -(self.offset - 4) % n

    def scalar(self, fmt: str):
        size = struct.calcsize(fmt)
        self._align(size)
        (value,) = struct.unpack_from("<" + fmt, self.data, self.offset)
        self.offset += size
        return value

    def string(self) -> str:
        n = self.scalar("I")
        text = self.data[self.offset : self.offset + n - 1].decode()
        self.offset += n
        return text

    def sequence(self, dtype: str) -> np.ndarray:
        n = self.scalar("I")
        item = np.dtype(dtype).itemsize
        self._align(item)
        values = np.frombuffer(self.data, dtype=dtype, count=n, offset=self.offset)
        self.offset += n * item
        return values
