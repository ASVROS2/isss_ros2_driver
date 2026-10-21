"""Utilities for reading recorded ISSS binary logs."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import BinaryIO, Iterator

from isss.config import SonarConfig
from isss.protocol import (
    BYTES_PER_SAMPLE,
    HEADER_SIZE,
    PACKET_TYPE_ISSS,
    SOP1,
    SOP2,
    SampleData,
    parse_header,
    parse_payload,
)


def find_packet_offsets(
    data: bytes,
    *,
    num_samples: int = 1000,
    max_offset: int | None = None,
) -> list[int]:
    """Find byte offsets of ISSS packet headers in a binary log."""
    bytes_per_ping = HEADER_SIZE + BYTES_PER_SAMPLE * num_samples
    search_end = len(data) - 4
    if max_offset is not None:
        search_end = min(search_end, max_offset)

    indices: list[int] = []
    for i in range(max(0, search_end)):
        if (
            data[i] == SOP1
            and data[i + 1] == SOP2
            and data[i + 4] == PACKET_TYPE_ISSS
        ):
            indices.append(i)

    return indices


def iter_packets_from_log(
    path: str | Path,
    *,
    num_samples: int = 1000,
    config: SonarConfig | None = None,
) -> Iterator[tuple[SampleData, bytes]]:
    """Yield decoded pings from a binary log file."""
    cfg = config or SonarConfig(num_samples=num_samples)
    data = Path(path).read_bytes()
    bytes_per_ping = HEADER_SIZE + BYTES_PER_SAMPLE * num_samples

    for offset in find_packet_offsets(data, num_samples=num_samples):
        packet = data[offset : offset + bytes_per_ping]
        if len(packet) < bytes_per_ping:
            break
        yield parse_payload(packet[HEADER_SIZE:], num_samples, cfg), packet


def read_log(
    path: str | Path,
    *,
    num_samples: int = 1000,
    config: SonarConfig | None = None,
) -> list[tuple[SampleData, bytes]]:
    """Read all complete pings from a binary log file."""
    return list(iter_packets_from_log(path, num_samples=num_samples, config=config))


class BinLogWriter:
    """Append raw packets to ``<directory>/<unix time>.bin``, the format the drivers have always written.

    Thread-safe: ``write`` may be called from a read loop while ``open``/``close``
    are called from elsewhere.
    """

    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory)
        self.path: Path | None = None
        self._file: BinaryIO | None = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._file is not None

    def open(self) -> Path:
        """Start a new log file, closing any current one; returns its path."""
        with self._lock:
            self._close_unlocked()
            self.directory.mkdir(parents=True, exist_ok=True)
            self.path = self.directory / f"{time.time():.6f}.bin"
            self._file = self.path.open("wb")
            return self.path

    def write(self, packet: bytes) -> None:
        with self._lock:
            if self._file is not None:
                self._file.write(packet)

    def close(self) -> None:
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
