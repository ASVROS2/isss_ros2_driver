"""Utilities for reading recorded ISSS binary logs."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import BinaryIO, Callable, Iterable, Iterator, Optional

from isss import sidecar

from isss.config import SonarConfig
from isss.protocol import (
    BYTES_PER_SAMPLE,
    HEADER_SIZE,
    SOP1,
    SOP2,
    SampleData,
    is_packet_sync_byte,
    parse_payload,
)

SOP_BYTES = bytes([SOP1, SOP2])


def find_packet_offsets(
    data: bytes,
    *,
    num_samples: int = 1000,
    max_offset: int | None = None,
) -> list[int]:
    """Byte offsets of the ISSS packets in a binary log.

    Frames like the live reader: a packet starts where the header pattern
    matches (``is_packet_sync_byte``), and the search resumes after its payload,
    so matches inside a payload are not taken for packets.
    """
    bytes_per_ping = HEADER_SIZE + BYTES_PER_SAMPLE * num_samples
    search_end = len(data) - HEADER_SIZE + 1
    if max_offset is not None:
        search_end = min(search_end, max_offset)

    indices: list[int] = []
    i = data.find(SOP_BYTES, 0, max(0, search_end + 1))
    while 0 <= i < search_end:
        if is_packet_sync_byte(data[i : i + HEADER_SIZE]):
            indices.append(i)
            i += bytes_per_ping
        else:
            i += 1
        i = data.find(SOP_BYTES, i, search_end + 1)
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
    """Append raw packets to ``<directory>/<unix time>.bin``, with a JSON sidecar.

    The ``.bin`` format is what the drivers have always written: packets
    exactly as received, back to back. The sidecar (``isss.sidecar``) records
    the configuration, register writes and timing the ``.bin`` cannot.

    Thread-safe: ``write`` may be called from a read loop while ``open``,
    ``note_setup`` and ``close`` are called from elsewhere. ``on_closed`` is
    called with the ``.bin`` path after each log is closed, outside the lock.
    """

    def __init__(self, directory: str | Path, on_closed: Optional[Callable[[Path], None]] = None) -> None:
        self.directory = Path(directory)
        self.on_closed = on_closed
        self.path: Path | None = None
        self._file: BinaryIO | None = None
        self._sidecar: dict | None = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._file is not None

    @property
    def sidecar(self) -> dict | None:
        with self._lock:
            return None if self._sidecar is None else dict(self._sidecar)

    def open(self, config: SonarConfig, register_writes: Iterable[tuple[int, int]] = ()) -> Path:
        """Start a new log, closing any current one; returns the ``.bin`` path.

        ``register_writes`` are the writes that configured the sonar for it.
        """
        with self._lock:
            closed = self._close_unlocked()
            self.directory.mkdir(parents=True, exist_ok=True)
            self.path = self.directory / f"{time.time():.6f}.bin"
            self._file = self.path.open("wb")
            self._sidecar = sidecar.new_sidecar(self.path, config, register_writes)
            self._config_key = _config_key(config)
            sidecar.write_sidecar(self.path, self._sidecar)
            path = self.path
        self._notify(closed)
        return path

    def note_setup(self, config: SonarConfig, register_writes: Iterable[tuple[int, int]]) -> None:
        """Record register writes, and a new segment if the configuration changed."""
        with self._lock:
            if self._sidecar is None or self.path is None:
                return
            self._sidecar["register_writes"].extend(sidecar.register_entries(register_writes))
            if _config_key(config) != self._config_key:
                self._sidecar["segments"].append(sidecar.segment(config, self._sidecar["bytes"]))
                self._config_key = _config_key(config)
            sidecar.write_sidecar(self.path, self._sidecar)

    def write(self, packet: bytes) -> None:
        with self._lock:
            if self._file is None or self._sidecar is None:
                return
            self._file.write(packet)
            stamp = {
                "utc": sidecar.utc_now(),
                "unix": time.time(),
                "ping_time": int.from_bytes(packet[5:9], "big") if len(packet) >= HEADER_SIZE else None,
            }
            first = self._sidecar["first_packet"] is None
            if first:
                self._sidecar["first_packet"] = stamp
            self._sidecar["last_packet"] = stamp
            self._sidecar["packets"] += 1
            self._sidecar["bytes"] += len(packet)
            if first:
                assert self.path is not None
                sidecar.write_sidecar(self.path, self._sidecar)

    def close(self) -> None:
        with self._lock:
            closed = self._close_unlocked()
        self._notify(closed)

    def _close_unlocked(self) -> Optional[Path]:
        if self._file is None:
            return None
        self._file.close()
        self._file = None
        if self._sidecar is not None and self.path is not None:
            self._sidecar["closed_utc"] = sidecar.utc_now()
            sidecar.write_sidecar(self.path, self._sidecar)
        return self.path

    def _notify(self, closed: Optional[Path]) -> None:
        if closed is not None and self.on_closed is not None:
            self.on_closed(closed)


def _config_key(config: SonarConfig) -> tuple:
    return (config.range_m, config.num_samples, config.num_phases)
