"""Utilities for reading recorded ISSS binary logs."""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

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
