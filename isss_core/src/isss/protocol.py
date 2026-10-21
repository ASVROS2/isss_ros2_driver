"""ISSS packet format constants and parsing."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isss.config import SonarConfig

HEADER_SIZE = 12
DEFAULT_SAMPLES = 1000
BYTES_PER_SAMPLE = 6
PACKET_TYPE_ISSS = 0xE1
PACKET_TYPE_SONAR_8BIT = 0xE2
FREQ_ID_DEFAULT = 0x01
SOP1 = 0xFE
SOP2 = 0x05

_MAX_POSITIVE = 32768
_FULL_RANGE = 65536


@dataclass
class PacketHeader:
    """Parsed 12-byte ISSS packet header."""

    sop1: int = 0
    sop2: int = 0
    packet_size: int = 0
    packet_type: int = 0
    ping_time: int = 0
    frequency_id: int = 0
    decimation: int = 0

    @property
    def ping_time_sec(self) -> float:
        """Ping timestamp in seconds (device counter, 100 µs units)."""
        return self.ping_time * 1e-4

    @property
    def is_valid(self) -> bool:
        return self.sop1 == SOP1 and self.sop2 == SOP2


@dataclass
class SampleData:
    """Decoded interferometric sidescan samples for one ping."""

    port_rss: list[int]
    starboard_rss: list[int]
    port_phase_diff: list[float]
    starboard_phase_diff: list[float]
    port_bearing: list[float]
    starboard_bearing: list[float]


def compute_decimation(range_m: float, num_samples: int) -> int:
    """Compute the decimation register value from range and sample count."""
    return int(range_m / num_samples / 750.0 * (12.5e6 / 3.0))


def data_size(num_samples: int) -> int:
    return num_samples * BYTES_PER_SAMPLE


def packet_size(num_samples: int) -> int:
    return HEADER_SIZE + data_size(num_samples)


def parse_header(header: bytes) -> PacketHeader:
    """Parse a 12-byte ISSS packet header."""
    if len(header) < HEADER_SIZE:
        raise ValueError(f"Header must be at least {HEADER_SIZE} bytes, got {len(header)}")

    sop1, sop2, packet_size, packet_type = struct.unpack(">BBHB", header[:5])
    ping_time = struct.unpack(">I", header[5:9])[0]
    frequency_id = header[9]
    decimation = struct.unpack(">H", header[10:12])[0]

    return PacketHeader(
        sop1=sop1,
        sop2=sop2,
        packet_size=packet_size,
        packet_type=packet_type,
        ping_time=ping_time,
        frequency_id=frequency_id,
        decimation=decimation,
    )


def _decode_phase(high: int, low: int) -> float:
    phase = (high << 8) | low
    if phase > _MAX_POSITIVE:
        phase -= _FULL_RANGE
    return float(phase)


def _phase_to_radians(raw_phase: float) -> float:
    return (raw_phase / 512.0) * (math.pi / 180.0)


def _compute_bearing(raw_phase: float, config: SonarConfig) -> float:
    asin_input = (raw_phase * config.sound_velocity) / (
        config.carrier_frequency * config.baseline
    )
    asin_input = max(-1.0, min(1.0, asin_input))
    return math.asin(asin_input)


def parse_payload(payload: bytes, num_samples: int, config: SonarConfig) -> SampleData:
    """Parse the sample payload for one ISSS ping."""
    expected = num_samples * BYTES_PER_SAMPLE
    if len(payload) < expected:
        raise ValueError(f"Payload must be at least {expected} bytes, got {len(payload)}")

    port_rss: list[int] = []
    starboard_rss: list[int] = []
    port_phase_diff: list[float] = []
    starboard_phase_diff: list[float] = []
    port_bearing: list[float] = []
    starboard_bearing: list[float] = []

    for i in range(num_samples):
        offset = i * BYTES_PER_SAMPLE
        port_raw = _decode_phase(payload[offset], payload[offset + 1])
        port_rss.append(payload[offset + 2])
        port_phase_diff.append(_phase_to_radians(port_raw))
        port_bearing.append(_compute_bearing(port_raw, config))

        starboard_raw = _decode_phase(payload[offset + 3], payload[offset + 4])
        starboard_rss.append(payload[offset + 5])
        starboard_phase_diff.append(_phase_to_radians(starboard_raw))
        starboard_bearing.append(_compute_bearing(starboard_raw, config))

    return SampleData(
        port_rss=port_rss,
        starboard_rss=starboard_rss,
        port_phase_diff=port_phase_diff,
        starboard_phase_diff=starboard_phase_diff,
        port_bearing=port_bearing,
        starboard_bearing=starboard_bearing,
    )


@dataclass
class Ping:
    """One decoded ISSS packet: header, samples, and the raw bytes it came from."""

    header: PacketHeader
    samples: SampleData
    raw: bytes


def parse_packet(packet: bytes, config: SonarConfig) -> Ping:
    """Parse a full packet (header + payload) using ``config.num_samples``."""
    header = parse_header(packet[:HEADER_SIZE])
    samples = parse_payload(packet[HEADER_SIZE:], config.num_samples, config)
    return Ping(header=header, samples=samples, raw=bytes(packet))


def is_packet_sync_byte(
    header_window: bytes,
    *,
    packet_type: int = PACKET_TYPE_ISSS,
    frequency_id: int = FREQ_ID_DEFAULT,
) -> bool:
    """Return True if the 12-byte window looks like a valid ISSS packet header."""
    if len(header_window) < HEADER_SIZE:
        return False
    return (
        header_window[0] == SOP1
        and header_window[1] == SOP2
        and header_window[4] == packet_type
        and header_window[9] == frequency_id
    )
