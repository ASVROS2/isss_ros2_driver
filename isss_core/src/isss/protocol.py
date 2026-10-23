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

_FULL_RANGE = 65536
REGISTER_MAX = 0xFFFF  # registers are 16 bits wide

# One electrical phase cycle (2 pi) is 20480 counts: the counts fill +-10240
# and wrap there (sonar3d_reference mesh check, Oct 2026; docs/protocol.md).
PHASE_COUNTS_PER_CYCLE = 20480

SAMPLE_CLOCK_HZ = 12.5e6
SAMPLE_CLOCK_DIVIDER = 3
# Recorded logs ping once per two-way travel time at roughly this speed.
PING_PACING_SOUND_SPEED = 1500.0


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
    """Decoded interferometric sidescan samples for one ping, one entry per range bin.

    ``*_phase_counts`` are the signed hardware values and lose nothing.
    ``*_phase_diff`` is the same phase in radians. ``*_depression`` is the
    angle below the sensor frame's horizontal, toward that side, from the
    interferometer model (``depression_from_counts``); NaN where the model
    has no solution. See docs/protocol.md, "Phase and depression".
    """

    port_rss: list[int]
    starboard_rss: list[int]
    port_phase_counts: list[int]
    starboard_phase_counts: list[int]
    port_phase_diff: list[float]
    starboard_phase_diff: list[float]
    port_depression: list[float]
    starboard_depression: list[float]


def compute_decimation(range_m: float, num_samples: int) -> int:
    """Compute the decimation register value from range and sample count."""
    return int(range_m / num_samples / 750.0 * (SAMPLE_CLOCK_HZ / SAMPLE_CLOCK_DIVIDER))


def check_settings(range_m: float, num_samples: int, num_phases: int) -> None:
    """Raise ``ValueError`` unless the settings fit the 16-bit registers.

    ``set_register`` would otherwise silently write the low 16 bits.
    """
    if range_m <= 0:
        raise ValueError(f"range must be > 0, got {range_m}")
    if not 0 < num_samples <= REGISTER_MAX:
        raise ValueError(f"num_samples must be in [1, {REGISTER_MAX}], got {num_samples}")
    if not 0 <= num_phases <= REGISTER_MAX:
        raise ValueError(f"num_phases must be in [0, {REGISTER_MAX}], got {num_phases}")
    decimation = compute_decimation(range_m, num_samples)
    if not 0 < decimation <= REGISTER_MAX:
        raise ValueError(
            f"range {range_m} m at {num_samples} samples gives decimation {decimation}, "
            f"outside [1, {REGISTER_MAX}]"
        )


def range_from_decimation(decimation: int, num_samples: int) -> float:
    """Invert ``compute_decimation``: the range a decimation and sample count correspond to."""
    return decimation * num_samples * 750.0 / (SAMPLE_CLOCK_HZ / SAMPLE_CLOCK_DIVIDER)


def sample_interval(decimation: int) -> float:
    """Time between range bins in seconds: ``decimation * 3 / 12.5 MHz``."""
    return decimation * SAMPLE_CLOCK_DIVIDER / SAMPLE_CLOCK_HZ


def expected_ping_interval(range_m: float) -> float:
    """Ping period seen in every recorded log: one two-way travel time at about 1500 m/s."""
    return 2.0 * range_m / PING_PACING_SOUND_SPEED


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


def _decode_phase(high: int, low: int) -> int:
    phase = (high << 8) | low
    if phase >= 0x8000:  # two's complement
        phase -= _FULL_RANGE
    return phase


def phase_counts_to_radians(counts: float) -> float:
    """Electrical phase difference in radians; 20480 counts are one cycle."""
    return counts * (2.0 * math.pi / PHASE_COUNTS_PER_CYCLE)


def depression_from_counts(
    counts: float,
    counts_per_unit_sine: float,
    boresight_depression_rad: float,
    phase_sign: int = 1,
) -> float:
    """Angle below horizontal of a return, from its phase counts.

    Two-element interferometer: ``counts = sign * K * sin(beta0 - beta)``, so
    ``beta = beta0 - asin(sign * counts / K)``. ``phase_sign`` is -1 for a
    transducer mounted upside down. Returns NaN where ``|counts| > K``.
    Returns more than ``asin(10240 / K)`` (about 42 degrees for the default K)
    from the boresight alias onto the other side of the cycle; that cannot be
    detected from one sample.
    """
    s = phase_sign * counts / counts_per_unit_sine
    if not -1.0 <= s <= 1.0:
        return math.nan
    return boresight_depression_rad - math.asin(s)


def counts_from_depression(
    depression_rad: float,
    counts_per_unit_sine: float,
    boresight_depression_rad: float,
    phase_sign: int = 1,
) -> float:
    """Forward model: the phase counts a return at ``depression_rad`` produces, wrapped to one cycle."""
    counts = phase_sign * counts_per_unit_sine * math.sin(boresight_depression_rad - depression_rad)
    half = PHASE_COUNTS_PER_CYCLE / 2
    return (counts + half) % PHASE_COUNTS_PER_CYCLE - half


def parse_payload(payload: bytes, num_samples: int, config: SonarConfig) -> SampleData:
    """Parse the sample payload for one ISSS ping."""
    expected = num_samples * BYTES_PER_SAMPLE
    if len(payload) < expected:
        raise ValueError(f"Payload must be at least {expected} bytes, got {len(payload)}")

    k = config.counts_per_unit_sine
    beta0 = math.radians(config.boresight_depression_deg)
    port_counts = [_decode_phase(payload[i], payload[i + 1]) for i in range(0, expected, BYTES_PER_SAMPLE)]
    starboard_counts = [_decode_phase(payload[i + 3], payload[i + 4]) for i in range(0, expected, BYTES_PER_SAMPLE)]

    return SampleData(
        port_rss=list(payload[2:expected:BYTES_PER_SAMPLE]),
        starboard_rss=list(payload[5:expected:BYTES_PER_SAMPLE]),
        port_phase_counts=port_counts,
        starboard_phase_counts=starboard_counts,
        port_phase_diff=[phase_counts_to_radians(c) for c in port_counts],
        starboard_phase_diff=[phase_counts_to_radians(c) for c in starboard_counts],
        port_depression=[depression_from_counts(c, k, beta0, config.port_phase_sign) for c in port_counts],
        starboard_depression=[depression_from_counts(c, k, beta0, config.starboard_phase_sign)
                              for c in starboard_counts],
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
