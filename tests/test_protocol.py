"""Unit tests for ISSS protocol parsing."""

import math
import struct

import pytest

from isss.config import SonarConfig
from isss.protocol import (
    PACKET_TYPE_ISSS,
    SOP1,
    SOP2,
    compute_decimation,
    is_packet_sync_byte,
    packet_size,
    parse_header,
    parse_payload,
)


def _make_header(
    *,
    ping_time: int = 123456,
    decimation: int = 42,
) -> bytes:
    header = bytearray(12)
    header[0] = SOP1
    header[1] = SOP2
    struct.pack_into(">H", header, 2, packet_size(1000))
    header[4] = PACKET_TYPE_ISSS
    struct.pack_into(">I", header, 5, ping_time)
    header[9] = 0x01
    struct.pack_into(">H", header, 10, decimation)
    return bytes(header)


def _make_sample(
    port_phase: int,
    port_rss: int,
    starboard_phase: int,
    starboard_rss: int,
) -> bytes:
    return bytes([
        (port_phase >> 8) & 0xFF,
        port_phase & 0xFF,
        port_rss & 0xFF,
        (starboard_phase >> 8) & 0xFF,
        starboard_phase & 0xFF,
        starboard_rss & 0xFF,
    ])


class TestDecimation:
    def test_default_range(self):
        assert compute_decimation(20.0, 1000) == 111


class TestParseHeader:
    def test_round_trip(self):
        header = _make_header(ping_time=1000, decimation=99)
        parsed = parse_header(header)
        assert parsed.is_valid
        assert parsed.packet_type == PACKET_TYPE_ISSS
        assert parsed.ping_time == 1000
        assert parsed.decimation == 99
        assert parsed.ping_time_sec == pytest.approx(0.1)

    def test_too_short(self):
        with pytest.raises(ValueError):
            parse_header(b"\x00" * 8)


class TestParsePayload:
    def test_single_sample(self):
        config = SonarConfig()
        payload = _make_sample(256, 128, 512, 64)
        samples = parse_payload(payload, 1, config)

        assert samples.port_rss == [128]
        assert samples.starboard_rss == [64]
        assert samples.port_phase_diff[0] == pytest.approx((256 / 512.0) * (math.pi / 180.0))

    def test_negative_phase_wrap(self):
        config = SonarConfig()
        payload = _make_sample(40000, 0, 0, 0)
        samples = parse_payload(payload, 1, config)
        assert samples.port_phase_diff[0] == pytest.approx((-25536 / 512.0) * (math.pi / 180.0))


class TestPacketSync:
    def test_valid_header(self):
        header = _make_header()
        assert is_packet_sync_byte(header)

    def test_invalid_sop(self):
        header = bytearray(_make_header())
        header[0] = 0x00
        assert not is_packet_sync_byte(bytes(header))
