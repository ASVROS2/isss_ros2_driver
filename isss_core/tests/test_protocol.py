"""Unit tests for ISSS protocol parsing."""

import math
import struct

import pytest

from isss.config import SonarConfig
from isss.protocol import (
    PACKET_TYPE_ISSS,
    PHASE_COUNTS_PER_CYCLE,
    SOP1,
    SOP2,
    compute_decimation,
    counts_from_depression,
    depression_from_counts,
    is_packet_sync_byte,
    packet_size,
    parse_header,
    parse_payload,
    phase_counts_to_radians,
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
        assert samples.port_phase_counts == [256]
        assert samples.starboard_phase_counts == [512]
        assert samples.port_phase_diff[0] == pytest.approx(256 * 2 * math.pi / 20480)

    def test_negative_phase_wrap(self):
        config = SonarConfig()
        payload = _make_sample(40000, 0, 0, 0)
        samples = parse_payload(payload, 1, config)
        assert samples.port_phase_counts == [40000 - 65536]

    def test_zero_counts_is_boresight(self):
        config = SonarConfig()
        samples = parse_payload(_make_sample(0, 0, 0, 0), 1, config)
        assert samples.port_depression[0] == pytest.approx(math.radians(config.boresight_depression_deg))
        assert samples.starboard_depression[0] == samples.port_depression[0]


class TestPhaseModel:
    K = 15218.0
    BETA0 = math.radians(44.5)

    def test_one_cycle_is_two_pi(self):
        assert phase_counts_to_radians(PHASE_COUNTS_PER_CYCLE) == pytest.approx(2 * math.pi)

    @pytest.mark.parametrize("depression_deg", [5.0, 20.0, 44.5, 60.0, 85.0])
    def test_round_trip_inside_unambiguous_sector(self, depression_deg):
        beta = math.radians(depression_deg)
        counts = counts_from_depression(beta, self.K, self.BETA0)
        assert depression_from_counts(counts, self.K, self.BETA0) == pytest.approx(beta)

    def test_positive_counts_point_above_boresight(self):
        assert depression_from_counts(5000, self.K, self.BETA0) < self.BETA0

    def test_counts_beyond_k_have_no_solution(self):
        assert math.isnan(depression_from_counts(self.K + 1, self.K, self.BETA0))

    def test_steep_returns_alias(self):
        """Nadir is 45.5 degrees off boresight, beyond the ~42 degree unambiguous sector."""
        counts = counts_from_depression(math.radians(90.0), self.K, self.BETA0)
        assert -10240 <= counts < 10240
        assert depression_from_counts(counts, self.K, self.BETA0) != pytest.approx(math.radians(90.0), abs=0.1)

    def test_matches_sonar3d_reference_calibration(self):
        """Values sonar3d_reference's phase.depression_from_counts gives for its calibration."""
        assert math.degrees(depression_from_counts(0, self.K, self.BETA0)) == pytest.approx(44.5)
        assert math.degrees(depression_from_counts(7609, self.K, self.BETA0)) == pytest.approx(44.5 - 30.0, abs=1e-3)
        assert math.degrees(depression_from_counts(-7609, self.K, self.BETA0)) == pytest.approx(44.5 + 30.0, abs=1e-3)


class TestPacketSync:
    def test_valid_header(self):
        header = _make_header()
        assert is_packet_sync_byte(header)

    def test_invalid_sop(self):
        header = bytearray(_make_header())
        header[0] = 0x00
        assert not is_packet_sync_byte(bytes(header))
