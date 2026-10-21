"""Diagnostics: the vectorised decoder agrees with isss.protocol, and the checks catch what they should."""

import math
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

from isss import SonarConfig  # noqa: E402
from isss.diagnostics import (  # noqa: E402
    decode_packets,
    health,
    load_log,
    sensor_frame_points,
)
from isss.log import find_packet_offsets, iter_packets_from_log  # noqa: E402
from isss.protocol import HEADER_SIZE, parse_header  # noqa: E402
from isss.sim import synthetic_packet  # noqa: E402

LOG = Path(__file__).parent / "data" / "stick_2025-08-19_5pings.bin"


def _checks(report):
    return {c.name: c for c in report.checks}


def _synthetic(n=60, bottom=300, step=268):
    return [synthetic_packet(ping_time=i * step, bottom_sample=bottom) for i in range(n)]


def test_vectorised_decoder_matches_reference():
    config = SonarConfig()
    arrays = load_log(LOG)
    reference = list(iter_packets_from_log(LOG, config=config))
    assert arrays.n_pings == len(reference) == 5
    depression = arrays.depression(config)
    for i, (samples, packet) in enumerate(reference):
        assert arrays.ping_time[i] == parse_header(packet[:HEADER_SIZE]).ping_time
        assert arrays.intensity[i, :, 0].tolist() == samples.port_rss
        assert arrays.intensity[i, :, 1].tolist() == samples.starboard_rss
        assert arrays.counts[i, :, 0].tolist() == samples.port_phase_counts
        assert arrays.counts[i, :, 1].tolist() == samples.starboard_phase_counts
        np.testing.assert_allclose(depression[i, :, 1], samples.starboard_depression, rtol=0, atol=1e-12)


def test_framing_skips_sync_patterns_inside_payloads():
    packet = bytearray(synthetic_packet())
    packet[100:112] = packet[:12]  # a header-looking pattern inside the payload
    data = b"\x00\x01" + bytes(packet) * 3
    assert find_packet_offsets(data) == [2, 2 + 6012, 2 + 2 * 6012]


def test_synthetic_log_is_healthy():
    report = health(decode_packets(_synthetic()), SonarConfig())
    assert report.worst == "ok", [c for c in report.checks if c.status == "warn"]


def test_out_of_water_log_is_flagged():
    checks = _checks(health(load_log(LOG), SonarConfig()))
    assert checks["port echoes"].status == "warn"
    assert checks["starboard echoes"].status == "warn"
    assert checks["phase encoding"].status == "ok"


def test_shifted_phase_encoding_is_flagged():
    arrays = decode_packets(_synthetic())
    arrays.counts[:] = (arrays.counts.astype(np.int32) % 20480).astype(np.int16)  # the July 2024 look
    assert _checks(health(arrays, SonarConfig()))["phase encoding"].status == "warn"


def test_gaps_and_wrong_pacing_are_flagged():
    packets = _synthetic(n=60)
    del packets[20:25]
    checks = _checks(health(decode_packets(packets), SonarConfig()))
    assert checks["gaps"].status == "warn"
    assert "about 5 pings missing" in checks["gaps"].detail
    slow = _checks(health(decode_packets(_synthetic(step=600)), SonarConfig()))
    assert slow["ping interval"].status == "warn"


def test_flat_bottom_is_flat_in_the_sensor_frame():
    config = SonarConfig()
    bottom = 300
    arrays = decode_packets(_synthetic(n=3, bottom=bottom))
    pts = sensor_frame_points(arrays, config)
    altitude = arrays.slant_range(config)[bottom]
    # Select by the true geometry: over a flat bottom, slant range fixes the angle.
    # (Aliased returns decode to angles inside the sector too; that is what aliasing is.)
    true_beta = np.arcsin(np.clip(altitude / np.hypot(pts["y"], pts["z"]), -1, 1))
    unaliased = np.abs(math.radians(config.boresight_depression_deg) - true_beta) < math.asin(10240 / 15218) - 0.01
    assert unaliased.mean() > 0.9
    np.testing.assert_allclose(pts["z"][unaliased], -altitude, atol=0.02)
    assert (pts["y"][pts["side"] == 0] < 0).all() and (pts["y"][pts["side"] == 1] > 0).all()
