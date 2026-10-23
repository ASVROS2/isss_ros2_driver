"""Pings from the fields of isss_msgs/ISSS messages agree with the packets they came from."""

import math
from array import array

import numpy as np
import pytest

from isss.config import SonarConfig
from isss.diagnostics import decode_packets
from isss.protocol import parse_packet
from isss.sim import synthetic_packet
from isss.sources import config_from_message_fields, pings_from_message_fields

NUM_SAMPLES = 400


def packets(n=4, phase_signs=(1, 1)):
    return [synthetic_packet(num_samples=NUM_SAMPLES, ping_time=1000 + 268 * i, decimation=111 + i,
                             bottom_sample=80 + 10 * i, phase_signs=phase_signs) for i in range(n)]


def message_fields(packet):
    """What the driver node puts in an isss_msgs/ISSS message, with rclpy's array types."""
    ping = parse_packet(packet, SonarConfig(num_samples=NUM_SAMPLES))
    s = ping.samples
    return {
        "ping_time": ping.header.ping_time,
        "decimation": ping.header.decimation,
        "port_intensity": array("B", s.port_rss),
        "starboard_intensity": array("B", s.starboard_rss),
        "port_phase_counts": array("h", s.port_phase_counts),
        "starboard_phase_counts": array("h", s.starboard_phase_counts),
    }


def assert_same_pings(got, expected):
    for name in ("ping_time", "decimation", "intensity", "counts"):
        np.testing.assert_array_equal(getattr(got, name), getattr(expected, name), err_msg=name)
        assert getattr(got, name).dtype == getattr(expected, name).dtype, name
    assert (got.size_field == 0).all()  # not in the message


@pytest.mark.parametrize("phase_signs", [(1, 1), (-1, 1)])
def test_one_message_matches_the_decoded_packet(phase_signs):
    packet = packets(1, phase_signs)[0]
    assert_same_pings(pings_from_message_fields(**message_fields(packet)), decode_packets([packet], NUM_SAMPLES))


def test_several_messages_stack_in_order():
    ps = packets(4)
    fields = [message_fields(p) for p in ps]
    stacked = pings_from_message_fields(**{k: [f[k] for f in fields] for k in fields[0]})
    assert stacked.n_pings == 4 and stacked.num_samples == NUM_SAMPLES
    assert_same_pings(stacked, decode_packets(ps, NUM_SAMPLES))


def test_numpy_arrays_and_bytes_are_accepted():
    ps = packets(3)
    expected = decode_packets(ps, NUM_SAMPLES)
    from_numpy = pings_from_message_fields(
        expected.ping_time, expected.decimation,
        expected.intensity[..., 0], expected.intensity[..., 1],
        expected.counts[..., 0], expected.counts[..., 1])
    assert_same_pings(from_numpy, expected)
    fields = message_fields(ps[0])
    fields["port_intensity"] = bytes(fields["port_intensity"])
    fields["starboard_intensity"] = bytes(fields["starboard_intensity"])
    assert_same_pings(pings_from_message_fields(**fields), expected.select(slice(0, 1)))


def test_the_depression_agrees_with_the_driver():
    packet = packets(1)[0]
    config = SonarConfig(num_samples=NUM_SAMPLES)
    depression = pings_from_message_fields(**message_fields(packet)).depression(config)
    s = parse_packet(packet, config).samples
    np.testing.assert_allclose(depression[0, :, 0], s.port_depression, atol=1e-9, equal_nan=True)
    np.testing.assert_allclose(depression[0, :, 1], s.starboard_depression, atol=1e-9, equal_nan=True)


def test_mismatched_sample_counts_are_rejected():
    fields = message_fields(packets(1)[0])
    fields["starboard_phase_counts"] = fields["starboard_phase_counts"][:-1]
    with pytest.raises(ValueError):
        pings_from_message_fields(**fields)


def test_config_from_message_fields():
    config = config_from_message_fields(
        range=30.0, num_samples=1000, sound_velocity=1480.0, counts_per_unit_sine=15000.0,
        boresight_depression=math.radians(40.0), port_phase_sign=-1, starboard_phase_sign=1)
    assert config.range_m == 30.0 and config.num_samples == 1000 and config.sound_velocity == 1480.0
    assert config.counts_per_unit_sine == 15000.0
    assert config.boresight_depression_deg == pytest.approx(40.0)
    assert (config.port_phase_sign, config.starboard_phase_sign) == (-1, 1)
