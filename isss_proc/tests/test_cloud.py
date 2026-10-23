"""The PointCloud2 layout of the processing node, and messages to cloud without ROS."""

import numpy as np

from isss import SonarConfig
from isss.diagnostics import decode_packets
from isss.sim import synthetic_packet
from isss.sources import pings_from_message_fields
from isss_proc import Stream, make_pipeline
from isss_proc.cloud import FLOAT32, POINT_DTYPE, POINT_FIELDS, POINT_STEP, UINT8, pack_points, point_records
from isss_proc.filters import IntensityExcess, MostProbableAngle


def result(n=20, num_samples=1000):
    packets = [synthetic_packet(ping_time=i * 268, bottom_sample=num_samples // 4, num_samples=num_samples)
               for i in range(n)]
    a = decode_packets(packets, num_samples)
    return Stream(SonarConfig()).push(a)


def test_fields_describe_the_record_layout():
    assert [f[0] for f in POINT_FIELDS] == ["x", "y", "z", "intensity", "range", "side"]
    assert POINT_STEP == 24
    sizes = {FLOAT32: 4, UINT8: 1}
    ends = [offset + sizes[datatype] for _, offset, datatype in POINT_FIELDS]
    assert all(end <= next_offset for end, (_, next_offset, _) in zip(ends, POINT_FIELDS[1:]))
    assert ends[-1] <= POINT_STEP
    assert dict((name, datatype) for name, _, datatype in POINT_FIELDS)["side"] == UINT8


def test_packed_points_read_back():
    pts = result().points()
    assert pts["y"].size > 0
    data = pack_points(pts)
    assert len(data) == POINT_STEP * pts["y"].size
    back = np.frombuffer(data, dtype=POINT_DTYPE)
    assert (back["x"] == 0).all()
    for name in ("y", "z", "intensity", "range"):
        np.testing.assert_allclose(back[name], pts[name], rtol=1e-6)
    np.testing.assert_array_equal(back["side"], pts["side"])
    assert (back["y"][back["side"] == 0] < 0).all() and (back["y"][back["side"] == 1] > 0).all()
    assert (back["z"] < 0).all()  # a seabed below the sensor


def test_no_points_pack_to_nothing():
    empty = {k: np.zeros(0) for k in ("y", "z", "intensity", "range", "side")}
    assert pack_points(empty) == b"" and point_records(empty).shape == (0,)


def test_message_fields_in_blocks_match_decoded_packets():
    """What the node does: pings from message fields, pushed a few at a time."""
    num_samples, n, block = 1000, 30, 5
    packets = [synthetic_packet(ping_time=i * 268, bottom_sample=num_samples // 4, num_samples=num_samples)
               for i in range(n)]
    decoded = decode_packets(packets, num_samples)
    from_packets, from_messages = Stream(SonarConfig()), Stream(SonarConfig())
    for start in range(0, n, block):
        part = decoded.select(slice(start, start + block))
        msgs = pings_from_message_fields(part.ping_time, part.decimation, part.intensity[..., 0],
                                         part.intensity[..., 1], part.counts[..., 0], part.counts[..., 1])
        assert pack_points(from_messages.push(msgs).points()) == pack_points(from_packets.push(part).points())


def test_make_pipeline_options():
    assert make_pipeline().names == ["range", "intensity", "bias_states", "first_surface"]
    p = make_pipeline(min_excess=0, bias_states=False, first_surface=False, mpaa=True, mpaa_pings=5,
                      mpaa_band=2.0, max_range=15.0)
    assert p.names == ["range", "mpaa"]
    assert p.stages[0].max_m == 15.0
    assert isinstance(p.stages[1], MostProbableAngle) and p.stages[1].pings == 5 and p.stages[1].band_deg == 2.0
    assert isinstance(make_pipeline(min_excess=40).stages[1], IntensityExcess)
