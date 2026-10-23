"""The pipeline's bookkeeping, and live processing agreeing with batch."""

import numpy as np

from isss import SonarConfig
from isss.diagnostics import decode_packets
from isss.sim import synthetic_packet
from isss_proc import MostProbableAngle, Pipeline, Stream, Swath, default_stages, viewer_keep
from isss_proc.pipeline import NO_SOLUTION


def packets(n, num_samples=1000, decimation=111):
    return [synthetic_packet(ping_time=i * 268, bottom_sample=num_samples // 4, num_samples=num_samples,
                             decimation=decimation) for i in range(n)]


def arrays(n=30, **kw):
    return decode_packets(packets(n, **kw), kw.get("num_samples", 1000))


def test_rejected_by_and_summary_account_for_every_sample():
    a = arrays(10)
    a.counts[:, 500, 0] = 32000  # beyond K: no depression
    result = Pipeline().run(Swath(a, SonarConfig()))
    assert (result.keep == (result.rejected_by == 0)).all()
    assert (result.rejected_by[:, 500, 0] == NO_SOLUTION).all()
    for row in result.summary().values():
        assert row["samples"] == sum(v for k, v in row.items() if k != "samples")
    assert result.summary()["port"]["no_solution"] == 10


def test_points_match_the_mask():
    result = Pipeline().run(Swath(arrays(5), SonarConfig()))
    pts = result.points()
    assert pts["y"].size == result.keep.sum()
    assert (pts["y"][pts["side"] == 0] < 0).all() and (pts["y"][pts["side"] == 1] > 0).all()
    assert np.allclose(np.hypot(pts["y"], pts["z"]), pts["range"], atol=1e-6)
    # A flat seabed: every kept return at the same height, except at the bottom onset, where
    # the smoothed intensity reaches two water-column bins and the nadir return aliases.
    seabed = pts["sample"] > 250
    assert np.ptp(pts["z"][seabed]) < 0.05 and (~seabed).sum() <= 3 * 2 * 5


def test_stream_agrees_with_batch_over_the_same_window():
    a = arrays(40)
    pipeline = Pipeline([*default_stages(), MostProbableAngle()])
    stream = Stream(SonarConfig(), pipeline, context=15)
    for i in range(a.n_pings):
        live = stream.push(a.select(slice(i, i + 1)))
    batch = pipeline.run(Swath(a.select(slice(a.n_pings - 16, None)), SonarConfig()))
    assert live.keep.shape[0] == 1
    assert (live.keep[0] == batch.keep[-1]).all()
    assert stream.window.n_pings == 16


def test_stream_takes_blocks_and_packets():
    stream = Stream(SonarConfig(), context=10)
    a = arrays(12)
    assert stream.push(a.select(slice(0, 8))).keep.shape[0] == 8
    assert stream.push_packet(packets(1)[0]).keep.shape[0] == 1
    assert stream.window.n_pings == 9


def test_stream_restarts_when_the_configuration_changes():
    stream = Stream(SonarConfig(), context=50)
    stream.push(arrays(5))
    stream.push(arrays(3, num_samples=500))
    assert stream.window.n_pings == 3 and stream.window.num_samples == 500
    stream.push(arrays(2, num_samples=500, decimation=222))
    assert stream.window.n_pings == 2


def test_viewer_keep_processes_with_context():
    a = arrays(30)
    keep = viewer_keep(context=10)
    mask = keep(a, SonarConfig(), slice(25, 28))
    assert mask.shape == (3, 1000, 2)
    batch = Pipeline().run(Swath(a.select(slice(15, 28)), SonarConfig()))
    assert (mask == batch.keep[-3:]).all()
