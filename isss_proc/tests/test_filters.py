"""Each stage on a case it must catch and one it must leave alone, on synthetic pings."""

import numpy as np
import pytest

from isss import SonarConfig
from isss.diagnostics import decode_packets
from isss.sim import synthetic_packet
from isss_proc import (
    BiasStates,
    CrossTalk,
    FirstSurface,
    IntensityExcess,
    MostProbableAngle,
    Pipeline,
    RangeGate,
    Stream,
    Swath,
    clean_intensity,
    estimate_leak,
)

BOTTOM = 250  # first bottom return, in bins (4.8 m at the default 20 m setting)


def flat(n=20) -> Swath:
    """Pings over a flat seabed: water column, then returns that follow the phase model exactly."""
    return Swath(decode_packets([synthetic_packet(ping_time=i * 268, bottom_sample=BOTTOM) for i in range(n)]),
                 SonarConfig())


def run(swath, *stages):
    return Pipeline(list(stages)).run(swath)


def test_range_gate():
    s = flat(2)
    keep = run(s, RangeGate(min_m=1.0, max_m=10.0)).keep
    r = s.slant_range
    assert not keep[:, r < 1.0].any() and not keep[:, r > 10.0].any()
    assert keep[:, (r > 5.0) & (r < 10.0)].all()


def test_intensity_drops_the_water_column_and_keeps_the_seabed():
    s = flat(3)
    keep = run(s, IntensityExcess()).keep
    assert not keep[:, : BOTTOM - 2].any()  # the 5-bin smoothing lets the onset reach 2 bins early
    assert keep[:, BOTTOM : BOTTOM + 200].all()


def test_bias_states_drop_a_state_that_contaminates_returns_and_spare_one_that_does_not():
    s = flat(20)
    rng = np.random.default_rng(0)
    counts = s.arrays.counts
    wc = slice(60, 130)  # 1.1-2.5 m: water column
    shape = counts[:, wc].shape
    # The water column collapses onto two states, at -6000 and +3000 counts.
    counts[:, wc] = np.where(rng.random(shape) < 0.5, -6000, 3000) + rng.integers(-40, 40, shape)
    # A fifth of the seabed returns are stuck at -6000: that state contaminates them.
    stuck = (rng.random(counts.shape[:2]) < 0.2)[:, :, None] & (np.arange(counts.shape[1]) > BOTTOM)[None, :, None]
    stuck = np.broadcast_to(stuck, counts.shape)
    counts[:] = np.where(stuck, -6000, counts)
    s = Swath(s.arrays, s.config)  # depression from the edited counts

    result = run(s, IntensityExcess(), BiasStates())
    states = {round(st["center_counts"], -2): st for st in result.info["bias_states"]["starboard"]}
    assert states[-6000]["rejected"] and not states[3000]["rejected"]
    assert not result.keep[stuck].any()
    near_spared = (np.abs(counts - 3000) < 100) & ~stuck
    assert near_spared[:, BOTTOM:].any() and not (result.rejected_by[near_spared] == 2).any()


def test_bias_states_drop_the_wrap_edge():
    s = flat(2)
    s.arrays.counts[:, 600, :] = 10200
    s = Swath(s.arrays, s.config)
    assert not run(s, BiasStates()).keep[:, 600].any()


def ghost(s: Swath, start=300, stop=340) -> Swath:
    """Multipath: bins 2*start..2*stop repeat the phase of start..stop, at twice the range."""
    c = s.arrays.counts
    for j in range(2 * start, 2 * stop):
        c[:, j] = c[:, j // 2]
    return Swath(s.arrays, s.config)


def test_first_surface_drops_a_multipath_ghost():
    s = ghost(flat(3))
    result = run(s, IntensityExcess(), FirstSurface())
    assert not result.keep[:, 600:680].any()
    assert result.keep[:, 300:340].all()


@pytest.mark.parametrize("bin_extent", [True, False])
def test_first_surface_on_a_flat_seabed(bin_extent):
    """With the bin extent the seabed is untouched; without it (sonar3d_reference) its far part is striped."""
    result = run(flat(3), IntensityExcess(), FirstSurface(bin_extent=bin_extent))
    dropped = (result.rejected_by == 2).sum()
    assert (dropped == 0) if bin_extent else (dropped > 0)


def test_mpaa_keeps_a_consistent_bin_and_drops_a_scattered_one():
    s = flat(4)
    rng = np.random.default_rng(1)
    c = s.arrays.counts
    c[:, 700:760, 0] = rng.integers(-10000, 10000, c[:, 700:760, 0].shape)  # port: no consistent angle
    c[:, 400, 1] = 9000  # starboard: one sample far off its neighbours
    s = Swath(s.arrays, s.config)
    result = run(s, IntensityExcess(), MostProbableAngle())
    assert not result.keep[:, 702:756, 0].any()
    assert result.keep[:, 702:756, 1].all()
    assert not result.keep[:, 400, 1].any() and result.keep[:, 380:399, 1].all()


def test_mpaa_drops_steep_returns_where_the_angle_changes_within_a_bin():
    """Near nadir a flat seabed's depression changes by more than ``max_spread_deg`` across one range bin."""
    keep = run(flat(2), IntensityExcess(), MostProbableAngle()).keep
    assert not keep[:, BOTTOM : BOTTOM + 40].any()
    assert keep[:, BOTTOM + 100 : BOTTOM + 300].all()
    finer = run(flat(2), IntensityExcess(), MostProbableAngle(bin_m=0.15)).keep
    assert finer[:, BOTTOM + 30 : BOTTOM + 40].all()


def test_mpaa_pools_pings():
    s = flat(6)
    a = run(s, IntensityExcess(), MostProbableAngle(pings=1)).keep
    b = run(s, IntensityExcess(), MostProbableAngle(pings=3)).keep
    assert (a == b)[:, BOTTOM + 100 : BOTTOM + 300].all() and b[:, BOTTOM + 100 : BOTTOM + 300].all()


LEAK = slice(600, 640)  # starboard target bins
FLOOR = 5  # water-column intensity of the synthetic pings


def leaking(port_excess, starboard=220, n=3) -> Swath:
    """A strong starboard target over the seabed, with port at ``port_excess`` above the water column there."""
    s = flat(n)
    s.arrays.intensity[:, LEAK, 1] = starboard
    s.arrays.intensity[:, LEAK, 0] = FLOOR + port_excess
    return s


def expected_leak(starboard=220):
    x, y = np.asarray(CrossTalk.DEFAULT_LEAK[0]).T
    return float(np.interp(starboard, x, y))


def test_crosstalk_flags_a_leak_on_port_and_leaves_starboard():
    s = leaking(expected_leak())
    flags, score = CrossTalk().evaluate(s)
    inner = slice(LEAK.start + 3, LEAK.stop - 3)  # clear of the 5-bin smoothing at the edges
    assert flags[:, inner, 0].all() and not flags[:, :, 1].any()
    assert np.allclose(score[:, inner, 0], 0.0, atol=0.5)
    assert np.isnan(score[:, 300:500]).all()  # the seabed alone leaks too little to judge
    keep = run(s, IntensityExcess(), CrossTalk()).keep
    assert not keep[:, inner, 0].any() and keep[:, inner, 1].all()


def test_crosstalk_spares_returns_above_the_leak_and_samples_without_it():
    assert not CrossTalk().flags(leaking(expected_leak() + 40))[:, LEAK, 0].any()
    assert not CrossTalk().flags(leaking(220 - FLOOR))[:, LEAK].any()  # equal on both sides
    assert not CrossTalk().flags(leaking(5))[:, LEAK, 0].any()  # starboard strong, but no leak on port


def test_crosstalk_ignores_moderate_sources_and_can_be_turned_off():
    assert not CrossTalk().flags(leaking(10, starboard=170)).any()
    assert not CrossTalk(leak=(None, None)).flags(leaking(expected_leak())).any()


def test_estimate_leak_recovers_the_injected_curve():
    rng = np.random.default_rng(2)
    s = Swath(decode_packets([synthetic_packet(ping_time=i * 268, bottom_sample=900) for i in range(400)]),
              SonarConfig())
    level = rng.uniform(150, 240, s.n_pings)
    leaks = rng.random(s.n_pings) < 0.6  # all or nothing, per ping
    s.arrays.intensity[:, 200:700, 1] = level[:, None].astype(np.uint8)
    s.arrays.intensity[:, 200:700, 0] = (FLOOR + leaks * np.maximum(0, level - 150))[:, None].astype(np.uint8)
    est = estimate_leak(s, bottom_excess=200)  # only the seabed at bin 900 counts as the bottom
    curve = dict(est["port"]["curve"])
    assert abs(curve[205.0] - 55) <= 3 and 165.0 not in curve  # a 15-count leak is too small to tell
    assert CrossTalk(estimate=True).curves(s)[0] == estimate_leak(s)["port"]["curve"]


def test_clean_intensity_sets_only_the_flagged_samples_to_the_water_column():
    s = leaking(expected_leak())
    flags = CrossTalk().flags(s)
    clean = clean_intensity(s, flags).intensity
    assert (clean[flags] == FLOOR).all()
    assert (clean[~flags] == s.intensity[~flags]).all()


def test_crosstalk_streams_like_batch():
    s = leaking(expected_leak(), n=12)
    pipeline = Pipeline([IntensityExcess(), CrossTalk()])
    stream = Stream(SonarConfig(), pipeline, context=4)
    for i in range(s.n_pings):
        live = stream.push(s.arrays.select(slice(i, i + 1)))
    assert (live.keep[0] == pipeline.run(s).keep[-1]).all()
