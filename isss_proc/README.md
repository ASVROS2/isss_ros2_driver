# isss_proc

Processing of **DeepVision ISSS** pings, with no ROS dependency: filter
stages that decide which returns to trust, run over a whole recording or over
live pings as they arrive. It reads its data through
[`isss_core`](../isss_core), so the same code takes the live driver, `.bin`
logs, exported `.nc` files and bags of the old C++ driver. Where this is
going, and how a ROS node and older data fit in: [../docs/processing.md](../docs/processing.md).

## Install

```bash
cd isss_proc
uv sync            # also installs ../isss_core (editable), numpy, scipy, matplotlib
uv run pytest
```

Under colcon it is an `ament_python` package next to `isss_core`.

## Command line

```bash
uv run isss-proc-view ../isss_data/1755610748.553636.bin        # isss-view, filtered
uv run isss-proc-view rosbag2_2025_12_14-12_18_12.nc --mpaa     # add MPAA to the chain
uv run isss-proc-view --ip 192.168.2.62                          # live
uv run isss-proc rosbag2_2025_12_14-12_18_12.nc -o cloud.npz    # what each stage drops; save the cloud
uv run isss-proc rosbag2_2025_12_14-12_18_12.nc --labels labels.npz   # per-sample labels
uv run isss-proc-crosstalk rosbag2_2025_12_14-12_18_12.nc -o xt.png   # cross talk (parked, see below)
```

`isss-proc-view` is `isss-view` with the filter chain plugged in: the bearing
panel highlights the kept samples of the selected ping, and the profile shows
the kept returns of the last `--trail` pings in the sensor frame, the selected
one bold and the strong returns the chain drops in grey. Each ping is
processed with the `--context` pings before it, as it would be live. The
calibration options (`--k`, `--boresight`, `--flip-port`, ...) and keys are
those of `isss-view`; the filter options are listed by `--help`.

`isss-proc` prints, per side, the share of samples with no depression
solution, the share each stage drops, and the share kept. `-o` saves the kept
returns as `.npz`: `y`, `z` (sensor frame, port negative, up), `intensity`,
`side`, `ping`, `sample`, `range`, `depression`. `--labels` saves per-sample
labels for the whole recording, for training: `rejected_by` (ping, sample,
side; 0 kept, k the k-th entry of `stages`, 255 no solution), `stages`,
`crosstalk_score` (see `CrossTalk`), `ping_time` and `slant_range`.

`isss-proc-crosstalk` measures the leak between the sides in each side's water
column, prints the curve and the share of samples flagged by range, and draws
three things:
- the waterfalls with the flags, and port cleaned;
- the measured and the used leak curves;
- the phase of the flagged samples against port noise.

`--clean-crosstalk` on `isss-proc-view` shows the intensity with the flagged
samples set to the ping's water-column level. With today's curve it removes
port's nadir line, so leave it off (see `CrossTalk` below).

## Python

```python
from isss import SonarConfig
from isss.sources import load_pings
from isss_proc import Pipeline, Swath, MostProbableAngle, default_stages

config = SonarConfig()                                   # the calibration
arrays = load_pings("rosbag2_2025_12_14-12_18_12.nc")    # .bin, .nc or old-driver bag
result = Pipeline().run(Swath(arrays.select(slice(0, 500)), config))
result.summary()          # per side: samples dropped by each stage, kept
result.points()           # kept returns in the sensor frame
result.rejected_by        # (ping, sample, side): 0 kept, k = stage k, 255 no solution
result.info["bias_states"]

Pipeline([*default_stages(), MostProbableAngle(pings=5)])   # a custom chain

from isss_proc import CrossTalk, clean_intensity, estimate_leak
swath = Swath(arrays.select(slice(0, 500)), config)
flags, score = CrossTalk().evaluate(swath)   # parked: today's curve flags port's nadir
clean = clean_intensity(swath, flags)        # PingArrays, flagged samples at the water column
estimate_leak(swath)["port"]["table"]        # the leak curve into port, measured
```

Live, with the driver (`ping.raw` is the packet):

```python
from isss import SonarDriver
from isss_proc import Stream

stream = Stream(config, context=200)
driver = SonarDriver(config, lambda ping: handle(stream.push_packet(ping.raw).points()))
```

## Stages

A stage is any callable `stage(swath, keep, info) -> mask`: it gets the
samples still kept and returns its own verdict as a `(ping, sample, side)`
bool array. Stages keep no state between calls; one that needs neighbouring
pings looks at the swath it is given, which live is the rolling window.

| Stage | Drops | Defaults | From |
|---|---|---|---|
| `RangeGate` | Bins nearer than `min_m` (ring-down) or beyond `max_m` | 1.0 m, none | sonar3d_reference mesh check |
| `IntensityExcess` | Returns less than `min_excess` above the ping's water column (median of the 5-bin-smoothed intensity between 1.0 and 2.5 m) | 30 | sonar3d_reference mesh check |
| `BiasStates` | Phase values that noise collapses to, if they also stand out among returns; counts within 150 of ±10240 | up to 3 states, return ratio ≥ 1.3 | sonar3d_reference mesh check |
| `FirstSurface` | Returns more than `margin_m` (plus the range a flat seabed spans within the angle bin) behind the first cluster of returns at the same angle (multipath, wrapped phase) | 0.5 m, 2° bins, 3 returns within 0.3 m | sonar3d_reference mesh check; the bin extent is new |
| `CrossTalk` (parked) | Samples whose excess lies within `margin` of the leak expected from the other side's intensity at the same bin | a curve into port that is really port's nadir (see below), none into starboard; 20, 20 | new; obscures the nadir, do not use yet |
| `MostProbableAngle` | Returns more than `band_deg` from the most probable depression of their range bin, and bins too sparse or spread | 0.5 m bins, 1 ping, 3°, spread ≤ 2° | `mpaa.py`, 2024 scripts |

`default_stages()` is the first four, the chain measured against the
Beckholmen dock mesh. There, at an excess of 30, it kept about half of the
starboard returns with 67–69 % of them within 0.25 m of the mesh (results and
reasoning in the sonar3d_reference ISSS notes). Three things are new and not
yet measured against the mesh:

- `FirstSurface` widens its margin by the range a flat seabed spans within one
  angle bin. Without that, a seabed at shallow grazing angles comes out
  striped. On `12_18_12` it keeps 6 % more starboard returns, and the share
  below the floor (the multipath ghost) stays at 6.4 % against 6.7 %.
  `FirstSurface(bin_extent=False)` is the original rule.
- `CrossTalk` is **parked, with an open issue: it obscures the nadir.** Its
  default curve into port is really port's own first bottom return, which
  reads about 29 counts below starboard's at the same range. So it flags
  port's nadir line, and `--clean-crosstalk` wipes it out. No same-bin cross
  talk is confirmed in the dock bags. Leave it off until a real leak has been
  measured away from the nadir. What is done and what is open:
  [../docs/processing.md](../docs/processing.md#parked-cross-talk-between-the-sides-oct-2026).
- MPAA, at its defaults, removes the ghost but also much of the floor beyond
  7–8 m and the walls. It also drops returns close to nadir, where the angle
  changes across a range bin.

The class docstrings in `filters.py` give the details, and what changed from
the original scripts.

## Layout

```
src/isss_proc/
  swath.py     Swath: pings + calibration; slant range and depression
  filters.py   the stages
  pipeline.py  Pipeline, Result, Stream (live), viewer_keep (isss-proc-view)
  cli.py       isss-proc, isss-proc-view
tests/
```

## Tests

`uv run pytest`, on synthetic pings from `isss.sim`: each stage on a case it
must catch and one it must leave alone, batch and streaming agree, the
viewer and the commands run headless.
