# Processing: structure and plan

How processing of ISSS data is organised in this repository, and what comes
next. What is built is described in [`isss_proc/README.md`](../isss_proc/README.md);
the protocol and phase model are in [protocol.md](protocol.md).

## The idea: one data model, three ways in, two ways out

```
 sources (isss_core)                 processing (isss_proc)                  outputs
 ─────────────────────               ─────────────────────────               ───────────────────────────
 live driver  ─ ping.raw ─┐
 .bin log + sidecar ──────┤                                                  ┌─ Python: Result.points(), .npz
 .nc export ──────────────┼─► PingArrays + SonarConfig ─► Swath ─► Pipeline ─┤
 old-driver bag ──────────┤      (isss.sources.load_pings)        (stages)   ├─ isss-proc-view (sensor frame)
 older formats (planned) ─┘                                                  └─ ROS node: PointCloud2 (planned)
```

- **Everything becomes `PingArrays` first.** Intensity and signed phase counts per
  (ping, sample, side), with the ping counter and decimation. Readers live in
  `isss_core` (`isss.sources`), so processing never sees a file format. Supporting an
  old recording means writing a reader into `PingArrays`, or a converter to
  the NetCDF format ([netcdf-format.md](netcdf-format.md)), not changing a filter.
- **Calibration travels with the data, not in it.** Depression is derived from
  the counts through `SonarConfig` (K, β0, phase signs, sound speed) every time.
  Old recordings are reprocessed with today's calibration, and recalibrating
  never needs a re-export. This is why stages work on counts and depressions,
  never on the angles old drivers stored, which were wrong (protocol.md).
- **A stage is a pure function of a swath.** `stage(swath, keep, info) -> mask`.
  It holds no state between calls. "Online" is the same call on a rolling
  window (`Stream`, `viewer_keep`), so a stage written and tested on a log
  runs unchanged on a live sonar or in a ROS node. A stage that needs history
  (bias states, MPAA pooling) gets it from the window.
- **The output is a mask plus a reason.** `Result.keep` and `Result.rejected_by`
  say, per sample, whether it was kept and which stage dropped it. Clouds,
  statistics and figures are all derived from that, and any stage can be
  scored on its own.

## Where things go

| Package | Holds | Depends on |
|---|---|---|
| `isss_core` (`isss`) | Protocol, driver, logs, readers and converters of every format, diagnostics, the viewer | stdlib; numpy/matplotlib/h5netcdf optional |
| `isss_proc` | Stages, pipeline, streaming, processing CLIs; later georeferencing and gridding | `isss_core`, numpy, scipy |
| `isss_ros2_driver` | ROS wrappers: the driver node now, the processing node next | `isss_core`, `isss_proc`, rclpy |
| `isss_msgs` | Messages | |

The viewer stays in `isss_core` and takes a `keep` callable, so it has no
dependency on `isss_proc`; `isss-proc-view` plugs the pipeline in.

## Older data

| Data | Recorded by | Status |
|---|---|---|
| `.bin` logs with sidecar (2026 onwards) | this driver | read |
| Aug 2025 stick `.bin` logs, no sidecar | earlier logger | read |
| Dec 2025 Beckholmen bags, `smarc_msgs/ISSS` | old C++ driver, ROS 2 | read (`isss.bag`): the packet header and exact phase word survive in the message |
| Bags of this driver, `isss_msgs/ISSS` | ROS 2 node | **to do**: a reader from the message fields (counts and header values are all there) |
| Askö Oct 2024 ROS 1 bags (`port_channel_phase_15_8` / `_7_0` byte fields) | older driver | **to do**: needs a ROS 1 reader (`rosbags`). The messages carry no decimation, so the range must be given. The MPAA scripts were tuned on these, with DVL altitude |
| July 2024 logs, phase outside ±10240 | older driver | **to do**: the encoding differs; understand it before converting (`isss-report` flags it) |
| `.npy` dumps of the 2024 scripts (`*_raw_isss.npy`) | `py_files/parse_ros_bag.py` | Prefer the bags. The dumps can be read if needed: their signed conversion subtracted 65535, not 65536, which is recoverable |

Each reader produces `PingArrays` (and per-ping UTC time, see below) and is
tested against a few pings decoded by hand, as `test_netcdf.py` does for bags.

## Next steps

1. **Measure the stages against the dock mesh.** The default chain's numbers
   come from sonar3d_reference. Two things here are new and unmeasured: the
   bin-extent margin of `FirstSurface` (it keeps a flat seabed whole, and on
   `12_18_12` it keeps 6 % more starboard returns without more below the floor)
   and MPAA. MPAA removes the multipath ghost but also much of the floor beyond
   7–8 m and the walls at its defaults. The mesh check can score a `keep` mask directly.
2. **Time per ping.** `PingArrays` carries the device counter only; readers
   already know UTC (`netcdf.read_netcdf` drops it). Add `time` to `PingArrays`
   so that processing can attach navigation.
3. **ROS processing node** in `isss_ros2_driver`: subscribe to `isss/raw`,
   convert `isss_msgs/ISSS` to `PingArrays` (the same function as the bag reader
   above), push to a `Stream` every few pings, publish `sensor_msgs/PointCloud2`
   in the message's `frame_id` (x along-track = 0, y, z, intensity, side,
   range). TF and RViz then place it in the world. The default chain plus MPAA takes about 50 ms for a
   200-ping window on a laptop, so process in blocks of a few pings, not per ping.
4. **Georeferencing** in `isss_proc`: pose per ping from navigation (interpolated
   on UTC), lever arm and mount from TF or a config, sensor frame to world.
   sonar3d_reference has the pose handling for the Beckholmen bags; the 2024
   scripts show the pitfalls (roll applied in the angle, heading conventions,
   meridian convergence).
5. **More stages, as needed:** nadir and aliasing handling (returns beyond
   β0 ± 42° wrap; altitude from the first bottom return could gate them), sound
   speed per sample from an SVP (`raytrace.py` in the 2024 scripts), a roll/heave
   artefact correction (Crawford's method, ported in `roll_heave_artifacts_removal.py`),
   gridding to a DEM (the scripts used CUBE). Each one is a stage or a step after
   `Result`, tested on synthetic pings first.
   Related, still to do:
   - a stage for the single-ping streaks across all ranges on starboard,
     probably another acoustic sensor;
   - a saturation flag for starboard returns at RSS 200 or more, whose phase
     sticks near a quarter cycle.
6. **A quality flag in the export.** Write `rejected_by` into the `.nc` as a
   CF flag variable, so processed and raw data stay in one file.

## Parked: cross talk between the sides (Oct 2026)

Processing is an extra for now, so this work is parked as it stands. It was
started to identify port/starboard cross talk for three uses: training
labels, point-cloud filtering, and cleaning the raw intensity.

**Done**, in `isss_proc`:
- `CrossTalk`: a stage with a same-bin leak model. It flags a sample whose
  excess lies within `margin` of a leak curve read off the other side's
  intensity.
- `estimate_leak`: measures that curve in each side's water column.
- `clean_intensity`: sets flagged samples to the ping's water-column level.
- Command-line options: `isss-proc --crosstalk --labels` (per-sample
  `rejected_by` and the cross-talk score, for training),
  `isss-proc-view --clean-crosstalk`, and `isss-proc-crosstalk` (curve, flags
  and a check figure).
- Tests run on synthetic pings.

**Open issue: it obscures the nadir.** The default curve into port came from
`estimate_leak` on the Beckholmen bags, but what it measured is port's own
first bottom return. Both sides see the seabed at the same range, and port
reads about 29 counts below starboard there. So the stage, and the cleaned
intensity, wipe out port's nadir line. That must not happen.

No same-bin cross talk is confirmed in those bags
([protocol.md](protocol.md#recorded-data-in-hand)):
- port's shapes that look like starboard's correlate at 0.05;
- port stays at its water column under a saturated starboard object closer
  than the floor.

**Before using it:**
- Find real cross talk first: a recording or bench test where it shows,
  ideally with one side covered.
- Measure it away from the nadir: `estimate_leak`'s water column ends at the
  victim's first sustained return, so a narrow nadir line counts as "water".
- Replace or empty `CrossTalk.DEFAULT_LEAK`.

Until then, leave `--crosstalk` and `--clean-crosstalk` off; neither is in
`default_stages()`.

## Sources of the methods

- sonar3d_reference, `src/sonar3d_reference/experimental/isss/mesh_check.py`:
  intensity excess, bias states, first surface; the calibration of the phase model.
- `py_files` (the 2024 post-processing scripts, related to
  [`ASVROS2/post_process_smarc`](https://github.com/ASVROS2/post_process_smarc)):
  `mpaa.py` (MPAA), the georeferencing, SVP, ray tracing and roll/heave scripts.
  They assume the old angle formula and constants, so port the method, not the code.
