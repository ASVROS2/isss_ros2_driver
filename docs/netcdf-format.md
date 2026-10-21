# NetCDF export format (`isss-netcdf`, version 1)

The `.bin` log is the lossless record of what the sonar sent. The `.nc` file is
the one to analyse: one self-describing NetCDF4 (HDF5) file per log, with named
dimensions, units and the calibration inside it. It opens in xarray, h5py,
MATLAB, Panoply and `ncdump`.

```python
import xarray as xr

ds = xr.open_dataset("1761040000.123456.nc")
starboard = ds.intensity.sel(side=1)          # (ping, sample)
ds.depression.where(ds.intensity > 120)       # angles of strong returns only
ds.time                                       # UTC per ping, decoded to datetime64
```

## Where the files come from

| Source | How | `time` |
|---|---|---|
| The driver (`SonarDriver`, ROS node) | Automatically, `<log>.nc` next to each `<log>.bin` when the log closes (`export_netcdf`, on by default) | Host time of the first and last packets from the sidecar, paired with the device ping counter, linear in between, so device clock drift is absorbed |
| An old `.bin` log | `isss-export log.bin` | As above with a sidecar. Without one, the time in the file name (when the log was opened), approximate. Otherwise NaT |
| A rosbag2 bag of the old C++ driver (`smarc_msgs/ISSS`) | `isss-export rosbag2_dir/` → `rosbag2_dir.nc` | ROS header stamps, sorted |

The `time_source` attribute says which. Old-driver bags still hold the
original packet: `extra_channel` is the 12-byte header, and the phase field is
the unsigned phase word / 512, which is exact in float32 (`isss/bag.py`). So
the export from a bag carries the same samples a `.bin` would have. Only
sqlite3 bags are read, and no ROS install is needed.

Every file holds one sample count and one decimation (range). A log in which
either changes is split into `<stem>.nc`, `<stem>_2.nc`, ... (`part`
attribute). Files appear only when complete: they are written as `.nc.tmp` and
renamed.

## Layout

Dimensions: `ping` (unlimited), `sample` (range bins, nearest first), `side`
(2: 0 = port, 1 = starboard).

| Variable | Dims | Type | Units | Meaning |
|---|---|---|---|---|
| `time` | ping | float64 | seconds since 1970-01-01 UTC | Ping time, see above; NaN if unknown |
| `slant_range` | sample | float32 | m | `bin · sample_interval · sound_velocity / 2` |
| `side` | side | int8 | | 0 port, 1 starboard (`flag_meanings`) |
| `intensity` | ping, sample, side | uint8 | | Echo intensity (RSS) |
| `phase_counts` | ping, sample, side | int16 | | Signed hardware phase. One cycle (2π) is 20480 counts, values normally in [−10240, 10240) |
| `depression` | ping, sample, side | float32 | rad | Angle below the sensor horizontal, toward that side, from the phase model and the calibration in the attributes; NaN where there is no solution |
| `ping_time` | ping | uint32 | 1e-4 s | Device ping counter from the packet header |
| `decimation` | ping | uint16 | | Decimation register; `sample_interval = decimation · 3 / 12.5 MHz` |
| `sample_interval` | ping | float32 | s | Time between range bins |
| `size_field` | ping | uint16 | | Header bytes 2–3 (not the packet size) |

`depression` is derived: it bakes in the calibration at export time. To try
another calibration, recompute it from `phase_counts`, or re-export with
`isss-export --k --boresight --flip-port ...`. Returns more than
`asin(10240 / K)` (about 42°) from the boresight alias; see
[protocol.md](protocol.md#phase-and-depression).

Global attributes: `Conventions` (CF-1.10), `isss_format`,
`isss_format_version`, `software`, `phase_model`, `counts_per_unit_sine`,
`boresight_depression_deg`, `port_phase_sign`, `starboard_phase_sign`,
`sound_velocity`, `source_file`, `history`, `time_source`, `part`, `pings`, and
`sidecar` (the log's JSON sidecar, if it had one).

The sample arrays are gzip-compressed (level 4, shuffle) in chunks of 256
pings. A 20 m, 1000-sample log takes about 11 KB per ping.
