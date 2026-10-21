"""Export pings to NetCDF4: one self-describing file per log, for analysis.

The ``.bin`` log stays the lossless record of what the sonar sent; this is the
format to analyse. Open it with ``xarray.open_dataset(path)`` (or h5py, MATLAB,
Panoply, ncdump). Layout, CF-1.10 where it applies::

    dimensions:  ping (unlimited), sample, side = 2 (0 port, 1 starboard)
    coordinates: time(ping)            UTC, seconds since 1970-01-01
                 slant_range(sample)   m, at the file's sound speed and decimation
                 side(side)
    variables:   intensity(ping, sample, side)     uint8 RSS
                 phase_counts(ping, sample, side)  int16, one cycle = 20480 counts
                 depression(ping, sample, side)    float32 rad, from the phase model
                 ping_time(ping) uint32 [100 us], decimation(ping), size_field(ping),
                 sample_interval(ping) [s]

Global attributes carry the calibration (``counts_per_unit_sine``,
``boresight_depression_deg``, ``*_phase_sign``, ``sound_velocity``), the source
file, and the log's sidecar as JSON. Every file holds one sample count and one
decimation; a log in which either changes is split into ``<stem>.nc``,
``<stem>_2.nc``, ... Needs the ``export`` extra (h5netcdf, h5py, numpy).
"""

from __future__ import annotations

import json
import mmap
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Optional

import numpy as np

from isss.config import SonarConfig
from isss.diagnostics import PingArrays, decode_packets
from isss.log import find_packet_offsets
from isss.protocol import (
    BYTES_PER_SAMPLE,
    DEFAULT_SAMPLES,
    HEADER_SIZE,
    PHASE_COUNTS_PER_CYCLE,
    packet_size,
    sample_interval,
)
from isss.sidecar import read_sidecar, segment_ranges

FORMAT = "isss-netcdf"
VERSION = 1
BLOCK = 1024  # pings decoded and written at a time
_WRAP = 2**32


class NetcdfWriter:
    """Append pings of one sample count and decimation to a new NetCDF4 file."""

    def __init__(self, path: str | Path, config: SonarConfig, num_samples: int, decimation: int,
                 attrs: Optional[dict[str, Any]] = None) -> None:
        import h5netcdf

        self.path = Path(path)
        self.config = config
        self.num_samples, self.decimation = num_samples, decimation
        self.n_pings = 0
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._tmp = self.path.with_suffix(".nc.tmp")
        f = self._file = h5netcdf.File(self._tmp, "w")
        f.dimensions = {"ping": None, "sample": num_samples, "side": 2}
        f.attrs.update(_global_attrs(config))
        f.attrs.update({k: v for k, v in (attrs or {}).items() if v is not None})

        side = f.create_variable("side", ("side",), np.int8, data=np.array([0, 1], np.int8))
        side.attrs.update(long_name="sonar side", flag_values=np.array([0, 1], np.int8),
                          flag_meanings="port starboard")
        dt = sample_interval(decimation)
        r = f.create_variable("slant_range", ("sample",), np.float32,
                              data=(np.arange(num_samples) * dt * config.sound_velocity / 2.0).astype(np.float32))
        r.attrs.update(long_name="slant range of the range bin", units="m",
                       comment=f"bin * sample_interval * sound_velocity / 2, decimation {decimation}, "
                               f"sound_velocity {config.sound_velocity} m/s")

        def ping_var(name, dtype, **attrs):
            v = f.create_variable(name, ("ping",), dtype, chunks=(BLOCK,))
            v.attrs.update(attrs)

        ping_var("time", np.float64, standard_name="time", long_name="ping time (UTC)",
                 units="seconds since 1970-01-01 00:00:00", calendar="standard")
        ping_var("ping_time", np.uint32, long_name="device ping counter", units="1e-4 s",
                 comment="from the packet header; wraps after about 4.97 days, resets at power-on")
        ping_var("decimation", np.uint16, long_name="decimation register",
                 comment="sample interval = decimation * 3 / 12.5 MHz")
        ping_var("sample_interval", np.float32, long_name="time between range bins", units="s")
        ping_var("size_field", np.uint16, long_name="packet header bytes 2-3",
                 comment="not the packet size; see docs/protocol.md")

        def sample_var(name, dtype, fill=None, **attrs):
            v = f.create_variable(name, ("ping", "sample", "side"), dtype, chunks=(256, num_samples, 2),
                                  compression="gzip", compression_opts=4, shuffle=True, fillvalue=fill)
            v.attrs.update(attrs)

        sample_var("intensity", np.uint8, long_name="echo intensity (RSS)", units="1",
                   coordinates="time slant_range")
        sample_var("phase_counts", np.int16, long_name="interferometric phase difference between the two receivers",
                   units="1", counts_per_cycle=np.int32(PHASE_COUNTS_PER_CYCLE),
                   comment="signed hardware counts; one electrical cycle (2 pi) is 20480 counts, "
                           "values normally in [-10240, 10240)", coordinates="time slant_range")
        sample_var("depression", np.float32, fill=np.float32(np.nan),
                   long_name="depression angle of the return below the sensor horizontal, toward that side",
                   units="rad", coordinates="time slant_range",
                   comment="beta = beta0 - asin(sign * counts / K), with the calibration in the global "
                           "attributes; NaN where |counts| > K. Returns more than asin(10240 / K) from "
                           "the boresight alias (nadir does). See docs/protocol.md.")

    def append(self, arrays: PingArrays, time: np.ndarray) -> None:
        """Append pings; ``time`` is UTC [s] per ping (NaN where unknown)."""
        n = arrays.n_pings
        if n == 0:
            return
        if arrays.num_samples != self.num_samples or (arrays.decimation != self.decimation).any():
            raise ValueError("pings do not match this file's sample count and decimation")
        a, b = self.n_pings, self.n_pings + n
        f = self._file
        f.resize_dimension("ping", b)
        v = f.variables
        v["time"][a:b] = time
        v["ping_time"][a:b] = arrays.ping_time
        v["decimation"][a:b] = arrays.decimation
        v["sample_interval"][a:b] = sample_interval(self.decimation)
        v["size_field"][a:b] = arrays.size_field
        v["intensity"][a:b] = arrays.intensity
        v["phase_counts"][a:b] = arrays.counts
        v["depression"][a:b] = arrays.depression(self.config).astype(np.float32)
        self.n_pings = b

    def close(self) -> Path:
        """Finish the file; it appears at ``path`` only now, so a crash leaves no half file."""
        if self._file is not None:
            self._file.attrs["pings"] = np.int64(self.n_pings)
            self._file.close()
            self._file = None
            self._tmp.replace(self.path)
        return self.path

    def abort(self) -> None:
        """Discard the file."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._tmp.unlink(missing_ok=True)


def write_runs(
    out: Path,
    config: SonarConfig,
    blocks: Iterable[tuple[PingArrays, np.ndarray]],
    attrs: dict[str, Any],
) -> list[Path]:
    """Write ``(arrays, time)`` blocks, starting a new file whenever sample count or decimation change."""
    paths: list[Path] = []
    writer: Optional[NetcdfWriter] = None
    try:
        for arrays, time in blocks:
            for start, end in _runs(arrays.decimation):
                part = arrays.select(slice(start, end))
                key = (part.num_samples, int(part.decimation[0]))
                if writer is None or key != (writer.num_samples, writer.decimation):
                    if writer is not None:
                        paths.append(writer.close())
                    path = out if not paths else out.with_name(f"{out.stem}_{len(paths) + 1}.nc")
                    writer = NetcdfWriter(path, config, *key, attrs={**attrs, "part": np.int32(len(paths) + 1)})
                writer.append(part, time[start:end])
        if writer is not None:
            paths.append(writer.close())
    except BaseException:
        if writer is not None:
            writer.abort()
        raise
    return paths


def read_netcdf(path: str | Path) -> PingArrays:
    """The pings of an exported file, as the diagnostic tools use them."""
    import h5netcdf

    with h5netcdf.File(path, "r") as f:
        if f.attrs.get("isss_format") != FORMAT:
            raise ValueError(f"{path} is not an {FORMAT} file")
        v = f.variables
        return PingArrays(
            ping_time=v["ping_time"][:].astype(np.uint32),
            decimation=v["decimation"][:].astype(np.uint32),
            size_field=v["size_field"][:].astype(np.uint32),
            intensity=v["intensity"][:],
            counts=v["phase_counts"][:],
        )


# .bin logs


def export_bin(bin_path: str | Path, out: str | Path | None = None, config: SonarConfig | None = None) -> list[Path]:
    """Export a ``.bin`` log (and its sidecar) to ``<log>.nc``; returns the files written."""
    bin_path = Path(bin_path)
    config = config or SonarConfig()
    meta = read_sidecar(bin_path)
    to_utc, time_source = _bin_clock(bin_path, meta)
    attrs = _source_attrs(bin_path, f"isss-export of {bin_path.name}", time_source, meta)
    out = Path(out) if out else bin_path.with_suffix(".nc")
    return write_runs(out, config, _bin_blocks(bin_path, meta, to_utc), attrs)


def is_bag(path: str | Path) -> bool:
    path = Path(path)
    return path.suffix == ".db3" or (path.is_dir() and any(path.glob("*.db3")))


def _bin_blocks(bin_path: Path, meta: Optional[dict], to_utc: Callable) -> Iterator[tuple[PingArrays, np.ndarray]]:
    with bin_path.open("rb") as fh:
        size = bin_path.stat().st_size
        if size == 0:
            return
        with mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as data:
            if meta is not None:
                ranges = [(s, e, seg["num_samples"]) for s, e, seg in segment_ranges(meta, size)]
            else:
                ranges = [(0, size, DEFAULT_SAMPLES)]
            for start, end, num_samples in ranges:
                chunk = data if (start, end) == (0, size) else data[start:end]
                n = packet_size(num_samples)
                offsets = [o for o in find_packet_offsets(chunk, num_samples=num_samples) if o + n <= end - start]
                for i in range(0, len(offsets), BLOCK):
                    arrays = decode_packets([chunk[o : o + n] for o in offsets[i : i + BLOCK]], num_samples)
                    yield arrays, to_utc(arrays.ping_time)


def _bin_clock(bin_path: Path, meta: Optional[dict]) -> tuple[Callable[[np.ndarray], np.ndarray], str]:
    """Map the device ping counter to UTC.

    With a sidecar: through its first and last packets (host time paired with
    the counter), linear between them so device clock drift is absorbed.
    Without one: the log's file name (host time when it was opened) for the
    first packet, which is only good to a fraction of a second.
    """
    first = (meta or {}).get("first_packet") or {}
    last = (meta or {}).get("last_packet") or {}
    if first.get("unix") is not None and first.get("ping_time") is not None:
        u0, p0 = float(first["unix"]), int(first["ping_time"])
        rate = 1e-4
        if last.get("unix") is not None and last.get("ping_time") is not None:
            span = ((int(last["ping_time"]) - p0) % _WRAP) * 1e-4
            if span > 10.0:
                fitted = (float(last["unix"]) - u0) / span * 1e-4
                if abs(fitted / 1e-4 - 1.0) < 1e-3:  # a sane clock; else the counter reset mid-log
                    rate = fitted
        source = "sidecar: host time of the first and last packets paired with the device ping counter"
        return (lambda pt: u0 + ((pt.astype(np.int64) - p0) % _WRAP) * rate), source
    try:
        u0 = float(bin_path.stem)
    except ValueError:
        return (lambda pt: np.full(pt.shape, np.nan)), "unknown: no sidecar and no time in the file name"
    state: dict[str, int] = {}

    def from_name(pt: np.ndarray) -> np.ndarray:
        if pt.size and "p0" not in state:
            state["p0"] = int(pt[0])
        return u0 + ((pt.astype(np.int64) - state.get("p0", 0)) % _WRAP) * 1e-4

    return from_name, "log file name (host time when the log was opened) and the device ping counter; approximate"


# rosbag2


def bag_packets(bag: str | Path, topic: str | None = None) -> list[tuple[int, bytes]]:
    """``(stamp [ns], packet)`` of a bag's ISSS pings, in stamp order."""
    from isss.bag import iter_bag_packets

    return sorted(iter_bag_packets(bag, topic), key=lambda sp: sp[0])


def export_bag(bag: str | Path, out: str | Path | None = None, config: SonarConfig | None = None,
               topic: str | None = None) -> list[Path]:
    """Export the ``smarc_msgs/ISSS`` pings of a rosbag2 bag to ``<bag>.nc`` (see ``isss.bag``)."""
    bag = Path(bag)
    config = config or SonarConfig()
    stamped = bag_packets(bag, topic)
    attrs = _source_attrs(bag, f"isss-export of rosbag2 {bag.name} (topic {topic or 'auto'})",
                          "ROS header stamps of the old C++ driver", None)
    out = Path(out) if out else bag.with_name(bag.name + ".nc")

    def blocks() -> Iterator[tuple[PingArrays, np.ndarray]]:
        i = 0
        while i < len(stamped):
            num_samples = (len(stamped[i][1]) - HEADER_SIZE) // BYTES_PER_SAMPLE
            j = i
            while j < len(stamped) and j - i < BLOCK and len(stamped[j][1]) == len(stamped[i][1]):
                j += 1
            arrays = decode_packets([p for _, p in stamped[i:j]], num_samples)
            yield arrays, np.array([s for s, _ in stamped[i:j]], dtype=np.float64) * 1e-9
            i = j

    return write_runs(out, config, blocks(), attrs)


# Attributes


def _global_attrs(config: SonarConfig) -> dict[str, Any]:
    from isss import __version__

    return {
        "Conventions": "CF-1.10",
        "title": "DeepVision ISSS interferometric sidescan pings",
        "isss_format": FORMAT,
        "isss_format_version": np.int32(VERSION),
        "instrument": "DeepVision interferometric sidescan sonar (ISSS)",
        "software": f"isss-core {__version__}",
        "references": "https://github.com/julRusVal/isss_ros2_driver, docs/protocol.md",
        "phase_model": "counts = sign * K * sin(beta0 - depression)",
        "counts_per_unit_sine": float(config.counts_per_unit_sine),
        "boresight_depression_deg": float(config.boresight_depression_deg),
        "port_phase_sign": np.int8(config.port_phase_sign),
        "starboard_phase_sign": np.int8(config.starboard_phase_sign),
        "sound_velocity": float(config.sound_velocity),
    }


def _source_attrs(source: Path, history: str, time_source: str, meta: Optional[dict]) -> dict[str, Any]:
    now = datetime.now(timezone.utc).isoformat(timespec="seconds")
    return {
        "source_file": source.name,
        "history": f"{now} {history}",
        "time_source": time_source,
        "sidecar": json.dumps(meta) if meta is not None else None,
    }


def _runs(values: np.ndarray) -> list[tuple[int, int]]:
    """``(start, end)`` of runs of equal values."""
    if values.size == 0:
        return []
    cuts = np.flatnonzero(np.diff(values.astype(np.int64))) + 1
    edges = [0, *cuts.tolist(), values.size]
    return list(zip(edges[:-1], edges[1:]))
