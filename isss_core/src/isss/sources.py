"""Pings from any recorded source, as ``PingArrays``.

Every format the tools read lands here: ``.bin`` logs of this driver, exported
``.nc`` files, rosbag2 bags of this driver (``isss_msgs/ISSS``, MCAP or
sqlite3) and of the old C++ driver (``smarc_msgs/ISSS``).
Readers of other old formats belong here too, so that everything downstream
(the viewer, the report, ``isss_proc``) sees one kind of data.
``pings_from_message_fields`` converts the ``isss_msgs/ISSS`` messages of the
ROS node, for the processing node and for reading its bags.
"""

from __future__ import annotations

import math
from collections import Counter
from pathlib import Path

import numpy as np

from isss.config import SonarConfig
from isss.diagnostics import PingArrays, decode_packets, load_log
from isss.protocol import BYTES_PER_SAMPLE, HEADER_SIZE


def load_pings(
    path: str | Path,
    *,
    num_samples: int | None = None,
    segment: int | None = None,
    topic: str | None = None,
) -> PingArrays:
    """Pings of a ``.bin`` log, an exported ``.nc``, or a rosbag2 bag of either driver.

    ``num_samples`` and ``segment`` apply to ``.bin`` logs (see ``load_log``),
    ``topic`` to bags with more than one ISSS topic. A bag whose sample count
    changes keeps the pings of its most common one. Raises ``ValueError`` for
    a file it cannot read.
    """
    return load_recording(path, num_samples=num_samples, segment=segment, topic=topic)[0]


def load_recording(
    path: str | Path,
    *,
    num_samples: int | None = None,
    segment: int | None = None,
    topic: str | None = None,
) -> tuple[PingArrays, SonarConfig | None]:
    """``load_pings``, plus the calibration the recording carries, or None.

    A bag of this driver has it in every message (the pings of its most common
    configuration are kept), an exported ``.nc`` in its attributes, a ``.bin``
    in its sidecar. Old-driver bags and ``.bin`` logs without a sidecar have none.
    """
    from isss import netcdf, ros2bag
    from isss.sidecar import read_sidecar, recorded_config

    path = Path(path)
    if path.suffix == ".nc":
        try:
            return netcdf.read_netcdf(path), netcdf.read_netcdf_config(path)
        except ImportError:
            raise ValueError("reading .nc files needs the export extra: uv sync --extra export") from None
    if ros2bag.is_bag(path) and ros2bag.bag_kind(path) == "isss":
        pings = ros2bag.read_pings(path, topic)
        if not pings:
            raise ValueError(f"{path} has no ISSS pings")
        key = Counter(p.config_key for p in pings).most_common(1)[0][0]
        pings = [p for p in pings if p.config_key == key]
        return ros2bag.to_arrays(pings), pings[0].config()
    if ros2bag.is_bag(path):
        return _load_other(path, num_samples, segment, topic), None
    return _load_other(path, num_samples, segment, topic), recorded_config(read_sidecar(path))


def _load_other(path: Path, num_samples, segment, topic) -> PingArrays:
    from isss import netcdf

    if netcdf.is_bag(path):
        packets = [p for _, p in netcdf.bag_packets(path, topic)]
        if not packets:
            raise ValueError(f"{path} has no ISSS pings")
        size = Counter(len(p) for p in packets).most_common(1)[0][0]
        return decode_packets([p for p in packets if len(p) == size], (size - HEADER_SIZE) // BYTES_PER_SAMPLE)
    return load_log(path, num_samples=num_samples, segment=segment)


_BYTES = (bytes, bytearray, memoryview)


def pings_from_message_fields(
    ping_time,
    decimation,
    port_intensity,
    starboard_intensity,
    port_phase_counts,
    starboard_phase_counts,
) -> PingArrays:
    """``PingArrays`` from the fields of ``isss_msgs/ISSS`` messages, without ROS.

    Give one ping (a scalar ``ping_time`` and ``decimation``, one sequence of
    samples per array field) or several (a sequence per header field, one row
    of samples per ping). The arrays may be numpy arrays, lists, ``array.array``
    as rclpy gives them, or ``bytes`` for the intensities. The counts are
    already signed in the message. ``size_field`` (header bytes 2-3) is not in
    the message, so it is 0. Raises ``ValueError`` if the sample counts differ.
    """
    intensity = [_samples(port_intensity, np.uint8), _samples(starboard_intensity, np.uint8)]
    counts = [_samples(port_phase_counts, np.int16), _samples(starboard_phase_counts, np.int16)]
    ping_time = np.atleast_1d(np.asarray(ping_time, dtype=np.uint32))
    decimation = np.atleast_1d(np.asarray(decimation, dtype=np.uint32))
    shapes = {a.shape for a in intensity + counts}
    if len(shapes) != 1 or next(iter(shapes))[0] != ping_time.shape[0] or decimation.shape != ping_time.shape:
        raise ValueError(f"ISSS message fields disagree: {ping_time.shape[0]} ping times, "
                         f"{decimation.shape[0]} decimations, sample arrays {sorted(shapes)}")
    return PingArrays(
        ping_time=ping_time,
        decimation=decimation,
        size_field=np.zeros_like(ping_time),
        intensity=np.stack(intensity, axis=-1),
        counts=np.stack(counts, axis=-1),
    )


def config_from_message_fields(
    *,
    range: float,
    num_samples: int,
    sound_velocity: float,
    counts_per_unit_sine: float,
    boresight_depression: float,
    port_phase_sign: int,
    starboard_phase_sign: int,
) -> SonarConfig:
    """The ``SonarConfig`` an ``isss_msgs/ISSS`` message was recorded with (``boresight_depression`` in rad)."""
    return SonarConfig(
        range_m=float(range),
        num_samples=int(num_samples),
        sound_velocity=float(sound_velocity),
        counts_per_unit_sine=float(counts_per_unit_sine),
        boresight_depression_deg=math.degrees(boresight_depression),
        port_phase_sign=int(port_phase_sign),
        starboard_phase_sign=int(starboard_phase_sign),
    )


def _samples(values, dtype) -> np.ndarray:
    """(pings, samples) from one ping's samples or a sequence of them."""
    if isinstance(values, np.ndarray):
        return np.atleast_2d(values.astype(dtype, copy=False))
    if isinstance(values, _BYTES) or (len(values) and not isinstance(values[0], _BYTES) and np.isscalar(values[0])):
        return _row(values, dtype)[None]
    rows = [_row(v, dtype) for v in values]
    return np.stack(rows) if rows else np.zeros((0, 0), dtype)


def _row(values, dtype) -> np.ndarray:
    if isinstance(values, _BYTES):
        return np.frombuffer(values, dtype=np.uint8).astype(dtype)
    return np.asarray(values, dtype=dtype)
