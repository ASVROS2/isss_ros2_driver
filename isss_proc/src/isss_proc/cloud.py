"""Kept returns packed as the data of a ``sensor_msgs/PointCloud2``, without ROS.

The ROS processing node fills the message from ``POINT_FIELDS``,
``POINT_STEP`` and ``pack_points``; everything here is numpy, so the layout
is tested without ROS. Points are in the sensor frame of ``Result.points``:
``x`` along-track is 0 (no motion), ``y`` across-track with port negative,
``z`` up. TF places them in the world.
"""

from __future__ import annotations

import numpy as np

# sensor_msgs/PointField datatypes
UINT8 = 2
FLOAT32 = 7

# Little-endian, 4-byte aligned; side is padded to a 24-byte point
POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "range", "side"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "u1"],
    "offsets": [0, 4, 8, 12, 16, 20],
    "itemsize": 24,
})
POINT_STEP = POINT_DTYPE.itemsize

POINT_FIELDS: list[tuple[str, int, int]] = [
    (name, POINT_DTYPE.fields[name][1], UINT8 if POINT_DTYPE.fields[name][0].kind == "u" else FLOAT32)
    for name in POINT_DTYPE.names
]
"""(name, offset, PointField datatype) of each field, in offset order; count is 1."""


def point_records(points: dict[str, np.ndarray]) -> np.ndarray:
    """One ``POINT_DTYPE`` record per point of ``Result.points()``: x = 0, y, z, intensity, range, side (0 port)."""
    out = np.zeros(len(points["y"]), dtype=POINT_DTYPE)
    for name in ("y", "z", "intensity", "range", "side"):
        out[name] = points[name]
    return out


def pack_points(points: dict[str, np.ndarray]) -> bytes:
    """The ``data`` of a PointCloud2 (height 1, width = number of points, little-endian)."""
    return point_records(points).tobytes()
