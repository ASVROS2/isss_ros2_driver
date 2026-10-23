"""Pings from any recorded source, as ``PingArrays``.

Every format the tools read lands here: ``.bin`` logs of this driver, exported
``.nc`` files, and rosbag2 bags of the old C++ driver (``smarc_msgs/ISSS``).
Readers of other old formats belong here too, so that everything downstream
(the viewer, the report, ``isss_proc``) sees one kind of data.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path

from isss.diagnostics import PingArrays, decode_packets, load_log
from isss.protocol import BYTES_PER_SAMPLE, HEADER_SIZE


def load_pings(
    path: str | Path,
    *,
    num_samples: int | None = None,
    segment: int | None = None,
    topic: str | None = None,
) -> PingArrays:
    """Pings of a ``.bin`` log, an exported ``.nc``, or a rosbag2 bag of the old driver.

    ``num_samples`` and ``segment`` apply to ``.bin`` logs (see ``load_log``),
    ``topic`` to bags with more than one ISSS topic. A bag whose sample count
    changes keeps the pings of its most common one. Raises ``ValueError`` for
    a file it cannot read.
    """
    from isss import netcdf

    path = Path(path)
    if path.suffix == ".nc":
        try:
            return netcdf.read_netcdf(path)
        except ImportError:
            raise ValueError("reading .nc files needs the export extra: uv sync --extra export") from None
    if netcdf.is_bag(path):
        packets = [p for _, p in netcdf.bag_packets(path, topic)]
        if not packets:
            raise ValueError(f"{path} has no ISSS pings")
        size = Counter(len(p) for p in packets).most_common(1)[0][0]
        return decode_packets([p for p in packets if len(p) == size], (size - HEADER_SIZE) // BYTES_PER_SAMPLE)
    return load_log(path, num_samples=num_samples, segment=segment)
