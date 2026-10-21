"""The JSON sidecar written next to every ``.bin`` log.

A ``.bin`` holds the sonar's packets exactly as received and nothing else: not
the sample count needed to split it, not the registers written, not when it was
recorded. The sidecar ``<log>.json`` records those. Example::

    {
      "format": "isss-bin-sidecar", "version": 1,
      "bin": "1761040000.123456.bin",
      "writer": {"package": "isss-core", "version": "0.2.0"},
      "sonar": {"ip": "192.168.2.62", "port": 57349},
      "calibration": {"sound_velocity": 1440.9, "counts_per_unit_sine": 15218.0,
                      "boresight_depression_deg": 44.5,
                      "port_phase_sign": 1, "starboard_phase_sign": 1},
      "segments": [{"byte_offset": 0, "utc": "...", "range_m": 20.0, "num_samples": 1000,
                    "num_phases": 2, "decimation": 111}],
      "register_writes": [{"utc": "...", "register": 16, "value": 111}, ...],
      "first_packet": {"utc": "...", "unix": 1761040000.2, "ping_time": 141481840},
      "last_packet": {...},
      "packets": 127, "bytes": 763524,
      "opened_utc": "...", "closed_utc": "..."
    }

A new segment starts whenever range, sample count or phase count change; its
``byte_offset`` is where the next packet was written, so the first packets of a
segment may still have the old size. ``first_packet``/``last_packet`` pair host
wall-clock time with the device ping counter, which maps ping time to UTC.
The sidecar is rewritten on every change, so a crash leaves it without
``closed_utc`` but otherwise valid.
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from isss.config import SonarConfig
from isss.protocol import compute_decimation

FORMAT = "isss-bin-sidecar"
VERSION = 1


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def sidecar_path(bin_path: str | Path) -> Path:
    return Path(bin_path).with_suffix(".json")


def read_sidecar(bin_path: str | Path) -> dict[str, Any] | None:
    """The sidecar of a log, or None if there is none (older logs)."""
    path = sidecar_path(bin_path)
    if not path.exists():
        return None
    data = json.loads(path.read_text())
    if data.get("format") != FORMAT:
        raise ValueError(f"{path} is not an ISSS sidecar")
    return data


def segment(config: SonarConfig, byte_offset: int) -> dict[str, Any]:
    return {
        "byte_offset": byte_offset,
        "utc": utc_now(),
        "range_m": config.range_m,
        "num_samples": config.num_samples,
        "num_phases": config.num_phases,
        "decimation": compute_decimation(config.range_m, config.num_samples),
    }


def register_entries(writes: Iterable[tuple[int, int]]) -> list[dict[str, Any]]:
    now = utc_now()
    return [{"utc": now, "register": reg, "value": value} for reg, value in writes]


def new_sidecar(bin_path: Path, config: SonarConfig, register_writes: Iterable[tuple[int, int]]) -> dict[str, Any]:
    from isss import __version__

    return {
        "format": FORMAT,
        "version": VERSION,
        "bin": bin_path.name,
        "writer": {"package": "isss-core", "version": __version__},
        "sonar": {"ip": config.ip, "port": config.port},
        "calibration": {
            "sound_velocity": config.sound_velocity,
            "counts_per_unit_sine": config.counts_per_unit_sine,
            "boresight_depression_deg": config.boresight_depression_deg,
            "port_phase_sign": config.port_phase_sign,
            "starboard_phase_sign": config.starboard_phase_sign,
        },
        "segments": [segment(config, 0)],
        "register_writes": register_entries(register_writes),
        "first_packet": None,
        "last_packet": None,
        "packets": 0,
        "bytes": 0,
        "opened_utc": utc_now(),
        "closed_utc": None,
    }


def write_sidecar(bin_path: Path, data: dict[str, Any]) -> None:
    """Write atomically, so a reader never sees half a file."""
    path = sidecar_path(bin_path)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    os.replace(tmp, path)


def segment_ranges(sidecar: dict[str, Any], file_size: int) -> list[tuple[int, int, dict[str, Any]]]:
    """``(start, end, segment)`` byte ranges of a log's segments."""
    segs = sidecar.get("segments") or []
    ends = [s["byte_offset"] for s in segs[1:]] + [file_size]
    return [(s["byte_offset"], end, s) for s, end in zip(segs, ends)]
