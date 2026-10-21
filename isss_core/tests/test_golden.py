"""Pin the decoder's output on real data so any change to decoding is deliberate.

The expected values are a snapshot of the current decoder, not ground truth.
Bearing is left out on purpose: its formula is unresolved (docs/protocol.md).
After an intended decoder change, regenerate with:

    ISSS_REGEN_GOLDEN=1 uv run pytest tests/test_golden.py
"""

import json
import os
from pathlib import Path

import pytest

from isss.config import SonarConfig
from isss.log import iter_packets_from_log
from isss.protocol import HEADER_SIZE, parse_header

DATA = Path(__file__).parent / "data"
LOG = DATA / "stick_2025-08-19_5pings.bin"
EXPECTED = DATA / "stick_2025-08-19_5pings.expected.json"


def _decode() -> list[dict]:
    pings = []
    for samples, packet in iter_packets_from_log(LOG, config=SonarConfig()):
        header = parse_header(packet[:HEADER_SIZE])
        pings.append({
            "header": {
                "packet_size": header.packet_size,
                "packet_type": header.packet_type,
                "ping_time": header.ping_time,
                "frequency_id": header.frequency_id,
                "decimation": header.decimation,
            },
            "port_rss": samples.port_rss,
            "starboard_rss": samples.starboard_rss,
            "port_phase_diff": [round(v, 9) for v in samples.port_phase_diff],
            "starboard_phase_diff": [round(v, 9) for v in samples.starboard_phase_diff],
        })
    return pings


def test_decoder_matches_snapshot():
    decoded = _decode()
    if os.environ.get("ISSS_REGEN_GOLDEN"):
        EXPECTED.write_text(json.dumps(decoded) + "\n")
        pytest.skip("Regenerated golden snapshot")
    assert decoded == json.loads(EXPECTED.read_text())


def test_snapshot_covers_every_ping():
    assert len(_decode()) == 5
