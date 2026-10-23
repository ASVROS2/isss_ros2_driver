"""The commands and the filtered viewer, headless."""

from types import SimpleNamespace

import numpy as np
import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from isss import SonarConfig  # noqa: E402
from isss.diagnostics import decode_packets  # noqa: E402
from isss.sim import synthetic_packet  # noqa: E402
from isss.viz import PingViewer  # noqa: E402
from isss_proc import viewer_keep  # noqa: E402
from isss_proc.cli import main_crosstalk, main_process  # noqa: E402


def _packets(n=40):
    return [synthetic_packet(ping_time=i * 268, bottom_sample=250) for i in range(n)]


def test_process_reports_stages_and_saves_the_cloud(tmp_path, capsys):
    log = tmp_path / "flat.bin"
    log.write_bytes(b"".join(_packets()))
    out = tmp_path / "cloud.npz"
    main_process([str(log), "-o", str(out), "--block", "16", "--mpaa"])
    text = capsys.readouterr().out
    assert "40 pings" in text and "first_surface" in text and "mpaa" in text
    cloud = np.load(out)
    assert set(cloud.files) >= {"y", "z", "intensity", "side", "ping", "range", "depression"}
    assert cloud["ping"].max() >= 32  # pings numbered across blocks


def test_process_saves_labels(tmp_path, capsys):
    log = tmp_path / "flat.bin"
    log.write_bytes(b"".join(_packets()))
    out = tmp_path / "labels.npz"
    main_process([str(log), "--labels", str(out), "--block", "16", "--crosstalk"])
    assert "crosstalk" in capsys.readouterr().out
    labels = np.load(out)
    assert labels["rejected_by"].shape == labels["crosstalk_score"].shape == (40, 1000, 2)
    assert list(labels["stages"])[:3] == ["kept", "range", "intensity"] and "crosstalk" in labels["stages"]
    assert labels["slant_range"].shape == (1000,) and labels["ping_time"].shape == (40,)


def test_crosstalk_command_reports_and_draws(tmp_path, capsys):
    log = tmp_path / "flat.bin"
    log.write_bytes(b"".join(_packets()))
    fig = tmp_path / "crosstalk.png"
    main_crosstalk([str(log), "-o", str(fig), "--pings", "5:35"])
    text = capsys.readouterr().out
    assert "into port" in text and "flagged starboard" in text and fig.stat().st_size > 0


def test_viewer_shows_the_kept_returns_of_the_last_pings():
    arrays = decode_packets(_packets())
    viewer = PingViewer(arrays, SonarConfig(), keep=viewer_keep(context=20), trail=10)
    assert "Last 10 pings" in viewer.ax_prof.get_title() and "kept" in viewer.ax_prof.get_title()
    viewer._select(3)
    assert "Last 4 pings" in viewer.ax_prof.get_title()
    viewer._on_key(SimpleNamespace(key="2"))  # flipping still redraws through the filters
    assert viewer.config.starboard_phase_sign == -1
