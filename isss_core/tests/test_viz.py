"""The figures and the viewer build and respond, headless."""

import math
from types import SimpleNamespace

import pytest

pytest.importorskip("numpy")
matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from isss import SonarConfig  # noqa: E402
from isss.cli import main_report  # noqa: E402
from isss.diagnostics import decode_packets  # noqa: E402
from isss.sim import synthetic_packet  # noqa: E402
from isss.viz import PingViewer, report_figures  # noqa: E402

from pathlib import Path  # noqa: E402

LOG = Path(__file__).parent / "data" / "stick_2025-08-19_5pings.bin"


def _arrays(n=30):
    return decode_packets([synthetic_packet(ping_time=i * 268, bottom_sample=250) for i in range(n)])


def test_report_figures():
    figs = report_figures(_arrays(), SonarConfig())
    assert set(figs) == {"waterfalls", "timing", "phase_histograms", "phase_vs_range", "sensor_profile"}


def test_report_command_writes_files(tmp_path, capsys):
    main_report([str(LOG), "-o", str(tmp_path)])
    for name in ("README.md", "summary.json", "waterfalls.png", "sensor_profile.png"):
        assert (tmp_path / name).exists()
    assert "WARN" in capsys.readouterr().out  # the fixture was recorded out of the water


def test_viewer_selection_and_keys():
    viewer = PingViewer(_arrays(), SonarConfig())
    assert viewer.index == 29
    viewer._select(5)
    assert viewer.index == 5 and not viewer.follow
    viewer._on_key(SimpleNamespace(key="right"))
    viewer._on_key(SimpleNamespace(key="down"))
    assert viewer.index == 16
    viewer._on_key(SimpleNamespace(key="p"))
    assert viewer.show_phase


def test_viewer_flips_one_side():
    config = SonarConfig()
    viewer = PingViewer(_arrays(), config)
    viewer._on_key(SimpleNamespace(key="1"))
    assert viewer.config.port_phase_sign == -1 and viewer.config.starboard_phase_sign == 1
    assert viewer.flip_boxes.get_status() == [True, False]
    assert "port flipped" in viewer.ax_prof.get_title(loc="left")
    viewer.flip_boxes.set_active(1)  # clicking the starboard box
    assert viewer.config.starboard_phase_sign == -1
    viewer._on_key(SimpleNamespace(key="1"))
    assert viewer.config.port_phase_sign == 1 and viewer.flip_boxes.get_status() == [False, True]
    assert config.port_phase_sign == 1 and config.starboard_phase_sign == 1  # caller's config untouched


def test_viewer_angle_readout():
    viewer = PingViewer(_arrays(), SonarConfig())
    boresight = viewer.angles_at(math.cos(math.radians(44.5)), -math.sin(math.radians(44.5)))
    assert boresight["side"] == 1 and abs(boresight["counts"]) < 1e-6
    shallow = viewer.angles_at(-10.0, -1.0)  # port, near horizontal: above the boresight
    assert shallow["side"] == 0 and shallow["off_boresight_deg"] > 0 and shallow["counts"] > 0
    viewer.flip(0)
    assert viewer.angles_at(-10.0, -1.0)["counts"] == pytest.approx(-shallow["counts"])
    assert viewer.angles_at(-0.1, -10.0)["aliased"]  # nadir


def test_live_viewer_follows_new_pings():
    viewer = PingViewer(decode_packets([]), SonarConfig(), live=True)
    viewer.set_arrays(_arrays(10))
    assert viewer.index == 9
    viewer.set_arrays(_arrays(20))
    assert viewer.index == 19
    viewer._on_key(SimpleNamespace(key="f"))  # stop following
    viewer.set_arrays(_arrays(25))
    assert viewer.index == 19


def test_live_buffer_from_simulator():
    import time

    from isss import SonarDriver
    from isss.diagnostics import PacketBuffer
    from isss.sim import SimulatedSonar

    buffer = PacketBuffer(maxlen=20)
    with SimulatedSonar(rate_hz=100) as sim:
        config = SonarConfig(ip="127.0.0.1", port=sim.port, recv_timeout_sec=1.0, reconnect_interval_sec=0.1)
        with SonarDriver(config, lambda ping: buffer.add(ping.raw)) as driver:
            deadline = time.monotonic() + 3
            while len(buffer) < 20 and time.monotonic() < deadline:
                time.sleep(0.02)
            viewer = PingViewer(buffer.arrays(), config, live=True)
            assert viewer.arrays.n_pings == 20
            driver.configure(num_samples=500)
            deadline = time.monotonic() + 3
            while buffer.arrays().num_samples != 500 and time.monotonic() < deadline:
                time.sleep(0.02)
            viewer.set_arrays(buffer.arrays())
            assert viewer.arrays.num_samples == 500
