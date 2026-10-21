"""Diagnostic figures and an interactive ping viewer (matplotlib).

Every figure function draws into axes it is given, so the report (one PNG per
figure) and the viewer (one window) share them.
"""

from __future__ import annotations

import dataclasses
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.widgets import CheckButtons, Slider

from isss.config import SonarConfig
from isss.diagnostics import (
    HALF_CYCLE,
    PORT,
    SIDES,
    STARBOARD,
    PingArrays,
    detect_bottom,
    phase_histograms,
    sensor_frame_points,
    strong_mask,
)
from isss.protocol import expected_ping_interval, range_from_decimation

# Reference dataviz palette: categorical slots 1 and 2, text and surface tokens,
# and the blue ramp for sequential densities.
SIDE_COLORS = {PORT: "#2a78d6", STARBOARD: "#eb6834"}
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
DENSITY = LinearSegmentedColormap.from_list(
    "isss_density", [SURFACE, "#cde2fb", "#86b6ef", "#3987e5", "#256abf", "#184f95", "#0d366b"]
)
PHASE_CMAP = "twilight"  # cyclic: -10240 and +10240 counts are the same phase

STYLE = {
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": TEXT_SECONDARY,
    "axes.titlecolor": TEXT,
    "axes.titlesize": 11,
    "axes.labelsize": 9,
    "xtick.color": TEXT_SECONDARY,
    "ytick.color": TEXT_SECONDARY,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.grid": False,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
    "legend.fontsize": 8,
    "lines.linewidth": 1.2,
}


def _grid(ax) -> None:
    ax.grid(True, axis="both")
    ax.set_axisbelow(True)


# Waterfalls


def _mirrored(values: np.ndarray) -> np.ndarray:
    """(n, samples, 2) -> (n, 2 * samples): port reversed on the left, starboard on the right."""
    return np.concatenate([values[:, ::-1, PORT], values[:, :, STARBOARD]], axis=1)


def _waterfall_extent(arrays: PingArrays, config: SonarConfig) -> list[float]:
    r_max = float(arrays.slant_range(config)[-1]) if arrays.num_samples else 1.0
    return [-r_max, r_max, arrays.n_pings - 0.5, -0.5]


def draw_intensity_waterfall(ax, arrays: PingArrays, config: SonarConfig, *, percentile: float = 99.0):
    image = _mirrored(arrays.intensity.astype(np.float32))
    vmax = float(np.percentile(image, percentile)) if image.size else 1.0
    im = ax.imshow(image, aspect="auto", cmap="gray", vmin=0, vmax=max(vmax, 1.0),
                   extent=_waterfall_extent(arrays, config), interpolation="nearest")
    _waterfall_axes(ax, "Intensity")
    return im


def draw_phase_waterfall(ax, arrays: PingArrays, config: SonarConfig):
    im = ax.imshow(_mirrored(arrays.counts.astype(np.float32)), aspect="auto", cmap=PHASE_CMAP,
                   vmin=-HALF_CYCLE, vmax=HALF_CYCLE, extent=_waterfall_extent(arrays, config),
                   interpolation="nearest")
    _waterfall_axes(ax, "Phase counts (one cycle)")
    return im


def _waterfall_axes(ax, title: str) -> None:
    ax.set_title(title, loc="left")
    ax.set_xlabel("slant range [m]   ← port | starboard →")
    ax.set_ylabel("ping")
    ax.axvline(0, color=TEXT_SECONDARY, linewidth=0.6)


# Timing and phase statistics


def draw_timing(ax, arrays: PingArrays) -> None:
    if arrays.n_pings < 2:
        ax.text(0.5, 0.5, "fewer than two pings", ha="center", transform=ax.transAxes, color=TEXT_SECONDARY)
        return
    dt_ms = np.diff(arrays.ping_time.astype(np.int64)) * 0.1
    dec = int(np.bincount(arrays.decimation.astype(np.int64)).argmax())
    expected_ms = expected_ping_interval(range_from_decimation(dec, arrays.num_samples)) * 1e3
    ax.plot(np.arange(1, arrays.n_pings), dt_ms, color=SIDE_COLORS[PORT], linewidth=1.2)
    ax.axhline(expected_ms, color=TEXT_SECONDARY, linestyle="--", linewidth=1.0)
    ax.annotate(f"expected {expected_ms:.1f} ms", (1, expected_ms), xytext=(4, 4),
                textcoords="offset points", color=TEXT_SECONDARY, fontsize=8)
    ax.set_title("Ping interval", loc="left")
    ax.set_xlabel("ping")
    ax.set_ylabel("ms")
    ax.set_ylim(0, max(float(dt_ms.max()), expected_ms) * 1.15)
    _grid(ax)


def draw_phase_histograms(axes, arrays: PingArrays, *, min_excess: float = 25.0) -> None:
    """One panel per side: weak samples (water column, bias states) and strong returns."""
    hist = phase_histograms(arrays, min_excess=min_excess)
    centers = 0.5 * (hist["edges"][1:] + hist["edges"][:-1])
    for ax, (side, name) in zip(axes, enumerate(SIDES)):
        ax.step(centers, hist[f"{name}_weak"], where="mid", color=TEXT_SECONDARY, linewidth=1.0, label="weak")
        ax.step(centers, hist[f"{name}_strong"], where="mid", color=SIDE_COLORS[side], linewidth=1.4, label="strong")
        for x in (-HALF_CYCLE, HALF_CYCLE):
            ax.axvline(x, color=GRID, linewidth=1.0)
        ax.set_yscale("log")
        ax.set_title(f"{name.capitalize()} phase distribution", loc="left")
        ax.set_xlabel("phase counts (lines: ±10240, one cycle)")
        ax.set_ylabel("samples")
        ax.legend(loc="upper right")
        _grid(ax)


def draw_phase_vs_range(axes, arrays: PingArrays, config: SonarConfig, *, min_excess: float = 25.0) -> None:
    """Density of strong-return phase against slant range, one panel per side."""
    r = arrays.slant_range(config)
    strong = strong_mask(arrays, min_excess)
    rr = np.broadcast_to(r[None, :], arrays.intensity.shape[:2])
    for ax, (side, name) in zip(axes, enumerate(SIDES)):
        m = strong[:, :, side]
        H, xe, ye = np.histogram2d(rr[m], arrays.counts[:, :, side][m], bins=[120, 128],
                                   range=[[0, max(float(r[-1]), 1e-3)], [-32768, 32768]])
        ax.imshow(np.log1p(H.T), origin="lower", aspect="auto", cmap=DENSITY,
                  extent=[xe[0], xe[-1], ye[0], ye[-1]], interpolation="nearest")
        for y in (-HALF_CYCLE, HALF_CYCLE):
            ax.axhline(y, color=TEXT_SECONDARY, linewidth=0.6, linestyle=":")
        ax.set_title(f"{name.capitalize()}: phase of strong returns vs range", loc="left")
        ax.set_xlabel("slant range [m]")
        ax.set_ylabel("phase counts")


def _phase_signs(config: SonarConfig) -> dict[int, int]:
    return {PORT: config.port_phase_sign, STARBOARD: config.starboard_phase_sign}


def _draw_sector(ax, config: SonarConfig, reach: float) -> None:
    """Boresight rays and the edges of the unambiguous sector, both sides.

    Each edge is labelled with the sign of the counts that land there: with
    phase sign +1, positive counts are above the boresight (toward horizontal).
    """
    beta0 = math.radians(config.boresight_depression_deg)
    half = math.asin(min(1.0, HALF_CYCLE / config.counts_per_unit_sine))
    for side, sign in _phase_signs(config).items():
        x = -1.0 if side == PORT else 1.0
        for angle, style in ((beta0, "--"), (beta0 - half, ":"), (beta0 + half, ":")):
            ax.plot([0, x * reach * math.cos(angle)], [0, -reach * math.sin(angle)],
                    color=TEXT_SECONDARY, linestyle=style, linewidth=0.8)
        for angle, edge, at in ((beta0 - half, 1, 0.97), (beta0 + half, -1, 0.75)):
            ax.text(x * at * reach * math.cos(angle), -at * reach * math.sin(angle),
                    f" counts {'+' if edge * sign > 0 else '−'} ", fontsize=7, color=SIDE_COLORS[side],
                    ha="right" if side == PORT else "left", va="center")


def draw_sensor_profile(ax, arrays: PingArrays, config: SonarConfig, *, min_excess: float = 25.0) -> None:
    """All strong returns of all pings in the sensor frame. A flat seabed is a horizontal band."""
    pts = sensor_frame_points(arrays, config, min_excess=min_excess)
    reach = float(arrays.slant_range(config)[-1]) if arrays.num_samples else 1.0
    if pts["y"].size:
        H, xe, ye = np.histogram2d(pts["y"], pts["z"], bins=[200, 120],
                                   range=[[-reach, reach], [-reach, 0.0]])
        ax.imshow(np.log1p(H.T), origin="lower", aspect="equal", cmap=DENSITY,
                  extent=[xe[0], xe[-1], ye[0], ye[-1]], interpolation="nearest")
    _draw_sector(ax, config, reach)
    _profile_axes(ax, reach, _profile_title(config, "Strong returns in the sensor frame, all pings"))


def _profile_title(config: SonarConfig, title: str) -> str:
    flipped = [name for side, name in enumerate(SIDES) if _phase_signs(config)[side] < 0]
    return title + (f" ({' and '.join(flipped)} flipped)" if flipped else "")


def _profile_axes(ax, reach: float, title: str) -> None:
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, 0.05 * reach)
    ax.set_aspect("equal")
    ax.set_title(title, loc="left")
    ax.set_xlabel("across-track [m]   ← port | starboard →")
    ax.set_ylabel("height [m]")
    ax.text(0.99, 0.02, "dashed: boresight; dotted: ±42° unambiguous sector",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=7, color=TEXT_SECONDARY)


# Report figures


def report_figures(arrays: PingArrays, config: SonarConfig) -> dict[str, plt.Figure]:
    """The report's figures, keyed by file stem."""
    with plt.rc_context(STYLE):
        figs = {}
        fig, (a, b) = plt.subplots(1, 2, figsize=(12, 6), sharey=True)
        draw_intensity_waterfall(a, arrays, config)
        draw_phase_waterfall(b, arrays, config)
        b.set_ylabel("")
        fig.tight_layout()
        figs["waterfalls"] = fig

        fig, ax = plt.subplots(figsize=(8, 3))
        draw_timing(ax, arrays)
        fig.tight_layout()
        figs["timing"] = fig

        fig, axes = plt.subplots(1, 2, figsize=(12, 3.5), sharey=True)
        draw_phase_histograms(axes, arrays)
        axes[1].set_ylabel("")
        fig.tight_layout()
        figs["phase_histograms"] = fig

        fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
        draw_phase_vs_range(axes, arrays, config)
        axes[1].set_ylabel("")
        fig.tight_layout()
        figs["phase_vs_range"] = fig

        fig, ax = plt.subplots(figsize=(9, 5))
        draw_sensor_profile(ax, arrays, config)
        fig.tight_layout()
        figs["sensor_profile"] = fig
    return figs


# Interactive viewer


class PingViewer:
    """Waterfall on the left, the selected ping on the right.

    Click the waterfall or drag the slider to select a ping; arrow keys step
    (←/→ by 1, ↑/↓ by 10); ``p`` toggles intensity and phase waterfalls;
    ``f`` toggles following the newest ping when streaming; ``1``/``2`` (or
    the check boxes) flip the port/starboard phase sign, mirroring that side's
    returns about its boresight, as for a transducer mounted upside down.
    Hovering the profile reads out the angles and the counts for that point.
    """

    def __init__(self, arrays: PingArrays, config: SonarConfig, *, title: str = "ISSS", live: bool = False) -> None:
        self.config = dataclasses.replace(config)  # the flips change our copy only
        self.live = live
        self.arrays = arrays
        self.title = title
        self.follow = True
        self.show_phase = False
        self.index = max(arrays.n_pings - 1, 0)

        with plt.rc_context(STYLE):
            self.fig = plt.figure(figsize=(14, 8))
            gs = self.fig.add_gridspec(3, 2, width_ratios=[1.0, 1.25], height_ratios=[1.0, 1.0, 1.6],
                                       hspace=0.55, wspace=0.18,
                                       left=0.06, right=0.98, top=0.92, bottom=0.12)
            self.ax_wf = self.fig.add_subplot(gs[:, 0])
            self.ax_int = self.fig.add_subplot(gs[0, 1])
            self.ax_phase = self.fig.add_subplot(gs[1, 1], sharex=self.ax_int)
            self.ax_prof = self.fig.add_subplot(gs[2, 1])
            slider_ax = self.fig.add_axes([0.06, 0.03, 0.40, 0.03])
            self.slider = Slider(slider_ax, "ping", 0, max(arrays.n_pings - 1, 1), valinit=self.index,
                                 valstep=1, color=SIDE_COLORS[PORT])
            flip_ax = self.fig.add_axes([0.55, 0.005, 0.25, 0.07], frameon=False)
            self._flip_labels = {f"flip {name} (key {side + 1})": side for side, name in enumerate(SIDES)}
            self.flip_boxes = CheckButtons(flip_ax, list(self._flip_labels),
                                           [self._phase_sign(side) < 0 for side in (PORT, STARBOARD)])
            self._readout = self.fig.text(0.80, 0.075, "", fontsize=8, color=TEXT, va="top")
        self.slider.on_changed(lambda v: self._select(int(v), from_slider=True))
        self.flip_boxes.on_clicked(lambda label: self._set_flip(self._flip_labels[label]))
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._draw_waterfall()
        self._draw_ping()

    # Data

    def set_arrays(self, arrays: PingArrays) -> None:
        """Replace the data (live mode); keeps the selection unless following."""
        self.arrays = arrays
        self.slider.valmax = max(arrays.n_pings - 1, 1)
        self.slider.ax.set_xlim(self.slider.valmin, self.slider.valmax)
        if self.follow or self.index >= arrays.n_pings:
            self.index = max(arrays.n_pings - 1, 0)
        self._draw_waterfall()
        self._draw_ping()

    # Drawing

    def _draw_waterfall(self) -> None:
        with plt.rc_context(STYLE):
            self.ax_wf.clear()
            if self.arrays.n_pings == 0:
                self.ax_wf.text(0.5, 0.5, "waiting for pings", ha="center", transform=self.ax_wf.transAxes)
                return
            draw = draw_phase_waterfall if self.show_phase else draw_intensity_waterfall
            draw(self.ax_wf, self.arrays, self.config)
            self._marker = self.ax_wf.axhline(self.index, color=SIDE_COLORS[STARBOARD], linewidth=1.0)

    def _draw_ping(self) -> None:
        a, i = self.arrays, self.index
        with plt.rc_context(STYLE):
            for ax in (self.ax_int, self.ax_phase, self.ax_prof):
                ax.clear()
            if a.n_pings == 0:
                self.fig.canvas.draw_idle()
                return
            r = a.slant_range(self.config)
            one = a.select(slice(i, i + 1))
            strong = strong_mask(one)[0]
            bottom = detect_bottom(one)[0]
            for side, name in enumerate(SIDES):
                color = SIDE_COLORS[side]
                self.ax_int.plot(r, a.intensity[i, :, side], color=color, label=name)
                self.ax_phase.scatter(r, a.counts[i, :, side], s=4, color=color, alpha=0.35, linewidths=0)
                self.ax_phase.scatter(r[strong[:, side]], a.counts[i, strong[:, side], side], s=9,
                                      color=color, linewidths=0, label=f"{name}, strong")
                if bottom[side] >= 0:
                    self.ax_int.axvline(r[bottom[side]], color=color, linewidth=0.8, linestyle=":")
            self.ax_int.set_title("Intensity (dotted: first sustained return)", loc="left")
            self.ax_int.set_ylabel("RSS")
            self.ax_int.set_ylim(0, 260)
            self.ax_int.legend(loc="upper right", ncols=2)
            _grid(self.ax_int)
            self.ax_phase.set_title("Phase counts (faint: weak samples)", loc="left")
            self.ax_phase.set_xlabel("slant range [m]")
            self.ax_phase.set_ylabel("counts")
            self.ax_phase.set_ylim(-32768, 32768)
            for y in (-HALF_CYCLE, HALF_CYCLE):
                self.ax_phase.axhline(y, color=GRID, linewidth=1.0)
            _grid(self.ax_phase)

            pts = sensor_frame_points(one, self.config)
            reach = float(r[-1]) if r.size else 1.0
            for side in (PORT, STARBOARD):
                m = pts["side"] == side
                self.ax_prof.scatter(pts["y"][m], pts["z"][m], s=6 + pts["intensity"][m] / 25.0,
                                     color=SIDE_COLORS[side], linewidths=0, alpha=0.8)
            _draw_sector(self.ax_prof, self.config, reach)
            _profile_axes(self.ax_prof, reach, _profile_title(self.config, "This ping in the sensor frame (strong returns)"))

            t = a.ping_time[i] * 1e-4
            self.fig.suptitle(f"{self.title}   ping {i + 1}/{a.n_pings}   device time {t:.3f} s"
                              f"{'   [following]' if self.follow and self.live else ''}",
                              x=0.06, ha="left", color=TEXT, fontsize=12)
            if hasattr(self, "_marker"):
                self._marker.set_ydata([i, i])
        if int(self.slider.val) != i:
            self.slider.eventson = False
            self.slider.set_val(i)
            self.slider.eventson = True
        self.fig.canvas.draw_idle()

    # Phase sign

    def _phase_sign(self, side: int) -> int:
        return _phase_signs(self.config)[side]

    def flip(self, side: int) -> None:
        """Invert one side's phase sign and redraw."""
        field = "port_phase_sign" if side == PORT else "starboard_phase_sign"
        setattr(self.config, field, -getattr(self.config, field))
        if self.flip_boxes.get_status()[side] != (self._phase_sign(side) < 0):
            self.flip_boxes.eventson = False
            self.flip_boxes.set_active(side)
            self.flip_boxes.eventson = True
        self._draw_ping()

    def _set_flip(self, side: int) -> None:
        # The box was clicked: it already shows the new state, so follow it.
        if self.flip_boxes.get_status()[side] != (self._phase_sign(side) < 0):
            self.flip(side)

    def angles_at(self, y: float, z: float) -> dict[str, float]:
        """Depression, off-boresight angle and expected counts of a point in the sensor frame."""
        side = STARBOARD if y >= 0 else PORT
        beta = math.atan2(-z, abs(y))
        alpha = math.radians(self.config.boresight_depression_deg) - beta
        counts = self._phase_sign(side) * self.config.counts_per_unit_sine * math.sin(alpha)
        aliased = not -HALF_CYCLE <= counts < HALF_CYCLE
        wrapped = (counts + HALF_CYCLE) % (2 * HALF_CYCLE) - HALF_CYCLE if aliased else counts
        return {"side": side, "range": math.hypot(y, z), "depression_deg": math.degrees(beta),
                "off_boresight_deg": math.degrees(alpha), "counts": wrapped, "aliased": aliased}

    def _on_motion(self, event) -> None:
        text = ""
        if event.inaxes is self.ax_prof and event.xdata is not None and event.ydata < 0:
            a = self.angles_at(event.xdata, event.ydata)
            text = (f"{SIDES[a['side']]}, r {a['range']:.1f} m, depression {a['depression_deg']:.1f}°\n"
                    f"off boresight {a['off_boresight_deg']:+.1f}°, counts {a['counts']:+.0f}"
                    f"{' (aliased)' if a['aliased'] else ''}")
        if text != self._readout.get_text():
            self._readout.set_text(text)
            self.fig.canvas.draw_idle()

    # Interaction

    def _select(self, index: int, *, from_slider: bool = False) -> None:
        if self.arrays.n_pings == 0:
            return
        index = int(np.clip(index, 0, self.arrays.n_pings - 1))
        if from_slider or index != self.arrays.n_pings - 1:
            self.follow = False
        self.index = index
        self._draw_ping()

    def _on_click(self, event) -> None:
        if event.inaxes is self.ax_wf and event.ydata is not None:
            self._select(int(round(event.ydata)))

    def _on_key(self, event) -> None:
        step = {"right": 1, "left": -1, "down": 10, "up": -10}.get(event.key)
        if step:
            self._select(self.index + step)
        elif event.key == "p":
            self.show_phase = not self.show_phase
            self._draw_waterfall()
            self._draw_ping()
        elif event.key in ("1", "2"):
            self.flip(PORT if event.key == "1" else STARBOARD)
        elif event.key == "f":
            self.follow = not self.follow
            if self.follow:
                self.index = max(self.arrays.n_pings - 1, 0)
            self._draw_ping()
