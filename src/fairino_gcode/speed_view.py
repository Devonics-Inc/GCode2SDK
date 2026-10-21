#!/usr/bin/env python3
"""
speed_view.py - the speed panel under the 3D view: the TCP speed as the robot reports it, against the speed
it should hold. Numbers beside the graph on a wide window, above it on a narrow one.

    graph       speed (mm/s) over time; dashed line = reference, shaded = the +/- tolerance band,
                red = moving outside the band; after a run, a dot where the TCP went through each target
                (orange = blended corner, grey = an exact stop)
    numbers     now, reference, mean +/- standard deviation, steadiness (std as % of the mean), share
                of the moving time inside the band, min / max, how often it left the band, and one line
                about the targets
    controls    reference (automatic = median while moving, or typed in), tolerance, clear

The numbers come from speed_monitor.py. Nothing here talks to the robot.
"""
from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.qt_compat import QtCore, QtWidgets
from matplotlib.figure import Figure

from .speed_monitor import RAPID, SpeedRecorder, SpeedStats, TargetSpeed, plan_targets, target_summary

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal
Qt = QtCore.Qt

GREEN, AMBER, RED, GREY, BLUE, ORANGE = "#1a9850", "#e08a00", "#d62728", "#8a8f98", "#1f6fd0", "#e8710a"
MAX_DRAWN = 6000                 # points drawn at most (the statistics always use all of them)


class SpeedView(QtWidgets.QWidget):
    targets_changed = Signal(object)        # {command index: TargetSpeed} after analyse(), {} after clear()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorder = SpeedRecorder()
        self.targets: Dict[int, TargetSpeed] = {}
        self._plan = None
        self._dirty = True

        # ---- numbers
        self.values: Dict[str, QtWidgets.QLabel] = {}
        self._cells = []                         # (caption, value) labels, put in the grid by _arrange()
        self._grid = grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(0)
        cells = [("now", "Now"), ("ref", "Reference"), ("mean", "Mean ± std"), ("cv", "Steadiness"),
                 ("within", "Inside band"), ("range", "Min / max"), ("left", "Left the band"),
                 ("moving", "Moving time")]
        for key, caption in cells:
            head = QtWidgets.QLabel(caption)
            head.setStyleSheet(f"color: {GREY};")
            value = QtWidgets.QLabel("-")
            value.setStyleSheet("font-weight: 600;")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            self._cells.append((head, value))
            self.values[key] = value
        self.values["cv"].setToolTip("Standard deviation as a % of the mean speed: 0 % = perfectly steady.")
        self.values["within"].setToolTip("Share of the time spent moving with the speed inside the band.")
        self.values["left"].setToolTip("How many times the speed went outside the band while moving "
                                       "(corners, accelerating, slowing down for a stop).")
        self.target_line = QtWidgets.QLabel("Targets: run the plan to see the speed at each target point.")
        self.target_line.setWordWrap(True)
        self.target_line.setStyleSheet(f"color: {GREY};")

        # ---- graph
        self.fig = Figure(figsize=(5, 2.2), constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.canvas.setMinimumHeight(120)
        self.canvas.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
        self.ax = self.fig.add_subplot()
        self.ax.set_xlabel("time (s)", fontsize=8)
        self.ax.set_ylabel("TCP speed (mm/s)", fontsize=8)
        self.ax.tick_params(labelsize=7)
        self.ax.grid(True, alpha=0.3)
        (self._rapid,) = self.ax.plot([], [], color=GREY, linewidth=0.9, alpha=0.7)
        (self._line,) = self.ax.plot([], [], color=BLUE, linewidth=1.1)
        (self._out,) = self.ax.plot([], [], color=RED, linewidth=1.4)
        (self._ref,) = self.ax.plot([], [], color="#444444", linestyle="--", linewidth=0.9)
        (self._corners,) = self.ax.plot([], [], "o", color=ORANGE, ms=4.5)
        (self._stops,) = self.ax.plot([], [], "v", color=GREY, ms=4)
        self._band = None
        # the key is a line of text under the graph: a legend inside a small graph hides the curve
        self.key = QtWidgets.QLabel(
            f'<span style="color:{BLUE}">━</span> speed &nbsp; '
            f'<span style="color:{RED}">━</span> outside the band &nbsp; '
            '<span style="color:#444444">╌</span> reference &nbsp; '
            f'<span style="color:{GREEN}">█</span> band &nbsp; '
            f'<span style="color:{ORANGE}">●</span> through a target &nbsp; '
            f'<span style="color:{GREY}">▼</span> exact stop &nbsp; '
            f'<span style="color:{GREY}">━</span> G0 rapid (not counted)')
        self.key.setTextFormat(Qt.TextFormat.RichText)
        self.key.setWordWrap(True)
        self.key.setStyleSheet("font-size: 8pt;")

        # ---- controls
        self.reference = QtWidgets.QDoubleSpinBox()
        self.reference.setRange(0.0, 5000.0)
        self.reference.setDecimals(1)
        self.reference.setSingleStep(5.0)
        self.reference.setSuffix(" mm/s")
        self.reference.setSpecialValueText("auto")
        self.reference.setMinimumWidth(110)
        self.reference.setToolTip("The speed the TCP should hold. Auto: the median of the speeds while moving.")
        self.tolerance = QtWidgets.QDoubleSpinBox()
        self.tolerance.setRange(1.0, 50.0)
        self.tolerance.setDecimals(0)
        self.tolerance.setValue(10.0)
        self.tolerance.setPrefix("± ")
        self.tolerance.setSuffix(" %")
        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.setToolTip("Forget the recorded speeds (Start does this too).")
        self.reference.valueChanged.connect(self._settings_changed)
        self.tolerance.valueChanged.connect(self._settings_changed)
        self.clear_btn.clicked.connect(self.clear)
        # one row above the graph, two short rows beside it (_arrange)
        self._ref_label, self._band_label = QtWidgets.QLabel("Reference"), QtWidgets.QLabel("Band")
        self._controls = controls = QtWidgets.QGridLayout()
        controls.setContentsMargins(0, 0, 0, 0)

        # numbers beside the graph when there is width for it (_arrange), above it when there is not
        self.numbers_box = QtWidgets.QWidget()
        nbox = QtWidgets.QVBoxLayout(self.numbers_box)
        nbox.setContentsMargins(0, 0, 0, 0)
        nbox.addLayout(controls)
        nbox.addLayout(grid)
        nbox.addWidget(self.target_line)
        nbox.addStretch(1)
        graph_box = QtWidgets.QWidget()
        gbox = QtWidgets.QVBoxLayout(graph_box)
        gbox.setContentsMargins(0, 0, 0, 0)
        gbox.addWidget(self.canvas, 1)
        gbox.addWidget(self.key)
        # inside a scroll area: on a very small window it scrolls instead of squeezing things together
        content = QtWidgets.QWidget()
        self._body = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.Direction.TopToBottom, content)
        self._body.setContentsMargins(4, 4, 4, 4)
        self._body.addWidget(self.numbers_box)
        self._body.addWidget(graph_box, 1)
        self._wide: Optional[bool] = None
        self._arrange()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setWidget(content)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._timer = QtCore.QTimer(self)        # samples arrive 20 times a second; 4 updates are plenty
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def _tick(self):
        # the layout follows the width here, not in a resizeEvent override: Qt can send resize events
        # while it tears the window down, and Python code run then crashes (seen in the test suite).
        # This timer belongs to the panel, so it stops with it.
        self._arrange()
        self.refresh()

    # ---- input
    def add_sample(self, sample: dict):
        if self.recorder.add_sample(sample, time.monotonic()):
            self._dirty = True

    def mark(self, phase: str, t: Optional[float] = None):
        """The robot starts a FEED move or a RAPID at time.monotonic() t (default: now)."""
        self.recorder.mark(time.monotonic() if t is None else t, phase)
        self._dirty = True

    def clear(self):
        self.recorder.clear()
        self._set_targets({}, None)
        self._dirty = True
        self.refresh()

    def custom_reference(self) -> Optional[float]:
        return self.reference.value() or None

    def analyse(self, plan):
        """Work out the speed through every target of `plan` (placed on the robot) from what was recorded."""
        self._set_targets(self._targets_for(plan), plan)
        self._dirty = True
        self.refresh()

    def _targets_for(self, plan) -> Dict[int, TargetSpeed]:
        if plan is None or not len(self.recorder):
            return {}
        ref, _auto = self.recorder.reference(self.custom_reference())
        return self.recorder.at_targets(plan_targets(plan), ref)

    def _set_targets(self, found: Dict[int, TargetSpeed], plan):
        self.targets, self._plan = found, plan
        if plan is None:
            self.target_line.setText("Targets: run the plan to see the speed at each target point.")
        else:
            self.target_line.setText("Targets: " + target_summary(found, len(plan.motions), self.tolerance.value()))
        self.targets_changed.emit(found)

    def _settings_changed(self, _value=None):
        if self._plan is not None:
            self._set_targets(self._targets_for(self._plan), self._plan)
        self._dirty = True
        self.refresh()

    # ---- output
    def stats(self) -> SpeedStats:
        return self.recorder.stats(self.custom_reference(), self.tolerance.value())

    def refresh(self):
        """Numbers and graph, only while someone can see them (the samples are kept either way)."""
        if not self._dirty or not self.isVisible():
            return
        s = self.stats()
        self._show_numbers(s)
        self._draw(s)
        self._dirty = False
        # a long recording costs more to summarise: then once a second is enough
        self._timer.setInterval(250 if len(self.recorder) < 20000 else 1000)

    def _show_numbers(self, s: SpeedStats):
        def mm_s(v):
            return "-" if v is None else f"{v:.1f} mm/s"

        v = self.values
        v["now"].setText(mm_s(s.now))
        v["ref"].setText("-" if s.reference is None else f"{s.reference:.1f} mm/s" + (" (auto)" if s.auto else ""))
        v["mean"].setText("-" if s.mean is None else f"{s.mean:.1f} ± {s.std:.1f} mm/s")
        v["cv"].setText("-" if s.cv is None else f"{s.cv:.1f} %")
        v["range"].setText("-" if s.low is None else f"{s.low:.1f} / {s.high:.1f}")
        v["left"].setText(str(s.excursions) if s.within is not None else "-")
        v["moving"].setText(f"{s.moving_s:.1f} s")
        if s.within is None:
            v["within"].setText("-")
            v["within"].setStyleSheet("font-weight: 600;")
        else:
            colour = GREEN if s.within >= 90 else AMBER if s.within >= 70 else RED
            v["within"].setText(f"{s.within:.0f} % (±{s.tolerance:g} %)")
            v["within"].setStyleSheet(f"font-weight: 600; color: {colour};")

    def _draw(self, s: SpeedStats):
        t, v = np.asarray(self.recorder.t, dtype=float), np.asarray(self.recorder.v, dtype=float)
        rapid = np.array([p == RAPID for p in self.recorder.phases()], dtype=bool)
        step = max(1, len(t) // MAX_DRAWN)
        td, vd, rd = t[::step], v[::step], rapid[::step]
        self._rapid.set_data(td, np.where(rd, vd, np.nan))
        vd = np.where(rd, np.nan, vd)                    # from here on: the counted (feed) samples only
        self._line.set_data(td, vd)
        if self._band is not None:
            self._band.remove()
            self._band = None
        if s.reference and len(t):
            ref, band = s.reference, s.reference * s.tolerance / 100.0
            self._ref.set_data([td[0], td[-1]], [ref, ref])
            self._band = self.ax.axhspan(ref - band, ref + band, color=GREEN, alpha=0.12, lw=0)
            outside = np.where((vd > 1.0) & (np.abs(vd - ref) > band), vd, np.nan)
            self._out.set_data(td, outside)
        else:
            self._ref.set_data([], [])
            self._out.set_data([], [])
        corners = [(x.t, x.speed) for x in self.targets.values() if not x.stop]
        stops = [(x.t, x.speed) for x in self.targets.values() if x.stop]
        self._corners.set_data(*(zip(*corners) if corners else ([], [])))
        self._stops.set_data(*(zip(*stops) if stops else ([], [])))
        # scaled to the feed moves: a fast rapid may run off the top rather than flatten the curve that matters
        feed_max = float(np.nanmax(vd)) if np.isfinite(vd).any() else 0.0
        top = max([1.0, feed_max] + ([s.reference * 1.3] if s.reference else []))
        self.ax.set_xlim(0.0, max(1.0, float(td[-1]) if len(td) else 1.0))
        self.ax.set_ylim(0.0, top * 1.08)
        self.canvas.draw_idle()

    # ---- layout: follows the space the panel gets
    WIDE = 480                                   # px from which the numbers go beside the graph

    def _arrange(self):
        wide = self.width() >= self.WIDE
        if wide:                                 # the graph keeps at least about half the width
            self.numbers_box.setMaximumWidth(max(220, min(320, int(self.width() * 0.45))))
        if wide == self._wide:
            return
        self._wide = wide
        cols = 2 if wide else 4                  # beside the graph: a narrow column of numbers
        for head, value in self._cells:
            self._grid.removeWidget(head)
            self._grid.removeWidget(value)
        for n, (head, value) in enumerate(self._cells):
            row, col = divmod(n, cols)
            self._grid.addWidget(head, 2 * row, col)
            self._grid.addWidget(value, 2 * row + 1, col)
        c = self._controls
        for w in (self._ref_label, self.reference, self._band_label, self.tolerance, self.clear_btn):
            c.removeWidget(w)
        if wide:                                 # Reference [ ]  /  Band [ ] Clear
            c.addWidget(self._ref_label, 0, 0)
            c.addWidget(self.reference, 0, 1, 1, 2)
            c.addWidget(self._band_label, 1, 0)
            c.addWidget(self.tolerance, 1, 1)
            c.addWidget(self.clear_btn, 1, 2)
        else:                                    # Reference [ ]  Band [ ]       Clear
            for col, w in enumerate((self._ref_label, self.reference, self._band_label, self.tolerance)):
                c.addWidget(w, 0, col)
            c.addWidget(self.clear_btn, 0, 5)
        for col in range(6):
            c.setColumnStretch(col, 1 if (col == 4 and not wide) else 0)
        self._body.setDirection(QtWidgets.QBoxLayout.Direction.LeftToRight if wide
                                else QtWidgets.QBoxLayout.Direction.TopToBottom)
        if not wide:
            self.numbers_box.setMaximumWidth(16777215)

    def showEvent(self, event):
        super().showEvent(event)
        self._dirty = True                       # draw what came in while it was hidden
        self.refresh()
