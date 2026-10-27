#!/usr/bin/env python3
"""
speed_view.py - the speed panel under the 3D view: the TCP speed as the robot reports it, against the speed
it should hold. Nothing to set: the reference is automatic and the band is fixed at +/- BAND_PCT %.

    line        how much of the moving time the speed stayed inside the band, and the reference; Clear
    graph       speed (mm/s) over time; dashed line = reference (the commanded TCP speed in mm/s mode, else
                the median while moving), shaded = the band, red = moving outside it; after a run, a dot
                where the TCP went through each target (orange = blended corner, grey = an exact stop)
    per command after a run, the speed along every command (speed_monitor.command_speeds); selecting a
                row of the command table shades that command's time on the graph (highlight())

The numbers come from speed_monitor.py. Nothing here talks to the robot.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.qt_compat import QtCore, QtWidgets
from matplotlib.figure import Figure

from .speed_monitor import (RAPID, CommandSpeed, SpeedRecorder, SpeedStats, TargetSpeed, command_speeds,
                            plan_targets, write_command_csv, write_samples_csv)

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal
Qt = QtCore.Qt

GREEN, AMBER, RED, GREY, BLUE, ORANGE = "#1a9850", "#e08a00", "#d62728", "#8a8f98", "#1f6fd0", "#e8710a"
MAX_DRAWN = 6000                 # points drawn at most (the statistics always use all of them)
BAND_PCT = 15.0                  # the speed counts as on target within +/- this % of the reference


class SpeedView(QtWidgets.QWidget):
    targets_changed = Signal(object)        # {command index: TargetSpeed} after analyse(), {} after clear()
    commands_changed = Signal(object)       # {command index: CommandSpeed} after analyse(), {} after clear()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorder = SpeedRecorder()
        self.targets: Dict[int, TargetSpeed] = {}
        self.commands: Dict[int, CommandSpeed] = {}
        self._owner: List[Optional[int]] = []           # command of every sample (after analyse())
        self._plan = None
        self._dirty = True
        self._span = None                               # the shaded time of the selected command

        self.band = BAND_PCT

        # ---- one line above the graph: how steady, and Clear
        self.status = QtWidgets.QLabel("Run the plan to see how steady the TCP speed is.")
        self.status.setTextFormat(Qt.TextFormat.RichText)
        self.status.setWordWrap(True)
        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.setToolTip("Forget the recorded speeds (Start does this too).")
        self.clear_btn.clicked.connect(self.clear)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.status, 1)
        top.addWidget(self.clear_btn)

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
            f'<span style="color:#444444">╌</span> reference &nbsp; '
            f'<span style="color:{GREEN}">█</span> ±{BAND_PCT:g} % band &nbsp; '
            f'<span style="color:{ORANGE}">●</span> through a target &nbsp; '
            f'<span style="color:{GREY}">▼</span> exact stop &nbsp; '
            f'<span style="color:{GREY}">━</span> G0 rapid (not counted)')
        self.key.setTextFormat(Qt.TextFormat.RichText)
        self.key.setWordWrap(True)
        self.key.setStyleSheet("font-size: 8pt;")

        content = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(content)
        body.setContentsMargins(4, 4, 4, 4)
        body.addLayout(top)
        body.addWidget(self.canvas, 1)
        body.addWidget(self.key)
        # inside a scroll area: on a very small window it scrolls instead of squeezing things together
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setWidget(content)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._timer = QtCore.QTimer(self)        # samples arrive 20 times a second; 4 updates are plenty
        self._timer.setInterval(250)
        self._timer.timeout.connect(self.refresh)
        self._timer.start()

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
        self._set_commands({}, [])
        self._dirty = True
        self.refresh()

    def set_commanded(self, speed: Optional[float]):
        """The TCP speed the job commands (mm/s mode), or None (% mode): "auto" compares with it."""
        self.recorder.commanded = speed or None
        self._dirty = True

    def analyse(self, plan):
        """Work out the speed through every target and along every command of `plan` (placed on the robot)
        from what was recorded."""
        self._set_targets(self._targets_for(plan), plan)
        self._set_commands(*self._commands_for(plan))
        self._dirty = True
        self.refresh()

    def _commands_for(self, plan):
        if plan is None or not len(self.recorder):
            return {}, []
        ref, _auto = self.recorder.reference()
        return command_speeds(self.recorder, plan, ref, self.band)

    def _set_commands(self, found: Dict[int, CommandSpeed], owner):
        self.commands, self._owner = found, list(owner)
        self.highlight(None)
        self.commands_changed.emit(found)

    def highlight(self, index: Optional[int]):
        """Shade the time the arm spent on command `index` (None: no shading)."""
        if self._span is not None:
            self._span.remove()
            self._span = None
        c = self.commands.get(index) if index is not None else None
        if c is not None:
            # at least one sample period wide, so a command with a single sample still shows
            half = 0.5 * max(c.seconds, 0.05)
            middle = 0.5 * (c.t0 + c.t1)
            self._span = self.ax.axvspan(middle - half, middle + half, color=ORANGE, alpha=0.18, lw=0)
        self.canvas.draw_idle()

    # ---- export
    def export(self, path: str) -> List[str]:
        """Write the speed per command to `path` (after a run) and every sample to <path>_samples.csv."""
        if not path.lower().endswith(".csv"):
            path += ".csv"
        samples = path[:-4] + "_samples.csv"
        written = []
        if self._plan is not None:
            write_command_csv(path, self._plan, self.commands)
            written.append(path)
        else:
            samples = path
        # samples recorded after the run was analysed (the robot stays connected) belong to no command;
        # the ones before keep the command they were matched to
        n = len(self.recorder)
        owner = (list(self._owner) + [None] * n)[:n]
        write_samples_csv(samples, self.recorder, owner, self._plan)
        written.append(samples)
        return written

    def _targets_for(self, plan) -> Dict[int, TargetSpeed]:
        if plan is None or not len(self.recorder):
            return {}
        ref, _auto = self.recorder.reference()
        return self.recorder.at_targets(plan_targets(plan), ref)

    def _set_targets(self, found: Dict[int, TargetSpeed], plan):
        self.targets, self._plan = found, plan
        self.targets_changed.emit(found)

    # ---- output
    def stats(self) -> SpeedStats:
        return self.recorder.stats(None, self.band)

    def refresh(self):
        """Numbers and graph, only while someone can see them (the samples are kept either way)."""
        if not self._dirty or not self.isVisible():
            return
        s = self.stats()
        self._show_status(s)
        self._draw(s)
        self._dirty = False
        # a long recording costs more to summarise: then once a second is enough
        self._timer.setInterval(250 if len(self.recorder) < 20000 else 1000)

    def _show_status(self, s: SpeedStats):
        if s.within is None or s.reference is None:
            self.status.setText("Run the plan to see how steady the TCP speed is.")
            return
        colour = GREEN if s.within >= 90 else AMBER if s.within >= 70 else RED
        self.status.setText(f'<b style="color:{colour}">{s.within:.0f} %</b> of the moving time within '
                            f"±{s.tolerance:g} % of {s.reference:.1f} mm/s")

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

    def showEvent(self, event):
        super().showEvent(event)
        self._dirty = True                       # draw what came in while it was hidden
        self.refresh()
