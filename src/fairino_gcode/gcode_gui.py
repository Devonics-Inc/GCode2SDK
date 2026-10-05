#!/usr/bin/env python3
"""
gcode_gui.py - graphical front end for gcode_to_fairino.py.

Step 1 - plan viewer (no robot needed):
    * open a G-code file (button, Ctrl+O, drag and drop, or as command-line argument)
    * 3D view of the planned TCP path in X, Y, Z (mm), coloured by robot instruction
    * the list of commands (G-code line -> robot instruction, target, speed); selecting a row
      highlights that move in the 3D view
    * the plan summary and parser warnings

Step 2 - live robot (read-only, nothing here moves the robot):
    * Connect... asks for the robot's IP address and listens to its CNDE status stream
      (fairino_CNDE_listener.py, used through robot_monitor.py)
    * the current TCP is drawn as a red dot in the same 3D view, with the trail it leaves
    * the Robot panel shows the TCP coordinates and the robot's state and fault codes

Step 3 - run the plan on the robot:
    * Set up run... says where the drawing sits on the robot (reference teach point, the current TCP,
      or typed coordinates) and how fast it runs. Accepting it checks the plan: the robot is only read,
      the plan is redrawn where it will really run, and every target is tested for reachability
    * Start sends the checked plan (after a confirmation); the table and the 3D view follow the
      command being executed, the Log tab shows every command sent
    * Stop ends the run and stops the arm within milliseconds
    * --simulate runs all of this against a simulated robot (sim_robot.py) instead of a real one

The plan is built by the same code the command-line tool runs (parse_gcode -> plan), so what is
drawn here is what would be sent to the robot.

Needs matplotlib and one Qt binding (PyQt5, PySide6 or PyQt6):
    python gcode_gui.py samples/demo_part.gcode
    python gcode_gui.py --simulate samples/demo_part.gcode
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from collections import deque
from dataclasses import replace
from typing import Dict, Optional

# The application ships with PySide6 (LGPL). If several Qt bindings are installed, use that one;
# without it, matplotlib falls back to whichever binding is there (for example PyQt5).
try:
    import PySide6  # noqa: F401
    os.environ.setdefault("QT_API", "pyside6")
except ImportError:
    pass

import matplotlib

matplotlib.use("QtAgg")
import numpy as np  # noqa: E402
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg  # noqa: E402
from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402

from . import APP_NAME, __version__  # noqa: E402
from . import gcode_to_fairino as eng  # noqa: E402
from .plan_model import Plan, build_plan, default_settings  # noqa: E402
from .robot_monitor import TCP, RobotMonitor, Target  # noqa: E402
from .robot_widgets import ConnectDialog, RobotStatusPanel  # noqa: E402
from .run_control import AUTO, HERE, MANUAL, Job, JobResult, RobotSession, RunSetup, RunSetupDialog  # noqa: E402

Qt = QtCore.Qt

# How each kind of move is drawn. Rapids (G0) are dashed: the arm gets there with a joint move,
# so the straight line only shows where it goes, not the exact path it takes.
STYLES = {
    "rapid": dict(color="#8a8f98", linestyle=(0, (4, 3)), linewidth=1.0, label="G0 rapid"),
    "MoveL": dict(color="#1f6fd0", linestyle="-", linewidth=1.6, label="G1  \u2192 MoveL"),
    "MoveC": dict(color="#e8710a", linestyle="-", linewidth=1.6, label="G2/G3 arc \u2192 MoveC"),
    "Circle": dict(color="#1a9850", linestyle="-", linewidth=1.6, label="G2/G3 full circle \u2192 Circle"),
}
HIGHLIGHT = "#d81b60"
TCP_COLOUR, TCP_OLD_COLOUR = "#e53935", "#8a8f98"     # live robot TCP / last known when data stopped
TRAIL_POINTS, TRAIL_STEP_MM = 3000, 0.2               # how much of the travelled path is kept
VIEWS = {"Iso": (28, -58), "Top (XY)": (90, -90), "Front (XZ)": (0, -90), "Side (YZ)": (0, 0)}


def style_key(cmd: eng.Cmd) -> str:
    return "rapid" if cmd.rapid else cmd.instr


# ============================================================================ 3D view
class PlanCanvas(FigureCanvasQTAgg):
    """3D view of the planned TCP path. Left-drag rotates, right-drag or wheel zooms."""

    def __init__(self, parent=None):
        self.fig = Figure()
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax = self.fig.add_subplot(projection="3d")
        self.fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        self._plan: Optional[Plan] = None
        self._centre = np.zeros(3)
        self._half = 50.0
        self._hi_line = None
        self._hi_pts = None
        self._cloud = np.zeros((0, 3))            # every drawn plan point (for fitting the view)
        self._tcp: Optional[np.ndarray] = None    # latest robot TCP, None = not connected
        self._tcp_live = True
        self._trail: deque = deque(maxlen=TRAIL_POINTS)
        self._live: list = []
        self._bg = None                           # saved picture of everything that does not move
        self.mpl_connect("scroll_event", self._on_scroll)
        self.mpl_connect("draw_event", self._on_draw)
        self._empty()

    # ---- drawing
    def _empty(self):
        self.ax.clear()
        self._label_axes()
        self.ax.text2D(0.5, 0.96, "Open a G-code file to see the plan", transform=self.ax.transAxes,
                       ha="center", va="top", color="#8a8f98")
        self._add_live_artists()
        self.refit()
        self._look("Iso")
        self._redraw()

    def _redraw(self):
        """Full redraw (slow, 3D). The saved background is dropped until it has been painted again."""
        self._bg = None
        self.draw_idle()

    def _label_axes(self):
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")

    def show_plan(self, plan: Plan, keep_view: bool = False):
        view = (self.ax.elev, self.ax.azim)
        self._plan = plan
        self.ax.clear()
        self._label_axes()

        # one collection per kind of move, as plain 2-point segments (fast, and works on old matplotlib)
        groups: Dict[str, list] = {}
        for k in plan.motions:
            p = plan.paths[k]
            if len(p) > 1:
                groups.setdefault(style_key(plan.cmds[k]), []).append(np.stack([p[:-1], p[1:]], axis=1))
        for key, style in STYLES.items():
            if key in groups:
                st = dict(style)
                self.ax.add_collection3d(Line3DCollection(
                    np.concatenate(groups[key]), colors=st.pop("color"), linestyles=[st.pop("linestyle")],
                    linewidths=st.pop("linewidth"), **st))

        pts = [plan.paths[k] for k in plan.motions]
        if pts:
            start, end = pts[0][-1], pts[-1][-1]
            self.ax.plot(*[[v] for v in start], marker="o", ms=7, color="#1a9850", ls="none", label="first point")
            self.ax.plot(*[[v] for v in end], marker="s", ms=6, color="#222222", ls="none", label="last point")
        self._draw_origin(plan, pts)
        self._cloud = np.vstack(pts + [self._origin[None, :]])
        if plan.unreachable:
            self.ax.plot(*np.array(plan.unreachable).T, marker="x", ms=9, mew=2.2, color="#d62728", ls="none",
                         label="cannot be reached")
        if plan.placed:
            where = f"workpiece {plan.settings.user}" if plan.settings.user else "base"
            note, colour = f"Placed on the robot: tool {plan.settings.tool}, {where} frame", "#1a9850"
        else:
            note, colour = "Not placed on the robot yet - drawn at the G-code's own coordinates", "#8a8f98"
        self.ax.text2D(0.5, 0.985, note, transform=self.ax.transAxes, ha="center", va="top",
                       fontsize=8.5, color=colour)

        handles, labels = self.ax.get_legend_handles_labels()
        handles.append(Line2D([], [], marker="o", ms=8, color=TCP_COLOUR, mec="white", ls="none"))
        labels.append("robot TCP (live)")
        self.ax.legend(handles, labels, loc="upper left", fontsize=8, frameon=True, framealpha=0.9)
        self._add_live_artists()
        self.refit()
        if keep_view:
            self.ax.view_init(*view)
        else:
            self._look("Iso")
        self._redraw()

    def _draw_origin(self, plan: Plan, pts):
        """Small X/Y/Z triad at G-code (0, 0, 0), so the orientation of the drawing is visible."""
        span = max((np.ptp(np.vstack(pts), axis=0).max() if pts else 0.0), 10.0)
        o = np.array(plan.frame.pose((0.0, 0.0, 0.0))[:3])
        length = 0.12 * span / max(abs(plan.frame.scale), 1e-9)
        for axis, colour in (((1, 0, 0), "#d62728"), ((0, 1, 0), "#2ca02c"), ((0, 0, 1), "#1f77b4")):
            tip = np.array(plan.frame.pose(tuple(length * v for v in axis))[:3])
            self.ax.plot(*zip(o, tip), color=colour, linewidth=2.0)
        self.ax.text(*o, " G-code 0,0,0", fontsize=8, color="#555555")
        self._origin = o

    def refit(self):
        """Fit the view to the plan and, when connected, the robot TCP - with the same scale on all
        three axes, so shapes are not distorted."""
        cloud = self._cloud if self._tcp is None else np.vstack([self._cloud, self._tcp[None, :]])
        if len(cloud):
            lo, hi = cloud.min(axis=0), cloud.max(axis=0)
        else:
            lo = hi = np.zeros(3)
        self._centre = (lo + hi) / 2.0
        smallest = 1.0 if self._plan is not None else 150.0     # robot only: show 300 mm around it
        self._home = self._half = max((hi - lo).max() / 2.0 * 1.08, smallest)
        self.ax.set_box_aspect((1, 1, 1), zoom=1.05)
        self._apply_limits()

    def _apply_limits(self):
        c, h = self._centre, self._half
        self.ax.set_xlim(c[0] - h, c[0] + h)
        self.ax.set_ylim(c[1] - h, c[1] + h)
        self.ax.set_zlim(c[2] - h, c[2] + h)
        self._place_tcp()                          # its drop line ends on the floor of the box

    # ---- live robot TCP
    def _add_live_artists(self):
        """The parts that move with the robot. They are "animated": left out of the normal drawing and
        painted on top of a saved copy of it, so a new TCP position costs a quick blit instead of a
        full 3D redraw."""
        # the selected / currently executed command is in this layer too: it changes many times a second
        (self._hi_line,) = self.ax.plot([], [], [], color=HIGHLIGHT, linewidth=3.2, animated=True)
        (self._hi_pts,) = self.ax.plot([], [], [], color=HIGHLIGHT, marker="o", ms=6, ls="none", animated=True)
        kw = dict(color=TCP_COLOUR, animated=True)
        (trail,) = self.ax.plot([], [], [], linewidth=1.3, alpha=0.75, **kw)
        (drop,) = self.ax.plot([], [], [], linewidth=0.9, linestyle=":", **kw)
        (halo,) = self.ax.plot([], [], [], marker="o", ms=21, alpha=0.22, ls="none", **kw)
        (dot,) = self.ax.plot([], [], [], marker="o", ms=9, mec="white", mew=1.5, ls="none", **kw)
        self._tcp_art = [trail, drop, halo, dot]
        self._live = [self._hi_line, self._hi_pts] + self._tcp_art
        self._place_tcp()

    def _place_tcp(self):
        if not self._live:
            return
        trail, drop, halo, dot = self._tcp_art
        if self._tcp is None:
            for art in self._tcp_art:
                art.set_data_3d([], [], [])
            return
        x, y, z = self._tcp
        colour = TCP_COLOUR if self._tcp_live else TCP_OLD_COLOUR
        trail.set_data_3d(*(np.array(self._trail).T if len(self._trail) > 1 else ([], [], [])))
        drop.set_data_3d([x, x], [y, y], [z, self._centre[2] - self._half])
        for art in (halo, dot):
            art.set_data_3d([x], [y], [z])
        for art in self._tcp_art:
            art.set_color(colour)

    def _on_draw(self, _event):
        """After every full draw: keep a copy of it, then paint the moving parts on top."""
        self._bg = self.copy_from_bbox(self.fig.bbox)
        for art in self._live:
            self.ax.draw_artist(art)

    def _paint_live(self):
        if self._bg is None:                       # a full redraw is pending anyway
            self.draw_idle()
            return
        self.restore_region(self._bg)
        for art in self._live:
            self.ax.draw_artist(art)
        self.blit(self.fig.bbox)

    def set_tcp(self, xyz, live: bool = True):
        """Show the robot TCP at xyz (mm, the coordinates the robot reports). None removes it.
        live=False greys it out: last known position, no fresh data."""
        first = self._tcp is None and xyz is not None
        self._tcp_live = live
        if xyz is None:
            self._tcp = None
            self._trail.clear()
        else:
            self._tcp = np.asarray(xyz[:3], dtype=float)
            if not self._trail or np.linalg.norm(self._trail[-1] - self._tcp) >= TRAIL_STEP_MM:
                self._trail.append(self._tcp)
        self._place_tcp()
        if first and np.any(np.abs(self._tcp - self._centre) > self._half):
            self.refit()                           # the robot is outside the picture: zoom out once
            self._redraw()
        else:
            self._paint_live()

    def set_tcp_live(self, live: bool):
        """Grey the TCP out (no fresh data) or colour it again."""
        if self._tcp is not None:
            self.set_tcp(self._tcp, live)

    def clear_trail(self):
        self._trail.clear()
        if self._tcp is not None:
            self._trail.append(self._tcp)
        self._place_tcp()
        self._paint_live()

    # ---- interaction
    def _on_scroll(self, event):
        self._half = float(np.clip(self._half * (0.85 if event.button == "up" else 1 / 0.85),
                                   self._home / 200.0, self._home * 20.0))
        self._apply_limits()
        self._redraw()

    def set_view(self, name: str):
        self.refit()
        self._look(name)
        self._redraw()

    def _look(self, name: str):
        # the straight-on views are true projections (no perspective), so distances can be compared
        self.ax.set_proj_type("persp" if name == "Iso" else "ortho")
        self.ax.view_init(*VIEWS[name])
        edge_on = {"Top (XY)": "z", "Front (XZ)": "y", "Side (YZ)": "x"}.get(name)
        for axis in "xyz":                         # the axis seen end-on has nothing to read
            self.ax.tick_params(axis=axis, labelcolor="none" if axis == edge_on else "black")
            getattr(self.ax, f"set_{axis}label")("" if axis == edge_on else f"{axis.upper()} (mm)")

    def highlight(self, index: Optional[int]):
        """Mark one command's move (None clears it)."""
        if self._plan is None or self._hi_line is None:
            return
        pts = self._plan.paths.get(index) if index is not None else None
        if pts is None:
            self._hi_line.set_data_3d([], [], [])
            self._hi_pts.set_data_3d([], [], [])
        else:
            self._hi_line.set_data_3d(*pts.T)
            targets = np.array([p[:3] for p in self._plan.cmds[index].poses])
            self._hi_pts.set_data_3d(*targets.T)
        self._paint_live()


# ============================================================================ command list
class CommandTable(QtCore.QAbstractTableModel):
    """One row per command of the plan: what the G-code said and what the robot will be told."""
    HEAD = ["#", "Line", "G-code", "Robot command", "X", "Y", "Z", "mm/s"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._plan: Optional[Plan] = None

    def set_plan(self, plan: Optional[Plan]):
        self.beginResetModel()
        self._plan = plan
        self.endResetModel()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() or self._plan is None else len(self._plan.cmds)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.HEAD)

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if role == Qt.ItemDataRole.DisplayRole and orientation == Qt.Orientation.Horizontal:
            return self.HEAD[section]
        return None

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid() or self._plan is None:
            return None
        c, col = self._plan.cmds[index.row()], index.column()
        motion = c.kind in eng.MOTION_KINDS
        if role == Qt.ItemDataRole.DisplayRole:
            if col == 0:
                return index.row() + 1
            if col == 1:
                return c.line
            if col == 2:
                return c.src
            if col == 3:
                return c.instr if motion else self.describe(index.row()).split(" -> ", 1)[1]
            if motion and col in (4, 5, 6):
                return f"{c.poses[-1][col - 4] + 0.0:.3f}"
            if motion and col == 7:
                return f"{eng.speed_mm_s(c, self._plan.settings):.1f}"
            return None
        if role == Qt.ItemDataRole.TextAlignmentRole and col not in (2, 3):
            return int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        if role == Qt.ItemDataRole.ForegroundRole and col == 3 and motion:
            return QtGui.QBrush(QtGui.QColor(STYLES[style_key(c)]["color"]))
        if role == Qt.ItemDataRole.ToolTipRole:
            return self.describe(index.row())
        return None

    def describe(self, row: int) -> str:
        """The full trace line of a command, exactly as the command-line tool prints it."""
        return eng.describe(self._plan.cmds[row], self._plan.settings)


# ============================================================================ main window
TITLE = APP_NAME
HERE_DIR = os.path.dirname(os.path.abspath(__file__))
DEMO_FILE = os.path.join(HERE_DIR, "samples", "demo_part.gcode")
ICON_FILE = os.path.join(HERE_DIR, "resources", "icon.png")
BANNER = {"info": ("#eef2f7", "#374151"), "good": ("#e6f4ea", "#137333"), "bad": ("#fdecea", "#b3261e"),
          "busy": ("#fff4e5", "#8a4b00")}


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: argparse.Namespace, simulator=None):
        super().__init__()
        self.settings = settings
        self.plan: Optional[Plan] = None
        self.monitor = None                        # RobotMonitor while connected
        self._sample: Optional[dict] = None        # newest robot status, shown by _show_live()
        self.simulator = simulator                 # sim_robot.SimRobot in --simulate mode, else None
        self._sim_port: Optional[int] = None
        self.session: Optional[RobotSession] = None    # command connection, while connected
        self.job: Optional[Job] = None             # check or run in progress
        self.setup = RunSetup(placement=HERE) if simulator is not None else RunSetup()
        self._has_setup = False                    # Set up run... has been accepted at least once
        self.checked: Optional[Plan] = None        # the plan that passed the check and may be started
        self._run_row: Optional[int] = None        # newest command being executed, shown by _show_live()
        self.title = TITLE + ("  [SIMULATION]" if simulator is not None else "")
        self.setWindowTitle(self.title)
        if os.path.exists(ICON_FILE):
            self.setWindowIcon(QtGui.QIcon(ICON_FILE))
        self.resize(1360, 780)
        self.setAcceptDrops(True)

        self.canvas = PlanCanvas(self)
        self.table_model = CommandTable(self)
        self.table = QtWidgets.QTableView()
        self.table.setModel(self.table_model)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setWordWrap(False)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.selectionModel().currentRowChanged.connect(self._on_row)

        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        self.summary = QtWidgets.QPlainTextEdit()
        self.summary.setReadOnly(True)
        self.summary.setFont(mono)
        self.summary.setPlaceholderText("Plan summary")
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(mono)
        self.log.setMaximumBlockCount(5000)
        self.log.setPlaceholderText("Checks and runs are logged here: every command as it is sent.")
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.summary, "Summary")
        self.tabs.addTab(self.log, "Log")
        self.banner = QtWidgets.QLabel()
        self.banner.setWordWrap(True)
        self.banner.setMargin(6)
        self.detail = QtWidgets.QLabel("Select a command to see what is sent to the robot.")
        self.detail.setFont(mono)
        self.detail.setWordWrap(True)
        self.detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail.setMargin(4)

        self.robot_panel = RobotStatusPanel()

        right = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        right.addWidget(self.robot_panel)
        right.addWidget(self.tabs)
        lower = QtWidgets.QWidget()
        box = QtWidgets.QVBoxLayout(lower)
        box.setContentsMargins(0, 0, 0, 0)
        box.addWidget(self.table, 1)
        box.addWidget(self.detail)
        right.addWidget(lower)
        right.setStretchFactor(2, 1)
        right.setSizes([150, 160, 450])

        split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
        split.addWidget(self.canvas)
        split.addWidget(right)
        split.setStretchFactor(0, 1)
        split.setSizes([750, 610])
        centre = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(centre)
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(0)
        column.addWidget(self.banner)
        column.addWidget(split, 1)
        self.setCentralWidget(centre)

        bar = self.addToolBar("Main")
        bar.setMovable(False)
        self.act_open = bar.addAction("Open G-code\u2026", self.open_dialog)
        self.act_open.setShortcut(QtGui.QKeySequence.StandardKey.Open)
        bar.addSeparator()
        self.act_connect = bar.addAction("Connect\u2026", self.connect_robot)
        self.act_disconnect = bar.addAction("Disconnect", self.disconnect_robot)
        bar.addSeparator()
        icon = self.style().standardIcon
        self.act_setup = bar.addAction("Set up run\u2026", self.setup_run)
        self.act_start = bar.addAction(icon(QtWidgets.QStyle.StandardPixmap.SP_MediaPlay), "Start", self.start_run)
        self.act_stop = bar.addAction(icon(QtWidgets.QStyle.StandardPixmap.SP_MediaStop), "Stop", self.stop_run)
        self.act_continue = bar.addAction("Continue", self.continue_run)
        self.act_continue.setVisible(False)
        for act in (self.act_start, self.act_stop):
            bar.widgetForAction(act).setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        bar.addSeparator()
        bar.addWidget(QtWidgets.QLabel(" View: "))
        for name in VIEWS:
            bar.addAction(name, lambda n=name: self.canvas.set_view(n))
        bar.addSeparator()
        bar.addAction("Clear trail", self.canvas.clear_trail)
        self.statusBar().showMessage("No file loaded")

        # status frames arrive faster than a screen needs: keep the newest, show it 25 times a second
        self._live_timer = QtCore.QTimer(self)
        self._live_timer.setInterval(40)
        self._live_timer.timeout.connect(self._show_live)
        self._live_timer.start()
        self._say("info", "Open a G-code file, connect to the robot, then set up the run.")
        self._update_actions()

    # ---- what can be pressed right now
    def _update_actions(self):
        busy, connected = self.job is not None, self.session is not None
        running = busy and self.job.mode == "run"
        self.act_open.setEnabled(not busy)
        self.act_connect.setEnabled(not connected and not busy)
        self.act_disconnect.setEnabled(connected and not busy)
        self.act_setup.setEnabled(connected and not busy and self.plan is not None)
        self.act_start.setEnabled(connected and not busy and self.checked is not None)
        self.act_stop.setEnabled(running)
        if not running:
            self.act_continue.setVisible(False)

    def _say(self, kind: str, text: str):
        """The strip under the toolbar: what state the job is in and what to do next."""
        background, colour = BANNER[kind]
        self.banner.setText(text)
        self.banner.setStyleSheet(f"background: {background}; color: {colour}; font-weight: 600;")
        self.statusBar().showMessage(text)

    # ---- robot connection
    def connect_robot(self):
        if self.simulator is not None:
            self.attach_simulator()
            return
        dialog = ConnectDialog(self)
        dialog.exec()
        if dialog.monitor is not None:
            self.attach_monitor(dialog.monitor)

    def attach_simulator(self):
        """--simulate: the simulated robot publishes a real status stream on a local port; listen to it."""
        if self._sim_port is None:
            self._sim_port = free_udp_port()
            self.simulator.serve_status(self._sim_port)
        monitor = RobotMonitor(Target("127.0.0.1", port=self._sim_port))
        monitor.start()
        self.attach_monitor(monitor)

    def attach_monitor(self, monitor):
        """Take over a RobotMonitor that is already listening."""
        self.disconnect_robot()
        self.monitor = monitor
        monitor.sample.connect(self._on_sample)
        monitor.stale.connect(self._on_stale)
        monitor.lost.connect(self._on_lost)
        monitor.robot_message.connect(lambda text: self.statusBar().showMessage("Robot: " + text, 10000))
        self.robot_panel.set_connected(monitor.target)
        # the command connection is opened later, by the first check, in its own thread
        connector = (lambda _ip: self.simulator) if self.simulator is not None else None
        self.session = RobotSession(monitor.target.ip, connector)
        self._say("info", f"Listening to {monitor.target}. "
                          + ("Set up the run to place the plan on the robot." if self.plan else "Open a G-code file."))
        self._update_actions()

    def disconnect_robot(self, reason: str = ""):
        if self.monitor is None:
            return
        if self.job is not None:                   # never leave a run going without its window
            self.job.stop()
            self.job.wait(8000)
            self.job = None
        monitor, self.monitor, self._sample = self.monitor, None, None
        monitor.stop()
        if self.session is not None and self.simulator is None:
            self.session.close()
        self.session, self.checked = None, None
        self.robot_panel.set_disconnected(reason)
        self.canvas.set_tcp(None)
        self._say("bad" if reason else "info", "Robot connection lost: " + reason if reason else "Disconnected")
        self._update_actions()

    # ---- set up, check, start, stop
    def setup_run(self):
        if self.plan is None or self.session is None or self.job is not None:
            return
        dialog = RunSetupDialog(self.setup, self)
        dialog.exec()
        if dialog.chosen is not None:
            self.apply_setup(dialog.chosen)

    def apply_setup(self, setup: RunSetup):
        """Use this setup and check the plan with it (reads the robot, does not move it)."""
        self.setup, self._has_setup = setup, True
        self.check_plan()

    def check_plan(self):
        if self.plan is None or self.session is None or self.job is not None:
            return
        self.checked = None
        self._launch(Job(self.session, self.plan.path, self.setup.apply(self.settings), "check", parent=self))
        self._say("busy", "Checking the plan with the robot (the robot does not move) \u2026")

    def start_run(self, confirm: bool = True):
        if self.checked is None or self.session is None or self.job is not None:
            return
        if confirm and not self._confirm_start():
            return
        self.canvas.clear_trail()
        self._launch(Job(self.session, self.checked.path, self.checked.settings, "run", expect=self.checked,
                         parent=self))
        self._say("busy", "Running - press Stop to stop the robot.")

    def _confirm_start(self) -> bool:
        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Icon.Warning)
        box.setWindowTitle("Start run")
        box.setText("The robot will move.")
        box.setInformativeText(
            f"{os.path.basename(self.checked.path)}: {len(self.checked.motions)} moves on "
            f"{'the SIMULATED robot' if self.simulator is not None else self.session.ip}\n"
            f"Tool {self.checked.settings.tool}, workpiece {self.checked.settings.user}, "
            f"global speed {self.checked.settings.global_speed} %\n\n"
            "Make sure the work area is clear and the emergency stop is within reach.")
        start = box.addButton("Start", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        box.setDefaultButton(box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel))
        box.exec()
        return box.clickedButton() is start

    def stop_run(self):
        if self.job is not None:
            self.job.stop()
            self._say("busy", "Stopping \u2026")

    def continue_run(self):
        self.act_continue.setVisible(False)
        if self.job is not None:
            self.job.proceed()
            self._say("busy", "Running - press Stop to stop the robot.")

    def _launch(self, job: Job):
        self.job = job
        job.log.connect(self.log.appendPlainText)
        job.command.connect(self._on_command)
        job.placed.connect(self._on_placed)
        job.paused.connect(self._on_paused)
        job.done.connect(self._on_job_done)
        self.log.appendPlainText(f"\n=== {job.mode.upper()}  {os.path.basename(job.path)}  ({self.setup.describe()})")
        self.tabs.setCurrentWidget(self.log)
        self._update_actions()
        job.start()

    def _on_placed(self, plan: Plan):
        if self.job is not None and self.job.mode == "check":
            self._show_plan(plan)                  # the intention, where the robot will really run it

    def _on_command(self, index: int, text: str):
        self.log.appendPlainText(text)
        if index >= 0:
            self._run_row = index

    def _on_paused(self, text: str):
        self.act_continue.setVisible(True)
        self._say("busy", f"Paused by the program ({text.split(' pause')[0].strip('[] ')}). "
                          "Press Continue to go on, or Stop.")

    def _on_job_done(self, result: JobResult):
        job, self.job = self.job, None
        if job is not None:
            job.wait(2000)
        if result.mode == "check":
            if result.plan is not None:
                self._show_plan(result.plan)       # again: now with the unreachable targets marked
                if self.setup.placement in (HERE, AUTO):   # keep this origin; do not read the TCP again
                    self.setup = replace(self.setup, placement=MANUAL, origin=list(result.plan.frame.origin),
                                         rpy=list(result.plan.frame.rpy), tool=result.settings.tool,
                                         user=result.settings.user, yaw=result.settings.yaw, tool_down=False)
            self.checked = result.plan if result.ok else None
            self._say("good" if result.ok else "bad",
                      result.message + ("  Press Start to run it." if result.ok else ""))
        elif result.ok:
            self._say("good", "Run finished.")
        elif result.stopped:
            self._say("info", "Stopped by the operator. The robot is at rest; Start runs the plan from the beginning.")
        else:
            self._say("bad", "Run ended with an error, the robot was stopped: " + result.message)
        if result.stop_failed:
            QtWidgets.QMessageBox.critical(self, "Stop command failed", result.message)
        self._update_actions()

    def _on_sample(self, sample: dict):
        if self.monitor is not None:               # late frames after a disconnect are dropped
            self._sample = sample

    def _show_live(self):
        self._show_running()
        sample, self._sample = self._sample, None
        if sample is None:
            return
        self.robot_panel.show_sample(sample)
        if TCP in sample:
            self.canvas.set_tcp(sample[TCP])

    def _show_running(self):
        row, self._run_row = self._run_row, None
        if row is not None and self.plan is not None and row < len(self.plan.cmds):
            self.table.selectRow(row)              # also highlights the move in the 3D view
            self.table.scrollTo(self.table_model.index(row, 0),
                                QtWidgets.QAbstractItemView.ScrollHint.PositionAtCenter)

    def _on_stale(self, stale: bool):
        if self.monitor is None:
            return
        self.robot_panel.set_stale(stale)
        self.canvas.set_tcp_live(not stale)

    def _on_lost(self, reason: str):
        self.disconnect_robot(reason or "connection closed")       # stops a run in progress first

    def closeEvent(self, event):
        self.disconnect_robot()
        super().closeEvent(event)

    # ---- loading
    def open_dialog(self):
        start = os.path.dirname(self.plan.path) if self.plan else ""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open G-code", start, "G-code (*.gcode *.nc *.ngc *.tap *.gc *.txt);;All files (*)")
        if path:
            self.load(path)

    def load(self, path: str) -> bool:
        if self.job is not None:
            return False
        try:
            plan = build_plan(path, self.settings)
        except (OSError, ValueError) as e:
            QtWidgets.QMessageBox.critical(self, "Cannot load G-code", f"{os.path.basename(path)}\n\n{e}")
            return False
        self.checked = None                        # a different file has to be checked again
        self._show_plan(plan)
        self.tabs.setCurrentWidget(self.summary)
        self.setWindowTitle(f"{os.path.basename(path)} - {self.title}")
        self._say("info", f"{os.path.basename(path)}: {len(plan.cmds)} commands, {len(plan.motions)} moves"
                          + (f", {len(plan.warnings)} warnings" if plan.warnings else "") + ".  "
                          + ("Set up the run to place it on the robot." if self.session else "Connect to the robot."))
        self._update_actions()
        if self.session is not None and self._has_setup:
            self.check_plan()                      # same setup as before: place and check the new file
        return True

    def _show_plan(self, plan: Plan):
        """Make `plan` the one that is listed and drawn."""
        self.plan = plan
        self._run_row = None
        self.table_model.set_plan(plan)
        self.table.resizeColumnsToContents()
        for col in (2, 3):                         # long G-code lines must not push the numbers out of sight
            self.table.setColumnWidth(col, min(self.table.columnWidth(col), 150))
        self.canvas.show_plan(plan)
        text = list(plan.summary)
        if plan.warnings:
            text += ["", f"Warnings ({len(plan.warnings)}):"] + ["  " + w for w in plan.warnings]
        self.summary.setPlainText("\n".join(text))
        self.detail.setText("Select a command to see what is sent to the robot.")

    def _on_row(self, current, _previous):
        if self.plan is None or not current.isValid():
            self.canvas.highlight(None)
            return
        self.detail.setText(self.table_model.describe(current.row()))
        self.canvas.highlight(current.row() if current.row() in self.plan.paths else None)

    # ---- drag and drop
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if urls and urls[0].isLocalFile():
            self.load(urls[0].toLocalFile())


def self_test(app) -> int:
    """Prove that this copy of the application is complete: open the built-in demo part, place it,
    check it and run it on the simulated robot. No real robot is contacted. Returns 0 if all went well."""
    from . import sim_robot

    def wait(condition, seconds):
        end = time.monotonic() + seconds
        while not condition() and time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)
        return condition()

    try:
        import fairino  # noqa: F401
        sdk = "included"
    except ImportError:
        sdk = "NOT included - this copy can only simulate"
    print(f"{APP_NAME} {__version__}   Qt binding: {QtCore.__name__.split('.')[0]}   FAIRINO SDK: {sdk}")
    settings = default_settings()
    settings.start_wait = 0.05
    win = MainWindow(settings, sim_robot.SimRobot(time_scale=40))
    win.show()
    steps = [("open the demo part", lambda: win.load(DEMO_FILE), lambda: win.plan is not None, 5),
             ("connect to the simulated robot", win.attach_simulator, lambda: win.canvas._tcp is not None, 10),
             ("place and check the plan", lambda: win.apply_setup(RunSetup(placement=HERE, approach=0)),
              lambda: win.job is None and win.checked is not None, 30),
             ("run the plan", lambda: win.start_run(confirm=False),
              lambda: win.job is None and win.banner.text() == "Run finished.", 60)]
    result = 0
    for name, action, done, seconds in steps:
        action()
        ok = wait(done, seconds)
        print(f"  {'ok    ' if ok else 'FAILED'}  {name}")
        if not ok:
            print("          " + win.banner.text())
            result = 1
            break
    win.close()
    print("self-test passed" if result == 0 else "self-test FAILED")
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="fairino-gcode", description=APP_NAME)
    ap.add_argument("file", nargs="?", help="G-code file to open")
    ap.add_argument("--simulate", action="store_true",
                    help="use a simulated robot instead of a real one (nothing is sent to any robot)")
    ap.add_argument("--self-test", action="store_true",
                    help="check that this installation is complete (runs the demo part on the simulated robot)")
    ap.add_argument("--version", action="version", version=f"{APP_NAME} {__version__}")
    args = ap.parse_args(argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName(APP_NAME)
    if args.self_test:
        return self_test(app)
    simulator = None
    if args.simulate:
        from . import sim_robot
        simulator = sim_robot.SimRobot()
    win = MainWindow(default_settings(), simulator)
    win.show()
    if args.file:
        win.load(args.file)
    if simulator is not None:
        win.attach_simulator()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
