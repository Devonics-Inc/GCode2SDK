#!/usr/bin/env python3
"""
run_control.py - running a plan on the robot from the GUI.

    RunSetup      where the drawing sits on the robot, how fast it runs, and the per-line digital outputs
                  (edited in RunSetupDialog: a "Global" tab and a "Per line" tab)
    RobotSession  the command connection to the robot (FAIRINO SDK), opened on first use
    Job           a background thread that does one of two things with a file:
                    "check"  place the plan with the robot and test that every target is reachable
                             - reads only, the robot does not move
                    "run"    the same check, then send the plan to the robot

Everything that talks to the robot happens inside the Job thread. The GUI only sets flags (stop,
continue) and listens to signals, so the SDK connection is never used from two threads at once.
A run is sent in the engine's "stoppable" mode, so Stop takes effect within milliseconds.
"""
from __future__ import annotations

import argparse
import copy
import threading
import time
import traceback
from dataclasses import dataclass, field, replace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets

from . import auto_place
from . import gcode_to_fairino as eng
from .plan_model import Plan, build_plan
from .speed_monitor import FEED, RAPID

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal
Qt = QtCore.Qt

REF, HERE, MANUAL, AUTO = "ref", "here", "manual", "auto"
MAX_OUTPUTS = 3                             # per-line digital outputs at most


# ============================================================================ setup
def speed_summary(s) -> str:
    """How fast a job runs, in words. s: a RunSetup or the engine settings (same attribute names)."""
    if s.speed_mode == "servo":
        return (f"fixed TCP speed {s.tcp_speed:g} mm/s streamed with ServoJ (acc {s.acc:g} mm/s²), "
                f"G0 at {s.global_speed} %")
    if s.speed_mode == "physical":
        return (f"TCP speed {s.tcp_speed:g} mm/s move by move (acc {s.acc:g} mm/s²), blend {s.blend:g} mm, "
                f"G0 at {s.global_speed} %")
    return f"{s.global_speed} % of full speed for every move, blend {s.blend:g} mm"


@dataclass
class RunSetup:
    placement: str = REF                    # REF / HERE / AUTO from the dialog; MANUAL once a check has fixed
                                            # the place found by HERE / AUTO (the window does that)
    placed_by: str = ""                     # MANUAL: whether HERE or AUTO found it (the dialog shows that)
    ref_point: str = ""
    origin: List[float] = field(default_factory=lambda: [400.0, 0.0, 200.0])
    rpy: List[float] = field(default_factory=lambda: [180.0, 0.0, 0.0])
    tool: int = 0
    user: int = 0
    global_speed: int = 20                  # % of full speed for every move; deliberately slow by default
    approach: float = 20.0                  # mm above the first / last point; 0 = none
    spindle_do: int = -1                    # control-box DO for M3/M5; -1 = none
    auto_range: float = 500.0               # AUTO: search this far (mm) around the tool tip
    auto_rotate: bool = True                # AUTO: may also turn the drawing in steps of 90 deg
    yaw: float = 0.0                        # the drawing turned about Z by this much (deg)
    turn_with_path: bool = False            # False: the tool keeps one posture. True: as the WebApp does
    tool_down: bool = False                 # use "straight down" instead of the posture the tool has
    speed_mode: str = "percent"             # "percent": every move at global_speed %; "physical": see below
    tcp_speed: float = 20.0                 # physical: MoveL / MoveC / Circle at this TCP speed, mm/s...
    acc: float = 200.0                      # ...with this acceleration, mm/s^2 (MoveJ keeps global_speed %)
    blend: float = 0.5                      # mm, blend radius between moves that run on into each other
    outputs: List[int] = field(default_factory=list)            # per line: control-box DO numbers (<= 3)
    output_lines: Dict[int, List[bool]] = field(default_factory=dict)   # G-code line -> high? for each of
                                            # `outputs`; a line that is not in it: all low
    output_defaults: List[bool] = field(default_factory=list)   # high? for each of `outputs` outside the lines
    line_delays: Dict[int, int] = field(default_factory=dict)   # G-code line -> ms to wait before it starts

    def apply(self, base: argparse.Namespace) -> argparse.Namespace:
        """The engine settings for this setup (a copy of `base` with the setup filled in)."""
        a = copy.deepcopy(base)
        a.ref_point = self.ref_point.strip() if self.placement == REF else None
        a.origin_here = self.placement == HERE
        a.auto_place, a.auto_range = self.placement == AUTO, float(self.auto_range)
        a.auto_yaws = [0.0, 90.0, 180.0, 270.0] if self.auto_rotate else [float(self.yaw)]
        a.yaw = float(self.yaw)
        a.posture = "tangent" if self.turn_with_path else "fixed"
        a.tool_down = bool(self.tool_down)
        a.origin = list(self.origin) if self.placement == MANUAL else None
        a.rpy = list(self.rpy)
        a.tool, a.user = (self.tool, self.user) if self.placement == MANUAL else (0, 0)
        a.global_speed = int(self.global_speed)
        a.speed_mode, a.tcp_speed, a.acc = self.speed_mode, float(self.tcp_speed), float(self.acc)
        a.blend = float(self.blend)
        a.approach, a.spindle_do = float(self.approach), int(self.spindle_do)
        a.outputs = [int(p) for p in self.outputs]
        a.output_lines = {int(k): [bool(v) for v in vs] for k, vs in self.output_lines.items()}
        a.output_defaults = [bool(v) for v in self.output_defaults]
        a.line_delays = {int(k): int(v) for k, v in self.line_delays.items() if int(v) > 0}
        a.quiet, a.dry_run = False, False
        return a

    def describe(self) -> str:
        where = {REF: f"reference teach point '{self.ref_point}'", HERE: "G-code 0,0,0 at the TCP position",
                 AUTO: f"automatic placement within \u00b1{self.auto_range:g} mm of the TCP",
                 MANUAL: f"origin {[round(v, 2) for v in self.origin]}, tool {self.tool}, "
                         f"workpiece {self.user}"}[self.placement]
        turned = f", drawing turned {self.yaw:g}\u00b0" if self.yaw and self.placement != AUTO else ""
        posture = ("tool turns with the path" if self.turn_with_path else "fixed posture") \
            + (", straight down" if self.tool_down else "")
        outputs = (f"; {', '.join(eng.output_name(p) for p in self.outputs)} set per line" if self.outputs else "")
        delays = f"; {len(self.line_delays)} line delay(s)" if self.line_delays else ""
        return f"{where}{turned}; {posture}; {speed_summary(self)}{outputs}{delays}"


class OutputBox(QtWidgets.QComboBox):
    """Picks an output: control-box DO0-DO7 (digital) or CO0-CO7 (configurable), or the tool's (end-effector's)
    Tool DO0-DO1. value() is its number: 0-7 DO0-DO7, 8-15 CO0-CO7, 16-17 Tool DO0-DO1 (-1 = none, if offered)."""
    valueChanged = Signal(int)

    def __init__(self, value: int, none: bool = False, parent=None):
        super().__init__(parent)
        if none:
            self.addItem("none", -1)
        for pin in range(eng.OUTPUTS):
            if pin in (8, eng.TOOL_DO):
                self.insertSeparator(self.count())
            self.addItem(eng.output_name(pin), pin)
            call, number = eng.output_call(pin)
            where = "on the tool (end-effector) flange" if pin >= eng.TOOL_DO else "in the control box"
            self.setItemData(self.count() - 1, f"{call}({number}, ...) - {where}", Qt.ItemDataRole.ToolTipRole)
        self.setValue(value)
        self.currentIndexChanged.connect(lambda _i: self.valueChanged.emit(self.value()))

    def value(self) -> int:
        return int(self.currentData())

    def setValue(self, pin: int):
        self.setCurrentIndex(max(0, self.findData(int(pin))))


MAX_DELAY_MS = 600000                       # a per-line delay of at most 10 minutes


class DelayDelegate(QtWidgets.QStyledItemDelegate):
    """The "Wait" column of the "Per line" tab: a whole number of ms, edited in a spin box; 0 shows empty."""

    def createEditor(self, parent, option, index):
        box = QtWidgets.QSpinBox(parent)
        box.setRange(0, MAX_DELAY_MS)
        box.setSingleStep(50)
        box.setSuffix(" ms")
        box.setFrame(False)
        return box

    def setEditorData(self, editor, index):
        editor.setValue(int(index.data(Qt.ItemDataRole.EditRole) or 0))

    def setModelData(self, editor, model, index):
        editor.interpretText()
        model.setData(index, int(editor.value()), Qt.ItemDataRole.EditRole)

    def displayText(self, value, locale):
        return f"{int(value)} ms" if value else ""


class RunSetupDialog(QtWidgets.QDialog):
    """Edits a RunSetup. Accepting it means "check the plan with these settings".
    Two tabs: "Global" (placement, posture, speed) and "Per line" (digital outputs for every G-code line).
    lines: the G-code lines of the loaded file, [(line number, text), ...] - the rows of the "Per line" tab."""

    def __init__(self, setup: RunSetup, parent=None, lines: Optional[Sequence[Tuple[int, str]]] = None):
        super().__init__(parent)
        self.setWindowTitle("Set up run")
        self.setModal(True)
        self.chosen: Optional[RunSetup] = None      # set when the dialog is accepted
        self._base = setup                          # what the dialog does not show is kept from here

        def spin(value, lo, hi, step=1.0, decimals=1, suffix=""):
            w = QtWidgets.QDoubleSpinBox()
            w.setRange(lo, hi)
            w.setDecimals(decimals)
            w.setSingleStep(step)
            w.setValue(value)
            w.setSuffix(suffix)
            return w

        def note(text):                 # fixed line breaks: wrapped labels make Qt misjudge the height
            label = QtWidgets.QLabel(text)
            label.setStyleSheet("color: #6b7280; margin-left: 22px;")
            return label

        # ---- where the drawing sits
        self.by_ref = QtWidgets.QRadioButton("At a reference teach point (like the WebApp)")
        self.ref_point = QtWidgets.QLineEdit(setup.ref_point)
        self.ref_point.setPlaceholderText("name of the saved teach point")
        self.by_here = QtWidgets.QRadioButton("G-code 0,0,0 at the current TCP")
        self.by_auto = QtWidgets.QRadioButton("Automatic: find the place where the robot works best")
        self.auto_range = spin(setup.auto_range, 50.0, 2000.0, 50.0, 0, " mm")
        self.auto_rotate = QtWidgets.QCheckBox("and turn the drawing in steps of 90\u00b0 if that fits better")
        self.auto_rotate.setChecked(setup.auto_rotate)
        self.yaw = spin(setup.yaw, -180.0, 180.0, 90.0, 1, "\u00b0")
        self.yaw.setToolTip("Turns the drawing about Z round its own origin. 90\u00b0 lays a part that runs "
                            "along X along Y instead.")
        # a setup fixed by a check (MANUAL) shows the option that found the place
        shown = setup.placed_by if setup.placement == MANUAL else setup.placement
        {HERE: self.by_here, AUTO: self.by_auto}.get(shown, self.by_ref).setChecked(True)

        ref_row = QtWidgets.QHBoxLayout()
        ref_row.setContentsMargins(22, 0, 0, 0)
        ref_row.addWidget(QtWidgets.QLabel("Teach point"))
        ref_row.addWidget(self.ref_point, 1)
        auto_row = QtWidgets.QHBoxLayout()
        auto_row.setContentsMargins(22, 0, 0, 0)
        auto_row.addWidget(QtWidgets.QLabel("Search up to"))
        auto_row.addWidget(self.auto_range)
        auto_row.addWidget(QtWidgets.QLabel("around the tool tip"))
        auto_row.addStretch(1)
        rotate_row = QtWidgets.QHBoxLayout()
        rotate_row.setContentsMargins(22, 0, 0, 0)
        rotate_row.addWidget(self.auto_rotate)
        yaw_row = QtWidgets.QHBoxLayout()
        yaw_row.addWidget(QtWidgets.QLabel("Turn the drawing by"))
        yaw_row.addWidget(self.yaw)
        yaw_row.addWidget(QtWidgets.QLabel("about Z"))
        yaw_row.addStretch(1)

        place = QtWidgets.QGroupBox("Where the drawing sits on the robot")
        box = QtWidgets.QVBoxLayout(place)
        box.addWidget(self.by_ref)
        box.addLayout(ref_row)
        box.addWidget(note("G-code X, Y, Z are coordinates in that point's workpiece frame.\n"
                           "Tool frame and posture come from the point."))
        box.addWidget(self.by_here)
        box.addWidget(note("Jog the tool tip to where the drawing's origin should be, then check.\n"
                           "Posture and the active tool / workpiece frame are read from the robot then."))
        box.addWidget(self.by_auto)
        box.addLayout(auto_row)
        box.addLayout(rotate_row)
        box.addWidget(note("Jog the tool tip to the height of the work surface, in the working posture, then check.\n"
                           "The drawing is put on that level where every point can be reached with the most room\n"
                           "to joint limits and singular positions. Collisions are NOT checked: look at the preview."))
        box.addLayout(yaw_row)

        # ---- how the tool is held: the WebApp's two choices
        self.keep_posture = QtWidgets.QRadioButton("Fixed: the tool keeps one posture for the whole job")
        self.turn_posture = QtWidgets.QRadioButton("Follow the path: the tool turns with the direction of the path "
                                                   "(as the WebApp does)")
        (self.turn_posture if setup.turn_with_path else self.keep_posture).setChecked(True)
        hold = QtWidgets.QGroupBox("How the tool is held")
        hold_box = QtWidgets.QVBoxLayout(hold)
        hold_box.addWidget(self.keep_posture)
        hold_box.addWidget(self.turn_posture)
        hold_box.addWidget(note("For a tilted tool that must lead or trail, like a welding torch. On a closed\n"
                                "outline the tool then turns a full circle, which the last joint may not manage."))

        # ---- how it runs: pick a speed mode, then only the fields that mode uses are shown
        self.by_percent = QtWidgets.QRadioButton("% of the robot's full speed")
        self.by_physical = QtWidgets.QRadioButton("TCP speed in mm/s, move by move")
        self.by_servo = QtWidgets.QRadioButton("Fixed TCP speed, streamed (ServoJ)")
        {"physical": self.by_physical, "servo": self.by_servo}.get(setup.speed_mode, self.by_percent).setChecked(True)
        self.mode_note = note("")
        self.tcp_speed = spin(setup.tcp_speed, 0.5, 1000.0, 5.0, 1, " mm/s")
        self.acc = spin(setup.acc, 10.0, 10000.0, 50.0, 0, " mm/s²")
        self.acc.setToolTip("Halved automatically if the controller refuses it (error 183).")
        self.global_speed = QtWidgets.QSpinBox()
        self.global_speed.setRange(1, 100)
        self.global_speed.setValue(setup.global_speed)
        self.global_speed.setSuffix(" %")
        self.speed_label = QtWidgets.QLabel()       # "Speed" or "G0 (rapid) moves", by mode
        self.blend = spin(setup.blend, 0.0, 50.0, 0.5, 1, " mm")
        self.blend.setToolTip("How far the path may round a corner between two moves so the arm need not stop "
                              "there.\nLarger = steadier speed through corners, but the corner is cut more. "
                              "Never more than\nabout half the shorter of the two moves.")
        self.approach = spin(setup.approach, 0.0, 500.0, 5.0, 1, " mm")
        self.approach.setSpecialValueText("none (as the WebApp)")
        self.spindle_do = OutputBox(setup.spindle_do, none=True)

        run = QtWidgets.QGroupBox("How it runs")
        self.run_form = form = QtWidgets.QFormLayout(run)
        form.addRow(self.by_percent)
        form.addRow(self.by_physical)
        form.addRow(self.by_servo)
        form.addRow(self.mode_note)
        form.addRow("TCP speed", self.tcp_speed)
        form.addRow("Acceleration", self.acc)
        form.addRow(self.speed_label, self.global_speed)
        form.addRow("Blend radius between moves", self.blend)
        form.addRow("Approach / retract height", self.approach)
        form.addRow("Output for M3 / M5", self.spindle_do)

        lines_tab = self._build_lines_tab(setup, lines)

        self.message = QtWidgets.QLabel("Checking reads from the robot only. It does not move.")
        self.message.setWordWrap(True)
        buttons = QtWidgets.QDialogButtonBox()
        buttons.addButton("Check plan", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        buttons.addButton(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        # the settings scroll on small screens; the message and the buttons always stay in view
        content = QtWidgets.QWidget()
        inner = QtWidgets.QVBoxLayout(content)
        inner.setContentsMargins(0, 0, 0, 0)
        inner.addWidget(place)
        inner.addWidget(hold)
        inner.addWidget(run)
        inner.addStretch(1)
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.scroll.setWidget(content)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.scroll, "Global")
        self.tabs.addTab(lines_tab, "Per line")
        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self.tabs, 1)
        outer.addWidget(self.message)
        outer.addWidget(buttons)
        for button in (self.by_ref, self.by_here, self.by_auto, self.auto_rotate,
                       self.by_percent, self.by_physical, self.by_servo):
            button.toggled.connect(self._enable)
        self._enable()
        self._fit_to_screen(content, parent)

    # ---- the "Per line" tab
    DELAY_COL, OUT0 = 3, 4                      # line table: Line, G-code, Robot command, Wait, then the outputs

    def _build_lines_tab(self, setup: RunSetup, lines) -> QtWidgets.QWidget:
        """Up to MAX_OUTPUTS control-box outputs: their state outside the G-code lines, and for each line
        whether each one is high. lines: [(number, G-code, robot command, colour or None), ...] (or just
        (number, G-code))."""
        self.lines = [(int(row[0]), str(row[1]), str(row[2]) if len(row) > 2 else "",
                       row[3] if len(row) > 3 else None) for row in (lines or ())]
        # the states are kept per slot (output 1 / 2 / 3), whether or not the slot is used just now
        self._high: Dict[int, List[bool]] = {}
        for n, states in setup.output_lines.items():
            self._high[int(n)] = (list(map(bool, states)) + [False] * MAX_OUTPUTS)[:MAX_OUTPUTS]
        for n, *_rest in self.lines:
            self._high.setdefault(n, [False] * MAX_OUTPUTS)
        self._delay: Dict[int, int] = {int(n): int(ms) for n, ms in setup.line_delays.items() if int(ms) > 0}
        self._busy = False                      # set while several cells are changed at once
        self._pressed_rows: Optional[List[int]] = None   # the selection when the mouse went down

        tab = QtWidgets.QWidget()
        box = QtWidgets.QVBoxLayout(tab)
        pins = QtWidgets.QGroupBox("Outputs: control-box DO0-DO7 / CO0-CO7, tool (end-effector) Tool DO0-DO1")
        grid = QtWidgets.QGridLayout(pins)
        head = QtWidgets.QLabel("Outside the lines")
        head.setToolTip("The state of the output whenever no G-code line is running: during the approach and\n"
                        "the retract move, after the job ends, and when it is stopped or fails.")
        grid.addWidget(head, 0, 2)
        self.out_use: List[QtWidgets.QCheckBox] = []
        self.out_pin: List[OutputBox] = []
        self.out_default: List[QtWidgets.QComboBox] = []
        defaults = (list(map(bool, setup.output_defaults)) + [False] * MAX_OUTPUTS)[:MAX_OUTPUTS]
        for k in range(MAX_OUTPUTS):
            use = QtWidgets.QCheckBox(f"Output {k + 1}")
            use.setChecked(k < len(setup.outputs))
            pin = OutputBox(setup.outputs[k] if k < len(setup.outputs) else k)
            rest = QtWidgets.QComboBox()
            rest.addItems(["Low", "High"])
            rest.setCurrentIndex(int(defaults[k] and k < len(setup.outputs)))
            rest.setToolTip(head.toolTip())
            grid.addWidget(use, k + 1, 0)
            grid.addWidget(pin, k + 1, 1)
            grid.addWidget(rest, k + 1, 2)
            use.toggled.connect(self._outputs_changed)
            pin.valueChanged.connect(self._outputs_changed)
            self.out_use.append(use)
            self.out_pin.append(pin)
            self.out_default.append(rest)
        grid.setColumnStretch(3, 1)
        box.addWidget(pins)
        box.addWidget(QtWidgets.QLabel(
            "Tick an output on a line to have it high (on) while that line runs; unticked = low (off).\n"
            "Where a state changes, the arm stops at the start of that line and switches the output.\n"
            "Outside the lines (approach, retract, after the end, a stop) each output takes its state above.\n"
            "Wait: the line waits this long before it starts, after its outputs have switched (double-click)."))

        self.line_table = table = QtWidgets.QTableWidget(len(self.lines), self.OUT0 + MAX_OUTPUTS)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked
                              | QtWidgets.QAbstractItemView.EditTrigger.EditKeyPressed
                              | QtWidgets.QAbstractItemView.EditTrigger.AnyKeyPressed)
        table.setItemDelegateForColumn(self.DELAY_COL, DelayDelegate(table))
        fixed = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        for row, (n, text, command, colour) in enumerate(self.lines):
            num = QtWidgets.QTableWidgetItem(str(n))
            num.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            items = [num, QtWidgets.QTableWidgetItem(text), QtWidgets.QTableWidgetItem(command)]
            if colour:                          # the same colours as the command list and the 3D view
                items[2].setForeground(QtGui.QBrush(QtGui.QColor(colour)))
            for col, item in enumerate(items):
                item.setFlags(fixed)
                table.setItem(row, col, item)
            wait = QtWidgets.QTableWidgetItem()
            wait.setData(Qt.ItemDataRole.EditRole, self._delay.get(n, 0))
            wait.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            wait.setFlags(fixed | Qt.ItemFlag.ItemIsEditable)
            wait.setToolTip("ms to wait before this line starts (double-click or type to change)")
            table.setItem(row, self.DELAY_COL, wait)
            for k in range(MAX_OUTPUTS):
                cell = QtWidgets.QTableWidgetItem()
                cell.setFlags(fixed | Qt.ItemFlag.ItemIsUserCheckable)
                cell.setCheckState(Qt.CheckState.Checked if self._high[n][k] else Qt.CheckState.Unchecked)
                table.setItem(row, self.OUT0 + k, cell)
        table.itemChanged.connect(self._cell_changed)
        # Line and Robot command always as wide as their header or their widest cell, whichever is wider
        # (whatever the font); the G-code takes the room that is left
        header = table.horizontalHeader()
        header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Interactive)
        for col in (0, 2):
            header.setSectionResizeMode(col, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeMode.Stretch)
        # Wait: room for the longest value while it is edited
        table.setColumnWidth(self.DELAY_COL, table.fontMetrics().horizontalAdvance(f"{MAX_DELAY_MS} ms") + 48)
        table.setMinimumHeight(220)
        # a tick in a line that is part of a selection of several lines goes to all of them: remember the
        # selection as the mouse goes down (the click itself would reduce it to the one line)
        table.viewport().installEventFilter(self)
        table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        table.customContextMenuRequested.connect(self._line_menu)
        self.no_lines = QtWidgets.QLabel("Open a G-code file to set the outputs line by line.")
        self.no_lines.setStyleSheet("color: #6b7280;")
        self.no_lines.setVisible(not self.lines)
        table.setVisible(bool(self.lines))
        box.addWidget(self.no_lines)
        box.addWidget(table, 1)

        # many lines in one go: select them, then High / Low for an output
        self.bulk_box = QtWidgets.QWidget()
        bulk = QtWidgets.QHBoxLayout(self.bulk_box)
        bulk.setContentsMargins(0, 0, 0, 0)
        bulk.addWidget(QtWidgets.QLabel("Selected lines:"))
        self.bulk_name: List[QtWidgets.QLabel] = []
        self.bulk_buttons: List[Tuple[QtWidgets.QPushButton, QtWidgets.QPushButton]] = []
        for k in range(MAX_OUTPUTS):
            name = QtWidgets.QLabel()
            high, low = QtWidgets.QPushButton("High"), QtWidgets.QPushButton("Low")
            high.clicked.connect(lambda _c=False, k=k: self.set_selected(k, True))
            low.clicked.connect(lambda _c=False, k=k: self.set_selected(k, False))
            for w in (name, high, low):
                bulk.addWidget(w)
            bulk.addSpacing(10)
            self.bulk_name.append(name)
            self.bulk_buttons.append((high, low))
        bulk.addStretch(1)
        self.bulk_box.setVisible(bool(self.lines))
        self.bulk_delay = QtWidgets.QSpinBox()
        self.bulk_delay.setRange(0, MAX_DELAY_MS)
        self.bulk_delay.setSingleStep(50)
        self.bulk_delay.setSuffix(" ms")
        self.bulk_delay.setValue(100)
        set_delay = QtWidgets.QPushButton("Set wait")
        set_delay.clicked.connect(lambda: self.set_selected_delay(self.bulk_delay.value()))
        no_delay = QtWidgets.QPushButton("No wait")
        no_delay.clicked.connect(lambda: self.set_selected_delay(0))
        self.delay_box = QtWidgets.QWidget()
        drow = QtWidgets.QHBoxLayout(self.delay_box)
        drow.setContentsMargins(0, 0, 0, 0)
        drow.addWidget(QtWidgets.QLabel("Selected lines:  wait"))
        for w in (self.bulk_delay, set_delay, no_delay):
            drow.addWidget(w)
        drow.addStretch(1)
        self.delay_box.setVisible(bool(self.lines))
        box.addWidget(self.bulk_box)
        box.addWidget(self.delay_box)
        hint = QtWidgets.QLabel("Select several lines with click-and-drag, Shift / Ctrl-click or Ctrl+A. Ticking a "
                                "box\nor typing a wait in a selected line sets all the selected lines; right-click "
                                "offers High / Low too.")
        hint.setStyleSheet("color: #6b7280;")
        hint.setVisible(bool(self.lines))
        box.addWidget(hint)
        self._outputs_changed()
        return tab

    def used_slots(self) -> List[int]:
        return [k for k in range(MAX_OUTPUTS) if self.out_use[k].isChecked()]

    def _outputs_changed(self, _value=None):
        """An output was switched on / off or given another DO / CO: columns, headers and buttons follow."""
        used = self.used_slots()
        for k in range(MAX_OUTPUTS):
            name = eng.output_name(self.out_pin[k].value())
            self.out_pin[k].setEnabled(k in used)
            self.out_default[k].setEnabled(k in used)
            self.line_table.setColumnHidden(self.OUT0 + k, k not in used)
            self.bulk_name[k].setText(f"{name}:")
            for w in (self.bulk_name[k], *self.bulk_buttons[k]):
                w.setVisible(k in used)
        self.line_table.setHorizontalHeaderLabels(["Line", "G-code", "Robot command", "Wait"]
                                                  + [eng.output_name(w.value()) for w in self.out_pin])
        self.bulk_box.setEnabled(bool(used))

    def selected_rows(self) -> List[int]:
        return sorted({i.row() for i in self.line_table.selectionModel().selectedRows()})

    def eventFilter(self, obj, event):
        if obj is self.line_table.viewport():
            if event.type() == QtCore.QEvent.Type.MouseButtonPress:
                self._pressed_rows = self.selected_rows()
            elif event.type() == QtCore.QEvent.Type.MouseButtonRelease:
                QtCore.QTimer.singleShot(0, self._forget_press)
        return super().eventFilter(obj, event)

    def _forget_press(self):
        self._pressed_rows = None

    def _cell_changed(self, item):
        if self._busy:
            return
        if item.column() == self.DELAY_COL:
            ms = int(item.data(Qt.ItemDataRole.EditRole) or 0)
            rows = self.selected_rows()
            self.set_delay([item.row()] + (rows if item.row() in rows and len(rows) > 1 else []), ms)
            return
        k = item.column() - self.OUT0
        if k < 0:
            return
        high = item.checkState() == Qt.CheckState.Checked
        self._high[self.lines[item.row()][0]][k] = high
        # the line is one of several selected: they all get the same state
        rows = self._pressed_rows if self._pressed_rows is not None else self.selected_rows()
        if item.row() in rows and len(rows) > 1:
            self._set_rows(rows, k, high)
            if self._pressed_rows is not None:             # the click narrowed the selection: put it back
                QtCore.QTimer.singleShot(0, lambda rows=list(rows): self._select_rows(rows))

    def _set_rows(self, rows, k: int, high: bool):
        self._busy = True
        try:
            for row in rows:
                self.line_table.item(row, self.OUT0 + k).setCheckState(
                    Qt.CheckState.Checked if high else Qt.CheckState.Unchecked)
                self._high[self.lines[row][0]][k] = high
        finally:
            self._busy = False

    def _select_rows(self, rows):
        model, sel = self.line_table.model(), QtCore.QItemSelection()
        for row in rows:
            sel.select(model.index(row, 0), model.index(row, model.columnCount() - 1))
        self.line_table.selectionModel().select(sel, QtCore.QItemSelectionModel.SelectionFlag.ClearAndSelect)

    def set_selected(self, k: int, high: bool):
        """Output slot k high / low on every selected line."""
        self._set_rows(self.selected_rows(), k, high)

    def set_delay(self, rows, ms: int):
        """Lines `rows` wait `ms` before they start (0: no wait)."""
        self._busy = True
        try:
            for row in sorted(set(rows)):
                n = self.lines[row][0]
                self.line_table.item(row, self.DELAY_COL).setData(Qt.ItemDataRole.EditRole, int(ms))
                if ms > 0:
                    self._delay[n] = int(ms)
                else:
                    self._delay.pop(n, None)
        finally:
            self._busy = False

    def set_selected_delay(self, ms: int):
        self.set_delay(self.selected_rows(), ms)

    def _line_menu(self, pos):
        rows = self.selected_rows()
        if not rows:
            return
        menu = QtWidgets.QMenu(self)
        for k in self.used_slots():
            name = eng.output_name(self.out_pin[k].value())
            menu.addAction(f"{name} high on {len(rows)} line(s)", lambda k=k: self.set_selected(k, True))
            menu.addAction(f"{name} low on {len(rows)} line(s)", lambda k=k: self.set_selected(k, False))
        menu.addSeparator()
        ms = self.bulk_delay.value()
        menu.addAction(f"Wait {ms} ms before {len(rows)} line(s)", lambda: self.set_selected_delay(ms))
        menu.addAction(f"No wait before {len(rows)} line(s)", lambda: self.set_selected_delay(0))
        menu.exec(self.line_table.viewport().mapToGlobal(pos))

    def _outputs_problem(self) -> Optional[str]:
        pins = [self.out_pin[k].value() for k in self.used_slots()]
        if len(set(pins)) < len(pins):
            return "Each output needs its own DO / CO / Tool DO: two outputs use the same one."
        if self.spindle_do.value() >= 0 and self.spindle_do.value() in pins:
            return (f"{eng.output_name(self.spindle_do.value())} is already the output for M3 / M5 (Global tab). "
                    "Choose another one for the per-line outputs.")
        return None

    def _fit_to_screen(self, content, parent):
        """Open as big as the settings need, but never bigger than the screen the dialog is on.
        (A QScrollArea asks for a small default size, so the size is set here.)"""
        screen = (parent.screen() if parent is not None else None) or QtGui.QGuiApplication.primaryScreen()
        if screen is None:
            return
        avail = screen.availableGeometry()
        bar = self.style().pixelMetric(QtWidgets.QStyle.PixelMetric.PM_ScrollBarExtent)
        m = self.layout().contentsMargins()
        spacing = self.layout().spacing()
        need = content.sizeHint()
        extra_h = (self.message.sizeHint().height() + self.layout().itemAt(2).widget().sizeHint().height()
                   + 2 * spacing + m.top() + m.bottom())
        width = need.width() + bar + m.left() + m.right() + 24                       # + the tab frame
        height = need.height() + extra_h + self.tabs.tabBar().sizeHint().height() + 24
        self.resize(min(width, int(avail.width() * 0.95)), min(height, int(avail.height() * 0.90)))

    def _enable(self):
        self.ref_point.setEnabled(self.by_ref.isChecked())
        self.auto_range.setEnabled(self.by_auto.isChecked())
        self.auto_rotate.setEnabled(self.by_auto.isChecked())
        self.yaw.setEnabled(not (self.by_auto.isChecked() and self.auto_rotate.isChecked()))
        mode = self.speed_mode()
        mm_s = mode != "percent"
        self.run_form.setRowVisible(self.tcp_speed, mm_s)
        self.run_form.setRowVisible(self.acc, mm_s)
        self.run_form.setRowVisible(self.blend, mode != "servo")      # fixed speed rounds the corners itself
        self.speed_label.setText("G0 (rapid) moves" if mm_s else "Speed")
        self.mode_note.setText({
            "percent": "Every move, G0 included, runs at this % of the robot's full speed.\n"
                       "F and S in the G-code are ignored. Start low for a first run.",
            "physical": "MoveL / MoveC / Circle run at the TCP speed; the arm slows a little at each join\n"
                        "between moves. G0 (MoveJ) moves run at the % below. F and S are ignored.",
            "servo": "The TCP holds this speed through the joins between moves. Sharp corners are\n"
                     "rounded by up to speed² / acceleration (the Summary says how much). G0 (MoveJ)\n"
                     "moves run at the % below. F and S are ignored. Try it slowly first."}[mode])

    def speed_mode(self) -> str:
        return "servo" if self.by_servo.isChecked() else "physical" if self.by_physical.isChecked() else "percent"

    def setup(self) -> RunSetup:
        placement = REF if self.by_ref.isChecked() else HERE if self.by_here.isChecked() else AUTO
        used = self.used_slots()
        output_lines = {n: [states[k] for k in used] for n, states in sorted(self._high.items())
                        if any(states[k] for k in used)}
        return replace(self._base, placement=placement,
                       outputs=[self.out_pin[k].value() for k in used], output_lines=output_lines,
                       output_defaults=[self.out_default[k].currentIndex() == 1 for k in used],
                       line_delays=dict(sorted(self._delay.items())),
                       placed_by="", ref_point=self.ref_point.text().strip(),
                       global_speed=self.global_speed.value(),
                       approach=self.approach.value(), spindle_do=self.spindle_do.value(),
                       auto_range=self.auto_range.value(), auto_rotate=self.auto_rotate.isChecked(),
                       yaw=self.yaw.value(), turn_with_path=self.turn_posture.isChecked(), tool_down=False,
                       speed_mode=self.speed_mode(),
                       tcp_speed=self.tcp_speed.value(), acc=self.acc.value(), blend=self.blend.value())

    def accept(self):
        if self.by_ref.isChecked() and not self.ref_point.text().strip():
            self.message.setText("Type the name of the reference teach point saved on the robot.")
            self.message.setStyleSheet("color: #d62728;")
            self.tabs.setCurrentIndex(0)
            self.ref_point.setFocus()
            return
        problem = self._outputs_problem()
        if problem:
            self.message.setText(problem)
            self.message.setStyleSheet("color: #d62728;")
            self.tabs.setCurrentIndex(1)
            return
        self.chosen = self.setup()
        super().accept()


# ============================================================================ connection
class RobotSession:
    """The command connection to one robot. Opened on first use, inside the Job thread, and kept."""

    def __init__(self, ip: str, connector: Optional[Callable] = None):
        self.ip = ip
        self._connector = connector or eng.connect
        self.robot = None

    def get(self):
        if self.robot is None:
            try:
                self.robot = self._connector(self.ip)
            except ImportError as exc:
                raise eng.SetupError("The FAIRINO Python SDK was not found. Install it (or put its Robot.py "
                                     "next to this program) to send commands to the robot.") from exc
        return self.robot

    def close(self):
        robot, self.robot = self.robot, None
        if robot is not None:
            try:
                robot.CloseRPC()
            except Exception:           # closing must never stop the GUI from disconnecting
                pass


# ============================================================================ job
@dataclass
class JobResult:
    mode: str                               # "check" / "run"
    ok: bool = False
    stopped: bool = False                   # ended by the operator
    message: str = ""
    plan: Optional[Plan] = None             # the plan placed on the robot (None if it got that far)
    settings: Optional[argparse.Namespace] = None    # the settings it was placed with, for the run
    stop_failed: bool = False               # the arm could not be told to stop


class Job(QtCore.QThread):
    log = Signal(str)
    command = Signal(int, str)              # index into plan.cmds (-1: approach / retract), trace line
    phase = Signal(str, float)              # "feed" / "rapid" (G0 -> MoveJ) starts: time.monotonic() in
                                            # this thread, at the moment the command is sent (speed graph)
    placed = Signal(object)                 # Plan, as soon as it is positioned (before the IK check)
    paused = Signal(str)                    # M0 / M1: waiting for continue()
    done = Signal(object)                   # JobResult

    def __init__(self, session: RobotSession, path: str, settings: argparse.Namespace, mode: str,
                 expect: Optional[Plan] = None, parent=None):
        super().__init__(parent)
        self.session, self.path, self.settings, self.mode, self.expect = session, path, settings, mode, expect
        self._abort = threading.Event()
        self._continue = threading.Event()
        self._streamer = None

    # ---- called from the GUI thread: flags only
    def stop(self):
        self._abort.set()

    def proceed(self):
        self._continue.set()

    # ---- job thread
    def run(self):
        result = JobResult(self.mode)
        moving = False
        try:
            robot = self.session.get()
            a = copy.deepcopy(self.settings)
            auto = getattr(a, "auto_place", False)
            if a.origin_here or auto:       # these use whatever frames are active on the robot now
                a.tool = int(eng.unpack(robot.GetActualTCPNum())[1][0])
                a.user = int(eng.unpack(robot.GetActualWObjNum())[1][0])
            if auto:
                self._place_automatically(robot, a)
            frame = eng.resolve_frame(a, robot, self.log.emit)
            if getattr(a, "tool_down", False):
                frame.rpy = straight_down(frame.rpy)
                self.log.emit(f"Tool straight down: posture {[round(v, 2) for v in frame.rpy]}")
            if a.origin_here:               # ...and is then fixed: a later run must not re-read the TCP
                a.origin, a.origin_here = list(frame.origin), False
            a.rpy = list(frame.rpy)
            if not a.ref_point:             # (a teach point is read again by the run, so it stays "down")
                a.tool_down = False
            plan = build_plan(self.path, a, frame)
            result.plan, result.settings = plan, a
            if self.expect is not None and not same_placement(self.expect, plan):
                raise eng.SetupError("The placement is not the one that was checked (teach point or active "
                                     "frames changed). Check the plan again.")
            self.placed.emit(plan)

            streamer = self._streamer = eng.Streamer(robot, frame, a)
            streamer.abort = self._abort
            streamer.log = self.log.emit
            streamer.pause = self._pause
            index = {id(c): k for k, c in enumerate(plan.cmds)}
            streamer.on_command = lambda c, text: self._sent(index.get(id(c), -1), c, text)
            streamer.stoppable = True

            self.log.emit(f"Checking that the robot can reach all {len(plan.motions)} moves, "
                          f"with a point every {a.check_step:g} mm along them \u2026")
            reachable = streamer.check_ik(plan.cmds)
            plan.unreachable = [list(pose[:3]) for _c, pose in streamer.unreachable]
            streamer.check_abort()
            if not reachable:
                result.message = (f"{len(plan.unreachable)} point(s) of the path cannot be reached from this "
                                  "placement (marked in the view; details in the Log). Move the origin or "
                                  "change the posture.")
            elif self.mode == "check":
                result.ok, result.message = True, "Plan checked: the whole path is reachable."
            else:
                self.log.emit(f"Starting. Speed: {speed_summary(a)}.")
                moving = True
                streamer.prepare()
                streamer.run(plan.cmds)
                result.ok, result.message = True, "Run finished."
        except eng.Aborted:
            result.stopped, result.message = True, "Stopped by the operator."
        except (eng.SetupError, eng.RobotCommandError, OSError, ValueError) as exc:
            result.message = str(exc)
        except Exception as exc:            # a bug must still stop the arm and reach the operator
            result.message = f"Unexpected error: {exc!r}"
            self.log.emit(traceback.format_exc())
        if moving and not result.ok:
            try:
                self._streamer.emergency_stop()
                self.log.emit("Robot stopped.")
            except Exception as exc:
                result.stop_failed = True
                result.message += f"  THE STOP COMMAND FAILED ({exc!r}) - USE THE EMERGENCY STOP."
        self.log.emit(result.message)
        self.done.emit(result)

    def _sent(self, index: int, c, text: str):
        """A command is about to go out. A MoveJ always runs on its own, from an idle arm until it is
        idle again, so it starts a rapid and the next command sent ends it."""
        if self.mode == "run":
            self.phase.emit(RAPID if c.instr == "MoveJ" else FEED, time.monotonic())
        self.command.emit(index, text)

    def _place_automatically(self, robot, a):
        """Fill in a.origin / a.rpy: height and posture from the tool as it is now, X and Y searched."""
        err, rest = eng.unpack(robot.GetActualTCPPose())
        if err != 0:
            raise eng.SetupError(f"GetActualTCPPose failed ({err})")
        pose = [float(v) for v in rest[0]]
        joints = [float(v) for v in eng.unpack(robot.GetActualJointPosDegree())[1][0]]
        eng.check_frames(robot, a, pose, joints, "the current TCP pose")
        a.rpy = straight_down(pose[3:]) if getattr(a, "tool_down", False) else pose[3:]
        layouts = []
        for yaw in a.auto_yaws:
            local = build_plan(self.path, a, eng.Frame([0.0, 0.0, 0.0], a.rpy, yaw, a.scale))
            if local.bbox is None:
                raise eng.SetupError("The file has no moves to place.")
            lo, hi = local.bbox
            layouts.append(auto_place.Layout(yaw, auto_place.sample_targets(local, a.approach),
                                             [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]))

        def whole_path_fits(origin, yaw):
            """The same check the run makes, for one candidate, without filling the log."""
            frame = eng.Frame(origin, a.rpy, yaw, a.scale)
            quick = copy.copy(a)
            quick.check_max = 1             # the first point that fails settles it
            streamer = eng.Streamer(robot, frame, quick)
            streamer.abort, streamer.log = self._abort, lambda _text: None
            fits = streamer.check_ik(build_plan(self.path, a, frame).cmds)
            streamer.check_abort()
            return fits

        found = auto_place.find_origin(robot, layouts, pose[:3], joints, a.auto_range, self.log.emit,
                                       self._abort.is_set, whole_path_fits)
        a.origin, a.yaw, a.auto_place, a.tool_down = list(found.origin), found.yaw, False, False

    def _pause(self, text: str):
        self._continue.clear()
        self.paused.emit(text)
        while not self._continue.wait(0.05):
            if self._abort.is_set():
                return


def straight_down(rpy) -> list:
    """The posture with the tool axis square to the drawing plane, keeping the way the tool is turned."""
    return [180.0, 0.0, float(rpy[2])]


def same_placement(a: Plan, b: Plan) -> bool:
    """Both plans put the same file at the same place, turned the same way, in the same frames, with the
    same posture."""
    def key(p: Plan):
        return (p.path, [round(v, 3) for v in p.frame.origin], [round(v, 3) for v in p.frame.rpy],
                round(p.settings.yaw, 3), p.settings.posture, p.settings.tool, p.settings.user, len(p.cmds))
    return key(a) == key(b)
