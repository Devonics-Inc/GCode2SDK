#!/usr/bin/env python3
"""
run_control.py - running a plan on the robot from the GUI.

    RunSetup      where the drawing sits on the robot and how fast it runs (edited in RunSetupDialog)
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
import traceback
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from matplotlib.backends.qt_compat import QtCore, QtWidgets

from . import auto_place
from . import gcode_to_fairino as eng
from .plan_model import Plan, build_plan

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal
Qt = QtCore.Qt

REF, HERE, MANUAL, AUTO = "ref", "here", "manual", "auto"


# ============================================================================ setup
@dataclass
class RunSetup:
    placement: str = REF                    # REF / HERE / MANUAL / AUTO
    ref_point: str = ""
    origin: List[float] = field(default_factory=lambda: [400.0, 0.0, 200.0])
    rpy: List[float] = field(default_factory=lambda: [180.0, 0.0, 0.0])
    tool: int = 0
    user: int = 0
    global_speed: int = 20                  # %, deliberately slow by default
    rapid_speed: Optional[float] = None     # mm/s for G0; None = the file's S word
    approach: float = 20.0                  # mm above the first / last point; 0 = none
    spindle_do: int = -1                    # control-box DO for M3/M5; -1 = none
    auto_range: float = 500.0               # AUTO: search this far (mm) around the tool tip
    auto_rotate: bool = True                # AUTO: may also turn the drawing in steps of 90 deg
    yaw: float = 0.0                        # the drawing turned about Z by this much (deg)
    turn_with_path: bool = False            # False: the tool keeps one posture. True: as the WebApp does
    tool_down: bool = False                 # use "straight down" instead of the posture the tool has

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
        a.global_speed, a.rapid_speed = int(self.global_speed), self.rapid_speed
        a.approach, a.spindle_do = float(self.approach), int(self.spindle_do)
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
        return f"{where}{turned}; {posture}; global speed {self.global_speed} %"


class RunSetupDialog(QtWidgets.QDialog):
    """Edits a RunSetup. Accepting it means "check the plan with these settings"."""

    def __init__(self, setup: RunSetup, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set up run")
        self.setModal(True)
        self.chosen: Optional[RunSetup] = None      # set when the dialog is accepted

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
        self.by_manual = QtWidgets.QRadioButton("At coordinates I type in")
        self.origin = [spin(v, -5000, 5000, 1.0, 2) for v in setup.origin]
        self.rpy = [spin(v, -180, 180, 1.0, 2) for v in setup.rpy]
        self.tool = QtWidgets.QSpinBox()
        self.tool.setRange(0, 14)
        self.tool.setValue(setup.tool)
        self.user = QtWidgets.QSpinBox()
        self.user.setRange(0, 14)
        self.user.setValue(setup.user)
        {REF: self.by_ref, HERE: self.by_here, AUTO: self.by_auto,
         MANUAL: self.by_manual}[setup.placement].setChecked(True)

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
        self.manual_box = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(self.manual_box)
        grid.setContentsMargins(22, 0, 0, 0)
        for col, text in enumerate(("X", "Y", "Z")):
            grid.addWidget(QtWidgets.QLabel(text), 0, col + 1, Qt.AlignmentFlag.AlignHCenter)
        grid.addWidget(QtWidgets.QLabel("Origin (mm)"), 1, 0)
        grid.addWidget(QtWidgets.QLabel("Posture RX RY RZ (\u00b0)"), 2, 0)
        for col in range(3):
            grid.addWidget(self.origin[col], 1, col + 1)
            grid.addWidget(self.rpy[col], 2, col + 1)
        grid.addWidget(QtWidgets.QLabel("Tool / workpiece frame"), 3, 0)
        grid.addWidget(self.tool, 3, 1)
        grid.addWidget(self.user, 3, 2)

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
        box.addWidget(self.by_manual)
        box.addWidget(self.manual_box)
        box.addLayout(yaw_row)

        # ---- how the tool is held
        self.keep_posture = QtWidgets.QRadioButton("Keep one posture for the whole job")
        self.turn_posture = QtWidgets.QRadioButton("Turn the tool with the direction of the path (as the WebApp does)")
        (self.turn_posture if setup.turn_with_path else self.keep_posture).setChecked(True)
        self.tool_down = QtWidgets.QCheckBox("Point the tool straight down at the drawing, whatever posture it has now")
        self.tool_down.setChecked(setup.tool_down)
        hold = QtWidgets.QGroupBox("How the tool is held")
        hold_box = QtWidgets.QVBoxLayout(hold)
        hold_box.addWidget(self.keep_posture)
        hold_box.addWidget(self.turn_posture)
        hold_box.addWidget(note("Only for a tilted tool that must lead or trail, like a welding torch. On a closed\n"
                                "outline the tool then has to turn a full circle, which the last joint cannot do."))
        hold_box.addWidget(self.tool_down)
        hold_box.addWidget(note("Otherwise the posture is the one the tool has when you check (or the teach point's,\n"
                                "or the typed one). A tool jogged by hand is rarely exactly square to the drawing."))

        # ---- how it runs
        self.global_speed = QtWidgets.QSpinBox()
        self.global_speed.setRange(1, 100)
        self.global_speed.setValue(setup.global_speed)
        self.global_speed.setSuffix(" %")
        self.rapid = spin(setup.rapid_speed or 0.0, 0.0, 2000.0, 10.0, 0, " mm/s")
        self.rapid.setSpecialValueText("from the file's S word")
        self.approach = spin(setup.approach, 0.0, 500.0, 5.0, 1, " mm")
        self.approach.setSpecialValueText("none (as the WebApp)")
        self.spindle_do = QtWidgets.QSpinBox()
        self.spindle_do.setRange(-1, 15)
        self.spindle_do.setValue(setup.spindle_do)
        self.spindle_do.setSpecialValueText("none")

        run = QtWidgets.QGroupBox("How it runs")
        form = QtWidgets.QFormLayout(run)
        form.addRow("Global speed", self.global_speed)
        hint = note("Feed rates (F) are only true at 100 %. Start low for a first run.")
        hint.setStyleSheet("color: #6b7280;")
        form.addRow("", hint)
        form.addRow("Rapid (G0) speed", self.rapid)
        form.addRow("Approach / retract height", self.approach)
        form.addRow("Output for M3 / M5 (DO)", self.spindle_do)

        self.message = QtWidgets.QLabel("Checking reads from the robot only. It does not move.")
        self.message.setWordWrap(True)
        buttons = QtWidgets.QDialogButtonBox()
        buttons.addButton("Check plan", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        buttons.addButton(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(place)
        outer.addWidget(hold)
        outer.addWidget(run)
        outer.addWidget(self.message)
        outer.addWidget(buttons)
        for button in (self.by_ref, self.by_here, self.by_auto, self.by_manual, self.auto_rotate):
            button.toggled.connect(self._enable)
        self._enable()

    def _enable(self):
        self.ref_point.setEnabled(self.by_ref.isChecked())
        self.auto_range.setEnabled(self.by_auto.isChecked())
        self.auto_rotate.setEnabled(self.by_auto.isChecked())
        self.yaw.setEnabled(not (self.by_auto.isChecked() and self.auto_rotate.isChecked()))
        self.manual_box.setEnabled(self.by_manual.isChecked())

    def setup(self) -> RunSetup:
        placement = (REF if self.by_ref.isChecked() else HERE if self.by_here.isChecked() else
                     AUTO if self.by_auto.isChecked() else MANUAL)
        return RunSetup(placement, self.ref_point.text().strip(), [w.value() for w in self.origin],
                        [w.value() for w in self.rpy], self.tool.value(), self.user.value(),
                        self.global_speed.value(), self.rapid.value() or None, self.approach.value(),
                        self.spindle_do.value(), self.auto_range.value(), self.auto_rotate.isChecked(),
                        self.yaw.value(), self.turn_posture.isChecked(), self.tool_down.isChecked())

    def accept(self):
        if self.by_ref.isChecked() and not self.ref_point.text().strip():
            self.message.setText("Type the name of the reference teach point saved on the robot.")
            self.message.setStyleSheet("color: #d62728;")
            self.ref_point.setFocus()
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
            streamer.on_command = lambda c, text: self.command.emit(index.get(id(c), -1), text)
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
                self.log.emit(f"Starting. Global speed {a.global_speed} %.")
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
