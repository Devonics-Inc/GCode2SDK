#!/usr/bin/env python3
"""
robot_widgets.py - the GUI pieces for the robot connection:

    ConnectDialog      asks for the robot's IP address and starts listening to it
    RobotStatusPanel   shows the live TCP coordinates and the robot's state
"""
from __future__ import annotations

import ipaddress
from typing import Optional

from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets

from .robot_monitor import (DEFAULT_IP, PROGRAM_STATE, ROBOT_MODE, ROBOT_STATE, TCP, TCP_PORT, UDP_PORT,
                           RobotMonitor, Target, label)

Qt = QtCore.Qt

GREEN, AMBER, RED, GREY = "#1a9850", "#e08a00", "#d62728", "#8a8f98"


def _settings() -> QtCore.QSettings:
    return QtCore.QSettings("FAIRINO", "gcode-runner")


# ============================================================================ connect
class ConnectDialog(QtWidgets.QDialog):
    """Asks where the robot is and tries to listen to it. The dialog only closes with "accepted"
    once status data is really arriving; `monitor` is then the running RobotMonitor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to robot")
        self.setModal(True)
        self.monitor: Optional[RobotMonitor] = None

        self.ip = QtWidgets.QLineEdit(str(_settings().value("ip", DEFAULT_IP)))
        self.ip.setPlaceholderText("e.g. 192.168.58.2")
        self.ip.selectAll()
        self.protocol = QtWidgets.QComboBox()
        self.protocol.addItem(f"UDP (port {UDP_PORT})", False)
        self.protocol.addItem(f"TCP (port {TCP_PORT})", True)
        self.port = QtWidgets.QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(UDP_PORT)
        self.protocol.currentIndexChanged.connect(
            lambda _i: self.port.setValue(TCP_PORT if self.protocol.currentData() else UDP_PORT))
        self.period = QtWidgets.QSpinBox()
        self.period.setRange(10, 200)
        self.period.setSingleStep(10)
        self.period.setValue(int(_settings().value("period_ms", 50)))
        self.period.setSuffix(" ms")
        self.period.setToolTip("How often the robot sends its status. 50 ms = 20 updates per second.")

        form = QtWidgets.QFormLayout()
        form.addRow("Robot IP address", self.ip)
        form.addRow("Protocol", self.protocol)
        form.addRow("Port", self.port)
        form.addRow("Update every", self.period)

        self.note = QtWidgets.QLabel("This only reads the robot's status. It does not move the robot.")
        self.note.setWordWrap(True)
        self.note.setMinimumWidth(360)

        self.buttons = QtWidgets.QDialogButtonBox()
        self.connect_btn = self.buttons.addButton("Connect", QtWidgets.QDialogButtonBox.ButtonRole.AcceptRole)
        self.buttons.addButton(QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.try_connect)
        self.buttons.rejected.connect(self.reject)

        box = QtWidgets.QVBoxLayout(self)
        box.addLayout(form)
        box.addWidget(self.note)
        box.addWidget(self.buttons)

    def target(self) -> Target:
        """The form as a Target. Raises ValueError if the IP address is not valid."""
        ip = self.ip.text().strip()
        ipaddress.IPv4Address(ip)
        return Target(ip, bool(self.protocol.currentData()), self.port.value(), self.period.value())

    def try_connect(self):
        try:
            target = self.target()
        except ValueError:
            self._say(f"\u201c{self.ip.text().strip()}\u201d is not an IP address. Example: 192.168.58.2", RED)
            self.ip.setFocus()
            return
        self._busy(True)
        self._say(f"Connecting to {target} \u2026", GREY)
        self.monitor = RobotMonitor(target)
        self.monitor.connected.connect(self._on_connected)
        self.monitor.failed.connect(self._on_failed)
        self.monitor.start()

    def _on_connected(self, _description: str):
        s = _settings()
        s.setValue("ip", self.monitor.target.ip)
        s.setValue("period_ms", self.monitor.target.period_ms)
        self.accept()

    def _on_failed(self, reason: str):
        self.monitor = None
        self._busy(False)
        self._say("Could not connect: " + reason, RED)

    def reject(self):                      # Cancel / Esc / window close, also while connecting
        if self.monitor is not None:
            self.monitor.connected.disconnect(self._on_connected)
            self.monitor.failed.disconnect(self._on_failed)
            self.monitor.stop()
            self.monitor = None
        super().reject()

    def _busy(self, busy: bool):
        for w in (self.ip, self.protocol, self.port, self.period, self.connect_btn):
            w.setEnabled(not busy)

    def _say(self, text: str, colour: str):
        self.note.setText(text)
        self.note.setStyleSheet(f"color: {colour};")


# ============================================================================ status panel
class RobotStatusPanel(QtWidgets.QGroupBox):
    """Live TCP coordinates and robot state. Fed with the samples of a RobotMonitor."""

    def __init__(self, parent=None):
        super().__init__("Robot", parent)
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        big = QtGui.QFont(mono)
        big.setPointSizeF(mono.pointSizeF() * 1.25)
        big.setBold(True)

        self.link = QtWidgets.QLabel()
        self.link.setTextFormat(Qt.TextFormat.RichText)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        self.pose = []
        for col, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                                            ("RX", "\u00b0"), ("RY", "\u00b0"), ("RZ", "\u00b0")]):
            head = QtWidgets.QLabel(f"{name} ({unit})")
            head.setAlignment(Qt.AlignmentFlag.AlignRight)
            head.setStyleSheet(f"color: {GREY};")
            value = QtWidgets.QLabel("-")
            value.setFont(big if col < 3 else mono)
            value.setAlignment(Qt.AlignmentFlag.AlignRight)
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            grid.addWidget(head, 0, col)
            grid.addWidget(value, 1, col)
            self.pose.append(value)

        self.state = QtWidgets.QLabel()
        self.state.setWordWrap(True)
        self.fault = QtWidgets.QLabel()
        self.fault.setWordWrap(True)

        box = QtWidgets.QVBoxLayout(self)
        box.addWidget(self.link)
        box.addLayout(grid)
        box.addWidget(self.state)
        box.addWidget(self.fault)
        self.set_disconnected()

    # ---- connection state
    def _link(self, colour: str, text: str):
        self.link.setText(f'<span style="color:{colour};">\u25cf</span> {text}')

    def set_connected(self, target: Target):
        self._target = target
        self._link(GREEN, f"Connected to <b>{target.ip}</b> &nbsp;<span style='color:{GREY};'>"
                          f"{'TCP' if target.use_tcp else 'UDP'} {target.resolved_port}, "
                          f"{1000 / target.period_ms:.0f} updates/s</span>")

    def set_stale(self, stale: bool):
        if stale:
            self._link(AMBER, f"No data from <b>{self._target.ip}</b> - the values below are old")
        else:
            self.set_connected(self._target)

    def set_disconnected(self, reason: str = ""):
        self._target = None
        self._link(RED if reason else GREY, f"Connection lost: {reason}" if reason else "Not connected")
        for value in self.pose:
            value.setText("-")
        self.state.setText("")
        self.fault.setText("")

    # ---- data
    def show_sample(self, s: dict):
        for value, number in zip(self.pose, s.get(TCP, [])):
            value.setText(f"{number + 0.0:.3f}")
        parts = []
        if "tool_id" in s:
            parts.append(f"Tool {s['tool_id']}, workpiece {s.get('wobj_id', '?')}")
        if "robot_mode" in s:
            parts.append("Mode: " + label(ROBOT_MODE, s["robot_mode"]))
        if "robot_state" in s:
            parts.append("Robot: " + label(ROBOT_STATE, s["robot_state"]))
        if "program_state" in s:
            parts.append("Program: " + label(PROGRAM_STATE, s["program_state"]))
        if "motion_done" in s:
            parts.append("in position" if s["motion_done"] else "moving to target")
        if "actual_TCP_cmpvel" in s:
            parts.append(f"{s['actual_TCP_cmpvel'][0]:.1f} mm/s")
        self.state.setText("   |   ".join(parts))

        alarms = []
        if s.get("emergency_stop"):
            alarms.append("EMERGENCY STOP pressed")
        if s.get("main_code") or s.get("sub_code"):
            alarms.append(f"Fault: main code {s.get('main_code')}, sub code {s.get('sub_code')}")
        if alarms:
            self.fault.setText("\u26a0 " + "   ".join(alarms))
            self.fault.setStyleSheet(f"color: {RED}; font-weight: bold;")
        elif "main_code" in s:
            self.fault.setText("No fault")
            self.fault.setStyleSheet(f"color: {GREEN};")
        else:
            self.fault.setText("")
