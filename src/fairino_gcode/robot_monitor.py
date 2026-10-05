#!/usr/bin/env python3
"""
robot_monitor.py - live robot status for the GUI.

Reads the controller's CNDE status stream with the CNDEClient from fairino_CNDE_listener.py (that
script is used as it is, not modified) in a background thread, and hands every sample to the GUI
through Qt signals. Nothing in here commands the robot: it only listens.

    monitor = RobotMonitor(Target("192.168.58.25"))
    monitor.sample.connect(on_sample)      # dict, e.g. {"actual_TCP_pos": [x, y, z, rx, ry, rz], ...}
    monitor.start()
    ...
    monitor.stop()
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from matplotlib.backends.qt_compat import QtCore

from . import fairino_CNDE_listener as cnde

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal

TCP = "actual_TCP_pos"

# What to ask the controller for, best first. A controller refuses the whole configuration if it does
# not know one of the names, so on a refusal the next, smaller set is tried.
ITEM_SETS: List[List[str]] = [
    # everything the status panel can show (names as in the SDK's CNDE state table)
    [TCP, "actual_TCP_cmpvel", "tool_id", "wobj_id", "robot_mode", "robot_state", "program_state",
     "motion_done", "main_code", "sub_code", "emergency_stop"],
    # the names fairino_CNDE_listener.py asks for by default
    [TCP, "robot_mode", "program_state", "motion_done", "main_code", "sub_code", "emergency_stop"],
    [TCP],
]

ROBOT_MODE = {0: "Auto", 1: "Manual"}
ROBOT_STATE = {1: "Stopped", 2: "Moving", 3: "Paused", 4: "Drag"}
PROGRAM_STATE = {1: "Stopped", 2: "Running", 3: "Paused"}

DEFAULT_IP = "192.168.58.25"
UDP_PORT, TCP_PORT = 20006, 20005
STALE_AFTER_S = 2.0               # no status frame for this long -> the picture is no longer live


def label(table: Dict[int, str], value) -> str:
    """'Running (2)' - the meaning and the raw number, so an unknown code is still readable."""
    return "-" if value is None else f"{table.get(value, 'unknown')} ({value})"


@dataclass(frozen=True)
class Target:
    """Where and how to listen."""
    ip: str = DEFAULT_IP
    use_tcp: bool = False
    port: Optional[int] = None    # None = the protocol's default port
    period_ms: int = 50           # 1-200; 50 ms = 20 samples per second

    @property
    def resolved_port(self) -> int:
        return self.port or (TCP_PORT if self.use_tcp else UDP_PORT)

    def __str__(self) -> str:
        return f"{self.ip}:{self.resolved_port} ({'TCP' if self.use_tcp else 'UDP'})"


class RobotMonitor(QtCore.QThread):
    """Listens to one robot. Emits `connected` or `failed` once, then `sample` for every frame."""

    connected = Signal(str)       # what was agreed with the controller (for the log / tooltip)
    failed = Signal(str)          # could not start listening; the thread has ended
    sample = Signal(object)       # dict: item name -> value
    stale = Signal(bool)          # True: the stream went quiet; False: it is back
    lost = Signal(str)            # the connection broke after it had been working; thread has ended
    robot_message = Signal(str)   # text the controller sent on its own

    def __init__(self, target: Target, parent=None):
        super().__init__(parent)
        self.target = target
        self.items: List[str] = []
        self._stop = threading.Event()

    def stop(self, wait_ms: int = 3000) -> None:
        """Stop listening and wait for the thread to finish."""
        self._stop.set()
        if self.isRunning():
            self.wait(wait_ms)

    # ---- thread
    def run(self) -> None:
        t = self.target
        client = None
        try:
            client = cnde.CNDEClient(t.ip, t.resolved_port, use_tcp=t.use_tcp)
            client.drain()
            client.stop_output()                  # in case an earlier session left the stream running
            client.drain()
            layout = self._configure(client)
            client.start_output()
        except (OSError, RuntimeError, ValueError) as exc:     # OSError covers timeouts and refusals
            if client is not None:
                client.close()
            self.failed.emit(str(exc) or type(exc).__name__)
            return

        self.connected.emit(f"{t}, every {t.period_ms} ms: " + ", ".join(self.items))
        try:
            self._listen(client, layout)
        except OSError as exc:
            self.lost.emit(str(exc) or type(exc).__name__)
        finally:
            client.stop_output()
            client.close()

    def _configure(self, client) -> "cnde.Layout":
        refusal: Optional[Exception] = None
        for items in ITEM_SETS:
            if self._stop.is_set():
                break
            try:
                layout = client.configure_output(items, self.target.period_ms)
            except TimeoutError:
                raise                              # nobody answered: a smaller set will not help
            except RuntimeError as exc:            # the controller answered "no" - try fewer items
                refusal = exc
                continue
            self.items = list(items)
            return layout
        raise RuntimeError(str(refusal) if refusal else "cancelled")

    def _listen(self, client, layout) -> None:
        wanted = {cnde.FT_OUTPUT_DATA, cnde.FT_MESSAGE}
        last_frame, is_stale = time.monotonic(), False
        while not self._stop.is_set():
            frame = client.recv_frame(wanted, timeout=0.25)
            if frame is None:
                if not is_stale and time.monotonic() - last_frame > STALE_AFTER_S:
                    is_stale = True
                    self.stale.emit(True)
                continue
            _count, ftype, content = frame
            if ftype == cnde.FT_MESSAGE:
                mtype, text = client._decode_message(content)
                self.robot_message.emit(f"[{cnde.MSG_TYPE_NAMES.get(mtype, mtype)}] {text}")
                continue
            try:
                data = layout.unpack(content)
            except ValueError:
                continue                           # damaged frame - the next one is 50 ms away
            last_frame = time.monotonic()
            if is_stale:
                is_stale = False
                self.stale.emit(False)
            self.sample.emit(data)
