#!/usr/bin/env python3
"""
robot_monitor.py - live robot status for the GUI.

Reads the controller's CNDE status stream with the CNDEClient from fairino_CNDE_listener.py (that
script is used as it is, not modified) in a background thread, and hands every sample to the GUI
through Qt signals. Nothing in here commands the robot: it only listens.

Two things are done here rather than with CNDEClient.recv_frame():
    FrameReader  every frame that arrives is used. recv_frame() returns the first frame of each datagram /
                 TCP read and drops the rest, and reads bunch up whenever this program is busy (drawing
                 the 3D view): that lost about half the samples of a fast run - the TCP trail of a circle
                 came out as a polygon, and the speed graph spiked.
    SampleClock  the time each sample was sent. Frames that waited while this program was busy are all
                 read at once; the last of them was sent just now, the one before it a period earlier, and
                 so on. With the arrival time alone they would all get the same time, and the speed graph
                 would spike. The controller's frame counter shows frames lost on the way (SampleClock.lost).

    monitor = RobotMonitor(Target("192.168.58.25"))
    monitor.sample.connect(on_sample)      # dict, e.g. {"actual_TCP_pos": [x, y, z, rx, ry, rz], ...}
    monitor.start()
    ...
    monitor.stop()
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional

from matplotlib.backends.qt_compat import QtCore

from . import fairino_CNDE_listener as cnde

Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal

TCP = "actual_TCP_pos"
RECEIVED = "_received"            # added to every sample: when it was sent, on this program's time.monotonic()
                                  # clock (SampleClock) - the speed graph's time axis

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


def split_frames(buf: bytes):
    """([(count, type, content), ...], rest): the complete frames in `buf`, and the bytes of a frame that has
    not fully arrived yet (TCP: they are completed by the next read)."""
    out, i, n = [], 0, len(buf)
    while True:
        start = buf.find(cnde.HEADER, i)
        if start < 0:
            return out, (buf[-1:] if buf[-1:] == cnde.HEADER[:1] else b"")
        if start + cnde.HEADER_LEN > n:
            return out, buf[start:]
        count, ftype, length = struct.unpack_from("<BBH", buf, start + 2)
        end = start + cnde.HEADER_LEN + length
        if end + cnde.TAIL_LEN > n:
            return out, buf[start:]
        if buf[end:end + cnde.TAIL_LEN] != cnde.TAIL:
            i = start + 2                      # not a frame after all: look for the next header
            continue
        out.append((count, ftype, buf[start + cnde.HEADER_LEN:end]))
        i = end + cnde.TAIL_LEN


class FrameReader:
    """Every frame from a CNDEClient's socket, in order, each with the time.monotonic() it was read at and how
    many frames came after it in the same read. A read takes everything that is waiting: one datagram or TCP
    chunk, and all that queued up behind it while this program was busy."""

    MAX_PENDING = 1 << 20                      # bytes of an unfinished TCP frame kept at most

    def __init__(self, client):
        self.client = client
        self._rest = b""
        self._frames: deque = deque()

    def next(self, timeout: float):
        """(arrived, count, type, content, behind), or None if nothing came within `timeout` s."""
        if self._frames:
            return self._frames.popleft()
        sock, deadline = self.client.sock, time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            sock.settimeout(remaining)
            try:
                if self.client.use_tcp:
                    data = sock.recv(65535)
                    if not data:
                        raise ConnectionError("robot closed the TCP connection")
                else:
                    data, addr = sock.recvfrom(65535)
                    if addr[0] != self.client.ip:
                        continue
            except socket.timeout:
                return None
            chunks = [data] + self._waiting()
            arrived = time.monotonic()
            frames = []
            if self.client.use_tcp:                # a stream: a frame can be split over two reads
                frames, self._rest = split_frames(self._rest + b"".join(chunks))
                if len(self._rest) > self.MAX_PENDING:
                    self._rest = b""
            else:                                  # each datagram holds whole frames
                for chunk in chunks:
                    frames += split_frames(chunk)[0]
            self._frames.extend((arrived, *f, len(frames) - 1 - k) for k, f in enumerate(frames))
            if self._frames:
                return self._frames.popleft()

    def _waiting(self) -> list:
        """Whatever else has already arrived (without waiting for more)."""
        sock, out = self.client.sock, []
        sock.settimeout(0.0)
        try:
            while True:
                if self.client.use_tcp:
                    data = sock.recv(65535)
                    if not data:
                        break                      # closed: the next blocking read reports it
                    out.append(data)
                else:
                    data, addr = sock.recvfrom(65535)
                    if addr[0] == self.client.ip:
                        out.append(data)
        except (BlockingIOError, socket.timeout, InterruptedError):
            pass
        return out


class SampleClock:
    """When each data frame was sent: the last frame of a read was sent about when it arrived, each one before
    it a period earlier (the controller sends one every period). Frames lost on the way show as gaps in the
    controller's frame counter (0-255)."""

    def __init__(self, period_s: float):
        self.period = float(period_s)
        self._t: Optional[float] = None
        self._count: Optional[int] = None
        self.lost = 0                          # frames the counter says never arrived

    def counted(self, count: int):
        """A frame that is not a sample (a message) used up a number of the counter."""
        self._count = count

    def stamp(self, count: int, arrived: float, behind: int = 0) -> float:
        """behind: frames read together with this one, after it."""
        if self._count is not None:
            steps = (count - self._count) & 0xFF
            if steps > 1:
                self.lost += steps - 1
        self._count = count
        t = arrived - behind * self.period
        if self._t is not None and t <= self._t:
            t = self._t + 1e-6                 # never before the previous sample
        self._t = t
        return t


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
        self.clock = SampleClock(target.period_ms / 1000.0)      # .lost: frames that never arrived

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
        reader = FrameReader(client)
        last_frame, is_stale = time.monotonic(), False
        while not self._stop.is_set():
            frame = reader.next(timeout=0.25)
            if frame is None:
                if not is_stale and time.monotonic() - last_frame > STALE_AFTER_S:
                    is_stale = True
                    self.stale.emit(True)
                continue
            arrived, count, ftype, content, behind = frame
            if ftype == cnde.FT_MESSAGE:
                self.clock.counted(count)
                mtype, text = client._decode_message(content)
                self.robot_message.emit(f"[{cnde.MSG_TYPE_NAMES.get(mtype, mtype)}] {text}")
                continue
            if ftype != cnde.FT_OUTPUT_DATA:
                continue
            try:
                data = layout.unpack(content)
            except ValueError:
                continue                           # damaged frame - the next one is a period away
            last_frame = arrived
            data[RECEIVED] = self.clock.stamp(count, arrived, behind)    # the speed graph's time axis
            if is_stale:
                is_stale = False
                self.stale.emit(False)
            self.sample.emit(data)
