#!/usr/bin/env python3
"""
speed_monitor.py - how fast the TCP really moves, and how steady that speed is.

Every move of a job runs at one speed (the global speed %), so the ideal is a flat speed curve. This
records the TCP speed from the robot's status stream and measures how far it strays from that:

    reference    the speed the TCP should hold. By default the median of the speeds while moving
                 (the robot's cruise speed: the TCP speed at a given % is not known exactly in
                 advance), or a value the operator types in.
    tolerance    a band of +/- tol % round the reference. Time spent outside it while moving is
                 deviation: slowing down into corners, speeding up, ramps into and out of stops.
    stats        mean, standard deviation, share of the moving time inside the band, min / max, and
                 how many times the speed left the band.
    at targets   after a run: the speed with which the TCP went through every target point of the
                 plan (the lowest speed close to the point - a blended corner shows up as a dip).
                 Targets where the move ends exactly (before a MoveJ, an M-code, a dwell, the end)
                 are marked "stop": the arm halts there on purpose.

The speed is the status item actual_TCP_cmpvel (TCP speed, mm/s). If the controller does not send
it, the speed is worked out from the change in TCP position over about 0.2 s. While the TCP position
stands still (under STILL_MM in STILL_WINDOW_S) the speed is taken as 0: the reported speed of a robot
at rest is noise.

No Qt in here - speed_view.py draws it.
"""
from __future__ import annotations

import csv
import math
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import gcode_to_fairino as eng

TCP = "actual_TCP_pos"          # status item: [x, y, z, rx, ry, rz] (same name as robot_monitor.TCP)
SPEED = "actual_TCP_cmpvel"     # status item: [TCP speed mm/s, ...]
RECEIVED = "_received"          # added by robot_monitor.py: time.monotonic() when the frame was sent
MOVING_MM_S = 1.0               # slower than this = standing still (status noise is ~0.2 mm/s)
DIFF_WINDOW_S = 0.2             # speed from positions: over at least this much time
# Standstill filter. A robot at rest reports a TCP speed that jitters between 0 and ~1 mm/s while its
# position changes by ~0.001 mm. So "standing still" is decided from the position: if the TCP has not
# moved STILL_MM in the last STILL_WINDOW_S, the speed is 0 (any motion faster than ~0.2 mm/s shows).
STILL_MM = 0.05
STILL_WINDOW_S = 0.25
MAX_SAMPLES = 20 * 3600         # one hour at 20 samples per second
FEED, RAPID = "feed", "rapid"   # what the arm is doing: a G1/G2/G3 move, or a G0 (MoveJ)


@dataclass
class SpeedStats:
    now: Optional[float] = None         # latest speed, mm/s
    reference: Optional[float] = None   # what the speed is compared to, mm/s
    auto: bool = True                   # reference = median of the moving speeds
    tolerance: float = 10.0             # +/- % of the reference
    moving_s: float = 0.0               # time spent moving
    mean: Optional[float] = None        # all of these: over the moving samples only
    std: Optional[float] = None
    low: Optional[float] = None
    high: Optional[float] = None
    within: Optional[float] = None      # % of the moving time inside the band
    excursions: int = 0                 # times the speed left the band while moving
    source: str = "median"              # where the reference comes from: "typed", "commanded", "median"

    @property
    def cv(self) -> Optional[float]:
        """Standard deviation as a % of the mean - one number for "how steady"."""
        return None if not self.mean else 100.0 * self.std / self.mean


@dataclass
class TargetSpeed:
    speed: float                        # mm/s with which the TCP went through the target
    deviation: Optional[float]          # % against the reference (None: no reference yet)
    t: float                            # s since the start of the recording
    miss: float                         # mm, how close the TCP came to the target
    stop: bool                          # the move ends exactly there: the arm halts on purpose


class SpeedRecorder:
    """The speed samples since the last clear(): time (s), TCP position (mm), speed (mm/s)."""

    def __init__(self, max_samples: int = MAX_SAMPLES):
        self.t: deque = deque(maxlen=max_samples)
        self.xyz: deque = deque(maxlen=max_samples)
        self.v: deque = deque(maxlen=max_samples)
        self._t0: Optional[float] = None
        self._marks: List[Tuple[float, str]] = []      # (time, FEED / RAPID) - see mark()
        self.commanded: Optional[float] = None          # mm/s the job commands (mm/s mode), else None

    def __len__(self):
        return len(self.v)

    def clear(self):
        self.t.clear()
        self.xyz.clear()
        self.v.clear()
        self._t0 = None
        self._marks = []

    def mark(self, t: float, phase: str):
        """From time t (same clock as add()) the arm does a FEED move or a RAPID (G0 -> MoveJ).
        A MoveJ runs at the global % of the JOINT speed, so its TCP speed says nothing about how steady
        the cutting speed is: rapid stretches are drawn, but left out of the numbers."""
        self._marks.append((t, phase))

    def phases(self) -> List[str]:
        """The phase of every sample (FEED when nothing was marked: outside a run all motion counts)."""
        if not self._marks or self._t0 is None:
            return [FEED] * len(self.t)
        marks = sorted((t - self._t0, phase) for t, phase in self._marks)
        times = [t for t, _p in marks]
        return [marks[k - 1][1] if (k := bisect_right(times, t)) else FEED for t in self.t]

    def add(self, t: float, xyz: Sequence[float], v: Optional[float] = None):
        """One status sample. t in seconds (any clock). v None: work it out from the positions."""
        xyz = tuple(float(c) for c in xyz[:3])
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        if self.t and rel <= self.t[-1]:
            return                                       # same frame twice, or a clock step back
        if v is None:
            v = self._from_positions(rel, xyz)
        v = abs(float(v))
        if self._still(rel, xyz, v):
            v = 0.0                                      # status noise, not motion
        self.t.append(rel)
        self.xyz.append(xyz)
        self.v.append(v)

    def _still(self, t: float, xyz, v: float) -> bool:
        """True if the TCP has moved less than STILL_MM from any position of the last STILL_WINDOW_S."""
        if not self.t:
            return v < MOVING_MM_S                       # nothing to compare with yet
        moved = 0.0
        for k in range(len(self.t) - 1, -1, -1):          # newest first; stops after ~0.25 s of samples
            moved = max(moved, math.dist(xyz, self.xyz[k]))
            if moved >= STILL_MM:
                return False
            if t - self.t[k] >= STILL_WINDOW_S:
                break
        return True

    def add_sample(self, sample: dict, t: Optional[float] = None) -> bool:
        """A status-stream sample (dict). Returns False if it has no TCP position."""
        pose = sample.get(TCP)
        if pose is None:
            return False
        speed = sample.get(SPEED)
        if isinstance(speed, (list, tuple)):
            speed = speed[0] if speed else None
        self.add(sample.get(RECEIVED, t if t is not None else 0.0), pose, speed)
        return True

    def _from_positions(self, t: float, xyz) -> float:
        if not self.t:
            return 0.0
        k = len(self.t) - 1                              # go back until the gap is long enough
        while k > 0 and t - self.t[k] < DIFF_WINDOW_S:
            k -= 1
        dt = t - self.t[k]
        return math.dist(xyz, self.xyz[k]) / dt if dt > 0 else 0.0

    # ---- analysis
    def moving(self) -> List[int]:
        """The samples the numbers are about: moving, and not in a rapid."""
        return [k for k, (v, phase) in enumerate(zip(self.v, self.phases())) if v > MOVING_MM_S and phase != RAPID]

    def reference(self, custom: Optional[float] = None) -> Tuple[Optional[float], bool]:
        """(reference speed, True if it is the automatic median). A typed-in value first, then the speed the
        job commands (self.commanded, mm/s mode), else the median of the moving speeds."""
        if custom:
            return float(custom), False
        if self.commanded:
            return float(self.commanded), False
        moving = [self.v[k] for k in self.moving()]
        return (median(moving) if moving else None), True

    def reference_source(self, custom: Optional[float] = None) -> str:
        return "typed" if custom else "commanded" if self.commanded else "median"

    def stats(self, custom_reference: Optional[float] = None, tolerance: float = 10.0) -> SpeedStats:
        ref, auto = self.reference(custom_reference)
        s = SpeedStats(now=self.v[-1] if self.v else None, reference=ref, auto=auto, tolerance=tolerance,
                       source=self.reference_source(custom_reference))
        idx = self.moving()
        if not idx:
            return s
        v = [self.v[k] for k in idx]
        # each sample stands for the time to the next one (capped: a gap in the stream is not motion)
        dt = [min(self.t[k + 1] - self.t[k], 0.5) if k + 1 < len(self.t) else 0.0 for k in idx]
        total = sum(dt) or float(len(idx))
        weights = dt if sum(dt) else [1.0] * len(idx)
        s.moving_s = sum(dt)
        s.mean = sum(w * x for w, x in zip(weights, v)) / total
        s.std = math.sqrt(sum(w * (x - s.mean) ** 2 for w, x in zip(weights, v)) / total)
        s.low, s.high = min(v), max(v)
        if ref:
            band = ref * tolerance / 100.0
            inside = [abs(x - ref) <= band for x in v]
            s.within = 100.0 * sum(w for w, ok in zip(weights, inside) if ok) / total
            prev_k, prev_ok = None, True
            for k, ok in zip(idx, inside):
                if not ok and (prev_ok or prev_k != k - 1):   # a new stretch outside the band
                    s.excursions += 1
                prev_k, prev_ok = k, ok
        return s

    def step(self) -> float:
        """Typical distance (mm) between two moving samples."""
        idx = self.moving()
        gaps = [math.dist(self.xyz[k], self.xyz[k - 1]) for k in idx if k > 0]
        return median(gaps) if gaps else 0.0

    def at_targets(self, targets: Sequence[Tuple[int, Sequence[float], bool]], reference: Optional[float],
                   radius: Optional[float] = None) -> Dict[int, TargetSpeed]:
        """The speed through each target, for targets given in path order as (key, xyz, stop).

        The samples are walked once, in order: for each target, the sample closest to it (within
        `radius` mm) after the previous target's. The speed there is the lowest one within 2 * radius
        of the point - in a blended corner the TCP is slowest a little before or after the point itself -
        counting only samples nearer to this target than to the one before or after it, so a short move
        does not take on its neighbour's stop.
        Targets the TCP did not come near (a stopped run, a different placement) are left out."""
        n = len(self.v)
        if not n or not targets:
            return {}
        if radius is None:
            radius = max(2.0, 1.5 * self.step())
        points = [tuple(float(c) for c in p[:3]) for _key, p, _stop in targets]
        phases = self.phases()
        out: Dict[int, TargetSpeed] = {}
        i = 0
        for m, (key, _p, stop) in enumerate(targets):
            p = points[m]
            neighbours = [q for q in (points[m - 1] if m else None, points[m + 1] if m + 1 < len(points) else None)
                          if q is not None and q != p]

            def near(k):
                d = math.dist(self.xyz[k], p)
                return d <= 2 * radius and all(d <= math.dist(self.xyz[k], q) for q in neighbours)
            best, best_j, j = math.inf, None, i
            limit = min(n, i + 20 * 600)                 # look at most ~10 min ahead for one target
            while j < limit:
                d = math.dist(self.xyz[j], p)
                if d < best:
                    best, best_j = d, j
                elif best <= radius and d > best + 2 * radius:
                    break                                # went through it and moved on
                j += 1
            if best_j is None or best > radius:
                continue
            lo = hi = best_j
            while lo > i and near(lo - 1):
                lo -= 1
            while hi + 1 < n and near(hi + 1):
                hi += 1
            # a rapid (MoveJ) passing the point says nothing about the feed speed through it
            feed =[self.v[k] for k in range(lo, hi + 1) if phases[k] != RAPID]
            speed = min(feed) if feed else self.v[best_j]
            dev = None if not reference or stop else 100.0 * (speed - reference) / reference
            out[key] = TargetSpeed(speed, dev, self.t[best_j], best, stop)
            i = best_j
        return out


def plan_targets(plan) -> List[Tuple[int, List[float], bool]]:
    """The plan's target points in path order: (command index, xyz where that move ends, stop).
    stop: the move ends exactly on its point, as Streamer.run() sends it (not blended into the next)."""
    def feed_move(c):
        return c is not None and c.kind in eng.MOTION_KINDS and c.instr != "MoveJ"

    out = []
    for k in plan.motions:
        c = plan.cmds[k]
        nxt = plan.cmds[k + 1] if k + 1 < len(plan.cmds) else None
        out.append((k, [float(v) for v in plan.paths[k][-1]], not (feed_move(c) and feed_move(nxt))))
    return out


def target_summary(found: Dict[int, TargetSpeed], total: int, tolerance: float) -> str:
    """One line about the blended targets: how many were passed and the slowest one."""
    corners = [(k, t) for k, t in found.items() if not t.stop]
    if not found:
        return f"0 of {total} targets passed"
    text = f"{len(found)} of {total} targets passed"
    if corners and corners[0][1].deviation is not None:
        k, worst = min(corners, key=lambda item: item[1].speed)
        off = sum(1 for _k, t in corners if abs(t.deviation) > tolerance)
        text += (f"; {len(corners)} blended: slowest {worst.speed:.1f} mm/s ({worst.deviation:+.0f} %)"
                 f" at #{k + 1}, {off} outside ±{tolerance:g} %")
    return text


# ============================================================================ speed of every command
# Which command was the arm executing when a sample was taken? Not the one being SENT then: blended
# moves are queued up to --queue-max ahead. So each sample is matched by POSITION: to the nearest
# planned path of the feed commands, walking forward through the plan (never back).
MATCH_MM = 2.0          # + the blend radius: how far from the planned path a sample may be and still count
LOOK_AHEAD = 8          # commands ahead of the current one a sample may be matched to
SWITCH_MM = 0.2         # a later command must be this much nearer to take over (a path crossing itself
                        # near the current command does not pull the samples forward)


@dataclass
class CommandSpeed:
    """How fast the TCP went along one command, from the samples matched to it."""
    index: int                          # into plan.cmds
    samples: int
    t0: float                           # s since the start of the recording: first sample...
    t1: float                           # ...and last sample on this command
    entry: float                        # mm/s: speed of the first sample
    low: float
    mean: float
    high: float
    exit: float                         # speed of the last sample
    length: float                       # mm, planned path of the command
    on_speed: Optional[float]           # % of the samples within the band round the reference

    @property
    def seconds(self) -> float:
        return self.t1 - self.t0


def _segments(path):
    """A planned path (N, 3) as segment starts, directions and squared lengths, for distance queries."""
    p = np.asarray(path, dtype=float)
    a, d = p[:-1], p[1:] - p[:-1]
    dd = (d * d).sum(axis=1)
    dd[dd == 0.0] = 1e-12
    return a, d, dd


def _distance(point, seg) -> float:
    a, d, dd = seg
    t = np.clip(((point - a) * d).sum(axis=1) / dd, 0.0, 1.0)
    return float(np.sqrt((((a + d * t[:, None]) - point) ** 2).sum(axis=1)).min())


def command_speeds(rec: SpeedRecorder, plan, reference: Optional[float] = None, tolerance: float = 10.0):
    """The speed along every feed command (MoveL / MoveC / Circle) of a plan placed on the robot.
    Returns ({command index: CommandSpeed}, owner) where owner[j] is the command index sample j was
    matched to, or None (standing still, a rapid, or off the planned path)."""
    n = len(rec)
    owner: List[Optional[int]] = [None] * n
    feed = [k for k in plan.motions if plan.cmds[k].instr != "MoveJ" and len(plan.paths[k]) > 1]
    if not feed or not n:
        return {}, owner
    geo = [_segments(plan.paths[k]) for k in feed]
    tol = MATCH_MM + max(0.0, float(getattr(plan.settings, "blend", 0.0) or 0.0))
    phases = rec.phases()
    m = 0                                                   # the command the arm is on (index into feed)
    for j in range(n):
        if rec.v[j] < MOVING_MM_S or phases[j] == RAPID:
            continue
        point = np.asarray(rec.xyz[j], dtype=float)
        here = _distance(point, geo[m])
        best, best_i = here, m
        for i in range(m + 1, min(len(feed), m + LOOK_AHEAD + 1)):
            d = _distance(point, geo[i])
            if d < best - (SWITCH_MM if best_i == m else 0.0):
                best, best_i = d, i
        if best > tol:
            continue                                        # not on the planned path
        m = best_i
        owner[j] = feed[m]

    out: Dict[int, CommandSpeed] = {}
    by_cmd: Dict[int, List[int]] = {}
    for j, k in enumerate(owner):
        if k is not None:
            by_cmd.setdefault(k, []).append(j)
    for k, js in by_cmd.items():
        v = [rec.v[j] for j in js]
        path = np.asarray(plan.paths[k], dtype=float)
        on = (100.0 * sum(abs(x - reference) <= reference * tolerance / 100.0 for x in v) / len(v)
              if reference else None)
        out[k] = CommandSpeed(k, len(js), rec.t[js[0]], rec.t[js[-1]], v[0], min(v), sum(v) / len(v), max(v),
                              v[-1], float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()), on)
    return out, owner


def write_command_csv(path: str, plan, speeds: Dict[int, CommandSpeed]):
    """One row per motion command: what was sent, and how fast the TCP really went along it."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["#", "G-code line", "G-code", "Robot command", "Commanded", "Length (mm)", "Start (s)",
                    "Duration (s)", "Samples", "Entry (mm/s)", "Min (mm/s)", "Mean (mm/s)", "Max (mm/s)",
                    "Exit (mm/s)", "On speed (%)"])
        for k in plan.motions:
            c, s = plan.cmds[k], speeds.get(k)
            row = [k + 1, c.line, c.src, c.instr, eng.speed_text(c, plan.settings)]
            if s is None:
                row += [""] * 10
            else:
                row += [f"{s.length:.2f}", f"{s.t0:.3f}", f"{s.seconds:.3f}", s.samples, f"{s.entry:.2f}",
                        f"{s.low:.2f}", f"{s.mean:.2f}", f"{s.high:.2f}", f"{s.exit:.2f}",
                        "" if s.on_speed is None else f"{s.on_speed:.0f}"]
            w.writerow(row)


def write_samples_csv(path: str, rec: SpeedRecorder, owner: Sequence[Optional[int]], plan=None):
    """Every recorded sample: time, TCP position, speed, rapid or feed, and the command it was matched to."""
    phases = rec.phases()
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t (s)", "x (mm)", "y (mm)", "z (mm)", "speed (mm/s)", "phase", "#", "Robot command"])
        for j in range(len(rec)):
            k = owner[j] if j < len(owner) else None
            instr = plan.cmds[k].instr if (plan is not None and k is not None) else ""
            x, y, z = rec.xyz[j]
            w.writerow([f"{rec.t[j]:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{rec.v[j]:.2f}", phases[j],
                        "" if k is None else k + 1, instr])
