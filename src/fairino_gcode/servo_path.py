#!/usr/bin/env python3
"""
servo_path.py - the fixed-speed (ServoJ) path: where the TCP is every servo cycle.

In the normal modes every G1 / G2 / G3 becomes one MoveL / MoveC / Circle and the controller plans the
speed of each one on its own: it slows down at every hand-over between two commands, so the TCP speed
dips at every junction. In the fixed-speed mode the app plans the motion itself and sends the arm a
joint position every servo cycle (ServoJ, 8 ms): the speed is then what this module computes.

    groups      every run of consecutive feed moves (G1 / G2 / G3) with nothing in between is one
                continuous path. G0 (MoveJ), dwells, M-codes and pauses end a group: the arm stops
                there anyway, and they are sent as in the other modes.
    corners     a sharp corner cannot be taken at constant speed (it would need infinite acceleration).
                Each one is rounded with the radius that the acceleration allows at the speed:
                r = v^2 / a. The rounding stays inside 45 % of the shorter of the two moves; where that
                is too little, the TCP slows down for that corner (and the report says where).
    profile     the speed along the path: the fixed speed v, but never more than sqrt(a * R) where the
                path curves with radius R, starting and ending at rest, with acceleration a.
                Arcs from the G-code keep their radius; a tight one may need a lower speed too.
    samples     the time along the path, sampled every servo cycle: one joint position per cycle.
                Joint positions come from the controller's inverse kinematics at "knots" about 1 mm
                apart (and on every arc point) and are interpolated in between.

No robot and no Qt in here: the inverse kinematics is passed in as a function.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from . import gcode_to_fairino as eng

CYCLE_S = 0.008          # ServoJ command period (s) - the SDK's default and FAIRINO's examples
KNOT_MM = 1.0            # inverse kinematics at least every KNOT_MM along the path
ARC_TOL_MM = 0.005       # arcs are followed with chords at most this far from the true arc
SHARP_DEG = 2.0          # a change of direction above this between two moves is a corner to round
REVERSAL_DEG = 170.0     # ...unless it turns back on itself: then the TCP stops there and turns


@dataclass
class ServoGroup:
    first: int                          # index into cmds of the first feed command of the group
    last: int                           # ...and of the last
    xyz: np.ndarray                     # (K, 3) knots along the path, robot frame, mm
    rpy: np.ndarray                     # (K, 3) orientation at each knot, deg
    owner: np.ndarray                   # (K,) index into cmds of the command each knot belongs to
    s: np.ndarray = None                # (K,) distance along the path, mm
    v: np.ndarray = None                # (K,) planned speed, mm/s
    t: np.ndarray = None                # (K,) planned time, s
    corners: int = 0                    # corners rounded
    corner_dev: float = 0.0             # mm, the most a rounded corner stays away from the G-code corner
    slow: List[Tuple[int, float]] = field(default_factory=list)   # (cmd index, lowest mm/s) where < v
    joints: Optional[np.ndarray] = None # (K, 6) filled in by solve_joints()

    @property
    def length(self) -> float:
        return float(self.s[-1]) if self.s is not None and len(self.s) else 0.0

    @property
    def seconds(self) -> float:
        return float(self.t[-1]) if self.t is not None and len(self.t) else 0.0


def is_feed(c) -> bool:
    """A move the fixed-speed mode streams: G1 / G2 / G3 (not a G0, which stays a MoveJ / MoveL)."""
    return c.kind in eng.MOTION_KINDS and not c.rapid and c.instr != "MoveJ"


def feed_groups(cmds) -> List[Tuple[int, int]]:
    """(first, last) command indices of every run of consecutive feed moves."""
    out, start = [], None
    for k, c in enumerate(cmds):
        if is_feed(c):
            if start is None:
                start = k
        else:
            if start is not None:
                out.append((start, k - 1))
            start = None
    if start is not None:
        out.append((start, len(cmds) - 1))
    return out


# ============================================================================ geometry
def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def _lerp_rpy(a, b, f):
    return [a[i] + _wrap(b[i] - a[i]) * f for i in range(3)]


def _command_points(c, frame, start_rpy):
    """Points (xyz, rpy) along one command, from just after its start to its end, in the robot frame.
    Orientation goes through the command's target poses at their place along the path."""
    if c.arc:
        pts = eng.polyline(c, step_deg=_arc_step(c, frame))
    else:
        pts = [c.start, c.end]
    xyz = [frame.pose(p)[:3] for p in pts]
    # where the target orientations are along the command
    if c.kind == "circle":
        keys = [(0.25, c.poses[0][3:]), (0.5, c.poses[1][3:]), (1.0, list(start_rpy))]
    elif c.kind == "arc":
        keys = [(0.5, c.poses[0][3:]), (1.0, c.poses[1][3:])]
    else:
        keys = [(1.0, c.poses[0][3:])]
    keys = [(0.0, list(start_rpy))] + keys
    seg = [math.dist(p, q) for p, q in zip(xyz, xyz[1:])]
    total = sum(seg) or 1.0
    out, done = [], 0.0
    for p, d in zip(xyz[1:], seg):
        done += d
        f = done / total
        k = next(i for i in range(1, len(keys)) if f <= keys[i][0] + 1e-12 or i == len(keys) - 1)
        (f0, r0), (f1, r1) = keys[k - 1], keys[k]
        out.append((p, _lerp_rpy(r0, r1, (f - f0) / (f1 - f0) if f1 > f0 else 1.0)))
    return out


def _along(p, s, at):
    """Point at distance `at` along polyline p (cumulative distances s) and the unit direction there."""
    i = int(np.clip(np.searchsorted(s, at, side="right") - 1, 0, len(p) - 2))
    while i < len(p) - 2 and s[i + 1] - s[i] < 1e-12:
        i += 1
    d = p[i + 1] - p[i]
    n = float(np.linalg.norm(d)) or 1e-12
    f = (at - s[i]) / n
    return p[i] + d * f, d / n


def _arc_step(c, frame) -> float:
    """Degrees between two points of an arc as _command_points samples it (0 for a line)."""
    if not c.arc:
        return 0.0
    radius = abs(c.arc[2]) * abs(frame.scale)
    step = math.degrees(2.0 * math.acos(max(-1.0, 1.0 - ARC_TOL_MM / max(radius, 1e-9))))
    return min(5.0, max(0.25, step))


def _round_corners(pts, rpys, owners, speed, acc, radius_of=None, slack_of=None):
    """Round every sharp corner between two commands with radius speed^2 / acc, or less where there is no
    room: the rounding takes at most 45 % of either command (so two roundings on one command never
    overlap) and, next to an arc, at most a quarter of its radius (the rounding is computed from the
    directions at the corner, which is exact for a line and very nearly so that close to an arc).
    radius_of: {command index: arc radius in mm} for the arcs.
    slack_of: {command index: degrees} - an arc's first / last chord points half a chord angle away from the
    arc's real direction, so a line that runs on tangentially into it seems to turn by that much: it is not
    a corner.
    Returns new (pts, rpys, owners, corners rounded, largest distance from a corner)."""
    p = [np.asarray(x, dtype=float) for x in pts]
    n = len(p)
    seg = [float(np.linalg.norm(p[i + 1] - p[i])) for i in range(n - 1)]
    s = np.concatenate([[0.0], np.cumsum(seg)])
    length = {}                                             # how long each command is along the path
    for i in range(1, n):
        length[owners[i]] = length.get(owners[i], 0.0) + seg[i - 1]
    radius_of = radius_of or {}
    want = speed * speed / max(acc, 1e-9)
    fillets = {}                                            # vertex index -> (d, points)
    worst = 0.0
    for i in range(1, n - 1):
        # only the junctions between two commands are corners; inside an arc the bend between two of its
        # chords is the arc itself, and the speed profile keeps to sqrt(a * R) there
        if owners[i] == owners[i + 1] or seg[i - 1] < 1e-9 or seg[i] < 1e-9:
            continue
        uh, wh = (p[i] - p[i - 1]) / seg[i - 1], (p[i + 1] - p[i]) / seg[i]
        turn = math.degrees(math.acos(float(np.clip(np.dot(uh, wh), -1.0, 1.0))))
        slack = (slack_of or {}).get(owners[i], 0.0) + (slack_of or {}).get(owners[i + 1], 0.0)
        if turn <= SHARP_DEG + slack or turn >= REVERSAL_DEG:
            continue
        half = math.radians(turn) / 2.0
        room = 0.45 * min(length[owners[i]], length[owners[i + 1]])
        for k in (owners[i], owners[i + 1]):
            if radius_of.get(k):
                room = min(room, 0.25 * radius_of[k])
        d = min(want * math.tan(half), room)                   # corner -> where the rounding starts / ends
        if d < 1e-6:
            continue
        r = d / math.tan(half)
        # the rounding starts and ends ON the path, d before and after the corner, in the direction the path
        # has there (exact next to a line; next to an arc it follows the arc's own direction, so the
        # rounding joins it without a kink). A cubic Bezier with these end directions and handles
        # 4/3 tan(turn / 4) r long is a circular arc of radius r next to lines (to 0.03 %).
        a, ta = _along(p, s, s[i] - d)
        b, tb = _along(p, s, s[i] + d)
        bend = math.acos(float(np.clip(np.dot(ta, tb), -1.0, 1.0)))
        h = 4.0 / 3.0 * math.tan(bend / 4.0) * (0.5 * float(np.linalg.norm(b - a)) / max(math.sin(bend / 2.0), 1e-9))
        c1, c2 = a + ta * h, b - tb * h
        m = max(2, math.ceil(math.degrees(bend) / 5.0))
        arc = []
        for k in range(m + 1):
            f = k / m
            q = ((1 - f) ** 3) * a + 3 * ((1 - f) ** 2) * f * c1 + 3 * (1 - f) * f * f * c2 + (f ** 3) * b
            arc.append((q, owners[i] if f < 0.5 else owners[i + 1]))
        fillets[i] = (d, arc)
        worst = max(worst, float(np.linalg.norm(arc[m // 2][0] - p[i])) if m % 2 == 0 else
                    r * (1.0 / math.cos(half) - 1.0))
    out_p, out_r, out_o = [], [], []
    for i in range(n):
        if i in fillets:
            for q, own in fillets[i][1]:
                out_p.append(q), out_r.append(rpys[i]), out_o.append(own)
            continue
        # a point the rounding of a nearby corner replaces is left out
        if any(abs(s[i] - s[j]) < fillets[j][0] - 1e-9 for j in fillets):
            continue
        out_p.append(p[i]), out_r.append(rpys[i]), out_o.append(owners[i])
    return out_p, out_r, out_o, len(fillets), worst


def _resample(pts, rpys, owners, spacing):
    """Insert points so that no two neighbours are more than `spacing` apart (all points are kept)."""
    xyz, rpy, own = [pts[0]], [rpys[0]], [owners[0]]
    for i in range(1, len(pts)):
        a, b = np.asarray(pts[i - 1]), np.asarray(pts[i])
        d = float(np.linalg.norm(b - a))
        if d < 1e-9:
            continue
        n = max(1, math.ceil(d / spacing - 1e-9))
        for k in range(1, n + 1):
            f = k / n
            xyz.append(a + (b - a) * f)
            rpy.append(_lerp_rpy(rpys[i - 1], rpys[i], f))
            own.append(owners[i])
    return np.array(xyz), np.array(rpy), np.array(own)


def build_group(cmds, first: int, last: int, frame, speed: float, acc: float,
                start_rpy: Optional[Sequence[float]] = None) -> ServoGroup:
    """The fixed-speed path of commands first..last (all feed moves), planned at `speed` mm/s with
    acceleration `acc` mm/s^2. start_rpy: orientation at the start (the end of the previous move)."""
    c0 = cmds[first]
    rpy0 = list(start_rpy) if start_rpy is not None else list(c0.poses[0][3:])
    pts, rpys, owners = [frame.pose(c0.start)[:3]], [rpy0], [first]
    for k in range(first, last + 1):
        for p, r in _command_points(cmds[k], frame, rpys[-1]):
            pts.append(p)
            rpys.append(r)
            owners.append(k)
    radius_of = {k: abs(cmds[k].arc[2]) * abs(frame.scale) for k in range(first, last + 1) if cmds[k].arc}
    slack_of = {k: 0.5 * _arc_step(cmds[k], frame) for k in range(first, last + 1) if cmds[k].arc}
    pts, rpys, owners, corners, dev = _round_corners(pts, rpys, owners, speed, acc, radius_of, slack_of)
    xyz, rpy, own = _resample(pts, rpys, owners, KNOT_MM)
    g = ServoGroup(first, last, xyz, rpy, own, corners=corners, corner_dev=dev)
    plan_profile(g, speed, acc)
    return g


# ============================================================================ speed along the path
def plan_profile(g: ServoGroup, speed: float, acc: float):
    """Fill in g.s, g.v, g.t: the speed and the time along the path (and g.slow)."""
    d = np.linalg.norm(np.diff(g.xyz, axis=0), axis=1)
    g.s = np.concatenate([[0.0], np.cumsum(d)])
    n = len(g.s)
    limit = np.full(n, float(speed))
    # where the path curves: turn angle at each knot -> radius -> speed that acceleration allows
    for i in range(1, n - 1):
        u, w = g.xyz[i] - g.xyz[i - 1], g.xyz[i + 1] - g.xyz[i]
        lu, lw = np.linalg.norm(u), np.linalg.norm(w)
        if lu < 1e-12 or lw < 1e-12:
            continue
        turn = math.acos(float(np.clip(np.dot(u, w) / (lu * lw), -1.0, 1.0)))
        if turn > 1e-6:
            radius = 0.5 * (lu + lw) / turn
            limit[i] = min(limit[i], math.sqrt(acc * radius))
    v = limit.copy()
    v[0] = v[-1] = 0.0
    for i in range(1, n):                                  # can only speed up at acc...
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * acc * d[i - 1]))
    for i in range(n - 2, -1, -1):                         # ...and only slow down at acc
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * acc * d[i]))
    g.v = v
    mean = 0.5 * (v[1:] + v[:-1])
    dt = np.where(mean > 1e-9, d / np.maximum(mean, 1e-9), 0.0)
    g.t = np.concatenate([[0.0], np.cumsum(dt)])
    # where the path forces the speed below the fixed speed (not counting the start and end ramps)
    slow = {}
    for i in np.nonzero(limit < speed * 0.98)[0]:
        k = int(g.owner[i])
        slow[k] = min(slow.get(k, speed), float(limit[i]))
    g.slow = sorted(slow.items())


def ticks(g: ServoGroup, cycle: float = CYCLE_S):
    """Distance along the path (mm) at every servo cycle after the start, ending exactly at the end.
    Between two knots the speed changes at constant acceleration (that is how the profile was planned),
    so the distance is v0 * tau + a * tau^2 / 2, not a straight line in time."""
    n = max(1, math.ceil(g.seconds / cycle - 1e-9))
    t = np.minimum(np.arange(1, n + 1) * cycle, g.seconds)
    i = np.clip(np.searchsorted(g.t, t, side="right") - 1, 0, len(g.t) - 2)
    d = g.s[i + 1] - g.s[i]
    v0, v1 = g.v[i], g.v[i + 1]
    a = np.where(d > 1e-12, (v1 * v1 - v0 * v0) / (2.0 * np.maximum(d, 1e-12)), 0.0)
    tau = t - g.t[i]
    s = g.s[i] + v0 * tau + 0.5 * a * tau * tau
    return np.clip(np.minimum(s, g.s[i + 1]), 0.0, g.s[-1])


def joints_at(g: ServoGroup, s: np.ndarray) -> np.ndarray:
    """(N, 6) joint positions at distances s along the path, interpolated between the knots."""
    return np.column_stack([np.interp(s, g.s, g.joints[:, j]) for j in range(6)])


def solve_joints(g: ServoGroup, ik: Callable[[List[float], int], List[float]], progress=None):
    """Joint positions at every knot from ik(pose, cmd index), which must reference the previous solution
    (so the arm stays on one configuration). progress(done, total) is called now and then."""
    out = []
    for i, (p, r) in enumerate(zip(g.xyz, g.rpy)):
        out.append(ik([float(v) for v in p] + [float(v) for v in r], int(g.owner[i])))
        if progress is not None and i % 200 == 0:
            progress(i, len(g.xyz))
    g.joints = np.asarray(out, dtype=float)


def joint_check(g: ServoGroup, cycle: float = CYCLE_S):
    """(largest joint step between two knots in deg, fastest joint speed in deg/s, knot index of it)."""
    q = g.joints
    step = np.abs((np.diff(q, axis=0) + 180.0) % 360.0 - 180.0)
    dt = np.maximum(np.diff(g.t), cycle)
    speed = step.max(axis=1) / dt
    i = int(np.argmax(speed)) if len(speed) else 0
    return (float(step.max()) if step.size else 0.0), (float(speed.max()) if speed.size else 0.0), i


def describe_groups(groups: Sequence[ServoGroup], cmds, speed: float) -> List[str]:
    """Plan summary lines: what the fixed-speed mode will do with this file."""
    if not groups:
        return ["Fixed speed     : no G1 / G2 / G3 moves to stream"]
    length = sum(g.length for g in groups)
    held = 0.0
    for g in groups:
        at = np.abs(g.v - speed) <= 0.02 * speed
        d = np.diff(g.s)
        held += float(np.sum(d[at[1:] & at[:-1]]))
    corners = sum(g.corners for g in groups)
    dev = max(g.corner_dev for g in groups)
    lines = [f"Fixed speed     : {speed:g} mm/s on {100.0 * held / max(length, 1e-9):.0f} % of {length:.1f} mm "
             f"of G1/G2/G3 in {len(groups)} continuous path(s) (the rest: starting and stopping)"]
    if corners:
        lines.append(f"Corners rounded : {corners}, at most {dev:.2f} mm inside the G-code corner")
    slow = [(k, v) for g in groups for k, v in g.slow]
    if slow:
        worst = sorted(slow, key=lambda kv: kv[1])[:5]
        lines.append("Slower at       : " + ", ".join(f"line {cmds[k].line} {v:.1f} mm/s" for k, v in worst)
                     + (f" (+{len(slow) - 5} more)" if len(slow) > 5 else "")
                     + " - tight curve for the acceleration")
    return lines
