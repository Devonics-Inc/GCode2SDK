#!/usr/bin/env python3
"""
auto_place.py - choose where to put the drawing so that the robot can do it comfortably.

What is fixed and what is searched
    The height and the tool posture are taken from where the tool is now: jog the tool tip to the
    level of the work surface, in the posture to work in. G-code Z0 is put at that height. The
    position on that level (X and Y of the origin) is searched in a square around the tool tip, and
    the drawing may be turned in steps of 90 deg: a long part that points at the robot's own axis
    cannot be done, the same part turned sideways can.

What "best" means
    A placement is possible if the controller's own inverse kinematics finds a joint position for
    every tested point of the path. The candidates are then put through the same full check the run
    uses, best first, until one passes. Among the possible ones, the best is the one whose WORST point
    is the most comfortable, where comfort is the smallest of
        * the distance to the nearest joint limit,
        * the distance of joint 5 from 0 / 180 deg (wrist singularity),
        * the distance of joint 3 from 0 / 180 deg (arm fully stretched or folded).
    A point counts as fully comfortable from 45 deg to a joint limit and 30 deg to a singular
    position; more than that is not rewarded. Ties go to the placement with the better average,
    then to the one nearest to the tool - so the drawing is not sent across the workspace for nothing -
    and then to the one that is not turned.

What it does not know
    Tables, fixtures, cables, the robot's own base: nothing here checks for collisions. The result
    says where the robot can work well, not that the space is free. Look at the preview before Start.

The robot is only asked questions (inverse kinematics, joint limits). It does not move.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from . import gcode_to_fairino as eng

LIMIT_FULL, SINGULAR_FULL = 45.0, 30.0      # deg of margin that count as "fully comfortable"
DEFAULT_LIMITS = [(-175.0, 175.0)] * 6      # used if the controller does not report its soft limits


@dataclass
class Placement:
    origin: List[float]                      # where G-code 0,0,0 goes (mm, in the active frame)
    yaw: float = 0.0                         # how far the drawing is turned about Z (deg)
    comfort: float = 0.0                     # 0..1, the worst tested point
    limit_deg: float = 0.0                   # smallest distance to a joint limit over the tested points
    wrist_deg: float = 0.0                   # smallest distance of joint 5 from a wrist singularity
    elbow_deg: float = 0.0                   # smallest distance of joint 3 from stretched / folded
    tried: int = 0                           # placements examined
    possible: int = 0                        # ...of which every tested point was reachable
    mean: float = field(default=0.0, repr=False)


def joint_limits(robot) -> List[Tuple[float, float]]:
    """The controller's joint soft limits as [(low, high)] * 6, or generous defaults."""
    try:
        err, rest = eng.unpack(robot.GetJointSoftLimitDeg())
        v = [float(x) for x in rest[0]]
        if err != 0 or len(v) != 12:
            return list(DEFAULT_LIMITS)
    except Exception:                        # older SDK / simulator without this call
        return list(DEFAULT_LIMITS)
    if all(v[2 * k] < v[2 * k + 1] for k in range(6)):      # [j1 low, j1 high, j2 low, ...]
        return [(v[2 * k], v[2 * k + 1]) for k in range(6)]
    if all(v[k] < v[k + 6] for k in range(6)):               # [six lows, six highs]
        return [(v[k], v[k + 6]) for k in range(6)]
    return list(DEFAULT_LIMITS)


def margins(joints: Sequence[float], limits) -> Tuple[float, float, float]:
    """(deg to the nearest joint limit, deg of joint 5 from singular, deg of joint 3 from singular)."""
    def from_singular(angle):
        a = abs((angle + 180.0) % 360.0 - 180.0)
        return min(a, 180.0 - a)
    limit = min(min(q - lo, hi - q) for q, (lo, hi) in zip(joints, limits))
    return limit, from_singular(joints[4]), from_singular(joints[2])


def comfort_of(m: Tuple[float, float, float]) -> float:
    return max(0.0, min(1.0, m[0] / LIMIT_FULL, m[1] / SINGULAR_FULL, m[2] / SINGULAR_FULL))


def sample_targets(plan, approach: float = 0.0, most: int = 28) -> List[List[float]]:
    """A manageable set of the plan's target poses that still spans the whole job: the points that
    stick out furthest in X, Y and Z, the approach / retract points, and an even spread of the rest.
    The first few are the most telling, so a hopeless placement is given up on quickly."""
    poses = [list(p) for k in plan.motions for p in plan.cmds[k].poses]
    if not poses:
        return []
    key = []
    for axis in range(3):
        key.append(min(poses, key=lambda p: p[axis]))
        key.append(max(poses, key=lambda p: p[axis]))
    if approach > 0:
        key += [p[:2] + [p[2] + approach] + p[3:] for p in (poses[0], poses[-1])]
    # the middle of the long straight moves: both ends can be reachable when the middle is not
    long_moves = sorted(((math.dist(plan.paths[k][0], plan.paths[k][-1]), k) for k in plan.motions
                         if plan.cmds[k].instr == "MoveL" and len(plan.paths[k]) > 1), reverse=True)
    for length, k in long_moves[:6]:
        if length > 100.0:
            middle = (plan.paths[k][0] + plan.paths[k][-1]) / 2.0
            key.append([float(v) for v in middle] + plan.cmds[k].poses[-1][3:])
    step = max(1, len(poses) // max(1, most - len(key)))
    out, seen = [], set()
    for p in key + [poses[0]] + poses[::step] + [poses[-1]]:
        if tuple(p) not in seen:
            seen.add(tuple(p))
            out.append(p)
    return out


@dataclass
class Layout:
    """The drawing turned by `yaw`: its sampled target poses and its middle, relative to the origin."""
    yaw: float
    targets: List[List[float]]
    centre: Sequence[float]


def find_origin(robot, layouts: List[Layout], start: Sequence[float], ref_joints: Sequence[float],
                half_range: float = 500.0, log: Callable = print,
                should_stop: Callable[[], bool] = lambda: False,
                verify: Optional[Callable[[List[float], float], bool]] = None) -> Placement:
    """Search origin and rotation. start: the tool tip now (x, y, z). verify(origin, yaw) is the full
    check of the whole path; a candidate only wins if it passes it.
    Raises eng.SetupError if nothing in range works, eng.Aborted if should_stop() turns true."""
    limits = joint_limits(robot)
    tried = dict(positions=0, quick=0, sampled=0, verified=0)

    def evaluate(layout: Layout, spot, points) -> Optional[Placement]:
        """Put the middle of the drawing at `spot` and ask the controller about `points`."""
        if should_stop():
            raise eng.Aborted("Stopped by the operator.")
        origin = [spot[0] - layout.centre[0], spot[1] - layout.centre[1], start[2]]
        ref, worst, total = list(ref_joints), (1e9, 1e9, 1e9), 0.0
        for p in points:
            pose = [p[0] + origin[0], p[1] + origin[1], p[2] + origin[2], p[3], p[4], p[5]]
            err, rest = eng.unpack(robot.GetInverseKinRef(0, pose, ref))
            if err != 0 or not rest or rest[0] is None:
                return None
            ref = [float(q) for q in rest[0]]
            m = margins(ref, limits)
            if m[0] < 0:                                      # a solution outside the soft limits is no solution
                return None
            worst = tuple(min(a, b) for a, b in zip(worst, m))
            total += comfort_of(m)
        return Placement([round(v, 3) for v in origin], layout.yaw, comfort_of(worst), *worst,
                         mean=total / len(points))

    def rank(p: Placement, spot):
        return (round(p.comfort, 2), round(p.mean, 2), -round(math.dist(spot, start[:2])), -abs(p.yaw))

    # 1. coarse: every rotation on a grid over the whole search square, with a handful of telling points
    step = max(half_range / 5.0, 20.0)
    n = int(half_range // step)
    grid = [(start[0] + i * step, start[1] + j * step) for i in range(-n, n + 1) for j in range(-n, n + 1)]
    turns = ", ".join(f"{layout.yaw:g}\u00b0" for layout in layouts)
    log(f"Automatic placement: trying {len(grid)} positions within \u00b1{half_range:g} mm of the tool tip, "
        f"drawing turned by {turns} \u2026")
    shortlist = []
    for layout in layouts:
        for spot in grid:
            tried["positions"] += 1
            found = evaluate(layout, spot, layout.targets[:14])
            if found is not None:
                tried["quick"] += 1
                shortlist.append((rank(found, spot), layout, spot))
    shortlist.sort(key=lambda item: item[0], reverse=True)

    # 2. best first: all sampled points, then the full check of the path; the first to pass both wins
    best = None
    for _rank, layout, spot in shortlist[:40]:
        found = evaluate(layout, spot, layout.targets)
        if found is None:
            continue
        tried["sampled"] += 1
        if verify is None or verify(found.origin, layout.yaw):
            tried["verified"] += 1
            best = (found, layout, spot)
            break
    if best is None:
        if not shortlist:
            why = "no position where even the outermost points of the drawing are reachable in this posture"
        else:
            why = (f"{tried['quick']} positions fit the outermost points, {tried['sampled']} of the best also fit "
                   "the sampled path, but none passes the full check of every move")
        raise eng.SetupError(
            f"Automatic placement found {why} (searched \u00b1{half_range:g} mm round the tool tip at this height). "
            "Things that help: a tool pointing straight down with a fixed posture, a different height, "
            "a wider search, or a smaller part.")

    # 3. refine round the winner; a refinement only counts if it passes the full check as well
    found, layout, spot = best
    for _round in range(3):
        step /= 2.0
        better = None
        for s in [(spot[0] + i * step, spot[1] + j * step) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]:
            if max(abs(s[0] - start[0]), abs(s[1] - start[1])) > half_range + 1e-6:
                continue
            tried["positions"] += 1
            other = evaluate(layout, s, layout.targets)
            if other is not None and rank(other, s) > rank(found, spot) and (better is None
                                                                            or rank(other, s) > rank(*better)):
                better = (other, s)
        if better is None:
            continue
        if verify is None or verify(better[0].origin, layout.yaw):
            found, spot = better
    found.tried, found.possible = tried["positions"], tried["quick"]
    turned = f", drawing turned by {found.yaw:g}\u00b0" if found.yaw else ""
    log(f"Automatic placement: origin X {found.origin[0]:.1f}  Y {found.origin[1]:.1f}  Z {found.origin[2]:.1f} mm"
        f"{turned}  ({found.possible} of {found.tried} tested positions were possible)")
    log(f"  worst point there: {found.limit_deg:.0f}\u00b0 from a joint limit, wrist {found.wrist_deg:.0f}\u00b0 and "
        f"elbow {found.elbow_deg:.0f}\u00b0 from a singular position (comfort {found.comfort:.2f} of 1)")
    return found
