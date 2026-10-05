#!/usr/bin/env python3
"""
gcode_to_fairino.py - run a G-code file on a FAIRINO cobot through the Python SDK.

It does what the WebApp "G-code Conversion" tool does (manual 14.8), but sends the instructions
straight to the controller instead of writing a LUA file, and prints every command as it goes.

Supported G-code:
    G0               rapid move          -> MoveJ  (speed = S word read as mm/min)
    G1               linear move         -> MoveL  (speed = F, mm/min)
    G2 / G3          XY arc (I/J or R)   -> MoveC  (speed = F)
    G2 / G3          full XY circle      -> Circle (speed = F)
    G4 P/S           dwell
    G17 G20 G21 G90 G91
    M3/M4/M5         spindle/laser/pen on/off -> control-box DO (--spindle-do)
    M0/M1            pause (press Enter)   M2/M30 end of program
Anything else is reported as a warning and ignored.

How coordinates are mapped:
    WebApp mode (--ref-point NAME): G-code XYZ are coordinates in the workpiece frame of the saved
    teach point NAME, which also supplies the tool frame and the reference posture. The reference
    posture applies at the start of the path; along the path the tool keeps its angle to the drawing
    plane and to the path tangent (--posture).
    Manual mode: G-code (0,0,0) is placed at --origin and optionally rotated about Z by --yaw, with
    the posture from --rpy. --origin-here takes origin and posture from the current TCP pose.

The tool / workpiece frames of the job must be the ones ACTIVE on the controller (its IK solves for
the active frames). This is checked before anything moves.

Examples:
    python gcode_to_fairino.py part.gcode --dry-run --preview
    python gcode_to_fairino.py part.gcode --ref-point refPose --check-ik --dry-run
    python gcode_to_fairino.py part.gcode --ref-point refPose
    python gcode_to_fairino.py part.nc --origin-here --tool 1 --spindle-do 0
"""
import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

WORD_RE = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
MOTION_KINDS = ("line", "arc", "circle")


# ============================================================================ parsing
@dataclass
class Cmd:
    kind: str                     # "line", "arc", "circle", "dwell", "m"
    line: int                     # source line number (for error messages)
    start: Tuple[float, ...] = None
    end: Tuple[float, ...] = None
    mid: Tuple[float, ...] = None  # arc: a point between start and end. circle: point at 1/4 turn
                                   # (and end = point at 1/2 turn; the move finishes back at start)
    feed: float = 0.0             # mm/min
    rapid: bool = False
    code: int = 0                 # M-code number
    seconds: float = 0.0          # dwell time
    src: str = ""                 # the G-code text this came from (for the trace)
    spindle: float = 0.0          # S word in effect; the WebApp uses it as the G0 speed (mm/min)
    arc: Tuple[float, ...] = None  # arc/circle: (cx, cy, radius, start angle, signed sweep), rad
    instr: str = ""               # robot instruction this becomes - filled in by plan()
    poses: list = None            # robot target poses - filled in by plan()


def expand_arc(start, end, cw, i, j, r, max_seg_deg=360.0, min_radius=0.5):
    """Return [(kind, mid, end, geometry), ...] for an XY arc, or None if it is too small (use a line).

    Like the WebApp converter: an arc is ONE MoveC, a full circle is ONE Circle instruction.
    max_seg_deg < 360 splits arcs into several MoveC instead (and circles too).
    """
    sx, sy, sz = start
    ex, ey, ez = end
    if r is not None:
        dx, dy = ex - sx, ey - sy
        d = math.hypot(dx, dy)
        if d < 1e-9:
            raise ValueError("R-format arc with identical start and end point")
        h = math.sqrt(max(r * r - (d / 2.0) ** 2, 0.0))
        side = -1.0 if cw else 1.0          # centre is right of chord for CW, left for CCW
        if r < 0:                           # negative R = the long way round (>180 deg)
            side = -side
        cx = sx + dx / 2.0 + side * h * (-dy / d)
        cy = sy + dy / 2.0 + side * h * (dx / d)
    else:
        cx, cy = sx + (i or 0.0), sy + (j or 0.0)

    radius = math.hypot(sx - cx, sy - cy)
    if radius < min_radius:
        return None
    a0 = math.atan2(sy - cy, sx - cx)
    a1 = math.atan2(ey - cy, ex - cx)
    sweep = ((a0 - a1) if cw else (a1 - a0)) % (2 * math.pi)
    full = sweep < 1e-6                     # start == end -> full circle
    if full:
        sweep = 2 * math.pi
    direction = -1.0 if cw else 1.0

    def on_circle(angle, z):
        return (cx + radius * math.cos(angle), cy + radius * math.sin(angle), z)

    if full and max_seg_deg >= 360.0 and abs(ez - sz) < 1e-9:
        # Circle instruction: start -> path point (1/4 turn) -> target point (1/2 turn) -> start
        return [("circle", on_circle(a0 + direction * sweep * 0.25, sz),
                 on_circle(a0 + direction * sweep * 0.5, sz), (cx, cy, radius, a0, direction * sweep))]

    n = max(1, math.ceil(math.degrees(sweep) / max_seg_deg - 1e-9))
    if full:
        n = max(n, 2)                       # one MoveC cannot close a full turn (helical circle)
    seg = sweep / n

    out = []
    for k in range(n):
        am = a0 + direction * seg * (k + 0.5)
        ae = a0 + direction * seg * (k + 1)
        zm = sz + (ez - sz) * (k + 0.5) / n
        ze = sz + (ez - sz) * (k + 1) / n
        seg_end = (ex, ey, ez) if k == n - 1 else on_circle(ae, ze)
        out.append(("arc", on_circle(am, zm), seg_end,
                    (cx, cy, radius, a0 + direction * seg * k, direction * seg)))
    return out


def parse_gcode(text: str, default_feed: float, dwell_p_ms: bool, arc_split: float = 360.0):
    cmds: List[Cmd] = []
    warnings: List[str] = []
    pos = [0.0, 0.0, 0.0]
    absolute, unit, feed, motion, spindle = True, 1.0, default_feed, None, 0.0

    for n, raw in enumerate(text.splitlines(), 1):
        line = re.sub(r"\(.*?\)", "", raw).split(";")[0].strip().upper()
        if not line or line.startswith("%"):
            continue
        words = WORD_RE.findall(line)
        if not words:
            continue
        src = " ".join(line.split())

        g_codes = [float(v) for k, v in words if k == "G"]
        m_codes = [int(float(v)) for k, v in words if k == "M"]
        params = {k: float(v) for k, v in words if k not in "GMN"}

        dwell = False
        for g in g_codes:
            if g in (0, 1, 2, 3):
                motion = int(g)
            elif g == 4:
                dwell = True
            elif g == 17:
                pass
            elif g in (18, 19):
                raise ValueError(f"line {n}: only G17 (XY-plane) arcs are supported")
            elif g == 20:
                unit = 25.4
            elif g == 21:
                unit = 1.0
            elif g == 90:
                absolute = True
            elif g == 91:
                absolute = False
            else:
                warnings.append(f"line {n}: G{g:g} ignored")

        # spindle / laser commands take effect before motion on the same line
        for m in m_codes:
            if m in (3, 4, 5):
                cmds.append(Cmd("m", n, code=m, src=src))

        if dwell:
            p = params.get("P", 0.0)
            secs = params.get("S", 0.0) + (p / 1000.0 if dwell_p_ms else p)
            cmds.append(Cmd("dwell", n, seconds=secs, src=src))
            continue

        if "F" in params and params["F"] > 0:
            feed = params["F"] * unit
        if "S" in params:                   # WebApp rule: S is the G0 (MoveJ) speed, RPM read as mm/min
            spindle = params["S"]

        axes = {a: params[a] * unit for a in "XYZ" if a in params}
        full_circle = motion in (2, 3) and not axes and ("I" in params or "J" in params)
        if axes or full_circle:
            if motion is None:
                warnings.append(f"line {n}: coordinates without G0/G1 - treated as G1")
                motion = 1
            target = list(pos)
            for idx, a in enumerate("XYZ"):
                if a in axes:
                    target[idx] = axes[a] if absolute else pos[idx] + axes[a]

            if motion in (0, 1):
                if target != pos:
                    cmds.append(Cmd("line", n, start=tuple(pos), end=tuple(target),
                                    feed=feed, rapid=(motion == 0), spindle=spindle, src=src))
            else:
                i = params["I"] * unit if "I" in params else None
                j = params["J"] * unit if "J" in params else None
                r = params["R"] * unit if "R" in params else None
                segs = expand_arc(tuple(pos), tuple(target), motion == 2, i, j, r, arc_split)
                if segs is None:
                    cmds.append(Cmd("line", n, start=tuple(pos), end=tuple(target), feed=feed, src=src))
                else:
                    seg_start = tuple(pos)
                    for kind, mid, seg_end, geom in segs:
                        cmds.append(Cmd(kind, n, start=seg_start, mid=mid, end=seg_end, feed=feed,
                                        arc=geom, src=src))
                        seg_start = seg_end
            pos = target

        stop = False
        for m in m_codes:
            if m in (0, 1):
                cmds.append(Cmd("m", n, code=m, src=src))
            elif m in (2, 30):
                stop = True
            elif m not in (3, 4, 5):
                warnings.append(f"line {n}: M{m} ignored")
        if stop:
            break

    return cmds, warnings


# ============================================================================ geometry
class Frame:
    """Maps G-code XYZ (mm) to a robot base-frame pose [x, y, z, rx, ry, rz]."""

    def __init__(self, origin, rpy, yaw_deg=0.0, scale=1.0):
        self.origin, self.rpy, self.scale = list(origin), list(rpy), scale
        self.c, self.s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))

    def pose(self, p, dz=0.0, yaw_off=0.0):
        """yaw_off (deg) turns the reference posture about the frame Z axis (= normal of the G-code plane).
        FAIRINO rx/ry/rz are fixed-axis X-Y-Z angles, so that rotation is simply added to rz."""
        x, y, z = (v * self.scale for v in p)
        rx, ry, rz = self.rpy
        if yaw_off:
            rz = round((rz + yaw_off + 180.0) % 360.0 - 180.0, 4)
        return [round(self.origin[0] + self.c * x - self.s * y, 4),
                round(self.origin[1] + self.s * x + self.c * y, 4),
                round(self.origin[2] + z + dz, 4),
                rx, ry, rz]

    def tilt_deg(self):
        """Angle between the tool Z axis and the normal of the G-code plane (0 = tool perpendicular)."""
        rx, ry = math.radians(self.rpy[0]), math.radians(self.rpy[1])
        return math.degrees(math.acos(min(1.0, abs(math.cos(rx) * math.cos(ry)))))


def targets(c):
    """Points a motion is sent with: [end] for a line, [path point, target point] for MoveC / Circle."""
    return [c.end] if c.kind == "line" else [c.mid, c.end]


def polyline(c, step_deg=5.0):
    """Points along the real toolpath of a motion (for length, bounding box and preview)."""
    if not c.arc:
        return [c.start, c.end]
    cx, cy, radius, a0, sweep = c.arc
    n = max(2, math.ceil(abs(math.degrees(sweep)) / step_deg))
    z0 = c.start[2]
    z1 = z0 if c.kind == "circle" else c.end[2]
    return [(cx + radius * math.cos(a0 + sweep * k / n), cy + radius * math.sin(a0 + sweep * k / n),
             z0 + (z1 - z0) * k / n) for k in range(n + 1)]


def headings(c):
    """Direction of travel in the XY plane (rad) at the start of a motion and at each of its targets.
    None = no XY direction (pure Z move)."""
    if c.arc:
        _, _, _, a0, sweep = c.arc
        h0 = a0 + math.copysign(math.pi / 2.0, sweep)
        return h0, [h0 + sweep * f for f in ((0.25, 0.5) if c.kind == "circle" else (0.5, 1.0))]
    dx, dy = c.end[0] - c.start[0], c.end[1] - c.start[1]
    if math.hypot(dx, dy) < 1e-6:
        return None, [None]
    return math.atan2(dy, dx), [math.atan2(dy, dx)]


def plan(cmds, frame, a):
    """Turn every motion into the robot instruction + target poses it is sent as - the in-memory
    equivalent of the LUA file the WebApp generates. Returns True if the posture follows the path.

    Posture rule (manual 14.8.2): the reference posture applies at the start of the path; further along,
    the tool keeps its angle to the drawing plane and its angle to the path tangent. That is the
    reference posture turned about the plane normal by however much the tangent has turned.
    """
    motions = [c for c in cmds if c.kind in MOTION_KINDS]
    heads = [headings(c) for c in motions]
    # for each motion: start direction of the next feed move (a G0 arrives already turned for its path)
    ahead, nxt = [None] * len(motions), None
    for k in range(len(motions) - 1, -1, -1):
        ahead[k] = nxt
        if not motions[k].rapid and heads[k][0] is not None:
            nxt = heads[k][0]
    ref = nxt                                # tangent at the start of the path
    follow = ref is not None and (a.posture == "tangent" or
                                  (a.posture == "auto" and frame.tilt_deg() > 1.0))
    yaw = 0.0
    for c, (h0, hs), nx in zip(motions, heads, ahead):
        if c.rapid:
            hs = [nx]
        offs = []
        for h in hs:
            if follow and h is not None:     # no XY direction -> keep the posture we have
                yaw = math.degrees(h - ref)
            offs.append(yaw)
        if follow and c.kind == "circle":
            yaw = math.degrees(h0 - ref)     # a full circle finishes back at its start direction
        c.poses = [frame.pose(p, yaw_off=o) for p, o in zip(targets(c), offs)]
        c.instr = ("Circle" if c.kind == "circle" else "MoveC" if c.kind == "arc" else
                   "MoveJ" if c.rapid and a.g0 == "movej" else "MoveL")
    return follow


def speed_mm_s(c, a, cap=True):
    """TCP speed of a motion: F for G1/G2/G3; for G0 the S word (--rapid-speed overrides it)."""
    if not c.rapid:
        v = c.feed / 60.0
    elif a.rapid_speed is not None:
        v = a.rapid_speed
    else:
        v = c.spindle / 60.0 if c.spindle > 0 else 100.0
    return max(0.1, min(v, a.max_speed)) if cap else v


def describe(c, a, note=""):
    """One trace line: the G-code that was read -> the robot command it is implemented as."""
    head = f"N{c.line:<5d} {c.src[:32]:<32} -> "
    if c.kind == "dwell":
        return head + f"wait {c.seconds:g} s"
    if c.kind == "m":
        if c.code in (0, 1):
            return head + "pause until Enter"
        if a.spindle_do < 0:
            return head + f"M{c.code} skipped (no --spindle-do)"
        return head + f"SetDO({a.spindle_do}, {0 if c.code == 5 else 1})"

    def fmt(p):
        return "[" + ", ".join(f"{round(v, 3) + 0.0:.3f}" for v in p) + "]"

    pts = fmt(c.poses[0]) if len(c.poses) == 1 else f"via {fmt(c.poses[0])} to {fmt(c.poses[1])}"
    return head + f"{c.instr:<6} {pts} @ {speed_mm_s(c, a):.1f} mm/s{note}"


def summarize(cmds, frame, a, follow, out=print):
    """Report the plan through out() (one line per call) and return its bounding box (lo, hi)."""
    pts, length, seconds, capped, count = [], 0.0, 0.0, 0, {}
    for c in cmds:
        if c.kind not in MOTION_KINDS:
            if c.kind == "dwell":
                seconds += c.seconds
            continue
        seq = polyline(c)
        d = sum(math.dist(p, q) for p, q in zip(seq, seq[1:])) * frame.scale
        length += d
        seconds += d / speed_mm_s(c, a)
        capped += speed_mm_s(c, a, cap=False) > a.max_speed
        count[c.instr] = count.get(c.instr, 0) + 1
        pts += [frame.pose(p)[:3] for p in seq[1:]]
    if not pts:
        out("No motion found in file.")
        return None
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    where = f"workpiece {a.user}" if a.user else "BASE"
    out(f"Frames          : tool {a.tool}, {where} frame")
    out("Posture         : " + (f"follows the path tangent (tool tilted {frame.tilt_deg():.1f} deg)"
                                if follow else f"fixed {[round(v, 3) for v in frame.rpy]}"))
    out("Instructions    : " + ", ".join(f"{k} x{v}" for k, v in sorted(count.items())))
    out(f"Path length     : {length:.1f} mm   (est. {seconds / 60:.1f} min)")
    out(f"Frame bbox      : X {lo[0]:.1f}..{hi[0]:.1f}  Y {lo[1]:.1f}..{hi[1]:.1f}  Z {lo[2]:.1f}..{hi[2]:.1f} mm")
    if capped:
        out(f"warning: {capped} moves ask for more than --max-speed {a.max_speed:g} mm/s and are capped")
    return lo, hi


def preview(cmds):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    for c in cmds:
        if c.kind not in MOTION_KINDS:
            continue
        seq = polyline(c)
        xs, ys, zs = zip(*seq)
        ax.plot(xs, ys, zs, "r--" if c.rapid else "b-", linewidth=0.8)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("G-code preview (G-code coordinates; red dashed = G0)")
    plt.show()


# ============================================================================ robot
class RobotCommandError(RuntimeError):
    pass


# Return codes of SDK calls (FAIRINO "Error Code Comparison Table"). A command answered with one of
# these was refused: it is not a robot fault, so nothing shows up in the WebApp.
ERR_SPEED_LIMIT, ERR_ACC_LIMIT = 182, 183
ERROR_TEXT = {
    3: "wrong number of parameters", 4: "a parameter value is out of range",
    14: "the command could not be executed - check the WebApp for a fault",
    18: "a robot program is running, stop it first", 28: "inverse kinematics failed for this pose",
    32: "a joint is beyond its limit", 34: "wrong workpiece frame number", 37: "wrong tool frame number",
    38: "singular pose", 42: "the posture changes too much in one move", 64: "not added to the motion queue",
    66: "full circle: path point 1 is wrong", 67: "full circle: path point 2 is wrong",
    69: "arc: the path point is wrong", 70: "arc: the target point is wrong", 74: "line: the point is wrong",
    99: "a safety stop is active", 101: "the robot is not enabled", 112: "the pose cannot be reached",
    151: "the joint configuration changes between the current and the target position",
    153: "arc points are too close together", 170: "the workpiece frame is not applied",
    171: "a joint is beyond its soft limit", 172: "the speed cannot be 0",
    ERR_SPEED_LIMIT: "the speed in mm/s is above the controller's limit - lower --max-speed or the feed",
    ERR_ACC_LIMIT: "the acceleration in mm/s^2 is above the controller's limit - lower --acc",
    185: "a fault signal stopped the motion", 186: "the emergency stop stopped the motion",
    204: "the controller's command queue is full",
}


class SetupError(RuntimeError):
    """The job cannot be set up: no connection, wrong active frames, unknown teach point..."""


class Aborted(RobotCommandError):
    """The run was stopped on request."""


def unpack(ret):
    """SDK calls return either an int error code or a tuple (error, values...)."""
    if isinstance(ret, (list, tuple)):
        return ret[0], ret[1:]
    return ret, ()


def connect(ip):
    try:
        from fairino import Robot
    except ImportError:
        import Robot  # Robot.py placed next to this script
    robot = Robot.RPC(ip)
    if not Robot.RPC.is_connect:
        raise SetupError(f"Could not connect to robot at {ip} (check network / controller).")
    return robot


def check_frames(robot, a, pose=None, joints=None, what=""):
    """Targets are sent in tool a.tool / workpiece a.user, and the controller solves IK with whatever
    frames are ACTIVE on it - so they have to be the same ones. Refuse to move if they are not.
    If a pose + its joint position are given, also prove that the pose is expressed in those frames."""
    time.sleep(0.5)                          # let the first state packet arrive
    _, tool = unpack(robot.GetActualTCPNum())
    _, user = unpack(robot.GetActualWObjNum())
    if int(tool[0]) != a.tool or int(user[0]) != a.user:
        raise SetupError(f"Controller has tool {tool[0]} / workpiece {user[0]} active, but this job runs in "
                         f"tool {a.tool} / workpiece {a.user}. Activate those frames on the controller and retry.")
    if pose is None:
        return
    err, rest = unpack(robot.GetInverseKinRef(0, pose, joints))
    off = (max(abs((q - j + 180.0) % 360.0 - 180.0) for q, j in zip(rest[0], joints))
           if err == 0 and rest and rest[0] is not None else None)
    if off is None or off > 1.0:
        raise SetupError(f"Frame check failed: the controller's IK of {what} does not give back its joint "
                         f"position ({'error ' + str(err) if off is None else f'{off:.2f} deg off'}). The pose "
                         "is not expressed in the active tool / workpiece frame - not safe to run.")


def resolve_frame(a, robot, log=print):
    """Work out where the G-code sits: returns the Frame and fills in a.tool / a.user / a.approach.
    The robot is only read here (teach point, current pose, active frames) - it never moves."""
    if (a.ref_point or a.origin_here) and robot is None:
        raise SetupError("--ref-point / --origin-here need a connected robot.")
    origin, rpy = a.origin, a.rpy
    if origin is None:
        origin = [0.0, 0.0, 0.0] if a.ref_point else [400.0, 0.0, 200.0]
    if a.approach is None:
        a.approach = 0.0 if a.ref_point else 20.0
    if a.ref_point:
        # what the WebApp does when you pick the reference point: posture, tool and workpiece from it
        err, rest = unpack(robot.GetRobotTeachingPoint(a.ref_point))
        if err != 0 or not rest or rest[0] is None:
            raise SetupError(f"Teach point '{a.ref_point}' could not be read (error {err}).")
        tp = [float(v) for v in rest[0]]
        rpy, a.tool, a.user = tp[3:6], int(tp[12]), int(tp[13])
        log(f"Reference point '{a.ref_point}': posture {rpy}, tool {a.tool}, workpiece {a.user}")
        check_frames(robot, a, tp[:6], tp[6:12], f"teach point '{a.ref_point}'")
    elif a.origin_here:
        err, rest = unpack(robot.GetActualTCPPose())
        if err != 0:
            raise SetupError(f"GetActualTCPPose failed ({err})")
        origin, rpy = list(rest[0][:3]), list(rest[0][3:])
        log(f"Origin from current TCP: {[round(v, 2) for v in rest[0]]}")
        _, joints = unpack(robot.GetActualJointPosDegree())
        check_frames(robot, a, rest[0], joints[0], "the current TCP pose")
    elif robot is not None:
        check_frames(robot, a)
    return Frame(origin, rpy, a.yaw, a.scale)


class Streamer:
    def __init__(self, robot, frame, args):
        self.robot, self.frame, self.a = robot, frame, args
        self.last_j = None
        self.last_pose = None     # where the previous instruction leaves the TCP
        self.sent, self.total = 0, 0
        self.spindle_on = False
        self.unreachable = []     # (cmd, pose) pairs found by check_ik()
        # Hooks. The defaults are the command-line behaviour; a GUI replaces them.
        self.on_command = None    # called as on_command(cmd, text) instead of printing the trace
        self.pause = input        # how M0 / M1 waits for the operator
        self.log = print
        self.abort = threading.Event()   # set it (from any thread) to end the run
        # stoppable: send every move non-blocking and wait for it here instead of inside the SDK call.
        # A blocking SDK call cannot be interrupted, so with it a stop request would have to wait for
        # the move to finish; this way it is noticed within milliseconds, between two short calls.
        self.stoppable = False

    # ---- helpers
    def check_abort(self):
        if self.abort.is_set():
            raise Aborted("Stopped by the operator.")

    def check(self, ret, what, line=None):
        self.check_abort()
        err, _ = unpack(ret)
        if err != 0:
            where = f" (G-code line {line})" if line else ""
            meaning = f": {ERROR_TEXT[err]}" if err in ERROR_TEXT else ""
            raise RobotCommandError(f"{what} failed with error {err}{meaning}{where}")

    def check_robot_fault(self):
        err, rest = unpack(self.robot.GetRobotErrorCode())
        if err == 0 and rest and rest[0][0] != 0:
            raise RobotCommandError(f"Robot fault: main code {rest[0][0]}, sub code {rest[0][1]}")

    def current_joints(self):
        err, rest = unpack(self.robot.GetActualJointPosDegree())
        if err != 0:
            raise RobotCommandError(f"GetActualJointPosDegree failed ({err})")
        return list(rest[0])

    def ik(self, pose, line):
        """IK referenced to the previous target, so the solution branch stays consistent."""
        err, rest = unpack(self.robot.GetInverseKinRef(0, pose, self.last_j))
        if err != 0 or not rest or rest[0] is None:
            error = RobotCommandError(f"No IK solution for {pose} (G-code line {line}, error {err})")
            error.code = err
            raise error
        self.last_j = list(rest[0])
        return self.last_j

    def trace(self, c, note=""):
        """Say which command is about to be sent. Blended moves are queued on the controller, so the
        arm can be up to --queue-max moves behind this line; blocking moves are exactly in step."""
        self.check_abort()
        self.sent += 1
        text = f"[{self.sent:>{len(str(self.total))}}/{self.total}] " + describe(c, self.a, note)
        if self.on_command is not None:
            self.on_command(c, text)
        elif not self.a.quiet:
            print(text, flush=True)
        elif self.sent % 200 == 1:
            print(f"  sent {self.sent}/{self.total} (G-code line {c.line})")

    def speed(self, c):
        mm_s = speed_mm_s(c, self.a)
        if self.a.speed_mode == "physical":   # ovl = mm/s, oacc = mm/s^2
            return dict(vel=100.0, ovl=mm_s, oacc=self.a.acc, velAccParamMode=1)
        pct = max(1.0, min(100.0, 100.0 * mm_s / self.a.max_speed))
        return dict(vel=pct, ovl=100.0, oacc=100.0, velAccParamMode=0)

    def throttle(self):
        """Keep the controller's motion queue from overflowing while blending."""
        while True:
            err, rest = unpack(self.robot.GetMotionQueueLength())
            if err != 0 or rest[0] < self.a.queue_max:
                return
            self.check_abort()
            self.check_robot_fault()
            time.sleep(0.01)

    def idle(self):
        _, q = unpack(self.robot.GetMotionQueueLength())
        _, done = unpack(self.robot.GetRobotMotionDone())
        return bool(q and q[0] == 0 and done and done[0] == 1)

    def wait_idle(self, just_sent=False):
        """Wait until the arm has finished everything it was given.
        just_sent: a move went out a moment ago. "Done" comes from the status stream and can still be
        the old value, so first wait (briefly) until the arm is seen busy."""
        if just_sent:
            deadline = time.time() + self.a.start_wait
            while self.idle() and time.time() < deadline:
                self.check_abort()
                time.sleep(0.005)
        else:
            time.sleep(0.05)
        while not self.idle():
            self.check_abort()
            self.check_robot_fault()
            time.sleep(0.02)

    def exact(self, blend):
        """The blend value to send. A move that must end exactly on its point (blend < 0) is normally
        a blocking call; in stoppable mode it goes out non-blocking with radius 0."""
        return 0.0 if (blend < 0 and self.stoppable) else blend

    def settle(self, blend):
        """...and in stoppable mode the waiting for that move is done here."""
        if blend < 0 and self.stoppable:
            self.wait_idle(just_sent=True)

    # ---- motion
    def move_joint(self, c):
        """G0 -> MoveJ, as the WebApp does. MoveJ only takes a joint-speed %, so use the % that makes
        the move last distance / speed (never more than speed / --max-speed)."""
        pose = c.poses[0]
        q0, joints = self.last_j, self.ik(pose, c.line)
        mm_s = speed_mm_s(c, self.a)
        if self.last_pose is None:           # first move, from wherever the arm happens to be
            pct = self.a.first_vel
        else:
            dist = max(math.dist(self.last_pose[:3], pose[:3]), 1e-6)
            busiest = max(abs(q - p) for p, q in zip(q0, joints))          # deg
            pct = min(100.0 * busiest * mm_s / dist / self.a.joint_speed, 100.0 * mm_s / self.a.max_speed)
        pct = max(1.0, min(100.0, pct))
        self.wait_idle()
        self.trace(c, f" = {pct:.1f} % joint speed")
        ret = self.robot.MoveJ(joint_pos=joints, tool=self.a.tool, user=self.a.user, desc_pos=pose,
                               vel=pct, blendT=self.exact(-1.0))
        self.check(ret, "MoveJ", c.line)
        self.settle(-1.0)
        self.last_pose = pose

    def cartesian(self, c, send):
        """Send a MoveL / MoveC / Circle through send(speed settings). If the controller refuses the
        acceleration (it has a limit that is not documented and differs between robots), halve it and
        send the same move again - a refused move was never queued. The lower value is then kept."""
        while True:
            ret = send(self.speed(c))
            if (unpack(ret)[0] != ERR_ACC_LIMIT or self.a.speed_mode != "physical"
                    or self.a.acc <= self.a.min_acc):
                return ret
            self.a.acc = max(self.a.acc / 2.0, self.a.min_acc)
            self.log(f"  the controller refused the acceleration; continuing with {self.a.acc:g} mm/s^2")

    def move_line(self, c, blend):
        pose = c.poses[0]
        joints = self.ik(pose, c.line)
        self.throttle()
        self.trace(c)
        ret = self.cartesian(c, lambda s: self.robot.MoveL(
            desc_pos=pose, tool=self.a.tool, user=self.a.user, joint_pos=joints, blendR=self.exact(blend), **s))
        self.check(ret, "MoveL", c.line)
        self.settle(blend)
        self.last_pose = pose

    def move_arc(self, c, blend):
        """MoveC (arc) and Circle (full circle) take the same arguments: a path point and a target point."""
        mid, end = c.poses
        j_start = self.last_j
        j_mid = self.ik(mid, c.line)
        j_end = self.ik(end, c.line)
        self.throttle()
        self.trace(c)
        move = self.robot.Circle if c.kind == "circle" else self.robot.MoveC
        ret = self.cartesian(c, lambda s: move(
            desc_pos_p=mid, tool_p=self.a.tool, user_p=self.a.user,
            desc_pos_t=end, tool_t=self.a.tool, user_t=self.a.user,
            joint_pos_p=j_mid, joint_pos_t=j_end,
            vel_p=s["vel"], vel_t=s["vel"], ovl=s["ovl"], oacc=s["oacc"],
            blendR=self.exact(blend), velAccParamMode=s["velAccParamMode"]))
        self.check(ret, c.instr, c.line)
        self.settle(blend)
        if c.kind == "circle":
            self.last_j = j_start            # a full circle ends where it started
        else:
            self.last_pose = end

    def m_code(self, c):
        self.wait_idle()
        self.trace(c)
        if c.code in (3, 4, 5) and self.a.spindle_do >= 0:
            on = 0 if c.code == 5 else 1
            self.check(self.robot.SetDO(self.a.spindle_do, on), "SetDO", c.line)
            self.spindle_on = bool(on)
        elif c.code in (0, 1):
            self.pause(f"[line {c.line}] M{c.code} pause - press Enter to continue...")
            self.check_abort()

    # ---- top level
    def check_points(self, c, start):
        """The poses to test for one motion, in path order, as (pose, on_path).
        Joint moves (and the first move, whose start is unknown): just the targets.
        Cartesian moves: also a point every --check-step mm along the line / arc from `start`;
        on_path is True for those, meaning the arm has to get there in a straight, continuous way."""
        step = self.a.check_step
        if c.instr == "MoveJ" or start is None or step <= 0:
            return [(pose, False) for pose in c.poses]
        pts = [self.frame.pose(p)[:3] for p in polyline(c)]
        seg = [math.dist(p, q) for p, q in zip(pts, pts[1:])]
        length = sum(seg)
        n = max(1, math.ceil(length / step))
        circle = c.kind == "circle"
        if n == 1 or (circle and c.poses[0][3:] != list(start[3:])):
            return [(pose, True) for pose in c.poses]         # short move, or a circle with a turning tool
        end = start if circle else c.poses[-1]                # a full circle comes back to where it began
        out, k, done = [], 0, 0.0
        for i in range(1, n):                                 # n - 1 points in between
            want = length * i / n
            while done + seg[k] < want:
                done += seg[k]
                k += 1
            t = (want - done) / seg[k] if seg[k] > 0 else 0.0
            xyz = [a + (b - a) * t for a, b in zip(pts[k], pts[k + 1])]
            turn = [a + ((b - a + 180.0) % 360.0 - 180.0) * i / n for a, b in zip(start[3:], end[3:])]
            out.append((i / n, xyz + turn))
        # ...and the real targets, at their place along the path
        if circle:
            out += [(0.25, c.poses[0]), (0.5, c.poses[1])]
        elif c.kind == "arc":
            out += [(0.5, c.poses[0]), (1.0, c.poses[1])]
        else:
            out += [(1.0, c.poses[0])]
        return [(pose, True) for _f, pose in sorted(out, key=lambda item: item[0])]

    def check_ik(self, cmds):
        """Ask the controller for a joint position for every target and for points along the Cartesian
        moves. Also refuses a path on which the joints would have to jump between two neighbouring
        points: that is a singular position or a change of arm configuration, which a straight move
        cannot do."""
        self.last_j = self.current_joints()
        reasons = {38: "singular position (too close to the robot's own axis, or the arm fully stretched)",
                   112: "out of reach in this posture"}
        bad, start, tested, previous_ok = 0, None, 0, True
        for c in cmds:
            if c.kind not in MOTION_KINDS:
                continue
            points = self.check_points(c, start)
            found = {}                       # reason -> the points of this command it applies to
            for pose, on_path in points:
                before, problem = self.last_j, None
                tested += 1
                try:
                    joints = self.ik(pose, c.line)
                    jump = max(abs((q - p + 180.0) % 360.0 - 180.0) for p, q in zip(before, joints))
                    if on_path and previous_ok and jump > self.a.check_jump:
                        problem = (f"a joint would have to turn {jump:.0f} deg within {self.a.check_step:g} mm - "
                                   "singular position or change of arm configuration on a straight move")
                    previous_ok = True
                except RobotCommandError as e:
                    code = getattr(e, "code", None)
                    problem = reasons.get(code, f"no joint position (error {code})")
                    previous_ok = False
                if problem:
                    bad += 1
                    self.unreachable.append((c, pose))
                    found.setdefault(problem, []).append(pose)
            for problem, poses in found.items():
                where = (f"at {[round(v, 1) for v in poses[0][:3]]}" if len(poses) == 1 else
                         f"from {[round(v, 1) for v in poses[0][:3]]} to {[round(v, 1) for v in poses[-1][:3]]}")
                self.log(f"  line {c.line}  {c.src}: {len(poses)} of {len(points)} points {where} - {problem}")
            if c.kind != "circle":
                start = c.poses[-1]
            if bad >= self.a.check_max:
                self.log(f"  ...stopped looking after {bad} points")
                break
        self.log(f"IK check passed for all {tested} tested points." if bad == 0
                 else f"IK check: {bad} of {tested} tested points cannot be reached.")
        return bad == 0

    def prepare(self):
        self.check_robot_fault()
        self.check(self.robot.Mode(0), "Mode(auto)")
        self.check(self.robot.RobotEnable(1), "RobotEnable")
        # same rule as the WebApp: F and S are only true with the global speed at 100 %
        self.check(self.robot.SetSpeed(self.a.global_speed), "SetSpeed")
        time.sleep(0.5)
        self.last_j = self.current_joints()

    def run(self, cmds):
        motions = [c for c in cmds if c.kind in MOTION_KINDS]
        if not motions:
            return
        first = motions[0]
        lift = self.a.approach                # extra to the WebApp; 0 = go straight to the first point
        self.total = len(cmds) + (2 if lift > 0 else 0)

        def hop(pose, line, text):            # rapid move to a point `lift` mm above a pose
            up = pose[:2] + [round(pose[2] + lift, 4)] + pose[3:]
            return Cmd("line", line, rapid=True, spindle=first.spindle, src=text, poses=[up],
                       instr="MoveJ" if self.a.g0 == "movej" else "MoveL")

        def feed_move(c):                     # Cartesian moves, which can blend into each other
            return c is not None and c.kind in MOTION_KINDS and c.instr != "MoveJ"

        if lift > 0:
            self.send(hop(first.poses[-1], first.line, "(approach)"))

        t0 = time.time()
        for idx, c in enumerate(cmds):
            nxt = cmds[idx + 1] if idx + 1 < len(cmds) else None
            self.send(c, self.a.blend if feed_move(c) and feed_move(nxt) else -1.0)

        self.wait_idle()
        if self.spindle_on:
            self.robot.SetDO(self.a.spindle_do, 0)
            self.spindle_on = False
        if lift > 0 and self.last_pose is not None:
            self.send(hop(self.last_pose, motions[-1].line, "(retract)"))
        self.log(f"Done in {time.time() - t0:.1f}s")

    def send(self, c, blend=-1.0):
        if c.kind == "line":
            if c.instr == "MoveJ":
                self.move_joint(c)
            else:
                self.move_line(c, blend)
        elif c.kind in ("arc", "circle"):
            self.move_arc(c, blend)
        elif c.kind == "dwell":
            self.wait_idle()
            self.trace(c)
            if self.abort.wait(c.seconds):
                raise Aborted("Stopped by the operator.")
        elif c.kind == "m":
            self.m_code(c)

    def emergency_stop(self):
        """Stop the arm, then switch the tool output off. Call it on the thread that talks to the robot."""
        self.abort.set()

        def twice(call):                     # after Ctrl-C the connection is left in the middle of a
            try:                             # request and the first call on it fails; the second one
                return call()                # goes out on a fresh connection
            except Exception:
                return call()

        try:
            twice(self.robot.StopMotion)
        finally:
            if self.spindle_on and self.a.spindle_do >= 0:
                twice(lambda: self.robot.SetDO(self.a.spindle_do, 0))
                self.spindle_on = False


# ============================================================================ CLI
def build_parser():
    """The command-line options. A GUI gets the same defaults from build_parser().parse_args([file])."""
    p = argparse.ArgumentParser(description="Run G-code on a FAIRINO cobot.")
    p.add_argument("file", help="G-code file")
    p.add_argument("--ip", default="192.168.58.25")
    p.add_argument("--ref-point", metavar="NAME",
                   help="WebApp mode: saved teach point that gives the reference posture and the tool / "
                        "workpiece frames; G-code XYZ are then coordinates in that workpiece frame")
    p.add_argument("--tool", type=int, default=0, help="active tool frame number on the controller")
    p.add_argument("--user", type=int, default=0, help="active workpiece frame number (0 = base)")
    p.add_argument("--origin", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                   help="position (mm) of G-code 0,0,0 in the frame; default 0 0 0 with --ref-point, "
                        "else 400 0 200")
    p.add_argument("--rpy", type=float, nargs=3, default=[180.0, 0.0, 0.0], metavar=("RX", "RY", "RZ"),
                   help="fixed tool orientation (deg); default = tool pointing down")
    p.add_argument("--origin-here", action="store_true",
                   help="use the current TCP pose as origin and orientation")
    p.add_argument("--yaw", type=float, default=0.0, help="rotate G-code XY about Z (deg)")
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--feed", type=float, default=600.0, help="default feed mm/min if file has no F")
    p.add_argument("--posture", choices=["auto", "tangent", "fixed"], default="auto",
                   help="tangent: the tool turns with the path like the WebApp; fixed: never turns; "
                        "auto: tangent unless the tool is perpendicular to the plane")
    p.add_argument("--g0", choices=["movej", "movel"], default="movej",
                   help="instruction used for G0 (WebApp: movej)")
    p.add_argument("--arc-split", type=float, default=360.0, metavar="DEG",
                   help="split arcs into MoveC pieces of at most DEG (360 = one MoveC per arc and "
                        "Circle for full circles, like the WebApp)")
    p.add_argument("--rapid-speed", type=float, default=None,
                   help="G0 speed in mm/s; default: the S word of the file (as mm/min), else 100")
    p.add_argument("--joint-speed", type=float, default=180.0,
                   help="max joint speed of the robot in deg/s, used to turn the G0 speed into a MoveJ %%")
    p.add_argument("--first-vel", type=float, default=10.0,
                   help="joint speed %% of the first MoveJ, from wherever the arm is")
    p.add_argument("--global-speed", type=int, default=100,
                   help="global speed %% set before the run; F/S are only true at 100 (WebApp rule)")
    p.add_argument("--max-speed", type=float, default=250.0,
                   help="speed cap mm/s (in percent mode: the speed that equals 100%%)")
    p.add_argument("--acc", type=float, default=200.0,
                   help="TCP acceleration mm/s^2 (physical mode); halved automatically if the controller "
                        "refuses it")
    p.add_argument("--min-acc", type=float, default=10.0, help=argparse.SUPPRESS)
    p.add_argument("--speed-mode", choices=["physical", "percent"], default="physical")
    p.add_argument("--blend", type=float, default=0.5, help="blend radius mm between moves")
    p.add_argument("--queue-max", type=int, default=20, help="max queued motions on the controller")
    p.add_argument("--start-wait", type=float, default=0.6, help=argparse.SUPPRESS)   # see wait_idle()
    p.add_argument("--approach", type=float, default=None,
                   help="approach/retract height mm; default 0 (none) with --ref-point, else 20")
    p.add_argument("--quiet", action="store_true", help="do not print the command-by-command trace")
    p.add_argument("--spindle-do", type=int, default=-1, help="control-box DO for M3/M5 (-1 = off)")
    p.add_argument("--dwell-ms", action="store_true", help="treat G4 P as milliseconds (Marlin)")
    p.add_argument("--min-z", type=float, default=None, help="refuse to run if any base Z is below this")
    p.add_argument("--dry-run", action="store_true", help="parse and summarize only, no motion")
    p.add_argument("--check-ik", action="store_true", help="verify every target is reachable (needs robot)")
    p.add_argument("--check-step", type=float, default=25.0, metavar="MM",
                   help="the reachability check also tests a point every MM along lines and arcs (0 = targets only)")
    p.add_argument("--check-jump", type=float, default=60.0, help=argparse.SUPPRESS)   # deg, see check_ik()
    p.add_argument("--check-max", type=int, default=400, help=argparse.SUPPRESS)       # give up after this many
    p.add_argument("--preview", action="store_true", help="3D plot of the toolpath (matplotlib)")
    p.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    return p


def main():
    p = build_parser()
    a = p.parse_args()
    if a.ref_point and a.origin_here:
        p.error("--ref-point and --origin-here cannot be combined")

    with open(a.file, "r", errors="replace") as f:
        cmds, warnings = parse_gcode(f.read(), a.feed, a.dwell_ms, a.arc_split)
    for w in warnings[:15]:
        print("warning:", w)
    if len(warnings) > 15:
        print(f"... and {len(warnings) - 15} more warnings")

    needs_robot = bool(a.ref_point) or a.origin_here or a.check_ik or not a.dry_run
    try:
        robot = connect(a.ip) if needs_robot else None
        frame = resolve_frame(a, robot)
    except SetupError as e:
        sys.exit(str(e))

    follow = plan(cmds, frame, a)
    bbox = summarize(cmds, frame, a, follow)
    if bbox is None:
        return
    if a.dry_run and not a.quiet:
        # the same listing the live run prints while it sends each command
        for k, c in enumerate(cmds, 1):
            print(f"[{k:>{len(str(len(cmds)))}}/{len(cmds)}] " + describe(c, a))
    if a.min_z is not None and bbox[0][2] < a.min_z:
        sys.exit(f"Lowest point Z={bbox[0][2]:.1f} is below --min-z {a.min_z}. Aborting.")
    if a.preview:
        preview(cmds)

    streamer = Streamer(robot, frame, a) if robot else None
    try:
        if a.check_ik and not streamer.check_ik(cmds):
            sys.exit("Fix reachability (origin/scale/orientation) before running.")
        if a.dry_run:
            return
        if not a.yes and input(f"Robot will MOVE (global speed will be set to {a.global_speed} %). "
                               "Type 'yes' to start: ").strip().lower() != "yes":
            print("Cancelled.")
            return
        streamer.prepare()
        streamer.run(cmds)
    except KeyboardInterrupt:
        print("\nInterrupted - stopping robot.")
        streamer.emergency_stop()
    except Aborted as e:
        print(e)
        streamer.emergency_stop()
    except RobotCommandError as e:
        print("ERROR:", e)
        streamer.emergency_stop()
    finally:
        if robot is not None:
            robot.CloseRPC()


if __name__ == "__main__":
    main()