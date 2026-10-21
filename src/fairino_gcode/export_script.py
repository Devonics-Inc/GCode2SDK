#!/usr/bin/env python3
"""
export_script.py - write a plan out as a stand-alone Python script that runs it through the FAIRINO SDK.

The exported script is the job frozen: every robot command of the plan, one line per G-code command,
at the coordinates the plan was placed at, plus the small runtime the engine uses to send them (IK
referenced to the previous target, blending, motion-queue throttling, waiting for the arm, Ctrl-C
stops the arm). Every move runs at the one global speed % of the job; F and S in the G-code are ignored. It needs Python 3 and the FAIRINO SDK - not this application.

Before it moves, the script checks that the controller has the job's tool / workpiece frame active
and that it finds a joint position for every target. The finer check along the path (a point every
--check-step mm) is only done by the application, before exporting.

    from .export_script import export_plan
    export_plan(plan, "part_fairino.py", ip="192.168.58.25")
"""
from __future__ import annotations

import datetime
import os

try:                                            # inside the package
    from . import gcode_to_fairino as eng
    from . import APP_NAME, __version__
except ImportError:                             # gcode_to_fairino.py run as a plain script
    import gcode_to_fairino as eng
    APP_NAME, __version__ = "FAIRINO G-code Runner", "?"


def num(v) -> str:
    """A float as Python source, exactly (repr keeps every digit the plan has)."""
    return repr(float(v))


def vec(p) -> str:
    return "[" + ", ".join(num(v) for v in p) + "]"


def program_lines(cmds, a):
    """The body of program(job): one call per robot step, in the order Streamer.run() sends them.
    Returns (lines, number of steps, number of target poses)."""
    motions = [c for c in cmds if c.kind in eng.MOTION_KINDS]
    if not motions:
        raise ValueError("The plan has no moves to export.")
    first, lift = motions[0], float(a.approach or 0.0)
    out, n_steps, n_targets = [], 0, 0

    def feed_move(c):                           # Cartesian moves, which can blend into each other
        return c is not None and c.kind in eng.MOTION_KINDS and c.instr != "MoveJ"

    def hop(pose, line, label):                 # rapid move `lift` mm above a pose (approach / retract)
        nonlocal n_steps, n_targets
        up = pose[:2] + [round(pose[2] + lift, 4)] + pose[3:]
        if a.g0 == "movej":
            out.append(f"job.movej({vec(up)}, {line}, {label!r})")
        else:
            out.append(f"job.movel({vec(up)}, EXACT, {line}, {label!r})")
        n_steps += 1
        n_targets += 1

    if lift > 0:
        out.append(f"# approach: {lift:g} mm above the first point")
        hop(first.poses[-1], first.line, "(approach)")

    last_pose = first.poses[-1] if lift > 0 else None
    for idx, c in enumerate(cmds):
        nxt = cmds[idx + 1] if idx + 1 < len(cmds) else None
        blend = "BLEND" if feed_move(c) and feed_move(nxt) else "EXACT"
        if blend == "BLEND":                    # same cap as Streamer.run(): the blend must fit short moves
            fit = eng.blend_limit(c, nxt, float(getattr(a, "scale", 1.0)))
            if fit < a.blend:
                blend = f"min(BLEND, {num(round(fit, 3))})"
        where = f"{c.line}, {c.src!r}"
        if c.kind == "line":
            if c.instr == "MoveJ":
                out.append(f"job.movej({vec(c.poses[0])}, {where})")
            else:
                out.append(f"job.movel({vec(c.poses[0])}, {blend}, {where})")
            last_pose = c.poses[0]
        elif c.kind in ("arc", "circle"):
            call = "circle" if c.kind == "circle" else "movec"
            out.append(f"job.{call}({vec(c.poses[0])}, {vec(c.poses[1])}, {blend}, {where})")
            if c.kind == "arc":                 # a full circle ends where it began
                last_pose = c.poses[1]
        elif c.kind == "dwell":
            out.append(f"job.dwell({num(c.seconds)}, {where})")
        elif c.kind == "m" and c.code in (3, 4, 5):
            out.append(f"job.set_do({c.code != 5}, {where})")
        elif c.kind == "m" and c.code in (0, 1):
            out.append(f"job.pause({where})")
        else:
            continue
        n_steps += 1
        n_targets += len(c.poses or ())

    out.append("job.end()                   # wait for the arm, switch the M3 output off")
    if lift > 0 and last_pose is not None:
        out.append(f"# retract: {lift:g} mm above the last point")
        hop(last_pose, motions[-1].line, "(retract)")
    return out, n_steps, n_targets


def build_script(cmds, a, source: str, summary=(), ip: str = "192.168.58.25", checked: bool = False,
                 note: str = "", script_name: str = "") -> str:
    """The text of the stand-alone script for these planned commands and settings."""
    body, n_steps, n_targets = program_lines(cmds, a)
    name = os.path.basename(source)
    script = script_name or os.path.basename(default_name(source))
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    check_line = ("The whole path was checked for reachability with the robot before exporting."
                  if checked else "The path was NOT checked with the robot before exporting.")
    head = "\n".join("    " + s for s in summary)
    errors = "\n".join(f"    {k}: {v!r}," for k, v in sorted(eng.ERROR_TEXT.items()))
    rows = [(f"ROBOT_IP = {ip!r}", ""),
            (f"TOOL, USER = {int(a.tool)}, {int(a.user)}", "must be the frames active on the controller"),
            (f"GLOBAL_SPEED = {int(a.global_speed)}",
             "% of full speed: every move, or only MoveJ when SPEED_MODE is physical (F / S are ignored)"),
            (f"SPEED_MODE = {('physical' if eng.physical(a) else 'percent')!r}",
             '"physical": MoveL / MoveC / Circle at TCP_SPEED mm/s and ACC mm/s^2'),
            (f"TCP_SPEED = {num(getattr(a, 'tcp_speed', 20.0))}", "mm/s (physical mode)"),
            (f"ACC = {num(getattr(a, 'acc', 200.0))}", "mm/s^2 (physical mode) - lower it if a move fails with 183"),
            (f"BLEND = {num(a.blend)}", "mm, blend radius between moves that run on into each other"),
            ("EXACT = -1.0", "the move ends exactly on its point"),
            (f"QUEUE_MAX = {int(a.queue_max)}", "moves queued on the controller at most"),
            (f"SPINDLE_DO = {int(a.spindle_do)}", "control-box DO switched by M3 / M5 (-1 = none)"),
            (f"START_WAIT = {num(a.start_wait)}", 's to wait for "motion done" to drop after a move is sent'),
            (f"STEPS, TARGETS = {n_steps}, {n_targets}", "")]
    width = max(len(code) for code, _ in rows) + 2
    settings = "\n".join((code.ljust(width) + "# " + why) if why else code for code, why in rows)
    notes = "".join(f"\n{line}" for line in note.splitlines()) + ("\n" if note else "")
    return f'''#!/usr/bin/env python3
"""
{name} on a FAIRINO cobot, through the FAIRINO Python SDK.

Exported by {APP_NAME} {__version__} on {stamp} from {name}.
{head}
{check_line}
{notes}
The coordinates below were fixed when this was exported. If the workpiece, the teach point or the
tool / workpiece frame has changed since, export again.

Before anything moves, this script checks that tool {a.tool} / workpiece {a.user} are the active frames
and that the controller finds a joint position for every target. Then it asks for confirmation.

    python {script:<28} # robot at ROBOT_IP below
    python {script} --ip 192.168.58.2
    python {script} --check  # check only, the robot does not move

Ctrl-C stops the arm. Keep the emergency stop within reach.
"""
import argparse
import math
import sys
import time

try:
    from fairino import Robot           # pip install fairino
except ImportError:
    import Robot                        # or the SDK's Robot.py next to this script

# ============================================================================ job settings
{settings}

ERROR_TEXT = {{
{errors}
}}


# ============================================================================ the program
def program(job):
''' + "\n".join("    " + line for line in body) + "\n\n" + RUNTIME


RUNTIME = r'''
# ============================================================================ runtime
class RobotError(RuntimeError):
    pass


def unpack(ret):
    """SDK calls return either an int error code or a tuple (error, values...)."""
    if isinstance(ret, (list, tuple)):
        return ret[0], ret[1:]
    return ret, ()


def fmt(p):
    return "[" + ", ".join(f"{round(v, 3) + 0.0:.3f}" for v in p) + "]"


FEED_SPEED = f"{TCP_SPEED:g} mm/s" if SPEED_MODE == "physical" else f"{GLOBAL_SPEED} %"   # MoveL / MoveC / Circle


class Job:
    """Sends the program to the robot. dry=True only asks the controller for a joint position for every
    target and notes the ones it cannot find - nothing moves."""

    def __init__(self, robot, dry=False):
        self.robot, self.dry = robot, dry
        self.last_j = None              # joint position of the previous target: the IK reference
        self.last_pose = None           # where the previous move leaves the TCP
        self.spindle_on = False
        self.count = 0
        self.problems = []              # dry run: the targets without a joint position

    # ---- helpers
    def check(self, ret, what, line):
        err = unpack(ret)[0]
        if err != 0:
            meaning = f": {ERROR_TEXT[err]}" if err in ERROR_TEXT else ""
            raise RobotError(f"{what} failed with error {err}{meaning} (G-code line {line})")

    def ik(self, pose, line):
        """IK referenced to the previous target, so the solution branch stays consistent."""
        err, rest = unpack(self.robot.GetInverseKinRef(0, pose, self.last_j))
        if err != 0 or not rest or rest[0] is None:
            meaning = f" - {ERROR_TEXT[err]}" if err in ERROR_TEXT else ""
            raise RobotError(f"no joint position for {fmt(pose)} (error {err}{meaning})")
        self.last_j = [float(q) for q in rest[0]]
        return self.last_j

    def joints(self, poses, line, src):
        if not self.dry:
            return [self.ik(p, line) for p in poses]
        for p in poses:
            try:
                self.ik(p, line)
            except RobotError as e:
                self.problems.append(f"line {line}  {src}: {e}")
        return None

    def current_joints(self):
        err, rest = unpack(self.robot.GetActualJointPosDegree())
        if err != 0:
            raise RobotError(f"GetActualJointPosDegree failed ({err})")
        return [float(q) for q in rest[0]]

    def say(self, line, src, text):
        self.count += 1
        print(f"[{self.count:>{len(str(STEPS))}}/{STEPS}] N{line:<5d} {src[:32]:<32} -> {text}", flush=True)

    def fault(self):
        err, rest = unpack(self.robot.GetRobotErrorCode())
        if err == 0 and rest and rest[0][0] != 0:
            raise RobotError(f"robot fault: main code {rest[0][0]}, sub code {rest[0][1]}")

    def idle(self):
        _, q = unpack(self.robot.GetMotionQueueLength())
        _, done = unpack(self.robot.GetRobotMotionDone())
        return bool(q and q[0] == 0 and done and done[0] == 1)

    def wait_idle(self, just_sent=False):
        """Wait until the arm has done everything it was given. Right after sending a move, first wait
        (briefly) until it is seen busy: "motion done" can still be the value from before."""
        if just_sent:
            deadline = time.time() + START_WAIT
            while self.idle() and time.time() < deadline:
                time.sleep(0.005)
        else:
            time.sleep(0.05)
        while not self.idle():
            self.fault()
            time.sleep(0.02)

    def throttle(self):
        """Keep the controller's motion queue from overflowing while blending."""
        while True:
            err, rest = unpack(self.robot.GetMotionQueueLength())
            if err != 0 or rest[0] < QUEUE_MAX:
                return
            self.fault()
            time.sleep(0.01)

    # percent: every move at 100 %, so GLOBAL_SPEED (set in prepare) is the speed of all of them.
    # physical: ovl = TCP speed in mm/s, oacc = acceleration in mm/s^2; GLOBAL_SPEED does not scale them.
    SPEED = (dict(vel=100.0, ovl=TCP_SPEED, oacc=ACC, velAccParamMode=1) if SPEED_MODE == "physical"
             else dict(vel=100.0, ovl=100.0, oacc=100.0, velAccParamMode=0))

    # ---- the steps of the program
    def movej(self, pose, line, src):
        """G0 -> MoveJ at 100 % joint speed (scaled by GLOBAL_SPEED like every move)."""
        js = self.joints([pose], line, src)
        if self.dry:
            self.last_pose = pose
            return
        self.wait_idle()
        self.say(line, src, f"MoveJ  {fmt(pose)} @ {GLOBAL_SPEED} %")
        self.check(self.robot.MoveJ(joint_pos=js[0], tool=TOOL, user=USER, desc_pos=pose, vel=100.0, blendT=0.0),
                   "MoveJ", line)
        self.wait_idle(just_sent=True)
        self.last_pose = pose

    def movel(self, pose, blend, line, src):
        js = self.joints([pose], line, src)
        if not self.dry:
            self.throttle()
            self.say(line, src, f"MoveL  {fmt(pose)} @ {FEED_SPEED}")
            ret = self.robot.MoveL(desc_pos=pose, tool=TOOL, user=USER, joint_pos=js[0],
                                   blendR=max(blend, 0.0), **self.SPEED)
            self.check(ret, "MoveL", line)
            if blend < 0:
                self.wait_idle(just_sent=True)
        self.last_pose = pose

    def _arc(self, full, mid, end, blend, line, src):
        name = "Circle" if full else "MoveC"
        j_start = self.last_j
        js = self.joints([mid, end], line, src)
        if not self.dry:
            self.throttle()
            self.say(line, src, f"{name:<6} via {fmt(mid)} to {fmt(end)} @ {FEED_SPEED}")
            move = self.robot.Circle if full else self.robot.MoveC
            s = self.SPEED
            ret = move(desc_pos_p=mid, tool_p=TOOL, user_p=USER, desc_pos_t=end, tool_t=TOOL, user_t=USER,
                       joint_pos_p=js[0], joint_pos_t=js[1], vel_p=s["vel"], vel_t=s["vel"], ovl=s["ovl"],
                       oacc=s["oacc"], blendR=max(blend, 0.0), velAccParamMode=s["velAccParamMode"])
            self.check(ret, name, line)
            if blend < 0:
                self.wait_idle(just_sent=True)
        if full:
            self.last_j = j_start or self.last_j    # a full circle ends where it started
        else:
            self.last_pose = end

    def movec(self, mid, end, blend, line, src):
        """G2 / G3 arc: path point, target point."""
        self._arc(False, mid, end, blend, line, src)

    def circle(self, mid, end, blend, line, src):
        """G2 / G3 full circle: the points at 1/4 and 1/2 turn; it finishes back at its start."""
        self._arc(True, mid, end, blend, line, src)

    def dwell(self, seconds, line, src):
        if self.dry:
            return
        self.wait_idle()
        self.say(line, src, f"wait {seconds:g} s")
        time.sleep(seconds)

    def set_do(self, on, line, src):
        """M3 / M4 / M5: the tool output (SPINDLE_DO) on or off."""
        if self.dry:
            return
        self.wait_idle()
        if SPINDLE_DO < 0:
            self.say(line, src, "skipped (SPINDLE_DO = -1)")
            return
        self.say(line, src, f"SetDO({SPINDLE_DO}, {int(on)})")
        self.check(self.robot.SetDO(SPINDLE_DO, int(on)), "SetDO", line)
        self.spindle_on = bool(on)

    def pause(self, line, src):
        """M0 / M1: wait for the operator."""
        if self.dry:
            return
        self.wait_idle()
        self.say(line, src, "pause")
        input(f"[line {line}] {src} pause - press Enter to continue...")

    def end(self):
        if self.dry:
            return
        self.wait_idle()
        if self.spindle_on:
            self.robot.SetDO(SPINDLE_DO, 0)
            self.spindle_on = False

    # ---- start / stop
    def prepare(self):
        self.fault()
        self.check(self.robot.Mode(0), "Mode(auto)", 0)
        self.check(self.robot.RobotEnable(1), "RobotEnable", 0)
        self.check(self.robot.SetSpeed(GLOBAL_SPEED), "SetSpeed", 0)
        time.sleep(0.5)
        self.last_j = self.current_joints()

    def stop(self):
        """Stop the arm, then switch the tool output off."""
        def twice(call):                # after Ctrl-C the first call on the connection can fail
            try:
                return call()
            except Exception:
                return call()
        try:
            twice(self.robot.StopMotion)
        finally:
            if self.spindle_on and SPINDLE_DO >= 0:
                twice(lambda: self.robot.SetDO(SPINDLE_DO, 0))
                self.spindle_on = False


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--ip", default=ROBOT_IP, help=f"robot IP address (default {ROBOT_IP})")
    p.add_argument("--check", action="store_true", help="only check the frames and the targets; no motion")
    p.add_argument("-y", "--yes", action="store_true", help="skip the confirmation prompt")
    args = p.parse_args(argv)

    robot = Robot.RPC(args.ip)
    if not Robot.RPC.is_connect:
        sys.exit(f"Could not connect to the robot at {args.ip}.")
    job = None
    try:
        time.sleep(0.5)                 # let the first state packet arrive
        tool = int(unpack(robot.GetActualTCPNum())[1][0])
        user = int(unpack(robot.GetActualWObjNum())[1][0])
        if (tool, user) != (TOOL, USER):
            sys.exit(f"The controller has tool {tool} / workpiece {user} active, but this job runs in tool {TOOL} / "
                     f"workpiece {USER}. Activate those frames on the controller and run it again.")
        dry = Job(robot, dry=True)
        dry.last_j = dry.current_joints()
        program(dry)
        if dry.problems:
            print("\n".join(dry.problems[:20]))
            sys.exit(f"{len(dry.problems)} of {TARGETS} targets cannot be reached. Nothing was moved.")
        print(f"Frames tool {TOOL} / workpiece {USER} active; all {TARGETS} targets can be reached.")
        if args.check:
            return
        if not args.yes and input(f"The robot will MOVE ({STEPS} steps, global speed {GLOBAL_SPEED} %). "
                                  "Type 'yes' to start: ").strip().lower() != "yes":
            print("Cancelled.")
            return
        job = Job(robot)
        job.prepare()
        t0 = time.time()
        program(job)
        print(f"Done in {time.time() - t0:.1f} s")
    except KeyboardInterrupt:
        print("\nInterrupted - stopping the robot.")
        if job is not None:
            job.stop()
    except RobotError as e:
        print("ERROR:", e)
        if job is not None:
            job.stop()
        sys.exit(1)
    finally:
        robot.CloseRPC()


if __name__ == "__main__":
    main()
'''


# ============================================================================ simple style
def short(v) -> str:
    """A float rounded to 0.0001 (mm / deg), written as short as possible."""
    v = round(float(v), 4) + 0.0
    return repr(int(v)) + ".0" if v == int(v) else repr(v)


def svec(p) -> str:
    return "[" + ", ".join(short(v) for v in p) + "]"


def build_simple_script(cmds, a, source: str, summary=(), ip: str = "192.168.58.25", note: str = "") -> str:
    """The job as a flat list of SDK calls, like the FAIRINO SDK examples: one robot.MoveJ / MoveL /
    MoveC / Circle per G-code command, every argument written out. Every move is blocking
    (blendR / blendT = -1): each call returns when the arm is there, so nothing can pile up
    in the controller's queue, and M-codes, dwells and pauses happen with the arm at rest. Return codes
    are not checked - the plan was checked by the app before it could be exported."""
    motions = [c for c in cmds if c.kind in eng.MOTION_KINDS]
    if not motions:
        raise ValueError("The plan has no moves to export.")
    first, lift = motions[0], float(a.approach or 0.0)
    do = int(a.spindle_do)
    body, moved, spindle_on = [], [False], False
    # physical mode: MoveL / MoveC / Circle at TCP_SPEED mm/s and ACC mm/s^2 instead of the global %
    phys = eng.physical(a)
    units = ", ovl=TCP_SPEED, oacc=ACC, velAccParamMode=1" if phys else ""

    # every move at vel=100 %: robot.SetSpeed(<global %>) below is the speed of all of them
    def movej(pose, c, label):
        moved[0] = True
        body.append(f"# {label}")
        body.append(f"p = {svec(pose)}")
        body.append("robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=100, blendT=-1)")

    def movel(pose, c, comment):
        body.append(f"# {comment}")
        body.append(f"robot.MoveL(desc_pos={svec(pose)}, tool=TOOL, user=USER, vel=100, blendR=-1{units})")

    def where(c):
        return f"N{c.line} {c.src}"

    rapid = eng.Cmd("line", first.line, rapid=True, spindle=first.spindle)
    if lift > 0:
        up = first.poses[-1][:2] + [round(first.poses[-1][2] + lift, 4)] + first.poses[-1][3:]
        if a.g0 == "movej":
            movej(up, rapid, f"approach: {lift:g} mm above the first point")
        else:
            movel(up, rapid, f"approach: {lift:g} mm above the first point")
            moved[0] = True
    last_pose = first.poses[-1] if lift > 0 else None

    for c in cmds:
        if c.kind == "line" and c.instr == "MoveJ":
            movej(c.poses[0], c, where(c))
            last_pose = c.poses[0]
        elif c.kind == "line":
            movel(c.poses[0], c, where(c))
            moved[0], last_pose = True, c.poses[0]
        elif c.kind in ("arc", "circle"):
            name = "Circle" if c.kind == "circle" else "MoveC"
            pad = " " * (len(name) + 7)
            body.append(f"# {where(c)}")
            body.append(f"robot.{name}(desc_pos_p={svec(c.poses[0])}, tool_p=TOOL, user_p=USER,\n"
                        f"{pad}desc_pos_t={svec(c.poses[1])}, tool_t=TOOL, user_t=USER,\n"
                        f"{pad}vel_p=100, vel_t=100, blendR=-1{units})")
            moved[0] = True
            if c.kind == "arc":
                last_pose = c.poses[1]
        elif c.kind == "dwell":
            body.append(f"# {where(c)}")
            body.append(f"time.sleep({short(c.seconds)})")
        elif c.kind == "m" and c.code in (3, 4, 5):
            on = int(c.code != 5)
            body.append(f"# {where(c)}")
            if do >= 0:
                body.append(f"robot.SetDO({do}, {on})")
                spindle_on = bool(on)
            else:
                body.append(f"# (no output chosen for M3 / M5 - this would be robot.SetDO(<DO>, {on}))")
        elif c.kind == "m" and c.code in (0, 1):
            body.append(f"# {where(c)}")
            body.append(f"input('N{c.line}: paused - press Enter to continue ')")
    if spindle_on:
        body.append("# switch the output off at the end")
        body.append(f"robot.SetDO({do}, 0)")
    if lift > 0 and last_pose is not None:
        up = last_pose[:2] + [round(last_pose[2] + lift, 4)] + last_pose[3:]
        if a.g0 == "movej":
            movej(up, rapid, f"retract: {lift:g} mm above the last point")
        else:
            movel(up, rapid, f"retract: {lift:g} mm above the last point")

    name = os.path.basename(source)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    head = "\n".join(summary)
    notes = f"\n{note}\n" if note else ""
    rows = [(f"TOOL, USER = {int(a.tool)}, {int(a.user)}", "must be the frames active on the controller")]
    if phys:
        rows += [(f"TCP_SPEED = {short(a.tcp_speed)}", "mm/s, every MoveL / MoveC / Circle"),
                 (f"ACC = {short(a.acc)}", "mm/s^2 - lower it if a move fails with error 183")]
    width = max(len(code) for code, _ in rows) + 2
    settings = [code.ljust(width) + "# " + why for code, why in rows]
    return f'''"""
{name} on a FAIRINO cobot - exported by {APP_NAME} {__version__} on {stamp}.
{head}
{notes}
The coordinates are fixed: if the workpiece or the frames change, export again.
Keep the emergency stop within reach. Ctrl-C stops the arm.
"""
import time
from fairino import Robot

{chr(10).join(settings)}

# Establish connection with robot controller
robot = Robot.RPC({ip!r})


def joints_for(pose):
    """Joint angles for a Cartesian pose, closest to where the arm is now (MoveJ needs them)."""
    return robot.GetInverseKinRef(0, pose, robot.GetActualJointPosDegree()[1])[1]


# Refuse to run in the wrong frames: the coordinates below are expressed in them
if (robot.GetActualTCPNum()[1], robot.GetActualWObjNum()[1]) != (TOOL, USER):
    raise SystemExit(f"Activate tool {{TOOL}} / workpiece {{USER}} on the controller first.")
if input("The robot will MOVE. Type yes to start: ").strip().lower() != "yes":
    raise SystemExit("Cancelled.")

robot.Mode(0)            # automatic mode
robot.RobotEnable(1)
robot.SetSpeed({int(a.global_speed):<3})      # % of full speed: {'MoveJ only' if phys else 'every move'}

try:
''' + "\n".join(("    " + line).replace("\n", "\n    ") for line in body) + '''
    print("Done.")
except KeyboardInterrupt:
    robot.StopMotion()
    print("Stopped.")
finally:
    robot.CloseRPC()
'''


def export_cmds(cmds, a, source: str, path: str, summary=(), ip: str = "192.168.58.25", checked: bool = False,
                note: str = "", style: str = "simple") -> str:
    """Write planned commands (after eng.plan()) and their settings to `path` as a stand-alone script.
    style "simple": a flat list of SDK calls; "checked": the same commands with the app's runtime
    (pre-flight frame and reach check, IK referenced to the previous target)."""
    if style == "simple":
        text = build_simple_script(cmds, a, source, summary, ip, note)
    else:
        text = build_script(cmds, a, source, summary, ip, checked, note, os.path.basename(path))
    compile(text, path, "exec")                 # a broken export must never reach the robot
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return path


def export_plan(plan, path: str, ip: str = "192.168.58.25", checked: bool = True, note: str = "",
                style: str = "simple") -> str:
    """Write a plan_model.Plan (placed on the robot) to `path`."""
    return export_cmds(plan.cmds, plan.settings, plan.path, path, plan.summary, ip, checked, note, style)


def default_name(gcode_path: str) -> str:
    stem, _ = os.path.splitext(gcode_path)
    return stem + "_fairino.py"
