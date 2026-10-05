#!/usr/bin/env python3
"""
sim_robot.py - a stand-in for the FAIRINO controller, for the GUI's --simulate mode and for the tests.

It implements the part of the SDK's ``Robot.RPC`` interface that gcode_to_fairino.py uses and moves a
virtual TCP in real time along MoveJ / MoveL / MoveC / Circle at the commanded speed, so a whole job
can be watched and tested without a robot. serve_status() also publishes that TCP as a CNDE status
stream (through fake_cnde_robot.py), so the GUI's Connect / live TCP path is the real one.

What it models: straight lines, arcs and full circles through the commanded points, speeds
(mm/s or percent, times the global speed), blocking (blend = -1) and queued moves, StopMotion,
digital outputs, teach points, the active tool / workpiece numbers and a reach limit for IK.
What it does NOT model: real kinematics (the "joints" are a linear function of the pose),
acceleration, blending radii, workpiece-frame transforms, collisions. It proves the command stream,
not that the real arm can follow it.
"""
import math
import threading
import time
from types import SimpleNamespace

JOINT_MAX_SPEED = 180.0          # deg/s, what 100 % means for MoveJ
ERR_NO_IK = 112                  # error codes are arbitrary non-zero values
ERR_BAD_ARC = 101
ERR_STOPPED = 3


def fake_joints(pose):
    """Invertible stand-in for inverse kinematics: 10 mm = 1 deg, orientation halved."""
    return [pose[0] / 10.0, pose[1] / 10.0, pose[2] / 10.0, pose[3] / 2.0, pose[4] / 2.0, pose[5] / 2.0]


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def _lerp_pose(p, q, u):
    """Position linearly, each orientation angle the short way round."""
    return [p[i] + (q[i] - p[i]) * u for i in range(3)] + [p[i] + _wrap(q[i] - p[i]) * u for i in range(3, 6)]


def _circle_through(a, b, c):
    """Centre, radius and an in-plane basis (e1, e2) of the circle through three 3-D points.
    e1 points from the centre to a; e2 is 90 deg further in the direction a -> b -> c."""
    ab = [b[i] - a[i] for i in range(3)]
    ac = [c[i] - a[i] for i in range(3)]
    n = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]]
    n2 = sum(v * v for v in n)
    if n2 < 1e-12:
        return None                                      # collinear or coincident points

    def cross(u, v):
        return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]

    ab2, ac2 = sum(v * v for v in ab), sum(v * v for v in ac)
    t1, t2 = cross(n, ab), cross(ac, n)
    centre = [a[i] + (ac2 * t1[i] + ab2 * t2[i]) / (2.0 * n2) for i in range(3)]
    e1 = [a[i] - centre[i] for i in range(3)]
    radius = math.sqrt(sum(v * v for v in e1))
    nn = math.sqrt(n2)
    e2 = cross([v / nn for v in n], e1)                  # same length as e1, rotated +90 deg about n
    return centre, radius, e1, e2


def _angle_on(circle, p):
    centre, radius, e1, e2 = circle
    d = [p[i] - centre[i] for i in range(3)]
    return math.atan2(sum(d[i] * e2[i] for i in range(3)), sum(d[i] * e1[i] for i in range(3))) % (2 * math.pi)


class SimRobot:
    """Drop-in replacement for ``Robot.RPC`` (only the calls gcode_to_fairino.py makes)."""

    is_connect = True

    def __init__(self, time_scale=1.0, start_pose=(300.0, -200.0, 400.0, 180.0, 0.0, 0.0),
                 tool=0, user=0, reach=1500.0, max_tcp_speed=250.0, teach_points=None, state_lag=0.0):
        self.time_scale = float(time_scale)      # > 1 = faster than real time
        self.state_lag = float(state_lag)        # s: "motion done" / queue length are this much out of
        self._last_command = 0.0                 #    date, as with a real status stream
        self._speed = 0.0                        # current TCP speed, mm/s
        self.reach = reach
        self.max_tcp_speed = max_tcp_speed       # what 100 % means in percent mode
        self.active_tool, self.active_user = tool, user
        self.teach_points = {
            "ref":      ([0.0, 0.0, 50.0, 180.0, 0.0, 0.0], 0, 0),       # tool straight down
            "ref_tilt": ([0.0, 0.0, 50.0, 180.0, 30.0, 0.0], 0, 0),      # tool tilted 30 deg
            "ref_wobj": ([0.0, 0.0, 50.0, 180.0, 0.0, 0.0], 1, 2),       # recorded in tool 1 / workpiece 2
        }
        self.teach_points.update(teach_points or {})
        self.global_speed = 100
        self.do = {}                             # digital outputs
        self.calls = []                          # (name, kwargs) of everything that was commanded
        self.travelled = []                      # XYZ every <= 0.25 mm along what was actually executed
        self.fail_next = {}                      # {"MoveL": 14} -> the next MoveL returns error 14
        self.fault = (0, 0)                      # (main, sub) error code reported by GetRobotErrorCode
        self.robot_state_pkg = SimpleNamespace(tl_cur_pos=list(start_pose))

        self._cv = threading.Condition()
        self._queue = []                         # segments waiting: dict(path=fn, duration=s, done=Event)
        self._active = None
        self._planned_end = list(start_pose)     # where the TCP will be once the queue has run
        self._generation = 0                     # bumped by StopMotion
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="SimRobot", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ motion engine
    @property
    def pose(self):
        return list(self.robot_state_pkg.tl_cur_pos)

    def _run(self):
        tick = 0.005
        while True:
            with self._cv:
                while not self._queue and not self._closed:
                    self._cv.wait()
                if self._closed:
                    return
                seg = self._active = self._queue.pop(0)
                generation = self._generation
                self._speed = seg["length"] / seg["duration"] if seg["duration"] > 0 else 0.0
            t0, stopped, u = time.monotonic(), False, 0.0
            while True:
                elapsed = (time.monotonic() - t0) * self.time_scale
                with self._cv:
                    if generation != self._generation or self._closed:
                        stopped = True
                        break
                    u = 1.0 if seg["duration"] <= 0 else min(1.0, elapsed / seg["duration"])
                    self.robot_state_pkg.tl_cur_pos = seg["path"](u)
                if u >= 1.0:
                    break
                time.sleep(min(tick, max((seg["duration"] - elapsed) / self.time_scale, 0.0)))
            with self._cv:
                # geometric record of the stretch that was executed, independent of timing
                n = max(1, min(4000, math.ceil(seg["length"] * u / 0.25)))
                self.travelled += [seg["path"](u * k / n)[:3] for k in range(1, n + 1)]
                seg["stopped"] = stopped
                self._active = None
                self._speed = 0.0
                seg["done"].set()
                self._cv.notify_all()

    def _enqueue(self, name, path, length, duration, blocking, kwargs):
        """length in mm (for the travelled record), duration in seconds at time_scale 1."""
        with self._cv:
            self.calls.append((name, kwargs))
            code = self.fail_next.pop(name, 0)
            if code:
                return code
            seg = dict(path=path, length=length, duration=duration, done=threading.Event(), stopped=False)
            self._planned_end = path(1.0)
            if self._active is None and not self._queue:
                self._last_command = time.monotonic()        # idle -> busy: the status lags behind
            self._queue.append(seg)
            self._cv.notify_all()
        if blocking:
            seg["done"].wait()
            return ERR_STOPPED if seg["stopped"] else 0
        return 0

    def _tcp_speed(self, vel, ovl, mode):
        v = ovl if mode == 1 else self.max_tcp_speed * vel / 100.0 * ovl / 100.0
        return v * self.global_speed / 100.0

    def _reachable(self, pose):
        return math.sqrt(sum(v * v for v in pose[:3])) <= self.reach

    # ------------------------------------------------------------------ motion commands
    def MoveJ(self, joint_pos, tool, user, desc_pos, vel=20.0, blendT=-1.0, **_):
        start, end = list(self._planned_end), list(map(float, desc_pos))
        travel = max(abs(a - b) for a, b in zip(fake_joints(start), fake_joints(end)))
        speed = JOINT_MAX_SPEED * vel / 100.0 * self.global_speed / 100.0       # deg/s
        return self._enqueue("MoveJ", lambda u: _lerp_pose(start, end, u), math.dist(start[:3], end[:3]),
                             travel / max(speed, 1e-6), blendT < 0,
                             dict(desc_pos=end, joint_pos=list(joint_pos), tool=tool, user=user,
                                  vel=vel, blendT=blendT))

    def MoveL(self, desc_pos, tool, user, joint_pos=None, vel=20.0, ovl=100.0, blendR=-1.0, oacc=100.0,
              velAccParamMode=0, **_):
        start, end = list(self._planned_end), list(map(float, desc_pos))
        length = math.dist(start[:3], end[:3])
        return self._enqueue("MoveL", lambda u: _lerp_pose(start, end, u), length,
                             length / max(self._tcp_speed(vel, ovl, velAccParamMode), 1e-6), blendR < 0,
                             dict(desc_pos=end, tool=tool, user=user, vel=vel, ovl=ovl, blendR=blendR,
                                  oacc=oacc, velAccParamMode=velAccParamMode))

    def _circular(self, name, full, desc_pos_p, desc_pos_t, tool, user, vel, ovl, blendR, oacc, mode):
        start, via, end = list(self._planned_end), list(map(float, desc_pos_p)), list(map(float, desc_pos_t))
        kwargs = dict(desc_pos_p=via, desc_pos_t=end, tool=tool, user=user, vel=vel, ovl=ovl, blendR=blendR,
                      oacc=oacc, velAccParamMode=mode)
        circle = _circle_through(start[:3], via[:3], end[:3])
        if circle is None:
            with self._cv:
                self.calls.append((name, kwargs))
            return ERR_BAD_ARC
        centre, radius, e1, e2 = circle
        a_via, a_end = _angle_on(circle, via), _angle_on(circle, end)
        sweep = 2 * math.pi if full else a_end
        # orientation: start -> via -> end (-> start again for a full circle), by angle travelled
        knots = [(0.0, start), (a_via, via), (a_end, end)] + ([(2 * math.pi, start)] if full else [])

        def path(u):
            ang = sweep * u
            xyz = [centre[i] + math.cos(ang) * e1[i] + math.sin(ang) * e2[i] for i in range(3)]
            for (a0, p0), (a1, p1) in zip(knots, knots[1:]):
                if ang <= a1 + 1e-12:
                    return xyz + _lerp_pose(p0, p1, (ang - a0) / max(a1 - a0, 1e-12))[3:]
            return xyz + list(knots[-1][1][3:])

        length = radius * sweep
        return self._enqueue(name, path, length, length / max(self._tcp_speed(vel, ovl, mode), 1e-6),
                             blendR < 0, kwargs)

    def MoveC(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=None, joint_pos_t=None,
              vel_p=20.0, vel_t=20.0, ovl=100.0, blendR=-1.0, oacc=100.0, velAccParamMode=0, **_):
        return self._circular("MoveC", False, desc_pos_p, desc_pos_t, tool_t, user_t, vel_t, ovl, blendR,
                              oacc, velAccParamMode)

    def Circle(self, desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, joint_pos_p=None, joint_pos_t=None,
               vel_p=20.0, vel_t=20.0, ovl=100.0, blendR=-1.0, oacc=100.0, velAccParamMode=0, **_):
        return self._circular("Circle", True, desc_pos_p, desc_pos_t, tool_t, user_t, vel_t, ovl, blendR,
                              oacc, velAccParamMode)

    def StopMotion(self):
        with self._cv:
            self.calls.append(("StopMotion", {}))
            self._generation += 1
            for seg in self._queue:
                seg["stopped"] = True
                seg["done"].set()
            self._queue.clear()
            self._planned_end = self.pose
            self._cv.notify_all()
        return 0

    # ------------------------------------------------------------------ state queries
    def _stale(self):
        """True while the status still shows the idle state from before the latest command."""
        return time.monotonic() - self._last_command < self.state_lag

    def busy(self):
        with self._cv:
            return self._active is not None or bool(self._queue)

    def GetMotionQueueLength(self):
        with self._cv:
            return 0, 0 if self._stale() else len(self._queue)

    def GetRobotMotionDone(self):
        with self._cv:
            return 0, int(self._stale() or (self._active is None and not self._queue))

    def GetActualTCPPose(self, flag=1):
        return 0, self.pose

    def GetActualJointPosDegree(self, flag=1):
        return 0, fake_joints(self.pose)

    def GetActualTCPNum(self, flag=1):
        return 0, self.active_tool

    def GetActualWObjNum(self, flag=1):
        return 0, self.active_user

    def GetRobotErrorCode(self):
        return 0, list(self.fault)

    def GetJointSoftLimitDeg(self, flag=1):
        return 0, [-175.0, 175.0] * 6

    def GetInverseKinRef(self, type, desc_pos, joint_pos_ref):
        if not self._reachable(desc_pos):
            return ERR_NO_IK, None
        return 0, fake_joints(desc_pos)

    def GetRobotTeachingPoint(self, name):
        if name not in self.teach_points:
            return 143, None
        pose, tool, user = self.teach_points[name]
        data = list(pose) + fake_joints(pose) + [tool, user, 100, 100, 0, 0, 0, 0]
        return 0, [str(v) for v in data]         # the SDK returns the fields as strings

    # ------------------------------------------------------------------ settings / IO
    def _record(self, name, **kwargs):
        with self._cv:
            kwargs["while_moving"] = self._active is not None or bool(self._queue)
            self.calls.append((name, kwargs))
            return self.fail_next.pop(name, 0)

    def Mode(self, mode):
        return self._record("Mode", mode=mode)

    def RobotEnable(self, state):
        return self._record("RobotEnable", state=state)

    def SetSpeed(self, vel):
        self.global_speed = int(vel)
        return self._record("SetSpeed", vel=int(vel))

    def SetDO(self, id, status, *_, **__):
        self.do[id] = status
        return self._record("SetDO", id=id, status=status)

    def CloseRPC(self):
        with self._cv:
            self._closed = True
            self._generation += 1
            for seg in self._queue:
                seg["done"].set()
            self._queue.clear()
            self._cv.notify_all()

    # ------------------------------------------------------------------ helpers for tests
    def moves(self):
        """The motion commands received so far, as (name, kwargs)."""
        with self._cv:
            return [(n, k) for n, k in self.calls if n in ("MoveJ", "MoveL", "MoveC", "Circle")]

    # ------------------------------------------------------------------ status stream
    def status(self, _t=None):
        """(pose, speed mm/s, moving, extra items) - what the CNDE status stream reports."""
        with self._cv:
            moving = self._active is not None or bool(self._queue)
            return (list(self.robot_state_pkg.tl_cur_pos), self._speed if moving else 0.0, moving,
                    dict(tool_id=self.active_tool, wobj_id=self.active_user,
                         main_code=self.fault[0], sub_code=self.fault[1]))

    def serve_status(self, port, stop=None):
        """Publish the status on UDP `port` in a background thread (see fake_cnde_robot.py)."""
        from . import fake_cnde_robot
        thread = threading.Thread(target=fake_cnde_robot.serve, args=(port, self.status),
                                  kwargs=dict(verbose=False, stop=stop), name="SimStatus", daemon=True)
        thread.start()
        return thread
