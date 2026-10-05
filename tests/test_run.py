"""Step 3: placing, checking, running and stopping a plan - against the simulated robot."""
import threading
import time

import numpy as np
import pytest
from conftest import DEMO, wait_until

from fairino_gcode import gcode_gui
from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode import sim_robot
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.run_control import HERE, MANUAL, REF, Job, RobotSession, RunSetup, RunSetupDialog

SMALL = """G21 G90
S6000 M3
G0 X0 Y0 Z5
G1 Z0 F600
G1 X40 Y0 F1200
G3 X50 Y10 I0 J10
M0
G1 X50 Y30
G2 I-10
G4 P0.05
G0 Z5 M5
"""


def settings(**kw):
    a = default_settings()
    a.start_wait = 0.05                     # the simulator's status has no lag (unless a test adds one)
    for key, value in kw.items():
        setattr(a, key, value)
    return a


def plan_points(plan):
    return np.vstack([plan.paths[k] for k in plan.motions])


def off_path(plan, travelled):
    """Largest distance from a planned point to the path the robot really took (mm)."""
    travelled = np.array(travelled)
    return max(np.min(np.linalg.norm(travelled - p, axis=1)) for p in plan_points(plan))


# ------------------------------------------------------------------ engine
def streamer_for(sim, path, **kw):
    a = settings(origin=[300.0, 0.0, 200.0], **kw)
    frame = eng.resolve_frame(a, sim, log=lambda _t: None)
    plan = build_plan(path, a, frame)
    s = eng.Streamer(sim, frame, a)
    s.log = lambda _t: None
    s.on_command = lambda _c, _t: None
    return s, plan


def test_command_line_mode_still_sends_blocking_moves():
    sim = sim_robot.SimRobot(time_scale=50)
    s, plan = streamer_for(sim, DEMO, approach=0.0)
    s.prepare()
    s.run(plan.cmds)
    blends = [k.get("blendR", k.get("blendT")) for _n, k in sim.moves()]
    assert -1.0 in blends
    assert off_path(plan, sim.travelled) < 0.3


def test_stoppable_mode_sends_no_blocking_move_and_follows_the_same_path():
    sim = sim_robot.SimRobot(time_scale=50)
    s, plan = streamer_for(sim, DEMO, approach=0.0)
    s.stoppable = True
    s.prepare()
    s.run(plan.cmds)
    blends = [k.get("blendR", k.get("blendT")) for _n, k in sim.moves()]
    assert min(blends) >= 0.0
    assert [n for n, _k in sim.moves()] == [plan.cmds[k].instr for k in plan.motions]
    assert off_path(plan, sim.travelled) < 0.3


def test_outputs_switch_only_when_the_arm_is_at_rest_even_with_a_lagging_status():
    sim = sim_robot.SimRobot(time_scale=10, state_lag=0.15)
    s, plan = streamer_for(sim, DEMO, approach=0.0, spindle_do=0, start_wait=0.6)
    s.stoppable = True
    s.prepare()
    s.run(plan.cmds)
    assert [(k["status"], k["while_moving"]) for n, k in sim.calls if n == "SetDO"] == [(1, False), (0, False)]


def test_abort_ends_the_run_within_a_moment_and_stops_the_arm():
    sim = sim_robot.SimRobot(time_scale=1)            # real time: the demo takes ~30 s
    s, plan = streamer_for(sim, DEMO, spindle_do=0)
    s.stoppable = True
    outcome = {}

    def work():
        try:
            s.prepare()
            s.run(plan.cmds)
        except eng.Aborted:
            outcome["at"] = time.monotonic()
            s.emergency_stop()

    thread = threading.Thread(target=work)
    thread.start()
    time.sleep(2.0)
    asked = time.monotonic()
    s.abort.set()
    thread.join(5)
    assert not thread.is_alive() and outcome["at"] - asked < 0.3
    assert ("StopMotion", {}) in sim.calls and sim.do[0] == 0
    time.sleep(0.1)
    here = sim.pose
    time.sleep(0.3)
    assert sim.pose == here and not sim.busy()


def test_emergency_stop_tries_again_when_the_first_call_fails():
    calls = []

    class Flaky:
        def StopMotion(self):
            calls.append("stop")
            if len(calls) == 1:
                raise RuntimeError("connection in the middle of a request")
            return 0

        def SetDO(self, number, value):
            calls.append(("do", number, value))
            return 0

    s = eng.Streamer(Flaky(), None, settings(spindle_do=3))
    s.spindle_on = True
    s.emergency_stop()
    assert calls == ["stop", "stop", ("do", 3, 0)] and s.abort.is_set() and not s.spindle_on


def test_refused_acceleration_is_halved_and_the_move_sent_again():
    """Seen on the real controller: MoveL answered 183 (acceleration above its limit)."""
    sim = sim_robot.SimRobot(time_scale=50)
    s, plan = streamer_for(sim, DEMO, approach=0.0, acc=400.0)
    logged = []
    s.log = logged.append
    sim.fail_next = {"MoveL": 183}
    s.stoppable = True
    s.prepare()
    s.run(plan.cmds)
    lines = [k["oacc"] for n, k in sim.calls if n == "MoveL"]
    assert lines[:2] == [400.0, 200.0] and set(lines[1:]) == {200.0}          # refused once, then kept lower
    assert [k["oacc"] for n, k in sim.calls if n in ("MoveC", "Circle")][0] == 200.0
    assert len(lines) == sum(plan.cmds[k].instr == "MoveL" for k in plan.motions) + 1
    assert any("refused the acceleration" in text for text in logged)
    assert off_path(plan, sim.travelled) < 0.3


def test_error_codes_are_explained():
    sim = sim_robot.SimRobot(time_scale=50)
    s, plan = streamer_for(sim, DEMO, approach=0.0, acc=10.0)                  # already at the floor: no retry
    sim.fail_next = {"MoveL": 183}
    s.prepare()
    with pytest.raises(eng.RobotCommandError, match="183: the acceleration .* above the controller's limit"):
        s.run(plan.cmds)
    sim.fail_next = {"MoveJ": 112}
    with pytest.raises(eng.RobotCommandError, match="112: the pose cannot be reached"):
        s.run(plan.cmds)


# ------------------------------------------------------------------ job
def run_job(qapp, sim, setup, mode, path=DEMO, expect=None, **kw):
    seen = dict(log=[], rows=[], result=None)
    job = Job(RobotSession("sim", lambda _ip: sim), path, setup.apply(settings(**kw)), mode, expect)
    job.log.connect(seen["log"].append)
    job.command.connect(lambda row, _text: seen["rows"].append(row))
    job.done.connect(lambda result: seen.update(result=result))
    job.start()
    assert wait_until(lambda: seen["result"] is not None, 30)
    job.wait()
    return seen


def test_check_reads_the_robot_and_never_moves_it(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    result = run_job(qapp, sim, RunSetup(placement=HERE), "check")["result"]
    assert result.ok and result.plan.placed and result.plan.frame.origin == [300.0, -200.0, 400.0]
    assert sim.moves() == [] and not any(n in ("Mode", "RobotEnable", "SetSpeed") for n, _k in sim.calls)
    assert result.settings.origin_here is False       # the origin is now fixed


def test_run_sends_the_plan_in_order(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    checked = run_job(qapp, sim, RunSetup(placement=HERE, approach=0), "check")["result"].plan
    seen = run_job(qapp, sim, RunSetup(placement=HERE, approach=0, global_speed=35), "run", expect=checked)
    assert seen["result"].ok and seen["rows"] == list(range(len(checked.cmds)))
    assert ("SetSpeed", {"vel": 35, "while_moving": False}) in sim.calls
    assert off_path(checked, sim.travelled) < 0.3


def test_unreachable_plan_is_reported_and_not_started(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    result = run_job(qapp, sim, RunSetup(placement=MANUAL, origin=[1480.0, 0.0, 200.0]), "run")["result"]
    assert not result.ok and "cannot be reached" in result.message
    assert result.plan.unreachable and sim.moves() == []


@pytest.mark.parametrize("ref, text", [("ref_wobj", "tool 1 / workpiece 2"), ("nope", "could not be read")])
def test_bad_reference_point_is_refused(qapp, ref, text):
    sim = sim_robot.SimRobot(time_scale=50)
    result = run_job(qapp, sim, RunSetup(placement=REF, ref_point=ref), "run")["result"]
    assert not result.ok and text in result.message and sim.moves() == []


def test_run_is_refused_when_the_placement_changed_since_the_check(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    setup = RunSetup(placement=REF, ref_point="ref_tilt", turn_with_path=True)
    checked = run_job(qapp, sim, setup, "check")["result"].plan
    assert checked.follow                                  # tilted tool, asked to turn with the path
    sim.teach_points["ref_tilt"] = ([10.0, 0.0, 50.0, 180.0, 25.0, 0.0], 0, 0)
    result = run_job(qapp, sim, setup, "run", expect=checked)["result"]
    assert not result.ok and "not the one that was checked" in result.message and sim.moves() == []


def test_missing_sdk_is_explained(qapp):
    def no_sdk(_ip):
        raise ImportError("No module named 'fairino'")
    seen = {}
    job = Job(RobotSession("10.0.0.1", no_sdk), DEMO, RunSetup(placement=HERE).apply(settings()), "check")
    job.done.connect(lambda result: seen.update(result=result))
    job.start()
    assert wait_until(lambda: "result" in seen, 5)
    assert not seen["result"].ok and "SDK was not found" in seen["result"].message


def test_setup_dialog_round_trip_and_validation(qapp):
    dialog = RunSetupDialog(RunSetup())
    dialog.accept()                                         # reference point chosen but no name typed
    assert dialog.chosen is None and "name of the reference teach point" in dialog.message.text()
    dialog.by_manual.setChecked(True)
    dialog.origin[0].setValue(250.5)
    dialog.global_speed.setValue(40)
    dialog.accept()
    assert dialog.chosen.placement == MANUAL and dialog.chosen.origin[0] == 250.5
    assert dialog.chosen.global_speed == 40 and dialog.chosen.rapid_speed is None
    a = dialog.chosen.apply(default_settings())
    assert a.origin == [250.5, 0.0, 200.0] and not a.origin_here and a.ref_point is None


# ------------------------------------------------------------------ window
def sim_window(qapp, path=DEMO, time_scale=40, **sim_kw):
    sim = sim_robot.SimRobot(time_scale=time_scale, **sim_kw)
    win = gcode_gui.MainWindow(settings(), simulator=sim)
    win.show()
    win.load(str(path))
    win.attach_simulator()
    return win, sim


def idle(win):
    return win.job is None


def test_buttons_follow_the_workflow(qapp):
    sim = sim_robot.SimRobot(time_scale=40)
    win = gcode_gui.MainWindow(settings(), simulator=sim)
    win.show()
    enabled = lambda: [a.isEnabled() for a in (win.act_setup, win.act_start, win.act_stop)]   # noqa: E731
    assert enabled() == [False, False, False]
    win.load(DEMO)
    assert enabled() == [False, False, False]              # not connected yet
    win.attach_simulator()
    assert enabled() == [True, False, False]               # can set up, cannot start unchecked
    win.apply_setup(RunSetup(placement=HERE))
    assert enabled() == [False, False, False]              # checking
    assert wait_until(lambda: idle(win), 10)
    assert enabled() == [True, True, False] and "Press Start" in win.banner.text()
    win.start_run(confirm=False)
    assert enabled() == [False, False, True] and not win.act_open.isEnabled()
    assert wait_until(lambda: idle(win), 30)
    assert enabled() == [True, True, False] and win.banner.text() == "Run finished."
    win.close()


def test_check_redraws_the_plan_where_the_robot_will_run_it(qapp):
    win, sim = sim_window(qapp)
    assert not win.plan.placed and win.plan.frame.origin == [0.0, 0.0, 0.0]
    win.apply_setup(RunSetup(placement=HERE))
    assert wait_until(lambda: idle(win), 10)
    assert win.plan.placed and win.plan.frame.origin == [300.0, -200.0, 400.0]
    assert win.plan is win.checked and sim.moves() == []
    # "at the current TCP" is remembered as coordinates, so the origin does not wander afterwards
    assert win.setup.placement == MANUAL and win.setup.origin == [300.0, -200.0, 400.0]
    assert wait_until(lambda: win.canvas._tcp is not None, 5)
    assert (abs(win.canvas._tcp - win.canvas._centre) <= win.canvas._half).all()     # robot and plan in view
    win.close()


def test_run_from_the_window_executes_the_plan_and_follows_it(qapp):
    win, sim = sim_window(qapp, time_scale=20)
    win.apply_setup(RunSetup(placement=HERE, approach=0, spindle_do=0))
    assert wait_until(lambda: idle(win), 10)
    win.start_run(confirm=False)
    rows = set()
    assert wait_until(lambda: rows.add(win.table.currentIndex().row()) or idle(win), 40)
    assert off_path(win.checked, sim.travelled) < 0.3
    assert len(rows) > 5                                    # the table followed the run
    assert "Run finished." in win.log.toPlainText() and "MoveC" in win.log.toPlainText()
    assert sim.do[0] == 0
    # the live dot travelled the plan too (status stream -> trail)
    trail = np.array(win.canvas._trail)
    assert len(trail) > 20 and np.min(np.linalg.norm(plan_points(win.checked)[:, None] - trail[None], axis=2)) < 1.0
    win.close()


def test_stop_button_stops_the_robot(qapp):
    win, sim = sim_window(qapp, time_scale=2)
    win.apply_setup(RunSetup(placement=HERE, spindle_do=0))
    assert wait_until(lambda: idle(win), 10)
    win.start_run(confirm=False)
    assert wait_until(lambda: len(sim.moves()) >= 4, 20)
    win.stop_run()
    assert wait_until(lambda: idle(win), 5)
    assert ("StopMotion", {}) in sim.calls and "Stopped by the operator" in win.banner.text()
    time.sleep(0.1)
    here = sim.pose
    time.sleep(0.3)
    assert sim.pose == here
    assert win.act_start.isEnabled() and not win.act_stop.isEnabled()
    win.close()


def test_start_needs_confirmation(qapp, monkeypatch):
    win, sim = sim_window(qapp)
    win.apply_setup(RunSetup(placement=HERE))
    assert wait_until(lambda: idle(win), 10)
    monkeypatch.setattr(win, "_confirm_start", lambda: False)
    win.start_run()
    assert win.job is None and sim.moves() == []
    win.close()


def test_unreachable_targets_are_marked_and_block_start(qapp):
    win, sim = sim_window(qapp)
    win.apply_setup(RunSetup(placement=MANUAL, origin=[1480.0, 0.0, 200.0]))
    assert wait_until(lambda: idle(win), 10)
    assert win.checked is None and not win.act_start.isEnabled()
    assert win.plan.unreachable and "cannot be reached" in win.banner.text()
    win.start_run(confirm=False)
    assert win.job is None and sim.moves() == []
    win.close()


def test_opening_another_file_checks_it_with_the_same_setup(qapp, tmp_path):
    other = tmp_path / "small.gcode"
    other.write_text(SMALL)
    win, sim = sim_window(qapp)
    win.apply_setup(RunSetup(placement=HERE))
    assert wait_until(lambda: idle(win), 10)
    sim.robot_state_pkg.tl_cur_pos = [350.0, 100.0, 300.0, 180.0, 0.0, 0.0]      # the robot is jogged away
    win.load(str(other))
    assert win.checked is None and not win.act_start.isEnabled()                  # not startable meanwhile
    assert wait_until(lambda: idle(win) and win.checked is not None, 10)
    assert win.checked.path == str(other) and win.checked.frame.origin == [300.0, -200.0, 400.0]
    win.close()


def test_program_pause_waits_for_continue(qapp, tmp_path):
    path = tmp_path / "small.gcode"
    path.write_text(SMALL)
    win, sim = sim_window(qapp, path=path)
    win.apply_setup(RunSetup(placement=HERE, approach=0))
    assert wait_until(lambda: idle(win), 10)
    win.start_run(confirm=False)
    assert wait_until(lambda: win.act_continue.isVisible(), 20)
    sent = len(sim.moves())
    time.sleep(0.3)
    assert len(sim.moves()) == sent and "Paused" in win.banner.text()            # really waiting
    win.continue_run()
    assert wait_until(lambda: idle(win), 20)
    assert win.banner.text() == "Run finished." and len(sim.moves()) > sent
    win.close()


def test_losing_the_connection_or_closing_the_window_stops_a_run(qapp):
    for end in ("lost", "close"):
        win, sim = sim_window(qapp, time_scale=2)
        win.apply_setup(RunSetup(placement=HERE))
        assert wait_until(lambda: idle(win), 10)
        win.start_run(confirm=False)
        assert wait_until(lambda: len(sim.moves()) >= 3, 20)
        if end == "lost":
            win._on_lost("network unreachable")
        else:
            win.close()
        assert ("StopMotion", {}) in sim.calls and win.job is None
        time.sleep(0.2)
        assert not sim.busy()
        win.close()


# ------------------------------------------------------------------ reachability along the path
class HollowArm:
    """Reaches 300..800 mm from its base axis - so a straight line can have both ends in reach and its
    middle too close to the base. With flip_at set, joint 4 turns over where X crosses that value."""

    def __init__(self, flip_at=None):
        self.flip_at, self.asked = flip_at, []

    def GetActualJointPosDegree(self, flag=1):
        return 0, [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]

    def GetInverseKinRef(self, _type, pose, _ref):
        self.asked.append(list(pose[:3]))
        r = (pose[0] ** 2 + pose[1] ** 2) ** 0.5
        if not 300.0 <= r <= 800.0:
            return 112, None
        wrist = 170.0 if self.flip_at is not None and pose[0] > self.flip_at else 0.0
        return 0, [pose[0] / 20.0, -90.0, 90.0, wrist, 90.0, 0.0]


def check_with(robot, gcode, tmp_path, **kw):
    path = tmp_path / "part.gcode"
    path.write_text(gcode)
    a = settings(origin=[0.0, 0.0, 100.0], **kw)
    frame = eng.Frame(a.origin, a.rpy, a.yaw, a.scale)
    plan = build_plan(str(path), a, frame)
    s = eng.Streamer(robot, frame, a)
    logged = []
    s.log = logged.append
    return s.check_ik(plan.cmds), s, logged


LONG_LINE = "G21 G90\nG0 X-400 Y250 Z0\nG1 X400 Y250 F600\n"       # ends at r = 472, middle at r = 250


def test_check_finds_an_unreachable_middle_of_a_long_line(tmp_path):
    ok, s, logged = check_with(HollowArm(), LONG_LINE, tmp_path)
    assert not ok
    xs = [pose[0] for _c, pose in s.unreachable]
    assert xs and max(abs(x) for x in xs) < 170.0              # only the part near the base: r < 300 -> |x| < 166
    assert all(c.line == 3 for c, _pose in s.unreachable)
    assert "cannot be reached" in logged[-1]


def test_endpoints_alone_would_have_passed(tmp_path):
    ok, s, _logged = check_with(HollowArm(), LONG_LINE, tmp_path, check_step=0.0)
    assert ok and len(s.robot.asked) == 2                       # the old behaviour: 2 targets, both fine


def test_check_refuses_a_joint_jump_on_a_straight_move(tmp_path):
    line = "G21 G90\nG0 X320 Y0 Z0\nG1 X700 Y0 F600\n"
    ok, s, logged = check_with(HollowArm(flip_at=500.0), line, tmp_path)
    assert not ok and len(s.unreachable) == 1
    assert "would have to turn 170 deg" in logged[0] and "singular" in logged[0]
    ok, _s, _logged = check_with(HollowArm(), line, tmp_path)   # same line without the flip is fine
    assert ok


def test_a_joint_move_may_change_configuration(tmp_path):
    rapid = "G21 G90\nG0 X320 Y0 Z0\nG0 X700 Y0\n"               # G0 = MoveJ: free to turn the wrist over
    ok, _s, _logged = check_with(HollowArm(flip_at=500.0), rapid, tmp_path)
    assert ok


def test_points_along_arcs_and_circles_are_tested_in_path_order(tmp_path):
    arcs = "G21 G90\nG0 X450 Y0 Z0\nG3 X550 Y100 I0 J100 F600\nG1 X650 Y100\nG2 I0 J-100\n"
    robot = HollowArm()
    ok, _s, _logged = check_with(robot, arcs, tmp_path)
    assert ok
    quarter = robot.asked[1:9]                                   # 157 mm of arc -> 7 steps and the real path point
    radii = [((p[0] - 450.0) ** 2 + (p[1] - 100.0) ** 2) ** 0.5 for p in quarter]
    assert all(abs(r - 100.0) < 0.2 for r in radii)              # on the arc, not on its chord
    assert [round(p[0]) for p in quarter] == sorted(round(p[0]) for p in quarter)
    ring = robot.asked[-26:]                                     # the full circle: 628 mm
    steps = [((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5 for p, q in zip(ring, ring[1:])]
    assert max(steps) < 30.0                                     # no jumping back and forth round the ring


def test_short_segments_cost_one_question_each(tmp_path):
    spline = "G21 G90\nG0 X400 Y0 Z0\n" + "".join(f"G1 X{400 + k} Y{k % 3} F600\n" for k in range(1, 61))
    robot = HollowArm()
    ok, _s, _logged = check_with(robot, spline, tmp_path)
    assert ok and len(robot.asked) == 61


# ------------------------------------------------------------------ how the tool is held
TILTED = (300.0, -200.0, 400.0, -175.16, -2.16, 151.84)       # a tool jogged by hand: 5 deg off square


def test_a_slightly_tilted_tool_keeps_its_posture_by_default(qapp):
    """On the real robot a 5 deg tilt made the tool turn with the path, which a closed outline cannot do."""
    sim = sim_robot.SimRobot(time_scale=50, start_pose=TILTED)
    plan = run_job(qapp, sim, RunSetup(placement=HERE), "check")["result"].plan
    assert not plan.follow
    assert {tuple(p[3:]) for k in plan.motions for p in plan.cmds[k].poses} == {TILTED[3:]}


def test_turning_with_the_path_is_still_available(qapp):
    sim = sim_robot.SimRobot(time_scale=50, start_pose=TILTED)
    plan = run_job(qapp, sim, RunSetup(placement=HERE, turn_with_path=True), "check")["result"].plan
    assert plan.follow and len({p[5] for k in plan.motions for p in plan.cmds[k].poses}) > 4


def test_straight_down_replaces_the_posture_and_the_run_uses_it(qapp):
    sim = sim_robot.SimRobot(time_scale=50, start_pose=TILTED)
    seen = run_job(qapp, sim, RunSetup(placement=HERE, tool_down=True, approach=0), "check")
    plan = seen["result"].plan
    assert plan.frame.rpy == [180.0, 0.0, 151.84] and any("straight down" in line for line in seen["log"])
    result = run_job(qapp, sim, RunSetup(placement=HERE, tool_down=True, approach=0), "run", expect=plan)["result"]
    assert result.ok and {tuple(k["desc_pos"][3:]) for n, k in sim.moves() if n == "MoveL"} == {(180.0, 0.0, 151.84)}


def test_straight_down_works_with_a_teach_point_too(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    setup = RunSetup(placement=REF, ref_point="ref_tilt", tool_down=True)
    checked = run_job(qapp, sim, setup, "check")["result"].plan
    assert checked.frame.rpy == [180.0, 0.0, 0.0] and not checked.follow
    assert run_job(qapp, sim, setup, "run", expect=checked)["result"].ok


def test_turning_the_drawing_lays_it_along_the_other_axis(qapp):
    sim = sim_robot.SimRobot(time_scale=50)
    plan = run_job(qapp, sim, RunSetup(placement=HERE, yaw=90.0), "check")["result"].plan
    pts = plan_points(plan)
    span = pts.max(axis=0) - pts.min(axis=0)
    assert span[1] == pytest.approx(100.0, abs=0.01) and span[0] == pytest.approx(60.0, abs=0.01)


def test_dialog_posture_and_rotation_fields(qapp):
    dialog = RunSetupDialog(RunSetup())
    assert dialog.keep_posture.isChecked() and not dialog.tool_down.isChecked()
    dialog.by_here.setChecked(True)
    dialog.turn_posture.setChecked(True)
    dialog.tool_down.setChecked(True)
    dialog.yaw.setValue(90)
    dialog.accept()
    chosen = dialog.chosen
    assert chosen.turn_with_path and chosen.tool_down and chosen.yaw == 90.0
    a = chosen.apply(default_settings())
    assert a.posture == "tangent" and a.tool_down and a.yaw == 90.0
    assert RunSetup().apply(default_settings()).posture == "fixed"
    dialog.by_auto.setChecked(True)                               # automatic + may turn: the angle is its to choose
    assert not dialog.yaw.isEnabled()
    dialog.auto_rotate.setChecked(False)
    assert dialog.yaw.isEnabled()


def test_report_is_one_line_per_gcode_line_and_covers_the_whole_path(tmp_path):
    ok, s, logged = check_with(HollowArm(), "G21 G90\nG0 X-400 Y250 Z0\nG1 X400 Y250 F600\nG1 X400 Y-250\n"
                               "G1 X-400 Y-250\n", tmp_path)
    assert not ok and len(s.unreachable) > 20                     # no longer cut off after 20
    lines = [text for text in logged if text.startswith("  line")]
    assert len(lines) == 2 and "line 3  G1 X400 Y250 F600" in lines[0] and "line 5" in lines[1]
    assert "out of reach in this posture" in lines[0] and " of 32 points from " in lines[0]
    assert not any("would have to turn" in text for text in logged)   # no false joint jump after the gap
