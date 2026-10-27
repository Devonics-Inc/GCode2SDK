"""Fixed TCP speed: the path planned by servo_path.py and streamed with ServoJ (on the simulated robot)."""
import math
import threading
import time

import numpy as np
import pytest
from conftest import DEMO, wait_until

from fairino_gcode import gcode_gui, servo_path as sp, sim_robot
from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.run_control import HERE, RunSetup, RunSetupDialog

SQUARE = "G21 G90\nG0 X0 Y0 Z5\nG1 Z0\nG1 X40 Y0\nG1 X40 Y40\nG1 X0 Y40\nG1 X0 Y0\nG0 Z5\n"


def settings(**kw):
    a = default_settings()
    a.origin, a.approach, a.posture, a.start_wait = [400.0, 0.0, 200.0], 0.0, "fixed", 0.05
    a.speed_mode, a.tcp_speed, a.acc = "servo", 60.0, 2000.0
    for key, value in kw.items():
        setattr(a, key, value)
    return a


def plan_of(tmp_path, text=SQUARE, **kw):
    path = tmp_path / "job.gcode"
    path.write_text(text)
    return build_plan(str(path), settings(**kw))


def tick_speeds(g, cycle=sp.CYCLE_S):
    s = sp.ticks(g, cycle)
    xyz = np.vstack([g.xyz[:1], np.column_stack([np.interp(s, g.s, g.xyz[:, j]) for j in range(3)])])
    return np.linalg.norm(np.diff(xyz, axis=0), axis=1) / cycle


# ------------------------------------------------------------------ the planned path
def test_groups_are_the_runs_of_feed_moves(tmp_path):
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z5\nG1 Z0\nG1 X10\nG4 P0.1\nG1 Y10\nG0 Z5\nG1 X20\nG1 Y20\n")
    groups = [[plan.cmds[k].src for k in range(f, la + 1)] for f, la in sp.feed_groups(plan.cmds)]
    assert groups == [["G1 Z0", "G1 X10"], ["G1 Y10"], ["G1 X20", "G1 Y20"]]


def test_a_square_runs_at_the_fixed_speed_through_its_rounded_corners(tmp_path):
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X40\nG1 Y40\nG1 X0\nG1 Y0\n", tcp_speed=20.0, acc=400.0)
    (f, la), = sp.feed_groups(plan.cmds)
    g = sp.build_group(plan.cmds, f, la, plan.frame, 20.0, 400.0)
    r = 20.0 ** 2 / 400.0                                   # 1 mm: the radius the acceleration allows
    assert g.corners == 3 and g.corner_dev == pytest.approx(r * (math.sqrt(2) - 1), rel=0.01)
    assert g.slow == []                                     # every corner had room for that radius
    v = tick_speeds(g)
    # ticks to speed up / slow down: at most twice v / a (the ramp is planned over whole 1 mm knot steps,
    # so it can be gentler than the acceleration allows), and a margin
    ramp = math.ceil(2 * 20.0 / 400.0 / sp.CYCLE_S) + 3
    assert np.abs(v[ramp:-ramp] - 20.0).max() < 0.2        # exactly the speed, corners included
    assert np.abs(np.diff(v)).max() / sp.CYCLE_S < 1.3 * 400.0   # never faster changes than the acceleration
    assert g.length == pytest.approx(160.0 - 3 * (2 * r - r * math.pi / 2), abs=0.05)


def test_where_a_corner_has_no_room_the_speed_drops_and_the_report_says_so(tmp_path):
    # a 2 mm move between two corners: 45 % of it is too little for the radius 60 mm/s needs
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X40\nG1 Y2\nG1 X0\n", tcp_speed=60.0, acc=2000.0)
    (f, la), = sp.feed_groups(plan.cmds)
    g = sp.build_group(plan.cmds, f, la, plan.frame, 60.0, 2000.0)
    assert g.slow and all(v < 60.0 for _k, v in g.slow)
    lines = "\n".join(sp.describe_groups([g], plan.cmds, 60.0))
    assert "Slower at" in lines and "line 4" in lines
    # sqrt(a * r) with r = 0.45 * 2 mm / tan(45 deg)
    assert min(v for _k, v in g.slow) == pytest.approx(math.sqrt(2000.0 * 0.9), rel=0.05)


def test_arcs_keep_their_shape_and_their_curve_limits_the_speed(tmp_path):
    plan = plan_of(tmp_path, "G21 G90\nG0 X10 Y0 Z0\nG1 X20\nG3 X20 Y10 I0 J5\nG1 X0\n", tcp_speed=80.0, acc=1000.0)
    (f, la), = sp.feed_groups(plan.cmds)
    g = sp.build_group(plan.cmds, f, la, plan.frame, 80.0, 1000.0)
    assert g.corners == 0                                   # line -> arc -> line, all tangent: nothing to round
    arc = g.owner == f + 1
    pts = g.xyz[arc][:, :2] - np.array([420.0, 5.0])
    assert np.abs(np.linalg.norm(pts, axis=1) - 5.0).max() < 0.01
    inside = np.nonzero(arc)[0][1:-1]                      # (its first / last knot is also on the line)
    assert g.v[inside].max() <= math.sqrt(1000.0 * 5.0) * 1.01   # 70.7 mm/s on the R5 arc
    assert g.v[inside].min() >= math.sqrt(1000.0 * 5.0) * 0.99   # ...and not slower than that
    turns = [math.degrees(math.acos(np.clip(np.dot(u, w) / np.linalg.norm(u) / np.linalg.norm(w), -1, 1)))
             for u, w in zip(np.diff(g.xyz, axis=0)[:-1], np.diff(g.xyz, axis=0)[1:])]
    assert max(turns) < 6.0                                 # no kink anywhere


def test_the_summary_tells_what_fixed_speed_will_do(tmp_path):
    plan = plan_of(tmp_path, tcp_speed=30.0, acc=500.0)
    text = "\n".join(plan.summary)
    assert "streamed point by point (ServoJ every 8 ms) at a fixed 30 mm/s" in text
    assert "Fixed speed     : 30 mm/s on" in text and "Corners rounded : 4" in text


# ------------------------------------------------------------------ streaming
def streamer(sim, plan, **kw):
    s = eng.Streamer(sim, plan.frame, plan.settings)
    for key, value in kw.items():
        setattr(s.a, key, value)
    s.stoppable = True
    s.log = lambda *_: None
    s.on_command = lambda *_: None
    return s


def watch(sim):
    """Record (servo active, reported TCP speed) every 5 ms until the returned event is set."""
    seen, stop = [], threading.Event()

    def run():
        while not stop.is_set():
            _pose, v, _moving, _extra = sim.status()
            seen.append((sim.servo, v))
            time.sleep(0.005)
    threading.Thread(target=run, daemon=True).start()
    return seen, stop


def test_the_stream_runs_the_path_at_the_fixed_speed(tmp_path):
    sim = sim_robot.SimRobot()
    plan = plan_of(tmp_path)
    plan = build_plan(plan.path, plan.settings, eng.resolve_frame(plan.settings, sim, log=lambda *_: None))
    s = streamer(sim, plan)
    logged = []
    s.log = logged.append
    seen, stop = watch(sim)
    s.prepare()
    s.run(plan.cmds)
    stop.set()
    names = [n for n, _k in sim.calls]
    first, last = names.index("ServoMoveStart"), names.index("ServoMoveEnd")
    servo = [k for n, k in sim.calls if n == "ServoJ"]
    assert first < names.index("ServoJ") and max(i for i, n in enumerate(names) if n == "ServoJ") < last
    assert all(k["cmdType"] == 1 for k in servo)           # UDP: this "SDK" has cmdType
    assert not any(n in ("MoveL", "MoveC", "Circle") for n in names)   # the feed moves were all streamed
    assert [n for n in names if n == "MoveJ"] == ["MoveJ", "MoveJ"]    # the two G0
    # the arm went along the planned path, corners rounded by no more than the summary says
    g = s.servo_groups[min(s.servo_groups)]
    travelled = np.array(sim.travelled)
    on = [np.min(np.linalg.norm(travelled - p, axis=1)) for p in np.vstack([plan.paths[k] for k in plan.motions])]
    assert max(on) < g.corner_dev + 0.1
    speeds = np.array([v for servo_on, v in seen if servo_on and v > 1.0])
    assert np.median(speeds) == pytest.approx(60.0, abs=0.5)
    assert any("streamed" in text and "UDP" in text for text in logged)


def test_xml_rpc_can_be_chosen(tmp_path):
    sim = sim_robot.SimRobot()
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X20\n", servo_transport="xmlrpc")
    s = streamer(sim, plan)
    s.prepare()
    s.run(plan.cmds)
    assert {k["cmdType"] for n, k in sim.calls if n == "ServoJ"} == {0}


class OldSdkSim(sim_robot.SimRobot):
    """An SDK from before ServoJ had axisPos and cmdType (no UDP)."""

    def ServoJ(self, joint_pos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0):
        return super().ServoJ(joint_pos, [0.0] * 4, cmdT=cmdT)

    def ServoMoveStart(self):
        return super().ServoMoveStart()

    def ServoMoveEnd(self):
        return super().ServoMoveEnd()


def test_an_older_sdk_streams_over_xml_rpc_and_cannot_be_forced_to_udp(tmp_path):
    sim = OldSdkSim()
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X20\n")
    s = streamer(sim, plan)
    s.prepare()
    s.run(plan.cmds)
    assert sum(n == "ServoJ" for n, _k in sim.calls) > 10
    s = streamer(OldSdkSim(), plan, servo_transport="udp")
    s.prepare()
    with pytest.raises(eng.SetupError, match="cannot send ServoJ over UDP"):
        s.run(plan.cmds)


def test_a_controller_that_ignores_servo_commands_is_caught(tmp_path):
    sim = sim_robot.SimRobot()
    sim.ignore_servo = True
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X80\n")
    s = streamer(sim, plan)
    s.prepare()
    with pytest.raises(eng.RobotCommandError, match="does not follow the ServoJ commands"):
        s.run(plan.cmds)
    names = [n for n, _k in sim.calls]
    assert "MotionQueueClear" in names and names[-1] == "ServoMoveEnd"


def test_stop_ends_the_stream_at_once(tmp_path):
    sim = sim_robot.SimRobot()
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X200\n", tcp_speed=40.0)
    s = streamer(sim, plan)
    s.prepare()
    threading.Timer(0.8, s.abort.set).start()
    t0 = time.monotonic()
    with pytest.raises(eng.Aborted):
        s.run(plan.cmds)
    assert time.monotonic() - t0 < 2.5                     # the whole line would take 5 s
    s.emergency_stop()
    names = [n for n, _k in sim.calls]
    assert "ServoMoveEnd" in names and "StopMotion" in names and "MotionQueueClear" in names
    time.sleep(0.05)
    here = sim.pose
    time.sleep(0.2)
    assert sim.pose == here


def test_a_path_the_joints_cannot_follow_is_refused_before_anything_moves(tmp_path):
    sim = sim_robot.SimRobot()
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X40\n", servo_joint_speed=1.0)
    s = streamer(sim, plan)
    s.prepare()
    with pytest.raises(eng.RobotCommandError, match="deg/s"):
        s.run(plan.cmds)
    assert not [n for n, _k in sim.calls if n in ("MoveJ", "MoveL", "ServoJ", "ServoMoveStart")]


def test_the_command_line_refuses_export_in_servo_mode(tmp_path, monkeypatch, capsys):
    path = tmp_path / "job.gcode"
    path.write_text(SQUARE)
    monkeypatch.setattr("sys.argv", ["x", str(path), "--speed-mode", "servo", "--export", str(tmp_path / "j.py")])
    with pytest.raises(SystemExit):
        eng.main()
    assert "--export is not available with --speed-mode servo" in capsys.readouterr().err


# ------------------------------------------------------------------ window
def test_the_dialog_offers_fixed_speed(qapp):
    dialog = RunSetupDialog(RunSetup(placement=HERE))
    dialog.by_servo.setChecked(True)
    assert not dialog.tcp_speed.isHidden() and not dialog.acc.isHidden() and dialog.blend.isHidden()
    assert dialog.speed_label.text() == "G0 (rapid) moves"
    dialog.tcp_speed.setValue(25.0)
    dialog.accept()
    a = dialog.chosen.apply(default_settings())
    assert (a.speed_mode, a.tcp_speed) == ("servo", 25.0) and "streamed with ServoJ" in dialog.chosen.describe()
    assert RunSetupDialog(dialog.chosen).by_servo.isChecked()


def test_a_fixed_speed_run_from_the_window(qapp, monkeypatch):
    sim = sim_robot.SimRobot()
    settings_ = default_settings()
    settings_.start_wait = 0.05
    win = gcode_gui.MainWindow(settings_, simulator=sim)
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=HERE, approach=0, speed_mode="servo", tcp_speed=40.0, acc=1000.0))
    assert wait_until(lambda: win.job is None and win.checked is not None, 30)
    assert "streamed point by point" in win.summary.toPlainText()
    model = win.table_model
    k = next(k for k in win.checked.motions if win.checked.cmds[k].instr == "MoveL")
    assert model.data(model.index(k, model.SPEED_COL)) == "40 mm/s fixed"
    win.start_run(confirm=False)
    assert wait_until(lambda: win.job is None, 90)
    assert "Run finished." in win.banner.text(), win.banner.text()
    assert wait_until(lambda: win.speed.commands, 5)
    assert win.speed.stats().reference == 40.0
    assert "streamed" in win.log.toPlainText()
    shown = []
    monkeypatch.setattr(gcode_gui.QtWidgets.QMessageBox, "information", lambda *a: shown.append(a[2]))
    win.export_python()
    assert shown and "not available for the fixed-speed" in shown[0]
    win.close()
