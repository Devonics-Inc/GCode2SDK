"""Constant TCP speed (mm/s mode) and the speed of every command, measured after a run."""
import contextlib
import io
import math
import runpy
import sys
import types

import numpy as np
import pytest
from conftest import DEMO, wait_until

from fairino_gcode import gcode_gui, sim_robot
from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode.export_script import export_plan
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.run_control import HERE, RunSetup

FEED_MOVES = ("MoveL", "MoveC", "Circle")


def settings(**kw):
    a = default_settings()
    a.origin, a.approach, a.posture, a.start_wait = [400.0, 0.0, 200.0], 0.0, "fixed", 0.05
    for key, value in kw.items():
        setattr(a, key, value)
    return a


def run_app(a, time_scale=200):
    sim = sim_robot.SimRobot(time_scale=time_scale)
    frame = eng.resolve_frame(a, sim, log=lambda *_: None)
    plan = build_plan(DEMO, a, frame)
    s = eng.Streamer(sim, frame, a)
    s.stoppable, s.log, s.pause, s.on_command = True, (lambda *_: None), (lambda *_: None), (lambda *_: None)
    s.prepare()
    s.run(plan.cmds)
    return plan, sim


# ------------------------------------------------------------------ engine
def test_mm_s_mode_sends_the_tcp_speed_and_acceleration_with_every_feed_move():
    plan, sim = run_app(settings(speed_mode="physical", tcp_speed=35.0, acc=300.0, global_speed=30))
    feed = [k for n, k in sim.moves() if n in FEED_MOVES]
    assert feed and all((k["ovl"], k["oacc"], k["velAccParamMode"]) == (35.0, 300.0, 1) for k in feed)
    assert all(k["vel"] == 100.0 for k in feed)                     # vel stays 100: ovl carries the mm/s
    assert [k["vel"] for n, k in sim.moves() if n == "MoveJ"] and sim.global_speed == 30    # MoveJ: the %
    assert all(eng.describe(plan.cmds[k], plan.settings).endswith("@ 35 mm/s")
               for k in plan.motions if plan.cmds[k].instr != "MoveJ")


def test_percent_mode_is_unchanged():
    _plan, sim = run_app(settings())
    feed = [k for n, k in sim.moves() if n in FEED_MOVES]
    assert all((k["vel"], k["ovl"], k["oacc"], k["velAccParamMode"]) == (100.0, 100.0, 100.0, 0) for k in feed)


def test_the_tcp_speed_does_not_depend_on_the_global_speed():
    """mm/s mode: the controller's global % scales only the MoveJ (as the simulator models it)."""
    seen = {}
    for pct in (10, 80):
        _plan, sim = run_app(settings(speed_mode="physical", tcp_speed=40.0, global_speed=pct))
        seen[pct] = {k["ovl"] for n, k in sim.moves() if n in FEED_MOVES}
    assert seen[10] == seen[80] == {40.0}


def test_summary_and_time_estimate_use_the_tcp_speed():
    a = settings(speed_mode="physical", tcp_speed=10.0)
    plan = build_plan(DEMO, a)
    assert any("at 10 mm/s" in line for line in plan.summary)
    slow = float(next(s for s in plan.summary if s.startswith("Path length")).split("est. ")[1].split()[0])
    fast = build_plan(DEMO, settings(speed_mode="physical", tcp_speed=40.0)).summary
    fast = float(next(s for s in fast if s.startswith("Path length")).split("est. ")[1].split()[0])
    assert slow > fast


# ------------------------------------------------------------------ exported scripts
def run_exported(path):
    sim = sim_robot.SimRobot(time_scale=200)

    class RPC:
        is_connect = True

        def __new__(cls, ip):
            return sim

    fake = types.ModuleType("fairino")
    fake.Robot = types.SimpleNamespace(RPC=RPC)
    saved = sys.modules.get("fairino"), sys.argv, sys.stdin
    sys.modules["fairino"] = fake
    sys.argv, sys.stdin = [path, "-y"], io.StringIO("yes\n" + "\n" * 50)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                runpy.run_path(path, run_name="__main__")
            except SystemExit:
                pass
    finally:
        if saved[0] is None:
            sys.modules.pop("fairino", None)
        else:
            sys.modules["fairino"] = saved[0]
        sys.argv, sys.stdin = saved[1], saved[2]
    return sim


@pytest.mark.parametrize("style", ["checked", "simple"])
def test_exported_scripts_send_the_same_speeds_as_the_app(tmp_path, style):
    a = settings(speed_mode="physical", tcp_speed=25.0, acc=250.0, global_speed=50)
    plan, sim_app = run_app(a)
    path = str(tmp_path / f"job_{style}.py")
    export_plan(plan, path, style=style)
    text = open(path).read()
    assert "TCP_SPEED = 25" in text and "ACC = 250" in text
    sim = run_exported(path)

    def speeds(s):
        return [(n, k.get("ovl"), k.get("oacc"), k.get("velAccParamMode")) for n, k in s.moves() if n in FEED_MOVES]
    assert speeds(sim) and speeds(sim) == speeds(sim_app)
    assert sim.global_speed == 50


def test_exported_percent_script_has_no_mm_s_settings(tmp_path):
    plan, _sim = run_app(settings())
    path = str(tmp_path / "job.py")
    export_plan(plan, path, style="simple")
    assert "velAccParamMode" not in open(path).read()


# ------------------------------------------------------------------ speed of every command
from fairino_gcode.speed_monitor import FEED, RAPID, SpeedRecorder, command_speeds  # noqa: E402


def plan_of(tmp_path, text, **kw):
    path = tmp_path / "job.gcode"
    path.write_text(text)
    return build_plan(str(path), settings(**kw))


def walk(rec, plan, speeds, dt=0.05, t=0.0, off=(0.0, 0.0, 0.0)):
    """Move a fake TCP along the plan's feed commands, command k at speeds[k] mm/s, `off` mm away from the
    planned path (as a real robot's measured TCP is); returns the end time."""
    for k in plan.motions:
        c = plan.cmds[k]
        if c.instr == "MoveJ" or k not in speeds:
            continue
        pts = np.asarray(plan.paths[k], dtype=float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s, total = 0.0, seg.sum()
        while s < total:
            s = min(total, s + speeds[k] * dt)
            i = min(int(np.searchsorted(np.cumsum(seg), s)), len(seg) - 1)
            before = np.cumsum(seg)[i] - seg[i]
            f = (s - before) / seg[i] if seg[i] else 0.0
            t += dt
            rec.add(t, pts[i] + (pts[i + 1] - pts[i]) * f + np.asarray(off), speeds[k])
    return t


SQUARE = "G21 G90\nG0 X0 Y0 Z0\nG1 X60 Y0\nG1 X60 Y40\nG1 X0 Y40\nG1 X0 Y0\n"


def test_every_command_gets_its_own_speed(tmp_path):
    plan = plan_of(tmp_path, SQUARE)
    feed = [k for k in plan.motions if plan.cmds[k].instr == "MoveL"]
    wanted = dict(zip(feed, [10.0, 30.0, 20.0, 40.0]))
    rec = SpeedRecorder()
    walk(rec, plan, wanted)
    found, owner = command_speeds(rec, plan, reference=20.0, tolerance=10.0)
    assert set(found) == set(feed)
    for k, v in wanted.items():
        s = found[k]
        assert s.mean == pytest.approx(v, abs=v * 0.15)       # corner samples can fall to a neighbour
        assert s.length == pytest.approx(60.0 if k in feed[::2] else 40.0)
        assert s.t1 > s.t0
    assert found[feed[2]].on_speed > 80 and found[feed[0]].on_speed < 20
    assert len(owner) == len(rec) and owner.count(None) <= 2


def test_a_path_crossing_itself_does_not_pull_the_samples_forward(tmp_path):
    # the last line crosses the first one at X50. The measured TCP runs 0.1 mm beside the plan, so at X50
    # the sample is nearer to the last line (0 mm) than to the first (0.1 mm): it must stay with the first
    plan = plan_of(tmp_path, "G21 G90\nG0 X0 Y0 Z0\nG1 X100 Y0\nG1 X100 Y20\nG1 X50 Y20\nG1 X50 Y-20\n")
    feed = [k for k in plan.motions if plan.cmds[k].instr == "MoveL"]
    rec = SpeedRecorder()
    walk(rec, plan, {k: 25.0 for k in feed}, off=(0.0, 0.1, 0.0))
    found, owner = command_speeds(rec, plan)
    assert [found[k].samples for k in feed] == pytest.approx([80, 16, 40, 32], abs=2)


def test_rapids_and_standing_still_belong_to_no_command(tmp_path):
    plan = plan_of(tmp_path, SQUARE)
    feed = [k for k in plan.motions if plan.cmds[k].instr == "MoveL"]
    rec = SpeedRecorder()
    rec.mark(0.0, RAPID)
    rec.add(0.0, (500.0, 0.0, 0.0), 300.0)                   # somewhere during the G0
    rec.add(0.05, (450.0, 0.0, 0.0), 300.0)
    rec.mark(0.07, FEED)
    t = walk(rec, plan, {feed[0]: 20.0}, t=0.1)
    for k in range(10):                                      # at rest at the end of the first line
        rec.add(t + 0.05 * (k + 1), tuple(plan.paths[feed[0]][-1]), 0.4)
    found, owner = command_speeds(rec, plan)
    assert list(found) == [feed[0]]
    assert owner[:2] == [None, None] and owner[-10:] == [None] * 10


def test_a_run_in_mm_s_measures_every_command_and_exports_it(qapp, tmp_path):
    sim = sim_robot.SimRobot(time_scale=2)
    win = gcode_gui.MainWindow(settings(), simulator=sim)
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=HERE, approach=0, speed_mode="physical", tcp_speed=30.0, global_speed=40))
    assert wait_until(lambda: win.job is None and win.checked is not None, 20)
    model = win.table_model
    k = next(k for k in win.checked.motions if win.checked.cmds[k].instr == "MoveL")
    assert model.data(model.index(k, model.SPEED_COL)) == "30 mm/s"
    win.start_run(confirm=False)
    assert wait_until(lambda: win.job is None, 90)
    assert wait_until(lambda: win.speed.commands, 5)
    assert win.speed.stats().source == "commanded" and win.speed.stats().reference == 30.0
    feed = [k for k in win.checked.motions if win.checked.cmds[k].instr != "MoveJ"]
    assert len(win.speed.commands) >= 0.8 * len(feed)
    # the simulator holds the commanded speed exactly
    assert all(math.isclose(s.mean, 30.0, abs_tol=0.5) for s in win.speed.commands.values())
    assert model.move_text(k) == "30.0 / 30.0 / 30.0"
    # selecting a row shades its stretch of the graph
    win.table.selectRow(k)
    assert win.speed._span is not None
    written = win.speed.export(str(tmp_path / "run.csv"))
    rows = open(written[0]).read().splitlines()
    assert rows[0].startswith("#,G-code line") and len(rows) == len(win.checked.motions) + 1
    samples = open(written[1]).read().splitlines()
    assert len(samples) == len(win.speed.recorder) + 1 and any(",feed," in r for r in samples[1:])
    win.close()
