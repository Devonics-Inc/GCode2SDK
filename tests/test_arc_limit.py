"""Arcs the controller refuses: a MoveC / Circle whose points are under 1.0 mm apart (error 185, measured
on a real FR controller at every radius and sweep) must never be sent - those arcs go out as MoveL."""
import glob
import math
import os

import pytest
from conftest import ROOT

from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode import sim_robot
from fairino_gcode.plan_model import build_plan, default_settings

# A glue path that failed on the robot: N14 and N24 are 2.5 mm fillets that turn only 45 deg, so their
# MoveC points are 0.976 mm apart. The other ten arcs are 1.67 mm or more apart and are fine.
GLUE = """N7 G00 X209.397 Y-47.647
N8 G00 Z100
N9 G01 Z3. F100.
N10 G03 X214.397 Y-42.647 I0 J5.
N11 G01 X214.397 Y-18.183
N12 G03 X209.397 Y-13.183 I-5. J0
N13 G01 X207.742
N14 G02 X205.974 Y-12.45 I0 J2.5
N15 G01 X201.28 Y-7.752
N16 G02 X202.341 Y-5.192 I1.061 J1.06
N17 G01 X209.898
N18 G03 X212.898 Y-2.192 I0 J3.
N19 G01 Y2.192
N20 G03 X209.898 Y5.192 I-3. J0
N21 G01 X202.341
N22 G02 X201.28 Y7.752 I0 J1.5
N23 G01 X205.974 Y12.45
N24 G02 X207.742 Y13.183 I1.769 J-1.767
N25 G01 X209.397
N26 G03 X214.397 Y18.183 I0 J5.
N27 G01 Y42.647
N28 G03 X209.397 Y47.647 I-5. J0
N29 G01 X-211.103
N30 G03 X-216.103 Y42.647 I0 J-5.
N31 G01 Y-42.647
N32 G03 X-211.103 Y-47.647 I5. J0
N33 G01 X209.397
N34 G00 Z100
N39 M30
"""
SMALL_ARC_LINES = {8, 18}           # file line numbers of N14 and N24
ARC_LINES = {4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26}     # N10, N12 ... N32


def glue_plan(tmp_path, **kw):
    path = tmp_path / "glue.gcode"
    path.write_text(GLUE)
    a = default_settings()
    for key, value in kw.items():
        setattr(a, key, value)
    return build_plan(str(path), a)


def instructions_by_line(plan):
    out = {}
    for c in plan.cmds:
        if c.kind in eng.MOTION_KINDS:
            out.setdefault(c.line, []).append(c.instr)
    return out


def test_only_the_arcs_with_points_under_1mm_apart_become_lines(tmp_path):
    plan = glue_plan(tmp_path)
    by_line = instructions_by_line(plan)
    for line in ARC_LINES:
        if line in SMALL_ARC_LINES:
            assert len(by_line[line]) >= 2 and set(by_line[line]) == {"MoveL"}, line
        else:
            assert by_line[line] == ["MoveC"], line
    notes = [w for w in plan.warnings if "sent as" in w]
    assert len(notes) == 2 and "N14" in notes[0] and "N24" in notes[1]


def test_the_replacement_segments_follow_the_arc(tmp_path):
    plan = glue_plan(tmp_path)
    segs = [c for c in plan.cmds if c.line == 8]                  # N14
    cx, cy, r = 207.742, -13.183 + 2.5, 2.5                       # starts at (207.742, -13.183), I0 J2.5
    assert segs[0].start[:2] == pytest.approx((207.742, -13.183))
    assert segs[-1].end[:2] == pytest.approx((205.974, -12.45))
    for c in segs:
        assert math.hypot(c.end[0] - cx, c.end[1] - cy) == pytest.approx(r, abs=1e-3)
        mx, my = (c.start[0] + c.end[0]) / 2, (c.start[1] + c.end[1]) / 2
        assert r - math.hypot(mx - cx, my - cy) <= eng.ARC_TOL + 1e-9         # chord error within tolerance


def test_scale_is_taken_into_account(tmp_path):
    # twice as big on the robot: N14's points are 1.95 mm apart, so it stays a MoveC
    by_line = instructions_by_line(glue_plan(tmp_path, scale=2.0))
    assert all(by_line[line] == ["MoveC"] for line in ARC_LINES)


def test_the_limit_can_be_switched_off(tmp_path):
    by_line = instructions_by_line(glue_plan(tmp_path, min_arc_gap=0.0))
    assert all(by_line[line] == ["MoveC"] for line in ARC_LINES)


def test_a_tiny_full_circle_becomes_a_closed_ring_of_lines(tmp_path):
    path = tmp_path / "ring.gcode"
    path.write_text("G0 X10 Y0 Z0\nG1 X10 Y0\nG2 I-0.5 J0\n")    # r = 0.5: Circle points 0.71 mm apart
    plan = build_plan(str(path), default_settings())
    ring = [c for c in plan.cmds if c.line == 3]
    assert {c.instr for c in ring} == {"MoveL"} and len(ring) >= 4
    assert ring[-1].end == ring[0].start


@pytest.mark.parametrize("sample", sorted(glob.glob(os.path.join(ROOT, "src", "fairino_gcode", "samples",
                                                                  "*.gcode"))), ids=os.path.basename)
def test_no_sample_plans_an_arc_the_controller_refuses(sample):
    try:
        plan = build_plan(sample, default_settings())
    except ValueError:
        pytest.skip("sample is meant to be rejected")
    for c in plan.cmds:
        if c.kind in ("arc", "circle"):
            assert eng.arc_gap(c, plan.frame.scale) >= eng.MIN_ARC_GAP


def run_on_sim(tmp_path, **kw):
    path = tmp_path / "glue.gcode"
    path.write_text(GLUE)
    a = default_settings()
    a.origin, a.approach, a.start_wait = [300.0, 0.0, 200.0], 0.0, 0.05
    for key, value in kw.items():
        setattr(a, key, value)
    sim = sim_robot.SimRobot(time_scale=400)
    frame = eng.resolve_frame(a, sim, log=lambda _t: None)
    plan = build_plan(str(path), a, frame)
    s = eng.Streamer(sim, frame, a)
    s.log = lambda _t: None
    s.on_command = lambda _c, _t: None
    s.prepare()
    s.run(plan.cmds)
    return sim


def test_the_simulator_refuses_the_small_arcs_like_the_robot(tmp_path):
    with pytest.raises(eng.RobotCommandError, match="185"):
        run_on_sim(tmp_path, min_arc_gap=0.0)


def test_the_glue_path_runs_and_every_blend_fits_its_moves(tmp_path):
    sim = run_on_sim(tmp_path)
    moves = sim.moves()
    assert [n for n, _k in moves].count("MoveC") == 10
    # every blend radius is below half of the move it ends and of the one it runs into
    prev = None
    for name, k in moves:
        end = k.get("desc_pos_t", k.get("desc_pos"))
        if prev is not None and name == "MoveL" and k["blendR"] > 0:
            assert k["blendR"] <= 0.45 * math.dist(prev[:3], end[:3]) + 1e-3      # poses are rounded to 1e-4
        prev = end
