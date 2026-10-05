"""Automatic placement: choosing the origin where the robot works best."""
import math

import pytest
from conftest import DEMO, wait_until

from fairino_gcode import auto_place
from fairino_gcode import gcode_gui
from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode import sim_robot
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.run_control import AUTO, MANUAL, Job, RobotSession, RunSetup, RunSetupDialog


class RingArm:
    """A stand-in arm with simple, known behaviour, to test the search itself:
    it reaches points 300..800 mm from its base axis, and its "elbow" (joint 3) goes from folded
    (0 deg) at 300 mm to stretched (180 deg) at 800 mm - so it is most comfortable at 550 mm."""

    def __init__(self, reach=(300.0, 800.0)):
        self.reach, self.asked = reach, 0

    def GetInverseKinRef(self, _type, pose, _ref):
        self.asked += 1
        r = math.hypot(pose[0], pose[1])
        if not self.reach[0] <= r <= self.reach[1]:
            return 112, None
        elbow = 180.0 * (r - self.reach[0]) / (self.reach[1] - self.reach[0])
        return 0, [math.degrees(math.atan2(pose[1], pose[0])), -90.0, elbow, 0.0, 90.0, 0.0]

    def GetJointSoftLimitDeg(self, flag=1):
        return 0, [-175.0, 175.0] * 6


def layout_of(path, yaw=0.0, approach=0.0):
    local = build_plan(str(path), default_settings(), eng.Frame([0.0, 0.0, 0.0], [180.0, 0.0, 0.0], yaw))
    lo, hi = local.bbox
    return auto_place.Layout(yaw, auto_place.sample_targets(local, approach),
                             [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2]), local


def demo_targets(approach=0.0):
    layout, local = layout_of(DEMO, approach=approach)
    return layout.targets, layout.centre, local


def find(arm, targets, centre, start, half_range, **kw):
    return auto_place.find_origin(arm, [auto_place.Layout(0.0, targets, centre)], start, [0.0] * 6, half_range,
                                  log=lambda _t: None, **kw)


def worst_comfort(arm, origin, local):
    """Comfort of the worst of ALL the plan's targets with the drawing at `origin` (None: unreachable)."""
    limits, worst = auto_place.joint_limits(arm), 1.0
    for k in local.motions:
        for p in local.cmds[k].poses:
            err, rest = arm.GetInverseKinRef(0, [p[0] + origin[0], p[1] + origin[1], p[2] + origin[2]] + p[3:], None)
            if err:
                return None
            worst = min(worst, auto_place.comfort_of(auto_place.margins(rest, limits)))
    return worst


def test_samples_span_the_drawing_and_include_the_approach():
    targets, _centre, local = demo_targets(approach=20.0)
    every = [p for k in local.motions for p in local.cmds[k].poses]
    assert len(targets) <= 40 < len(every) * 2
    for axis in range(2):
        assert min(p[axis] for p in targets) == min(p[axis] for p in every)
        assert max(p[axis] for p in targets) == max(p[axis] for p in every)
    assert max(p[2] for p in targets) == every[0][2] + 20.0


def test_finds_the_comfortable_ring_and_beats_placing_at_the_tool():
    arm = RingArm()
    targets, centre, local = demo_targets()
    start = [700.0, 250.0, 120.0]                               # tool near the edge of the reach (r = 743)
    found = find(arm, targets, centre, start, 500.0)
    middle = [found.origin[0] + centre[0], found.origin[1] + centre[1]]
    # elbow 30..150 deg (fully comfortable) is 383..717 mm; the drawing reaches ~60 mm from its middle
    assert 445.0 <= math.hypot(*middle) <= 655.0 and found.comfort == 1.0
    # ...and of the fully comfortable places it took one near the tool, not one across the workspace
    assert math.dist(middle, start[:2]) < 200.0
    assert found.origin[2] == 120.0                              # the height is the tool's, not searched
    assert found.possible > 0 and found.tried > found.possible   # some positions were out of reach
    here = worst_comfort(arm, start, local)                      # "G-code 0,0,0 at the TCP" for comparison
    best = worst_comfort(arm, found.origin, local)
    assert best is not None and (here is None or best > here + 0.2)
    assert best == pytest.approx(found.comfort, abs=0.05)        # the sampled points did not flatter it


def test_stays_inside_the_search_range():
    arm = RingArm()
    targets, centre, _local = demo_targets()
    start = [760.0, 0.0, 50.0]
    found = find(arm, targets, centre, start, 100.0)
    middle = [found.origin[0] + centre[0], found.origin[1] + centre[1]]
    assert max(abs(middle[0] - start[0]), abs(middle[1] - start[1])) <= 100.0 + 1e-6


def test_reports_when_nothing_in_range_works():
    arm = RingArm()
    targets, centre, _local = demo_targets()
    with pytest.raises(eng.SetupError, match="no position where even the outermost points"):
        find(arm, targets, centre, [2000.0, 0.0, 0.0], 300.0)


def test_can_be_stopped():
    arm = RingArm()
    targets, centre, _local = demo_targets()
    with pytest.raises(eng.Aborted):
        find(arm, targets, centre, [550.0, 0.0, 0.0], 500.0, should_stop=lambda: arm.asked > 50)
    assert arm.asked < 80


def test_a_solution_beyond_the_joint_limits_does_not_count():
    arm = RingArm()
    arm.GetJointSoftLimitDeg = lambda flag=1: (0, [-175.0, 175.0] * 2 + [0.0, 100.0] + [-175.0, 175.0] * 3)
    targets, centre, _local = demo_targets()
    found = find(arm, targets, centre, [550.0, 0.0, 0.0], 400.0)
    r = math.hypot(found.origin[0] + centre[0], found.origin[1] + centre[1])
    assert r < 300.0 + 500.0 * 100.0 / 180.0                     # elbow limit 100 deg = 578 mm


@pytest.mark.parametrize("values, expect", [
    ([-175, 175, -265, 85, -160, 160, -265, 85, -175, 175, -175, 175], (-265.0, 85.0)),     # low/high pairs
    ([-175, -265, -160, -265, -175, -175, 175, 85, 160, 85, 175, 175], (-265.0, 85.0)),     # six lows, six highs
])
def test_joint_limits_are_read_in_either_layout(values, expect):
    class Robot:
        def GetJointSoftLimitDeg(self, flag=1):
            return 0, values
    assert auto_place.joint_limits(Robot())[1] == expect


def test_joint_limits_fall_back_when_the_call_is_missing():
    assert auto_place.joint_limits(object()) == auto_place.DEFAULT_LIMITS


# ------------------------------------------------------------------ through the job and the window
def test_job_places_automatically_without_moving(qapp):
    sim = sim_robot.SimRobot(time_scale=50, start_pose=(1300.0, 0.0, 300.0, 180.0, 0.0, 0.0), reach=1400.0)
    seen = dict(log=[])
    a = RunSetup(placement=AUTO, auto_range=400.0, approach=0).apply(default_settings())
    job = Job(RobotSession("sim", lambda _ip: sim), DEMO, a, "check")
    job.log.connect(seen["log"].append)
    job.done.connect(lambda result: seen.update(result=result))
    job.start()
    assert wait_until(lambda: "result" in seen, 30)
    result = seen["result"]
    # placing at the tool (X 1300..1400) would stick out of the 1400 mm reach; automatic placement fits it in
    assert result.ok, result.message
    assert result.plan.placed and not result.plan.unreachable and sim.moves() == []
    assert result.plan.frame.origin[2] == 300.0 and result.plan.frame.rpy == [180.0, 0.0, 0.0]
    assert result.settings.auto_place is False                   # fixed now: the run will not search again
    assert any("Automatic placement: origin" in line for line in seen["log"])


def test_window_keeps_the_found_origin_as_coordinates(qapp):
    sim = sim_robot.SimRobot(time_scale=40)
    settings = gcode_gui.default_settings()
    settings.start_wait = 0.05
    win = gcode_gui.MainWindow(settings, simulator=sim)
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=AUTO, approach=0))
    assert wait_until(lambda: win.job is None, 30)
    assert win.checked is not None and win.act_start.isEnabled()
    assert win.setup.placement == MANUAL and win.setup.origin == win.checked.frame.origin
    origin = list(win.setup.origin)
    win.start_run(confirm=False)                                 # ...and the run uses exactly that placement
    assert wait_until(lambda: win.job is None, 40)
    assert win.banner.text() == "Run finished." and win.checked.frame.origin == origin
    win.close()


def test_dialog_offers_automatic_placement(qapp):
    dialog = RunSetupDialog(RunSetup())
    assert not dialog.auto_range.isEnabled()
    dialog.by_auto.setChecked(True)
    dialog.auto_range.setValue(300)
    assert dialog.auto_range.isEnabled() and not dialog.ref_point.isEnabled()
    dialog.accept()
    assert dialog.chosen.placement == AUTO and dialog.chosen.auto_range == 300.0
    assert dialog.chosen.apply(default_settings()).auto_place is True


def test_samples_include_the_middle_of_long_lines(tmp_path):
    path = tmp_path / "long.gcode"
    path.write_text("G21 G90\nG0 X0 Y0 Z5\nG1 Z0 F300\nG1 X400 Y0\nG1 X400 Y50\nG1 X0 Y50\nG1 X0 Y0\n")
    local = build_plan(str(path), default_settings(), eng.Frame([0.0, 0.0, 0.0], [180.0, 0.0, 0.0]))
    targets = auto_place.sample_targets(local)
    assert [200.0, 0.0, 0.0] in [p[:3] for p in targets] and [200.0, 50.0, 0.0] in [p[:3] for p in targets]


def test_long_part_is_not_placed_across_the_base(tmp_path):
    """A 400 mm line whose ends are in reach but whose middle would pass too close to the base axis."""
    path = tmp_path / "long.gcode"
    path.write_text("G21 G90\nG0 X0 Y0 Z0\nG1 X400 Y0 F300\n")
    local = build_plan(str(path), default_settings(), eng.Frame([0.0, 0.0, 0.0], [180.0, 0.0, 0.0]))
    arm = RingArm()
    start = [0.0, 310.0, 0.0]                                    # drawing centred here: ends r = 369, middle r = 310...
    found = find(arm, auto_place.sample_targets(local), [200.0, 0.0], start, 300.0)
    for x in range(0, 401, 10):                                  # ...every point of the line must be in reach
        r = math.hypot(found.origin[0] + x, found.origin[1])
        assert 300.0 <= r <= 800.0


# ------------------------------------------------------------------ turning the drawing, and the full check
LONG = "G21 G90\nG0 X0 Y0 Z5\nG1 Z0 F300\nG1 X430 Y0\nG1 X430 Y95\nG1 X0 Y95\nG1 X0 Y0\nG0 Z5\n"


def test_a_long_part_pointing_at_the_robot_is_turned_sideways(tmp_path):
    """What happened on the real robot: tool in front of the base, a 430 mm part laid out towards the base.
    Nothing near the tool works along X; turned by 90 deg the same part fits right where the tool is."""
    path = tmp_path / "long.gcode"
    path.write_text(LONG)
    arm, start = RingArm(), [-450.0, 0.0, 100.0]
    along_x, _local = layout_of(path, 0.0)
    with pytest.raises(eng.SetupError):                          # not turned, searching close by: no place
        auto_place.find_origin(arm, [along_x], start, [0.0] * 6, 100.0, log=lambda _t: None)
    layouts = [layout_of(path, yaw)[0] for yaw in (0.0, 90.0, 180.0, 270.0)]
    said = []
    found = auto_place.find_origin(arm, layouts, start, [0.0] * 6, 100.0, log=said.append)
    assert found.yaw in (90.0, 270.0) and "drawing turned by" in said[-2]
    frame = eng.Frame(found.origin, [180.0, 0.0, 0.0], found.yaw)
    for x, y in [(0, 0), (215, 0), (430, 0), (430, 95), (215, 95), (0, 95)]:
        px, py = frame.pose((x, y, 0.0))[:2]
        assert 300.0 <= math.hypot(px, py) <= 800.0


def test_an_unturned_drawing_is_preferred_when_it_fits_as_well():
    arm = RingArm()
    layouts = [layout_of(DEMO, yaw)[0] for yaw in (0.0, 90.0, 180.0, 270.0)]
    found = auto_place.find_origin(arm, layouts, [550.0, 0.0, 0.0], [0.0] * 6, 300.0, log=lambda _t: None)
    assert found.yaw == 0.0


def test_candidates_are_tried_in_turn_until_one_passes_the_full_check():
    arm = RingArm()
    targets, centre, _local = demo_targets()
    asked = []

    def only_far_left(origin, yaw):                              # a full check that is stricter than the samples
        asked.append(list(origin))
        return origin[1] + centre[1] > 150.0

    found = find(arm, targets, centre, [550.0, 0.0, 0.0], 400.0, verify=only_far_left)
    assert len(asked) > 1 and found.origin[1] + centre[1] > 150.0
    with pytest.raises(eng.SetupError, match="none passes the full check"):
        find(arm, targets, centre, [550.0, 0.0, 0.0], 400.0, verify=lambda origin, yaw: False)


def test_job_turns_a_long_part_and_the_window_keeps_the_rotation(qapp, tmp_path):
    path = tmp_path / "long.gcode"
    path.write_text(LONG)
    # the simulated arm reaches 600 mm; the tool is 480 mm out along X: 430 mm further along X cannot fit
    sim = sim_robot.SimRobot(time_scale=40, start_pose=(480.0, 0.0, 200.0, 180.0, 0.0, 0.0), reach=600.0)
    settings = gcode_gui.default_settings()
    settings.start_wait = 0.05
    win = gcode_gui.MainWindow(settings, simulator=sim)
    win.show()
    win.load(str(path))
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=AUTO, auto_range=100.0, approach=0))
    assert wait_until(lambda: win.job is None, 60)
    assert win.checked is not None, win.banner.text()
    assert win.checked.settings.yaw in (90.0, 270.0) and win.setup.yaw == win.checked.settings.yaw
    assert win.setup.placement == MANUAL and not win.checked.unreachable
    win.start_run(confirm=False)                                 # the run is placed exactly as checked
    assert wait_until(lambda: win.job is None, 60)
    assert win.banner.text() == "Run finished."
    win.close()
