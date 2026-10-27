"""The speed panel (speed_monitor.py / speed_view.py), the X/Y/Z-only panel and a window that fits the screen."""
import math
import types

import numpy as np
import pytest
from conftest import DEMO, wait_until

from fairino_gcode import gcode_gui, sim_robot
from fairino_gcode import robot_monitor as rm
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.robot_widgets import RobotStatusPanel
from fairino_gcode.run_control import HERE, RunSetup
from fairino_gcode.speed_monitor import FEED, RAPID, RECEIVED, SpeedRecorder, plan_targets


def straight(rec, speeds, t0=0.0, dt=0.05, start=0.0):
    """Feed `speeds` (mm/s) to rec along X, one sample per dt; returns the end time and X."""
    t, x = t0, start
    for v in speeds:
        rec.add(t, (x, 0.0, 0.0), v)
        x += v * dt
        t += dt
    return t, x


# ------------------------------------------------------------------ numbers
def test_steady_speed_is_all_inside_the_band():
    rec = SpeedRecorder()
    straight(rec, [50.0] * 100)
    s = rec.stats(tolerance=10)
    assert s.reference == pytest.approx(50.0) and s.auto
    assert s.mean == pytest.approx(50.0) and s.std == pytest.approx(0.0) and s.cv == pytest.approx(0.0)
    assert s.within == pytest.approx(100.0) and s.excursions == 0


def test_a_dip_is_measured_against_the_reference():
    rec = SpeedRecorder()
    straight(rec, [50.0] * 40 + [30.0] * 10 + [50.0] * 50)
    s = rec.stats(tolerance=10)
    assert s.reference == pytest.approx(50.0)                 # the median ignores the dip
    assert s.within == pytest.approx(90.0, abs=1.0)
    assert s.excursions == 1 and s.low == pytest.approx(30.0)
    custom = rec.stats(custom_reference=40.0, tolerance=10)    # a typed-in reference is used as it is
    assert custom.reference == 40.0 and not custom.auto and custom.within < 15


def test_standing_still_is_not_counted():
    rec = SpeedRecorder()
    straight(rec, [0.0] * 50 + [50.0] * 50 + [0.2] * 50)
    s = rec.stats()
    assert s.moving_s == pytest.approx(50 * 0.05, abs=0.06) and s.low == pytest.approx(50.0)


def test_rapids_are_drawn_but_not_counted():
    rec = SpeedRecorder()
    rec.mark(0.0, FEED)
    t, x = straight(rec, [50.0] * 50)
    rec.mark(t, RAPID)
    t, x = straight(rec, [400.0] * 10, t0=t, start=x)
    rec.mark(t, FEED)
    straight(rec, [50.0] * 50, t0=t, start=x)
    assert rec.phases().count(RAPID) == 10
    s = rec.stats()
    assert s.high == pytest.approx(50.0) and s.within == pytest.approx(100.0)


def test_speed_from_positions_when_the_robot_does_not_send_it():
    rec = SpeedRecorder()
    for k in range(40):                                        # 40 mm/s along Y, no speed item
        assert rec.add_sample({rm.TCP: [0.0, 2.0 * k, 0.0, 180, 0, 0], RECEIVED: 10.0 + 0.05 * k})
    assert list(rec.v)[-1] == pytest.approx(40.0)
    assert not rec.add_sample({"robot_mode": 0})               # no position: ignored


def test_the_speed_item_is_used_when_it_is_there():
    rec = SpeedRecorder()
    rec.add_sample({rm.TCP: [0, 0, 0, 0, 0, 0], "actual_TCP_cmpvel": [12.5, 3.0], RECEIVED: 1.0})
    assert list(rec.v) == [12.5]


# ------------------------------------------------------------------ at the targets
def test_the_dip_through_a_blended_corner_is_found_and_a_stop_is_marked():
    rec = SpeedRecorder()
    t = 0.0
    path = [((0.0, 0.0), (100.0, 0.0)), ((100.0, 0.0), (100.0, 100.0))]
    for (a, b) in path:                                        # 50 mm/s, dipping to 25 within 5 mm of the corner
        length = math.dist(a, b)
        d = 0.0
        while d < length:
            to_end = length - d
            v = 25.0 if to_end < 5.0 or (d < 5.0 and a != (0.0, 0.0)) else 50.0
            if b == (100.0, 100.0) and to_end < 5.0:
                v = 2.0                                        # slowing down for the final stop
            p = (a[0] + (b[0] - a[0]) * d / length, a[1] + (b[1] - a[1]) * d / length, 0.0)
            rec.add(t, p, v)
            d += v * 0.05
            t += 0.05
    rec.add(t, (100.0, 100.0, 0.0), 0.0)
    found = rec.at_targets([(0, (100.0, 0.0, 0.0), False), (1, (100.0, 100.0, 0.0), True),
                            (2, (500.0, 500.0, 0.0), False)], reference=50.0)
    assert found[0].speed == pytest.approx(25.0) and found[0].deviation == pytest.approx(-50.0)
    assert found[1].stop and found[1].deviation is None
    assert 2 not in found                                      # never went there


def test_a_short_move_does_not_take_its_neighbours_stop():
    rec = SpeedRecorder()
    # steady 50 mm/s through (0, 8), then stopping 11 mm further on at (8, 0)
    pts = [(0.0, 30.0 - 2.5 * k) for k in range(9)] + [(1.0 * k, 8.0 - 1.0 * k) for k in range(1, 9)]
    speeds = [50.0] * 9 + [50.0, 50, 50, 50, 40, 20, 5, 0.0]     # slows down in the stop's half
    for k, (p, v) in enumerate(zip(pts, speeds)):
        rec.add(0.05 * k, (*p, 0.0), v)
    found = rec.at_targets([(0, (0.0, 8.0, 0.0), False), (1, (8.0, 0.0, 0.0), True)], reference=50.0,
                           radius=7.5)
    assert found[0].speed == pytest.approx(50.0)               # not the 0 / 20 mm/s of the stop 11 mm away
    assert found[1].speed == 0.0 and found[1].stop


def test_targets_of_a_plan_know_which_moves_end_exactly():
    plan = build_plan(DEMO, default_settings())
    targets = plan_targets(plan)
    assert [k for k, _p, _s in targets] == plan.motions
    stops = {k for k, _p, stop in targets if stop}
    for k in plan.motions:
        c, nxt = plan.cmds[k], plan.cmds[k + 1] if k + 1 < len(plan.cmds) else None
        blended = c.instr != "MoveJ" and nxt is not None and nxt.kind in ("line", "arc", "circle") \
            and nxt.instr != "MoveJ"
        assert (k in stops) == (not blended)


# ------------------------------------------------------------------ window
def test_panel_shows_only_x_y_z(qapp):
    panel = RobotStatusPanel()
    panel.show_sample({rm.TCP: [1.0, 2.0, 3.0, 180.0, 0.0, 90.0]})
    assert [v.text() for v in panel.pose] == ["1.000", "2.000", "3.000"]
    assert "RX" not in "".join(w.text() for w in panel.findChildren(type(panel.link)))


def test_window_fits_the_screen_and_remembers_its_size(qapp):
    from matplotlib.backends.qt_compat import QtCore
    from fairino_gcode.robot_widgets import _settings
    _settings().remove("window/geometry")
    avail = qapp.primaryScreen().availableGeometry()
    win = gcode_gui.MainWindow(default_settings())
    win.show()
    assert win.isMaximized() or (win.width() <= avail.width() and win.height() <= avail.height())
    assert win.minimumSizeHint().width() <= 800 and win.minimumSizeHint().height() <= 600
    win.showNormal()
    win.resize(700, 500)
    win.close()
    again = gcode_gui.MainWindow(default_settings())
    assert (again.width(), again.height()) == (700, 500)
    again.close()
    # saved on a bigger screen: shrunk to this one
    big = gcode_gui.MainWindow(default_settings())
    big.resize(avail.width() + 900, avail.height() + 600)
    _settings().setValue("window/geometry", big.saveGeometry())
    big.deleteLater()
    small = gcode_gui.MainWindow(default_settings())
    assert small.width() <= avail.width() and small.height() <= avail.height()
    small.close()
    _settings().remove("window/geometry")
    assert QtCore is not None


def test_a_tall_window_puts_the_view_above_the_panels(qapp):
    win = gcode_gui.MainWindow(default_settings())
    win.show()
    win.showNormal()
    win.resize(1200, 700)
    qapp.processEvents()
    assert win.split.orientation() == gcode_gui.Qt.Orientation.Horizontal
    sizes = win.split.sizes()
    assert sizes[0] > sizes[1] > 0.3 * sum(sizes)              # about 3 : 2, not squeezed to a minimum
    win.resize(600, 900)
    qapp.processEvents()
    assert win.split.orientation() == gcode_gui.Qt.Orientation.Vertical
    win.close()


def test_the_speed_panel_sits_under_the_3d_view_and_has_nothing_to_set(qapp):
    win = gcode_gui.MainWindow(default_settings())
    win.show()
    win.showNormal()
    win.resize(1400, 800)
    qapp.processEvents()
    assert win.left_split.widget(0) is win.canvas and win.left_split.widget(1) is win.speed
    assert win.tabs.indexOf(win.speed) == -1
    canvas_h, speed_h = win.left_split.sizes()
    assert canvas_h > speed_h > 0.25 * (canvas_h + speed_h)       # about 3 : 2
    # automatic reference, a fixed +/- 15 % band: no spin boxes, only Clear
    assert win.speed.findChildren(gcode_gui.QtWidgets.QAbstractSpinBox) == []
    assert [b.text() for b in win.speed.findChildren(gcode_gui.QtWidgets.QPushButton)] == ["Clear"]
    assert win.speed.band == 15.0 and win.speed.stats().tolerance == 15.0
    win.close()


def test_a_corner_passed_during_a_rapid_is_judged_on_the_feed_samples_only():
    rec = SpeedRecorder()
    rec.mark(0.0, FEED)
    t, x = straight(rec, [50.0] * 40)                      # feed up to X 100 ...
    rec.mark(t, RAPID)
    straight(rec, [400.0] * 3, t0=t, start=x - 20.0)       # ...then a fast rapid passes the same point
    found = rec.at_targets([(0, (x, 0.0, 0.0), False)], 50.0)
    assert found[0].speed == pytest.approx(50.0)


def test_a_run_fills_the_speed_panel_and_the_target_column(qapp):
    settings = default_settings()
    settings.start_wait = 0.05
    sim = sim_robot.SimRobot(time_scale=4)
    win = gcode_gui.MainWindow(settings, simulator=sim)
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=HERE, approach=0))
    assert wait_until(lambda: win.job is None and win.checked is not None, 20)
    win.start_run(confirm=False)
    assert wait_until(lambda: win.job is None, 60)
    assert wait_until(lambda: win.speed.targets, 5)
    s = win.speed.stats()
    # the simulator holds the commanded speed exactly: 20 % of 250 mm/s, and rapids are not counted
    assert s.reference == pytest.approx(50.0, rel=0.05) and s.within > 95.0 and s.high < 60.0
    # (at 4x speed the samples are 10 mm apart, so a target with others close by can go without one)
    assert len(win.speed.targets) >= 0.8 * len(win.checked.motions)
    model = win.table_model
    cols = [model.headerData(c, gcode_gui.Qt.Orientation.Horizontal) for c in range(model.columnCount())]
    assert cols[model.SPEED_COL] == "Speed" and cols[model.TARGET_COL] == "At target"
    k = win.checked.motions[1]
    assert model.data(model.index(k, model.SPEED_COL)) == "20 %"
    texts = [model.target_text(k) for k in win.checked.motions]
    assert "stop" in texts and any(t and "mm/s" in t for t in texts)
    assert np.isfinite(np.asarray(win.speed._line.get_ydata(), dtype=float)).any()
    win.speed.clear()
    assert model.target_text(k) is None and len(win.speed.recorder) == 0
    win.close()


def test_stats_of_an_empty_recording(qapp):
    rec = SpeedRecorder()
    s = rec.stats()
    assert s.now is None and s.mean is None and s.within is None and rec.at_targets([(0, (0, 0, 0), False)], 50) == {}
    plan = types.SimpleNamespace(motions=[], cmds=[], paths={})
    assert plan_targets(plan) == []
