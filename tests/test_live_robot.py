"""Step 2: connecting to the robot's status stream and showing the live TCP."""
import math

import pytest
from conftest import DEMO, free_port, wait_until

from fairino_gcode import fake_cnde_robot as fake
from fairino_gcode import gcode_gui
from fairino_gcode import robot_monitor as rm
from fairino_gcode.robot_widgets import ConnectDialog, RobotStatusPanel


# ------------------------------------------------------------------ monitor (no widgets)
def listen(port, seconds=0.6, **target):
    seen = dict(samples=[], connected=None, failed=None)
    monitor = rm.RobotMonitor(rm.Target("127.0.0.1", port=port, **target))
    monitor.sample.connect(seen["samples"].append)
    monitor.connected.connect(lambda text: seen.update(connected=text))
    monitor.failed.connect(lambda text: seen.update(failed=text))
    monitor.start()
    wait_until(lambda: seen["failed"] or len(seen["samples"]) >= seconds * 1000 / monitor.target.period_ms, 6)
    monitor.stop()
    return monitor, seen


def test_monitor_streams_tcp_and_state(qapp, robot):
    port, _ = robot()
    monitor, seen = listen(port)
    assert seen["connected"] and not seen["failed"]
    assert monitor.items == rm.ITEM_SETS[0]
    last = seen["samples"][-1]
    x, y, z = last[rm.TCP][:3]
    assert math.hypot(x - 300.0, y) == pytest.approx(60.0, abs=1e-6) and z == 200.0   # on the fake's circle
    assert last["tool_id"] == 1 and last["program_state"] == 2 and last["main_code"] == 0


def test_monitor_falls_back_when_controller_refuses_new_names(qapp, robot):
    port, _ = robot(legacy=True)
    monitor, seen = listen(port)
    assert seen["connected"] and monitor.items == rm.ITEM_SETS[1]
    assert rm.TCP in seen["samples"][-1] and "tool_id" not in seen["samples"][-1]


def test_monitor_reports_when_nobody_answers(qapp):
    monitor, seen = listen(free_port())
    assert not seen["connected"] and not seen["samples"]
    assert "no reply" in seen["failed"]
    assert not monitor.isRunning()


def test_monitor_respects_update_period(qapp, robot):
    port, _ = robot()
    _, fast = listen(port, seconds=0.5, period_ms=20)
    assert len(fast["samples"]) >= 20


def test_monitor_notices_silence(qapp, robot, monkeypatch):
    monkeypatch.setattr(rm, "STALE_AFTER_S", 0.3)
    port, stop = robot()
    events, samples = [], []
    monitor = rm.RobotMonitor(rm.Target("127.0.0.1", port=port))
    monitor.stale.connect(events.append)
    monitor.sample.connect(samples.append)
    monitor.start()
    assert wait_until(lambda: samples, 5)
    stop.set()                                    # the controller goes away mid-stream
    assert wait_until(lambda: events == [True], 4)
    monitor.stop()


def test_state_labels():
    assert rm.label(rm.PROGRAM_STATE, 2) == "Running (2)"
    assert rm.label(rm.ROBOT_MODE, 1) == "Manual (1)"
    assert rm.label(rm.ROBOT_STATE, 9) == "unknown (9)"


# ------------------------------------------------------------------ connect dialog
def test_dialog_rejects_bad_ip_without_trying(qapp):
    dialog = ConnectDialog()
    dialog.ip.setText("192.168.58")
    dialog.try_connect()
    assert dialog.monitor is None and "not an IP address" in dialog.note.text()
    assert dialog.connect_btn.isEnabled()


def test_dialog_shows_failure_and_lets_you_retry(qapp):
    dialog = ConnectDialog()
    dialog.ip.setText("127.0.0.1")
    dialog.port.setValue(free_port())
    dialog.try_connect()
    assert not dialog.connect_btn.isEnabled()                       # busy while trying
    assert wait_until(lambda: dialog.connect_btn.isEnabled(), 6)
    assert dialog.monitor is None and dialog.note.text().startswith("Could not connect")
    assert dialog.result() == 0                                     # still open


def test_dialog_connects_and_hands_over_a_running_monitor(qapp, robot):
    port, _ = robot()
    dialog = ConnectDialog()
    dialog.ip.setText("127.0.0.1")
    dialog.port.setValue(port)
    dialog.try_connect()
    assert wait_until(lambda: dialog.result() == 1, 5)
    assert dialog.monitor.isRunning() and dialog.monitor.target.ip == "127.0.0.1"
    dialog.monitor.stop()


def test_dialog_cancel_while_connecting_stops_the_attempt(qapp):
    dialog = ConnectDialog()
    dialog.ip.setText("127.0.0.1")
    dialog.port.setValue(free_port())
    dialog.try_connect()
    monitor = dialog.monitor
    dialog.reject()
    assert dialog.monitor is None and not monitor.isRunning()


def test_dialog_protocol_sets_default_port(qapp):
    dialog = ConnectDialog()
    assert dialog.port.value() == rm.UDP_PORT
    dialog.protocol.setCurrentIndex(1)
    assert dialog.port.value() == rm.TCP_PORT and dialog.target().use_tcp


# ------------------------------------------------------------------ status panel
def test_panel_shows_pose_state_and_fault(qapp):
    panel = RobotStatusPanel()
    panel.set_connected(rm.Target("10.0.0.5"))
    panel.show_sample({rm.TCP: [1.5, -2.25, 300.0, 180.0, 0.0, 90.0], "tool_id": 2, "wobj_id": 1,
                       "robot_mode": 1, "program_state": 3, "main_code": 0, "sub_code": 0, "emergency_stop": 0})
    assert [v.text() for v in panel.pose[:3]] == ["1.500", "-2.250", "300.000"]
    assert "Tool 2, workpiece 1" in panel.state.text() and "Manual (1)" in panel.state.text()
    assert panel.fault.text() == "No fault"
    panel.show_sample({rm.TCP: [0] * 6, "main_code": 14, "sub_code": 3, "emergency_stop": 1})
    assert "main code 14, sub code 3" in panel.fault.text() and "EMERGENCY STOP" in panel.fault.text()
    panel.set_disconnected()
    assert panel.pose[0].text() == "-" and "Not connected" in panel.link.text()


# ------------------------------------------------------------------ main window
def connect(win, port):
    monitor = rm.RobotMonitor(rm.Target("127.0.0.1", port=port))
    monitor.start()
    win.attach_monitor(monitor)
    return monitor


def test_window_draws_the_tcp_on_the_plan(qapp, robot):
    port, _ = robot(path=fake.gcode_path(DEMO))
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    win.load(DEMO)
    connect(win, port)
    canvas = win.canvas
    assert wait_until(lambda: len(canvas._trail) > 5, 6)
    # the dot is where the robot says it is, and the panel shows the same numbers
    dot = canvas._live[-1].get_data_3d()
    assert [float(v[0]) for v in dot] == pytest.approx(list(canvas._tcp))
    assert win.robot_panel.pose[0].text() != "-"
    # the pretend robot traces the plan, so every trail point lies on a drawn plan point's path
    plan_pts = canvas._cloud
    for p in list(canvas._trail):
        assert min(((plan_pts - p) ** 2).sum(axis=1)) ** 0.5 < 60.0
    assert not win.act_connect.isEnabled() and win.act_disconnect.isEnabled()
    win.disconnect_robot()
    assert canvas._tcp is None and len(canvas._trail) == 0
    assert win.act_connect.isEnabled() and "Not connected" in win.robot_panel.link.text()
    win.close()


def test_window_moves_the_dot_without_full_redraws(qapp, robot):
    port, _ = robot()
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    win.load(DEMO)
    connect(win, port)
    assert wait_until(lambda: win.canvas._tcp is not None and win.canvas._bg is not None, 6)
    draws = []
    win.canvas.mpl_connect("draw_event", draws.append)
    first = win.canvas._tcp.copy()
    assert wait_until(lambda: abs(win.canvas._tcp - first).max() > 5.0, 6)
    assert draws == []                              # the dot moved by blitting only
    win.close()


def test_window_zooms_out_once_when_robot_is_far_from_plan(qapp, robot):
    port, _ = robot()                               # circle around (300, 0, 200); plan is at 0..100
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    win.load(DEMO)
    half_before = win.canvas._half
    connect(win, port)
    assert wait_until(lambda: win.canvas._tcp is not None, 6)
    assert win.canvas._half > half_before
    assert (abs(win.canvas._tcp - win.canvas._centre) <= win.canvas._half).all()
    win.close()


def test_window_shows_robot_without_a_file(qapp, robot):
    port, _ = robot()
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    connect(win, port)
    assert wait_until(lambda: win.canvas._tcp is not None, 6)
    assert (abs(win.canvas._tcp - win.canvas._centre) <= win.canvas._half).all()
    win.load(DEMO)                                  # loading a plan keeps the robot on screen
    assert win.canvas._tcp is not None and win.canvas._live[-1].axes is win.canvas.ax
    win.close()


def test_window_greys_the_dot_when_data_stops_and_recovers_from_loss(qapp, robot, monkeypatch):
    monkeypatch.setattr(rm, "STALE_AFTER_S", 0.3)
    port, stop = robot()
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    connect(win, port)
    assert wait_until(lambda: win.canvas._tcp is not None, 6)
    stop.set()
    assert wait_until(lambda: not win.canvas._tcp_live, 4)
    assert "No data" in win.robot_panel.link.text()
    win._on_lost("network unreachable")
    assert win.monitor is None and "Connection lost" in win.robot_panel.link.text()
    win.close()


def test_closing_the_window_stops_listening(qapp, robot):
    port, _ = robot()
    win = gcode_gui.MainWindow(gcode_gui.default_settings())
    win.show()
    monitor = connect(win, port)
    assert wait_until(lambda: win.canvas._tcp is not None, 6)
    win.close()
    assert not monitor.isRunning()
