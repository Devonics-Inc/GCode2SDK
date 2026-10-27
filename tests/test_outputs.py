"""Per-line digital outputs: up to 3 control-box DOs, high or low on every G-code line ("Per line" tab)."""
import os
import time

from conftest import DEMO, wait_until

from fairino_gcode import export_script, gcode_gui, sim_robot
from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode import servo_path as sp
from fairino_gcode.plan_model import build_plan, default_settings
from fairino_gcode.run_control import HERE, MAX_OUTPUTS, RunSetup, RunSetupDialog

SAMPLES = os.path.dirname(DEMO)
OUTLINE = list(range(9, 17))            # demo_part.gcode: the outline, after the plunge on line 8
HOLE = 22                               # ...and the full circle of the hole


def settings(**kw):
    a = default_settings()
    a.origin, a.approach, a.posture, a.start_wait = [400.0, 0.0, 200.0], 0.0, "fixed", 0.05
    for key, value in kw.items():
        setattr(a, key, value)
    return a


def demo_outputs():
    """DO3 high along the outline, DO5 high on the hole."""
    lines = {n: [True, False] for n in OUTLINE}
    lines[HOLE] = [False, True]
    return [3, 5], lines


# ------------------------------------------------------------------ engine
def test_a_setdo_step_where_the_states_change_and_one_at_the_start():
    cmds, _w = eng.parse_gcode("G0 X5 Y5\nG1 X10\nG1 X20\nG1 X30\nG0 X0\n", 600.0, False)
    out = eng.add_outputs(cmds, [3, 5], {2: [True, False], 3: [True, True]})
    steps = [(c.line, c.outputs) for c in out if c.kind == "do"]
    assert steps == [(1, [(3, False), (5, False)]),           # the start sets every output
                     (2, [(3, True)]),                        # then only what changes
                     (3, [(5, True)]),
                     (4, [(3, False), (5, False)])]
    assert [c.kind for c in out[:3]] == ["do", "line", "do"]  # each one right before its line
    states = eng.output_states(out)
    k = next(i for i, c in enumerate(out) if c.line == 3 and c.kind == "line")
    assert dict(states[k]) == {3: True, 5: True}
    assert eng.add_outputs(cmds, [], {2: [True]}) is cmds     # no outputs: nothing changes


def test_the_plan_lists_the_switches_and_breaks_blending_and_streaming_there():
    pins, lines = demo_outputs()
    plan = build_plan(DEMO, settings(outputs=pins, output_lines=lines))
    switches = [(k, c) for k, c in enumerate(plan.cmds) if c.kind == "do"]
    assert [c.line for _k, c in switches] == [4, 9, 17, HOLE, 23]
    assert "SetDO(3, 1)" in eng.describe(switches[1][1], plan.settings)
    assert any(line.startswith("Outputs") and "DO3 (low outside the lines), DO5" in line for line in plan.summary)
    k = next(k for k in plan.motions if plan.cmds[k].line == 12)
    assert dict(plan.outputs[k]) == {3: True, 5: False}
    for first, last in sp.feed_groups(plan.cmds):              # a fixed-speed stream never runs through one
        assert all(plan.cmds[i].kind != "do" for i in range(first, last + 1))


def test_the_robot_switches_each_output_at_rest_where_its_line_begins_and_all_off_at_the_end():
    pins, lines = demo_outputs()
    a = settings(outputs=pins, output_lines=lines)
    sim = sim_robot.SimRobot(time_scale=200)
    frame = eng.resolve_frame(a, sim, log=lambda *_: None)
    plan = build_plan(DEMO, a, frame)
    s = eng.Streamer(sim, frame, a)
    s.stoppable, s.log, s.pause, s.on_command = True, (lambda *_: None), (lambda *_: None), (lambda *_: None)
    s.prepare()
    s.run(plan.cmds)
    calls = [(n, k) for n, k in sim.calls if n in ("MoveJ", "MoveL", "MoveC", "Circle", "SetDO")]
    sets = [(i, k) for i, (n, k) in enumerate(calls) if n == "SetDO" and k["id"] in pins]
    assert all(not k["while_moving"] for _i, k in sets)       # the arm is at rest for every switch
    # DO3 goes high after exactly the moves before line 9
    i_on = next(i for i, k in sets if k["id"] == 3 and k["status"] == 1)
    moves_before = sum(1 for n, _k in calls[:i_on] if n != "SetDO")
    do_index = next(k for k, c in enumerate(plan.cmds) if c.kind == "do" and c.line == 9)
    assert moves_before == sum(1 for c in plan.cmds[:do_index] if c.kind in eng.MOTION_KINDS)
    assert sim.do[3] == 0 and sim.do[5] == 0 and s.outputs_live     # back at their (default low) rest state


def test_a_stop_puts_the_outputs_at_their_state_outside_the_lines():
    sim = sim_robot.SimRobot()
    a = settings(outputs=[3, 8], output_lines={}, output_defaults=[False, True])
    s = eng.Streamer(sim, eng.resolve_frame(a, sim, log=lambda *_: None), a)
    s.emergency_stop()                                        # nothing switched yet: nothing to put back
    assert 3 not in sim.do and 8 not in sim.do
    sim.SetDO(3, 1)
    sim.SetDO(8, 0)
    s.outputs_live = True
    s.emergency_stop()
    assert sim.do[3] == 0 and sim.do[8] == 1


def test_outside_the_lines_each_output_takes_its_default():
    """Approach, retract and the end of the job are outside the lines: CO0 is high there, low on the lines."""
    a = settings(outputs=[3, 8], output_lines={n: [True, False] for n in OUTLINE}, output_defaults=[False, True],
                 approach=20.0)
    sim = sim_robot.SimRobot(time_scale=200)
    frame = eng.resolve_frame(a, sim, log=lambda *_: None)
    plan = build_plan(DEMO, a, frame)
    logged = []
    s = eng.Streamer(sim, frame, a)
    s.stoppable, s.log, s.pause, s.on_command = True, logged.append, (lambda *_: None), (lambda *_: None)
    s.prepare()
    s.run(plan.cmds)
    calls = [(n, k) for n, k in sim.calls if n in ("MoveJ", "MoveL", "MoveC", "Circle", "SetDO")]
    co0 = [(i, k["status"]) for i, (n, k) in enumerate(calls) if n == "SetDO" and k["id"] == 8]
    moves = [i for i, (n, _k) in enumerate(calls) if n != "SetDO"]
    assert co0[0] == (co0[0][0], 1) and co0[0][0] < moves[0]          # high before the approach move...
    assert co0[1][1] == 0 and co0[1][0] < moves[1]                    # ...low once the first line starts...
    assert co0[-1][1] == 1 and co0[-1][0] < moves[-1]                 # ...and high again before the retract
    assert sim.do[8] == 1 and sim.do[3] == 0
    assert any("outside the lines" in line for line in logged)
    assert any("CO0 (high outside the lines)" in line for line in plan.summary)
    simple = export_script.build_simple_script(plan.cmds, plan.settings, DEMO)
    assert simple.count("robot.SetDO(8, 1)   # CO0 high") == 2 and "    robot.SetDO(8, 1)\n" in simple
    checked = export_script.build_script(plan.cmds, plan.settings, DEMO)
    assert "OUTPUT_REST = [(3, False), (8, True)]" in checked and "job.rest_outputs()" in checked


def test_both_exported_scripts_switch_the_outputs(tmp_path):
    pins, lines = demo_outputs()
    plan = build_plan(DEMO, settings(outputs=pins, output_lines=lines))
    with open(export_script.export_plan(plan, str(tmp_path / "s.py"))) as f:
        simple = f.read()
    assert "robot.SetDO(3, 1)" in simple and "robot.SetDO(5, 1)" in simple
    assert simple.count("robot.SetDO(3, 0)") >= 3        # off after the outline, and on Ctrl-C
    with open(export_script.export_plan(plan, str(tmp_path / "c.py"), style="checked")) as f:
        checked = f.read()
    assert "job.set_outputs([(3, True)], 9," in checked and "def set_outputs(" in checked


# ------------------------------------------------------------------ dialog
def test_the_per_line_tab_sets_outputs_line_by_line(qapp):
    lines = gcode_gui.gcode_lines(build_plan(DEMO, default_settings()))
    assert [row[:2] for row in lines][0] == (4, "S6000 M3") and (HOLE, "G2 I-10 J0 F900") in [r[:2] for r in lines]
    dialog = RunSetupDialog(RunSetup(placement=HERE), lines=lines)
    assert [dialog.tabs.tabText(i) for i in range(dialog.tabs.count())] == ["Global", "Per line"]
    assert len(dialog.out_use) == MAX_OUTPUTS == 3 and not dialog.used_slots()
    assert all(dialog.line_table.isColumnHidden(dialog.OUT0 + k) for k in range(3))
    dialog.out_use[0].setChecked(True)
    dialog.out_pin[0].setValue(3)
    dialog.out_use[1].setChecked(True)
    dialog.out_pin[1].setValue(5)
    assert dialog.line_table.horizontalHeaderItem(dialog.OUT0).text() == "DO3"
    assert not dialog.line_table.isColumnHidden(dialog.OUT0 + 1)
    # many lines at once: select the outline, High for DO 3
    rows = [r for r, (n, *_rest) in enumerate(lines) if n in OUTLINE]
    for r in rows:
        dialog.line_table.selectionModel().select(
            dialog.line_table.model().index(r, 0),
            gcode_gui.QtCore.QItemSelectionModel.SelectionFlag.Select
            | gcode_gui.QtCore.QItemSelectionModel.SelectionFlag.Rows)
    dialog.set_selected(0, True)
    # one line: tick DO 5 on the hole
    hole = next(r for r, (n, *_rest) in enumerate(lines) if n == HOLE)
    dialog.line_table.clearSelection()
    dialog.line_table.item(hole, dialog.OUT0 + 1).setCheckState(gcode_gui.Qt.CheckState.Checked)
    dialog.out_default[1].setCurrentIndex(1)                 # DO 5 high outside the lines
    dialog.accept()
    chosen = dialog.chosen
    assert chosen.outputs == [3, 5]
    assert chosen.output_lines == {**{n: [True, False] for n in OUTLINE}, HOLE: [False, True]}
    assert chosen.output_defaults == [False, True]
    assert "DO3, DO5 set per line" in chosen.describe()
    a = chosen.apply(default_settings())
    assert a.outputs == [3, 5] and a.output_lines[HOLE] == [False, True]
    # opened again: the same ticks
    again = RunSetupDialog(chosen, lines=lines)
    assert again.used_slots() == [0, 1] and again.out_pin[1].value() == 5
    assert again.line_table.item(hole, again.OUT0 + 1).checkState() == gcode_gui.Qt.CheckState.Checked
    assert again.line_table.item(hole, again.OUT0).checkState() == gcode_gui.Qt.CheckState.Unchecked
    assert again.out_default[1].currentText() == "High"
    # switching an output off drops it, and its ticks, from the setup
    again.out_use[0].setChecked(False)
    again.accept()
    assert again.chosen.outputs == [5] and again.chosen.output_lines == {HOLE: [True]}
    assert again.chosen.output_defaults == [True]


def test_outputs_0_to_7_are_do0_to_do7_and_8_to_15_are_co0_to_co7(qapp, tmp_path):
    assert [eng.output_name(p) for p in (0, 7, 8, 15)] == ["DO0", "DO7", "CO0", "CO7"]
    assert eng.set_do_text(9, True) == "SetDO(9, 1) CO1 high"
    dialog = RunSetupDialog(RunSetup(placement=HERE, spindle_do=-1), lines=[(1, "G1 X10"), (2, "G1 X20")])
    names = [dialog.out_pin[0].itemText(i) for i in range(dialog.out_pin[0].count())]
    assert [n for n in names if n] == [f"DO{i}" for i in range(8)] + [f"CO{i}" for i in range(8)]
    assert dialog.spindle_do.currentText() == "none" and dialog.spindle_do.value() == -1
    dialog.out_use[0].setChecked(True)
    dialog.out_pin[0].setValue(10)                            # CO2 is SetDO number 10
    assert dialog.out_pin[0].currentText() == "CO2"
    assert dialog.line_table.horizontalHeaderItem(dialog.OUT0).text() == "CO2" and dialog.bulk_name[0].text() == "CO2:"
    dialog.line_table.item(1, dialog.OUT0).setCheckState(gcode_gui.Qt.CheckState.Checked)
    dialog.accept()
    assert dialog.chosen.outputs == [10] and "CO2 set per line" in dialog.chosen.describe()
    # what is sent is SetDO(10, ...); everything the operator reads says CO2
    path = tmp_path / "two.gcode"
    path.write_text("G1 X10\nG1 X20\n")
    plan = build_plan(str(path), settings(outputs=[10], output_lines={2: [True]}))
    switch = next(c for c in plan.cmds if c.kind == "do" and c.line == 2)
    assert eng.describe(switch, plan.settings).endswith("SetDO(10, 1) CO2 high")
    assert any("CO2" in line for line in plan.summary)
    with open(export_script.export_plan(plan, str(tmp_path / "s.py"))) as f:
        assert "robot.SetDO(10, 1)   # CO2 high" in f.read()
    with open(export_script.export_plan(plan, str(tmp_path / "c.py"), style="checked")) as f:
        assert "job.set_outputs([(10, True)]" in f.read()


def test_the_per_line_tab_refuses_a_do_used_twice(qapp):
    dialog = RunSetupDialog(RunSetup(placement=HERE, spindle_do=2), lines=[(1, "G1 X1")])
    dialog.out_use[0].setChecked(True)
    dialog.out_use[1].setChecked(True)
    dialog.out_pin[0].setValue(4)
    dialog.out_pin[1].setValue(4)
    dialog.accept()
    assert dialog.chosen is None and "its own DO" in dialog.message.text() and dialog.tabs.currentIndex() == 1
    dialog.out_pin[1].setValue(2)                              # the M3 / M5 output
    dialog.accept()
    assert dialog.chosen is None and "M3 / M5" in dialog.message.text()
    dialog.out_pin[1].setValue(6)
    dialog.accept()
    assert dialog.chosen is not None and dialog.chosen.outputs == [4, 6]
    empty = RunSetupDialog(RunSetup())                         # no file: a note instead of the table
    assert not empty.no_lines.isHidden() and empty.line_table.isHidden()


def test_each_line_shows_its_robot_command_in_its_colour(qapp, tmp_path):
    lines = gcode_gui.gcode_lines(build_plan(DEMO, default_settings()))
    by_line = {n: (command, colour) for n, _src, command, colour in lines}
    assert by_line[7] == ("MoveJ", gcode_gui.STYLES["rapid"]["color"])
    assert by_line[10] == ("MoveC", gcode_gui.STYLES["MoveC"]["color"])
    assert by_line[HOLE] == ("Circle", gcode_gui.STYLES["Circle"]["color"])
    assert by_line[32] == ("wait 0.5 s", None)
    small = tmp_path / "small_arc.gcode"                      # MoveC points under 1 mm apart: short MoveL
    small.write_text("G0 X0 Y0\nG1 X1\nG2 X1 Y1.2 I0 J0.6\n")
    command = dict((n, c) for n, _s, c, _col in gcode_gui.gcode_lines(build_plan(str(small), default_settings())))[3]
    assert command.startswith("MoveL \u00d7")
    dialog = RunSetupDialog(RunSetup(), lines=lines)
    row = next(r for r, line in enumerate(lines) if line[0] == 10)
    assert dialog.line_table.horizontalHeaderItem(2).text() == "Robot command"
    assert dialog.line_table.item(row, 2).text() == "MoveC"
    assert dialog.line_table.item(row, 2).foreground().color().name() == gcode_gui.STYLES["MoveC"]["color"]


def test_ticking_a_box_in_a_selection_of_lines_sets_them_all(qapp):
    import importlib

    from matplotlib.backends.qt_compat import QtCore, QtWidgets
    QtTest = importlib.import_module(QtCore.__name__.rsplit(".", 1)[0] + ".QtTest")   # PySide6 / PyQt
    lines = gcode_gui.gcode_lines(build_plan(DEMO, default_settings()))
    dialog = RunSetupDialog(RunSetup(placement=HERE, outputs=[3]), lines=lines)
    dialog.resize(700, 700)
    dialog.tabs.setCurrentIndex(1)
    dialog.show()
    table, col = dialog.line_table, dialog.OUT0
    table.setRangeSelected(QtWidgets.QTableWidgetSelectionRange(3, 0, 6, table.columnCount() - 1), True)
    assert dialog.selected_rows() == [3, 4, 5, 6]
    # a real click on the box of one of them (the click would leave only that line selected)
    opt = QtWidgets.QStyleOptionViewItem()
    opt.rect = table.visualItemRect(table.item(5, col))
    opt.features = QtWidgets.QStyleOptionViewItem.ViewItemFeature.HasCheckIndicator
    box = table.style().subElementRect(QtWidgets.QStyle.SubElement.SE_ItemViewItemCheckIndicator, opt, table)
    QtTest.QTest.mouseClick(table.viewport(), QtCore.Qt.MouseButton.LeftButton, QtCore.Qt.KeyboardModifier.NoModifier,
                            box.center())
    assert wait_until(lambda: dialog.selected_rows() == [3, 4, 5, 6], 2)       # the selection is kept
    ticked = [r for r in range(table.rowCount()) if table.item(r, col).checkState() == gcode_gui.Qt.CheckState.Checked]
    assert ticked == [3, 4, 5, 6]
    # the keyboard (space on the current line) works the same way
    table.item(4, col).setCheckState(gcode_gui.Qt.CheckState.Unchecked)
    ticked = [r for r in range(table.rowCount()) if table.item(r, col).checkState() == gcode_gui.Qt.CheckState.Checked]
    assert ticked == []
    # a box outside the selection changes only its own line
    table.item(10, col).setCheckState(gcode_gui.Qt.CheckState.Checked)
    ticked = [r for r in range(table.rowCount()) if table.item(r, col).checkState() == gcode_gui.Qt.CheckState.Checked]
    assert ticked == [10]
    dialog.accept()
    assert dialog.chosen.output_lines == {lines[10][0]: [True]}


# ------------------------------------------------------------------ window
def test_the_command_table_shows_the_outputs_of_every_row(qapp):
    settings_ = default_settings()
    settings_.start_wait = 0.05
    sim = sim_robot.SimRobot(time_scale=50)
    win = gcode_gui.MainWindow(settings_, simulator=sim)
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    assert win.table.isColumnHidden(win.table_model.OUT_COL)  # nothing set: no Outputs column
    pins, lines = demo_outputs()
    win.apply_setup(RunSetup(placement=HERE, approach=0, outputs=pins, output_lines=lines))
    assert wait_until(lambda: win.job is None and win.checked is not None, 20)
    model, col = win.table_model, win.table_model.OUT_COL
    assert not win.table.isColumnHidden(col)
    switch = next(k for k, c in enumerate(win.plan.cmds) if c.kind == "do" and c.line == 9)
    assert model.data(model.index(switch, 3)) == "SetDO(3, 1) DO3 high"
    k = next(k for k in win.plan.motions if win.plan.cmds[k].line == 12)
    assert model.data(model.index(k, col)) == "DO3 ●  DO5 ○"
    assert model.data(model.index(k, col), gcode_gui.Qt.ItemDataRole.ToolTipRole) == "DO3 high, DO5 low"
    # a run sends them; the outputs are off again afterwards
    win.start_run(confirm=False)
    assert wait_until(lambda: win.job is None, 60)
    assert "Run finished." in win.banner.text(), win.banner.text()
    assert any(n == "SetDO" and k["id"] == 5 and k["status"] == 1 for n, k in sim.calls)
    assert sim.do[3] == 0 and sim.do[5] == 0
    # another file: its lines are different, so none are set (the DOs chosen stay)
    win.load(os.path.join(SAMPLES, "01_lines_square.gcode"))
    assert win.setup.outputs == pins and win.setup.output_lines == {}
    win.close()


# ------------------------------------------------------------------ per-line delays
def test_a_line_delay_waits_after_the_outputs_switch_and_before_the_line():
    cmds, _w = eng.parse_gcode("G0 X5 Y5\nG1 X10\nG1 X20\n", 600.0, False)
    out = eng.add_delays(eng.add_outputs(cmds, [3], {2: [True]}), {2: 250, 3: 0})
    assert [(c.kind, c.line) for c in out] == [("do", 1), ("line", 1), ("do", 2), ("dwell", 2), ("line", 2),
                                               ("do", 3), ("line", 3)]
    wait = out[3]
    assert wait.added == "delay" and wait.seconds == 0.25
    assert eng.describe(wait, default_settings()).endswith("wait 250 ms (per-line delay)")
    assert eng.add_delays(cmds, {}) is cmds and eng.add_delays(cmds, {2: 0}) is cmds


def test_the_robot_waits_before_a_delayed_line(tmp_path):
    a = settings(line_delays={9: 300})
    sim = sim_robot.SimRobot(time_scale=200)
    frame = eng.resolve_frame(a, sim, log=lambda *_: None)
    plan = build_plan(DEMO, a, frame)
    assert any(line.startswith("Line delays") and "0.3 s" in line for line in plan.summary)
    seen = []
    s = eng.Streamer(sim, frame, a)
    s.stoppable, s.log, s.pause = True, (lambda *_: None), (lambda *_: None)
    s.on_command = lambda c, _text: seen.append((c, time.monotonic()))
    s.prepare()
    s.run(plan.cmds)
    k = next(i for i, (c, _t) in enumerate(seen) if c.added)
    assert seen[k][0].line == 9 and seen[k + 1][0].line == 9 and seen[k + 1][0].kind == "line"
    assert seen[k + 1][1] - seen[k][1] >= 0.29                       # the next move went out 300 ms later
    # both exported scripts wait there too
    simple = export_script.build_simple_script(plan.cmds, plan.settings, DEMO)
    assert "# N9 G1 X92 Y0 F1200 - per-line delay\n    time.sleep(0.3)" in simple
    assert "job.dwell(0.3, 9, 'G1 X92 Y0 F1200')" in export_script.build_script(plan.cmds, plan.settings, DEMO)


def test_the_wait_column_sets_one_line_or_all_the_selected_ones(qapp):
    import importlib

    from matplotlib.backends.qt_compat import QtCore, QtWidgets
    QtTest = importlib.import_module(QtCore.__name__.rsplit(".", 1)[0] + ".QtTest")
    lines = gcode_gui.gcode_lines(build_plan(DEMO, default_settings()))
    dialog = RunSetupDialog(RunSetup(placement=HERE), lines=lines)
    dialog.resize(800, 700)
    dialog.tabs.setCurrentIndex(1)
    dialog.show()
    table, col = dialog.line_table, dialog.DELAY_COL
    assert table.horizontalHeaderItem(col).text() == "Wait" and not table.isColumnHidden(col)
    def type_in(text):          # the first key opens the editor (a spin box), the rest goes to it
        QtTest.QTest.keyClick(table, text[0])
        editors = [e for e in table.viewport().findChildren(QtWidgets.QSpinBox) if e.isVisible()]
        assert len(editors) == 1 and table.state() == QtWidgets.QAbstractItemView.State.EditingState
        editor = editors[0]
        QtTest.QTest.keyClicks(editor, text[1:])
        QtTest.QTest.keyClick(editor, QtCore.Qt.Key.Key_Return)

    # one line, typed in
    table.setCurrentCell(2, col)
    type_in("250")
    assert wait_until(lambda: table.item(2, col).data(gcode_gui.Qt.ItemDataRole.EditRole) == 250, 2)
    assert dialog._delay == {lines[2][0]: 250}
    # several selected lines: typing in one of them sets them all
    table.clearSelection()
    table.setRangeSelected(QtWidgets.QTableWidgetSelectionRange(5, 0, 7, table.columnCount() - 1), True)
    table.selectionModel().setCurrentIndex(table.model().index(6, col),
                                           QtCore.QItemSelectionModel.SelectionFlag.NoUpdate)
    type_in("500")
    assert wait_until(lambda: dialog._delay.get(lines[5][0]) == 500, 2)
    assert [dialog._delay.get(lines[r][0]) for r in (5, 6, 7)] == [500, 500, 500]
    assert dialog._delay[lines[2][0]] == 250                           # not selected: left alone
    # the buttons under the table
    dialog.bulk_delay.setValue(1200)
    dialog.set_selected_delay(dialog.bulk_delay.value())
    assert [dialog._delay.get(lines[r][0]) for r in (5, 6, 7)] == [1200, 1200, 1200]
    table.clearSelection()
    table.setRangeSelected(QtWidgets.QTableWidgetSelectionRange(6, 0, 6, table.columnCount() - 1), True)
    dialog.set_selected_delay(0)                                       # "No wait"
    dialog.accept()
    chosen = dialog.chosen
    assert chosen.line_delays == {lines[2][0]: 250, lines[5][0]: 1200, lines[7][0]: 1200}
    assert "3 line delay(s)" in chosen.describe()
    assert chosen.apply(default_settings()).line_delays == chosen.line_delays
    again = RunSetupDialog(chosen, lines=lines)
    assert again.line_table.item(5, again.DELAY_COL).data(gcode_gui.Qt.ItemDataRole.EditRole) == 1200
    assert again.line_table.item(6, again.DELAY_COL).data(gcode_gui.Qt.ItemDataRole.EditRole) == 0


def test_line_delays_show_in_the_command_table_and_go_with_the_file(qapp):
    settings_ = default_settings()
    settings_.start_wait = 0.05
    win = gcode_gui.MainWindow(settings_, simulator=sim_robot.SimRobot(time_scale=50))
    win.show()
    win.load(DEMO)
    win.attach_simulator()
    win.apply_setup(RunSetup(placement=HERE, approach=0, line_delays={9: 250}))
    assert wait_until(lambda: win.job is None and win.checked is not None, 20)
    model = win.table_model
    k = next(k for k, c in enumerate(win.plan.cmds) if c.added)
    assert model.data(model.index(k, 3)) == "wait 250 ms (per-line delay)"
    assert model.data(model.index(k, 3), gcode_gui.Qt.ItemDataRole.ForegroundRole).color().name() == \
        gcode_gui.OUTPUT_COLOR
    assert [r[0] for r in gcode_gui.gcode_lines(win.plan)].count(9) == 1      # the wait is not a line of its own
    win.load(os.path.join(SAMPLES, "01_lines_square.gcode"))
    assert win.setup.line_delays == {}
    win.close()


def test_the_per_line_headers_are_never_cut_off(qapp):
    lines = gcode_gui.gcode_lines(build_plan(DEMO, default_settings()))
    dialog = RunSetupDialog(RunSetup(placement=HERE, outputs=[3, 8]), lines=lines)
    dialog.tabs.setCurrentIndex(1)
    dialog.resize(560, 600)                                          # a narrow dialog
    dialog.show()
    qapp.processEvents()
    table, header = dialog.line_table, dialog.line_table.horizontalHeader()
    metrics = header.fontMetrics()
    for col in (0, 2, dialog.DELAY_COL, dialog.OUT0, dialog.OUT0 + 1):
        text = table.horizontalHeaderItem(col).text()
        assert table.columnWidth(col) >= metrics.horizontalAdvance(text), (col, text)
    assert table.columnWidth(2) >= header.sectionSizeFromContents(2).width()   # "Robot command", padding included
