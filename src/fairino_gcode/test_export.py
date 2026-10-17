#!/usr/bin/env python3
"""
test_export.py - check that an exported script sends exactly what the app sends, on the simulator.

    python test_export.py                                   # uses fairino_gcode/samples/demo_part.gcode
    python test_export.py my_part.gcode

Runs from the folder that contains fairino_gcode/. No robot and no FAIRINO SDK needed: the exported
script's "from fairino import Robot" is handed a stand-in that returns the simulated robot.
"""
import contextlib
import io
import math
import os
import runpy
import sys
import tempfile
import types

from fairino_gcode import gcode_to_fairino as eng
from fairino_gcode import sim_robot
from fairino_gcode.export_script import export_plan
from fairino_gcode.plan_model import build_plan, default_settings

GCODE = sys.argv[1] if len(sys.argv) > 1 else os.path.join("fairino_gcode", "samples", "demo_part.gcode")


def settings():
    a = default_settings()
    a.origin, a.approach, a.spindle_do = [400.0, 0.0, 200.0], 15.0, 1
    a.posture, a.start_wait, a.global_speed = "fixed", 0.05, 100
    return a


def run_with_app(a):
    """Place, check and run the plan the way the GUI's Start does."""
    sim = sim_robot.SimRobot(time_scale=200)
    frame = eng.resolve_frame(a, sim, log=lambda *_: None)
    plan = build_plan(GCODE, a, frame)
    s = eng.Streamer(sim, frame, a)
    s.stoppable, s.log, s.pause, s.on_command = True, (lambda *_: None), (lambda *_: None), (lambda *_: None)
    if not s.check_ik(plan.cmds):
        sys.exit("The app's own check refuses this plan on the simulator - try another origin.")
    s.prepare()
    s.run(plan.cmds)
    return plan, sim


def run_exported(path):
    """Run the exported script with a fake SDK that hands out a fresh simulated robot."""
    sim = sim_robot.SimRobot(time_scale=200)

    class RPC:
        is_connect = True

        def __new__(cls, ip):
            return sim

    fake = types.ModuleType("fairino")
    fake.Robot = types.SimpleNamespace(RPC=RPC)
    sys.modules["fairino"] = fake
    sys.argv, sys.stdin = [path, "-y"], io.StringIO("yes\n" + "\n" * 50)   # confirm, then Enter at M0 / M1
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        try:
            runpy.run_path(path, run_name="__main__")
        except SystemExit as e:
            out.write(f"\n(exit: {e.code})")
    return sim, out.getvalue()


def target(name, kw):
    """What a move does: instruction, Cartesian target(s) and speed."""
    poses = [kw[k] for k in ("desc_pos", "desc_pos_p", "desc_pos_t") if k in kw]
    return name, poses, kw.get("ovl"), kw.get("velAccParamMode")


def same_targets(a, b):
    """The simple script rounds to 0.0001, lets the controller solve the joints and makes every move
    wait for the arm (no blending): compare the rest."""
    if len(a) != len(b):
        return False
    for (n1, p1, *r1), (n2, p2, *r2) in (( target(*x), target(*y)) for x, y in zip(a, b)):
        if n1 != n2 or r1 != r2 or len(p1) != len(p2) or any(math.dist(u, v) > 1e-3 for u, v in zip(p1, p2)):
            return False
    return True


def main():
    plan, sim_app = run_with_app(settings())
    a = sim_app.moves()
    print(f"G-code : {GCODE}   (the app sent {len(a)} moves)")
    failed = 0
    for style, compare, what in (("checked", lambda x, y: x == y, "exactly the same moves"),
                                 ("simple", same_targets, "the same targets and speeds")):
        path = os.path.join(tempfile.mkdtemp(), f"exported_{style}.py")
        export_plan(plan, path, style=style)
        sim_script, output = run_exported(path)
        b = sim_script.moves()
        ok = compare(a, b)
        failed += not ok
        print(f"  {style:8s} script: {len(b)} moves, {'PASS' if ok else 'FAIL'} - {what} as the app"
              f"   [{output.strip().splitlines()[-1][-40:]}]")
    return failed


if __name__ == "__main__":
    sys.exit(main())
