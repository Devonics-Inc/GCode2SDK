# GCode2SDK

GCode2SDK is a tool that parses G-code and translates its motion commands into robot SDK bypassing webapp completely

Featuring:

* 3D preview of the plan, command by command (G0 -> MoveJ, G1 -> MoveL, G2/G3 -> MoveC / Circle)
* live robot TCP in the same view
* placement by teach point, at the tool, automatic, or typed; reachability check of the whole path
* Start / Stop, with every command logged

<video src="docs/GCode2SDK.mp4" width="100%" autoplay loop muted playsinline></</video>

# Disclaimer
GCode2SDK is provided as open-source software for experimentation, development, and educational purposes. It is provided "as is" and without warranty of any kind. The tool is not an official replacement for the robot manufacturer's software or webapp and has not been guaranteed for production or safety-critical applications.

Users are responsible for validating generated motion commands, robot reachability, tool configuration, coordinate systems, speeds, and the overall safety of the robot and its environment before execution. Use this software at your own risk.

## For users: the standalone application

Linux: unpack `fairino-gcode-<version>-linux.tar.gz`, then run `sh fairino-gcode/install.sh`.
It appears in the applications menu as "FAIRINO G-code Runner".

Windows: run `fairino-gcode-<version>-setup.exe`, or unpack the zip and start `fairino-gcode.exe`.

Try it without a robot: start it as `fairino-gcode --simulate`. Check an installation with
`fairino-gcode --self-test`.

## For developers

    pip install -e ".[dev]"        # from this folder; Python 3.9 or newer
    fairino-gcode                  # the application
    fairino-gcode --simulate       # with a simulated robot
    fairino-gcode-cli part.gcode --dry-run      # the command-line tool
    python -m pytest               # the tests (all against the simulated robot)

Commanding a real robot needs the FAIRINO Python SDK (the `fairino` package), which is not on PyPI.
Either start the tools from the SDK's `linux/` (or `windows/`) folder, or add that folder to `PYTHONPATH`.

### Layout

    src/fairino_gcode/
        gcode_to_fairino.py      the engine: G-code -> robot instructions -> SDK calls (also the CLI)
        plan_model.py            a file turned into a plan that can be drawn
        gcode_gui.py             the window
        run_control.py           set up / check / run / stop
        auto_place.py            automatic placement
        robot_monitor.py         live status (uses fairino_CNDE_listener.py)
        robot_widgets.py         connect dialog and robot panel
        sim_robot.py, fake_cnde_robot.py     the simulated robot
    tests/                       pytest
    packaging/                   building the standalone application

### Building the standalone application

    python packaging/build_app.py --sdk /path/to/fairino-python-sdk/linux

Builds `dist/fairino-gcode/`, runs its self-test, and packs it. Build on each operating system you
ship for. On Windows, `packaging/windows/installer.iss` (Inno Setup) turns the folder into a Setup.exe.

### Releasing

1. Set the version in `src/fairino_gcode/__init__.py`.
2. `python -m pytest`
3. Build on each operating system; the build must end with "self-test passed" and "FAIRINO SDK: included".
4. Run the built application against a real robot (see the checklist in the release notes).
5. Have `THIRD_PARTY_NOTICES.md` reviewed if the dependencies changed.
