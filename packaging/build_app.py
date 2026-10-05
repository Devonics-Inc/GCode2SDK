#!/usr/bin/env python3
"""
Build the standalone application (no Python needed on the target computer) with PyInstaller.

    python packaging/build_app.py --sdk /path/to/fairino-python-sdk/linux      # or .../windows

--sdk is the folder that CONTAINS the SDK's "fairino" package. It is copied into the application so
that it can command a real robot. Without it (--no-sdk) the result can only simulate.

The result is the folder dist/fairino-gcode/ (start it with the fairino-gcode program inside) and an
archive of it next to it. The build ends by starting the application once with --self-test, which runs
the demo part on the simulated robot: a build that passes is complete.

PyInstaller does not cross-compile: build on Linux for Linux and on Windows for Windows.
"""
import argparse
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = "fairino-gcode"
PACKAGE = os.path.join(ROOT, "src", "fairino_gcode")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--sdk", help='folder that contains the FAIRINO SDK\'s "fairino" package')
    where.add_argument("--no-sdk", action="store_true", help="build a simulation-only application")
    ap.add_argument("--skip-test", action="store_true", help="do not run the built application's self-test")
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(ROOT, "src"))
    from fairino_gcode import __version__

    cmd = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--windowed", "--name", NAME,
           "--distpath", os.path.join(ROOT, "dist"), "--workpath", os.path.join(ROOT, "build"),
           "--specpath", os.path.join(ROOT, "build"),
           "--paths", os.path.join(ROOT, "src"),
           "--add-data", os.path.join(PACKAGE, "samples") + os.pathsep + os.path.join("fairino_gcode", "samples"),
           "--add-data", os.path.join(PACKAGE, "resources") + os.pathsep + os.path.join("fairino_gcode", "resources"),
           "--icon", os.path.join(PACKAGE, "resources", "icon.png"),
           "--hidden-import", "matplotlib.backends.backend_qtagg",
           "--hidden-import", "PySide6.QtCore", "--hidden-import", "PySide6.QtGui",
           "--hidden-import", "PySide6.QtWidgets",
           # one Qt binding only (PySide6, LGPL), and nothing the application does not use
           "--exclude-module", "PyQt5", "--exclude-module", "PyQt6", "--exclude-module", "PySide2",
           "--exclude-module", "tkinter", "--exclude-module", "pytest"]
    if args.sdk:
        sdk = os.path.abspath(args.sdk)
        if not os.path.isdir(os.path.join(sdk, "fairino")):
            sys.exit(f'{sdk} does not contain a "fairino" folder. Point --sdk at the SDK\'s linux/ or windows/ folder.')
        cmd += ["--paths", sdk, "--hidden-import", "fairino.Robot", "--collect-all", "fairino"]
    cmd.append(os.path.join(ROOT, "packaging", "launcher.py"))
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=ROOT)

    app_dir = os.path.join(ROOT, "dist", NAME)
    program = os.path.join(app_dir, NAME + (".exe" if os.name == "nt" else ""))
    for extra in ("README.md", "THIRD_PARTY_NOTICES.md"):
        shutil.copy(os.path.join(ROOT, extra), app_dir)
    if sys.platform.startswith("linux"):
        shutil.copy(os.path.join(ROOT, "packaging", "linux", "install.sh"), app_dir)

    if not args.skip_test:
        print("\nself-test of the built application:")
        env = dict(os.environ)
        if not env.get("DISPLAY") and os.name != "nt" and sys.platform != "darwin":
            env["QT_QPA_PLATFORM"] = "offscreen"          # build machine without a screen
        test = subprocess.run([program, "--self-test"], env=env, capture_output=True, text=True, timeout=300)
        print(test.stdout.strip() or "(no output: a windowed program on Windows prints nothing)")
        if test.returncode != 0:
            sys.exit("The built application FAILED its self-test - do not ship this build.\n" + test.stderr[-2000:])
        if args.sdk and "FAIRINO SDK: included" not in test.stdout and os.name != "nt":
            sys.exit("The FAIRINO SDK did not make it into the application - do not ship this build.")

    system = {"nt": "windows", "posix": "macos" if sys.platform == "darwin" else "linux"}[os.name]
    archive = shutil.make_archive(os.path.join(ROOT, "dist", f"{NAME}-{__version__}-{system}"),
                                  "zip" if os.name == "nt" else "gztar", os.path.join(ROOT, "dist"), NAME)
    print(f"\nbuilt  {app_dir}\npacked {archive}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
