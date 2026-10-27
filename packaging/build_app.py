#!/usr/bin/env python3
"""
Build the standalone application (no Python needed on the target computer) with PyInstaller.

    python packaging/build_app.py --sdk /path/to/fairino-python-sdk/linux      # or .../windows
    python packaging/build_app.py --sdk ... --version 0.2.0                   # release 0.2.0

--sdk is the folder that CONTAINS the SDK's "fairino" package. It is copied into the application so
that it can command a real robot. Without it (--no-sdk) the result can only simulate.

The result is the folder dist/fairino-gcode/ (start it with the fairino-gcode program inside) and an
archive of it next to it. The build ends by starting the application once with --self-test, which runs
the demo part on the simulated robot: a build that passes is complete.

--version sets the release number first: it is written into src/fairino_gcode/__init__.py (commit that
change), so the application, its self-test, the archive name and the Windows installer all carry it.
Without --version the number already in that file is used.

PyInstaller does not cross-compile: build on Linux for Linux and on Windows for Windows.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = "fairino-gcode"
PACKAGE = os.path.join(ROOT, "src", "fairino_gcode")
VERSION_FILE = os.path.join(PACKAGE, "__init__.py")
VERSION_RE = re.compile(r'^__version__ = "([^"]*)"', re.MULTILINE)


def set_version(version: str) -> str:
    """Write `version` into src/fairino_gcode/__init__.py. Returns the number that was there before."""
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:[.-]?(?:a|b|rc|dev|post)\d*)?", version):
        sys.exit(f"--version {version!r}: use a number like 0.2.0 (or 0.2.0rc1).")
    with open(VERSION_FILE, encoding="utf-8") as f:
        text = f.read()
    found = VERSION_RE.search(text)
    if found is None:
        sys.exit(f"No __version__ line in {VERSION_FILE}.")
    with open(VERSION_FILE, "w", encoding="utf-8", newline="\n") as f:
        f.write(VERSION_RE.sub(f'__version__ = "{version}"', text, count=1))
    return found.group(1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--sdk", help='folder that contains the FAIRINO SDK\'s "fairino" package')
    where.add_argument("--no-sdk", action="store_true", help="build a simulation-only application")
    ap.add_argument("--skip-test", action="store_true", help="do not run the built application's self-test")
    ap.add_argument("--version", help="release number to build, e.g. 0.2.0 "
                                      "(written into src/fairino_gcode/__init__.py)")
    args = ap.parse_args()

    if args.version:
        before = set_version(args.version)
        if before != args.version:
            print(f"version {before} -> {args.version} in {os.path.relpath(VERSION_FILE, ROOT)} (commit this change)")
    sys.path.insert(0, os.path.join(ROOT, "src"))
    from fairino_gcode import __version__
    print(f"building {NAME} {__version__}")

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
    if os.name == "nt":
        print(f"installer: iscc /DAppVersion={__version__} packaging\\windows\\installer.iss")
    return 0


if __name__ == "__main__":
    sys.exit(main())
