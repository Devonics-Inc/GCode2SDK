"""Shared test fixtures. The GUI runs without a screen (Qt "offscreen") and talks to
fake_cnde_robot.py on a free local UDP port instead of a real controller."""
import os
import socket
import sys
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))       # run from the source tree without installing

import pytest  # noqa: E402
from matplotlib.backends.qt_compat import QtCore, QtWidgets  # noqa: E402

from fairino_gcode import fake_cnde_robot as fake  # noqa: E402

DEMO = os.path.join(ROOT, "src", "fairino_gcode", "samples", "demo_part.gcode")


@pytest.fixture(scope="session")
def qapp():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["test"])
    QtCore.QSettings.setDefaultFormat(QtCore.QSettings.Format.IniFormat)      # keep tests out of the
    QtCore.QSettings.setPath(QtCore.QSettings.Format.IniFormat,               # user's saved settings
                             QtCore.QSettings.Scope.UserScope, os.path.join(ROOT, ".pytest_cache", "qsettings"))
    return app


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(condition, timeout=5.0):
    """Run the Qt event loop until condition() is true. Returns False on timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        loop = QtCore.QEventLoop()
        QtCore.QTimer.singleShot(20, loop.quit)
        loop.exec()
    return bool(condition())


@pytest.fixture
def robot():
    """Start a pretend controller: robot(path=..., legacy=..., fault_after=...) -> (port, stop_event)."""
    stops = []

    def start(path=None, **kw):
        port, stop = free_port(), threading.Event()
        threading.Thread(target=fake.serve, args=(port, path or fake.circle_path()),
                         kwargs=dict(verbose=False, stop=stop, **kw), daemon=True).start()
        time.sleep(0.05)
        stops.append(stop)
        return port, stop

    yield start
    for stop in stops:
        stop.set()
