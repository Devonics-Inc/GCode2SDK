"""Entry point of the standalone application (PyInstaller needs a plain script to start from)."""
import sys

from fairino_gcode.gcode_gui import main

if __name__ == "__main__":
    sys.exit(main())
