# Third-party software in FAIRINO G-code Runner

The standalone application contains the following components. This list is a starting point for a
release review, not legal advice: have the licence terms checked before distributing to customers.

| Component | Used for | Licence |
|---|---|---|
| Python | runtime | PSF License |
| PySide6 / Qt 6 | the window | LGPL v3 (Qt also offers commercial licences) |
| matplotlib | the 3D view | matplotlib licence (BSD-style) |
| NumPy | geometry | BSD 3-Clause |
| PyInstaller bootloader | starting the application | GPL v2 with an exception that allows bundling any program |
| FAIRINO Python SDK | commands to the robot | FAIRINO's own terms |

Notes for the review

* LGPL (Qt / PySide6): the application is built as a folder, with Qt as separate library files that a
  user can replace. Keep it that way (do not switch to a single-file build) and ship the LGPL text and
  a pointer to the Qt and PySide6 sources with the product.
* matplotlib, NumPy and others pulled in by them ship their licence files inside their packages; they
  are copied into the application's `_internal` folder by the build.
