# Sample G-code files

Each file exercises one feature. They are small (most fit in 100 x 60 mm), use Z0 as the surface and
Z5 as the safe height, and can be run in the simulator:

    fairino-gcode --simulate src/fairino_gcode/samples/02_arcs_ij.gcode

`tests/test_samples.py` checks what every file is converted to and runs each one on the simulated
robot. The "what to look for" column is for a run on a real robot, where the simulator cannot help.

| File | What it tests | Converted to | What to look for on the robot |
|---|---|---|---|
| `01_lines_square` | G0 and G1 only | MoveJ 2, MoveL 5 | Square corners, 60 mm sides, no stop at the corners |
| `02_arcs_ij` | Arcs by centre (I, J), G2 and G3 | MoveC 8, MoveL 10, MoveJ 4 | Corners round and tangent to the lines; inner outline runs the other way round |
| `03_arcs_r` | Arcs by radius, incl. 180° and a 247° arc (negative R) | MoveC 3 | The long arc goes the long way round, as one smooth move |
| `04_full_circles` | Full circles, with and without an end point | Circle 2 | Each circle closes exactly on its start; R20 clockwise, R10 counter-clockwise |
| `05_modal_and_incremental` | Lines without a G word; G91 | MoveL 10 | Two equal 20 mm squares side by side |
| `06_inches` | G20 | MoveL 5 | 50.8 x 25.4 mm rectangle (measure it) |
| `07_spline_small_segments` | 200 short segments | MoveL 201 | Smooth, continuous motion; no stuttering between points |
| `08_dwell_pause_outputs` | M3 / M5, G4, M0 | 3 M-codes, 2 waits | Output on before the first cut and off after the last; waits of 1.5 s and 0.5 s; stops at M0 until Continue |
| `09_speeds` | F and S, incl. values above the limit | MoveL 5, warning "2 moves ... capped" | Sides at 2, 10, 50 mm/s are clearly different; nothing faster than 250 mm/s (at 100 % global speed) |
| `10_z_levels_and_helix` | Pure Z moves, arc with a Z change | MoveC 3, no Circle | Three passes at Z4, Z2, Z0; the descending arcs are flat arcs (about 0.3 mm off a true helix) |
| `11_long_part` | A 400 x 80 mm part | MoveC 4, MoveL 5 | Reach check and automatic placement: usually only fits turned by 90° |
| `12_formatting` | Line numbers, lower case, no spaces, comments, ignored codes, a tiny arc | MoveL 6, 6 warnings | A plain 20 mm square |
| `13_unsupported_plane` | An XZ-plane arc (G18) | refused: "line 6: only G17 ..." | Does not load |
| `demo_part` | A complete small part | MoveL 9, MoveC 6, Circle 1, MoveJ 7 | Used by `--self-test` |

Suggested order on a real robot: 01, 05, 02, 03, 04, 07, 08, 09, 10, then 11. Run each one first in
free air at 20 % global speed.
