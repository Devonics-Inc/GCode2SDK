; 03 - arcs given by radius (R)
; A quarter arc (R20), a half circle (R10), and a long arc of 247 deg (R-12: negative R = the long way round).
; Expect: MoveC x3, each a single instruction even for the 247 deg arc.
G21 G90 G17
S6000
G0 X20 Y0 Z5
G1 Z0 F300
G3 X0 Y20 R20 F900       ; 90 deg, counter-clockwise, centre (0, 0)
G1 X0 Y40
G2 X20 Y40 R10           ; 180 deg, clockwise, centre (10, 40)
G1 X40 Y40
G3 X40 Y20 R-12          ; 247 deg, counter-clockwise, the long way round
G1 X20 Y0
G0 Z5
M30
