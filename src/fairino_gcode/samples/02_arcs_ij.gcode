; 02 - arcs given by centre (I, J), both directions
; Outer outline 80 x 50 mm, 10 mm corners, counter-clockwise (G3).
; Inner outline 40 x 20 mm, 5 mm corners, clockwise (G2).
; Expect: MoveC x8 (one per corner), MoveL x10, MoveJ x4.
G21 G90 G17
S6000
G0 X10 Y0 Z5
G1 Z0 F300
G1 X70 Y0 F1200
G3 X80 Y10 I0 J10
G1 X80 Y40
G3 X70 Y50 I-10 J0
G1 X10 Y50
G3 X0 Y40 I0 J-10
G1 X0 Y10
G3 X10 Y0 I10 J0
G0 Z5
G0 X20 Y20
G1 Z0 F300
G1 X20 Y30 F900
G2 X25 Y35 I5 J0
G1 X55 Y35
G2 X60 Y30 I0 J-5
G1 X60 Y20
G2 X55 Y15 I-5 J0
G1 X25 Y15
G2 X20 Y20 I0 J5
G0 Z5
M30
