; 04 - full circles
; A clockwise circle R20 written without an end point, and a counter-clockwise circle R10 written
; with its end point equal to its start.
; Expect: Circle x2 (not MoveC). The robot ends each circle where it started it.
G21 G90 G17
S6000
G0 X50 Y30 Z5
G1 Z0 F300
G2 I-20 J0 F900          ; centre (30, 30), clockwise
G0 Z5
G0 X30 Y40
G1 Z0 F300
G3 X30 Y40 I0 J-10 F600  ; centre (30, 30), counter-clockwise
G0 Z5
M30
