; 05 - modal motion and incremental coordinates
; First square: after the first G1 the lines carry no G word (the motion mode stays G1).
; Second square: drawn with G91 (each move is relative to the previous point).
; Expect: two 20 mm squares side by side, X 0..30 / 40..60, Y 0..20. MoveL x12.
G21 G90 G17
S6000
G0 X0 Y0 Z5
G1 Z0 F300
G1 X30 Y0 F1200
X30 Y20
X0 Y20
Y0
G0 Z5
G0 X40 Y0
G1 Z0 F300
G91
G1 X20 F1200
Y20
X-20
Y-20
G90
G0 Z5
M30
