; 01 - straight lines only: a 60 mm square
; Expect: G0 -> MoveJ x2, G1 -> MoveL x5. Corners are blended by 0.5 mm.
G21 G90 G17
S6000                    ; rapid (G0) speed: 6000 mm/min = 100 mm/s
G0 X0 Y0 Z5
G1 Z0 F300
G1 X60 Y0 F1200
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0
G0 Z5
M30
