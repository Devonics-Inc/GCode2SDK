; 06 - inch units (G20)
; A 2 x 1 inch rectangle; feed 40 inch/min.
; Expect: 50.8 x 25.4 mm, safe height 5.08 mm, feed 16.9 mm/s.
G20 G90 G17
S6000
G0 X0 Y0 Z0.2
G1 Z0 F12
G1 X2 Y0 F40
G1 X2 Y1
G1 X0 Y1
G1 X0 Y0
G0 Z0.2
M30
