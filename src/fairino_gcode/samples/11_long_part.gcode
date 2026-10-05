; 11 - a long part: 400 x 80 mm outline with 10 mm corners
; For the reachability check and automatic placement. Laid out pointing at the robot it cannot be
; done; turned sideways it can.
; Expect: with "Automatic" placement the drawing is usually turned by 90 deg. Placed by hand so that
; it points at the robot's base, the check marks the unreachable part with red crosses.
G21 G90 G17
S6000
G0 X10 Y0 Z5
G1 Z0 F300
G1 X390 Y0 F1800
G3 X400 Y10 I0 J10
G1 X400 Y70
G3 X390 Y80 I-10 J0
G1 X10 Y80
G3 X0 Y70 I0 J-10
G1 X0 Y10
G3 X10 Y0 I10 J0
G0 Z5
M30
