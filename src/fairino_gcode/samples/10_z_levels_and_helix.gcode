; 10 - moves in Z: passes at several heights, a descending arc, a descending full turn
; Three passes of one line at Z4, Z2 and Z0, then a quarter arc that goes down 3 mm while it turns,
; then a full turn that goes down 6 mm.
; Expect: pure Z moves as MoveL; the descending arc as MoveC x1; the descending full turn as MoveC x2
; (a Circle instruction cannot change height). Note: an arc through three points is flat, so a
; descending arc is an approximation of a true helix.
G21 G90 G17
S6000
G0 X0 Y0 Z10
G1 Z4 F300
G1 X40 Y0 F1200
G1 Z2 F300
G1 X0 Y0 F1200
G1 Z0 F300
G1 X40 Y0 F1200
G0 Z10
G0 X60 Y0
G1 Z6 F300
G3 X70 Y10 I0 J10 Z3 F600
G0 Z10
G0 X80 Y30
G1 Z6 F300
G2 I-10 J0 Z0 F600
G0 Z10
M30
