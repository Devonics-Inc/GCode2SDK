; demo_part.gcode - 100 x 60 mm plate: rounded outline, a round hole, a slot
; units mm, absolute, XY plane. Z0 = surface, Z5 = safe height.
G21 G90 G17
S6000 M3                 ; S = rapid (G0) speed in mm/min, M3 = tool on

; --- outline, counter-clockwise, 8 mm corner radius
G0 X8 Y0 Z5
G1 Z0 F300
G1 X92 Y0 F1200
G3 X100 Y8 I0 J8
G1 X100 Y52
G3 X92 Y60 I-8 J0
G1 X8 Y60
G3 X0 Y52 I0 J-8
G1 X0 Y8
G3 X8 Y0 I8 J0
G0 Z5

; --- round hole, dia 20, centre (30, 30): one full circle, clockwise
G0 X40 Y30
G1 Z0 F300
G2 I-10 J0 F900
G0 Z5

; --- slot from (60, 22) to (80, 38), 8 mm wide (R-format arcs)
G0 X63.2 Y19.6
G1 Z0 F300
G1 X83.2 Y35.6 F900
G3 X76.8 Y40.4 R4
G1 X56.8 Y24.4
G3 X63.2 Y19.6 R4
G4 P0.5                  ; dwell 0.5 s
G0 Z5

M5
G0 X0 Y0
M30
