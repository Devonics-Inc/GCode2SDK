; 09 - feed rates and rapid speeds, including ones above the limit
; Sides of a 40 mm square at 2, 10, 50 and 500 mm/s. 500 mm/s is above the tool's limit (250 mm/s).
; Expect: a warning that 2 moves are capped; the third side is clearly faster than the second;
; the fourth side and the last rapid run at the limit, not above it.
G21 G90 G17
S12000                   ; rapid speed 200 mm/s
G0 X0 Y0 Z5
G1 Z0 F120               ; 2 mm/s
G1 X40 Y0 F600           ; 10 mm/s
G1 X40 Y40 F3000         ; 50 mm/s
G1 X0 Y40 F30000         ; 500 mm/s: capped
G1 X0 Y0 F1800           ; 30 mm/s
S60000                   ; rapid 1000 mm/s: capped
G0 Z5
M30
