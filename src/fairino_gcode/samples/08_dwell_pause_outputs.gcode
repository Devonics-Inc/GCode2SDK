; 08 - things that are not moves: tool on / off, dwell, program pause
; Expect: M3 switches the output on before the first move, M5 switches it off after the last cut,
; the robot waits 1.5 s and 0.5 s at two corners, and stops at M0 until Continue is pressed.
; (Set "Output for M3 / M5" in Set up run to see the output switch; without it M3 / M5 are skipped.)
G21 G90 G17
S3000 M3                 ; rapid speed 50 mm/s, tool on
G0 X0 Y0 Z5
G1 Z0 F300
G1 X30 Y0 F900
G4 P1.5                  ; wait 1.5 s
G1 X30 Y30
M0                       ; pause: waits for Continue
G1 X0 Y30
G4 S0.5                  ; wait 0.5 s
G1 X0 Y0
M5                       ; tool off
G0 Z5
M30
