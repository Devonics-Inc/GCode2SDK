%
O1001 (12 - the ways CAM programs write the same thing)
(Line numbers, lower case, no spaces, signs, leading and trailing dots, both comment styles,)
(set-up codes the tool does not need, and an arc too small to be an arc.)
(Expect: a 20 mm square; warnings for the codes that are ignored; the 0.14 mm arc sent as a line.)
N10 G21 G90 G17 G54 G40 G49 G80
N20 T1 M6
N30 S6000
N40 g0 x0 y0 z5
N50 G1Z0.F300
N60 G01 X+20. Y.0 F1200
N70 G1 X20 Y19.8 ; semicolon comment
N80 G2 X20.2 Y20 I0.1 J0.1 (radius 0.14 mm)
N90 G1 X0 Y20
N100 G1 X0 Y0
N110 G28
N120 G0 Z5
N130 M30
%
