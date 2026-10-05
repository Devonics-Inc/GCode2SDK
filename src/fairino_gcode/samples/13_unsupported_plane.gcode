; 13 - something the tool must refuse: an arc in the XZ plane (G18)
; Only arcs in the XY plane (G17) can be converted, as in the WebApp.
; Expect: the file does not load; the message names line 6.
G21 G90
G0 X0 Y0 Z5
G18
G2 X10 Z5 I5 K0 F600
M30
