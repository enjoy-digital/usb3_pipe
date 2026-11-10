#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Polling.LFPS: bursts of a low frequency square wave separated by electrical idle.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 180)
    y0, y1 = 70, 40   # electrical idle level, burst top
    x  = 40
    burst, repeat = 70, 230
    for i in range(3):
        # Burst: square wave drawn as a dense run of edges.
        pts = [(x, y0)]
        for k in range(8):
            xa = x + k*burst/8
            pts += [(xa, y1), (xa + burst/16, y1), (xa + burst/16, y0 + 30), (xa + burst/8, y0 + 30)]
        pts += [(x + burst, y0)]
        d.add(Bus(pts, color=COLOR_CYAN))
        d.add(Bus([(x + burst, y0), (x + repeat, y0)], dash="4,3"))
        x += repeat
    d.add(Arrow((40, 125), (110, 125), heads="both", label="tBurst 1 µs", label_pos=(120, 129),
        label_anchor="start"))
    d.add(Arrow((40, 155), (270, 155), heads="both", label="tRepeat 10 µs", label_pos=(280, 159),
        label_anchor="start"))
    d.add(Text(190, 58, "electrical idle", size=12, color="#444444"))
    return d
