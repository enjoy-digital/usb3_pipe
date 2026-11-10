#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# U1 entry and exit: LGO_U1 / LAU / LPMA link commands, then the U1 exit LFPS handshake and Recovery.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    dev  = d.add(Box(160, 20, 120, 40, label="Device", fill=COLOR_ACCENT))
    host = d.add(Box(480, 20, 120, 40, label="Host", fill=COLOR_HARDWARE))
    xd, xh = dev.cx, host.cx
    d.add(Bus([(xd, 60), (xd, 360)], dash="4,4"))
    d.add(Bus([(xh, 60), (xh, 360)], dash="4,4"))
    msgs = [
        (90,  "h", "LGO_U1"),
        (130, "d", "LAU"),
        (170, "h", "LPMA"),
        (250, "d", "U1 exit LFPS"),
        (290, "h", "U1 exit LFPS"),
    ]
    for y, src, label in msgs:
        a, b = ((xh, y), (xd, y)) if src == "h" else ((xd, y), (xh, y))
        kind = "accent" if "LFPS" in label else "control"
        d.add(Arrow(a, b, kind=kind, label=label, label_side="above"))
    for y, state in [(80, "U0"), (210, "U1"), (330, "Recovery, U0")]:
        d.add(Text(30, y, state, anchor="start", italic=True))
        d.add(Text(730, y, state, anchor="end", italic=True))
    return d
