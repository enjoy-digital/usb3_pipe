#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# TS1 Ordered Set (16 symbols) and its Link Configuration symbol.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 200)
    W = 42
    cells = [(W, "COM", COLOR_ACCENT)]*4 + [(W, "D0.0", COLOR_FILL), (W, "Cfg", COLOR_ACCENT_LIGHT)] +             [(W, "D10.2", COLOR_SHADE)]*10
    bitfield(d, 44, 40, cells, size=12)
    d.add(Text(44 + 2*W, 25, "symbols 0-3", size=12))
    d.add(Text(44 + 4.5*W, 25, "4", size=12))
    d.add(Text(44 + 5.5*W, 25, "5", size=12))
    d.add(Text(44 + 11*W, 25, "symbols 6-15 (TS1 identifier)", size=12))
    # Link Configuration bits.
    bx = 210
    bits = [(60, "Reset", COLOR_ACCENT_LIGHT), (60, "rsvd", COLOR_SHADE), (60, "Loopback", COLOR_ACCENT_LIGHT),
            (100, "No scrambling", COLOR_ACCENT_LIGHT), (100, "rsvd", COLOR_SHADE)]
    bitfield(d, bx, 125, bits, size=12)
    for x, t in ((bx + 30, "bit 0"), (bx + 90, "1"), (bx + 150, "2"), (bx + 230, "3"), (bx + 330, "4-7")):
        d.add(Text(x, 182, t, size=12))
    d.add(Arrow((44 + 5.5*W, 80), (bx + 190, 125), via=[(44 + 5.5*W, 102), (bx + 190, 102)], kind="control"))
    return d
