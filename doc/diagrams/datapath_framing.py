#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A Header Packet on the wire: Gen 1 words (framing at word boundaries) and Gen 2 Data Blocks
# (framing bytes at any position, Idle Symbols between packets).

from svgdiag import *
from datapath_helpers import strip

B = 20 # Pixels per byte.


def build():
    d = Diagram(CANVAS_WIDTH, 220)
    x0 = 20
    d.add(Text(x0, 25, "Gen 1: 32-bit words, K Symbols framing", anchor="start", italic=True))
    w = 7*B
    strip(d, x0, 40, [(w, "SHP SHP SHP EPF", COLOR_ACCENT), (w, "Header DW0", COLOR_FILL),
        (w, "Header DW1", COLOR_FILL), (w, "Header DW2", COLOR_FILL),
        (w//2, "CRC-16", COLOR_FILL), (w//2, "LCW", COLOR_FILL)])
    d.add(Text(x0, 92, "20 Symbols: framing, 12-byte header, CRC-16, Link Control Word",
        anchor="start", size=FONT_SIZE_SM))

    d.add(Text(x0, 135, "Gen 2: two Data Blocks, scrambled framing bytes", anchor="start", italic=True))
    blk_n = [(30, "H", COLOR_ACCENT_LIGHT), (3*B, "Idle", COLOR_SHADE), (4*B, "HPSTART", COLOR_ACCENT),
        (4*B, "DW0", COLOR_FILL), (4*B, "DW1", COLOR_FILL), (1*B, "", COLOR_FILL)] # Last cell: first byte of DW2.
    blk_m = [(30, "H", COLOR_ACCENT_LIGHT), (3*B, "DW2 end", COLOR_FILL), (2*B, "CRC", COLOR_FILL),
        (2*B, "LCW", COLOR_FILL), (9*B, "Idle (5Ah)", COLOR_SHADE)]
    strip(d, x0, 150, blk_n)
    strip(d, x0 + 30 + 16*B + 10, 150, blk_m)
    d.add(Text(x0 + 15 + 8*B, 202, "Data Block n", anchor="middle", size=FONT_SIZE_SM))
    d.add(Text(x0 + 30 + 16*B, 202, "DW2 start", anchor="end", size=FONT_SIZE_SM))
    d.add(Text(x0 + 30 + 16*B + 10 + 15 + 8*B, 202, "Data Block n+1", anchor="middle",
        size=FONT_SIZE_SM))
    return d
