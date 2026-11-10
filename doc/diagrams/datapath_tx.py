#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 1 TX datapath: link layer words to the transceiver, by clock domain.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(150, 15, 600, 135, title="sys clock domain", title_pos="tl"))
    d.add(Container(150, 175, 600, 135, title="tx clock domain", title_pos="tl"))
    link  = d.add(Box(15, 70, 110, 60, label="Link layer", fill=COLOR_ACCENT_LIGHT))
    swap  = d.add(Box(165, 70, 100, 60, label="Byte\nswap"))
    idle  = d.add(Box(283, 70, 100, 60, label="Logical\nIdle"))
    scr   = d.add(Box(401, 70, 100, 60, label="Scrambler", fill=COLOR_ACCENT))
    ts    = d.add(Box(519, 70, 100, 60, label="TS1/TS2\nselect"))
    skp   = d.add(Box(637, 70, 100, 60, label="SKP\ninserter", fill=COLOR_ACCENT))
    cdc   = d.add(Box(637, 230, 100, 60, label="CDC\nFIFO"))
    conv  = d.add(Box(480, 230, 120, 60, label="Width\n32 to 20/40"))
    xcvr  = d.add(Box(15, 230, 110, 60, label="Transceiver\n8b/10b, serial", fill=COLOR_HARDWARE))
    d.add(Arrow((link, "r"), (swap, "l"), kind="data"))
    d.add(Arrow((swap, "r"), (idle, "l"), kind="data"))
    d.add(Arrow((idle, "r"), (scr, "l"), kind="data"))
    d.add(Arrow((scr, "r"), (ts, "l"), kind="data"))
    d.add(Arrow((ts, "r"), (skp, "l"), kind="data"))
    d.add(Arrow((skp, "b"), (cdc, "t"), kind="data"))
    d.add(Arrow((cdc, "l"), (conv, "r"), kind="data"))
    d.add(Arrow((conv, "l"), (xcvr, "r"), kind="data", label="2 or 4 symbols", label_side="above"))
    return d
