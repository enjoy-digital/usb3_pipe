#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 1 RX datapath: transceiver to the link layer words, by clock domain.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(150, 15, 600, 135, title="sys clock domain", title_pos="tl"))
    d.add(Container(150, 175, 600, 135, title="rx clock domain", title_pos="tl"))
    xcvr  = d.add(Box(15, 230, 110, 60, label="Transceiver\nCDR, 8b/10b", fill=COLOR_HARDWARE))
    sub   = d.add(Box(165, 230, 100, 60, label="Error\nsubstitution"))
    conv  = d.add(Box(283, 230, 120, 60, label="Width\n20/40 to 32"))
    skp   = d.add(Box(421, 230, 100, 60, label="SKP\nremover", fill=COLOR_ACCENT))
    cdc   = d.add(Box(637, 230, 100, 60, label="CDC\nFIFO"))
    align = d.add(Box(637, 70, 100, 60, label="Word\naligner"))
    dscr  = d.add(Box(519, 70, 100, 60, label="Descrambler", fill=COLOR_ACCENT))
    fal   = d.add(Box(401, 70, 100, 60, label="Framing\naligner"))
    gate  = d.add(Box(283, 70, 100, 60, label="rx_ready\ngate"))
    swap  = d.add(Box(165, 70, 100, 60, label="Byte\nswap"))
    link  = d.add(Box(15, 70, 110, 60, label="Link layer", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((xcvr, "r"), (sub, "l"), kind="data"))
    d.add(Arrow((sub, "r"), (conv, "l"), kind="data"))
    d.add(Arrow((conv, "r"), (skp, "l"), kind="data"))
    d.add(Arrow((skp, "r"), (cdc, "l"), kind="data", label="data only", label_side="above"))
    d.add(Arrow((cdc, "t"), (align, "b"), kind="data"))
    d.add(Arrow((align, "l"), (dscr, "r"), kind="data"))
    d.add(Arrow((dscr, "l"), (fal, "r"), kind="data"))
    d.add(Arrow((fal, "l"), (gate, "r"), kind="data"))
    d.add(Arrow((gate, "l"), (swap, "r"), kind="data"))
    d.add(Arrow((swap, "l"), (link, "r"), kind="data"))
    return d
