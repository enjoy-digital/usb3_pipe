#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Transceiver clocking: reference clock, PLL, tx/rx clock domains and the sys clock domain.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    ref  = d.add(Box(15, 45, 110, 60, label="Reference\nclock", fill=COLOR_HARDWARE))
    d.add(Container(150, 15, 330, 230, title="Transceiver", title_pos="br"))
    pll  = d.add(Box(170, 45, 120, 60, label="PLL"))
    ser  = d.add(Box(330, 45, 120, 60, label="Serializer"))
    cdr  = d.add(Box(330, 145, 120, 60, label="CDR"))
    txd  = d.add(Box(510, 45, 115, 60, label="TX datapath\ntx clock", fill=COLOR_ACCENT))
    rxd  = d.add(Box(510, 145, 115, 60, label="RX datapath\nrx clock", fill=COLOR_ACCENT))
    sys  = d.add(Box(655, 45, 90, 160, label="PIPE\nLTSSM\nlink layer\nsys clock",
        fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((ref, "r"), (pll, "l"), kind="control"))
    d.add(Arrow((pll, "r"), (ser, "l"), kind="control"))
    d.add(Arrow((pll, "b"), (cdr, "l"), kind="control"))
    d.add(Arrow((ser, "r"), (txd, "l"), kind="control"))
    d.add(Arrow((cdr, "r"), (rxd, "l"), kind="control"))
    d.add(Arrow((txd, "r"), (sys, "l", 30/160), kind="feedback", heads="both"))
    d.add(Arrow((rxd, "r"), (sys, "l", 130/160), kind="feedback", heads="both"))
    return d
