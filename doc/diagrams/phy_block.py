#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PHY block diagram: transceiver, USB3SerDes, the USB3PHY blocks (PIPE units and LTSSM) and the
# link interface to the link layer.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(270, 20, 375, 290, title="USB3PHY"))
    xcvr   = d.add(Box(20, 140, 95, 60, label="FPGA\ntransceiver", fill=COLOR_HARDWARE))
    serdes = d.add(Box(140, 100, 105, 140, label="USB3SerDes\n8b/10b, SKP,\nalignment", fill=COLOR_ACCENT_LIGHT))
    lfps   = d.add(Box(290, 50, 125, 45, label="LFPS unit"))
    ts     = d.add(Box(290, 115, 125, 45, label="TS unit"))
    scr    = d.add(Box(290, 180, 125, 50, label="Scrambler\nDescrambler"))
    idle   = d.add(Box(290, 248, 125, 42, label="Idle handshake"))
    ltssm  = d.add(Box(460, 60, 105, 110, label="LTSSM", fill=COLOR_ACCENT))
    link   = d.add(Box(665, 60, 80, 170, label="Link\nlayer", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((xcvr, "r"), (serdes, "l"), kind="data", heads="both"))
    d.add(Arrow((serdes, "t"), (lfps, "l"), kind="control", heads="both"))
    d.add(Arrow((serdes, "r", (137.5 - 100)/140), (ts, "l"), kind="data", heads="both"))
    d.add(Arrow((serdes, "r", (205 - 100)/140), (scr, "l"), kind="data", heads="both"))
    d.add(Arrow((lfps, "r"), (460, 72.5), kind="control", heads="both"))
    d.add(Arrow((ts, "r"), (460, 137.5), kind="control", heads="both"))
    d.add(Arrow((idle, "r"), (ltssm, "b"), kind="control", heads="both"))
    d.add(Arrow((scr, "r"), (665, 205), kind="accent", heads="both", label="sink / source",
        label_pos=(598, 194)))
    d.add(Arrow((ltssm, "r"), (665, 115), kind="control", heads="both", label="status, PM",
        label_pos=(605, 104)))
    return d
