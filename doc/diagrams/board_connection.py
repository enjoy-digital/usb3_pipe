#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Board connection: host, cable, connector or adapter, one transceiver lane, reference clock.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(160, 20, 580, 260, title="FPGA board", title_pos="br"))
    host = d.add(Box(20, 100, 90, 80, label="USB 3\nhost port", fill=COLOR_HARDWARE))
    conn = d.add(Box(190, 100, 100, 80, label="Connector\nor adapter", fill=COLOR_HARDWARE))
    osc  = d.add(Box(330, 200, 110, 44, label="Oscillator"))
    gt   = d.add(Box(330, 80, 110, 80, label="Transceiver\nlane", fill=COLOR_ACCENT))
    phy  = d.add(Box(490, 80, 100, 80, label="PHY", fill=COLOR_ACCENT_LIGHT))
    core = d.add(Box(620, 80, 100, 80, label="Device\ncore", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((host, "r", 0.3), (conn, "l", 0.3), kind="accent"))
    d.add(Arrow((conn, "l", 0.7), (host, "r", 0.7), kind="accent"))
    d.add(Text(140, 112, "cable", size=FONT_SIZE_SM))
    d.add(Arrow((conn, "r", 0.25), (gt, "l", 0.25), kind="accent"))
    d.add(Arrow((gt, "l", 0.75), (conn, "r", 0.75), kind="accent"))
    d.add(Arrow((osc, "t"), (gt, "b"), kind="control", label="reference clock", label_side="right"))
    d.add(Arrow((gt, "r"), (phy, "l"), kind="data", heads="both"))
    d.add(Arrow((phy, "r"), (core, "l"), kind="data", heads="both"))
    return d
