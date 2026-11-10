#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Full-system simulation (sim.py): host model and device, each with a PHY and a transceiver model,
# in their own clock domains.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    host = d.add(Container(20, 20, 300, 290, title="Host side", title_pos="tl"))
    dev  = d.add(Container(440, 20, 300, 290, title="Device side", title_pos="tr"))
    hm   = d.add(Box(60, 70, 220, 46, label="Host model\n(script, link layer)", fill=COLOR_ACCENT_LIGHT))
    hphy = d.add(Box(60, 150, 220, 40, label="USB3PHY"))
    hser = d.add(Box(60, 225, 220, 40, label="Transceiver model", fill=COLOR_HARDWARE))
    core = d.add(Box(480, 70, 220, 46, label="Device core\n(Bulk loopback)", fill=COLOR_ACCENT))
    dphy = d.add(Box(480, 150, 220, 40, label="USB3PHY", fill=COLOR_ACCENT))
    dser = d.add(Box(480, 225, 220, 40, label="Transceiver model", fill=COLOR_HARDWARE))
    d.add(Arrow((hm, "b"), (hphy, "t"), kind="data", heads="both"))
    d.add(Arrow((hphy, "b"), (hser, "t"), kind="data", heads="both"))
    d.add(Arrow((core, "b"), (dphy, "t"), kind="data", heads="both"))
    d.add(Arrow((dphy, "b"), (dser, "t"), kind="data", heads="both"))
    d.add(Arrow((hser, "r", 0.3), (dser, "l", 0.3), kind="accent", label="symbols", label_side="above"))
    d.add(Arrow((dser, "l", 0.7), (hser, "r", 0.7), kind="accent"))
    d.add(Text(380, 290, "ppm, SSC, word shift", size=FONT_SIZE_SM))
    return d
