#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# USB 3 layers on both sides of the link, and the modules implementing them on the device side.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(20, 10, 230, 280, title="Host", title_pos="bl"))
    d.add(Container(510, 10, 230, 280, title="Device (FPGA)", title_pos="br"))
    rows = [("Protocol layer", "usb3_core\nprotocol.py", "Transactions"),
            ("Link layer",     "usb3_core\nlink.py",     "Packets, link commands"),
            ("Physical layer", "usb3_pipe",             "Symbols, LFPS")]
    y = 30
    for name, module, what in rows:
        h = d.add(Box(45, y, 180, 50, label=name, fill=COLOR_HARDWARE))
        v = d.add(Box(535, y, 180, 50, label=name, fill=COLOR_ACCENT))
        d.add(Arrow((h, "r"), (v, "l"), kind="feedback", heads="both", label=what, label_size=13))
        y += 75
    d.add(Text(380, 268, "cable: 5 or 10 Gbps per direction", size=13))
    return d
