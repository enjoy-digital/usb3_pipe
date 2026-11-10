#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The USB 3 cable: one SuperSpeed differential pair per direction, AC coupled, next to the USB 2 pair.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 240)
    host = d.add(Container(20, 15, 200, 210, title="Host port", title_pos="bl"))
    dev  = d.add(Container(540, 15, 200, 210, title="Device port", title_pos="br"))
    htx = d.add(Box(60, 35, 120, 40, label="TX", fill=COLOR_HARDWARE))
    hrx = d.add(Box(60, 95, 120, 40, label="RX", fill=COLOR_HARDWARE))
    h2  = d.add(Box(60, 155, 120, 34, label="USB 2", size=13))
    drx = d.add(Box(580, 35, 120, 40, label="RX", fill=COLOR_ACCENT))
    dtx = d.add(Box(580, 95, 120, 40, label="TX", fill=COLOR_ACCENT))
    d2  = d.add(Box(580, 155, 120, 34, label="USB 2", size=13))
    cap1 = d.add(Box(245, 45, 30, 20, label="", fill="#ffffff", shadow=False))
    cap2 = d.add(Box(485, 105, 30, 20, label="", fill="#ffffff", shadow=False))
    d.add(Arrow((htx, "r"), (drx, "l"), kind="data", label="SSTX+/SSTX- (host)", label_size=13))
    d.add(Arrow((dtx, "l"), (hrx, "r"), kind="data", label="SSTX+/SSTX- (device)", label_size=13))
    d.add(Arrow((h2, "r"), (d2, "l"), kind="control", heads="both", label="D+/D- (not used)", label_size=13))
    d.add(Text(260, 30, "AC coupling", size=12, color="#444444"))
    d.add(Text(500, 90, "AC coupling", size=12, color="#444444"))
    return d
