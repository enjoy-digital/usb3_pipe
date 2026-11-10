#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Composite device of the composite example: a UVC function and a CDC-ACM function, each grouped by
# an Interface Association Descriptor, sharing one device core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    d.add(Container(20, 10, 350, 240, title="UVC function", title_pos="bl"))
    d.add(Container(390, 10, 350, 240, title="CDC-ACM function", title_pos="bl"))
    vc  = d.add(Box(40, 40, 150, 60, label="Interface 0\nVideoControl", fill=COLOR_ACCENT_LIGHT))
    vs  = d.add(Box(40, 120, 150, 60, label="Interface 1\nVideoStreaming", fill=COLOR_ACCENT_LIGHT))
    ep1 = d.add(Box(230, 120, 120, 60, label="EP1 Bulk IN\nMaxBurst 16", fill=COLOR_ACCENT))
    cm  = d.add(Box(410, 40, 150, 60, label="Interface 2\nCommunication", fill=COLOR_ACCENT_LIGHT))
    da  = d.add(Box(410, 120, 150, 90, label="Interface 3\nData", fill=COLOR_ACCENT_LIGHT))
    ep4 = d.add(Box(600, 40, 120, 60, label="EP4\nInterrupt IN", fill=COLOR_ACCENT))
    ep2 = d.add(Box(600, 120, 120, 40, label="EP2 Bulk IN", fill=COLOR_ACCENT))
    ep3 = d.add(Box(600, 170, 120, 40, label="EP3 Bulk OUT", fill=COLOR_ACCENT))
    d.add(Arrow((vs, "r"), (ep1, "l"), kind="control"))
    d.add(Arrow((cm, "r"), (ep4, "l"), kind="control"))
    d.add(Arrow((da.x + da.w, ep2.cy), (ep2, "l"), kind="control"))
    d.add(Arrow((da.x + da.w, ep3.cy), (ep3, "l"), kind="control"))
    return d
