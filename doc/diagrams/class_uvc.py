#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# UVC function: PROBE/COMMIT control on the EP0 request handler, test pattern cut into payloads
# with a 12-byte header, sent on the video IN endpoint of the core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(150, 10, 390, 230, title="USB3UVCFunction", title_pos="bl"))
    ep0  = d.add(Box(20, 40, 100, 50, label="EP0\nhandler", fill=COLOR_ACCENT_LIGHT))
    ctrl = d.add(Box(370, 40, 150, 50, label="PROBE, COMMIT\ncontrol"))
    pat  = d.add(Box(170, 130, 150, 50, label="Test pattern\nYUY2"))
    pay  = d.add(Box(370, 130, 150, 50, label="Payload header\n12 bytes", fill=COLOR_ACCENT))
    ep   = d.add(Box(565, 130, 90, 50, label="Video\nIN EP", fill=COLOR_ACCENT_LIGHT))
    host = d.add(Box(680, 115, 65, 80, label="Host", fill=COLOR_HARDWARE))
    d.add(Arrow((ep0, "r"), (ctrl, "l"), kind="control"))
    d.add(Arrow((ctrl, "b"), (pay, "t"), kind="control", label="start, mode", label_pos=(453, 110),
        label_anchor="start"))
    d.add(Arrow((pat, "r"), (pay, "l"), kind="data"))
    d.add(Arrow((pay, "r"), (ep, "l"), kind="data"))
    d.add(Arrow((ep.x + ep.w, ep.cy), (host.x, ep.cy), kind="accent"))
    return d
