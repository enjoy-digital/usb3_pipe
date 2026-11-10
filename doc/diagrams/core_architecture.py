#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Device core architecture: PHY, link layer, protocol layer, endpoints, descriptors, user logic.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(150, 15, 450, 300, title="usb3_core", title_pos="bl"))
    top, h = 40, 230
    phy   = d.add(Box(20,  top, 100, h, label="usb3_pipe\nPHY", fill=COLOR_HARDWARE))
    link  = d.add(Box(170, top, 100, h, label="Link\nlayer",    fill=COLOR_ACCENT))
    proto = d.add(Box(300, top, 100, h, label="Protocol\nlayer", fill=COLOR_ACCENT))
    names = [("Control\nEP0", COLOR_ACCENT_LIGHT), ("Bulk IN", COLOR_FILL), ("Bulk OUT", COLOR_FILL),
        ("Iso IN", COLOR_FILL)]
    eps = []
    for i, (name, fill) in enumerate(names):
        yc = top + 30 + i*57
        eps.append(d.add(Box(430, yc - 22, 140, 44, label=name, fill=fill)))
    d.add(Arrow((phy, "r"), (link, "l"), kind="data", heads="both"))
    d.add(Arrow((link, "r"), (proto, "l"), kind="accent", heads="both"))
    for ep in eps:
        yc = ep.cy
        d.add(Arrow((proto.x + proto.w, yc), (ep.x, yc), kind="data", heads="both"))
    desc = d.add(Box(640, eps[0].y, 100, 44, label="Descriptors\nROM"))
    user = d.add(Box(640, eps[1].y, 100, eps[3].y + 44 - eps[1].y, label="User logic\nor class"))
    d.add(Arrow((eps[0], "r"), (desc, "l"), kind="control"))
    for ep in eps[1:]:
        d.add(Arrow((ep.x + ep.w, ep.cy), (user.x, ep.cy), kind="data", heads="both"))
    return d
