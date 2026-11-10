#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Endpoint converters: 32-bit function on 64/128-bit Bulk endpoints of a wide link layer.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(270, 10, 480, 230, title="Endpoints and function", title_pos="bl"))
    link  = d.add(Box(20,  40, 100, 160, label="Link layer\n64/128-bit", fill=COLOR_ACCENT_LIGHT))
    proto = d.add(Box(150, 40, 100, 160, label="Protocol\nlayer", fill=COLOR_ACCENT_LIGHT))
    ep_in  = d.add(Box(290, 40,  120, 60, label="Bulk IN\n64/128-bit"))
    ep_out = d.add(Box(290, 140, 120, 60, label="Bulk OUT\n64/128-bit"))
    up    = d.add(Box(450, 40,  120, 60, label="Up\nconverter",   fill=COLOR_ACCENT))
    down  = d.add(Box(450, 140, 120, 60, label="Down\nconverter", fill=COLOR_ACCENT))
    func  = d.add(Box(630, 40, 100, 160, label="32-bit\nfunction"))
    d.add(Arrow((link, "r"), (proto, "l"), kind="data", heads="both"))
    d.add(Arrow((ep_in, "l"), (proto.x + proto.w, ep_in.cy), kind="data"))
    d.add(Arrow((proto.x + proto.w, ep_out.cy), (ep_out, "l"), kind="data"))
    d.add(Arrow((up, "l"), (ep_in, "r"), kind="accent"))
    d.add(Arrow((ep_out, "r"), (down, "l"), kind="accent"))
    d.add(Arrow((func.x, up.cy), (up, "r"), kind="data"))
    d.add(Arrow((down, "r"), (func.x, down.cy), kind="data"))
    return d
