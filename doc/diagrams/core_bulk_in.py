#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Bulk IN endpoint: user stream, packet writer, packet slots ring, DPP reader, host requests.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(130, 10, 470, 230, title="USB3BulkIN", title_pos="bl"))
    user  = d.add(Box(20, 50, 80, 60, label="User\nlogic"))
    wr    = d.add(Box(160, 50, 110, 60, label="Packet\nwriter"))
    slots = d.add(Box(300, 50, 130, 60, label="Packet slots\n1 KiB each", fill=COLOR_ACCENT, stack=2))
    rd    = d.add(Box(465, 50, 110, 60, label="DPP\nreader"))
    fc    = d.add(Box(300, 150, 130, 50, label="Host requests\nACK, NumP"))
    proto = d.add(Box(640, 30, 100, 190, label="Protocol\nlayer", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((user, "r"), (wr, "l"), kind="data"))
    d.add(Arrow((wr, "r"), (slots, "l"), kind="data"))
    d.add(Arrow((slots.x + slots.w, rd.cy), (rd, "l"), kind="data"))
    d.add(Arrow((rd, "r"), (proto.x, rd.cy), kind="accent"))
    d.add(Arrow((proto.x, fc.cy), (fc, "r"), kind="control", label="ACK TP", label_pos=(535, fc.cy - 10)))
    d.add(Arrow((fc, "t"), (slots, "b"), kind="control"))
    d.add(Arrow((fc, "l"), (wr, "b"), kind="feedback", label="slots freed", label_pos=(255, 190)))
    return d
