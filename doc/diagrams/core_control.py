#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Control endpoint: SETUP decoding, requests table (descriptors ROM), standard requests state and
# the optional user request handler (outside the core).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(140, 10, 480, 210, title="USB3EndpointControl", title_pos="br"))
    proto = d.add(Box(20, 40, 90, 260, label="Protocol\nlayer", fill=COLOR_ACCENT_LIGHT))
    dec   = d.add(Box(165, 75, 110, 60, label="SETUP\ndecoder", fill=COLOR_ACCENT))
    tab   = d.add(Box(330, 40, 140, 50, label="Requests table\nROM"))
    std   = d.add(Box(330, 115, 140, 50, label="Standard\nrequests"))
    state = d.add(Box(500, 115, 100, 50, label="Device\nstate"))
    usr   = d.add(Box(330, 250, 140, 50, label="User handler"))
    tx    = d.add(Box(650, 40, 90, 260, label="IN data\nor STALL", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((proto.x + proto.w, dec.cy), (dec, "l"), kind="data"))
    d.add(Arrow((dec, "r"), (tab, "l"), kind="control"))
    d.add(Arrow((dec, "r"), (std, "l"), kind="control"))
    d.add(Arrow((dec, "b"), (usr, "l"), kind="control", label="class, vendor",
        label_pos=(232, 262), label_anchor="start"))
    d.add(Arrow((tab.x + tab.w, tab.cy), (tx.x, tab.cy), kind="data", label="descriptors",
        label_side="above"))
    d.add(Arrow((std, "r"), (state, "l"), kind="control"))
    d.add(Arrow((usr.x + usr.w, usr.cy), (tx.x, usr.cy), kind="data", label="req_in, req_stall",
        label_side="above"))
    return d
