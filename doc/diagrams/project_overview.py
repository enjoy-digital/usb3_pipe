#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Project overview: the host, the cable, the FPGA transceiver, the PHY and the device core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(160, 10, 585, 230, title="FPGA", title_pos="br"))
    d.add(Container(300, 25, 140, 175, title="usb3_pipe", title_pos="bl", title_size=15, fill="#f6f6f6"))
    d.add(Container(460, 25, 140, 175, title="usb3_core", title_pos="bl", title_size=15, fill="#f6f6f6"))
    host = d.add(Box(10, 40, 85, 50, label="USB 3 host", fill=COLOR_HARDWARE))
    gt   = d.add(Box(180, 40, 100, 50, label="Transceiver", fill=COLOR_HARDWARE))
    pipe = d.add(Box(315, 45, 110, 40, label="PIPE", fill=COLOR_ACCENT))
    lts  = d.add(Box(315, 115, 110, 40, label="LTSSM", fill=COLOR_ACCENT))
    link = d.add(Box(475, 45, 110, 40, label="Link layer", fill=COLOR_ACCENT))
    prot = d.add(Box(475, 115, 110, 40, label="Protocol layer", fill=COLOR_ACCENT))
    cls  = d.add(Box(620, 45, 100, 40, label="Classes"))
    ep   = d.add(Box(620, 115, 100, 40, label="Endpoints"))
    d.add(Arrow((host, "r"), (gt, "l"), kind="data", heads="both", label="cable"))
    d.add(Arrow((gt, "r"), (pipe, "l"), kind="data", heads="both"))
    d.add(Arrow((pipe, "b"), (lts, "t"), kind="control", heads="both"))
    d.add(Arrow((pipe, "r"), (link, "l"), kind="accent", heads="both"))
    d.add(Arrow((link, "b"), (prot, "t"), kind="data", heads="both"))
    d.add(Arrow((prot, "r"), (ep, "l"), kind="data", heads="both"))
    d.add(Arrow((ep, "t"), (cls, "b"), kind="data", heads="both"))
    return d
