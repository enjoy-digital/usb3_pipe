#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECPIX-5 USB-C: the DCU lanes through the SuperSpeed mux, polarity from the TUSB322 cable direction.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    d.add(Container(15, 15, 330, 230, title="ECP5-5G FPGA", title_pos="br"))
    dcu  = d.add(Box(40, 45, 140, 60, label="DCU0 channel", fill=COLOR_ACCENT))
    rdr  = d.add(Box(40, 145, 140, 60, label="TUSB322\nreader"))
    mux  = d.add(Box(420, 45, 140, 60, label="SuperSpeed\nmux", fill=COLOR_HARDWARE))
    cc   = d.add(Box(420, 145, 140, 60, label="TUSB322\nCC controller", fill=COLOR_HARDWARE))
    usbc = d.add(Box(620, 45, 125, 160, label="USB-C\nconnector", fill=COLOR_HARDWARE))
    d.add(Arrow((dcu, "r"), (mux, "l"), kind="data", heads="both", label="TX and RX pairs"))
    d.add(Arrow((mux, "r"), (usbc, "l", 30/160), kind="data", heads="both"))
    d.add(Arrow((cc, "r"), (usbc, "l", 130/160), kind="control", heads="both", label="CC"))
    d.add(Arrow((cc, "l", 0.7), (rdr, "r", 0.7), kind="control", label="I2C: cable direction",
        label_pos=(255, 200)))
    d.add(Arrow((rdr, "r", 0.2), (mux, "b"), kind="control",
        via=[(380, 157), (380, 125), (490, 125)], label="polarity", label_pos=(435, 115)))
    return d
