#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PHY with pipe_cd (slower fabrics): PIPE and SerDes in their own clock domain at 125 MHz, LTSSM and
# link interface in sys at half the frequency on 64-bit words.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 310)
    d.add(Container(20, 20, 335, 280, title="pipe_cd, 125 MHz", title_pos="bl"))
    d.add(Container(370, 20, 370, 280, title="sys, 62.5 MHz"))
    serdes = d.add(Box(35, 110, 95, 90, label="Transceiver\nSerDes", fill=COLOR_HARDWARE))
    pipe   = d.add(Box(150, 60, 90, 190, label="USB3PIPE", fill=COLOR_ACCENT))
    pkts   = d.add(Box(255, 50, 90, 45, label="Packets\nbuffer"))
    idle   = d.add(Box(255, 215, 90, 45, label="Idle filter\npacker"))
    txf    = d.add(Box(385, 50, 90, 45, label="TX FIFO", fill=COLOR_ACCENT_LIGHT))
    rxf    = d.add(Box(385, 215, 90, 45, label="RX FIFO", fill=COLOR_ACCENT_LIGHT))
    ltssm  = d.add(Box(510, 132, 90, 46, label="LTSSM"))
    link   = d.add(Box(635, 50, 90, 210, label="Link\nlayer", fill=COLOR_ACCENT_LIGHT))
    d.add(Arrow((serdes, "r"), (pipe, "l"), kind="data", heads="both"))
    d.add(Arrow((link, "l", 22.5/210), (txf, "r"), kind="data", label="64-bit",
        label_side="above"))
    d.add(Arrow((txf, "l"), (pkts, "r"), kind="data"))
    d.add(Arrow((pkts, "l"), (pipe, "r", 12.5/190), kind="data"))
    d.add(Arrow((pipe, "r", 177.5/190), (idle, "l"), kind="data"))
    d.add(Arrow((idle, "r"), (rxf, "l"), kind="data"))
    d.add(Arrow((rxf, "r"), (link, "l", 187.5/210), kind="data", label="64-bit",
        label_side="above"))
    d.add(Arrow((pipe, "r", 0.5), (ltssm, "l"), kind="control", heads="both", label="resync",
        label_pos=(440, 145)))
    d.add(Arrow((ltssm, "r"), (link, "l", 0.5), kind="control", heads="both"))
    return d
