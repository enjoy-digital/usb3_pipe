#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PHY clock domains (Gen 1, no pipe_cd): the transceiver tx/rx domains and the sys domain, with the
# clock domain crossing FIFOs of the TX and RX datapaths between them (usb3_pipe/datapath.py:
# TXDatapath inserts SKP in sys, RXDatapath removes SKP in rx and aligns the words in sys).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(20, 20, 260, 255, title="tx / rx", title_pos="bl"))
    d.add(Container(410, 20, 330, 255, title="sys"))
    xcvr  = d.add(Box(40, 95, 90, 100, label="Transceiver", fill=COLOR_HARDWARE))
    width = d.add(Box(155, 50, 105, 50, label="Width\nconversion"))
    skprm = d.add(Box(155, 190, 105, 50, label="SKP\nremover", fill=COLOR_ACCENT_LIGHT))
    txcdc = d.add(Box(305, 50, 80, 50, label="TX FIFO", fill=COLOR_ACCENT))
    rxcdc = d.add(Box(305, 190, 80, 50, label="RX FIFO", fill=COLOR_ACCENT))
    skpin = d.add(Box(430, 50, 105, 50, label="SKP\ninserter", fill=COLOR_ACCENT_LIGHT))
    align = d.add(Box(430, 190, 105, 50, label="Word\naligner"))
    pipe  = d.add(Box(570, 50, 150, 190, label="USB3PIPE\nLTSSM"))
    d.add(Arrow((pipe, "l", 25/190), (skpin, "r"), kind="data"))
    d.add(Arrow((skpin, "l"), (txcdc, "r"), kind="data"))
    d.add(Arrow((txcdc, "l"), (width, "r"), kind="data"))
    d.add(Arrow((width, "l"), (xcvr, "t"), kind="data"))
    d.add(Arrow((xcvr, "b"), (skprm, "l"), kind="data"))
    d.add(Arrow((skprm, "r"), (rxcdc, "l"), kind="data"))
    d.add(Arrow((rxcdc, "r"), (align, "l"), kind="data"))
    d.add(Arrow((align, "r"), (pipe, "l", 165/190), kind="data"))
    return d
