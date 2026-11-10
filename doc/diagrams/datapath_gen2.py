#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 2 datapaths: packet framing, blocks, scrambling, gearbox and block alignment.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    link  = d.add(Box(15, 40, 100, 170, label="Link layer", fill=COLOR_ACCENT_LIGHT))
    xcvr  = d.add(Box(645, 40, 100, 170, label="Transceiver\nraw words", fill=COLOR_HARDWARE))
    # TX row.
    ftx   = d.add(Box(150, 40, 105, 60, label="Packet\nframer"))
    gtx   = d.add(Box(275, 40, 105, 60, label="Blocks and\nscrambler", fill=COLOR_ACCENT))
    dcb   = d.add(Box(400, 40, 95, 60, label="DC\nbalance"))
    gbx   = d.add(Box(515, 40, 100, 60, label="Gearbox\nto raw words"))
    # RX row.
    frx   = d.add(Box(150, 150, 105, 60, label="Packet\nparser"))
    grx   = d.add(Box(275, 150, 160, 60, label="Descrambler and\nOrdered Sets", fill=COLOR_ACCENT))
    aln   = d.add(Box(470, 150, 145, 60, label="Block aligner\nand SKP removal"))
    d.add(Arrow((link, "r", 30/170), (ftx, "l"), kind="data"))
    d.add(Arrow((ftx, "r"), (gtx, "l"), kind="data"))
    d.add(Arrow((gtx, "r"), (dcb, "l"), kind="data"))
    d.add(Arrow((dcb, "r"), (gbx, "l"), kind="data"))
    d.add(Arrow((gbx, "r"), (xcvr, "l", 30/170), kind="data"))
    d.add(Arrow((xcvr, "l", 140/170), (aln, "r"), kind="data"))
    d.add(Arrow((aln, "l"), (grx, "r"), kind="data"))
    d.add(Arrow((grx, "l"), (frx, "r"), kind="data"))
    d.add(Arrow((frx, "l"), (link, "r", 140/170), kind="data"))
    return d
