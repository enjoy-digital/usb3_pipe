#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The transceiver wrapper (USB3SerDes): what the PIPE sees, what the wrapper adds to the transceiver.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 340)
    pipe = d.add(Box(15, 40, 100, 250, label="PIPE", fill=COLOR_ACCENT_LIGHT))
    d.add(Container(250, 15, 310, 310, title="USB3SerDes", title_pos="br"))
    txd  = d.add(Box(270, 40, 150, 50, label="TX datapath", fill=COLOR_ACCENT))
    rxd  = d.add(Box(270, 110, 150, 50, label="RX datapath", fill=COLOR_ACCENT))
    lfps = d.add(Box(270, 180, 150, 50, label="Electrical idle\nand LFPS"))
    cfg  = d.add(Box(270, 250, 150, 40, label="USB 3 settings"))
    xcvr = d.add(Box(600, 40, 145, 250, label="Transceiver\nGTP, GTX, GTY,\nGTR12, DCU",
        fill=COLOR_HARDWARE))
    d.add(Arrow((pipe, "r", 25/250), (txd, "l"), kind="data", label="32-bit + K flags"))
    d.add(Arrow((rxd, "l"), (pipe, "r", 95/250), kind="data", label="32-bit + K flags"))
    d.add(Arrow((pipe, "r", 155/250), (lfps, "l", 0.3), kind="control", label="tx_idle, tx_lfps"))
    d.add(Arrow((lfps, "l", 0.7), (pipe, "r", 175/250), kind="control", label="rx_idle, rx_errors",
        label_side="below"))
    d.add(Arrow((txd, "r"), (xcvr, "l", 25/250), kind="data"))
    d.add(Arrow((xcvr, "l", 95/250), (rxd, "r"), kind="data"))
    d.add(Arrow((lfps, "r"), (xcvr, "l", 165/250), kind="control"))
    d.add(Arrow((cfg, "r"), (xcvr, "l", 230/250), kind="control"))
    return d
