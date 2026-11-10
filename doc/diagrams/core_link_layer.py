#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link layer: RX parser and checks, header flow control (sequence numbers, credits, retries),
# Tx Header Buffers and TX framer.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 360)
    d.add(Container(130, 10, 500, 340, title="USB3LinkLayer", title_pos="bl"))
    phy = d.add(Box(20, 40, 80, 270, label="PHY", fill=COLOR_HARDWARE))
    up  = d.add(Box(660, 40, 80, 270, label="Protocol\nlayer", fill=COLOR_ACCENT_LIGHT))
    # RX row.
    rxp = d.add(Box(160, 40, 130, 50, label="RX parser\nframing"))
    crc = d.add(Box(320, 40, 130, 50, label="CRC-5/16/32\nchecks"))
    rxb = d.add(Box(480, 40, 130, 50, label="Rx Header\nBuffers"))
    # Middle: flow control.
    fc  = d.add(Box(320, 120, 130, 50, label="Flow control\nseq, credits", fill=COLOR_ACCENT))
    tim = d.add(Box(160, 120, 130, 50, label="Timers\nLMPs, PM"))
    # TX row.
    txf = d.add(Box(160, 200, 130, 50, label="TX framer\nCRCs"))
    txb = d.add(Box(320, 200, 130, 50, label="Tx Header\nBuffers"))
    arb = d.add(Box(320, 270, 130, 40, label="DPP splitter"))
    d.add(Arrow((phy.x + phy.w, rxp.cy), (rxp, "l"), kind="data"))
    d.add(Arrow((rxp, "r"), (crc, "l"), kind="data"))
    d.add(Arrow((crc, "r"), (rxb, "l"), kind="data"))
    d.add(Arrow((rxb, "r"), (up.x, rxb.cy), kind="data"))
    d.add(Arrow((up.x, txb.cy), (txb, "r"), kind="data", label="HP", label_side="above"))
    d.add(Arrow((up.x, arb.cy), (arb, "r"), kind="data", label="DPP", label_side="above"))
    d.add(Arrow((arb, "l"), (txf, "b"), kind="data"))
    d.add(Arrow((txb, "l"), (txf, "r"), kind="data"))
    d.add(Arrow((txf, "l"), (phy.x + phy.w, txf.cy), kind="data"))
    d.add(Arrow((crc, "b"), (fc, "t"), kind="control", label="LGOOD, LCRD", label_pos=(400, 105),
        label_anchor="start"))
    d.add(Arrow((fc, "b"), (txb, "t"), kind="control", label="retry", label_pos=(400, 185),
        label_anchor="start"))
    d.add(Arrow((tim, "r"), (fc, "l"), kind="feedback"))
    return d
