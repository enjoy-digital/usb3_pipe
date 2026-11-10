#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LFPS signaling: bursts of a low frequency clock separated by electrical idle, tBurst and tRepeat.

from svgdiag import *


def _burst(x0, x1, hi, lo, half=6):
    pts = [(x0, (hi + lo)/2), (x0, hi)]
    x, level = x0, hi
    while x + half < x1:
        x += half
        pts.append((x, level))
        level = lo if level == hi else hi
        pts.append((x, level))
    pts.append((x1, level))
    pts.append((x1, (hi + lo)/2))
    return pts


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    hi, lo, mid = 70, 110, 90
    bursts = [(80, 176), (380, 476), (680, 740)]
    for x0, x1 in bursts:
        d.add(Bus(_burst(x0, x1, hi, lo), kind="data", color=COLOR_CYAN))
    for x0, x1 in [(20, 80), (176, 380), (476, 680)]:
        d.add(Bus([(x0, mid), (x1, mid)], kind="data", dash="6,4"))
    d.add(Text(278, 52, "electrical idle", size=FONT_SIZE_SM, color=COLOR_STROKE))
    d.add(Text(578, 52, "electrical idle", size=FONT_SIZE_SM, color=COLOR_STROKE))
    d.add(Text(128, 52, "burst", size=FONT_SIZE_SM, color=COLOR_STROKE))
    for x in (80, 176, 380):
        d.add(Bus([(x, 118), (x, 200)], dash="2,3"))
    d.add(Arrow((80, 145), (176, 145), heads="both", label="tBurst", label_side="below"))
    d.add(Arrow((80, 185), (380, 185), heads="both", label="tRepeat", label_side="below"))
    return d
