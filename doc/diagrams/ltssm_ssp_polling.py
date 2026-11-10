#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# SuperSpeedPlus Polling (SSPPolling): SCD1/SCD2 then LBPM PHY Capability/Ready, before
# Polling.RxEQ at the negotiated rate, or the SuperSpeed fallback.

from svgdiag import *


def build():
    names = ["Polling.LFPS\n(SCD1)", "Polling.LFPSPlus\n(SCD2)", "Polling.PortMatch\n(LBPM capability)",
             "Polling.PortConfig\n(LBPM ready)", "Polling.RxEQ"]
    conds = ["SCD1 received", "SCD2 received", "capabilities matched", "rate switched"]
    step, h, x, w = 74, 46, 60, 210
    d = Diagram(CANVAS_WIDTH, 20 + step*(len(names) - 1) + h + 20)
    boxes = []
    for i, n in enumerate(names):
        fill = COLOR_ACCENT if i == len(names) - 1 else COLOR_FILL
        boxes.append(d.add(Box(x, 20 + i*step, w, h, label=n, fill=fill)))
    for a, b, c in zip(boxes, boxes[1:], conds):
        d.add(Arrow((a, "b"), (b, "t"), kind="data", label=c, label_side="right"))
    fb = d.add(Box(470, 20, 230, 46, label="SuperSpeed Polling.LFPS\n(Gen 1x1 fallback)"))
    d.add(Arrow((boxes[0], "r"), (fb, "l"), kind="control", label="no SCD received",
        label_side="above"))
    return d
