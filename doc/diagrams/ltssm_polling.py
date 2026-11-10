#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LTSSM Polling substates from power-on to U0, with the timeouts back to Rx.Detect.

from svgdiag import *


def build():
    names = ["Polling.Entry", "Polling.LFPS", "Polling.RxEQ", "Polling.Active",
             "Polling.Configuration", "Polling.Idle", "U0"]
    conds = ["", "LFPS handshake", "TSEQ sent", "8 TS1 or TS2 received", "8 TS2 received, 16 sent",
             "Idle handshake"]
    step, h, x, w = 62, 34, 60, 190
    d = Diagram(CANVAS_WIDTH, 30 + step*(len(names) - 1) + h + 20)
    boxes = []
    for i, n in enumerate(names):
        fill = COLOR_ACCENT if n == "U0" else COLOR_FILL
        boxes.append(d.add(Box(x, 20 + i*step, w, h, label=n, fill=fill)))
    for a, b, c in zip(boxes, boxes[1:], conds):
        d.add(Arrow((a, "b"), (b, "t"), kind="data", label=c or None, label_side="right"))
    # Timeouts of LFPS, Active, Configuration and Idle to Rx.Detect (dotted trunk).
    trunk = 420
    rows  = [boxes[i] for i in (1, 3, 4, 5)]
    for b in rows:
        d.add(Bus([(x + w, b.cy), (trunk, b.cy)], dash=DASH_FEEDBACK))
    d.add(Bus([(trunk, rows[0].cy), (trunk, rows[-1].cy)], dash=DASH_FEEDBACK))
    rx = d.add(Box(500, rows[1].cy - 24, 200, 48, label="Polling.ExitToRxDetect\n(Rx.Detect)"))
    d.add(Arrow((trunk, rx.cy), (rx, "l"), kind="feedback", label="timeout", label_side="above"))
    d.add(Arrow((rx, "t"), (boxes[0], "r"), kind="feedback", label="retrain",
        label_pos=(560, 30)))
    return d
