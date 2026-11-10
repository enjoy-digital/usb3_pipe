#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 1 clock compensation: the SKP word received in the rx clock domain is not written to the CDC
# FIFO, leaving room for a slower local clock.

from svgdiag import *
from datapath_helpers import strip


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    W = 70
    d.add(Text(20, 30, "Received words (rx clock)", anchor="start", italic=True))
    d.add(Text(740, 30, "2 SKP Ordered Sets every 176 words", anchor="end", size=FONT_SIZE_SM))
    tx = strip(d, 20, 50, [
        (W, "Word", COLOR_FILL), (W, "Word", COLOR_FILL), (W, "Word", COLOR_FILL),
        (2*W, "SKP SKP SKP SKP", COLOR_ACCENT),
        (W, "Word", COLOR_FILL), (W, "Word", COLOR_FILL), (W, "Word", COLOR_FILL),
        (W, "Word", COLOR_FILL), (W, "Word", COLOR_FILL)])
    d.add(Arrow((tx[3], "b"), (tx[3].cx, 125), kind="control", heads="end"))
    d.add(Text(tx[3].cx + 10, 112, "dropped", anchor="start", size=FONT_SIZE_SM))
    d.add(Text(20, 150, "Words written to the CDC FIFO", anchor="start", italic=True))
    strip(d, 20, 170, [(W, "Word", COLOR_FILL) for _ in range(3)] + [(2*W, "no write", COLOR_SHADE)] +
        [(W, "Word", COLOR_FILL) for _ in range(5)])
    return d
