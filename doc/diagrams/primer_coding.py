#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 1 transmit and receive symbol chains.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 220)
    d.add(Container(15, 10, 730, 95, title="TX", title_pos="tl", title_size=15))
    d.add(Container(15, 115, 730, 95, title="RX", title_pos="tl", title_size=15))
    tx = ["Link words", "SKP insert", "Scrambler", "8b/10b", "Serializer"]
    rx = ["Link words", "Descrambler", "SKP remove", "8b/10b", "Aligner"]
    fills_tx = [COLOR_FILL, COLOR_ACCENT_LIGHT, COLOR_ACCENT, COLOR_ACCENT, COLOR_HARDWARE]
    fills_rx = [COLOR_FILL, COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_ACCENT, COLOR_ACCENT_LIGHT]
    prev = None
    for i, (n, f) in enumerate(zip(tx, fills_tx)):
        b = d.add(Box(75 + i*135, 40, 115, 44, label=n, fill=f))
        if prev:
            d.add(Arrow((prev, "r"), (b, "l"), kind="data"))
        prev = b
    prev = None
    for i, (n, f) in enumerate(zip(rx, fills_rx)):
        b = d.add(Box(75 + i*135, 145, 115, 44, label=n, fill=f))
        if prev:
            d.add(Arrow((b, "l"), (prev, "r"), kind="data"))
        prev = b
    return d
