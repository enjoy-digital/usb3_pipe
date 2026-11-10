#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 2 128b/132b blocks: Block Header and 16 Symbols, the SYNC and SKP Ordered Set blocks.

from svgdiag import *
from datapath_helpers import strip

H, S = 60, 40 # Block Header and Symbol cell widths.


def build():
    d = Diagram(CANVAS_WIDTH, 280)
    x0 = 20
    d.add(Text(x0, 25, "Data or control block (132 bits)", anchor="start", italic=True))
    strip(d, x0, 40, [(H, "Header", COLOR_ACCENT)] + [(S, str(i), COLOR_FILL) for i in range(16)])
    d.add(Text(x0, 92, "Header 0011b: data, 1100b: control", anchor="start", size=FONT_SIZE_SM))
    d.add(Text(x0 + H + 8*S, 92, "Symbols 0 to 15, bit 0 first", anchor="middle", size=FONT_SIZE_SM))

    d.add(Text(x0, 130, "SYNC Ordered Set", anchor="start", italic=True))
    strip(d, x0, 145, [(H, "1100b", COLOR_ACCENT)] +
        [(S, "00h" if i % 2 == 0 else "FFh", COLOR_FILL) for i in range(16)])

    d.add(Text(x0, 215, "SKP Ordered Set (196 bits)", anchor="start", italic=True))
    strip(d, x0, 230, [(H, "1100b", COLOR_ACCENT), (400, "SKP (CCh) x 20", COLOR_SHADE),
        (80, "SKPEND", COLOR_FILL), (160, "LFSR state", COLOR_FILL)])
    return d
