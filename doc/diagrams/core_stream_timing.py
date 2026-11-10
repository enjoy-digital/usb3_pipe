#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Endpoint user stream: an 18-byte packet on a 32-bit Bulk IN sink, with one cycle of
# back-pressure (ready low).

from svgdiag import *
from core_helpers import Waves


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    w = Waves(d, x0=110, y0=10, cycles=10, cw=56, rh=34, label_x=95)
    w.clock("clk")
    w.bit("valid", "0111111000")
    w.bit("ready", "1110111111")
    w.bus("data",  [None, "D0", "D1", "D2", "D2", "D3", "D4", None, None, None], accent=["D4"])
    w.bus("be",    [None, "0xf", "0xf", "0xf", "0xf", "0xf", "0x3", None, None, None])
    w.bit("last",  "0000001000")
    w.marker(3, "stall")
    w.marker(6, "18 bytes")
    return d
