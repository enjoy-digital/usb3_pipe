#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Standalone Verilog core (usb3_pipe_gen): port groups of the generated module.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 320)
    d.add(Container(250, 20, 270, 280, title="usb3_pipe_core", title_pos="bl"))
    phy  = d.add(Box(290, 50, 190, 125, label="PHY", fill=COLOR_ACCENT))
    core = d.add(Box(290, 208, 190, 56, label="Device core\n(usb_core)", fill=COLOR_ACCENT_LIGHT))
    clk  = d.add(Box(30, 60, 150, 36, label="clk, rst"))
    ref  = d.add(Box(30, 102, 150, 36, label="refclk_p/n"))
    ser  = d.add(Box(30, 144, 150, 36, label="tx_p/n, rx_p/n", fill=COLOR_HARDWARE))
    sym  = d.add(Box(580, 60, 160, 36, label="sink/source"))
    stat = d.add(Box(580, 144, 160, 36, label="status, ltssm_state"))
    bulk = d.add(Box(580, 218, 160, 36, label="bulk_in/bulk_out"))
    d.add(Arrow((clk, "r"), (290, 78), kind="control"))
    d.add(Arrow((ref, "r"), (290, 120), kind="control"))
    d.add(Arrow((ser, "r"), (290, 162), kind="data", heads="both"))
    d.add(Arrow((480, 78), (sym, "l"), kind="data", heads="both"))
    d.add(Arrow((480, 162), (stat, "l"), kind="control"))
    d.add(Arrow((phy, "b"), (core, "t"), kind="accent", heads="both"))
    d.add(Arrow((core, "r", 0.5), (bulk, "l"), kind="data", heads="both"))
    return d
