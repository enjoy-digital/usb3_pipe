#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Device core unit tests: a reactive Python host drives the link interface of the device core
# through a PHY stand-in (Migen simulation, no PHY, no transceiver).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 170)
    sim  = d.add(Container(192, 20, 548, 130, title="Migen simulation", title_pos="br"))
    py   = d.add(Box(20, 55, 110, 50, label="Python host\n(PyHost)", fill=COLOR_ACCENT_LIGHT))
    pst  = d.add(Box(215, 55, 140, 50, label="PHY stand-in"))
    core = d.add(Box(380, 55, 140, 50, label="Device core", fill=COLOR_ACCENT))
    ept  = d.add(Box(560, 55, 140, 50, label="Endpoints\nand classes"))
    d.add(Arrow((py, "r"), (pst, "l"), kind="accent", heads="both", label="words", label_side="above"))
    d.add(Arrow((pst, "r"), (core, "l"), kind="data", heads="both"))
    d.add(Arrow((core, "r"), (ept, "l"), kind="data", heads="both"))
    return d
