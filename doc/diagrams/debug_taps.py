#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Debug infrastructure: where the debug registers, the LiteScope probes and the traces tap the
# datapath, all read from the host through the CSR bus.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    dp = d.add(Container(20, 20, 720, 110, title="Datapath", title_pos="br"))
    gt   = d.add(Box(40, 45, 150, 46, label="Transceiver", fill=COLOR_HARDWARE))
    phy  = d.add(Box(215, 45, 150, 46, label="PIPE and LTSSM"))
    link = d.add(Box(390, 45, 150, 46, label="Link layer"))
    prot = d.add(Box(565, 45, 150, 46, label="Protocol layer"))
    for a, b in ((gt, phy), (phy, link), (link, prot)):
        d.add(Arrow((a, "r"), (b, "l"), kind="data", heads="both"))
    dbg = d.add(Container(20, 160, 720, 120, title="Debug cores", title_pos="br"))
    sdb = d.add(Box(40, 185, 150, 46, label="Transceiver\ndebug", fill=COLOR_ACCENT))
    pdb = d.add(Box(215, 185, 150, 46, label="PHY debug", fill=COLOR_ACCENT))
    ls  = d.add(Box(390, 185, 150, 46, label="LiteScope", fill=COLOR_ACCENT_LIGHT))
    tr  = d.add(Box(565, 185, 150, 46, label="Link and packet\ntraces", fill=COLOR_ACCENT))
    d.add(Arrow((gt, "b"), (sdb, "t"), kind="feedback", heads="both"))
    d.add(Arrow((phy, "b"), (pdb, "t"), kind="feedback"))
    d.add(Arrow((phy, "b", 0.85), (ls, "t", 0.15), via=[(342.5, 138), (412.5, 138)], kind="feedback"))
    d.add(Arrow((link, "b"), (ls, "t", 0.5), kind="feedback"))
    d.add(Arrow((link, "b", 0.85), (tr, "t", 0.15), via=[(517.5, 145), (587.5, 145)], kind="feedback"))
    d.add(Arrow((prot, "b"), (tr, "t", 0.5), kind="feedback"))
    return d
