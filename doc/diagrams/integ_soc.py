#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Integration in a LiteX SoC: CRG, transceiver wrapper, PHY, device core, endpoints, user logic.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    d.add(Container(20, 20, 720, 290, title="LiteX SoC", title_pos="br"))
    crg   = d.add(Box(45, 60, 120, 60, label="CRG\nsys, oob"))
    serd  = d.add(Box(45, 200, 120, 70, label="Transceiver\nwrapper", fill=COLOR_ACCENT_LIGHT))
    phy   = d.add(Box(215, 200, 110, 70, label="USB3PHY", fill=COLOR_ACCENT))
    core  = d.add(Box(375, 200, 130, 70, label="USB3DeviceCore", fill=COLOR_ACCENT))
    eps   = d.add(Box(555, 200, 150, 70, label="Endpoints\nBulk, Iso, EP0"))
    user  = d.add(Box(555, 60, 150, 60, label="User logic"))
    csr   = d.add(Box(375, 60, 130, 60, label="CSR bus"))
    d.add(Arrow((crg, "b"), (serd, "t"), kind="control", label="clocks", label_side="right"))
    d.add(Arrow((serd, "r"), (phy, "l"), kind="data", heads="both"))
    d.add(Arrow((phy, "r"), (core, "l"), kind="accent", heads="both"))
    d.add(Arrow((core, "r"), (eps, "l"), kind="data", heads="both"))
    d.add(Arrow((eps, "t"), (user, "b"), kind="data", heads="both", label="streams", label_side="right"))
    d.add(Arrow((csr, "b"), (core, "t"), kind="control", label="status", label_side="right"))
    return d
