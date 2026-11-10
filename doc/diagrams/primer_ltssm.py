#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Simplified LTSSM: training, operation, power states, recovery and resets.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    rxd = d.add(Box(20,  30,  130, 44, label="Rx.Detect"))
    pol = d.add(Box(220, 30,  130, 44, label="Polling", fill=COLOR_ACCENT_LIGHT))
    u0  = d.add(Box(420, 30,  130, 44, label="U0", fill=COLOR_ACCENT))
    ux  = d.add(Box(610, 30,  130, 44, label="U1, U2, U3"))
    ina = d.add(Box(20,  150, 130, 44, label="SS.Inactive"))
    hot = d.add(Box(220, 150, 130, 44, label="Hot Reset"))
    rec = d.add(Box(420, 150, 130, 44, label="Recovery", fill=COLOR_ACCENT_LIGHT))
    # Training: Rx.Detect -> Polling -> U0, Polling timeout back to Rx.Detect.
    d.add(Arrow((rxd, "r", 0.3), (pol, "l", 0.3), kind="data", label="found"))
    d.add(Arrow((pol, "l", 0.75), (rxd, "r", 0.75), kind="feedback", label="timeout",
        label_side="below"))
    d.add(Arrow((pol, "r", 0.3), (u0, "l", 0.3), kind="data", label="trained"))
    # Power states: LGO_Ux handshake in, exit or wakeup LFPS then Recovery.
    d.add(Arrow((u0, "r"), (ux, "l"), kind="control", label="LGO_Ux"))
    d.add(Arrow((ux, "b"), (rec, "r"), via=[(675, 172)], kind="control", label="exit LFPS",
        label_pos=(612, 162)))
    # Recovery: retraining from U0 and back.
    d.add(Arrow((u0, "b", 0.3), (rec, "t", 0.3), kind="control", label="errors", label_side="left"))
    d.add(Arrow((rec, "t", 0.7), (u0, "b", 0.7), kind="data", label="retrained", label_side="right"))
    # Hot Reset: requested in the TS2 of Recovery, returns to U0.
    d.add(Arrow((rec, "l", 0.3), (hot, "r", 0.3), kind="control", label="TS2 Reset",
        label_pos=(385, 155)))
    d.add(Arrow((hot, "t", 0.8), (u0, "l", 0.8), via=[(324, 118), (385, 118), (385, 65)],
        kind="control"))
    # Recovery timeout: SS.Inactive, left by a Warm Reset.
    d.add(Arrow((rec, "b"), (ina, "b"), via=[(485, 230), (85, 230)], kind="feedback",
        label="timeout", label_pos=(285, 222)))
    d.add(Arrow((ina, "t"), (rxd, "b"), kind="feedback", label="Warm Reset", label_side="right"))
    return d
