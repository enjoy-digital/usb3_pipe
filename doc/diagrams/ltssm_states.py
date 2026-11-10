#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LTSSM link states around U0 (usb3_pipe/ltssm.py): U1, U2 and U3 entered from U0 by the power
# management handshake and left through Recovery, Recovery and its Hot Reset branch, Recovery
# timeouts to eSS.Inactive and the retraining after its 12ms quiet period.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 440)
    lp  = d.add(Container(20, 20, 175, 330, title="Low power", title_pos="bl"))
    u1  = d.add(Box(45, 45, 125, 46, label="U1", fill=COLOR_ACCENT_LIGHT))
    u2  = d.add(Box(45, 135, 125, 46, label="U2", fill=COLOR_ACCENT_LIGHT))
    u3  = d.add(Box(45, 225, 125, 46, label="U3", fill=COLOR_ACCENT_LIGHT))
    pol = d.add(Box(305, 20, 150, 46, label="Polling"))
    u0  = d.add(Box(305, 130, 150, 60, label="U0", fill=COLOR_ACCENT))
    rec = d.add(Box(305, 270, 150, 46, label="Recovery"))
    hot = d.add(Box(570, 270, 140, 46, label="Hot Reset"))
    ess = d.add(Box(305, 375, 150, 46, label="eSS.Inactive"))
    d.add(Arrow((pol, "b"), (u0, "t"), kind="data", label="trained", label_side="right"))
    d.add(Arrow((u1, "b"), (u2, "t"), kind="control", label="U2 timer", label_side="right"))
    d.add(Arrow((u0, "l"), (195, 160), kind="control", label="LGO_Ux", label_side="above"))
    d.add(Arrow((195, 293), (rec, "l"), kind="control", label="exit LFPS", label_side="above"))
    d.add(Arrow((u0, "b", 0.3), (rec, "t", 0.3), kind="data", label="TS1, timeout",
        label_side="left"))
    d.add(Arrow((rec, "t", 0.7), (u0, "b", 0.7), kind="data", label="Idle handshake",
        label_side="right"))
    d.add(Arrow((rec, "r"), (hot, "l"), kind="control", label="Reset bit", label_side="above"))
    d.add(Arrow((hot, "t"), (u0, "r"), kind="control", via=[(640, 160)]))
    d.add(Arrow((rec, "b"), (ess, "t"), kind="feedback", label="timeout", label_side="right"))
    d.add(Arrow((ess, "r"), (pol, "r"), kind="feedback", via=[(740, 398), (740, 43)],
        label="12 ms quiet", label_pos=(690, 388)))
    return d
