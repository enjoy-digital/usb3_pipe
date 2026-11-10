#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Bulk IN burst and flow control: ACK TP requests with NumP, DPs, NRDY then ERDY.

from svgdiag import *
from core_helpers import Lanes


def build():
    d = Diagram(CANVAS_WIDTH, 455)
    l = Lanes(d, "Host", "Bulk IN endpoint", rows=11, row=33)
    l.msg("ACK TP: Seq 0, NumP 4", to_right=True, kind="control", note="request 4 packets")
    for i in range(4):
        l.msg(f"DP Seq {i}", to_right=False, kind="accent")
    l.msg("ACK TP: Seq 4, NumP 4", to_right=True, kind="control", note="slots 0 to 3 freed")
    l.msg("NRDY", to_right=False, kind="control", note="slots empty")
    l.gap(0.6)
    l.msg("ERDY", to_right=False, kind="control", note="user wrote a packet")
    l.msg("ACK TP: Seq 4, NumP 4", to_right=True, kind="control")
    l.msg("DP Seq 4, short packet", to_right=False, kind="accent", note="ends the transfer")
    return d
