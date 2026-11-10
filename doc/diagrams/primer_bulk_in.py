#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Bulk IN transaction: the host requests packets with an ACK TP, the device sends data packets.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 380)
    e = Exchange(d, "Device", "Host", fill_left=COLOR_ACCENT, fill_right=COLOR_HARDWARE)
    e.msg("ACK TP: seq 0, NumP 2", to="left", note="request 2 packets")
    e.msg("DP: seq 0", to="right", kind="accent")
    e.msg("DP: seq 1", to="right", kind="accent", note="burst complete")
    e.msg("ACK TP: seq 2, NumP 2", to="left", note="got 2, asks 2 more")
    e.msg("NRDY TP", to="right", note="no data yet")
    e.msg("ERDY TP", to="right", note="data ready")
    e.msg("ACK TP: seq 2, NumP 2", to="left", note="request again")
    e.msg("DP: seq 2, EOB", to="right", kind="accent", note="only 1 ready")
    return d
