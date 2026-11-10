#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Bulk OUT transaction: the host sends data packets, the device acknowledges them and reports space.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    e = Exchange(d, "Device", "Host", fill_left=COLOR_ACCENT, fill_right=COLOR_HARDWARE)
    e.msg("DP: seq 0", to="left", kind="accent")
    e.msg("DP: seq 1", to="left", kind="accent")
    e.msg("ACK TP: seq 1, NumP 1", to="right", note="room for 1")
    e.msg("ACK TP: seq 2, NumP 0", to="right", note="buffer full")
    e.msg("ERDY TP: NumP 4", to="right", note="space again")
    e.msg("DP: seq 2", to="left", kind="accent")
    return d
