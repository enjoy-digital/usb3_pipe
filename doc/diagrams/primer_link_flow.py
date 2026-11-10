#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Header packet flow control: sequence numbers acknowledged with LGOOD_n, buffers freed with LCRD_x.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 350)
    e = Exchange(d, "Device", "Host", fill_left=COLOR_ACCENT, fill_right=COLOR_HARDWARE)
    e.msg("Advertisement: LGOOD_7, LCRD_A-D", to="right", note="on each U0 entry", step=32)
    e.msg("Advertisement: LGOOD_7, LCRD_A-D", to="left", step=40)
    e.msg("Header Packet, seq 0", to="right", kind="accent", note="uses 1 credit")
    e.msg("Header Packet, seq 1", to="right", kind="accent")
    e.msg("LGOOD_0", to="left", note="seq 0 received")
    e.msg("LCRD_A", to="left", note="buffer A free")
    e.msg("LBAD", to="left", note="seq 1 bad CRC", step=32)
    e.msg("LRTY, Header Packet, seq 1", to="right", kind="accent")
    return d
