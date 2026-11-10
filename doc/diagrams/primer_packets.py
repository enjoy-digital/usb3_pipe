#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Gen 1 framing of a Header Packet, a Data Packet (header + payload) and a Link Command.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    F = COLOR_SHADE
    d.add(Text(20, 30, "Header Packet", size=13, anchor="start", bold=True))
    bitfield(d, 20, 42, [(150, "SHP SHP SHP EPF", F), (250, "Header (12 bytes)", COLOR_ACCENT),
        (110, "CRC-16", COLOR_ACCENT_LIGHT), (150, "Link Control Word", COLOR_ACCENT_LIGHT)], h=34)
    d.add(Text(20, 104, "Data Packet Payload (after its header packet)", size=13, anchor="start", bold=True))
    bitfield(d, 20, 116, [(150, "SDP SDP SDP EPF", F), (250, "Data (0 to 1024 bytes)", COLOR_ACCENT),
        (110, "CRC-32", COLOR_ACCENT_LIGHT), (150, "END END END EPF", F)], h=34)
    d.add(Text(20, 178, "Link Command", size=13, anchor="start", bold=True))
    bitfield(d, 20, 190, [(150, "SLC SLC SLC EPF", F), (250, "Command word (x2)", COLOR_ACCENT)], h=34)
    d.add(Text(560, 207, "K symbols: grey", size=12, color="#444444"))
    return d
