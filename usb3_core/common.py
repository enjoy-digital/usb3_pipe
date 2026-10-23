#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Core common definitions (USB 3.2 chapters 7/8).

The core works on little-endian 32-bit words: the first symbol of a word is data[0:8] (header
DWords are then directly the received words). The PHY words (first symbol on data[31:24]) are
converted at the core boundary.
"""

from migen import *

from litex.gen import *

# Helpers ------------------------------------------------------------------------------------------

def swap_data(data):
    """PHY word <-> core word (symbols order swap)."""
    return Cat(data[24:32], data[16:24], data[8:16], data[0:8])

def swap_ctrl(ctrl):
    return Cat(ctrl[3], ctrl[2], ctrl[1], ctrl[0])

def framing_word(k):
    """Framing Ordered Set word (core order): 3 K symbols followed by EPF."""
    return (k << 0) | (k << 8) | (k << 16) | (EPF << 24)

def seq_add(seq, n, width=3):
    return (seq + n) & ((1 << width) - 1)

# Symbols (6.3.5) ----------------------------------------------------------------------------------

SHP = 0xfb # K27.7: Start Header Packet.
SDP = 0x5c # K28.2: Start Data Packet Payload.
EDB = 0x7c # K28.3: End Bad.
SLC = 0xfe # K30.7: Start Link Command.
END = 0xfd # K29.7: End.
EPF = 0xf7 # K23.7: End Packet Framing.

FRAMING_SHP = framing_word(SHP)
FRAMING_SDP = framing_word(SDP)
FRAMING_SLC = framing_word(SLC)
FRAMING_END = framing_word(END)

# Link Commands (7.2.2.1) --------------------------------------------------------------------------

LCMD_LGOOD = 0x000 # + Header Sequence Number (0-7).
LCMD_LCRD  = 0x080 # + Credit index (A-D: 0-3).
LCMD_LRTY  = 0x100
LCMD_LBAD  = 0x180
LCMD_LGO_U = 0x200 # + Ux (1-3).
LCMD_LAU   = 0x280
LCMD_LXU   = 0x300
LCMD_LPMA  = 0x380
LCMD_LUP   = 0x400
LCMD_LDN   = 0x580

# Link Control Word (7.2.1.1.1) --------------------------------------------------------------------

LCW_DL = (1 << 9) # Delayed.

# Packet Types (8.2) -------------------------------------------------------------------------------

TYPE_LMP = 0x00
TYPE_TP  = 0x04
TYPE_DP  = 0x08
TYPE_ITP = 0x0c

# LMP Subtypes (8.4).
LMP_SUB_SET_LINK_FUNC = 1
LMP_SUB_U2_INACT      = 2
LMP_SUB_VENDOR_TEST   = 3
LMP_SUB_PORT_CAP      = 4
LMP_SUB_PORT_CFG      = 5
LMP_SUB_PORT_CFG_RESP = 6

# TP Subtypes (8.5).
TP_SUB_ACK       = 1
TP_SUB_NRDY      = 2
TP_SUB_ERDY      = 3
TP_SUB_STATUS    = 4
TP_SUB_STALL     = 5
TP_SUB_DEV_NOTIF = 6
TP_SUB_PING      = 7
TP_SUB_PING_RESP = 8

# Layouts ------------------------------------------------------------------------------------------

def phy_layout():
    """PHY symbol stream (data/ctrl words)."""
    return [("data", 32), ("ctrl", 4)]

def header_layout():
    """Header Packet (3 DWords, without CRC/Link Control Word)."""
    return [("dw0", 32), ("dw1", 32), ("dw2", 32)]

def dpp_layout():
    """Data Packet Payload words (bytes: valid bytes of the word, 1-4, 0 for a zero-length DPP)."""
    return [("data", 32), ("bytes", 3)]

# Header Fields (8.3/8.5/8.6) ----------------------------------------------------------------------

def field(value, width):
    """Fixed width field from a constant or a Signal (zero-extended/truncated)."""
    if isinstance(value, int):
        return C(value, width)
    if len(value) < width:
        return Cat(value, C(0, width - len(value)))
    return value[:width]

def tp_dw1(subtype, ept=0, direction=0, nump=0, seq=0, retry=0):
    """Transaction Packet DWord 1 (constants or Signals)."""
    return Cat(field(subtype, 4), C(0, 2), field(retry, 1), field(direction, 1), field(ept, 4),
        C(0, 4), field(nump, 5), field(seq, 5), C(0, 6))

def dph_dw1(seq, ept, direction, length, eob=0, setup=0):
    """Data Packet Header DWord 1."""
    return Cat(field(seq, 5), C(0, 1), field(eob, 1), field(direction, 1), field(ept, 4), C(0, 3),
        field(setup, 1), field(length, 16))
