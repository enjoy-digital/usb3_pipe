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

from litex.soc.interconnect import stream

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

# Byte enables (be): valid bytes of each word, contiguous from data[0:8], full (0b1111)
# except on the last word of a packet.

def be_to_bytes(be):
    """be to valid bytes of the word (1-4, 1-8 with an 8-bit be)."""
    r = 1
    for i in range(1, len(be)):
        r = Mux(be[i], i + 1, r)
    return r

def bytes_to_be(nbytes):
    """Valid bytes of the word (1-4, 2 LSBs used: 0 = 4) to be."""
    return Mux(nbytes[0:2] == 1, 0b0001,
           Mux(nbytes[0:2] == 2, 0b0011,
           Mux(nbytes[0:2] == 3, 0b0111,
           0b1111)))

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
FRAMING_EDB = framing_word(EDB) # DPPABORT (nullified DPP).

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

# Transfer Type (TT) of the ACK TPs and DPHs in SuperSpeedPlus operation (DW1[14:12], 8.2).
TT_CONTROL     = 0b100
TT_ISOCHRONOUS = 0b101
TT_BULK        = 0b110
TT_INTERRUPT   = 0b111

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

def tx_header_layout():
    """Header Packet to transmit (dpp: followed by a Data Packet Payload)."""
    return header_layout() + [("dpp", 1)]

def dpp_layout(data_width=32):
    """Data Packet Payload words (bytes: valid bytes of the word, 1-4, 1-8 or 1-16 with 32, 64 or
    128-bit words, 0 for a zero-length DPP)."""
    return [("data", data_width), ("bytes", bits_for(data_width//8))]

def link_pairs_layout(n=2):
    """PHY Link Layer words pairs (USB3PHY link_words=2): words: 1 or 2 valid words, word 0 first
    (data[0:32]/ctrl[0:4]). n: words entries (USB3PHY link_words=4: 1-4 valid words)."""
    return [("data", 32*n), ("ctrl", 4*n), ("words", bits_for(n))]

class WordsWindow(LiteXModule):
    """Link words entries (link_pairs_layout(nwords)) -> window of the next nwords words (w[i]/c[i],
    w0/c0, w1/c1, n: 0-nwords available words), consume: words consumed this cycle (0-n)."""
    def __init__(self, nwords=2):
        self.sink    = sink = stream.Endpoint(link_pairs_layout(nwords))
        self.n       = Signal(bits_for(nwords))           # o
        self.w       = [Signal(32) for _ in range(nwords)] # o
        self.c       = [Signal(4)  for _ in range(nwords)] # o
        self.consume = Signal(bits_for(nwords))           # i
        self.w0, self.c0 = self.w[0], self.c[0]
        self.w1, self.c1 = self.w[1], self.c[1]

        # # #

        size  = 2*nwords
        data  = [Signal(32, reset_less=True) for _ in range(size)]
        ctrl  = [Signal(4,  reset_less=True) for _ in range(size)]
        level = Signal(bits_for(size))
        rem   = Signal(bits_for(size))
        self.comb += [
            [self.w[i].eq(data[i]) for i in range(nwords)],
            [self.c[i].eq(ctrl[i]) for i in range(nwords)],
            # level - consume: candidates computed from the registered level, selected by consume
            # (timings: consume late).
            Case(self.consume, {c: rem.eq(level - c) for c in range(nwords + 1)}),
            sink.ready.eq(rem <= nwords),
        ]
        accept = Signal()
        self.comb += accept.eq(sink.valid & sink.ready)
        for i in range(size):
            shifted_d = Signal(32)
            shifted_c = Signal(4)
            self.comb += Case(self.consume, {c: [
                shifted_d.eq(data[i + c] if (i + c) < size else 0),
                shifted_c.eq(ctrl[i + c] if (i + c) < size else 0)] for c in range(nwords + 1)})
            # Slot i written by the sink word j at rem + j == i (j < sink.words), else shifted.
            stmt  = [data[i].eq(shifted_d), ctrl[i].eq(shifted_c)]
            for j in reversed(range(min(nwords, i + 1))):
                cond = accept & (rem == (i - j)) & ((sink.words > j) if j > 0 else 1)
                stmt = If(cond,
                    data[i].eq(sink.data[32*j:32*(j + 1)]), ctrl[i].eq(sink.ctrl[4*j:4*(j + 1)])
                ).Else(*([stmt] if isinstance(stmt, If) else stmt))
            self.sync += stmt
        level_next = Signal(bits_for(size))
        self.comb += level_next.eq(rem + Mux(accept, sink.words, 0))
        self.sync += [
            level.eq(level_next),
            # Available words registered (timings).
            self.n.eq(Mux(level_next >= nwords, nwords, level_next)),
        ]

def bytes_to_be_n(nbytes, n):
    """Valid bytes of the word (1-n, 0 = n) to be (n bits)."""
    cases = {b: C((1 << b) - 1, n) for b in range(1, n)}
    return Array([C((1 << n) - 1, n)] + [cases[b] for b in range(1, n)] +
        [C((1 << n) - 1, n)]*(2**len(nbytes) - n))[nbytes]

def endpoint_layout(data_width=32):
    """Endpoints user streams: 32/64-bit words (first byte on data[0:8]), be: valid bytes."""
    return [("data", data_width), ("be", data_width//8)]

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

class HeaderFields:
    """Received Header Packet fields (TP/DPH)."""
    def __init__(self, dw0, dw1, dw2):
        self.type      = dw0[0:5]
        # TP.
        self.subtype   = dw1[0:4]
        self.retry     = dw1[6]
        self.direction = dw1[7]
        self.ept       = dw1[8:12]
        self.nump      = dw1[16:21]
        self.tp_seq    = dw1[21:26]
        # DPH.
        self.dp_seq    = dw1[0:5]
        self.setup     = dw1[15]
        self.length    = dw1[16:32]
