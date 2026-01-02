#
# This file is part of USB3-PIPE project.
#
# CRC assortment (LiteX/Migen translation)
#
# Copyright (C) 2009 OutputLogic.com
# Reformatting / adaptation by Marshall H., 2013
# This source file may be used and distributed without restriction
# provided that this copyright statement is not removed from the file
# and that any derivative work contains the original copyright notice
# and the associated disclaimer.
#
# SPDX-License-Identifier: BSD-2-Clause (matches "used and distributed without restriction" intent)
#

from migen import *

from litex.gen import *


# -------------------------------------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------------------------------------

def _sl(v, hi, lo):
    """Verilog-style inclusive slice v[hi:lo] -> Migen slice v[lo:hi+1]."""
    return v[lo:hi + 1]

def _rx(v):
    """Reduction XOR."""
    return Reduce("XOR", v)

def _rev_wire_from_di(di, width):
    """
    Verilog had: d = {di[0], di[1], ..., di[width-1]}
    => d[width-1] = di[0], d[0] = di[width-1] (bit-reversal).
    Implement as: d = Cat(di[width-1], ..., di[0]) so d[0]=di[width-1].
    """
    bits = [di[i] for i in range(width)]
    return Cat(*reversed(bits))

def _verilog_concat_bits_msb_first(bits_lsb_indexable):
    """
    Build a vector equivalent to Verilog {x[0], x[1], ..., x[N-1]}.
    In Verilog, x[0] becomes MSB. In Migen Cat(), first arg is LSB, so reverse.
    """
    n = len(bits_lsb_indexable)
    return Cat(*[bits_lsb_indexable[n - 1 - i] for i in range(n)])


# -------------------------------------------------------------------------------------------------
# CRC-5 for Command Words / Link Control Word (combinational)
# -------------------------------------------------------------------------------------------------

class USB3CRCCW(LiteXModule):
    def __init__(self):
        self.di      = Signal(11)
        self.crc_out = Signal(5)

        d = _rev_wire_from_di(self.di, 11)   # matches Verilog d mapping
        c = Const(0x1F, 5)

        q0 = _rx(_sl(d, 10, 9)) ^ _rx(_sl(d, 6, 5)) ^ d[3] ^ d[0] ^ c[0] ^ _rx(_sl(c, 4, 3))
        q1 = d[10] ^ _rx(_sl(d, 7, 6)) ^ d[4] ^ d[1] ^ _rx(_sl(c, 1, 0)) ^ c[4]
        q2 = _rx(_sl(d, 10, 7)) ^ d[6] ^ _rx(_sl(d, 3, 2)) ^ d[0] ^ _rx(_sl(c, 4, 0))
        q3 = _rx(_sl(d, 10, 7)) ^ _rx(_sl(d, 4, 3)) ^ d[1] ^ _rx(_sl(c, 4, 1))
        q4 = _rx(_sl(d, 10, 8)) ^ _rx(_sl(d, 5, 4)) ^ d[2] ^ _rx(_sl(c, 4, 2))

        q = Cat(q4, q3, q2, q1, q0)  # LSB first => q[0]=q0, etc.
        self.comb += self.crc_out.eq(~q)


# -------------------------------------------------------------------------------------------------
# CRC-16 for Header Packets (stateful)
# -------------------------------------------------------------------------------------------------

class USB3CRCHP(LiteXModule):
    def __init__(self):
        self.di      = Signal(32)
        self.crc_en  = Signal()
        self.crc_out = Signal(16)
        self.rst     = Signal()
        self.clk     = Signal()

        # Local clock domain driven by ports.
        self.clock_domains.cd_crc = ClockDomain()
        self.comb += [
            self.cd_crc.clk.eq(self.clk),
            self.cd_crc.rst.eq(self.rst),
        ]

        q = Signal(16, reset=0xFFFF)
        d = Signal(32)
        self.comb += d.eq(_rev_wire_from_di(self.di, 32))

        c = Array(Signal() for _ in range(16))

        # Output: ~{q[0], q[1], ..., q[15]} (Verilog ordering)
        q_bits = [q[i] for i in range(16)]
        self.comb += self.crc_out.eq(~_verilog_concat_bits_msb_first(q_bits))

        # Combinational next-state equations (verbatim ported).
        self.comb += [
            c[0].eq(q[4] ^ q[5] ^ q[7] ^ q[10] ^ q[12] ^ q[13] ^ q[15] ^
                    d[0] ^ d[4] ^ d[8] ^ d[12] ^ d[13] ^ d[15] ^ d[20] ^ d[21] ^ d[23] ^ d[26] ^ d[28] ^ d[29] ^ d[31]),
            c[1].eq(q[0] ^ q[4] ^ q[6] ^ q[7] ^ q[8] ^ q[10] ^ q[11] ^ q[12] ^ q[14] ^ q[15] ^
                    d[0] ^ d[1] ^ d[4] ^ d[5] ^ d[8] ^ d[9] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[20] ^ d[22] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[28] ^ d[30] ^ d[31]),
            c[2].eq(q[0] ^ q[1] ^ q[5] ^ q[7] ^ q[8] ^ q[9] ^ q[11] ^ q[12] ^ q[13] ^ q[15] ^
                    d[1] ^ d[2] ^ d[5] ^ d[6] ^ d[9] ^ d[10] ^ d[13] ^ d[15] ^ d[16] ^ d[17] ^ d[21] ^ d[23] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[29] ^ d[31]),
            c[3].eq(q[0] ^ q[1] ^ q[2] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[14] ^ q[15] ^
                    d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[30] ^ d[31]),
            c[4].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[10] ^ q[15] ^
                    d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[31]),
            c[5].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[10] ^ q[11] ^
                    d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[27]),
            c[6].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[7] ^ q[8] ^ q[9] ^ q[10] ^ q[11] ^ q[12] ^
                    d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28]),
            c[7].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[8] ^ q[9] ^ q[10] ^ q[11] ^ q[12] ^ q[13] ^
                    d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29]),
            c[8].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[9] ^ q[10] ^ q[11] ^ q[12] ^ q[13] ^ q[14] ^
                    d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30]),
            c[9].eq(q[0] ^ q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[10] ^ q[11] ^ q[12] ^ q[13] ^ q[14] ^ q[15] ^
                    d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[14] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]),
            c[10].eq(q[1] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[11] ^ q[12] ^ q[13] ^ q[14] ^ q[15] ^
                     d[7] ^ d[9] ^ d[10] ^ d[11] ^ d[13] ^ d[14] ^ d[15] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]),
            c[11].eq(q[0] ^ q[2] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[10] ^ q[12] ^ q[13] ^ q[14] ^ q[15] ^
                     d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[28] ^ d[29] ^ d[30] ^ d[31]),
            c[12].eq(q[0] ^ q[1] ^ q[3] ^ q[6] ^ q[8] ^ q[9] ^ q[11] ^ q[12] ^ q[14] ^
                     d[0] ^ d[4] ^ d[8] ^ d[9] ^ d[11] ^ d[16] ^ d[17] ^ d[19] ^ d[22] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[30]),
            c[13].eq(q[1] ^ q[2] ^ q[4] ^ q[7] ^ q[9] ^ q[10] ^ q[12] ^ q[13] ^ q[15] ^
                     d[1] ^ d[5] ^ d[9] ^ d[10] ^ d[12] ^ d[17] ^ d[18] ^ d[20] ^ d[23] ^ d[25] ^ d[26] ^ d[28] ^ d[29] ^ d[31]),
            c[14].eq(q[2] ^ q[3] ^ q[5] ^ q[8] ^ q[10] ^ q[11] ^ q[13] ^ q[14] ^
                     d[2] ^ d[6] ^ d[10] ^ d[11] ^ d[13] ^ d[18] ^ d[19] ^ d[21] ^ d[24] ^ d[26] ^ d[27] ^ d[29] ^ d[30]),
            c[15].eq(q[3] ^ q[4] ^ q[6] ^ q[9] ^ q[11] ^ q[12] ^ q[14] ^ q[15] ^
                     d[3] ^ d[7] ^ d[11] ^ d[12] ^ d[14] ^ d[19] ^ d[20] ^ d[22] ^ d[25] ^ d[27] ^ d[28] ^ d[30] ^ d[31]),
        ]

        # State update.
        self.sync.crc += [
            If(self.rst,
                q.eq(0xFFFF)
            ).Else(
                If(self.crc_en,
                    q.eq(Cat(*[c[i] for i in range(16)]))  # c[0] is LSB
                )
            )
        ]


# -------------------------------------------------------------------------------------------------
# CRC-32 for Data Packet Payloads (stateful)
# -------------------------------------------------------------------------------------------------

class USB3CRCDPP32(LiteXModule):
    def __init__(self):
        self.di      = Signal(32)
        self.lfsr_q  = Signal(32)
        self.crc_en  = Signal()
        self.crc_out = Signal(32)
        self.rst     = Signal()
        self.clk     = Signal()

        self.clock_domains.cd_crc = ClockDomain()
        self.comb += [
            self.cd_crc.clk.eq(self.clk),
            self.cd_crc.rst.eq(self.rst),
        ]

        q = Signal(32, reset=0xFFFFFFFF)
        d = Signal(32)
        self.comb += d.eq(_rev_wire_from_di(self.di, 32))

        # lfsr_q / crc_out
        self.comb += [
            self.lfsr_q.eq(q),
            self.crc_out.eq(~_verilog_concat_bits_msb_first([q[i] for i in range(32)])),
        ]

        c = Array(Signal() for _ in range(32))

        # Combinational next-state equations (ported verbatim).
        self.comb += [
            c[0].eq(q[0] ^ q[6] ^ q[9] ^ q[10] ^ q[12] ^ q[16] ^ q[24] ^ q[25] ^ q[26] ^ q[28] ^ q[29] ^ q[30] ^ q[31] ^
                    d[0] ^ d[6] ^ d[9] ^ d[10] ^ d[12] ^ d[16] ^ d[24] ^ d[25] ^ d[26] ^ d[28] ^ d[29] ^ d[30] ^ d[31]),
            c[1].eq(q[0] ^ q[1] ^ q[6] ^ q[7] ^ q[9] ^ q[11] ^ q[12] ^ q[13] ^ q[16] ^ q[17] ^ q[24] ^ q[27] ^ q[28] ^
                    d[0] ^ d[1] ^ d[6] ^ d[7] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[17] ^ d[24] ^ d[27] ^ d[28]),
            c[2].eq(q[0] ^ q[1] ^ q[2] ^ q[6] ^ q[7] ^ q[8] ^ q[9] ^ q[13] ^ q[14] ^ q[16] ^ q[17] ^ q[18] ^ q[24] ^ q[26] ^ q[30] ^ q[31] ^
                    d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[16] ^ d[17] ^ d[18] ^ d[24] ^ d[26] ^ d[30] ^ d[31]),
            c[3].eq(q[1] ^ q[2] ^ q[3] ^ q[7] ^ q[8] ^ q[9] ^ q[10] ^ q[14] ^ q[15] ^ q[17] ^ q[18] ^ q[19] ^ q[25] ^ q[27] ^ q[31] ^
                    d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[17] ^ d[18] ^ d[19] ^ d[25] ^ d[27] ^ d[31]),
            c[4].eq(q[0] ^ q[2] ^ q[3] ^ q[4] ^ q[6] ^ q[8] ^ q[11] ^ q[12] ^ q[15] ^ q[18] ^ q[19] ^ q[20] ^ q[24] ^ q[25] ^ q[29] ^ q[30] ^ q[31] ^
                    d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[11] ^ d[12] ^ d[15] ^ d[18] ^ d[19] ^ d[20] ^ d[24] ^ d[25] ^ d[29] ^ d[30] ^ d[31]),
            c[5].eq(q[0] ^ q[1] ^ q[3] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[10] ^ q[13] ^ q[19] ^ q[20] ^ q[21] ^ q[24] ^ q[28] ^ q[29] ^
                    d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13] ^ d[19] ^ d[20] ^ d[21] ^ d[24] ^ d[28] ^ d[29]),
            c[6].eq(q[1] ^ q[2] ^ q[4] ^ q[5] ^ q[6] ^ q[7] ^ q[8] ^ q[11] ^ q[14] ^ q[20] ^ q[21] ^ q[22] ^ q[25] ^ q[29] ^ q[30] ^
                    d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14] ^ d[20] ^ d[21] ^ d[22] ^ d[25] ^ d[29] ^ d[30]),
            c[7].eq(q[0] ^ q[2] ^ q[3] ^ q[5] ^ q[7] ^ q[8] ^ q[10] ^ q[15] ^ q[16] ^ q[21] ^ q[22] ^ q[23] ^ q[24] ^ q[25] ^ q[28] ^ q[29] ^
                    d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ d[8] ^ d[10] ^ d[15] ^ d[16] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[28] ^ d[29]),
            c[8].eq(q[0] ^ q[1] ^ q[3] ^ q[4] ^ q[8] ^ q[10] ^ q[11] ^ q[12] ^ q[17] ^ q[22] ^ q[23] ^ q[28] ^ q[31] ^
                    d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[17] ^ d[22] ^ d[23] ^ d[28] ^ d[31]),
            c[9].eq(q[1] ^ q[2] ^ q[4] ^ q[5] ^ q[9] ^ q[11] ^ q[12] ^ q[13] ^ q[18] ^ q[23] ^ q[24] ^ q[29] ^
                    d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[18] ^ d[23] ^ d[24] ^ d[29]),
            c[10].eq(q[0] ^ q[2] ^ q[3] ^ q[5] ^ q[9] ^ q[13] ^ q[14] ^ q[16] ^ q[19] ^ q[26] ^ q[28] ^ q[29] ^ q[31] ^
                     d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[9] ^ d[13] ^ d[14] ^ d[16] ^ d[19] ^ d[26] ^ d[28] ^ d[29] ^ d[31]),
            c[11].eq(q[0] ^ q[1] ^ q[3] ^ q[4] ^ q[9] ^ q[12] ^ q[14] ^ q[15] ^ q[16] ^ q[17] ^ q[20] ^ q[24] ^ q[25] ^ q[26] ^ q[27] ^ q[28] ^ q[31] ^
                     d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[9] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[31]),
            c[12].eq(q[0] ^ q[1] ^ q[2] ^ q[4] ^ q[5] ^ q[6] ^ q[9] ^ q[12] ^ q[13] ^ q[15] ^ q[17] ^ q[18] ^ q[21] ^ q[24] ^ q[27] ^ q[30] ^ q[31] ^
                     d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[9] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[18] ^ d[21] ^ d[24] ^ d[27] ^ d[30] ^ d[31]),
            c[13].eq(q[1] ^ q[2] ^ q[3] ^ q[5] ^ q[6] ^ q[7] ^ q[10] ^ q[13] ^ q[14] ^ q[16] ^ q[18] ^ q[19] ^ q[22] ^ q[25] ^ q[28] ^ q[31] ^
                     d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[22] ^ d[25] ^ d[28] ^ d[31]),
            c[14].eq(q[2] ^ q[3] ^ q[4] ^ q[6] ^ q[7] ^ q[8] ^ q[11] ^ q[14] ^ q[15] ^ q[17] ^ q[19] ^ q[20] ^ q[23] ^ q[26] ^ q[29] ^
                     d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14] ^ d[15] ^ d[17] ^ d[19] ^ d[20] ^ d[23] ^ d[26] ^ d[29]),
            c[15].eq(q[3] ^ q[4] ^ q[5] ^ q[7] ^ q[8] ^ q[9] ^ q[12] ^ q[15] ^ q[16] ^ q[18] ^ q[20] ^ q[21] ^ q[24] ^ q[27] ^ q[30] ^
                     d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[12] ^ d[15] ^ d[16] ^ d[18] ^ d[20] ^ d[21] ^ d[24] ^ d[27] ^ d[30]),
            c[16].eq(q[0] ^ q[4] ^ q[5] ^ q[8] ^ q[12] ^ q[13] ^ q[17] ^ q[19] ^ q[21] ^ q[22] ^ q[24] ^ q[26] ^ q[29] ^ q[30] ^
                     d[0] ^ d[4] ^ d[5] ^ d[8] ^ d[12] ^ d[13] ^ d[17] ^ d[19] ^ d[21] ^ d[22] ^ d[24] ^ d[26] ^ d[29] ^ d[30]),
            c[17].eq(q[1] ^ q[5] ^ q[6] ^ q[9] ^ q[13] ^ q[14] ^ q[18] ^ q[20] ^ q[22] ^ q[23] ^ q[25] ^ q[27] ^ q[30] ^ q[31] ^
                     d[1] ^ d[5] ^ d[6] ^ d[9] ^ d[13] ^ d[14] ^ d[18] ^ d[20] ^ d[22] ^ d[23] ^ d[25] ^ d[27] ^ d[30] ^ d[31]),
            c[18].eq(q[2] ^ q[6] ^ q[7] ^ q[10] ^ q[14] ^ q[15] ^ q[19] ^ q[21] ^ q[23] ^ q[24] ^ q[26] ^ q[28] ^ q[31] ^
                     d[2] ^ d[6] ^ d[7] ^ d[10] ^ d[14] ^ d[15] ^ d[19] ^ d[21] ^ d[23] ^ d[24] ^ d[26] ^ d[28] ^ d[31]),
            c[19].eq(q[3] ^ q[7] ^ q[8] ^ q[11] ^ q[15] ^ q[16] ^ q[20] ^ q[22] ^ q[24] ^ q[25] ^ q[27] ^ q[29] ^
                     d[3] ^ d[7] ^ d[8] ^ d[11] ^ d[15] ^ d[16] ^ d[20] ^ d[22] ^ d[24] ^ d[25] ^ d[27] ^ d[29]),
            c[20].eq(q[4] ^ q[8] ^ q[9] ^ q[12] ^ q[16] ^ q[17] ^ q[21] ^ q[23] ^ q[25] ^ q[26] ^ q[28] ^ q[30] ^
                     d[4] ^ d[8] ^ d[9] ^ d[12] ^ d[16] ^ d[17] ^ d[21] ^ d[23] ^ d[25] ^ d[26] ^ d[28] ^ d[30]),
            c[21].eq(q[5] ^ q[9] ^ q[10] ^ q[13] ^ q[17] ^ q[18] ^ q[22] ^ q[24] ^ q[26] ^ q[27] ^ q[29] ^ q[31] ^
                     d[5] ^ d[9] ^ d[10] ^ d[13] ^ d[17] ^ d[18] ^ d[22] ^ d[24] ^ d[26] ^ d[27] ^ d[29] ^ d[31]),
            c[22].eq(q[0] ^ q[9] ^ q[11] ^ q[12] ^ q[14] ^ q[16] ^ q[18] ^ q[19] ^ q[23] ^ q[24] ^ q[26] ^ q[27] ^ q[29] ^ q[31] ^
                     d[0] ^ d[9] ^ d[11] ^ d[12] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[29] ^ d[31]),
            c[23].eq(q[0] ^ q[1] ^ q[6] ^ q[9] ^ q[13] ^ q[15] ^ q[16] ^ q[17] ^ q[19] ^ q[20] ^ q[26] ^ q[27] ^ q[29] ^ q[31] ^
                     d[0] ^ d[1] ^ d[6] ^ d[9] ^ d[13] ^ d[15] ^ d[16] ^ d[17] ^ d[19] ^ d[20] ^ d[26] ^ d[27] ^ d[29] ^ d[31]),
            c[24].eq(q[1] ^ q[2] ^ q[7] ^ q[10] ^ q[14] ^ q[16] ^ q[17] ^ q[18] ^ q[20] ^ q[21] ^ q[27] ^ q[28] ^ q[30] ^
                     d[1] ^ d[2] ^ d[7] ^ d[10] ^ d[14] ^ d[16] ^ d[17] ^ d[18] ^ d[20] ^ d[21] ^ d[27] ^ d[28] ^ d[30]),
            c[25].eq(q[2] ^ q[3] ^ q[8] ^ q[11] ^ q[15] ^ q[17] ^ q[18] ^ q[19] ^ q[21] ^ q[22] ^ q[28] ^ q[29] ^ q[31] ^
                     d[2] ^ d[3] ^ d[8] ^ d[11] ^ d[15] ^ d[17] ^ d[18] ^ d[19] ^ d[21] ^ d[22] ^ d[28] ^ d[29] ^ d[31]),
            c[26].eq(q[0] ^ q[3] ^ q[4] ^ q[6] ^ q[10] ^ q[18] ^ q[19] ^ q[20] ^ q[22] ^ q[23] ^ q[24] ^ q[25] ^ q[26] ^ q[28] ^ q[31] ^
                     d[0] ^ d[3] ^ d[4] ^ d[6] ^ d[10] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[28] ^ d[31]),
            c[27].eq(q[1] ^ q[4] ^ q[5] ^ q[7] ^ q[11] ^ q[19] ^ q[20] ^ q[21] ^ q[23] ^ q[24] ^ q[25] ^ q[26] ^ q[27] ^ q[29] ^
                     d[1] ^ d[4] ^ d[5] ^ d[7] ^ d[11] ^ d[19] ^ d[20] ^ d[21] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[29]),
            c[28].eq(q[2] ^ q[5] ^ q[6] ^ q[8] ^ q[12] ^ q[20] ^ q[21] ^ q[22] ^ q[24] ^ q[25] ^ q[26] ^ q[27] ^ q[28] ^ q[30] ^
                     d[2] ^ d[5] ^ d[6] ^ d[8] ^ d[12] ^ d[20] ^ d[21] ^ d[22] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[30]),
            c[29].eq(q[3] ^ q[6] ^ q[7] ^ q[9] ^ q[13] ^ q[21] ^ q[22] ^ q[23] ^ q[25] ^ q[26] ^ q[27] ^ q[28] ^ q[29] ^ q[31] ^
                     d[3] ^ d[6] ^ d[7] ^ d[9] ^ d[13] ^ d[21] ^ d[22] ^ d[23] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[31]),
            c[30].eq(q[4] ^ q[7] ^ q[8] ^ q[10] ^ q[14] ^ q[22] ^ q[23] ^ q[24] ^ q[26] ^ q[27] ^ q[28] ^ q[29] ^ q[30] ^
                     d[4] ^ d[7] ^ d[8] ^ d[10] ^ d[14] ^ d[22] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30]),
            c[31].eq(q[5] ^ q[8] ^ q[9] ^ q[11] ^ q[15] ^ q[23] ^ q[24] ^ q[25] ^ q[27] ^ q[28] ^ q[29] ^ q[30] ^ q[31] ^
                     d[5] ^ d[8] ^ d[9] ^ d[11] ^ d[15] ^ d[23] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31]),
        ]

        self.sync.crc += [
            If(self.rst,
                q.eq(0xFFFFFFFF)
            ).Else(
                If(self.crc_en,
                    q.eq(Cat(*[c[i] for i in range(32)]))
                )
            )
        ]


# -------------------------------------------------------------------------------------------------
# CRC-32 combinational transforms for partial words (24/16/8)
# These take current LFSR state q[31:0] as input and produce crc_out.
# clk/rst ports exist in the original, but are unused; kept here for signature compatibility.
# -------------------------------------------------------------------------------------------------

class USB3CRCDPP24(LiteXModule):
    def __init__(self):
        self.di      = Signal(24)
        self.q       = Signal(32)
        self.crc_out = Signal(32)
        self.rst     = Signal()
        self.clk     = Signal()

        d = Signal(24)
        self.comb += d.eq(_rev_wire_from_di(self.di, 24))

        c = Array(Signal() for _ in range(32))

        self.comb += self.crc_out.eq(~_verilog_concat_bits_msb_first([c[i] for i in range(32)]))

        # Equations (ported verbatim).
        self.comb += [
            c[0].eq(self.q[8] ^ self.q[14] ^ self.q[17] ^ self.q[18] ^ self.q[20] ^ self.q[24] ^ d[0] ^ d[6] ^ d[9] ^ d[10] ^ d[12] ^ d[16]),
            c[1].eq(self.q[8] ^ self.q[9] ^ self.q[14] ^ self.q[15] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[21] ^ self.q[24] ^ self.q[25] ^ d[0] ^ d[1] ^ d[6] ^ d[7] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[17]),
            c[2].eq(self.q[8] ^ self.q[9] ^ self.q[10] ^ self.q[14] ^ self.q[15] ^ self.q[16] ^ self.q[17] ^ self.q[21] ^ self.q[22] ^ self.q[24] ^ self.q[25] ^ self.q[26] ^ d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[13] ^ d[14] ^ d[16] ^ d[17] ^ d[18]),
            c[3].eq(self.q[9] ^ self.q[10] ^ self.q[11] ^ self.q[15] ^ self.q[16] ^ self.q[17] ^ self.q[18] ^ self.q[22] ^ self.q[23] ^ self.q[25] ^ self.q[26] ^ self.q[27] ^ d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[17] ^ d[18] ^ d[19]),
            c[4].eq(self.q[8] ^ self.q[10] ^ self.q[11] ^ self.q[12] ^ self.q[14] ^ self.q[16] ^ self.q[19] ^ self.q[20] ^ self.q[23] ^ self.q[26] ^ self.q[27] ^ self.q[28] ^ d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[11] ^ d[12] ^ d[15] ^ d[18] ^ d[19] ^ d[20]),
            c[5].eq(self.q[8] ^ self.q[9] ^ self.q[11] ^ self.q[12] ^ self.q[13] ^ self.q[14] ^ self.q[15] ^ self.q[18] ^ self.q[21] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13] ^ d[19] ^ d[20] ^ d[21]),
            c[6].eq(self.q[9] ^ self.q[10] ^ self.q[12] ^ self.q[13] ^ self.q[14] ^ self.q[15] ^ self.q[16] ^ self.q[19] ^ self.q[22] ^ self.q[28] ^ self.q[29] ^ self.q[30] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14] ^ d[20] ^ d[21] ^ d[22]),
            c[7].eq(self.q[8] ^ self.q[10] ^ self.q[11] ^ self.q[13] ^ self.q[15] ^ self.q[16] ^ self.q[18] ^ self.q[23] ^ self.q[24] ^ self.q[29] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ d[8] ^ d[10] ^ d[15] ^ d[16] ^ d[21] ^ d[22] ^ d[23]),
            c[8].eq(self.q[8] ^ self.q[9] ^ self.q[11] ^ self.q[12] ^ self.q[16] ^ self.q[18] ^ self.q[19] ^ self.q[20] ^ self.q[25] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[17] ^ d[22] ^ d[23]),
            c[9].eq(self.q[9] ^ self.q[10] ^ self.q[12] ^ self.q[13] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[21] ^ self.q[26] ^ self.q[31] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[18] ^ d[23]),
            c[10].eq(self.q[8] ^ self.q[10] ^ self.q[11] ^ self.q[13] ^ self.q[17] ^ self.q[21] ^ self.q[22] ^ self.q[24] ^ self.q[27] ^ d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[9] ^ d[13] ^ d[14] ^ d[16] ^ d[19]),
            c[11].eq(self.q[8] ^ self.q[9] ^ self.q[11] ^ self.q[12] ^ self.q[17] ^ self.q[20] ^ self.q[22] ^ self.q[23] ^ self.q[24] ^ self.q[25] ^ self.q[28] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[9] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[20]),
            c[12].eq(self.q[8] ^ self.q[9] ^ self.q[10] ^ self.q[12] ^ self.q[13] ^ self.q[14] ^ self.q[17] ^ self.q[20] ^ self.q[21] ^ self.q[23] ^ self.q[25] ^ self.q[26] ^ self.q[29] ^ d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[9] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[18] ^ d[21]),
            c[13].eq(self.q[9] ^ self.q[10] ^ self.q[11] ^ self.q[13] ^ self.q[14] ^ self.q[15] ^ self.q[18] ^ self.q[21] ^ self.q[22] ^ self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[30] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[22]),
            c[14].eq(self.q[10] ^ self.q[11] ^ self.q[12] ^ self.q[14] ^ self.q[15] ^ self.q[16] ^ self.q[19] ^ self.q[22] ^ self.q[23] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ self.q[31] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14] ^ d[15] ^ d[17] ^ d[19] ^ d[20] ^ d[23]),
            c[15].eq(self.q[11] ^ self.q[12] ^ self.q[13] ^ self.q[15] ^ self.q[16] ^ self.q[17] ^ self.q[20] ^ self.q[23] ^ self.q[24] ^ self.q[26] ^ self.q[28] ^ self.q[29] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[12] ^ d[15] ^ d[16] ^ d[18] ^ d[20] ^ d[21]),
            c[16].eq(self.q[8] ^ self.q[12] ^ self.q[13] ^ self.q[16] ^ self.q[20] ^ self.q[21] ^ self.q[25] ^ self.q[27] ^ self.q[29] ^ self.q[30] ^ d[0] ^ d[4] ^ d[5] ^ d[8] ^ d[12] ^ d[13] ^ d[17] ^ d[19] ^ d[21] ^ d[22]),
            c[17].eq(self.q[9] ^ self.q[13] ^ self.q[14] ^ self.q[17] ^ self.q[21] ^ self.q[22] ^ self.q[26] ^ self.q[28] ^ self.q[30] ^ self.q[31] ^ d[1] ^ d[5] ^ d[6] ^ d[9] ^ d[13] ^ d[14] ^ d[18] ^ d[20] ^ d[22] ^ d[23]),
            c[18].eq(self.q[10] ^ self.q[14] ^ self.q[15] ^ self.q[18] ^ self.q[22] ^ self.q[23] ^ self.q[27] ^ self.q[29] ^ self.q[31] ^ d[2] ^ d[6] ^ d[7] ^ d[10] ^ d[14] ^ d[15] ^ d[19] ^ d[21] ^ d[23]),
            c[19].eq(self.q[11] ^ self.q[15] ^ self.q[16] ^ self.q[19] ^ self.q[23] ^ self.q[24] ^ self.q[28] ^ self.q[30] ^ d[3] ^ d[7] ^ d[8] ^ d[11] ^ d[15] ^ d[16] ^ d[20] ^ d[22]),
            c[20].eq(self.q[12] ^ self.q[16] ^ self.q[17] ^ self.q[20] ^ self.q[24] ^ self.q[25] ^ self.q[29] ^ self.q[31] ^ d[4] ^ d[8] ^ d[9] ^ d[12] ^ d[16] ^ d[17] ^ d[21] ^ d[23]),
            c[21].eq(self.q[13] ^ self.q[17] ^ self.q[18] ^ self.q[21] ^ self.q[25] ^ self.q[26] ^ self.q[30] ^ d[5] ^ d[9] ^ d[10] ^ d[13] ^ d[17] ^ d[18] ^ d[22]),
            c[22].eq(self.q[8] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[22] ^ self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[31] ^ d[0] ^ d[9] ^ d[11] ^ d[12] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[23]),
            c[23].eq(self.q[8] ^ self.q[9] ^ self.q[14] ^ self.q[17] ^ self.q[21] ^ self.q[23] ^ self.q[24] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ d[0] ^ d[1] ^ d[6] ^ d[9] ^ d[13] ^ d[15] ^ d[16] ^ d[17] ^ d[19] ^ d[20]),
            c[24].eq(self.q[0] ^ self.q[9] ^ self.q[10] ^ self.q[15] ^ self.q[18] ^ self.q[22] ^ self.q[24] ^ self.q[25] ^ self.q[26] ^ self.q[28] ^ self.q[29] ^ d[1] ^ d[2] ^ d[7] ^ d[10] ^ d[14] ^ d[16] ^ d[17] ^ d[18] ^ d[20] ^ d[21]),
            c[25].eq(self.q[1] ^ self.q[10] ^ self.q[11] ^ self.q[16] ^ self.q[19] ^ self.q[23] ^ self.q[25] ^ self.q[26] ^ self.q[27] ^ self.q[29] ^ self.q[30] ^ d[2] ^ d[3] ^ d[8] ^ d[11] ^ d[15] ^ d[17] ^ d[18] ^ d[19] ^ d[21] ^ d[22]),
            c[26].eq(self.q[2] ^ self.q[8] ^ self.q[11] ^ self.q[12] ^ self.q[14] ^ self.q[18] ^ self.q[26] ^ self.q[27] ^ self.q[28] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[3] ^ d[4] ^ d[6] ^ d[10] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[23]),
            c[27].eq(self.q[3] ^ self.q[9] ^ self.q[12] ^ self.q[13] ^ self.q[15] ^ self.q[19] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ self.q[31] ^ d[1] ^ d[4] ^ d[5] ^ d[7] ^ d[11] ^ d[19] ^ d[20] ^ d[21] ^ d[23]),
            c[28].eq(self.q[4] ^ self.q[10] ^ self.q[13] ^ self.q[14] ^ self.q[16] ^ self.q[20] ^ self.q[28] ^ self.q[29] ^ self.q[30] ^ d[2] ^ d[5] ^ d[6] ^ d[8] ^ d[12] ^ d[20] ^ d[21] ^ d[22]),
            c[29].eq(self.q[5] ^ self.q[11] ^ self.q[14] ^ self.q[15] ^ self.q[17] ^ self.q[21] ^ self.q[29] ^ self.q[30] ^ self.q[31] ^ d[3] ^ d[6] ^ d[7] ^ d[9] ^ d[13] ^ d[21] ^ d[22] ^ d[23]),
            c[30].eq(self.q[6] ^ self.q[12] ^ self.q[15] ^ self.q[16] ^ self.q[18] ^ self.q[22] ^ self.q[30] ^ self.q[31] ^ d[4] ^ d[7] ^ d[8] ^ d[10] ^ d[14] ^ d[22] ^ d[23]),
            c[31].eq(self.q[7] ^ self.q[13] ^ self.q[16] ^ self.q[17] ^ self.q[19] ^ self.q[23] ^ self.q[31] ^ d[5] ^ d[8] ^ d[9] ^ d[11] ^ d[15] ^ d[23]),
        ]


class USB3CRCDPP16(LiteXModule):
    def __init__(self):
        self.di      = Signal(16)
        self.q       = Signal(32)
        self.crc_out = Signal(32)
        self.rst     = Signal()
        self.clk     = Signal()

        d = Signal(16)
        self.comb += d.eq(_rev_wire_from_di(self.di, 16))

        c = Array(Signal() for _ in range(32))
        self.comb += self.crc_out.eq(~_verilog_concat_bits_msb_first([c[i] for i in range(32)]))

        self.comb += [
            c[0].eq(self.q[16] ^ self.q[22] ^ self.q[25] ^ self.q[26] ^ self.q[28] ^ d[0] ^ d[6] ^ d[9] ^ d[10] ^ d[12]),
            c[1].eq(self.q[16] ^ self.q[17] ^ self.q[22] ^ self.q[23] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ d[0] ^ d[1] ^ d[6] ^ d[7] ^ d[9] ^ d[11] ^ d[12] ^ d[13]),
            c[2].eq(self.q[16] ^ self.q[17] ^ self.q[18] ^ self.q[22] ^ self.q[23] ^ self.q[24] ^ self.q[25] ^ self.q[29] ^ self.q[30] ^ d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[13] ^ d[14]),
            c[3].eq(self.q[17] ^ self.q[18] ^ self.q[19] ^ self.q[23] ^ self.q[24] ^ self.q[25] ^ self.q[26] ^ self.q[30] ^ self.q[31] ^ d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14] ^ d[15]),
            c[4].eq(self.q[16] ^ self.q[18] ^ self.q[19] ^ self.q[20] ^ self.q[22] ^ self.q[24] ^ self.q[27] ^ self.q[28] ^ self.q[31] ^ d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[11] ^ d[12] ^ d[15]),
            c[5].eq(self.q[16] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[21] ^ self.q[22] ^ self.q[23] ^ self.q[26] ^ self.q[29] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13]),
            c[6].eq(self.q[17] ^ self.q[18] ^ self.q[20] ^ self.q[21] ^ self.q[22] ^ self.q[23] ^ self.q[24] ^ self.q[27] ^ self.q[30] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14]),
            c[7].eq(self.q[16] ^ self.q[18] ^ self.q[19] ^ self.q[21] ^ self.q[23] ^ self.q[24] ^ self.q[26] ^ self.q[31] ^ d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ d[8] ^ d[10] ^ d[15]),
            c[8].eq(self.q[16] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[28] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[8] ^ d[10] ^ d[11] ^ d[12]),
            c[9].eq(self.q[17] ^ self.q[18] ^ self.q[20] ^ self.q[21] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[9] ^ d[11] ^ d[12] ^ d[13]),
            c[10].eq(self.q[16] ^ self.q[18] ^ self.q[19] ^ self.q[21] ^ self.q[25] ^ self.q[29] ^ self.q[30] ^ d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[9] ^ d[13] ^ d[14]),
            c[11].eq(self.q[16] ^ self.q[17] ^ self.q[19] ^ self.q[20] ^ self.q[25] ^ self.q[28] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[9] ^ d[12] ^ d[14] ^ d[15]),
            c[12].eq(self.q[16] ^ self.q[17] ^ self.q[18] ^ self.q[20] ^ self.q[21] ^ self.q[22] ^ self.q[25] ^ self.q[28] ^ self.q[29] ^ self.q[31] ^ d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[9] ^ d[12] ^ d[13] ^ d[15]),
            c[13].eq(self.q[17] ^ self.q[18] ^ self.q[19] ^ self.q[21] ^ self.q[22] ^ self.q[23] ^ self.q[26] ^ self.q[29] ^ self.q[30] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[10] ^ d[13] ^ d[14]),
            c[14].eq(self.q[18] ^ self.q[19] ^ self.q[20] ^ self.q[22] ^ self.q[23] ^ self.q[24] ^ self.q[27] ^ self.q[30] ^ self.q[31] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[11] ^ d[14] ^ d[15]),
            c[15].eq(self.q[19] ^ self.q[20] ^ self.q[21] ^ self.q[23] ^ self.q[24] ^ self.q[25] ^ self.q[28] ^ self.q[31] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[12] ^ d[15]),
            c[16].eq(self.q[0] ^ self.q[16] ^ self.q[20] ^ self.q[21] ^ self.q[24] ^ self.q[28] ^ self.q[29] ^ d[0] ^ d[4] ^ d[5] ^ d[8] ^ d[12] ^ d[13]),
            c[17].eq(self.q[1] ^ self.q[17] ^ self.q[21] ^ self.q[22] ^ self.q[25] ^ self.q[29] ^ self.q[30] ^ d[1] ^ d[5] ^ d[6] ^ d[9] ^ d[13] ^ d[14]),
            c[18].eq(self.q[2] ^ self.q[18] ^ self.q[22] ^ self.q[23] ^ self.q[26] ^ self.q[30] ^ self.q[31] ^ d[2] ^ d[6] ^ d[7] ^ d[10] ^ d[14] ^ d[15]),
            c[19].eq(self.q[3] ^ self.q[19] ^ self.q[23] ^ self.q[24] ^ self.q[27] ^ self.q[31] ^ d[3] ^ d[7] ^ d[8] ^ d[11] ^ d[15]),
            c[20].eq(self.q[4] ^ self.q[20] ^ self.q[24] ^ self.q[25] ^ self.q[28] ^ d[4] ^ d[8] ^ d[9] ^ d[12]),
            c[21].eq(self.q[5] ^ self.q[21] ^ self.q[25] ^ self.q[26] ^ self.q[29] ^ d[5] ^ d[9] ^ d[10] ^ d[13]),
            c[22].eq(self.q[6] ^ self.q[16] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ self.q[30] ^ d[0] ^ d[9] ^ d[11] ^ d[12] ^ d[14]),
            c[23].eq(self.q[7] ^ self.q[16] ^ self.q[17] ^ self.q[22] ^ self.q[25] ^ self.q[29] ^ self.q[31] ^ d[0] ^ d[1] ^ d[6] ^ d[9] ^ d[13] ^ d[15]),
            c[24].eq(self.q[8] ^ self.q[17] ^ self.q[18] ^ self.q[23] ^ self.q[26] ^ self.q[30] ^ d[1] ^ d[2] ^ d[7] ^ d[10] ^ d[14]),
            c[25].eq(self.q[9] ^ self.q[18] ^ self.q[19] ^ self.q[24] ^ self.q[27] ^ self.q[31] ^ d[2] ^ d[3] ^ d[8] ^ d[11] ^ d[15]),
            c[26].eq(self.q[10] ^ self.q[16] ^ self.q[19] ^ self.q[20] ^ self.q[22] ^ self.q[26] ^ d[0] ^ d[3] ^ d[4] ^ d[6] ^ d[10]),
            c[27].eq(self.q[11] ^ self.q[17] ^ self.q[20] ^ self.q[21] ^ self.q[23] ^ self.q[27] ^ d[1] ^ d[4] ^ d[5] ^ d[7] ^ d[11]),
            c[28].eq(self.q[12] ^ self.q[18] ^ self.q[21] ^ self.q[22] ^ self.q[24] ^ self.q[28] ^ d[2] ^ d[5] ^ d[6] ^ d[8] ^ d[12]),
            c[29].eq(self.q[13] ^ self.q[19] ^ self.q[22] ^ self.q[23] ^ self.q[25] ^ self.q[29] ^ d[3] ^ d[6] ^ d[7] ^ d[9] ^ d[13]),
            c[30].eq(self.q[14] ^ self.q[20] ^ self.q[23] ^ self.q[24] ^ self.q[26] ^ self.q[30] ^ d[4] ^ d[7] ^ d[8] ^ d[10] ^ d[14]),
            c[31].eq(self.q[15] ^ self.q[21] ^ self.q[24] ^ self.q[25] ^ self.q[27] ^ self.q[31] ^ d[5] ^ d[8] ^ d[9] ^ d[11] ^ d[15]),
        ]


class USB3CRCDPP8(LiteXModule):
    def __init__(self):
        self.di      = Signal(8)
        self.q       = Signal(32)
        self.crc_out = Signal(32)
        self.rst     = Signal()
        self.clk     = Signal()

        d = Signal(8)
        self.comb += d.eq(_rev_wire_from_di(self.di, 8))

        c = Array(Signal() for _ in range(32))
        self.comb += self.crc_out.eq(~_verilog_concat_bits_msb_first([c[i] for i in range(32)]))

        self.comb += [
            c[0].eq(self.q[24] ^ self.q[30] ^ d[0] ^ d[6]),
            c[1].eq(self.q[24] ^ self.q[25] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[1] ^ d[6] ^ d[7]),
            c[2].eq(self.q[24] ^ self.q[25] ^ self.q[26] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7]),
            c[3].eq(self.q[25] ^ self.q[26] ^ self.q[27] ^ self.q[31] ^ d[1] ^ d[2] ^ d[3] ^ d[7]),
            c[4].eq(self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[28] ^ self.q[30] ^ d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[6]),
            c[5].eq(self.q[24] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ self.q[30] ^ self.q[31] ^ d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7]),
            c[6].eq(self.q[25] ^ self.q[26] ^ self.q[28] ^ self.q[29] ^ self.q[30] ^ self.q[31] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7]),
            c[7].eq(self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[29] ^ self.q[31] ^ d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[7]),
            c[8].eq(self.q[0] ^ self.q[24] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ d[0] ^ d[1] ^ d[3] ^ d[4]),
            c[9].eq(self.q[1] ^ self.q[25] ^ self.q[26] ^ self.q[28] ^ self.q[29] ^ d[1] ^ d[2] ^ d[4] ^ d[5]),
            c[10].eq(self.q[2] ^ self.q[24] ^ self.q[26] ^ self.q[27] ^ self.q[29] ^ d[0] ^ d[2] ^ d[3] ^ d[5]),
            c[11].eq(self.q[3] ^ self.q[24] ^ self.q[25] ^ self.q[27] ^ self.q[28] ^ d[0] ^ d[1] ^ d[3] ^ d[4]),
            c[12].eq(self.q[4] ^ self.q[24] ^ self.q[25] ^ self.q[26] ^ self.q[28] ^ self.q[29] ^ self.q[30] ^ d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6]),
            c[13].eq(self.q[5] ^ self.q[25] ^ self.q[26] ^ self.q[27] ^ self.q[29] ^ self.q[30] ^ self.q[31] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7]),
            c[14].eq(self.q[6] ^ self.q[26] ^ self.q[27] ^ self.q[28] ^ self.q[30] ^ self.q[31] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7]),
            c[15].eq(self.q[7] ^ self.q[27] ^ self.q[28] ^ self.q[29] ^ self.q[31] ^ d[3] ^ d[4] ^ d[5] ^ d[7]),
            c[16].eq(self.q[8] ^ self.q[24] ^ self.q[28] ^ self.q[29] ^ d[0] ^ d[4] ^ d[5]),
            c[17].eq(self.q[9] ^ self.q[25] ^ self.q[29] ^ self.q[30] ^ d[1] ^ d[5] ^ d[6]),
            c[18].eq(self.q[10] ^ self.q[26] ^ self.q[30] ^ self.q[31] ^ d[2] ^ d[6] ^ d[7]),
            c[19].eq(self.q[11] ^ self.q[27] ^ self.q[31] ^ d[3] ^ d[7]),
            c[20].eq(self.q[12] ^ self.q[28] ^ d[4]),
            c[21].eq(self.q[13] ^ self.q[29] ^ d[5]),
            c[22].eq(self.q[14] ^ self.q[24] ^ d[0]),
            c[23].eq(self.q[15] ^ self.q[24] ^ self.q[25] ^ self.q[30] ^ d[0] ^ d[1] ^ d[6]),
            c[24].eq(self.q[16] ^ self.q[25] ^ self.q[26] ^ self.q[31] ^ d[1] ^ d[2] ^ d[7]),
            c[25].eq(self.q[17] ^ self.q[26] ^ self.q[27] ^ d[2] ^ d[3]),
            c[26].eq(self.q[18] ^ self.q[24] ^ self.q[27] ^ self.q[28] ^ self.q[30] ^ d[0] ^ d[3] ^ d[4] ^ d[6]),
            c[27].eq(self.q[19] ^ self.q[25] ^ self.q[28] ^ self.q[29] ^ self.q[31] ^ d[1] ^ d[4] ^ d[5] ^ d[7]),
            c[28].eq(self.q[20] ^ self.q[26] ^ self.q[29] ^ self.q[30] ^ d[2] ^ d[5] ^ d[6]),
            c[29].eq(self.q[21] ^ self.q[27] ^ self.q[30] ^ self.q[31] ^ d[3] ^ d[6] ^ d[7]),
            c[30].eq(self.q[22] ^ self.q[28] ^ self.q[31] ^ d[4] ^ d[7]),
            c[31].eq(self.q[23] ^ self.q[29] ^ d[5]),
        ]
