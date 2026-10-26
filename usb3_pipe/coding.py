#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 line coding (USB 3.2 6.3): 8b/10b (Gen 1, 6.3.1) Python model/code tables and RX checker,
128b/132b (Gen 2, 6.3.2) blocks gearbox/aligner and DC balance helpers.

Gen 2 bits are serialized LSB first (6.3.2.1, Figure 6-11): Block Header bit 0 first, then the
payload Symbols in order, each Symbol LSB first. The Block Header values (0011b data, 1100b control)
are read with bit 0 as the LSB (data: bit 0 = 1 transmitted first).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.cores import code_8b10b

from usb3_pipe.common import BLOCK_HEADER_DATA, BLOCK_HEADER_CONTROL
from usb3_pipe.common import GEN2_SKP, GEN2_SKPEND, GEN2_SDS, GEN2_SYNC_BITS

# 8b/10b Code Tables (6.3.1, Appendix A) -----------------------------------------------------------

# Valid K-codes (Table 6-2 / Appendix A).
K_CODES = [(28, 0), (28, 1), (28, 2), (28, 3), (28, 4), (28, 5), (28, 6), (28, 7),
           (23, 7), (27, 7), (29, 7), (30, 7)]

def encode_8b10b(d, k, rd):
    """Python model of LiteX's SingleEncoder (code_8b10b): returns the 10-bit code (MSB first,
    abcdei fghj) and the running disparity (0: RD-, 1: RD+) after the symbol."""
    t = code_8b10b
    code5b, code3b = d & 0x1f, d >> 5
    if k and code5b == 28:
        code6b, unb6, flip6 = 0b110000, True, True
    else:
        code6b, unb6, flip6 = t.table_5b6b[code5b], t.table_5b6b_unbalanced[code5b], t.table_5b6b_flip[code5b]
    code4b = t.table_3b4b[code3b]
    unb4   = t.table_3b4b_unbalanced[code3b]
    flip4  = True if k else t.table_3b4b_flip[code3b]
    alt7_rd0 = (code3b == 7) and ((code5b in [17, 18, 20]) or k)
    alt7_rd1 = (code3b == 7) and ((code5b in [11, 13, 14]) or k)
    disp_inter = rd ^ unb6
    out6 = (~code6b & 0x3f) if (not rd and flip6) else code6b
    if not disp_inter and alt7_rd0:
        rd_out, out4 = 1, 0b0111
    elif disp_inter and alt7_rd1:
        rd_out, out4 = 0, 0b1000
    else:
        rd_out = disp_inter ^ unb4
        out4   = (~code4b & 0xf) if (not disp_inter and flip4) else code4b
    return (out6 << 4) | out4, int(rd_out)

def code_8b10b_table(lsb_first=True):
    """1024 entries (indexed by the received 10-bit code): (valid with RD-, valid with RD+, RD after
    the symbol for RD-, RD after the symbol for RD+)."""
    table = [[0, 0, 0, 0] for _ in range(1024)]
    symbols = [(d, 0) for d in range(256)] + [(x | (y << 5), 1) for x, y in K_CODES]
    for d, k in symbols:
        for rd in [0, 1]:
            code, rd_out = encode_8b10b(d, k, rd)
            if lsb_first:
                code = int("{:010b}".format(code)[::-1], 2)
            table[code][rd]     = 1
            table[code][2 + rd] = rd_out
    return table

# RX 8b/10b Checker (6.3.1.2, 6.3.1.4) -------------------------------------------------------------

class RX8b10bChecker(LiteXModule):
    """RX 8b/10b Checker

    Checks the received 10-bit symbols (decoders inputs) against the valid 8b/10b codes, with the
    same latency as the decoders outputs:
    - invalid:         Symbol not matching any valid 8b/10b code (decode error, 6.3.1.4).
    - disparity_error: Valid code received in the wrong running disparity column (6.3.1.2).

    The running disparity is tracked over the symbols and re-initialized from the received code on
    errors (6.3.1.2: initial disparity is the disparity of the first received symbol).
    """
    def __init__(self, decoders, lsb_first=True):
        n = len(decoders)
        self.invalid         = Signal(n) # o
        self.disparity_error = Signal(n) # o

        # # #

        table = code_8b10b_table(lsb_first)
        rom   = {code: e[0] | (e[1] << 1) | (e[2] << 2) | (e[3] << 3) for code, e in enumerate(table)}

        # Code lookup (distributed ROM), registered (as the decoders outputs). A block RAM would
        # absorb the decoders input register as its address register: transceiver RX data to the
        # block RAM address in one RX clock cycle (250MHz), missing timing on large designs.
        lookups = []
        for decoder in decoders:
            lookup_comb = Signal(4, attr={("rom_style", "distributed")})
            lookup      = Signal(4)
            self.comb += Case(decoder.input, {code: lookup_comb.eq(v) for code, v in rom.items()})
            self.sync += lookup.eq(lookup_comb)
            lookups.append(lookup)

        # Errors and running disparity.
        rd = Signal()
        rd_lane = rd
        for i, lookup in enumerate(lookups):
            valid_m, valid_p, rd_m, rd_p = [lookup[j] for j in range(4)]
            rd_next = Signal()
            self.comb += [
                self.invalid[i].eq(~valid_m & ~valid_p),
                self.disparity_error[i].eq(Mux(rd_lane, ~valid_p & valid_m, ~valid_m & valid_p)),
                If(rd_lane,
                    If(valid_p,  rd_next.eq(rd_p)).Elif(valid_m, rd_next.eq(rd_m)).Else(rd_next.eq(rd_lane))
                ).Else(
                    If(valid_m,  rd_next.eq(rd_m)).Elif(valid_p, rd_next.eq(rd_p)).Else(rd_next.eq(rd_lane))
                )
            ]
            rd_lane = rd_next
        self.sync += rd.eq(rd_lane)

# Gen 2 128b/132b helpers (6.3.2, 6.4.1.2.2) -------------------------------------------------------

# DC Balance Symbols (6.4.1.2.2): (Symbol 14, Symbol 15).
GEN2_DC_REDUCE_ONES  = (0x20, 0x08)
GEN2_DC_REDUCE_ZEROS = (0xdf, 0xf7)

def popcount(bits):
    """Number of bits set (combinatorial)."""
    return Reduce("ADD", [bits[i] for i in range(len(bits))])

def gen2_block_layout():
    """TX blocks: Block Header + payload (128-bit, 192-bit for SKP Ordered Sets: skp=1)."""
    return [("header", 4), ("data", 192), ("skp", 1)]

# Gen 2 Block Gearbox (TX) -------------------------------------------------------------------------

class Gen2BlockGearbox(LiteXModule):
    """Blocks to 32-bit words: 132-bit blocks (Block Header + 128-bit payload) or 196-bit SKP blocks
    (skp=1, Block Header + 192-bit payload). The output is continuous (source.valid always set once
    started), the block sink is accepted when there is room."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(gen2_block_layout())
        self.source = source = stream.Endpoint([("data", 32)])

        # # #

        buf   = Signal(196 + 32)
        level = Signal(max=196 + 32 + 1)
        out   = Signal()
        self.comb += [
            out.eq((level >= 32) & source.ready),
            source.valid.eq(level >= 32),
            source.data.eq(buf[0:32]),
            sink.ready.eq((level - Mux(out, 32, 0)) <= 32),
        ]
        block   = Signal(196)
        length  = Signal(8)
        self.comb += [
            If(sink.skp,
                block.eq(Cat(sink.header, sink.data)),
                length.eq(196),
            ).Else(
                block.eq(Cat(sink.header, sink.data[:128])),
                length.eq(132),
            )
        ]
        shifted = Signal(196 + 32)
        rlevel  = Signal(max=196 + 32 + 1)
        self.comb += [
            If(out,
                shifted.eq(buf[32:]),
                rlevel.eq(level - 32),
            ).Else(
                shifted.eq(buf),
                rlevel.eq(level),
            )
        ]
        self.sync += [
            If(sink.valid & sink.ready,
                buf.eq(shifted | (Cat(block, Constant(0, 32)) << rlevel)),
                level.eq(rlevel + length),
            ).Else(
                buf.eq(shifted),
                level.eq(rlevel),
            )
        ]

# Gen 2 Block Aligner (RX) -------------------------------------------------------------------------

class Gen2BlockAligner(LiteXModule):
    """32-bit words to blocks with block alignment (6.4.1.2.4).

    - Unaligned: the 132-bit SYNC Ordered Set block is searched at 32 bit offsets per cycle (the input
      rate). A SYNC received inverted (polarity inversion, 6.4.2.2) toggles the polarity (the buffer is
      flushed and the next SYNC is used for the alignment).
    - Aligned: blocks are extracted with the alignment, an SDS Ordered Set moves to Locked.
    - Locked: blocks are extracted with the alignment.
    In Aligned/Locked, an undefined Block Header (2 bits or more from 0011b/1100b) moves back to
    Unaligned (error pulse), single bit errors are corrected (header_error). SKP Ordered Sets (control
    block starting with SKP or SKPEND) are consumed by groups of 4 Symbols until SKPEND (4 to 36 SKP
    Symbols, multiple of 4, 6.4.3.3): they are not output, skp pulses with the received LFSR state.

    Approximation: SYNC Ordered Sets received with a different alignment in Aligned are not detected
    (the alignment is only acquired in Unaligned).

    The source must always be ready (one block every 132 bits).
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("header", 4), ("data", 128), ("header_error", 1)])

        self.polarity  = Signal()   # o
        self.aligned   = Signal()   # o
        self.locked    = Signal()   # o
        self.error     = Signal()   # o: Undefined Block Header (pulse).
        self.skp       = Signal()   # o: SKP Ordered Set received (pulse).
        self.skp_lfsr  = Signal(23) # o: SKP Ordered Set LFSR state (valid with skp).
        self.skp_count = Signal(6)  # o: SKP Ordered Set SKP Symbols (valid with skp).

        # # #

        n   = 256
        buf   = Signal(n)
        level = Signal(max=n + 1)

        consumed = Signal(max=n + 1)
        flush    = Signal()

        # Block Header decoding (single bit error correction: distance 4 between the two headers).
        header  = buf[0:4]
        dist_d  = Signal(3)
        dist_c  = Signal(3)
        self.comb += [
            dist_d.eq(popcount(header ^ BLOCK_HEADER_DATA)),
            dist_c.eq(popcount(header ^ BLOCK_HEADER_CONTROL)),
        ]
        invalid = Signal()
        control = Signal()
        self.comb += [
            invalid.eq((dist_d > 1) & (dist_c > 1)),
            control.eq(dist_c <= 1),
        ]
        sym0 = buf[4:12]

        # SYNC search (32 offsets).
        match     = Signal(32)
        match_inv = Signal(32)
        for k in range(32):
            self.comb += [
                match[k].eq(buf[k:k+132]     == GEN2_SYNC_BITS),
                match_inv[k].eq(buf[k:k+132] == (~GEN2_SYNC_BITS & (2**132 - 1))),
            ]
        offset = Signal(5)
        for k in reversed(range(32)):
            self.comb += If(match[k], offset.eq(k))

        skp_count = Signal(6)
        self.fsm = fsm = FSM(reset_state="UNALIGNED")
        fsm.act("UNALIGNED",
            If(level >= (132 + 31),
                If(match != 0,
                    consumed.eq(offset),
                    NextState("BLOCK")
                ).Elif(match_inv != 0,
                    flush.eq(1),
                    NextValue(self.polarity, ~self.polarity)
                ).Else(
                    consumed.eq(32)
                )
            )
        )
        fsm.act("BLOCK",
            self.aligned.eq(1),
            If(level >= 132,
                If(invalid,
                    self.error.eq(1),
                    NextValue(self.locked, 0),
                    NextState("UNALIGNED")
                ).Elif(control & ((sym0 == GEN2_SKP) | (sym0 == GEN2_SKPEND)),
                    consumed.eq(4),
                    NextValue(skp_count, 0),
                    NextState("SKP")
                ).Else(
                    consumed.eq(132),
                    source.valid.eq(1),
                    If(control & (sym0 == GEN2_SDS),
                        NextValue(self.locked, 1)
                    )
                )
            )
        )
        fsm.act("SKP",
            self.aligned.eq(1),
            If(level >= 32,
                consumed.eq(32),
                If(buf[0:8] == GEN2_SKPEND,
                    self.skp.eq(1),
                    NextState("BLOCK")
                ).Elif((buf[0:32] == 0xcccccccc) & (skp_count < 36),
                    NextValue(skp_count, skp_count + 4)
                ).Else(
                    self.error.eq(1),
                    NextValue(self.locked, 0),
                    NextState("UNALIGNED")
                )
            )
        )
        self.comb += [
            self.skp_lfsr.eq(Cat(buf[24:32], buf[16:24], buf[8:15])),
            self.skp_count.eq(skp_count),
            source.header.eq(Mux(control, BLOCK_HEADER_CONTROL, BLOCK_HEADER_DATA)),
            source.data.eq(buf[4:132]),
            source.header_error.eq((dist_d != 0) & (dist_c != 0)),
        ]

        # Buffer: words (polarity corrected) appended at level, blocks/offsets removed from the bottom.
        data      = Signal(32)
        remaining = Signal(n)
        rlevel    = Signal(max=n + 1)
        self.comb += [
            data.eq(sink.data ^ Replicate(self.polarity, 32)),
            remaining.eq(buf >> consumed),
            rlevel.eq(level - consumed),
            sink.ready.eq(level <= (n - 32)),
        ]
        self.sync += [
            If(flush,
                buf.eq(0),
                level.eq(0),
            ).Elif(sink.valid & sink.ready,
                buf.eq(remaining | (Cat(data, Constant(0, n - 32)) << rlevel)),
                level.eq(rlevel + 32),
            ).Else(
                buf.eq(remaining),
                level.eq(rlevel),
            )
        ]
