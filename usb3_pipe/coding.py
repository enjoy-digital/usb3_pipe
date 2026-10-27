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

from functools import reduce
from operator import and_, or_

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
    """Number of bits set (combinatorial).

    The bits are zero-extended to the result width: in Verilog, a sum of 1-bit operands compared to
    a constant (ex: popcount(x) > 1) is evaluated on the operands width (1-bit, sum modulo 2), the
    Migen simulator does not reproduce it."""
    w = bits_for(len(bits))
    return Reduce("ADD", [Cat(bits[i], Constant(0, w - 1)) for i in range(len(bits))])

def gen2_block_layout():
    """TX blocks: Block Header + payload (128-bit, 192-bit for SKP Ordered Sets: skp=1)."""
    return [("header", 4), ("data", 192), ("skp", 1)]

# Gen 2 Block Gearbox (TX) -------------------------------------------------------------------------

class Gen2BlockGearbox(LiteXModule):
    """Blocks to data_width-bit words (transceiver words, bit 0 transmitted first): 132-bit blocks
    (Block Header + 128-bit payload) or 196-bit SKP blocks (skp=1, Block Header + 192-bit payload).
    The output is continuous (source.valid always set once started), the block sink is accepted when
    there is room (one block every 132/data_width cycles on average)."""
    def __init__(self, data_width=32):
        self.sink   = sink   = stream.Endpoint(gen2_block_layout())
        self.source = source = stream.Endpoint([("data", data_width)])

        # # #

        dw    = data_width
        buf   = Signal(196 + dw)
        level = Signal(max=196 + dw + 1)
        out   = Signal()
        self.comb += [
            out.eq((level >= dw) & source.ready),
            source.valid.eq(level >= dw),
            source.data.eq(buf[0:dw]),
            sink.ready.eq((level - Mux(out, dw, 0)) <= dw),
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
        shifted = Signal(196 + dw)
        rlevel  = Signal(max=196 + dw + 1)
        self.comb += [
            If(out,
                shifted.eq(buf[dw:]),
                rlevel.eq(level - dw),
            ).Else(
                shifted.eq(buf),
                rlevel.eq(level),
            )
        ]
        self.sync += [
            If(sink.valid & sink.ready,
                buf.eq(shifted | (Cat(block, Constant(0, dw)) << rlevel)),
                level.eq(rlevel + length),
            ).Else(
                buf.eq(shifted),
                level.eq(rlevel),
            )
        ]

# Gen 2 Block Aligner (RX) -------------------------------------------------------------------------

class Gen2BlockAligner(LiteXModule):
    """data_width-bit words (transceiver words, bit 0 received first) to blocks with block alignment
    (6.4.1.2.4), one word per cycle (sink always ready).

    - Unaligned: the 132-bit SYNC Ordered Set block is searched at the data_width bit offsets of
      each new word. A SYNC received inverted (polarity inversion, 6.4.2.2) toggles the polarity
      (new words and buffered bits inverted, decodes valid one cycle later) and is used for the
      alignment.
    - Aligned: blocks are extracted with the alignment, an SDS Ordered Set moves to Locked. A SYNC
      received with a different alignment realigns (error pulse): a bit slip can keep valid Block
      Headers (ex: TSEQ, Symbol 0 bit 0 = 1 seen as a corrected Control Block Header).
    - Locked: blocks are extracted with the alignment.
    In Aligned/Locked, an undefined Block Header (2 bits or more from 0011b/1100b) moves back to
    Unaligned (error pulse), single bit errors are corrected (header_error). SKP Ordered Sets (control
    block starting with SKP or SKPEND) are consumed by Dwords of 4 SKP Symbols until SKPEND (0 to 36
    SKP Symbols, multiple of 4, 6.4.3.3): they are not output, skp pulses with the received LFSR
    state.

    Implementation (40-bit words at 250MHz for 10Gbps): the last words are kept in a window and
    blocks are read at a bit pointer in a small range (2**k >= data_width + 24 positions: pointer
    moves by the consumed length minus data_width each cycle). The per position decodes (Block
    Header, SKP Dwords/SKPEND, SYNC) are computed on the incoming window and registered with it: the
    pointer loop is a small mux, the length decision and an adder. The SYNC search is done one word
    ahead and its offset registered. Outputs are registered (1 cycle).

    Approximation: a SYNC received with a different alignment while a SKP Ordered Set is consumed
    does not realign (the next one does).

    The source must always be ready (one block every 132 bits).
    """
    def __init__(self, data_width=32):
        self.sink   = sink   = stream.Endpoint([("data", data_width)])
        self.source = source = stream.Endpoint([("header", 4), ("data", 128), ("header_error", 1)])

        self.polarity  = Signal()   # o
        self.aligned   = Signal()   # o
        self.locked    = Signal()   # o
        self.error     = Signal()   # o: Undefined Block Header/SKP Ordered Set (pulse).
        self.skp       = Signal()   # o: SKP Ordered Set received (pulse).
        self.skp_lfsr  = Signal(23) # o: SKP Ordered Set LFSR state (valid with skp).
        self.skp_count = Signal(6)  # o: SKP Ordered Set SKP Symbols (valid with skp).

        # # #

        dw   = data_width
        kr   = log2_int(dw + 24, need_pow2=False)
        r    = 2**kr        # Pointer range (blocks read at ptr < r).
        n    = r + 131 + dw # Window (+ SYNC search one word ahead).
        npos = r + 37       # SKP Dwords/SKPEND decode positions (ptr + 0/4/32/36).

        # Window (oldest bits at the bottom) -------------------------------------------------------
        win      = Signal(n)
        win_next = Signal(n)
        data     = Signal(dw)
        toggle   = Signal() # Polarity toggle (buffered bits inverted on the next cycle).
        toggle_d = Signal()
        self.comb += [
            sink.ready.eq(1),
            data.eq(sink.data ^ Replicate(self.polarity, dw)),
            If(sink.valid,
                win_next.eq(Cat(win[dw:] ^ Replicate(toggle_d, n - dw), data))
            ).Else(
                win_next.eq(win ^ Replicate(toggle_d, n))
            )
        ]
        self.sync += [
            win.eq(win_next),
            toggle_d.eq(toggle),
        ]

        # Per position decodes (registered with the window) ----------------------------------------
        ctl_pos   = Signal(r)    # Control Block Header (single bit error corrected).
        inv_pos   = Signal(r)    # Undefined Block Header.
        skp_pos   = Signal(r)    # Control Block Header + SKP/SKPEND Symbol 0.
        dword_pos = Signal(npos) # SKP Dword (4 SKP Symbols).
        end_pos   = Signal(npos) # SKPEND Symbol.
        for p in range(r):
            header = win_next[p:p+4]
            dist_d = popcount(header ^ BLOCK_HEADER_DATA)
            dist_c = popcount(header ^ BLOCK_HEADER_CONTROL)
            sym0   = win_next[p+4:p+12]
            self.sync += [
                ctl_pos[p].eq(dist_c <= 1),
                inv_pos[p].eq((dist_d > 1) & (dist_c > 1)),
                skp_pos[p].eq((dist_c <= 1) & ((sym0 == GEN2_SKP) | (sym0 == GEN2_SKPEND))),
            ]
        for p in range(npos):
            self.sync += [
                dword_pos[p].eq(win_next[p:p+32] == GEN2_SKP*0x01010101),
                end_pos[p].eq(  win_next[p:p+8]  == GEN2_SKPEND),
            ]

        # SYNC search (registered, 2 stages) -------------------------------------------------------
        # Offsets r to r + dw - 1 of the window (00h/FFh Symbols pairs shared), then the first match
        # offset: used one word later (window shifted), the SYNC is then at r - dw + offset (pointer
        # range).
        sync_pos     = Signal(dw)
        sync_inv_pos = Signal(dw)
        pairs        = {}
        pairs_inv    = {}
        for q in range(r + 4, n - 15):
            pairs[q]     = Signal(name=f"sync_pair{q}")
            pairs_inv[q] = Signal(name=f"sync_pair_inv{q}")
            self.sync += [
                pairs[q].eq(    win_next[q:q+16] == 0xff00),
                pairs_inv[q].eq(win_next[q:q+16] == 0x00ff),
            ]
        for k in range(dw):
            p = r + k
            self.comb += [
                sync_pos[k].eq((win[p:p+4] == BLOCK_HEADER_CONTROL) &
                    reduce(and_, [pairs[p + 4 + 16*i] for i in range(8)])),
                sync_inv_pos[k].eq((win[p:p+4] == (~BLOCK_HEADER_CONTROL & 0xf)) &
                    reduce(and_, [pairs_inv[p + 4 + 16*i] for i in range(8)])),
            ]
        # Offset encoding (OR trees): at most one SYNC match per word (SYNC pattern, 132-bit block).
        sync_offset     = Signal(max=dw)
        sync_inv_offset = Signal(max=dw)
        for j in range(len(sync_offset)):
            ks = [k for k in range(dw) if (k >> j) & 1]
            self.comb += [
                sync_offset[j].eq(    reduce(or_, [sync_pos[k]     for k in ks])),
                sync_inv_offset[j].eq(reduce(or_, [sync_inv_pos[k] for k in ks])),
            ]
        sync_hit     = Signal()
        sync_inv_hit = Signal()
        sync_ptr     = Signal(max=n + 1)
        sync_inv_ptr = Signal(max=n + 1)
        self.sync += [
            sync_hit.eq(    (sync_pos     != 0) & sink.valid & ~toggle & ~toggle_d),
            sync_inv_hit.eq((sync_inv_pos != 0) & sink.valid & ~toggle & ~toggle_d),
            sync_ptr.eq(    r - dw + sync_offset),
            sync_inv_ptr.eq(r - dw + sync_inv_offset),
        ]

        # Pointer / decisions ----------------------------------------------------------------------
        ptr      = Signal(max=n + 1)
        ptr_base = Signal(max=n + 1)
        ptr_next = Signal((bits_for(n + 132) + 1, True))
        avail    = Signal()
        idx      = Signal(kr)
        inv      = Signal(r)
        skp      = Signal(r)
        dwords   = Signal(npos)
        ends     = Signal(npos)
        self.comb += [
            avail.eq(ptr < r),
            idx.eq(ptr[:kr]),
            inv.eq(inv_pos >> idx),
            skp.eq(skp_pos >> idx),
            dwords.eq(dword_pos >> idx),
            ends.eq(end_pos >> idx),
        ]

        consumed  = Signal(8)
        error     = Signal()
        drop      = Signal() # Alignment lost.
        found     = Signal() # SYNC found (Unaligned/realignment), SYNC block output.
        realign   = Signal()
        extract   = Signal()
        skp_done  = Signal()
        skp_end   = Signal(6) # SKPEND position (from ptr).
        skp_count = Signal(6)
        skp_total = Signal(6)
        self.comb += [
            ptr_next.eq(ptr_base + consumed - Mux(sink.valid, dw, 0)),
            drop.eq(error | (self.aligned & (ptr_next < 0))),
            self.error.eq(drop | realign),
        ]

        self.fsm = fsm = FSM(reset_state="UNALIGNED")
        fsm.act("UNALIGNED",
            If(sync_hit,
                found.eq(1),
                ptr_base.eq(sync_ptr),
            ).Elif(sync_inv_hit,
                found.eq(1),
                toggle.eq(1),
                ptr_base.eq(sync_inv_ptr),
                NextValue(self.polarity, ~self.polarity)
            ),
            If(found,
                consumed.eq(132),
                If(toggle,
                    NextState("POLARITY")
                ).Else(
                    NextState("BLOCK")
                )
            )
        )
        fsm.act("POLARITY",
            # Buffered bits inverted, decodes not valid for this cycle.
            self.aligned.eq(1),
            ptr_base.eq(ptr),
            NextState("BLOCK"),
            If(drop, NextState("UNALIGNED"))
        )
        fsm.act("BLOCK",
            self.aligned.eq(1),
            ptr_base.eq(ptr),
            # SYNC at a position that is not the next block start (the block at ptr is a 132-bit
            # block ending after the SYNC start, or the SYNC start is already passed): realignment.
            If(sync_hit & (~avail | ((sync_ptr != ptr) & ~skp[0])),
                found.eq(1),
                realign.eq(1),
                ptr_base.eq(sync_ptr),
                consumed.eq(132),
            ).Elif(avail,
                If(inv[0],
                    error.eq(1)
                ).Elif(skp[0],
                    # SKP Dwords after the Block Header until SKPEND (up to 2 Dwords per cycle).
                    If(ends[4],
                        consumed.eq(36),
                        skp_done.eq(1),
                        skp_end.eq(4),
                        skp_total.eq(0),
                    ).Elif(dwords[4] & ends[36],
                        consumed.eq(68),
                        skp_done.eq(1),
                        skp_end.eq(36),
                        skp_total.eq(4),
                    ).Elif(dwords[4] & dwords[36],
                        consumed.eq(68),
                        NextValue(skp_count, 8),
                        NextState("SKP")
                    ).Else(
                        error.eq(1)
                    )
                ).Else(
                    consumed.eq(132),
                    extract.eq(1),
                )
            ),
            If(drop, NextState("UNALIGNED"))
        )
        fsm.act("SKP",
            self.aligned.eq(1),
            ptr_base.eq(ptr),
            If(avail,
                If(ends[0],
                    consumed.eq(32),
                    skp_done.eq(1),
                    skp_end.eq(0),
                    skp_total.eq(skp_count),
                ).Elif(dwords[0] & ends[32] & (skp_count <= 32),
                    consumed.eq(64),
                    skp_done.eq(1),
                    skp_end.eq(32),
                    skp_total.eq(skp_count + 4),
                ).Elif(dwords[0] & dwords[32] & (skp_count <= 28),
                    consumed.eq(64),
                    NextValue(skp_count, skp_count + 8),
                ).Else(
                    error.eq(1)
                )
            ),
            If(skp_done, NextState("BLOCK")),
            If(drop, NextState("UNALIGNED"))
        )
        self.sync += ptr.eq(ptr_next)

        # Outputs (registered) ---------------------------------------------------------------------
        bits     = Signal(132)
        lfsr     = Signal(24)
        out_bits = Signal(132)
        self.comb += [
            bits.eq(win >> idx),
            Case(skp_end, {e: lfsr.eq(bits[e+8:e+32]) for e in [0, 4, 32, 36]}),
        ]
        self.sync += [
            source.valid.eq((found | extract) & ~drop),
            out_bits.eq(Mux(found, GEN2_SYNC_BITS, bits)),
            self.skp.eq(skp_done & ~drop),
            self.skp_count.eq(skp_total),
            self.skp_lfsr.eq(Cat(lfsr[16:24], lfsr[8:16], lfsr[0:7])),
        ]
        out_dist_d = Signal(3)
        out_dist_c = Signal(3)
        out_ctl    = Signal()
        self.comb += [
            out_dist_d.eq(popcount(out_bits[0:4] ^ BLOCK_HEADER_DATA)),
            out_dist_c.eq(popcount(out_bits[0:4] ^ BLOCK_HEADER_CONTROL)),
            out_ctl.eq(out_dist_c <= 1),
            source.header.eq(Mux(out_ctl, BLOCK_HEADER_CONTROL, BLOCK_HEADER_DATA)),
            source.data.eq(out_bits[4:132]),
            source.header_error.eq((out_dist_d != 0) & (out_dist_c != 0)),
        ]
        self.sync += [
            If(source.valid & out_ctl & (out_bits[4:12] == GEN2_SDS),
                self.locked.eq(1)
            ),
            If(self.error,
                self.locked.eq(0)
            ),
        ]
