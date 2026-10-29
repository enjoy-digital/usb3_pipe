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
        # block RAM address in one RX clock cycle (250MHz), missing timing on large designs. Logic
        # on Gowin (syn_romstyle): GowinSynthesis merges the block ROMs inferred from the lookups on
        # the same address (decoders/checker) into wrong contents.
        lookups = []
        for decoder in decoders:
            lookup_comb = Signal(4, attr={("rom_style", "distributed")})
            lookup      = Signal(4, attr={("syn_romstyle", "logic")})
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
    The output is continuous (source.valid always set once started, source.ready must stay set:
    transceiver), the block sink is accepted when there is room (one block every 132/data_width
    cycles on average).

    The bits level after the output of the cycle (block placement position, 0 to data_width when a
    block is accepted) and the output are registered: the placement is a shift by a registered
    amount, the level loop an adder with constants and a selection."""
    def __init__(self, data_width=32):
        self.sink   = sink   = stream.Endpoint(gen2_block_layout())
        self.source = source = stream.Endpoint([("data", data_width)])

        # # #

        dw  = data_width
        buf = Signal(196 + dw)
        rl  = Signal(max=196 + dw + 1) # Bits after the output of the cycle (placement position).
        out = Signal()                 # Word output this cycle (bits >= dw).
        self.comb += [
            source.valid.eq(out),
            source.data.eq(buf[0:dw]),
            sink.ready.eq(rl <= dw),
        ]
        block  = Signal(196)
        length = Signal(8)
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
        self.comb += shifted.eq(Mux(out, buf[dw:], buf))
        accept = Signal()
        self.comb += accept.eq(sink.valid & sink.ready)
        self.sync += [
            If(accept,
                buf.eq(shifted | (Cat(block, Constant(0, dw)) << rl[:bits_for(dw)])), # rl <= dw.
                # Bits >= 132 > dw: word output on the next cycle.
                out.eq(1),
                rl.eq(rl + length - dw),
            ).Else(
                buf.eq(shifted),
                out.eq(rl >= dw),
                rl.eq(Mux(rl >= dw, rl - dw, rl)),
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
    state. Up to 2 Dwords per cycle, 4 with words wider than 64 bits (the consumed bits must keep up
    with the received words: else the pointer falls behind in a long SKP Ordered Set).

    Implementation (40-bit words at 250MHz, 80-bit words at 125MHz on slower fabrics): the last
    words are kept in a window and blocks are read at a bit pointer in a small range (2**k >=
    data_width + 24 positions). The per position decodes (Block Header, SKP Dwords/SKPEND) are
    computed on the incoming window and registered with it. The pointer is one-hot (one vector per
    context: Block, SKP Ordered Set continued after 4*m*k SKP Symbols): each position is set from a
    few positions of the previous cycle (pointer bit and decode at that position, per consumed
    length), no pointer mux/adder in the loop, a pointer bit dropped (undefined Block Header, SKP
    Ordered Set error, pointer behind the window) is an alignment loss. The SYNC search is done one
    word ahead (one-hot offset registered), a realignment is applied one cycle later (both
    decisions of that SYNC cancelled, SYNC block output). The read blocks/SKP Ordered Sets are
    extracted from the registered pointer/events (binary pointer, window delayed): outputs
    registered (3 cycles).

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
        m    = 2 if dw <= 64 else 4 # SKP Dwords consumed per cycle (at most).
        npos = r + 32*(m - 1) + 5   # SKP Dwords/SKPEND decode positions (ptr + 0/4 + 32*j).
        K    = 9//m                 # SKP contexts (4*m*k SKP Symbols consumed, up to 36).
        P    = r + 132 - dw         # Pointer positions.

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
                inv_pos[p].eq((dist_d > 1) & (dist_c > 1)),
                skp_pos[p].eq((dist_c <= 1) & ((sym0 == GEN2_SKP) | (sym0 == GEN2_SKPEND))),
            ]
        for p in range(npos):
            self.sync += [
                dword_pos[p].eq(win_next[p:p+32] == GEN2_SKP*0x01010101),
                end_pos[p].eq(  win_next[p:p+8]  == GEN2_SKPEND),
            ]

        # SYNC search (registered, one-hot offset) -------------------------------------------------
        # Offsets r to r + dw - 1 of the window (00h/FFh Symbols pairs shared): used one word later
        # (window shifted), the SYNC is then at r - dw + offset (at most one SYNC match per word: SYNC
        # pattern, 132-bit block).
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
                    Reduce("AND", [pairs[p + 4 + 16*i] for i in range(8)])),
                sync_inv_pos[k].eq((win[p:p+4] == (~BLOCK_HEADER_CONTROL & 0xf)) &
                    Reduce("AND", [pairs_inv[p + 4 + 16*i] for i in range(8)])),
            ]
        sync_hit     = Signal()
        sync_inv_hit = Signal()
        sync_oh      = Signal(dw)
        sync_inv_oh  = Signal(dw)
        sync_ok      = Signal()
        self.comb += sync_ok.eq(sink.valid & ~toggle & ~toggle_d)
        self.sync += [
            sync_hit.eq(    (sync_pos     != 0) & sync_ok),
            sync_inv_hit.eq((sync_inv_pos != 0) & sync_ok),
            sync_oh.eq(    Mux(sync_ok, sync_pos,     0)),
            sync_inv_oh.eq(Mux(sync_ok, sync_inv_pos, 0)),
        ]

        # One-hot pointer --------------------------------------------------------------------------
        # vb: block start (Block context), vs[k]: SKP Ordered Set continued (4*m*(k + 1) SKP Symbols
        # consumed). Positions 0 to P - 1 (P - 1: 132-bit block read at r - 1). Updated with the
        # window (held without a new word).
        vb        = Signal(P)
        vs        = [Signal(P, name=f"skp_ptr{k}") for k in range(K)]
        pol       = Signal() # Polarity toggled: decodes not valid, pointer shifted only.
        realign_d = Signal() # Realignment (SYNC of the previous cycle, Block context).
        load_d    = Signal() # SYNC loaded (Unaligned) on the previous cycle.
        sync_oh_d = Signal(dw)
        any_d     = Signal() # Pointer set on the previous cycle.
        unaligned = Signal()
        load      = Signal()
        step      = Signal() # Pointer moved by the decodes.
        self.comb += [
            unaligned.eq(~any_d & ~load_d),
            load.eq(unaligned & (sync_hit | sync_inv_hit) & sink.valid),
            toggle.eq(load & ~sync_hit),
            step.eq(~pol & ~realign_d),
        ]

        # Transitions: position p (context, decode at p, consumed bits c) -> position p + c - dw.
        next_b = [[] for _ in range(P)]
        next_s = [[[] for _ in range(P)] for _ in range(K)]
        events = {}
        def transition(v, p, conds, c, targets, event=None):
            q = p + c - dw
            if q < 0:
                return # Pointer behind the window: alignment loss.
            assert q < P
            t = Signal()
            self.comb += t.eq(Reduce("AND", [v[p]] + conds))
            targets[q].append(t)
            if event is not None:
                events.setdefault(event, []).append(t)
        def dwords(p, o, j):
            # SKP Dwords at p + o + 32*i (i < j).
            return [dword_pos[p + o + 32*i] for i in range(j)]
        for p in range(P):
            if p < r:
                # Block context: 132-bit block, SKP Ordered Set (SKPEND after j < m Dwords or
                # continued after m Dwords), undefined Block Header: dropped.
                transition(vb, p, [step, ~inv_pos[p], ~skp_pos[p]], 132, next_b, "extract")
                for j in range(m):
                    transition(vb, p, [step, skp_pos[p]] + dwords(p, 4, j) + [end_pos[p + 4 + 32*j]],
                        4 + 32*(j + 1), next_b, ("skp_block", j))
                transition(vb, p, [step, skp_pos[p]] + dwords(p, 4, m), 4 + 32*m, next_s[0])
                # SKP context (up to 36 SKP Symbols): SKPEND after j Dwords or continued.
                for k in range(K):
                    for j in range(min(m, 10 - m*(k + 1))):
                        transition(vs[k], p, [step] + dwords(p, 0, j) + [end_pos[p + 32*j]],
                            32*(j + 1), next_b, ("skp", k, j))
                    if (k + 1) < K:
                        transition(vs[k], p, [step] + dwords(p, 0, m), 32*m, next_s[k + 1])
                # Polarity toggled: shifted only.
                transition(vb, p, [pol], 0, next_b)
                for k in range(K):
                    transition(vs[k], p, [pol], 0, next_s[k])
            else:
                # Block not in the pointer range yet: shifted.
                transition(vb, p, [~realign_d], 0, next_b)
                for k in range(K):
                    transition(vs[k], p, [~realign_d], 0, next_s[k])
        # SYNC loads (Unaligned: SYNC at r - dw + k, realignment: one cycle later), 132-bit block.
        for k in range(dw):
            q = r - dw + k + 132 - dw
            t = Signal()
            self.comb += t.eq(load & Mux(sync_hit, sync_oh[k], sync_inv_oh[k]))
            next_b[q].append(t)
            q = r - dw + k + 132 - 2*dw
            t = Signal()
            self.comb += t.eq(realign_d & sync_oh_d[k])
            next_b[q].append(t)
        or_terms = lambda terms: Reduce("OR", terms) if terms else 0
        self.sync += If(sink.valid,
            vb.eq(Cat(*[or_terms(next_b[q]) for q in range(P)])),
            [vs[k].eq(Cat(*[or_terms(next_s[k][q]) for q in range(P)])) for k in range(K)],
        )

        # Realignment: SYNC in the Block context not at the pointer (pointer not at the SYNC, not in
        # a SKP Ordered Set).
        hit_at_ptr = Signal()
        skp_at_ptr = Signal()
        in_skp     = Signal()
        realign    = Signal()
        self.comb += [
            hit_at_ptr.eq(Reduce("OR", [sync_oh[k] & vb[r - dw + k] for k in range(dw)])),
            skp_at_ptr.eq(Reduce("OR", [vb[p] & skp_pos[p] for p in range(r)])),
            in_skp.eq(Reduce("OR", vs) != 0),
            realign.eq(sync_hit & any_d & ~hit_at_ptr & ~skp_at_ptr & ~in_skp & ~pol & ~realign_d),
        ]
        any_v = Signal()
        self.comb += any_v.eq((vb != 0) | in_skp)
        self.sync += If(sink.valid,
            pol.eq(toggle),
            realign_d.eq(realign),
            load_d.eq(load),
            sync_oh_d.eq(sync_oh),
            any_d.eq(any_v),
        )
        self.sync += If(toggle, self.polarity.eq(~self.polarity))
        self.comb += self.aligned.eq(any_d)

        # Events/pointer (registered) --------------------------------------------------------------
        # Events: OR of their transitions (partial ORs registered by 16 positions).
        def event(name):
            terms = events.get(name, [])
            parts = [Signal(name="ev_part") for _ in range((len(terms) + 15)//16)]
            for i, part in enumerate(parts):
                self.sync += part.eq(Reduce("OR", terms[16*i:16*(i+1)]) & sink.valid)
            return or_terms(parts)
        e_idx   = Signal(kr)
        e_win   = Signal(n)
        e_found = Signal()
        e_drop  = Signal()
        e_drop_d = Signal()
        any_dd  = Signal()
        vany    = [Reduce("OR", [vb[p]] + [vs[k][p] for k in range(K)]) for p in range(r)]
        self.sync += [
            e_idx.eq(Cat(*[Reduce("OR", [vany[p] for p in range(r) if (p >> i) & 1])
                for i in range(kr)])),
            e_win.eq(win),
            e_found.eq((load | realign_d) & sink.valid),
            # Pointer dropped (alignment loss) or realignment (decisions of the previous cycle
            # cancelled).
            any_dd.eq(any_d),
            e_drop.eq(realign),
            e_drop_d.eq(e_drop),
        ]
        e_extract = Signal()
        e_skp     = Signal()
        e_end     = Signal(7)
        e_total   = Signal(6)
        self.comb += e_extract.eq(event("extract"))
        skp_cases = []
        for j in range(m):
            skp_cases.append((event(("skp_block", j)), 4 + 32*j, 4*j))
        for k in range(K):
            for j in range(min(m, 10 - m*(k + 1))):
                skp_cases.append((event(("skp", k, j)), 32*j, 4*m*(k + 1) + 4*j))
        self.comb += [
            e_skp.eq(Reduce("OR", [c for c, _, _ in skp_cases])),
            [If(c, e_end.eq(end), e_total.eq(total)) for c, end, total in skp_cases],
        ]

        # Outputs (registered) ---------------------------------------------------------------------
        # Decisions of a realigned cycle cancelled (e_drop on the next cycle: realign_d).
        bits     = Signal(132)
        o_valid  = Signal()
        o_skp    = Signal()
        o_end    = Signal(7)
        o_total  = Signal(6)
        o_bits   = Signal(132)
        lfsr     = Signal(24)
        out_bits = Signal(132)
        self.comb += bits.eq(e_win >> e_idx)
        self.sync += [
            o_valid.eq((e_found | e_extract) & ~realign_d),
            o_bits.eq(Mux(e_found, GEN2_SYNC_BITS, bits)),
            o_skp.eq(e_skp & ~realign_d),
            o_end.eq(e_end),
            o_total.eq(e_total),
        ]
        self.comb += Case(o_end, {e: lfsr.eq(o_bits[e+8:e+32]) for o in [0, 4]
            for e in range(o, o + 32*m, 32)})
        self.sync += [
            source.valid.eq(o_valid),
            out_bits.eq(o_bits),
            self.skp.eq(o_skp),
            self.skp_count.eq(o_total),
            self.skp_lfsr.eq(Cat(lfsr[16:24], lfsr[8:16], lfsr[0:7])),
            # Alignment loss (pointer dropped, except on a realigned cycle) or realignment.
            self.error.eq((any_dd & ~any_d & ~e_drop_d) | e_drop),
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
