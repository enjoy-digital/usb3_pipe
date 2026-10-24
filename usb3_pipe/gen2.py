#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 2 (10Gbps, 128b/132b) PHY building blocks (USB 3.2 6.3.2/6.4), see doc/gen2-x2-roadmap.md.

- Gen2Scrambler    : Gen 2 LFSR (G(X) = X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1, seed 1DBFBCh) on
                     N bytes per cycle, per byte mode (advance+xor, advance+bypass, hold).
- Gen2TX           : Block transmitter: TSEQ/TS1/TS2 Ordered Sets (with DC balance Symbols 14/15),
                     SYNC (scrambler re-initialization, block alignment), SDS, Gen 2 SKP Ordered Sets
                     (Y/40 scheduling, LFSR state) and Data Blocks (Logical Idle when no data), with
                     the 6.3.2.3 scrambling rules.
- Gen2BlockGearbox : 132-bit blocks (196-bit SKP blocks) to 32-bit transceiver words.
- Gen2BlockAligner : 32-bit transceiver words to blocks: SYNC Ordered Set based block alignment
                     (Unaligned/Aligned/Locked phases, 6.4.1.2.4), polarity inversion detection on SYNC
                     (6.4.2.2), Block Header single bit error correction (6.3.2.4), variable length SKP
                     Ordered Sets (4-36 SKP Symbols, 6.4.3.3).
- Gen2RX           : Block receiver: descrambling (6.3.2.3 rules, re-initialization after SYNC), TSEQ/TS1/
                     TS2 detection, SDS detection and Data Blocks output.

Bits are serialized LSB first (6.3.2.1, Figure 6-11): Block Header bit 0 first, then the payload
Symbols in order, each Symbol LSB first. The Block Header values (0011b data, 1100b control) are read
with bit 0 as the LSB (data: bit 0 = 1 transmitted first). The LFSR is the Galois structure of
Figure 6-12: D0 <- D22, D2/D5/D8/D16/D21 <- D(n-1) ^ D22, Data Out = Data In ^ D22.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

# Constants ----------------------------------------------------------------------------------------

GEN2_LFSR_SEED   = 0x1dbfbc
GEN2_LFSR_SEED_1 = 0x0607bb   # Lane 1 (Gen 2x2, 6.13.5).
GEN2_LFSR_TAPS   = (1 << 21) | (1 << 16) | (1 << 8) | (1 << 5) | (1 << 2) | (1 << 0)

BLOCK_HEADER_DATA    = 0b0011
BLOCK_HEADER_CONTROL = 0b1100

# Gen 2 Symbols (Table 6-2) and Ordered Sets identifiers (Tables 6-7 to 6-11, 6-13).
GEN2_SKP    = 0xcc
GEN2_SKPEND = 0x33
GEN2_SDS    = 0xe1
GEN2_TS1    = 0x1e
GEN2_TS2    = 0x2d
GEN2_TSEQ   = 0x87

# DC Balance Symbols (6.4.1.2.2): (Symbol 14, Symbol 15).
GEN2_DC_REDUCE_ONES  = (0x20, 0x08)
GEN2_DC_REDUCE_ZEROS = (0xdf, 0xf7)

GEN2_SKP_TX_SYMBOLS = 20 # SKP Symbols in a transmitted SKP Ordered Set (24 Symbols, 6.4.3.3).
GEN2_SKP_INTERVAL   = 40 # Average interval between SKP Ordered Sets (blocks, 6.4.3.3).

# Ordered Sets payloads (16 Symbols, Symbol 0 first) -----------------------------------------------

def gen2_ts_payload(identifier, link_config=0):
    """TS1/TS2 (Tables 6-7/6-8) or TSEQ (Table 6-9) payload, Symbols 14/15 = identifier."""
    payload = [identifier]*16
    payload[4] = 0x00
    payload[5] = link_config if identifier in [GEN2_TS1, GEN2_TS2] else 0x00
    return payload

def gen2_sync_payload():
    """SYNC Ordered Set (Table 6-10)."""
    return [0x00, 0xff]*8

def gen2_sds_payload():
    """SDS Ordered Set (Table 6-11)."""
    return [GEN2_SDS]*4 + [0x55]*12

def gen2_skp_lfsr_symbols(lfsr):
    """SKP Ordered Set LFSR Symbols (Table 6-13): Bit[7] = ~LFSR[22], Bit[6:0] = LFSR[22:16],
    LFSR[15:8], LFSR[7:0]."""
    return [(((~lfsr >> 22) & 0x1) << 7) | ((lfsr >> 16) & 0x7f), (lfsr >> 8) & 0xff, lfsr & 0xff]

def gen2_skp_payload(lfsr, n_skp=GEN2_SKP_TX_SYMBOLS):
    """SKP Ordered Set (Table 6-13): n_skp SKP (multiple of 4), SKPEND, LFSR state."""
    assert n_skp % 4 == 0
    return [GEN2_SKP]*n_skp + [GEN2_SKPEND] + gen2_skp_lfsr_symbols(lfsr)

def symbols_to_int(symbols):
    return sum(s << (8*i) for i, s in enumerate(symbols))

GEN2_SYNC_BITS = BLOCK_HEADER_CONTROL | (symbols_to_int(gen2_sync_payload()) << 4) # 132-bit.

# Scrambler ----------------------------------------------------------------------------------------

def gen2_lfsr_advance(state):
    """Advance the Gen 2 LFSR by one bit (state: list of 23 masks over a reference state)."""
    msb = state[22]
    new = [0] + state[:22]
    for i in range(23):
        if (GEN2_LFSR_TAPS >> i) & 0x1:
            new[i] ^= msb
    return new

def _xor(signal, mask):
    bits = [signal[i] for i in range(len(signal)) if (mask >> i) & 0x1]
    return Reduce("XOR", bits) if bits else 0

class Gen2Scrambler(LiteXModule):
    """Gen 2 scrambler on nbytes bytes per cycle (byte 0 first).

    Per byte: advance[i] (the LFSR advances 8 bits) and scramble[i] (the byte is XORed with the LFSR
    output, only meaningful when advancing). init re-initializes the LFSR to the seed (SYNC Ordered Set).
    """
    def __init__(self, nbytes=4, seed=GEN2_LFSR_SEED):
        self.ce       = Signal()          # i
        self.init     = Signal()          # i
        self.advance  = Signal(nbytes)    # i
        self.scramble = Signal(nbytes)    # i
        self.i        = Signal(8*nbytes)  # i
        self.o        = Signal(8*nbytes)  # o
        self.state    = Signal(23)        # o: Current LFSR state (SKP Ordered Set LFSR Symbols).

        # # #

        cur   = Signal(23, reset=seed)
        state = cur
        self.comb += self.state.eq(cur)
        for b in range(nbytes):
            masks = [1 << n for n in range(23)]
            key   = Signal(8)
            for n in range(8):
                self.comb += key[n].eq(_xor(state, masks[22]))
                masks = gen2_lfsr_advance(masks)
            self.comb += If(self.advance[b] & self.scramble[b],
                self.o[8*b:8*(b+1)].eq(self.i[8*b:8*(b+1)] ^ key)
            ).Else(
                self.o[8*b:8*(b+1)].eq(self.i[8*b:8*(b+1)])
            )
            new = Signal(23)
            self.comb += If(self.advance[b],
                new.eq(Cat(*[_xor(state, mask) for mask in masks]))
            ).Else(
                new.eq(state)
            )
            state = new
        self.sync += [
            If(self.init,
                cur.eq(seed)
            ).Elif(self.ce,
                cur.eq(state)
            )
        ]

# Helpers ------------------------------------------------------------------------------------------

def _popcount(bits):
    return Reduce("ADD", [bits[i] for i in range(len(bits))])

def _symbols(symbols):
    return Cat(*[Constant(s, 8) for s in symbols])

def gen2_block_layout():
    """TX blocks: Block Header + payload (128-bit, 192-bit for SKP Ordered Sets: skp=1)."""
    return [("header", 4), ("data", 192), ("skp", 1)]

# Gen 2 TX -----------------------------------------------------------------------------------------

class Gen2TX(LiteXModule):
    """Gen 2 block transmitter (one block per cycle on source).

    Controls (levels, one at a time): tx_tseq/tx_ts1/tx_ts2 (Ordered Sets, a SYNC Ordered Set first
    and then every sync_interval_ts TS1/TS2 or sync_interval_tseq TSEQ, 6.4.1.2.1) or tx_data (SDS
    Ordered Set then Data Blocks from sink, Logical Idle Data Blocks when sink has no data).

    SKP Ordered Sets (6.4.3.3): Y counts the non SKP blocks (reset while sending TSEQ: Y restarts on
    Polling.Active entry), Z = Y/skp_interval accumulated (2 max) and inserted after SYNC, TS1, TS2,
    SDS, Logical Idle and the last Data Block of a packet (sink.last), never inside a packet. SYNC has
    priority over SKP.
    """
    def __init__(self, seed=GEN2_LFSR_SEED, skp_interval=GEN2_SKP_INTERVAL, sync_interval_ts=32,
        sync_interval_tseq=16384):
        self.enable      = Signal()  # i
        self.tx_tseq     = Signal()  # i
        self.tx_ts1      = Signal()  # i
        self.tx_ts2      = Signal()  # i
        self.tx_data     = Signal()  # i
        self.link_config = Signal(8) # i: TS1/TS2 Link Functionality (Table 6-6).
        self.tx_os       = Signal()  # o: TSEQ/TS1/TS2 Ordered Set sent (pulse).

        self.sink   = sink   = stream.Endpoint([("data", 128)])
        self.source = source = stream.Endpoint(gen2_block_layout())

        # # #

        self.scrambler = scrambler = Gen2Scrambler(nbytes=16, seed=seed)

        # Block selection --------------------------------------------------------------------------
        tx_os        = Signal()
        sync_count   = Signal(max=max(sync_interval_ts, sync_interval_tseq))
        sync_period  = Signal.like(sync_count)
        y            = Signal(max=skp_interval)
        skp_pending  = Signal(2)
        skp_allowed  = Signal()
        sds_sent     = Signal()
        self.comb += [
            tx_os.eq(self.tx_tseq | self.tx_ts1 | self.tx_ts2),
            sync_period.eq(Mux(self.tx_tseq, sync_interval_tseq - 1, sync_interval_ts - 1)),
        ]

        sel_sync = Signal()
        sel_skp  = Signal()
        sel_sds  = Signal()
        sel_data = Signal()
        sel_ts   = Signal()
        self.comb += [
            If(self.enable,
                If(tx_os & (sync_count == 0),
                    sel_sync.eq(1)
                ).Elif((skp_pending != 0) & skp_allowed,
                    sel_skp.eq(1)
                ).Elif(self.tx_data,
                    If(~sds_sent,
                        sel_sds.eq(1)
                    ).Else(
                        sel_data.eq(1)
                    )
                ).Elif(tx_os,
                    sel_ts.eq(1)
                )
            )
        ]

        # Payload / scrambling modes (6.3.2.3) -----------------------------------------------------
        ts_identifier = Signal(8)
        self.comb += [
            If(self.tx_tseq,
                ts_identifier.eq(GEN2_TSEQ)
            ).Elif(self.tx_ts1,
                ts_identifier.eq(GEN2_TS1)
            ).Else(
                ts_identifier.eq(GEN2_TS2)
            )
        ]
        ts_payload = Signal(128)
        self.comb += ts_payload.eq(Cat(
            Replicate(ts_identifier, 4),
            Constant(0, 8),
            Mux(self.tx_tseq, 0, self.link_config),
            Replicate(ts_identifier, 10),
        ))
        idle = Signal()
        self.comb += idle.eq(sel_data & ~sink.valid)
        self.comb += [
            source.valid.eq(sel_sync | sel_skp | sel_sds | sel_data | sel_ts),
            scrambler.ce.eq(source.valid & source.ready),
            If(sel_sync,
                # All Symbols bypass, LFSR re-initialized after the SYNC Ordered Set.
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(_symbols(gen2_sync_payload())),
                scrambler.init.eq(source.ready),
            ).Elif(sel_skp,
                # Bypass, no advance.
                source.header.eq(BLOCK_HEADER_CONTROL),
                source.skp.eq(1),
            ).Elif(sel_sds,
                # Bypass, advance.
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(_symbols(gen2_sds_payload())),
                scrambler.advance.eq(0xffff),
            ).Elif(sel_data,
                # Scrambled, advance (Logical Idle: 00h Symbols).
                source.header.eq(BLOCK_HEADER_DATA),
                scrambler.i.eq(Mux(idle, 0, sink.data)),
                scrambler.advance.eq(0xffff),
                scrambler.scramble.eq(0xffff),
                sink.ready.eq(source.ready),
            ).Elif(sel_ts,
                # Symbol 0 bypass/advance, Symbols 1-15 scrambled (14/15 DC Balance override below).
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(ts_payload),
                scrambler.advance.eq(0xffff),
                scrambler.scramble.eq(0xfffe),
            )
        ]

        # DC Balance (6.4.1.2.2) -------------------------------------------------------------------
        # Running DC Balance (1s - 0s) of the transmitted TSEQ/TS1/TS2 bits (Block Header included),
        # saturated at +-511, reset at the start of the Data Blocks transmission. Evaluated at the end
        # of Symbol 11: > 31: DC Balance Symbols 14 and 15, > 15: Symbol 15 only (unscrambled, the LFSR
        # still advances).
        self.dc_balance = dc_balance = Signal((11, True))
        ones_11   = Signal((9, True))
        ones_15   = Signal((9, True))
        balance   = Signal((12, True))
        magnitude = Signal(11)
        ts_out    = Signal(128)
        sym14     = Signal(8)
        sym15     = Signal(8)
        self.comb += [
            ones_11.eq(_popcount(Cat(source.header, scrambler.o[0:96]))),
            balance.eq(dc_balance + 2*ones_11 - 100),
            magnitude.eq(Mux(balance < 0, -balance, balance)),
            sym14.eq(scrambler.o[112:120]),
            sym15.eq(scrambler.o[120:128]),
            If(magnitude > 31,
                sym14.eq(Mux(balance > 0, GEN2_DC_REDUCE_ONES[0], GEN2_DC_REDUCE_ZEROS[0])),
                sym15.eq(Mux(balance > 0, GEN2_DC_REDUCE_ONES[1], GEN2_DC_REDUCE_ZEROS[1])),
            ).Elif(magnitude > 15,
                sym15.eq(Mux(balance > 0, GEN2_DC_REDUCE_ONES[1], GEN2_DC_REDUCE_ZEROS[1])),
            ),
            ts_out.eq(Cat(scrambler.o[0:112], sym14, sym15)),
            ones_15.eq(_popcount(Cat(source.header, ts_out))),
        ]
        new_balance = Signal((12, True))
        self.comb += new_balance.eq(dc_balance + 2*ones_15 - 132)

        # Output -----------------------------------------------------------------------------------
        lfsr = scrambler.state
        self.comb += [
            If(sel_skp,
                source.data.eq(Cat(
                    _symbols([GEN2_SKP]*GEN2_SKP_TX_SYMBOLS + [GEN2_SKPEND]),
                    lfsr[16:23], ~lfsr[22], lfsr[8:16], lfsr[0:8]))
            ).Elif(sel_ts,
                source.data.eq(ts_out)
            ).Else(
                source.data.eq(scrambler.o)
            ),
            self.tx_os.eq(sel_ts & source.ready),
        ]

        # State ------------------------------------------------------------------------------------
        self.sync += [
            If(~self.enable,
                sync_count.eq(0),
                y.eq(0),
                skp_pending.eq(0),
                skp_allowed.eq(0),
                sds_sent.eq(0),
                dc_balance.eq(0),
            ).Elif(source.valid & source.ready,
                # SYNC scheduling (TS Ordered Sets count).
                If(sel_ts,
                    If(sync_count == sync_period,
                        sync_count.eq(0)
                    ).Else(
                        sync_count.eq(sync_count + 1)
                    )
                ).Elif(sel_sync,
                    sync_count.eq(1)
                ),
                # SKP scheduling.
                If(~sel_skp,
                    If(self.tx_tseq & ~self.tx_data,
                        y.eq(0)
                    ).Elif(y == (skp_interval - 1),
                        y.eq(0),
                        If(skp_pending != 2,
                            skp_pending.eq(skp_pending + 1)
                        )
                    ).Else(
                        y.eq(y + 1)
                    ),
                    skp_allowed.eq(sel_sync | sel_sds | (sel_ts & ~self.tx_tseq) | idle |
                        (sel_data & sink.last)),
                ).Else(
                    skp_pending.eq(skp_pending - 1)
                ),
                # Data Stream.
                If(sel_sds,
                    sds_sent.eq(1)
                ).Elif(sel_sync | sel_ts,
                    sds_sent.eq(0)
                ),
                # DC Balance.
                If(sel_ts,
                    If(new_balance > 511,
                        dc_balance.eq(511)
                    ).Elif(new_balance < -511,
                        dc_balance.eq(-511)
                    ).Else(
                        dc_balance.eq(new_balance)
                    )
                ).Elif(sel_sds | sel_data,
                    dc_balance.eq(0)
                )
            ),
            If(~self.tx_data & ~tx_os, sds_sent.eq(0)),
        ]

# Block Gearbox (TX) -------------------------------------------------------------------------------

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

# Block Aligner (RX) -------------------------------------------------------------------------------

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
            dist_d.eq(_popcount(header ^ BLOCK_HEADER_DATA)),
            dist_c.eq(_popcount(header ^ BLOCK_HEADER_CONTROL)),
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

# Gen 2 RX -----------------------------------------------------------------------------------------

class Gen2RX(LiteXModule):
    """Gen 2 block receiver (from Gen2BlockAligner).

    Descrambling (6.3.2.3): Data Blocks advance/descramble, control blocks advance the LFSR (Symbol 0
    bypass), SYNC Ordered Sets re-initialize it (SKP Ordered Sets are removed by the aligner and do not
    advance it). TSEQ/TS1/TS2 detection on Symbols 0-13 (Symbols 14/15 not checked, 6.4.1.2.2, Link
    Functionality excluded): rx_tseq/rx_ts1/rx_ts2 pulse every n_ts consecutive Ordered Sets (SKP and
    SYNC Ordered Sets are not interruptions). The Data Blocks following an SDS are output on source.
    """
    def __init__(self, seed=GEN2_LFSR_SEED, n_ts=8):
        self.sink   = sink   = stream.Endpoint([("header", 4), ("data", 128), ("header_error", 1)])
        self.source = source = stream.Endpoint([("data", 128)])

        self.rx_tseq     = Signal()  # o (pulse)
        self.rx_ts1      = Signal()  # o (pulse)
        self.rx_ts2      = Signal()  # o (pulse)
        self.rx_sync     = Signal()  # o (pulse)
        self.rx_sds      = Signal()  # o (pulse)
        self.link_config = Signal(8) # o: Link Functionality of the last TS1/TS2.
        self.data_stream = Signal()  # o: Receiving the Data Stream (after an SDS).
        self.lfsr        = Signal(23) # o: Descrambler LFSR state.

        # # #

        self.descrambler = descrambler = Gen2Scrambler(nbytes=16, seed=seed)
        self.comb += self.lfsr.eq(descrambler.state)

        control = Signal()
        sym0    = Signal(8)
        is_sync = Signal()
        is_sds  = Signal()
        self.comb += [
            sink.ready.eq(1),
            control.eq(sink.header == BLOCK_HEADER_CONTROL),
            sym0.eq(sink.data[0:8]),
            is_sync.eq(control & (sink.data == symbols_to_int(gen2_sync_payload()))),
            is_sds.eq( control & (sym0 == GEN2_SDS)),
            descrambler.ce.eq(sink.valid),
            descrambler.i.eq(sink.data),
            If(control,
                If(is_sync,
                    descrambler.init.eq(sink.valid)
                ).Elif(is_sds,
                    descrambler.advance.eq(0xffff)
                ).Else(
                    descrambler.advance.eq(0xffff),
                    descrambler.scramble.eq(0xfffe)
                )
            ).Else(
                descrambler.advance.eq(0xffff),
                descrambler.scramble.eq(0xffff)
            ),
            source.valid.eq(sink.valid & ~control & self.data_stream),
            source.data.eq(descrambler.o),
            self.rx_sync.eq(sink.valid & is_sync),
            self.rx_sds.eq(sink.valid & is_sds),
        ]

        # Training Sequences detection.
        o = descrambler.o
        def ts_match(identifier, with_link_config):
            sym = [o[8*i:8*(i+1)] for i in range(14)]
            checks = [sym[i] == identifier for i in list(range(4)) + list(range(6, 14))]
            checks.append(sym[4] == 0)
            if not with_link_config:
                checks.append(sym[5] == 0)
            return reduce(and_, checks)
        matches = {
            "tseq" : ts_match(GEN2_TSEQ, False),
            "ts1"  : ts_match(GEN2_TS1,  True),
            "ts2"  : ts_match(GEN2_TS2,  True),
        }
        for name, m in matches.items():
            matched = Signal(name=f"{name}_matched")
            count   = Signal(max=n_ts, name=f"{name}_count")
            pulse   = getattr(self, f"rx_{name}")
            self.comb += matched.eq(sink.valid & control & ~is_sync & ~is_sds & m)
            self.sync += [
                pulse.eq(0),
                If(matched,
                    If(count == (n_ts - 1),
                        count.eq(0),
                        pulse.eq(1)
                    ).Else(
                        count.eq(count + 1)
                    )
                ).Elif(sink.valid & ~is_sync,
                    count.eq(0)
                )
            ]
            if name != "tseq":
                self.sync += If(matched, self.link_config.eq(o[40:48]))
        self.sync += [
            If(sink.valid & control,
                If(is_sds,
                    self.data_stream.eq(1)
                ).Else(
                    self.data_stream.eq(0)
                )
            )
        ]
