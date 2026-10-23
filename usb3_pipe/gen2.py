#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 2 (10Gbps, 128b/132b) PHY building blocks (USB 3.2 6.3.2), see doc/gen2-x2-roadmap.md.

- Gen2Scrambler    : Gen 2 LFSR (G(X) = X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1, seed 1DBFBCh) on
                     N bytes per cycle, per byte mode (advance+xor, advance+bypass, hold).
- Gen2BlockGearbox : 132-bit blocks (4-bit Block Header + 128-bit payload) to 32-bit transceiver words.
- Gen2BlockAligner : 32-bit transceiver words to 132-bit blocks: block alignment (Block Header search,
                     bit slips), Block Header single bit error correction.

Bits are serialized LSB first (6.3.2.1): Block Header bits first (header[0] first), then the payload
bytes in order, each byte LSB first. The LFSR structure (Galois, output = LFSR[22]) must be checked
against Figure 6-12 (or a Gen 2 capture) before use.
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

        # # #

        cur   = Signal(23, reset=seed)
        state = cur
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

# Block Gearbox (TX) -------------------------------------------------------------------------------

class Gen2BlockGearbox(LiteXModule):
    """132-bit blocks to 32-bit words (33 words for 8 blocks). The output is continuous (source.valid
    always set once started), the block sink is accepted when there is room."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("header", 4), ("data", 128)])
        self.source = source = stream.Endpoint([("data", 32)])

        # # #

        buf   = Signal(132 + 32)
        level = Signal(max=132 + 32 + 1)
        out   = Signal()
        self.comb += [
            out.eq((level >= 32) & source.ready),
            source.valid.eq(level >= 32),
            source.data.eq(buf[0:32]),
            sink.ready.eq((level - Mux(out, 32, 0)) <= 32),
        ]
        block = Cat(sink.header, sink.data)
        shifted = Signal(132 + 32)
        rlevel  = Signal(max=132 + 32 + 1)
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
                Case(rlevel, {l: buf.eq(shifted | (block << l)) for l in range(33)}),
                level.eq(rlevel + 132),
            ).Else(
                buf.eq(shifted),
                level.eq(rlevel),
            )
        ]

# Block Aligner (RX) -------------------------------------------------------------------------------

class Gen2BlockAligner(LiteXModule):
    """32-bit words to 132-bit blocks with block alignment.

    Unlocked: a bit is slipped after each invalid Block Header. Locked after lock_count consecutive
    valid Block Headers, unlocked after unlock_count consecutive invalid ones. Block Headers with a
    single bit error are corrected (header_error set), double bit errors are reported (header_invalid).
    """
    def __init__(self, lock_count=4, unlock_count=4):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("header", 4), ("data", 128),
            ("header_error", 1), ("header_invalid", 1)])
        self.locked = Signal() # o

        # # #

        buf   = Signal(132 + 64)
        level = Signal(max=132 + 64 + 1)

        # Block Header decoding (single bit error correction: distance 4 between the two headers).
        header  = buf[0:4]
        dist_d  = Signal(3)
        dist_c  = Signal(3)
        self.comb += [
            dist_d.eq(Reduce("ADD", [header[i] ^ ((BLOCK_HEADER_DATA    >> i) & 0x1) for i in range(4)])),
            dist_c.eq(Reduce("ADD", [header[i] ^ ((BLOCK_HEADER_CONTROL >> i) & 0x1) for i in range(4)])),
        ]
        exact   = Signal()
        invalid = Signal()
        self.comb += [
            exact.eq((dist_d == 0) | (dist_c == 0)),
            invalid.eq((dist_d > 1) & (dist_c > 1)),
        ]

        good_count = Signal(max=lock_count + 1)
        bad_count  = Signal(max=unlock_count + 1)
        extract = Signal()
        slip    = Signal()
        self.comb += [
            # Block available: aligned (locked) or exact Block Header while searching.
            extract.eq((level >= 132) & (self.locked | exact)),
            slip.eq((level >= 132) & ~self.locked & ~exact),
            source.valid.eq(extract & self.locked),
            source.header.eq(Mux(dist_d <= 1, BLOCK_HEADER_DATA, BLOCK_HEADER_CONTROL)),
            source.data.eq(buf[4:132]),
            source.header_error.eq(~exact),
            source.header_invalid.eq(invalid),
            sink.ready.eq(level < 132 + 32),
        ]

        # Buffer: words appended at level, blocks/slips removed from the bottom.
        consumed  = Signal(max=133)
        remaining = Signal(132 + 64)
        rlevel    = Signal(max=132 + 64 + 1)
        self.comb += [
            If(extract,
                consumed.eq(132)
            ).Elif(slip,
                consumed.eq(1)
            ),
            remaining.eq(buf >> consumed),
            rlevel.eq(level - consumed),
        ]
        self.sync += [
            If(sink.valid & sink.ready,
                Case(rlevel, {l: buf.eq(remaining | (sink.data << l)) for l in range(132 + 32)}),
                level.eq(rlevel + 32),
            ).Else(
                buf.eq(remaining),
                level.eq(rlevel),
            ),
            If(extract,
                If(~self.locked,
                    good_count.eq(good_count + 1),
                    If(good_count == (lock_count - 1),
                        self.locked.eq(1),
                        good_count.eq(0),
                    )
                ).Else(
                    If(invalid,
                        bad_count.eq(bad_count + 1),
                        If(bad_count == (unlock_count - 1),
                            self.locked.eq(0),
                            bad_count.eq(0),
                        )
                    ).Else(
                        bad_count.eq(0)
                    )
                )
            ),
            If(slip, good_count.eq(0)),
        ]
