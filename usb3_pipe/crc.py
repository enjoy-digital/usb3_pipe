#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CRCs (USB 3.2 7.2.1/7.2.2): CRC-5 (Link Commands/Link Control Words), CRC-16 (Header
Packets), CRC-32 (Data Packet Payloads). Used by the PHY (CRC-5 of the fast Link Commands) and
the Link Layer (usb3_core.crc re-exports this module).

Bits are transmitted LSB first (bytes in order), the remainder is inverted and bit-reversed. The
parallel equations are generated from the polynom (symbolic serial computation), data bit i is bit
i of the data Signal (byte k on data[8*k:8*(k+1)]).

The pure Python reference models (crc5/crc16/crc32) are used by the host models and the tests.
"""

import zlib

from migen import *

from litex.gen import *

# CRC Equations ------------------------------------------------------------------------------------

def crc_equations(width, poly, nbits):
    """Parallel CRC equations: returns, for each bit of the next state, a mask over the current
    state bits (bits 0..width-1) and the data bits (bits width..width+nbits-1)."""
    state = [1 << i for i in range(width)]
    for i in range(nbits):
        msb   = state[width - 1]
        state = [0] + state[:-1]
        fb    = msb ^ (1 << (width + i))
        for j in range(width):
            if (poly >> j) & 0x1:
                state[j] ^= fb
    return state

def crc_xor(bits, mask):
    terms = [bits[i] for i in range(len(bits)) if (mask >> i) & 0x1]
    return Reduce("XOR", terms) if terms else 0

def crc_next(width, poly, state, data):
    """Next CRC state expressions (list of width bits) from state/data Signals."""
    bits = [state[i] for i in range(width)] + [data[i] for i in range(len(data))]
    return [crc_xor(bits, mask) for mask in crc_equations(width, poly, len(data))]

def crc_value(width, state_bits):
    """CRC value from the state bits: inverted remainder, bit-reversed."""
    return Cat(*[~state_bits[width - 1 - i] for i in range(width)])

# CRC Combinatorial --------------------------------------------------------------------------------

class CRCComb(LiteXModule):
    """Combinatorial CRC of a fixed size data (CRC-5 of Link Commands, CRC-16 of Header Packets)."""
    def __init__(self, width, poly, init, data_width):
        self.data  = Signal(data_width) # i
        self.value = Signal(width)      # o

        # # #

        init_bits = Constant(init, width)
        self.comb += self.value.eq(crc_value(width, crc_next(width, poly, init_bits, self.data)))

class CRC5(CRCComb):
    """CRC-5 (x^5+x^2+1) of the 11-bit Link Command/Link Control Word information."""
    def __init__(self):
        CRCComb.__init__(self, width=5, poly=0x05, init=0x1f, data_width=11)

class CRC16(CRCComb):
    """CRC-16 (x^16+x^12+x^3+x+1) of the 12 bytes of a Header Packet."""
    def __init__(self):
        CRCComb.__init__(self, width=16, poly=0x100b, init=0xffff, data_width=96)

# CRC-16 Words -------------------------------------------------------------------------------------

class CRC16Words(LiteXModule):
    """CRC-16 of a Header Packet computed on its 32-bit words (one word per cycle: smaller and
    faster than the 96-bit combinatorial CRC16).

    clear: initialize the CRC, ce: process data (4 bytes, data[0:8] first). value: CRC of the
    processed words (registered state).
    """
    def __init__(self):
        self.clear = Signal()   # i
        self.ce    = Signal()   # i
        self.data  = Signal(32) # i
        self.value = Signal(16) # o

        # # #

        state = Signal(16, reset=0xffff)
        self.sync += [
            If(self.clear,
                state.eq(0xffff)
            ).Elif(self.ce,
                state.eq(Cat(*crc_next(16, 0x100b, state, self.data)))
            )
        ]
        self.comb += self.value.eq(crc_value(16, state))

# CRC-32 -------------------------------------------------------------------------------------------

class CRC32(LiteXModule):
    """CRC-32 of a Data Packet Payload, computed on 32-bit words (1 to 4 valid bytes per word), on
    64-bit words (data_width=64: 1 to 4 valid bytes on data[0:32], or 8 valid bytes) or on 128-bit
    words (data_width=128: 1 to 16 valid bytes).

    clear: initialize the CRC (before the first word), ce: process data (bytes valid bytes:
    data[0:8] first, 0: no data). value: CRC of the processed data (registered state), next_value:
    CRC including the current data.

    cascade: the 1 to 4 bytes results from 4 chained 1-byte stages (taps) instead of 4 parallel
    equation sets (less logic: ~-27% with data_width=64, deeper: ~+3 LUT levels), the 8 bytes
    equations unchanged.

    data_width=128: the full 4-byte words of the data (4, 8, 12 or 16 bytes) processed with the
    data, the last 1-3 bytes (a word not a multiple of 4 bytes: the last word of a DPP) registered
    and processed in value/next_value (state loop on the full words only, timings): the next data
    is then processed after a clear. next_value32: CRC including 1-4 bytes (bytes) of data[0:32] (a
    last word on 32 bits: shallower than next_value).
    """
    def __init__(self, data_width=32, cascade=False):
        assert data_width in [32, 64, 128]
        self.clear        = Signal()                        # i
        self.ce           = Signal()                        # i
        self.data         = Signal(data_width)              # i
        # Valid bytes (0-4, 8 with data_width=64, 0-16 with data_width=128).
        self.bytes        = Signal(bits_for(data_width//8)) # i
        self.value        = Signal(32)                      # o
        self.next_value   = Signal(32)                      # o
        self.next_value32 = Signal(32)                      # o

        # # #

        poly  = 0x04c11db7
        state = Signal(32, reset=0xffffffff)
        new   = Signal(32)
        new32 = Signal(32)
        self.comb += [
            Case(self.bytes, {**{n: new32.eq(Cat(*crc_next(32, poly, state, self.data[:8*n])))
                for n in [1, 2, 3, 4]}, "default": new32.eq(state)}),
            self.next_value32.eq(crc_value(32, new32)),
        ]
        if data_width == 128:
            self.add_words_tail(state, new, poly)
            return
        sizes = [1, 2, 3, 4] + ([8] if data_width == 64 else [])
        if cascade:
            taps = [state]
            for i in range(4):
                tap = Signal(32)
                self.comb += tap.eq(Cat(*crc_next(32, poly, taps[-1], self.data[8*i:8*(i + 1)])))
                taps.append(tap)
            cases = {n: new.eq(taps[n]) for n in [1, 2, 3, 4]}
            if data_width == 64:
                cases[8] = new.eq(Cat(*crc_next(32, poly, state, self.data[:64])))
        else:
            cases = {n: new.eq(Cat(*crc_next(32, poly, state, self.data[:8*n])))
                for n in sizes}
        cases["default"] = new.eq(state)
        self.comb += Case(self.bytes, cases)
        self.sync += [
            If(self.clear,
                state.eq(0xffffffff)
            ).Elif(self.ce,
                state.eq(new)
            )
        ]
        self.comb += [
            self.value.eq(crc_value(32, state)),
            self.next_value.eq(crc_value(32, new)),
        ]

    def add_words_tail(self, state, new, poly):
        # Full words (bytes[2:5]: 0-4 words) -> state, tail bytes (bytes[0:2]: 0-3) registered.
        words     = self.bytes[2:5]
        tail      = self.bytes[0:2]
        full      = Signal(32)
        tail_data = Signal(24)
        self.comb += Case(words, {
            **{q: [
                full.eq(Cat(*crc_next(32, poly, state, self.data[:32*q])) if q else state),
                tail_data.eq(self.data[32*q:32*q + 24] if q < 4 else 0),
            ] for q in range(5)},
            "default": full.eq(state),
        })
        tail_r      = Signal(2)
        tail_data_r = Signal(24, reset_less=True)
        self.sync += [
            If(self.clear,
                state.eq(0xffffffff),
                tail_r.eq(0),
            ).Elif(self.ce,
                state.eq(full),
                tail_r.eq(tail),
                tail_data_r.eq(tail_data),
            )
        ]
        def with_tail(s, n, d):
            # s with the n (Signal: 0-3) bytes of d.
            r = Signal(32)
            self.comb += Case(n, {b: r.eq(Cat(*crc_next(32, poly, s, d[:8*b])) if b else s)
                for b in range(4)})
            return r
        self.comb += [
            new.eq(with_tail(full, tail, tail_data)),
            self.value.eq(crc_value(32, with_tail(state, tail_r, tail_data_r))),
            self.next_value.eq(crc_value(32, new)),
        ]

# CRC Reference Models -----------------------------------------------------------------------------

def _crc(data_bits, width, poly, init):
    crc = init
    for bit in data_bits:
        msb = (crc >> (width - 1)) & 0x1
        crc = (crc << 1) & ((1 << width) - 1)
        if bit ^ msb:
            crc ^= poly
    crc ^= (1 << width) - 1
    return int("{:0{}b}".format(crc, width)[::-1], 2)

def crc5(value, nbits=11):
    """CRC-5 of Link Commands/Link Control Words (x^5+x^2+1)."""
    return _crc([(value >> i) & 0x1 for i in range(nbits)], 5, 0x05, 0x1f)

def crc16(data):
    """CRC-16 of Header Packets (x^16+x^12+x^3+x+1)."""
    return _crc([(byte >> i) & 0x1 for byte in data for i in range(8)], 16, 0x100b, 0xffff)

def crc32(data):
    """CRC-32 of Data Packet Payloads."""
    return zlib.crc32(bytes(data))
