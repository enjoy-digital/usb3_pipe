#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CRCs (USB 3.2 7.2.1/7.2.2): CRC-5 (Link Commands/Link Control Words), CRC-16 (Header
Packets), CRC-32 (Data Packet Payloads).

Bits are transmitted LSB first (bytes in order), the remainder is inverted and bit-reversed. The
parallel equations are generated from the polynom (symbolic serial computation), data bit i is bit
i of the data Signal (byte k on data[8*k:8*(k+1)]).
"""

from migen import *

from litex.gen import *

# CRC Equations ------------------------------------------------------------------------------------

def crc_equations(width, poly, nbits):
    """Parallel CRC equations: returns, for each bit of the next state, a mask over the current state
    bits (bits 0..width-1) and the data bits (bits width..width+nbits-1)."""
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

# CRC-32 -------------------------------------------------------------------------------------------

class CRC32(LiteXModule):
    """CRC-32 of a Data Packet Payload, computed on 32-bit words (1 to 4 valid bytes per word).

    clear: initialize the CRC (before the first word), ce: process data (bytes valid bytes: data[0:8]
    first, 0: no data). value: CRC of the processed data (registered state), next_value: CRC including
    the current data.
    """
    def __init__(self):
        self.clear      = Signal()   # i
        self.ce         = Signal()   # i
        self.data       = Signal(32) # i
        self.bytes      = Signal(3)  # i: Valid bytes (0-4).
        self.value      = Signal(32) # o
        self.next_value = Signal(32) # o

        # # #

        state = Signal(32, reset=0xffffffff)
        new   = Signal(32)
        cases = {n: new.eq(Cat(*crc_next(32, 0x04c11db7, state, self.data[:8*n]))) for n in range(1, 5)}
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
