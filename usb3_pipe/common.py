#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

# Helpers ------------------------------------------------------------------------------------------

def K(x, y):
    """K code generator ex: K(28, 5) is COM Symbol"""
    return (y << 5) | x

def D(x, y):
    """D code generator"""
    return (y << 5) | x

def LinkConfig(reset=0, loopback=0, scrambling=1):
    """Link Configuration of TS1/TS2 Ordered Sets."""
    value  = (      reset   << 0)
    value |= (   loopback   << 2)
    value |= ((not scrambling) << 3)
    return value

# Symbols (6.3.5) ----------------------------------------------------------------------------------

class Symbol:
    """Symbol definition with name, 8-bit value and description"""
    def __init__(self, name, value, description=""):
        self.name        = name
        self.value       = value
        self.description = description

SKP =  Symbol("SKP", K(28, 1), "Skip")
SDP =  Symbol("SDP", K(28, 2), "Start Data Packet")
EDB =  Symbol("EDB", K(28, 3), "End Bad")
SUB =  Symbol("SUB", K(28, 4), "Decode Error Substitution")
COM =  Symbol("COM", K(28, 5), "Comma")
RSD =  Symbol("RSD", K(28, 6), "Reserved")
SHP =  Symbol("SHP", K(27, 7), "Start Header Packet")
END =  Symbol("END", K(29, 7), "End")
SLC =  Symbol("SLC", K(30, 7), "Start Link Command")
EPF =  Symbol("EPF", K(23, 7), "End Packet Framing")

symbols = [SKP, SDP, EDB, SUB, COM, RSD, SHP, END, SLC, EPF]

# Training Sequence Ordered Sets (6.4.1.2) ---------------------------------------------------------

class OrderedSet(list):
    """Ordered Set definition with name, 8-bit values and description"""
    def __init__(self, name, values, description=""):
        self.name        = name
        self.values      = values
        self.description = description
        list.__init__(self, values)

    def to_bytes(self):
        r = bytes()
        for e in self:
            if isinstance(e, Symbol):
                r += bytes([e.value])
            else:
                r += bytes([e])
        return r

TSEQ = OrderedSet("TSEQ",
    [COM,      D(31, 7), D(23, 0), D( 0, 6)] +
    [D(20, 0), D(18, 5), D( 7, 7), D( 2, 0)] +
    [D( 2, 4), D(18, 3), D(14, 3), D( 8, 1)] +
    [D( 6, 5), D(30, 5), D(13, 3), D(31, 5)] +
    [D(10, 2) for i in range(16)])

# TSEQ (32 bytes):
# [0] : 0xc017ffbc
# [1] : 0x02e7b214
# [2] : 0x286e7282
# [3] : 0xbf6dbea6
# [4] : 0x4a4a4a4a
# [5] : 0x4a4a4a4a
# [6] : 0x4a4a4a4a
# [7] : 0x4a4a4a4a

TS1 = OrderedSet("TS1",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(10, 2) for i in range(10)])

# TS1 (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0x4a4a0000
# [2] : 0x4a4a4a4a
# [3] : 0x4a4a4a4a

TS1_INV = OrderedSet("TS1_INV",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(21, 5) for i in range(10)])

# TS1_INV (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0xb5b50000
# [2] : 0xb5b5b5b5
# [3] : 0xb5b5b5b5

TS2 = OrderedSet("TS2",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(5, 2) for i in range(10)])

# TS2 (16 bytes):
# [0] : 0xbcbcbcbc
# [1] : 0x45450000
# [2] : 0x45454545
# [3] : 0x45454545

ordered_sets = [TSEQ, TS1, TS1_INV, TS2]

# Endianness Swap ----------------------------------------------------------------------------------

class EndiannessSwap(LiteXModule):
    """Swap the data bytes/ctrl bits of stream"""
    def __init__(self, sink, source):
        assert len(sink.data) == len(source.data)
        assert len(sink.ctrl) == len(source.ctrl)
        self.comb += sink.connect(source, omit={"data", "ctrl"})
        n = len(sink.ctrl)
        for i in range(n):
            self.comb += source.data[8*i:8*(i+1)].eq(sink.data[8*(n-1-i):8*(n-1-i+1)])
            self.comb += source.ctrl[i].eq(sink.ctrl[n-1-i])
