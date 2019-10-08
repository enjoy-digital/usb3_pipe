# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

# Helpers ------------------------------------------------------------------------------------------

def K(x, y):
    return (y << 5) | x

def D(x, y):
    return (y << 5) | x

def LinkConfig(reset=0, loopback=0, scrambling=1):
    value  = (      reset   << 0)
    value |= (   loopback   << 2)
    value |= ((not scrambling) << 3)
    return value

# Symbols ------------------------------------------------------------------------------------------

class Symbol:
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

# Training Sequence Ordered Sets -------------------------------------------------------------------

class OrderedSet(list):
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

TS1 = OrderedSet("TS1",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(10, 2) for i in range(10)])

TS2 = OrderedSet("TS2",
    [COM for i in range(4)] +
    [D( 0, 0), LinkConfig(reset=0, loopback=0, scrambling=1)] +
    [D(5, 2) for i in range(10)])

ordered_sets = [TSEQ, TS1, TS2]
