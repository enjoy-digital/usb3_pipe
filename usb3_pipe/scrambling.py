#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Gen 1 scrambling (USB 3.2 6.3.1.3, Appendix B): symbol level LFSR, Scrambler/Descrambler.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM, SKP

# LFSR helpers (Appendix B) ------------------------------------------------------------------------

def lfsr_advance(state):
    """Advance the X^16 + X^5 + X^4 + X^3 + 1 LFSR by one serial shift.

    state is a list of 16 masks: each bit of the LFSR is the XOR of the bits of a reference state
    selected by its mask.
    """
    msb = state[15]
    new = [msb] + state[:15]
    for i in [3, 4, 5]:
        new[i] ^= msb
    return new

def lfsr_xor(signal, mask):
    bits = [signal[i] for i in range(len(signal)) if (mask >> i) & 0x1]
    return Reduce("XOR", bits) if bits else 0

# Scrambler Unit (Appendix B) ----------------------------------------------------------------------

@ResetInserter()
@CEInserter()
class ScramblerUnit(LiteXModule):
    """Scrambler Unit

    This module generates the scrambling values of the 4 symbols of a USB3.0 link word (X^16 + X^5 +
    X^4 + X^3 + 1 polynom). The LFSR advances 8 serial shifts on each symbol except SKP and is
    initialized after each COM, including in the middle of a word (USB 3.2 6.3.1.3).
    """
    def __init__(self, reset=0xffff, seed=0xffff):
        self.skp   = Signal(4)  # i: SKP symbol on lane i.
        self.com   = Signal(4)  # i: COM symbol on lane i.
        self.value = Signal(32) # o: Scrambling value of lane i: value[8*i:8*(i+1)].

        # # #

        cur   = Signal(16, reset=reset)
        state = cur
        for i in range(4):
            # Symbol i value: LFSR output (D15) on each of its 8 serial shifts.
            masks = [1 << n for n in range(16)]
            for n in range(8):
                self.comb += self.value[8*i + n].eq(lfsr_xor(state, masks[15]))
                masks = lfsr_advance(masks)

            # LFSR state after symbol i.
            new = Signal(16)
            self.comb += [
                If(self.com[i],
                    new.eq(seed)
                ).Elif(self.skp[i],
                    new.eq(state)
                ).Else(
                    new.eq(Cat(*[lfsr_xor(state, mask) for mask in masks]))
                )
            ]
            state = new
        self.sync += cur.eq(state)

# Scrambler (Appendix B) ---------------------------------------------------------------------------

class Scrambler(LiteXModule):
    """Scrambler

    This module scrambles the TX data/ctrl stream. K codes shall not be scrambled, the LFSR does not
    advance on SKP and is initialized after COM.
    """
    def __init__(self, reset=0x7dbd):
        self.enable = Signal(reset=1)
        self.sink   =   sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        self.unit = unit = ScramblerUnit(reset=reset)
        self.comb += unit.ce.eq(sink.valid & sink.ready)
        self.comb += sink.connect(source)
        for i in range(4):
            self.comb += [
                unit.skp[i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == SKP.value)),
                unit.com[i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == COM.value)),
                If(~self.enable | sink.ctrl[i], # K codes shall not be scrambled.
                    source.data[8*i:8*(i+1)].eq(sink.data[8*i:8*(i+1)])
                ).Else(
                    source.data[8*i:8*(i+1)].eq(sink.data[8*i:8*(i+1)] ^ unit.value[8*i:8*(i+1)])
                )
            ]

# Descrambler (Scrambler + Auto-Synchronization) (Appendix B) --------------------------------------

class Descrambler(LiteXModule):
    """Descrambler

    This module descrambles the RX data/ctrl stream. K codes shall not be scrambled. The descrambler
    automatically synchronizes itself to the incoming stream: the LFSR is initialized after each
    received COM.
    """
    def __init__(self, reset=0xffff):
        self.enable = Signal(reset=1)
        self.sink   =   sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        self.scrambler = scrambler = Scrambler(reset=reset)
        self.comb += scrambler.enable.eq(self.enable)

        # Descramble data
        self.comb += [
            sink.connect(scrambler.sink),
            scrambler.source.connect(source)
        ]
