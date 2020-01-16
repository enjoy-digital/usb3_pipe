# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from functools import reduce
from operator import xor

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM

# Scrambler Unit (Appendix B) ----------------------------------------------------------------------

@ResetInserter()
@CEInserter()
class ScramblerUnit(Module):
    """Scrambler Unit

    This module generates the scrambled datas for the USB3.0 link (X^16 + X^5 + X^4 + X^3 + 1 polynom).
    """
    def __init__(self, reset=0xffff):
        self.value = Signal(32)

        # # #

        new = Signal(16)
        cur = Signal(16, reset=reset)

        self.comb += [
            new[0].eq(cur[0]  ^ cur[6] ^ cur[8]  ^ cur[10]),
            new[1].eq(cur[1]  ^ cur[7] ^ cur[9]  ^ cur[11]),
            new[2].eq(cur[2]  ^ cur[8] ^ cur[10] ^ cur[12]),
            new[3].eq(cur[3]  ^ cur[6] ^ cur[8]  ^ cur[9]  ^ cur[10] ^ cur[11] ^ cur[13]),
            new[4].eq(cur[4]  ^ cur[6] ^ cur[7]  ^ cur[8]  ^ cur[9]  ^ cur[11] ^ cur[12] ^ cur[14]),
            new[5].eq(cur[5]  ^ cur[6] ^ cur[7]  ^ cur[9]  ^ cur[12] ^ cur[13] ^ cur[15]),
            new[6].eq(cur[0]  ^ cur[6] ^ cur[7]  ^ cur[8]  ^ cur[10] ^ cur[13] ^ cur[14]),
            new[7].eq(cur[1]  ^ cur[7] ^ cur[8]  ^ cur[9]  ^ cur[11] ^ cur[14] ^ cur[15]),
            new[8].eq(cur[0]  ^ cur[2] ^ cur[8]  ^ cur[9]  ^ cur[10] ^ cur[12] ^ cur[15]),
            new[9].eq(cur[1]  ^ cur[3] ^ cur[9]  ^ cur[10] ^ cur[11] ^ cur[13]),
            new[10].eq(cur[0] ^ cur[2] ^ cur[4]  ^ cur[10] ^ cur[11] ^ cur[12] ^ cur[14]),
            new[11].eq(cur[1] ^ cur[3] ^ cur[5]  ^ cur[11] ^ cur[12] ^ cur[13] ^ cur[15]),
            new[12].eq(cur[2] ^ cur[4] ^ cur[6]  ^ cur[12] ^ cur[13] ^ cur[14]),
            new[13].eq(cur[3] ^ cur[5] ^ cur[7]  ^ cur[13] ^ cur[14] ^ cur[15]),
            new[14].eq(cur[4] ^ cur[6] ^ cur[8]  ^ cur[14] ^ cur[15]),
            new[15].eq(cur[5] ^ cur[7] ^ cur[9]  ^ cur[15]),

            self.value[0].eq(cur[15]),
            self.value[1].eq(cur[14]),
            self.value[2].eq(cur[13]),
            self.value[3].eq(cur[12]),
            self.value[4].eq(cur[11]),
            self.value[5].eq(cur[10]),
            self.value[6].eq(cur[9]),
            self.value[7].eq(cur[8]),
            self.value[8].eq(cur[7]),
            self.value[9].eq(cur[6]),
            self.value[10].eq(cur[5]),
            self.value[11].eq(cur[4]  ^ cur[15]),
            self.value[12].eq(cur[3]  ^ cur[14] ^ cur[15]),
            self.value[13].eq(cur[2]  ^ cur[13] ^ cur[14] ^ cur[15]),
            self.value[14].eq(cur[1]  ^ cur[12] ^ cur[13] ^ cur[14]),
            self.value[15].eq(cur[0]  ^ cur[11] ^ cur[12] ^ cur[13]),
            self.value[16].eq(cur[10] ^ cur[11] ^ cur[12] ^ cur[15]),
            self.value[17].eq(cur[9]  ^ cur[10] ^ cur[11] ^ cur[14]),
            self.value[18].eq(cur[8]  ^ cur[9]  ^ cur[10] ^ cur[13]),
            self.value[19].eq(cur[7]  ^ cur[8]  ^ cur[9]  ^ cur[12]),
            self.value[20].eq(cur[6]  ^ cur[7]  ^ cur[8]  ^ cur[11]),
            self.value[21].eq(cur[5]  ^ cur[6]  ^ cur[7]  ^ cur[10]),
            self.value[22].eq(cur[4]  ^ cur[5]  ^ cur[6]  ^ cur[9]  ^ cur[15]),
            self.value[23].eq(cur[3]  ^ cur[4]  ^ cur[5]  ^ cur[8]  ^ cur[14]),
            self.value[24].eq(cur[2]  ^ cur[3]  ^ cur[4]  ^ cur[7]  ^ cur[13] ^ cur[15]),
            self.value[25].eq(cur[1]  ^ cur[2]  ^ cur[3]  ^ cur[6]  ^ cur[12] ^ cur[14]),
            self.value[26].eq(cur[0]  ^ cur[1]  ^ cur[2]  ^ cur[5]  ^ cur[11] ^ cur[13] ^ cur[15]),
            self.value[27].eq(cur[0]  ^ cur[1]  ^ cur[4]  ^ cur[10] ^ cur[12] ^ cur[14]),
            self.value[28].eq(cur[0]  ^ cur[3]  ^ cur[9]  ^ cur[11] ^ cur[13]),
            self.value[29].eq(cur[2]  ^ cur[8]  ^ cur[10] ^ cur[12]),
            self.value[30].eq(cur[1]  ^ cur[7]  ^ cur[9]  ^ cur[11]),
            self.value[31].eq(cur[0]  ^ cur[6]  ^ cur[8]  ^ cur[10]),
        ]
        self.sync += cur.eq(new)

# Scrambler (Appendix B) ---------------------------------------------------------------------------

class Scrambler(Module):
    """Scrambler

    This module scrambles the TX data/ctrl stream. K codes shall not be scrambled.
    """
    def __init__(self, reset=0x7dbd):
        self.enable = Signal(reset=1)
        self.sink   =   sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        self.submodules.unit = unit = ScramblerUnit(reset=reset)
        self.comb += unit.ce.eq(sink.valid & sink.ready)
        self.comb += sink.connect(source)
        for i in range(4):
            self.comb += [
                If(~self.enable | sink.ctrl[i], # K codes shall not be scrambled.
                    source.data[8*i:8*(i+1)].eq(sink.data[8*i:8*(i+1)])
                ).Else(
                    source.data[8*i:8*(i+1)].eq(sink.data[8*i:8*(i+1)] ^ unit.value[8*i:8*(i+1)])
                )
            ]

# Descrambler (Scrambler + Auto-Synchronization) (Appendix B) --------------------------------------

class Descrambler(Module):
    """Descrambler

    This module descrambles the RX data/ctrl stream. K codes shall not be scrambled. The descrambler
    automatically synchronizes itself to the incoming stream and resets the scrambler unit when COM
    characters are seen.
    """
    def __init__(self, reset=0xffff):
        self.enable = Signal(reset=1)
        self.sink   =   sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        scrambler = Scrambler(reset=reset)
        self.submodules += scrambler
        self.comb += scrambler.enable.eq(self.enable)

        # Synchronize on COM
        for i in range(4):
            self.comb += [
                If(sink.valid &
                   sink.ready &
                   (sink.data[8*i:8*(i+1)] == COM.value) &
                   sink.ctrl[i],
                   scrambler.unit.reset.eq(1)
                )
            ]

        # Descramble data
        self.comb += [
            sink.connect(scrambler.sink),
            scrambler.source.connect(source)
        ]
