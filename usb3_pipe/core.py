# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.training import TSUnit
from usb3_pipe.ltssm import LTSSM
from usb3_pipe.scrambling import Scrambler, Descrambler

# USB3 PIPE ----------------------------------------------------------------------------------------

@ResetInserter()
class USB3PIPE(Module):
    def __init__(self, serdes, sys_clk_freq, with_scrambling=True):
        assert sys_clk_freq > 125e6
        self.ready  = Signal() # o

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # LFPS -------------------------------------------------------------------------------------
        lfps = LFPSUnit(sys_clk_freq=sys_clk_freq, serdes=serdes)
        self.submodules.lfps = lfps

        # TS----------------------------------------------------------------------------------------
        ts = TSUnit(serdes=serdes)
        self.submodules.ts = ts

        # LTSSM ------------------------------------------------------------------------------------
        ltssm = LTSSM(serdes=serdes, lfps_unit=lfps, ts_unit=ts, sys_clk_freq=sys_clk_freq)
        self.submodules.ltssm = ltssm
        self.comb += self.ready.eq(ltssm.polling.idle)

        # Scrambling -------------------------------------------------------------------------------
        if with_scrambling:
            scrambler = Scrambler()
            scrambler = CEInserter()(scrambler)
            self.comb += scrambler.ce.eq(self.ready)
            self.submodules.scrambler = scrambler
            self.comb += [
                self.sink.connect(scrambler.sink),
                If(self.ready, scrambler.source.connect(serdes.sink))
            ]

            descrambler = Descrambler()
            descrambler = CEInserter()(descrambler)
            self.comb += descrambler.ce.eq(self.ready)
            self.submodules.descrambler = descrambler
            self.comb += [
                If(self.ready, serdes.source.connect(descrambler.sink)),
                descrambler.source.connect(self.source),
            ]
        else:
            self.comb += If(self.ready, self.sink.connect(serdes.sink))
            self.comb += If(self.ready, serdes.source.connect(self.source))
