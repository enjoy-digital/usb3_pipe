# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import *
from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.training import TSUnit
from usb3_pipe.ltssm import LTSSM
from usb3_pipe.scrambling import Scrambler, Descrambler

# USB3 PIPE ----------------------------------------------------------------------------------------

@ResetInserter()
class USB3PIPE(Module):
    def __init__(self, serdes, sys_clk_freq, with_scrambling=True, with_endianness_swap=True):
        assert sys_clk_freq >= 125e6
        self.ready  = Signal() # o

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Endianness Swap --------------------------------------------------------------------------
        if with_endianness_swap:
            sink        = stream.Endpoint([("data", 32), ("ctrl", 4)])
            source      = stream.Endpoint([("data", 32), ("ctrl", 4)])
            sink_swap   = EndiannessSwap(self.sink, sink)
            source_swap = EndiannessSwap(source, self.source)
            self.submodules += sink_swap, source_swap
        else:
            sink   = self.sink
            source = self.source

        # LFPS -------------------------------------------------------------------------------------
        lfps = LFPSUnit(serdes=serdes, sys_clk_freq=sys_clk_freq)
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
            scrambler = ResetInserter()(scrambler)
            self.comb += scrambler.reset.eq(~ltssm.polling.tx_ready)
            self.submodules.scrambler = scrambler
            self.comb += [
                sink.connect(scrambler.sink),
                If(ltssm.polling.tx_ready, scrambler.source.connect(serdes.sink))
            ]

            descrambler = Descrambler()
            descrambler = ResetInserter()(descrambler)
            descrambler = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(descrambler)
            self.comb += descrambler.reset.eq(~ltssm.polling.rx_ready)
            self.submodules.descrambler = descrambler
            self.comb += [
                serdes.source.connect(descrambler.sink, keep={"data", "ctrl"}),
                If(ltssm.polling.rx_ready, serdes.source.connect(descrambler.sink, omit={"data", "ctrl"})),
                descrambler.source.connect(source),
            ]
        else:
            self.comb += If(ltssm.polling.tx_ready, sink.connect(serdes.sink))
            self.comb += If(ltssm.polling.rx_ready, serdes.source.connect(source))
