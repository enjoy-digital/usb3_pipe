#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

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
    """USB3.0 PIPE Core

    Wrap an FPGA transceiver exposing 2 TX and RX data/ctrl streams into a USB3.0 PIPE by adding:
    - LFPS detection/generation.
    - Training Sequence Ordered Sets detection/generation.
    - Clock compensation Ordered Sets removing/insertion.
    - Convertion to/from a 32-bit/4-bit data/ctrl stream.
    - Clock domain crossing to/from sys_clk (>=125MHz).
    - RX words alignment.
    - TX scrambling/RX descrambling.
    - Link Training State Machine.
    """
    def __init__(self, serdes, sys_clk_freq, with_endianness_swap=True):
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
        self.comb += self.ready.eq(ltssm.polling.idle | ltssm.polling.recovery)

        # Scrambling -------------------------------------------------------------------------------
        scrambler = Scrambler()
        scrambler = ResetInserter()(scrambler)
        self.comb += scrambler.reset.eq(~ltssm.polling.tx_ready)
        self.submodules.scrambler = scrambler
        self.comb += [
            sink.connect(scrambler.sink),
            If(ltssm.polling.tx_ready, scrambler.source.connect(serdes.sink))
        ]

        descrambler = Descrambler()
        self.submodules.descrambler = descrambler
        self.comb += [
            serdes.source.connect(descrambler.sink, keep={"data", "ctrl"}),
            If(ltssm.polling.rx_ready, serdes.source.connect(descrambler.sink, omit={"data", "ctrl"})),
            descrambler.source.connect(source),
        ]
