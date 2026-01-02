#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import *
from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.training import TSUnit
from usb3_pipe.scrambling import Scrambler, Descrambler

# USB3 PIPE ----------------------------------------------------------------------------------------

@ResetInserter()
class USB3PIPE(LiteXModule):
    """USB3.0 PIPE Core

    Wrap an FPGA transceiver exposing 2 TX and RX data/ctrl streams into a USB3.0 PIPE by adding:
    - LFPS detection/generation.
    - Training Sequence Ordered Sets detection/generation.
    - Clock compensation Ordered Sets removing/insertion.
    - Convertion to/from a 32-bit/4-bit data/ctrl stream.
    - Clock domain crossing to/from sys_clk (>=125MHz).
    - RX words alignment.
    - TX scrambling/RX descrambling.
    """
    def __init__(self, serdes, sys_clk_freq, with_endianness_swap=True):
        assert sys_clk_freq >= 125e6

        # Endpoints --------------------------------------------------------------------------------

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # LTSSM signals ----------------------------------------------------------------------------

        self.rx_ready           = Signal()   # i
        self.tx_ready           = Signal()   # i

        self.serdes_rx_align    = Signal()   # i
        self.serdes_rx_polarity = Signal()   # i

        self.lfps_tx_polling    = Signal()   # i
        self.lfps_tx_idle       = Signal()   # i
        self.lfps_rx_polling    = Signal()   # o
        self.lfps_tx_count      = Signal(16) # o

        self.ts_rx_enable       = Signal()   # i
        self.ts_rx_ts1          = Signal()   # o
        self.ts_rx_ts1_inv      = Signal()   # o
        self.ts_rx_ts2          = Signal()   # o

        self.ts_tx_enable       = Signal()   # i
        self.ts_tx_tseq         = Signal()   # i
        self.ts_tx_ts1          = Signal()   # i
        self.ts_tx_ts2          = Signal()   # i
        self.ts_tx_done         = Signal()   # o

        # # #

        # Drive SerDes control ---------------------------------------------------------------------
        self.comb += [
            serdes.rx_align.eq(   self.serdes_rx_align),
            serdes.rx_polarity.eq(self.serdes_rx_polarity),
        ]

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
        self.lfps = lfps = LFPSUnit(serdes=serdes, sys_clk_freq=sys_clk_freq)
        self.comb += [
            # LFPS Control.
            lfps.tx_polling.eq(self.lfps_tx_polling),
            lfps.tx_idle.eq(self.lfps_tx_idle),

            # LFPS Status.
            self.lfps_rx_polling.eq(lfps.rx_polling),
            self.lfps_tx_count.eq(lfps.tx_count),
        ]

        # TS ---------------------------------------------------------------------------------------
        self.ts = ts = TSUnit(serdes=serdes)
        self.comb += [
            # TS Control.
            ts.rx_enable.eq(self.ts_rx_enable),
            ts.tx_enable.eq(self.ts_tx_enable),
            ts.tx_tseq.eq(  self.ts_tx_tseq),
            ts.tx_ts1.eq(   self.ts_tx_ts1),
            ts.tx_ts2.eq(   self.ts_tx_ts2),

            # TS Status.
            self.ts_rx_ts1.eq(    ts.rx_ts1),
            self.ts_rx_ts1_inv.eq(ts.rx_ts1_inv),
            self.ts_rx_ts2.eq(    ts.rx_ts2),
            self.ts_tx_done.eq(   ts.tx_done),
        ]

        # Scrambling -------------------------------------------------------------------------------
        scrambler = Scrambler()
        scrambler = ResetInserter()(scrambler)
        self.comb += scrambler.reset.eq(~self.tx_ready)
        self.scrambler = scrambler
        self.comb += [
            sink.connect(scrambler.sink),
            If(self.tx_ready,
                scrambler.source.connect(serdes.sink)
            )
        ]

        # Descrambling -----------------------------------------------------------------------------
        self.descrambler = descrambler = Descrambler()
        self.comb += [
            serdes.source.connect(descrambler.sink, keep={"data", "ctrl"}),
            If(self.rx_ready,
                serdes.source.connect(descrambler.sink, omit={"data", "ctrl"})
            ),
            descrambler.source.connect(source),
        ]
