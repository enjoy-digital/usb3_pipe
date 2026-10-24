#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PIPE (USB 3.2 chapter 6): LFPS and Training Sequences units, scrambling, Logical Idle, Idle
handshake, framing alignment, on top of a USB3 SerDes.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import *
from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.training import TSUnit
from usb3_pipe.scrambling import Scrambler, Descrambler
from usb3_pipe.serdes import RXWordAligner

# Idle Handshake -----------------------------------------------------------------------------------

class IdleHandshake(LiteXModule):
    """Idle Symbol Handshake (7.5.4.10.2, 7.5.10.4.2, 7.5.12.4.2)

    While enabled, reports `done` when:
    1. Eight consecutive Idle Symbols are received.
    2. Sixteen Idle Symbols are sent after receiving one Idle Symbol.

    Checked on 32-bit words (4 symbols): 2 consecutive Idle words received, 4 Idle words sent
    after the first Idle word received.
    """
    def __init__(self):
        self.enable  = Signal() # i
        self.rx_idle = Signal() # i: Idle word received (descrambled 00h, valid cycle).
        self.rx_data = Signal() # i: Non-Idle word received (valid cycle).
        self.tx_idle = Signal() # i: Idle word sent.
        self.done    = Signal() # o

        # # #

        rx_count = Signal(max=3)
        rx_seen  = Signal()
        tx_count = Signal(max=5)
        self.sync += [
            If(~self.enable,
                rx_count.eq(0),
                rx_seen.eq(0),
                tx_count.eq(0),
            ).Else(
                If(self.rx_idle,
                    rx_seen.eq(1),
                    If(rx_count != 2,
                        rx_count.eq(rx_count + 1)
                    )
                ).Elif(self.rx_data,
                    rx_count.eq(0)
                ),
                If(rx_seen & self.tx_idle & (tx_count != 4),
                    tx_count.eq(tx_count + 1)
                )
            )
        ]
        self.comb += self.done.eq(self.enable & (rx_count == 2) & (tx_count == 4))

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
    def __init__(self, serdes, sys_clk_freq, with_endianness_swap=True, timer_scale=1):
        assert sys_clk_freq >= 125e6

        # Endpoints --------------------------------------------------------------------------------

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # LTSSM signals ----------------------------------------------------------------------------

        self.rx_ready           = Signal()   # i
        self.tx_ready           = Signal()   # i

        self.idle_handshake     = Signal()   # i

        self.disable_scrambling = Signal()   # i: Directed to disable scrambling (7.5.4.9.1).
        self.ts2_decode         = Signal()   # i: Decode the received TS2 Disable Scrambling bit.
        self.idle_done          = Signal()   # o

        self.serdes_rx_align    = Signal()   # i
        self.serdes_rx_polarity = Signal()   # i

        self.lfps_tx_polling    = Signal()   # i
        self.lfps_tx_idle       = Signal()   # i
        self.lfps_rx_polling    = Signal()   # o
        self.lfps_tx_count      = Signal(16) # o

        self.lfps_tx_ping       = Signal()   # i
        self.lfps_tx_exit_u1    = Signal()   # i
        self.lfps_tx_exit_u2    = Signal()   # i
        self.lfps_tx_wakeup_u3  = Signal()   # i
        self.lfps_tx_reset      = Signal()   # i

        self.lfps_rx_ping       = Signal()   # o
        self.lfps_rx_exit_u1    = Signal()   # o
        self.lfps_rx_exit_u2    = Signal()   # o
        self.lfps_rx_wakeup_u3  = Signal()   # o
        self.lfps_rx_reset      = Signal()   # o

        self.ts_rx_enable       = Signal()   # i
        self.ts_rx_ts1          = Signal()   # o
        self.ts_rx_ts1_inv      = Signal()   # o
        self.ts_rx_ts2          = Signal()   # o
        self.ts_rx_ts2_reset    = Signal()   # o
        self.ts_rx_ts2_loopback = Signal()   # o

        self.ts_tx_enable       = Signal()   # i
        self.ts_tx_tseq         = Signal()   # i
        self.ts_tx_ts1          = Signal()   # i
        self.ts_tx_ts2          = Signal()   # i
        self.ts_tx_ts2_reset    = Signal()   # i
        self.ts_tx_ts2_loopback = Signal()   # i
        self.ts_tx_done         = Signal()   # o

        # # #

        # Drive SerDes control ---------------------------------------------------------------------
        self.comb += [
            serdes.rx_align.eq(   self.serdes_rx_align),
            serdes.rx_polarity.eq(self.serdes_rx_polarity),
        ]
        # No SKP scheduled during TSEQ (only after TS1/TS2, packets and Logical Idle) and count of
        # transmitted symbols (Y) restarting on Polling.Active entry (6.4.3.1). SerDes without SKP
        # control: no-op.
        if hasattr(serdes, "tx_skp_enable"):
            self.comb += serdes.tx_skp_enable.eq(~self.ts_tx_tseq)

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
        self.lfps = lfps = LFPSUnit(serdes=serdes, sys_clk_freq=sys_clk_freq, timer_scale=timer_scale)
        self.comb += [
            # LFPS Control.
            lfps.tx_polling.eq(self.lfps_tx_polling),
            lfps.tx_idle.eq(self.lfps_tx_idle),
            lfps.tx_ping.eq(     self.lfps_tx_ping),
            lfps.tx_exit_u1.eq(  self.lfps_tx_exit_u1),
            lfps.tx_exit_u2.eq(  self.lfps_tx_exit_u2),
            lfps.tx_wakeup_u3.eq(self.lfps_tx_wakeup_u3),
            lfps.tx_reset.eq(    self.lfps_tx_reset),

            # LFPS Status.
            self.lfps_rx_polling.eq(lfps.rx_polling),
            self.lfps_tx_count.eq(lfps.tx_count),
            self.lfps_rx_ping.eq(     lfps.rx_ping),
            self.lfps_rx_exit_u1.eq(  lfps.rx_exit_u1),
            self.lfps_rx_exit_u2.eq(  lfps.rx_exit_u2),
            self.lfps_rx_wakeup_u3.eq(lfps.rx_wakeup_u3),
            self.lfps_rx_reset.eq(    lfps.rx_reset),
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
            ts.tx_ts2_reset.eq(   self.ts_tx_ts2_reset),
            ts.tx_ts2_loopback.eq(self.ts_tx_ts2_loopback),
            ts.tx_ts2_scrambling.eq(~self.disable_scrambling),

            # TS Status.
            self.ts_rx_ts1.eq(    ts.rx_ts1),
            self.ts_rx_ts1_inv.eq(ts.rx_ts1_inv),
            self.ts_rx_ts2.eq(    ts.rx_ts2),
            self.ts_rx_ts2_reset.eq(   ts.rx_ts2_reset),
            self.ts_rx_ts2_loopback.eq(ts.rx_ts2_loopback),
            self.ts_tx_done.eq(   ts.tx_done),
        ]

        # Scrambling enable (Gen 1, 7.5.4.10.1, 7.5.10.5.1) -----------------------------------------
        # Scrambling is disabled if directed, or if the Disable Scrambling bit is asserted in the TS2
        # received in Polling/Recovery.Configuration. The bit is only decoded in Polling.Idle and
        # Recovery.Idle (ignored in Hot Reset, 7.5.12.3.1).
        scrambling   = Signal()
        scrambling_r = Signal(reset=1)
        self.comb += If(self.ts2_decode,
            scrambling.eq(ts.rx_ts2_scrambling & ~self.disable_scrambling)
        ).Else(
            scrambling.eq(scrambling_r)
        )
        self.sync += scrambling_r.eq(scrambling)

        # Scrambling -------------------------------------------------------------------------------
        # Scrambled transmission starts with the Idle Symbol handshake (Polling.Idle, Recovery.Idle,
        # Hot Reset.Exit) and continues in U0, the Link Layer data is only accepted with tx_ready.
        tx_enable = Signal()
        self.comb += tx_enable.eq(self.tx_ready | self.idle_handshake)
        scrambler = Scrambler()
        scrambler = ResetInserter()(scrambler)
        self.comb += scrambler.reset.eq(~tx_enable)
        self.scrambler = scrambler
        self.comb += scrambler.enable.eq(scrambling)
        tx_idle = Signal()
        self.comb += [
            # Logical Idle (7.2.3): send Idle Symbols (00h, scrambled) when the Link Layer has no data,
            # the transceiver would otherwise re-transmit stale symbols and desync the partner's
            # descrambler.
            If(sink.valid & self.tx_ready,
                sink.connect(scrambler.sink),
            ).Else(
                tx_idle.eq(1),
                scrambler.sink.valid.eq(1),
                scrambler.sink.data.eq(0x00000000),
                scrambler.sink.ctrl.eq(0b0000),
            ),
            If(tx_enable,
                scrambler.source.connect(serdes.sink)
            )
        ]

        # Descrambling -----------------------------------------------------------------------------
        # The descrambler processes all received symbols, including the Training Sequences received
        # before U0: its LFSR is initialized on every received COM and advances on every non-SKP
        # symbol (6.3.1.3), so it is synchronized with the partner's scrambler when rx_ready is set.
        # Only the output to the Link Layer is gated by rx_ready.
        self.descrambler = descrambler = Descrambler()
        self.comb += descrambler.enable.eq(scrambling)
        self.comb += serdes.source.connect(descrambler.sink)

        # Framing Alignment ------------------------------------------------------------------------
        # The SerDes word alignment is acquired on COM (TS1/TS2) and no COM is received in U0. The
        # partner's Link Layer framing is aligned on its own symbol count, which includes the
        # SKP Ordered Sets (2 symbols, 6.4.3.1) removed by the SerDes: the framing can then be shifted
        # by 2 symbols in the received words. Re-align the words on the framing Ordered Sets (SHP, SDP,
        # SLC, 7.2.1/7.2.2), after the descrambler so its LFSR still advances on every symbol: the
        # symbols dropped/repeated on a re-alignment are the Logical Idle symbols before the framing.
        # The aligner always processes the received words (one word of latency), only its output to
        # the Link Layer is gated by rx_ready.
        self.framing_aligner = framing_aligner = RXWordAligner(markers=[SHP, SDP, SLC])
        self.comb += [
            descrambler.source.connect(framing_aligner.sink),
            framing_aligner.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(framing_aligner.source.valid & self.rx_ready),
            framing_aligner.source.ready.eq(source.ready | ~self.rx_ready),
        ]

        # Idle Handshake ---------------------------------------------------------------------------
        self.idle_handshake_unit = idle_handshake = IdleHandshake()
        rx_valid = Signal()
        self.comb += [
            rx_valid.eq(descrambler.source.valid & descrambler.source.ready),
            idle_handshake.enable.eq(self.idle_handshake),
            idle_handshake.rx_idle.eq(rx_valid & (descrambler.source.data == 0) & (descrambler.source.ctrl == 0)),
            idle_handshake.rx_data.eq(rx_valid & ((descrambler.source.data != 0) | (descrambler.source.ctrl != 0))),
            idle_handshake.tx_idle.eq(tx_enable & tx_idle & scrambler.sink.ready),
            self.idle_done.eq(idle_handshake.done),
        ]
