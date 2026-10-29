#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PIPE (USB 3.2 chapter 6): LFPS and Training Sequences units, scrambling, Logical Idle, Idle
handshake, framing alignment, on top of a USB3 SerDes.
"""

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer, GrayCounter

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import *
from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.training import TSUnit
from usb3_pipe.scrambling import Scrambler, Descrambler, lfsr_after_ts, LFSR_SEED_LANE0
from usb3_pipe.datapath import RXWordAligner, Gen2TXDatapath, Gen2RXDatapath, GEN2_DATA_WIDTH
from usb3_pipe.datapath import GEN2_IDLE_BLOCK
from usb3_pipe.framing  import Gen2LinkTX, Gen2LinkRX, gen2_link_layout

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
    - Conversion to/from a 32-bit/4-bit data/ctrl stream.
    - Clock domain crossing to/from sys_clk (>=125MHz).
    - RX words alignment.
    - TX scrambling/RX descrambling.

    Gen 1x2 lanes (USB3PHY lanes=2): lfsr_seed is the lane scrambler seed (FFFFh Lane 0, 8000h
    Lane 1, 6.13.5) and with x2_lane the descrambled lane stream is output as is on source (no
    framing alignment and no rx_ready gating: the lanes are deskewed on the Training Sequences, the
    framing alignment/gating is done after the lanes destriping). With ssp (Configuration Lane), the
    SuperSpeedPlus Polling LFPS (lfps_tx_ssp/lfps_tx_ssp_lfps, from SSPPolling) has priority on the
    LFPS/electrical idle TX controls.
    """
    def __init__(self, serdes, sys_clk_freq, with_endianness_swap=True, timer_scale=1,
        lfsr_seed=LFSR_SEED_LANE0, x2_lane=False, ssp=False):
        assert sys_clk_freq >= 125e6

        # Endpoints --------------------------------------------------------------------------------

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # LTSSM signals (LTSSM_PIPE_INTERFACE, LTSSM outputs are PIPE inputs) ----------------------
        for name, width, direction in LTSSM_PIPE_INTERFACE:
            setattr(self, name, Signal(width, name=name))

        self.disable_scrambling = Signal() # i: Directed to disable scrambling (7.5.4.9.1).

        if ssp:
            self.lfps_tx_ssp      = Signal() # i: SuperSpeedPlus Polling LFPS transmission.
            self.lfps_tx_ssp_lfps = Signal() # i: SuperSpeedPlus Polling LFPS burst.

        # # #

        # Drive SerDes control ---------------------------------------------------------------------
        self.comb += [
            serdes.rx_align.eq(   self.serdes_rx_align),
            serdes.rx_polarity.eq(self.serdes_rx_polarity),
        ]
        # No SKP scheduled during TSEQ (only after TS1/TS2, packets and Logical Idle) and count of
        # transmitted symbols (Y) restarting on Polling.Active entry (6.4.3.1).
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
        self.lfps = lfps = LFPSUnit(serdes,
            sys_clk_freq = sys_clk_freq,
            timer_scale  = timer_scale,
            with_ssp     = ssp)
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
        if ssp:
            self.comb += [
                lfps.tx_ssp.eq(     self.lfps_tx_ssp),
                lfps.tx_ssp_lfps.eq(self.lfps_tx_ssp_lfps),
            ]

        # TS ---------------------------------------------------------------------------------------
        self.ts = ts = TSUnit(serdes=serdes)
        self.comb += [
            # TS Control.
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

        # Scrambling enable (Gen 1, 7.5.4.10.1, 7.5.10.5.1) ----------------------------------------
        # Scrambling is disabled if directed, or if the Disable Scrambling bit is asserted in the
        # TS2 received in Polling/Recovery.Configuration. The bit is only decoded in Polling.Idle
        # and Recovery.Idle (ignored in Hot Reset, 7.5.12.3.1).
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
        # No COM/SKP in the scrambled stream (Training Sequences not scrambled, SKP Ordered Sets
        # inserted by the SerDes TX datapath).
        scrambler = Scrambler(reset=lfsr_after_ts(lfsr_seed), seed=lfsr_seed, with_skp=False,
            with_com=False)
        scrambler = ResetInserter()(scrambler)
        self.comb += scrambler.reset.eq(~tx_enable)
        self.scrambler = scrambler
        self.comb += scrambler.enable.eq(scrambling)
        tx_idle = Signal()
        self.comb += [
            # Logical Idle (7.2.3): send Idle Symbols (00h, scrambled) when the Link Layer has no
            # data, the transceiver would otherwise re-transmit stale symbols and desync the
            # partner's descrambler.
            If(sink.valid & self.tx_ready,
                sink.connect(scrambler.sink, omit={"ready"}),
                # Link Layer words accepted on the SerDes ready only (timings): the Training
                # Sequences are not transmitted with tx_ready (U0, entered from the Idle Symbol
                # handshake; on an exit, tx_ready is deasserted with the TS1 request: at most one
                # word dropped while leaving U0).
                sink.ready.eq(serdes.sink.ready),
            ).Else(
                tx_idle.eq(1),
                scrambler.sink.valid.eq(1),
                scrambler.sink.data.eq(0x00000000),
                scrambler.sink.ctrl.eq(0b0000),
            ),
            # Once the Training Sequences Ordered Set in progress is transmitted.
            If(tx_enable & ~ts.tx_active,
                scrambler.source.connect(serdes.sink)
            )
        ]

        # Descrambling -----------------------------------------------------------------------------
        # The descrambler processes all received symbols, including the Training Sequences received
        # before U0: its LFSR is initialized on every received COM and advances on every non-SKP
        # symbol (6.3.1.3), so it is synchronized with the partner's scrambler when rx_ready is set.
        # Only the output to the Link Layer is gated by rx_ready.
        # No SKP in the received stream (SKP Ordered Sets removed by the SerDes RX datapath).
        self.descrambler = descrambler = Descrambler(reset=lfsr_seed, seed=lfsr_seed, with_skp=False)
        self.comb += descrambler.enable.eq(scrambling)
        # Registered descrambler input (timings: SerDes RX datapath output to the descrambler).
        self.descrambler_buffer = descrambler_buffer = stream.Buffer([("data", 32), ("ctrl", 4)])
        self.comb += [
            serdes.source.connect(descrambler_buffer.sink),
            descrambler_buffer.source.connect(descrambler.sink),
        ]

        # Framing Alignment ------------------------------------------------------------------------
        # The SerDes word alignment is acquired on COM (TS1/TS2) and no COM is received in U0. The
        # partner's Link Layer framing is aligned on its own symbol count, which includes the SKP
        # Ordered Sets (2 symbols, 6.4.3.1) removed by the SerDes: the framing can then be shifted
        # by 2 symbols in the received words. Re-align the words on the framing Ordered Sets (SHP,
        # SDP, SLC, 7.2.1/7.2.2), after the descrambler so its LFSR still advances on every symbol:
        # the symbols dropped/repeated on a re-alignment are the Logical Idle symbols before the
        # framing. The aligner always processes the received words (one word of latency), only its
        # output to the Link Layer is gated by rx_ready.
        if x2_lane:
            self.comb += descrambler.source.connect(source)
        else:
            # Registered descrambler output and ready (timings).
            self.framing_buffer  = framing_buffer  = stream.Buffer([("data", 32), ("ctrl", 4)],
                pipe_ready=True)
            self.framing_aligner = framing_aligner = RXWordAligner(markers=[SHP, SDP, SLC])
            self.comb += [
                descrambler.source.connect(framing_buffer.sink),
                framing_buffer.source.connect(framing_aligner.sink),
                framing_aligner.source.connect(source, omit={"valid", "ready"}),
                source.valid.eq(framing_aligner.source.valid & self.rx_ready),
                framing_aligner.source.ready.eq(source.ready | ~self.rx_ready),
            ]

        # Idle Handshake ---------------------------------------------------------------------------
        # Idle Symbols sent, registered (timings).
        tx_idle_sent = Signal()
        self.sync += tx_idle_sent.eq(tx_enable & tx_idle & scrambler.sink.valid & scrambler.sink.ready)
        # Received symbols from the registered descrambler output (timings).
        self.idle_handshake_unit = idle_handshake = IdleHandshake()
        rx_words = descrambler.source if x2_lane else framing_buffer.source
        rx_valid = Signal()
        rx_data  = rx_words.data
        rx_ctrl  = rx_words.ctrl
        self.comb += [
            rx_valid.eq(rx_words.valid & rx_words.ready),
            idle_handshake.enable.eq(self.idle_handshake),
            idle_handshake.rx_idle.eq(rx_valid & (rx_data == 0) & (rx_ctrl == 0)),
            idle_handshake.rx_data.eq(rx_valid & ((rx_data != 0) | (rx_ctrl != 0))),
            idle_handshake.tx_idle.eq(tx_idle_sent),
            self.idle_handshake_done.eq(idle_handshake.done),
        ]

# Gen 2 PIPE ---------------------------------------------------------------------------------------

# TSEQ Ordered Sets transmitted in Polling.RxEQ in Gen 2 operation (7.5.4.7.2).
GEN2_TSEQ_COUNT = 524288

# LTSSM <-> PIPE signals handled by the Gen2PIPE (Training Sequences, Idle Symbol handshake): the
# LFPS signals stay on the Gen 1 PIPE (LFPSUnit, electrical idle/patterns at both rates).
GEN2_PIPE_INTERFACE = [
    "ts_tx_enable", "ts_tx_tseq", "ts_tx_ts1", "ts_tx_ts2", "ts_tx_ts2_reset", "ts_tx_ts2_loopback",
    "ts_tx_done", "ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2", "ts_rx_ts2_reset", "ts_rx_ts2_loopback",
    "idle_handshake", "idle_handshake_done", "rx_ready", "tx_ready",
]

class Gen2PIPE(LiteXModule):
    """Gen 2 PIPE (Gen 2x1, 10Gbps): Training Sequences and Idle Symbol handshake for the LTSSM on
    the Gen 2 TX/RX datapaths (128b/132b blocks, raw transceiver words).

    LTSSM side (sys clock domain, GEN2_PIPE_INTERFACE signals with the LTSSM_PIPE_INTERFACE
    semantics):
    - ts_tx_tseq/ts_tx_ts1/ts_tx_ts2 (with ts_tx_enable): Gen 2 TSEQ/TS1/TS2 Ordered Sets with the
      SYNC Ordered Sets (6.4.1.2.1), ts_tx_done pulses every tseq_count TSEQ (Polling.RxEQ,
      7.5.4.7.2) or 16 TS1/TS2 transmitted.
    - ts_rx_ts1/ts_rx_ts2: 8 consecutive and identical TS1/TS2 received (Symbols 14/15 excluded,
      SYNC/SKP Ordered Sets are not interruptions, 7.5.4.8.2), Link Functionality of the last TS2.
      Polarity is detected/corrected by the block aligner (ts_rx_ts1_inv not used).
    - idle_handshake/tx_ready: SDS Ordered Set then Data Blocks (Logical Idle: 5Ah Idle Symbols in
      Gen 2 operation, 7.2.3, 7.5.4.10.1), idle_handshake_done once a Logical Idle Data Block (16
      Idle Symbols) is received and one is transmitted after it (7.5.4.10.2).

    Transceiver side: Gen2TXDatapath (words on tx_data, tx clock domain) and Gen2RXDatapath (words
    from rx_data, rx clock domain), data_width-bit raw words, bit 0 first. The control levels are
    transferred to the tx clock domain on a stable command (no transient Ordered Set selection), the
    pulses/levels back to sys with PulseSynchronizers/MultiRegs.

    Link Layer side (sys clock domain, U0): sink/source 32-bit words with K Symbols (Symbol 0 on
    data[0:8], framing Ordered Sets at word boundaries, as in Gen 1 operation), Gen 2 packet framing
    in the Data Blocks (Gen2TXFramer/Gen2TXPacker, Gen2RXFraming): TX units (Header Packet, Link
    Command, DPH with its DPP) stored and forwarded (tx_buffer_depth entries), RX packets
    buffered (rx_buffer_depth entries, ~1.2GB/s peak on the line vs 500MB/s on the Link Layer
    words at 125MHz). The framing processes 8 bytes per cycle up to 40-bit raw words (a Data Block
    every 3.3 cycles), 16 bytes per cycle on wider words (ex: 80-bit, a Data Block every 1.65
    cycles).

    Not implemented yet: Disable Scrambling, Gen 2 elastic buffer.
    """
    def __init__(self, data_width=GEN2_DATA_WIDTH, tseq_count=GEN2_TSEQ_COUNT, ts_count=16,
        tx_buffer_depth=512, rx_buffer_depth=1024, link_words=1, link_cd=None):
        # link_cd: clock domain of the TX framing and sink (default: sys), ex: the Link Layer clock
        # domain when this module runs at a higher PIPE frequency.
        framing_bytes = 8 if data_width <= 40 else 16
        link_layout = [("data", 32), ("ctrl", 4)] if link_words == 1 else gen2_link_layout()
        for name in GEN2_PIPE_INTERFACE:
            setattr(self, name, Signal(name=name))
        self.tx_data = Signal(data_width) # o: TX words (tx clock domain).
        self.rx_data = Signal(data_width) # i: RX words (rx clock domain).
        self.sink    = stream.Endpoint(link_layout) # Link Layer TX words/words pairs (sys).
        self.source  = stream.Endpoint(link_layout) # Link Layer RX words/words pairs (sys).
        self.source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)]) # Link Layer RX LGOOD_n/LCRD_x (sys).
        self.source_lc_enable = Signal() # LGOOD_n/LCRD_x on source_lc (else in source).
        self.rx_overflow = Signal() # o: RX buffer overflow (sticky, rx clock domain).

        # # #

        # TX (tx clock domain) ---------------------------------------------------------------------
        self.tx = tx = ClockDomainsRenamer("tx")(Gen2TXDatapath(data_width=data_width))
        self.comb += [
            tx.source.ready.eq(1),
            self.tx_data.eq(tx.source.data),
        ]

        # Command: enable, tseq, ts1, ts2, data, ts2 reset/loopback (sys, registered).
        cmd = Signal(7)
        self.sync += cmd.eq(Cat(
            self.ts_tx_enable | self.idle_handshake | self.tx_ready,
            self.ts_tx_enable & self.ts_tx_tseq,
            self.ts_tx_enable & self.ts_tx_ts1,
            self.ts_tx_enable & self.ts_tx_ts2,
            self.idle_handshake | self.tx_ready,
            self.ts_tx_ts2_reset,
            self.ts_tx_ts2_loopback,
        ))
        # Applied in the tx clock domain once stable (2 consecutive identical samples).
        cmd_tx   = Signal(7)
        cmd_tx_d = Signal(7)
        cmd_cur  = Signal(7)
        self.specials += MultiReg(cmd, cmd_tx, "tx")
        self.sync.tx += [
            cmd_tx_d.eq(cmd_tx),
            If(cmd_tx == cmd_tx_d, cmd_cur.eq(cmd_tx)),
        ]
        self.comb += [
            tx.tx.enable.eq(cmd_cur[0]),
            tx.tx.tx_tseq.eq(cmd_cur[1]),
            tx.tx.tx_ts1.eq(cmd_cur[2]),
            tx.tx.tx_ts2.eq(cmd_cur[3]),
            tx.tx.tx_data.eq(cmd_cur[4]),
            # Link Functionality (Table 6-6): Reset (bit 0), Loopback (bit 2).
            tx.tx.link_config.eq(Cat(cmd_cur[5], 0, cmd_cur[6])),
        ]

        # TSEQ/TS1/TS2 bursts (tx_done every tseq_count TSEQ/ts_count TS1/TS2 transmitted, the count
        # restarts when the command changes).
        tx_count = Signal(max=max(tseq_count, ts_count))
        tx_done  = Signal()
        cmd_cur_d = Signal(7)
        self.sync.tx += [
            cmd_cur_d.eq(cmd_cur),
            tx_done.eq(0),
            If(cmd_cur != cmd_cur_d,
                tx_count.eq(0)
            ).Elif(tx.tx.tx_os,
                If(tx_count == Mux(cmd_cur[1], tseq_count - 1, ts_count - 1),
                    tx_count.eq(0),
                    tx_done.eq(1)
                ).Else(
                    tx_count.eq(tx_count + 1)
                )
            )
        ]
        tx_done_ps = PulseSynchronizer("tx", "sys")
        self.submodules += tx_done_ps
        self.comb += [
            tx_done_ps.i.eq(tx_done),
            self.ts_tx_done.eq(tx_done_ps.o),
        ]

        # RX (rx clock domain) ---------------------------------------------------------------------
        self.rx = rx = ClockDomainsRenamer("rx")(Gen2RXDatapath(data_width=data_width))
        self.comb += [
            rx.sink.valid.eq(1),
            rx.sink.data.eq(self.rx_data),
            rx.source.ready.eq(1),
        ]
        rx_ts1_ps = PulseSynchronizer("rx", "sys")
        rx_ts2_ps = PulseSynchronizer("rx", "sys")
        self.submodules += rx_ts1_ps, rx_ts2_ps
        link_config = Signal(8)
        self.specials += MultiReg(rx.link_config, link_config)
        self.comb += [
            rx_ts1_ps.i.eq(rx.rx.rx_ts1),
            rx_ts2_ps.i.eq(rx.rx.rx_ts2),
            self.ts_rx_ts1.eq(rx_ts1_ps.o),
            self.ts_rx_ts2.eq(rx_ts2_ps.o),
            self.ts_rx_ts1_inv.eq(0),
            self.ts_rx_ts2_reset.eq(link_config[0]),
            self.ts_rx_ts2_loopback.eq(link_config[2]),
        ]

        # Link Layer TX: framing (sys) -> units buffer (sys -> tx) -> blocks packer (tx) -----------
        link_tx = Gen2LinkTX(tx_buffer_depth, link_words=link_words, nbytes=framing_bytes)
        tx_ready = self.tx_ready
        if link_cd is not None:
            link_tx  = ClockDomainsRenamer({"sys": link_cd})(link_tx)
            tx_ready = Signal()
            self.specials += MultiReg(self.tx_ready, tx_ready, link_cd)
        self.link_tx = link_tx
        self.comb += [
            self.sink.connect(link_tx.sink, omit={"valid", "ready"}),
            link_tx.sink.valid.eq(self.sink.valid & tx_ready),
            self.sink.ready.eq(link_tx.sink.ready & tx_ready),
            link_tx.source.connect(tx.sink),
            link_tx.flush.eq(~tx_ready),
            link_tx.packer.skp_pending.eq(tx.tx.skp_pending),
        ]
        self.tx_packer = link_tx.packer

        # Link Layer RX: blocks (rx) -> framing (rx) -> words buffer (rx -> sys) --------------------
        self.link_rx = link_rx = Gen2LinkRX(rx_buffer_depth, link_words=link_words,
            nbytes=framing_bytes)
        self.comb += [
            # Data Blocks (after SDS), framing reset out of the Data Stream.
            link_rx.sink.valid.eq(rx.source.valid),
            link_rx.sink.data.eq(rx.source.data),
            link_rx.data_stream.eq(rx.data_stream),
            link_rx.rx_ready.eq(self.rx_ready),
            link_rx.source.connect(self.source),
            link_rx.source_lc.connect(self.source_lc),
            link_rx.lc_enable.eq(self.source_lc_enable),
            self.rx_overflow.eq(link_rx.overflow),
        ]
        self.rx_framing = link_rx.framing

        # Idle Symbol handshake (7.5.4.10.2) -------------------------------------------------------
        # RX: a Logical Idle Data Block (16 Idle Symbols) received while enabled. TX: a Logical Idle
        # Data Block (16 Idle Symbols) transmitted after the first one received.
        enable_rx = Signal()
        enable_tx = Signal()
        rx_idle   = Signal()
        rx_idle_tx = Signal()
        tx_idle    = Signal()
        rx_idle_sys = Signal()
        tx_idle_sys = Signal()
        self.specials += [
            MultiReg(self.idle_handshake, enable_rx, "rx"),
            MultiReg(self.idle_handshake, enable_tx, "tx"),
            MultiReg(rx_idle, rx_idle_tx, "tx"),
            MultiReg(rx_idle, rx_idle_sys),
            MultiReg(tx_idle, tx_idle_sys),
        ]
        self.sync.rx += [
            If(~enable_rx,
                rx_idle.eq(0)
            ).Elif(rx.source.valid & (rx.source.data == GEN2_IDLE_BLOCK),
                rx_idle.eq(1)
            )
        ]
        self.sync.tx += [
            If(~enable_tx,
                tx_idle.eq(0)
            ).Elif(rx_idle_tx & tx.tx.tx_idle,
                tx_idle.eq(1)
            )
        ]
        self.comb += self.idle_handshake_done.eq(self.idle_handshake & rx_idle_sys & tx_idle_sys)

