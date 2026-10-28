#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY: USB3PIPE(s) + USB3LTSSM with the Link interface used by a USB3 Link Layer
(doc/usb3-pipe.md), on one lane (Gen 1x1) or two lanes (Gen 1x2, USB 3.2 6.13, see
doc/gen2-x2-roadmap.md).
"""

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import COM, LTSSM_PIPE_INTERFACE, EndiannessSwap
from usb3_pipe.core       import USB3PIPE, Gen2PIPE, GEN2_PIPE_INTERFACE
from usb3_pipe.ltssm      import USB3LTSSM, SSPPolling
from usb3_pipe.scrambling import LFSR_SEED_LANE0, LFSR_SEED_LANE1
from usb3_pipe.lanes      import LaneStriper, LaneDestriper, LaneDeskew, link_layout

# LTSSM <-> PIPE -----------------------------------------------------------------------------------

def connect_ltssm_pipe(ltssm, pipe, with_rx_ts_status=True):
    """LTSSM <-> PIPE wiring (LTSSM_PIPE_INTERFACE).

    with_rx_ts_status: TS1/TS2 received and Idle Symbol handshake status from this PIPE (else
    combined over the lanes by the caller).
    """
    rx_ts_status = ["ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2", "idle_handshake_done"]
    statements   = []
    for name, width, direction in LTSSM_PIPE_INTERFACE:
        if (name in rx_ts_status) and (not with_rx_ts_status):
            continue
        statements.append(connect_ltssm_pipe_signal(ltssm, pipe, name, direction))
    return statements

def connect_ltssm_pipe_signal(ltssm, pipe, name, direction):
    """LTSSM <-> PIPE wiring of a LTSSM_PIPE_INTERFACE signal."""
    if direction == "o":
        return getattr(pipe, name).eq(getattr(ltssm, name))
    else:
        return getattr(ltssm, name).eq(getattr(pipe, name))

# Lanes Event --------------------------------------------------------------------------------------

class LanesEvent(LiteXModule):
    """Lanes events combination (Gen 1x2, 6.13.6): a (pulse) event of each lane is held for window
    cycles, the output is set while both are held (the lanes being skewed by a few words)."""
    def __init__(self, a, b, window=64):
        self.o = Signal() # o

        # # #

        held = []
        for e in [a, b]:
            count = Signal(max=window + 1)
            self.sync += If(e, count.eq(window)).Elif(count != 0, count.eq(count - 1))
            held.append(e | (count != 0))
        self.comb += self.o.eq(held[0] & held[1])

# USB3 PHY -----------------------------------------------------------------------------------------

# LTSSM status outputs exposed by USB3PHY.
USB3PHY_STATUS = [
    "u0", "recovery", "rx_ready", "tx_ready", "u1", "u2", "u3", "compliance", "loopback",
    "hot_reset", "ess_inactive", "polling_idle",
]

# LTSSM Link-Layer control interface (PM handshake, directed requests): (name, direction).
USB3PHY_LINK_CTRL = [
    ("lgo_u1",            "i"), ("lgo_u2",         "i"), ("lgo_u3",        "i"),
    ("lau",               "i"), ("lxu",            "i"), ("lpma",          "i"),
    ("req_u1",            "i"), ("req_u2",         "i"), ("req_u3",        "i"),
    ("tx_lgo_u1",         "o"), ("tx_lgo_u2",      "o"), ("tx_lgo_u3",     "o"),
    ("tx_lau",            "o"), ("tx_lxu",         "o"), ("tx_lpma",       "o"),
    ("link_activity",     "i"),
    ("directed_recovery", "i"), ("hot_reset_req",  "i"), ("warm_reset_req", "i"),
    ("exit_u1_req",       "i"), ("exit_u2_req",    "i"), ("wakeup_u3_req",  "i"),
]

@ResetInserter(["sys"]) # sys only (the Gen2PIPE tx/rx clock domains: synchronized reset, see do_finalize).
class USB3PHY(LiteXModule):
    """USB3 PHY: USB3PIPE(s) + USB3LTSSM.

    Combines the PIPE (LFPS/TS/scrambling/datapath) with the LTSSM to provide a link-trained
    data/ctrl (K-char) symbol stream in the sys clock domain, ready for a USB3 Link Layer. The
    SerDes is not a submodule and must be added by the caller.

    - lanes=1 (Gen 1x1): serdes is a SerDes, 32-bit data/4-bit ctrl link words.
    - lanes=2 (Gen 1x2, 6.13): serdes is a list of 2 SerDes (Lane 0: Configuration Lane), 64-bit
      data/8-bit ctrl link words (Symbol 0 on data[0:8], no endianness swap). x2 is the negotiated
      Gen 1x2 operation (PHY Capability LBPM, x2_capable: advertised capability).
    - gen2_capable: Gen 2 (10Gbps) capability advertised in SuperSpeedPlus Polling (SCD1/SCD2, PHY
      Capability LBPM, also used with lanes=1), gen2 is the negotiated Gen 2 operation. With a
      gen2_capable SerDes (raw words, 10Gbps rate), the training continues at 10Gbps on the
      Gen2PIPE (add_gen2: Gen 2 TSEQ/TS1/TS2, SDS/Logical Idle, gen2_active), else at Gen 1 speed.
      The SuperSpeedPlus Link Layer datapath is not integrated yet (doc/gen2-x2-roadmap.md).

    Link interface (what a USB3 Link Layer needs from the PHY, the Link Layer does not have to know
    the PHY internals):
    - `sink`/`source`: data/ctrl symbol streams (Logical Idle sent when `sink` has no data).
    - Status (see USB3PHY_STATUS): mirrored from the LTSSM.
    - Link-Layer control (see USB3PHY_LINK_CTRL): PM handshake (LGO_Ux/LAU/LXU/LPMA decoded or to
      send by the Link Layer), directed Recovery/Hot Reset/Warm Reset and Ux exit requests. The
      signals are the LTSSM ones (same Signal objects); undriven inputs keep their reset values.
    """
    def __init__(self, serdes, sys_clk_freq, role="device", with_endianness_swap=True,
        timer_scale=1, lanes=1, x2_capable=True, gen2_capable=False, deskew_depth=8, link_words=1):
        assert role in ["device", "host"]
        assert lanes in [1, 2]
        serdes = [serdes] if lanes == 1 else serdes
        assert len(serdes) == lanes
        assert (lanes == 1) or (not with_endianness_swap)
        assert link_words in [1, 2]
        assert (lanes == 1) or (link_words == 1)
        self.link_words = link_words # Link Layer words (1: 32-bit) or words pairs (2: 64-bit).

        # PIPE(s) / LTSSM --------------------------------------------------------------------------
        if lanes == 1:
            ssp = gen2_capable # SuperSpeedPlus Polling (Gen 2x1 capability).
            self.pipe  = pipe  = USB3PIPE(serdes[0],
                sys_clk_freq         = sys_clk_freq,
                with_endianness_swap = with_endianness_swap,
                timer_scale          = timer_scale,
                ssp                  = ssp)
            self.pipes = [pipe]
            self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale,
                ssp=ssp)
            self.comb += ltssm.ds_port.eq(role == "host")
            if ssp:
                self.add_ssp_polling(pipe, serdes[0], sys_clk_freq, role, timer_scale,
                    x2_capable=False, gen2_capable=True)
            if ssp and getattr(serdes[0], "gen2_capable", False):
                self.add_gen2(pipe, serdes[0], with_endianness_swap)
            else:
                self.comb += connect_ltssm_pipe(ltssm, pipe)
        else:
            self.add_lanes(serdes, sys_clk_freq, role, timer_scale, x2_capable, gen2_capable)
            ltssm = self.ltssm

        # SerDes RX Re-initialization --------------------------------------------------------------
        # The transceiver RX initialization (PMA/CDR/offset calibration) does not always end in a
        # good state: on hardware (Artix7 GTP, xHCI host with SSC) ~1/12 RX resets ended with a
        # receiver unable to train or with a high error rate, recovered by another RX reset. Re-run
        # it on each training failure (and Warm Reset), when the LTSSM exits to Rx.Detect, or when
        # requested (serdes_rx_reset, ex: debug).
        self.serdes_rx_reset = Signal() # i
        for s in serdes:
            self.comb += s.rx_reset.eq(ltssm.exit_to_rx_detect | self.serdes_rx_reset)

        # Endpoints --------------------------------------------------------------------------------
        if lanes == 1:
            if hasattr(self, "gen2_pipe"):
                self.add_gen2_datapath(pipe)
            elif link_words == 2:
                self.sink, self.source = self.add_pairs_adapters(pipe)
            else:
                self.sink   = pipe.sink
                self.source = pipe.source
        else:
            self.add_lanes_datapath(deskew_depth)

        # Status -----------------------------------------------------------------------------------
        self.ready = Signal() # SerDes ready (PLL locked, transceiver reset done).
        self.comb += self.ready.eq(Reduce("AND", [s.ready for s in serdes]))
        for name in USB3PHY_STATUS:
            setattr(self, name, Signal(name=name))
            self.comb += getattr(self, name).eq(getattr(ltssm, name))

        # Link-Layer Control -----------------------------------------------------------------------
        for name, direction in USB3PHY_LINK_CTRL:
            setattr(self, name, getattr(ltssm, name))
        self.u2_timeout = ltssm.u2_timeout # i: U2 Inactivity Timeout (LMP value, 0: disabled).

        # LTSSM State ------------------------------------------------------------------------------
        ltssm.finalize() # Elaborate the FSM to expose its state/encoding.
        self.ltssm_states = ltssm.fsm.encoding # {state name: value}.
        self.ltssm_state  = Signal(len(ltssm.fsm.state))
        self.comb += self.ltssm_state.eq(ltssm.fsm.state)

    def add_lanes(self, serdes, sys_clk_freq, role, timer_scale, x2_capable, gen2_capable=False):
        """Gen 1x2 (6.13): one USB3PIPE per lane under a single LTSSM, Lane 0 is the Configuration
        Lane.

        - SuperSpeedPlus Polling (SCD1/SCD2, LBPM PHY Capability/Ready, SSPPolling) on Lane 0 before
          Polling.RxEQ (7.5.4.3-7.5.4.6), LFPS on Lane 0 only (6.13.12), Lane 1 in electrical idle.
        - Ordered Sets (TSEQ/TS1/TS2/SKP) transmitted simultaneously on both lanes (6.13.6), TS1/TS2
          requirements satisfied on both lanes (rx_ts1/rx_ts2 reported when seen on both lanes).
        - Per lane scramblers (Lane 0 FFFFh, Lane 1 8000h, 6.13.5), per lane polarity inversion
          (6.13.7: the LTSSM handles Lane 0, Lane 1 is inverted on TS1 received inverted), per lane
          SKP removal.

        The two lanes TX paths must run in lockstep (same clock, same SKP schedule: SKP Ordered Sets
        inserted simultaneously on both lanes, 6.4.3.2).

        Not implemented: dynamic fallback to Gen 1x1 on Lane 0 when Gen 1x1 is negotiated (x2
        de-asserted), SuperSpeedPlus Link Layer (Type 1/Type 2 traffic, 4-bit HSEQ...), framing
        alignment of the 64-bit link words.
        """
        # PIPEs / LTSSM ----------------------------------------------------------------------------
        self.pipes = pipes = []
        for lane, seed in enumerate([LFSR_SEED_LANE0, LFSR_SEED_LANE1]):
            pipe = USB3PIPE(serdes[lane],
                sys_clk_freq         = sys_clk_freq,
                with_endianness_swap = False,
                timer_scale          = timer_scale,
                lfsr_seed            = seed,
                x2_lane              = True,
                ssp                  = (lane == 0))
            self.add_module(name=f"pipe{lane}", module=pipe)
            pipes.append(pipe)
        self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale, ssp=True)
        self.comb += ltssm.ds_port.eq(role == "host")

        # Lane 0 (Configuration Lane): LTSSM <-> PIPE, TS/Idle status combined with Lane 1 below.
        self.comb += connect_ltssm_pipe(ltssm, pipes[0], with_rx_ts_status=False)

        # Lane 1: same TX/RX controls, no LFPS (6.13.12): electrical idle while Lane 0 sends LFPS.
        for name in ["lfps_tx_idle", "ts_tx_enable", "ts_tx_tseq", "ts_tx_ts1", "ts_tx_ts2",
            "ts_tx_ts2_reset", "ts_tx_ts2_loopback", "ts2_decode", "idle_handshake",
            "serdes_rx_align", "rx_ready", "tx_ready"]:
            self.comb += getattr(pipes[1], name).eq(getattr(ltssm, name))

        # Lane 1 polarity (6.13.7): inverted when TS1 are received inverted on Lane 1.
        lane1_polarity = Signal()
        self.comb += pipes[1].serdes_rx_polarity.eq(lane1_polarity)
        self.sync += If(pipes[1].ts_rx_ts1_inv & ~pipes[1].ts_rx_ts1,
            lane1_polarity.eq(~lane1_polarity)
        )

        # TS1/TS2 received on both lanes (6.13.6), Idle handshake on both lanes.
        lane1_ts1 = pipes[1].ts_rx_ts1 | pipes[1].ts_rx_ts1_inv
        self.both_ts1     = both_ts1     = LanesEvent(pipes[0].ts_rx_ts1,     lane1_ts1)
        self.both_ts1_inv = both_ts1_inv = LanesEvent(pipes[0].ts_rx_ts1_inv, lane1_ts1)
        self.both_ts2     = both_ts2     = LanesEvent(pipes[0].ts_rx_ts2,     pipes[1].ts_rx_ts2)
        self.comb += [
            ltssm.ts_rx_ts1.eq(both_ts1.o),
            ltssm.ts_rx_ts1_inv.eq(both_ts1_inv.o),
            ltssm.ts_rx_ts2.eq(both_ts2.o),
            ltssm.idle_handshake_done.eq(
                pipes[0].idle_handshake_done & pipes[1].idle_handshake_done),
        ]

        # SuperSpeedPlus Polling (Lane 0) ----------------------------------------------------------
        self.add_ssp_polling(pipes[0], serdes[0], sys_clk_freq, role, timer_scale,
            x2_capable=x2_capable, gen2_capable=gen2_capable)

    def add_ssp_polling(self, pipe, serdes, sys_clk_freq, role, timer_scale, x2_capable,
        gen2_capable):
        """SuperSpeedPlus Polling (SCD1/SCD2, LBPM PHY Capability/Ready, SSPPolling,
        7.5.4.3-7.5.4.6) on the Configuration Lane in Polling.LFPS: x2/gen2 are the negotiated
        lanes/speed."""
        self.ssp  = ssp = SSPPolling(sys_clk_freq, timer_scale=timer_scale,
            x2_capable   = x2_capable,
            gen2_capable = gen2_capable)
        self.x2   = Signal() # o: Gen X2 (two lanes) negotiated.
        self.gen2 = Signal() # o: Gen 2 (10Gbps) negotiated.
        self.comb += [
            ssp.start.eq(self.ltssm.ssp_polling),
            ssp.ds_port.eq(role == "host"),
            ssp.idle.eq(serdes.rx_idle),
            self.ltssm.ssp_done.eq(ssp.done),
            self.ltssm.ssp_fallback.eq(ssp.ss_fallback),
            # TX: SuperSpeedPlus Polling LFPS, else the PIPE (LFPS/electrical idle/data).
            pipe.lfps_tx_ssp.eq(ssp.tx_active),
            pipe.lfps_tx_ssp_lfps.eq(ssp.tx_lfps),
        ]
        self.sync += If(ssp.done,
            self.x2.eq(ssp.x2),
            self.gen2.eq(ssp.gen2),
        )

    @property
    def reset(self):
        """PHY reset (sys clock domain)."""
        return self.reset_sys

    def do_finalize(self):
        # Gen2PIPE tx/rx clock domains reset with the PHY reset (else, after a PHY reset, the sys sides
        # of the Gen 2 TX units/RX words buffers restart while their tx/rx sides keep their state:
        # units started before being written, Logical Idle Data Blocks inside the units).
        if hasattr(self, "gen2_pipe"):
            self.specials += [
                MultiReg(self.reset_sys, self.gen2_pipe.reset_tx, "tx"),
                MultiReg(self.reset_sys, self.gen2_pipe.reset_rx, "rx"),
            ]

    def add_pairs_adapters(self, pipe):
        """Words pairs (link_words=2) <-> Gen 1 PIPE words: returns the (sink, source) pairs streams."""
        from usb3_pipe.framing import WordsToPairs, PairsToWords
        to_words = PairsToWords()
        to_pairs = WordsToPairs()
        self.submodules += to_words, to_pairs
        self.comb += [
            to_words.source.connect(pipe.sink),
            pipe.source.connect(to_pairs.sink),
        ]
        return to_words.sink, to_pairs.source

    def add_gen2_datapath(self, pipe):
        """Link Layer words: Gen2PIPE (Gen 2 packet framing) while gen2_active, else Gen 1 PIPE
        (words or words pairs: link_words)."""
        from usb3_pipe.framing import gen2_link_layout
        layout = [("data", 32), ("ctrl", 4)] if self.link_words == 1 else gen2_link_layout()
        self.sink   = sink   = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        gen2_sink   = stream.Endpoint(layout)
        gen2_source = stream.Endpoint(layout)
        if self.link_words == 1:
            pipe_sink, pipe_source = pipe.sink, pipe.source
        else:
            pipe_sink, pipe_source = self.add_pairs_adapters(pipe)
        # LGOOD_n/LCRD_x Link Commands words (Gen 2, PHY words order), not behind the RX words, when
        # enabled by the Link Layer (source_lc_enable, else in source).
        self.source_lc        = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source_lc_enable = Signal()
        self.comb += self.gen2_pipe.source_lc_enable.eq(self.source_lc_enable)
        gen2_source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        if self.gen2_endianness_swap:
            # Gen2PIPE words: Symbol 0 on data[0:8], PHY words: Symbol 0 on data[24:32].
            self.submodules += EndiannessSwap(gen2_sink, self.gen2_pipe.sink, word_bytes=4)
            self.submodules += EndiannessSwap(self.gen2_pipe.source, gen2_source, word_bytes=4)
            self.submodules += EndiannessSwap(self.gen2_pipe.source_lc, gen2_source_lc)
        else:
            self.comb += [
                gen2_sink.connect(self.gen2_pipe.sink),
                self.gen2_pipe.source.connect(gen2_source),
                self.gen2_pipe.source_lc.connect(gen2_source_lc),
            ]
        self.comb += [
            gen2_source_lc.connect(source_lc, omit={"valid", "ready"}),
            source_lc.valid.eq(gen2_source_lc.valid & self.gen2_active),
            gen2_source_lc.ready.eq(source_lc.ready | ~self.gen2_active),
        ]
        self.comb += [
            If(self.gen2_active,
                sink.connect(gen2_sink),
                gen2_source.connect(source),
            ).Else(
                sink.connect(pipe_sink),
                pipe_source.connect(source),
            )
        ]

    def add_gen2(self, pipe, serdes, with_endianness_swap=True):
        """Gen 2x1 operation (gen2_capable SerDes): once Gen 2 is negotiated in the SuperSpeedPlus
        Polling, the line rate switches to 10Gbps (serdes.rate_10g) and the LTSSM Training Sequences
        and Idle Symbol handshake (GEN2_PIPE_INTERFACE) are done by the Gen2PIPE on the raw
        transceiver words (128b/132b blocks), the LFPS stay on the Gen 1 PIPE (LFPSUnit). Back to
        5Gbps for the next SuperSpeedPlus Polling."""
        ltssm = self.ltssm
        self.gen2_endianness_swap = with_endianness_swap
        # tx/rx clock domains reset with the PHY (sys) reset, synchronized (do_finalize): the TX units
        # and RX words buffers have their write/read sides in different clock domains.
        self.gen2_pipe = gen2_pipe = ResetInserter(["tx", "rx"])(Gen2PIPE(link_words=self.link_words))
        self.gen2_active = gen2_active = Signal() # o: Gen 2 operation (10Gbps).
        self.sync += gen2_active.eq(self.gen2 & ~ltssm.ssp_polling)
        self.comb += [
            serdes.rate_10g.eq(gen2_active),
            serdes.gen2_tx_enable.eq(gen2_active),
            serdes.gen2_tx_data.eq(gen2_pipe.tx_data),
            gen2_pipe.rx_data.eq(serdes.gen2_rx_data),
        ]
        # Gen 2 RX errors (Warm Reset LFPS detection at 10Gbps, see USB3SerDes.add_lfps): block
        # alignment lost.
        gen2_aligned = Signal()
        self.specials += MultiReg(gen2_pipe.rx.aligner.aligned, gen2_aligned)
        self.comb += serdes.gen2_rx_errors.eq(~gen2_aligned)
        # LTSSM <-> Gen 1 PIPE / Gen2PIPE. No comma alignment at Gen 2: the transceiver comma aligner
        # would slip bits on comma-like patterns of the raw 128b/132b words (block alignment by the
        # Gen 2 RX datapath).
        for name, width, direction in LTSSM_PIPE_INTERFACE:
            if name == "serdes_rx_align":
                self.comb += pipe.serdes_rx_align.eq(ltssm.serdes_rx_align & ~gen2_active)
            elif name not in GEN2_PIPE_INTERFACE:
                self.comb += connect_ltssm_pipe_signal(ltssm, pipe, name, direction)
            elif direction == "o":
                self.comb += [
                    getattr(pipe,      name).eq(getattr(ltssm, name) & ~gen2_active),
                    getattr(gen2_pipe, name).eq(getattr(ltssm, name) &  gen2_active),
                ]
            else:
                self.comb += getattr(ltssm, name).eq(
                    Mux(gen2_active, getattr(gen2_pipe, name), getattr(pipe, name)))

    def add_lanes_datapath(self, deskew_depth):
        """Gen 1x2 datapath: data striping (6.13.4): 64-bit/8-bit link words (Symbol 0 on
        data[0:8]), even Symbols on Lane 0, odd Symbols on Lane 1 (LaneStriper/LaneDestriper), lanes
        deskew on the TS1/TS2 COM words (LaneDeskew, Table 6-34: 6.4ns max at the receiver, a
        fraction of a word), re-acquired on each TS1 phase (Polling.Active, Recovery.Active)."""
        pipes = self.pipes
        ltssm = self.ltssm
        self.sink   = sink   = stream.Endpoint(link_layout())
        self.source = source = stream.Endpoint(link_layout())

        # TX: Striping -----------------------------------------------------------------------------
        self.striper = striper = LaneStriper()
        self.comb += sink.connect(striper.sink)
        for lane in range(2):
            self.comb += striper.sources[lane].connect(pipes[lane].sink)

        # RX: Deskew / Destriping ------------------------------------------------------------------
        # The deskew is re-acquired on the TS1/TS2 COM words (every 4 words) after each TSEQ/TS1
        # phase, pairing the COM words received within 1 word on both lanes (6.4ns max lane-to-lane
        # skew at the receiver, Table 6-34: less than a word, the word alignment of each lane on COM
        # can add one word).
        deskew = LaneDeskew(COM.value*0x01010101, 0b1111, depth=deskew_depth, max_skew=1)
        deskew = ResetInserter()(deskew)
        self.deskew    = deskew
        self.destriper = destriper = LaneDestriper()
        self.comb += deskew.reset.eq(ltssm.ts_tx_ts1 | ltssm.ts_tx_tseq)
        for lane in range(2):
            self.comb += [
                pipes[lane].source.connect(deskew.sinks[lane]),
                deskew.sources[lane].connect(destriper.sinks[lane]),
            ]
        self.comb += [
            destriper.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(destriper.source.valid & ltssm.rx_ready),
            destriper.source.ready.eq(source.ready | ~ltssm.rx_ready),
        ]
        self.deskew_aligned = deskew.aligned # o
