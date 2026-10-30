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
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import COM, LTSSM_PIPE_INTERFACE, EndiannessSwap, AsyncFIFO, Buffer
from usb3_pipe.core       import USB3PIPE, Gen2PIPE, GEN2_PIPE_INTERFACE
from usb3_pipe.datapath   import GEN2_DATA_WIDTH
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

# PIPE -> LTSSM status pulses (LTSSM_PIPE_INTERFACE inputs, others are levels).
LTSSM_PIPE_PULSES = [
    "lfps_rx_polling", "lfps_rx_ping", "ts_tx_done", "ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2",
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
    - with_u1u2=False (Gen 1x1): no U1/U2 Link Power Management (less logic, see USB3LTSSM).
    - pipe_cd (Gen 1x1): USB3PIPE (and SerDes, renamed by the caller) in the pipe_cd clock domain at
      pipe_clk_freq (>= 125MHz), LTSSM and Link interface in sys (add_pipe_cdc): ex. sys at 62.5MHz
      with link_words=2 on FPGAs where the LTSSM/Link Layer can't reach 125MHz.

    Link interface (what a USB3 Link Layer needs from the PHY, the Link Layer does not have to know
    the PHY internals):
    - `sink`/`source`: data/ctrl symbol streams (Logical Idle sent when `sink` has no data).
    - Status (see USB3PHY_STATUS): mirrored from the LTSSM.
    - Link-Layer control (see USB3PHY_LINK_CTRL): PM handshake (LGO_Ux/LAU/LXU/LPMA decoded or to
      send by the Link Layer), directed Recovery/Hot Reset/Warm Reset and Ux exit requests. The
      signals are the LTSSM ones (same Signal objects); undriven inputs keep their reset values.
    """
    def __init__(self, serdes, sys_clk_freq, role="device", with_endianness_swap=True,
        timer_scale=1, lanes=1, x2_capable=True, gen2_capable=False, deskew_depth=8, link_words=1,
        pipe_cd=None, pipe_clk_freq=None, with_u1u2=True):
        assert role in ["device", "host"]
        assert lanes in [1, 2]
        serdes = [serdes] if lanes == 1 else serdes
        assert len(serdes) == lanes
        assert (lanes == 1) or (not with_endianness_swap)
        assert link_words in [1, 2]
        assert (lanes == 1) or (link_words == 1)
        assert (pipe_cd is None) or (lanes == 1)
        assert (pipe_cd is None) or (not gen2_capable) or (link_words == 2)
        self.link_words = link_words # Link Layer words (1: 32-bit) or words pairs (2: 64-bit).
        self.pipe_cd    = pipe_cd
        self.with_u1u2  = with_u1u2 # U1/U2 Link Power Management (USB3LTSSM with_u1u2).
        assert with_u1u2 or (lanes == 1)

        # PIPE(s) / LTSSM --------------------------------------------------------------------------
        if lanes == 1:
            ssp = gen2_capable # SuperSpeedPlus Polling (Gen 2x1 capability).
            pipe = USB3PIPE(serdes[0],
                sys_clk_freq         = sys_clk_freq if pipe_cd is None else pipe_clk_freq,
                with_endianness_swap = with_endianness_swap,
                timer_scale          = timer_scale,
                ssp                  = ssp)
            if pipe_cd is not None:
                pipe = ClockDomainsRenamer({"sys": pipe_cd})(pipe)
            self.pipe  = pipe
            self.pipes = [pipe]
            self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale,
                ssp=ssp, with_u1u2=with_u1u2)
            self.comb += ltssm.ds_port.eq(role == "host")
            if ssp:
                self.add_ssp_polling(pipe, serdes[0], sys_clk_freq, role, timer_scale,
                    x2_capable=False, gen2_capable=True)
            if ssp and getattr(serdes[0], "gen2_capable", False):
                self.add_gen2(pipe, serdes[0], with_endianness_swap)
            elif pipe_cd is not None:
                self.add_ltssm_pipe_cdc(ltssm, pipe, pipe_cd)
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
            if pipe_cd is None:
                self.comb += s.rx_reset.eq(ltssm.exit_to_rx_detect | self.serdes_rx_reset)
            else:
                self.specials += MultiReg(ltssm.exit_to_rx_detect | self.serdes_rx_reset,
                    s.rx_reset, pipe_cd)

        # Endpoints --------------------------------------------------------------------------------
        if lanes == 1:
            if hasattr(self, "gen2_pipe") and (pipe_cd is None):
                self.add_gen2_datapath(pipe)
            elif pipe_cd is not None:
                self.sink, self.source = self.add_pipe_cdc(pipe, pipe_cd)
            elif link_words == 2:
                self.sink, self.source = self.add_pairs_adapters(pipe)
            else:
                self.sink   = pipe.sink
                self.source = pipe.source
        else:
            self.add_lanes_datapath(deskew_depth)

        # Status -----------------------------------------------------------------------------------
        self.ready = Signal() # SerDes ready (PLL locked, transceiver reset done).
        if pipe_cd is None:
            self.comb += self.ready.eq(Reduce("AND", [s.ready for s in serdes]))
        else:
            self.specials += MultiReg(Reduce("AND", [s.ready for s in serdes]), self.ready)
        # Registered with pipe_cd (timings: LTSSM state decoding to the Link Layer, the PIPE status
        # is resynchronized to the LTSSM anyway).
        for name in USB3PHY_STATUS:
            setattr(self, name, Signal(name=name))
            if pipe_cd is None:
                self.comb += getattr(self, name).eq(getattr(ltssm, name))
            else:
                self.sync += getattr(self, name).eq(getattr(ltssm, name))

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
            self.ltssm.ssp_done.eq(ssp.done),
            self.ltssm.ssp_fallback.eq(ssp.ss_fallback),
        ]
        # TX: SuperSpeedPlus Polling LFPS, else the PIPE (LFPS/electrical idle/data). With pipe_cd,
        # resynchronized (LBPM/SCD timings in us, a few cycles of uncertainty).
        if self.pipe_cd is None:
            self.comb += [
                ssp.idle.eq(serdes.rx_idle),
                pipe.lfps_tx_ssp.eq(ssp.tx_active),
                pipe.lfps_tx_ssp_lfps.eq(ssp.tx_lfps),
            ]
        else:
            self.specials += [
                MultiReg(serdes.rx_idle, ssp.idle),
                MultiReg(ssp.tx_active, pipe.lfps_tx_ssp,      self.pipe_cd),
                MultiReg(ssp.tx_lfps,   pipe.lfps_tx_ssp_lfps, self.pipe_cd),
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

    def add_ltssm_pipe_cdc(self, ltssm, pipe, pipe_cd):
        """LTSSM (sys) <-> PIPE (pipe_cd) wiring (LTSSM_PIPE_INTERFACE): LTSSM controls (levels) and
        PIPE status levels resynchronized, PIPE status pulses (LTSSM_PIPE_PULSES) through
        PulseSynchronizers (pulses spaced by more than a few sys cycles: Training Sequences
        detections/bursts ends, LFPS detections), lfps_tx_count updated when stable (2 equal
        resynchronized samples: incremented once per LFPS burst)."""
        for name, width, direction in LTSSM_PIPE_INTERFACE:
            if direction == "o":
                self.ltssm_to_pipe(getattr(ltssm, name), getattr(pipe, name), pipe_cd)
            else:
                self.pipe_to_ltssm(name, getattr(pipe, name), getattr(ltssm, name), pipe_cd)

    def ltssm_to_pipe(self, value, pipe_s, pipe_cd):
        """LTSSM control (level, sys) -> PIPE (pipe_cd)."""
        self.specials += MultiReg(value, pipe_s, pipe_cd)

    def pipe_to_ltssm(self, name, pipe_s, ltssm_s, pipe_cd):
        """PIPE status (pipe_cd) -> LTSSM (sys): pulses (LTSSM_PIPE_PULSES), multi-bit values (taken
        when stable) or levels."""
        width = len(pipe_s)
        if name in LTSSM_PIPE_PULSES:
            # Registered pulse (timings) to the PulseSynchronizer.
            pulse = Signal()
            sync  = getattr(self.sync, pipe_cd)
            sync += pulse.eq(pipe_s)
            ps = PulseSynchronizer(pipe_cd, "sys")
            self.submodules += ps
            self.comb += [ps.i.eq(pulse), ltssm_s.eq(ps.o)]
        elif width > 1:
            s_sys = Signal(width)
            s_d   = Signal(width)
            self.specials += MultiReg(pipe_s, s_sys)
            self.sync += [
                s_d.eq(s_sys),
                If(s_sys == s_d, ltssm_s.eq(s_sys)),
            ]
        else:
            self.specials += MultiReg(pipe_s, ltssm_s)

    def add_pipe_cdc(self, pipe, pipe_cd, fifo_depth=32):
        """Link words (link_words=1) or words pairs (link_words=2) sys <-> pipe_cd through AsyncFIFOs
        (registered FIFO writes: timings): returns the (sink, source) streams.

        With link_words=2 and the Link Layer at half the PIPE frequency, the RX Logical Idle words
        between the packets are dropped and the RX words packed in full pairs (the Link Layer consumes
        one word per cycle outside of the DPP data), the TX packets (last: end of a Link Command, Header Packet or DPH + DPP, the
        Link Layer sends single words at the PIPE words rate/2 outside of the DPP data) are stored
        and forwarded (PacketsBuffer): no Logical Idle inside the packets. The TX words received by
        the PIPE while not ready (tx_ready, outside of U0) are dropped: no stale words at the next U0
        entry (the Link Layer only sends words in U0, the PIPE tx_ready resynchronization is shorter
        than the FIFO latency). The RX path has no back-pressure (received words at the line rate:
        the Link Layer reads its words at least at the received words rate)."""
        from usb3_pipe.framing import gen2_link_layout, WordsPacker, PairsToWords, PacketsBuffer
        from usb3_pipe.framing import LogicalIdleFilter
        layout = [("data", 32), ("ctrl", 4)] if self.link_words == 1 else gen2_link_layout()
        sink   = stream.Endpoint(layout)
        source = stream.Endpoint(layout)
        if self.link_words == 1:
            pipe_sink, pipe_source = stream.Endpoint(layout), stream.Endpoint(layout)
            self.comb += [
                pipe_sink.connect(pipe.sink, omit={"ready"}),
                pipe_sink.ready.eq(pipe.sink.ready | ~pipe.tx_ready),
                pipe.source.connect(pipe_source, omit={"ready"}),
                pipe.source.ready.eq(1),
            ]
        else:
            self.tx_packets = tx_packets = ClockDomainsRenamer(pipe_cd)(PacketsBuffer(layout))
            to_words = ClockDomainsRenamer(pipe_cd)(PairsToWords())
            # Registered PIPE output (timings: framing aligner to the Logical Idle filter decoding).
            rx_pipe  = ClockDomainsRenamer(pipe_cd)(stream.Buffer([("data", 32), ("ctrl", 4)]))
            rx_idle  = ClockDomainsRenamer(pipe_cd)(LogicalIdleFilter())
            rx_words = ClockDomainsRenamer(pipe_cd)(Buffer([("data", 32), ("ctrl", 4)],
                pipe_ready=True))
            to_pairs = ClockDomainsRenamer(pipe_cd)(WordsPacker())
            self.submodules += to_words, rx_pipe, rx_idle, rx_words, to_pairs
            # TX skid buffer (registered ready: PIPE TX ready logic isolated from the packets
            # buffer), emptied while the PIPE is not ready (no stale words).
            tx_skid = ResetInserter()(Buffer([("data", 32), ("ctrl", 4)], pipe_ready=True))
            self.tx_skid = tx_skid = ClockDomainsRenamer(pipe_cd)(tx_skid)
            self.comb += [
                to_words.source.connect(tx_skid.sink),
                tx_skid.source.connect(pipe.sink),
                # RX: no back-pressure (received words at the line rate), always ready.
                pipe.source.connect(rx_pipe.sink, omit={"ready"}),
                pipe.source.ready.eq(1),
                rx_pipe.source.connect(rx_idle.sink, omit={"ready"}),
                rx_pipe.source.ready.eq(1),
                rx_idle.source.connect(rx_words.sink, omit={"ready"}),
                rx_idle.source.ready.eq(1),
                rx_words.source.connect(to_pairs.sink, omit={"ready"}),
                rx_words.source.ready.eq(1),
            ]
            # Skid buffer reset registered (fanout).
            sync  = getattr(self.sync, pipe_cd)
            sync += tx_skid.reset.eq(~pipe.tx_ready)
            pipe_sink, pipe_source = tx_packets.sink, to_pairs.source

            # Gen 2 (Gen2PIPE, words pairs) or Gen 1 words, selected by gen2_active.
            if hasattr(self, "gen2_pipe"):
                pipe_source = self.add_gen2_pipe_cdc(pipe, pipe_cd, layout, tx_packets, to_words,
                    to_pairs)
            else:
                self.comb += [
                    tx_packets.drop.eq(~pipe.tx_ready),
                    tx_packets.source.connect(to_words.sink),
                ]

        # TX: sys -> pipe_cd (registered ready: the Link Layer TX path from a register). Gen 2:
        # words to the Gen2PIPE framing (sys) while gen2_active.
        self.tx_cdc_buffer = tx_buffer = Buffer(layout, pipe_ready=True)
        self.tx_cdc        = tx_cdc    = ClockDomainsRenamer({"write": "sys", "read": pipe_cd})(
            AsyncFIFO(layout, fifo_depth))
        if hasattr(self, "gen2_tx_sink"):
            self.comb += If(self.gen2_active,
                sink.connect(self.gen2_tx_sink),
            ).Else(
                sink.connect(tx_buffer.sink),
            )
        else:
            self.comb += sink.connect(tx_buffer.sink)
        self.comb += [
            tx_buffer.source.connect(tx_cdc.sink),
            tx_cdc.source.connect(pipe_sink),
        ]

        # RX: pipe_cd -> sys.
        self.rx_cdc_buffer = rx_buffer = ClockDomainsRenamer(pipe_cd)(Buffer(layout,
            pipe_ready=True))
        # Deep (block RAM, read side at the Link Layer frequency): with words pairs at half the PIPE
        # frequency, the Link Layer consumes the Header Packets/DPP tails one word per cycle and
        # back-to-back DPH+DPPs (Bulk OUT bursts) accumulate words (~11 per 1024-byte DPP).
        # Gen 2: the Link Layer (64-bit at half the PIPE frequency) reads at about half the received
        # rate (~64 entries per 1024-byte DPP) and up to 8 Header Packets (Type 1/Type 2 credits) can
        # be received after the throttling: deeper FIFO, throttling from a quarter.
        gen2     = hasattr(self, "gen2_pipe")
        rx_depth = fifo_depth if self.link_words == 1 else (1024 if gen2 else 256)
        self.rx_cdc        = rx_cdc    = ClockDomainsRenamer({"write": pipe_cd, "read": "sys"})(
            stream.AsyncFIFO(layout, rx_depth, buffered=True))
        self.comb += [
            pipe_source.connect(rx_buffer.sink, omit={"ready"}),
            pipe_source.ready.eq(1),
            rx_buffer.source.connect(rx_cdc.sink, omit={"ready"}),
            rx_buffer.source.ready.eq(1), # Dropped when the FIFO is full (Link Layer not reading).
        ]
        if hasattr(self, "gen2_rx_source"):
            # Gen 2 words (sys) while gen2_active, else Gen 1 words.
            self.comb += [
                self.gen2_rx_source.ready.eq(1),
                rx_cdc.source.ready.eq(1),
                If(self.gen2_active,
                    self.gen2_rx_source.connect(source, omit={"ready"}),
                    self.gen2_rx_source.ready.eq(source.ready),
                ).Else(
                    rx_cdc.source.connect(source),
                )
            ]
        else:
            self.comb += rx_cdc.source.connect(source)

        # RX throttling (sys): rx_cdc level >= half, the Link Layer then holds its Rx Header Buffer
        # Credits (LCRD_x) so the partner stops after its outstanding Header Packets (and their
        # DPPs): bounded RX backlog with a Link Layer slower than the line rate. Level from a
        # resynchronized (Gray) count of the written entries and the count of the read entries.
        self.rx_throttle = Signal() # o
        if self.link_words == 2:
            cw       = bits_for(rx_depth) + 1
            wr_count = Signal(cw)
            wr_gray  = Signal(cw)
            wr_sys   = Signal(cw)
            rd_count = Signal(cw)
            wr_bin   = Signal(cw)
            level    = Signal(cw)
            throttle = Signal()
            getattr(self.sync, pipe_cd).__iadd__([
                If(rx_cdc.sink.valid & rx_cdc.sink.ready, wr_count.eq(wr_count + 1)),
                wr_gray.eq(wr_count ^ (wr_count >> 1)),
            ])
            self.specials += MultiReg(wr_gray, wr_sys)
            # Gray -> binary (per bit signals: no self-referencing combinatorial assignment).
            bins = [Signal() for _ in range(cw)]
            self.comb += [bins[cw-1].eq(wr_sys[cw-1])] + [
                bins[i].eq(bins[i+1] ^ wr_sys[i]) for i in reversed(range(cw-1))]
            self.comb += wr_bin.eq(Cat(*bins))
            self.sync += [
                If(source.valid & source.ready, rd_count.eq(rd_count + 1)),
                level.eq(wr_bin - rd_count),
                throttle.eq(level >= (rx_depth//4 if gen2 else rx_depth//2)),
            ]
            self.comb += self.rx_throttle.eq(throttle)
            if hasattr(self, "gen2_rx_source"):
                # Gen 2: words buffer level (entries of up to 8 words, 2048 bytes as above).
                gen2_throttle = Signal()
                self.sync += gen2_throttle.eq(self.gen2_pipe.rx_level >= 64)
                self.comb += If(self.gen2_active, self.rx_throttle.eq(gen2_throttle))
        return sink, source

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
        pipe_cd = self.pipe_cd
        # Raw words width (transceiver words at 10Gbps): from the SerDes (ex: 80-bit at 125MHz).
        gen2_data_width = getattr(serdes, "gen2_data_width", GEN2_DATA_WIDTH)
        # With pipe_cd: TX framing at the Link Layer frequency (sys, "gen2_link"), the rest of the
        # Gen2PIPE in pipe_cd (renamed in this order).
        gen2_pipe = ResetInserter(["tx", "rx"])(Gen2PIPE(link_words=self.link_words,
            data_width=gen2_data_width, link_cd=None if pipe_cd is None else "gen2_link"))
        if pipe_cd is not None:
            gen2_pipe = ClockDomainsRenamer({"sys": pipe_cd, "gen2_link": "sys"})(gen2_pipe)
        self.gen2_pipe = gen2_pipe
        self.gen2_active = gen2_active = Signal() # o: Gen 2 operation (10Gbps).
        self.sync += gen2_active.eq(self.gen2 & ~ltssm.ssp_polling)
        # gen2_active in the PIPE/SerDes clock domain.
        if pipe_cd is None:
            self.gen2_active_pipe = gen2_active_pipe = gen2_active
        else:
            self.gen2_active_pipe = gen2_active_pipe = Signal()
            self.specials += MultiReg(gen2_active, gen2_active_pipe, pipe_cd)
        self.comb += [
            serdes.rate_10g.eq(gen2_active_pipe),
            serdes.gen2_tx_enable.eq(gen2_active_pipe),
            serdes.gen2_tx_data.eq(gen2_pipe.tx_data),
            gen2_pipe.rx_data.eq(serdes.gen2_rx_data),
        ]
        # Gen 2 RX errors (Warm Reset LFPS detection at 10Gbps, see USB3SerDes.add_lfps): block
        # alignment lost.
        gen2_aligned = Signal()
        self.specials += MultiReg(gen2_pipe.rx.aligner.aligned, gen2_aligned,
            "sys" if pipe_cd is None else pipe_cd)
        self.comb += serdes.gen2_rx_errors.eq(~gen2_aligned)
        if pipe_cd is not None:
            self.add_gen2_ltssm_cdc(ltssm, pipe, gen2_pipe, gen2_active, pipe_cd)
            return
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

    def add_gen2_pipe_cdc(self, pipe, pipe_cd, layout, tx_packets, to_words, to_pairs):
        """Gen 2 words pairs with pipe_cd (see add_pipe_cdc): TX packets to the Gen2PIPE or the Gen 1
        words, RX words pairs from the Gen2PIPE or the Gen 1 words (returned), LGOOD_n/LCRD_x
        Link Commands (source_lc) through their own AsyncFIFO."""
        gen2_pipe = self.gen2_pipe
        gen2      = self.gen2_active_pipe
        gen2_sink   = stream.Endpoint(layout)
        gen2_source = stream.Endpoint(layout)
        gen2_lc     = stream.Endpoint([("data", 32), ("ctrl", 4)])
        if self.gen2_endianness_swap:
            # Gen2PIPE words: Symbol 0 on data[0:8], PHY words: Symbol 0 on data[24:32].
            self.submodules += EndiannessSwap(gen2_sink, gen2_pipe.sink, word_bytes=4)
            self.submodules += EndiannessSwap(gen2_pipe.source, gen2_source, word_bytes=4)
            self.submodules += EndiannessSwap(gen2_pipe.source_lc, gen2_lc)
        else:
            self.comb += [
                gen2_sink.connect(gen2_pipe.sink),
                gen2_pipe.source.connect(gen2_source),
                gen2_pipe.source_lc.connect(gen2_lc),
            ]

        # TX: Gen 1 words through the packets buffer, Gen 2 words to the Gen2PIPE framing (sys,
        # gen2_tx_sink, see add_pipe_cdc).
        self.comb += [
            tx_packets.drop.eq(~pipe.tx_ready),
            tx_packets.source.connect(to_words.sink),
        ]
        self.gen2_tx_sink = gen2_sink

        # LGOOD_n/LCRD_x Link Commands words (Gen 2, PHY words order), not behind the RX words, when
        # enabled by the Link Layer (source_lc_enable, else in source).
        self.source_lc        = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source_lc_enable = Signal()

        # Gen 2 RX words/Link Commands already in sys (framing at the Link Layer frequency, 32
        # bytes per cycle): selected after the Gen 1 words CDC (add_pipe_cdc), RX throttling from the
        # Gen 2 words buffer level.
        if gen2_pipe.link_rx_cd != "sys":
            self.gen2_rx_source = gen2_source
            self.comb += [
                gen2_pipe.source_lc_enable.eq(self.source_lc_enable),
                gen2_lc.connect(source_lc, omit={"valid", "ready"}),
                source_lc.valid.eq(gen2_lc.valid & self.gen2_active),
                gen2_lc.ready.eq(source_lc.ready | ~self.gen2_active),
            ]
            return to_pairs.source

        # RX (no back-pressure).
        rx_source = stream.Endpoint(layout)
        self.comb += [
            gen2_source.ready.eq(1),
            to_pairs.source.ready.eq(1),
            If(gen2,
                gen2_source.connect(rx_source, omit={"ready"}),
            ).Else(
                to_pairs.source.connect(rx_source, omit={"ready"}),
            )
        ]
        self.specials += MultiReg(self.source_lc_enable, gen2_pipe.source_lc_enable, pipe_cd)
        self.lc_cdc = lc_cdc = ClockDomainsRenamer({"write": pipe_cd, "read": "sys"})(
            stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True))
        self.comb += [
            gen2_lc.connect(lc_cdc.sink),
            lc_cdc.source.connect(source_lc, omit={"valid", "ready"}),
            source_lc.valid.eq(lc_cdc.source.valid & self.gen2_active),
            lc_cdc.source.ready.eq(source_lc.ready | ~self.gen2_active),
        ]
        return rx_source

    def add_gen2_ltssm_cdc(self, ltssm, pipe, gen2_pipe, gen2_active, pipe_cd):
        """LTSSM (sys) <-> Gen 1 PIPE / Gen2PIPE (pipe_cd) wiring: GEN2_PIPE_INTERFACE controls gated
        by gen2_active then resynchronized to each PIPE, status resynchronized from each PIPE then
        selected by gen2_active (sys)."""
        for name, width, direction in LTSSM_PIPE_INTERFACE:
            ltssm_s = getattr(ltssm, name)
            if name == "serdes_rx_align":
                self.ltssm_to_pipe(ltssm_s & ~gen2_active, pipe.serdes_rx_align, pipe_cd)
            elif name not in GEN2_PIPE_INTERFACE:
                if direction == "o":
                    self.ltssm_to_pipe(ltssm_s, getattr(pipe, name), pipe_cd)
                else:
                    self.pipe_to_ltssm(name, getattr(pipe, name), ltssm_s, pipe_cd)
            elif direction == "o":
                self.ltssm_to_pipe(ltssm_s & ~gen2_active, getattr(pipe,      name), pipe_cd)
                self.ltssm_to_pipe(ltssm_s &  gen2_active, getattr(gen2_pipe, name), pipe_cd)
            else:
                gen1_s = Signal(width)
                gen2_s = Signal(width)
                self.pipe_to_ltssm(name, getattr(pipe,      name), gen1_s, pipe_cd)
                self.pipe_to_ltssm(name, getattr(gen2_pipe, name), gen2_s, pipe_cd)
                self.comb += ltssm_s.eq(Mux(gen2_active, gen2_s, gen1_s))

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
