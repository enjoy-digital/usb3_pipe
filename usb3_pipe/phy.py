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

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import COM
from usb3_pipe.core       import USB3PIPE
from usb3_pipe.ltssm      import USB3LTSSM, SSPPolling
from usb3_pipe.scrambling import LFSR_SEED_LANE0, LFSR_SEED_LANE1
from usb3_pipe.lanes      import LaneStriper, LaneDestriper, LaneDeskew, link_layout

# LTSSM <-> PIPE -----------------------------------------------------------------------------------

def connect_ltssm_pipe(ltssm, pipe, with_rx_ts_status=True):
    """LTSSM <-> PIPE wiring (LFPS/TS face).

    with_rx_ts_status: TS1/TS2 received and Idle Symbol handshake status from this PIPE (else
    combined over the lanes by the caller).
    """
    return [
        # LTSSM -> PIPE
        pipe.lfps_tx_polling.eq(ltssm.lfps_tx_polling),
        pipe.lfps_tx_idle.eq(ltssm.lfps_tx_idle),
        pipe.lfps_tx_ping.eq(ltssm.lfps_tx_ping),
        pipe.lfps_tx_exit_u1.eq(ltssm.lfps_tx_exit_u1),
        pipe.lfps_tx_exit_u2.eq(ltssm.lfps_tx_exit_u2),
        pipe.lfps_tx_wakeup_u3.eq(ltssm.lfps_tx_wakeup_u3),
        pipe.lfps_tx_reset.eq(ltssm.lfps_tx_reset),
        pipe.ts_tx_enable.eq(ltssm.ts_tx_enable),
        pipe.ts_tx_tseq.eq(ltssm.ts_tx_tseq),
        pipe.ts_tx_ts1.eq(ltssm.ts_tx_ts1),
        pipe.ts_tx_ts2.eq(ltssm.ts_tx_ts2),
        pipe.ts_tx_ts2_reset.eq(ltssm.tx_ts2_reset),
        pipe.ts_tx_ts2_loopback.eq(ltssm.tx_ts2_loopback),
        pipe.ts2_decode.eq(ltssm.ts2_decode),

        pipe.serdes_rx_align.eq(ltssm.serdes_rx_align),
        pipe.serdes_rx_polarity.eq(ltssm.serdes_rx_polarity),

        pipe.rx_ready.eq(ltssm.rx_ready),
        pipe.tx_ready.eq(ltssm.tx_ready),
        pipe.idle_handshake.eq(ltssm.idle_handshake),

        # PIPE -> LTSSM
        ltssm.lfps_rx_polling.eq(pipe.lfps_rx_polling),
        ltssm.lfps_tx_count.eq(pipe.lfps_tx_count),
        ltssm.lfps_rx_ping.eq(pipe.lfps_rx_ping),
        ltssm.lfps_rx_exit_u1.eq(pipe.lfps_rx_exit_u1),
        ltssm.lfps_rx_exit_u2.eq(pipe.lfps_rx_exit_u2),
        ltssm.lfps_rx_wakeup_u3.eq(pipe.lfps_rx_wakeup_u3),
        ltssm.lfps_rx_reset.eq(pipe.lfps_rx_reset),

        *([
        ltssm.ts_rx_ts1.eq(pipe.ts_rx_ts1),
        ltssm.ts_rx_ts1_inv.eq(pipe.ts_rx_ts1_inv),
        ltssm.ts_rx_ts2.eq(pipe.ts_rx_ts2),
        ] if with_rx_ts_status else []),
        ltssm.rx_ts2_reset.eq(pipe.ts_rx_ts2_reset),
        ltssm.rx_ts2_loopback.eq(pipe.ts_rx_ts2_loopback),
        ltssm.ts_tx_done.eq(pipe.ts_tx_done),
        *([ltssm.rx_idle8.eq(pipe.idle_done)] if with_rx_ts_status else []),
    ]

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

@ResetInserter()
class USB3PHY(LiteXModule):
    """USB3 PHY: USB3PIPE(s) + USB3LTSSM.

    Combines the PIPE (LFPS/TS/scrambling/datapath) with the LTSSM to provide a link-trained
    data/ctrl (K-char) symbol stream in the sys clock domain, ready for a USB3 Link Layer. The
    SerDes is not a submodule and must be added by the caller.

    - lanes=1 (Gen 1x1): serdes is a SerDes, 32-bit data/4-bit ctrl link words.
    - lanes=2 (Gen 1x2, 6.13): serdes is a list of 2 SerDes (Lane 0: Configuration Lane), 64-bit
      data/8-bit ctrl link words (Symbol 0 on data[0:8], no endianness swap). x2 is the negotiated
      Gen 1x2 operation (PHY Capability LBPM, x2_capable: advertised capability).

    Link interface (what a USB3 Link Layer needs from the PHY, the Link Layer does not have to know
    the PHY internals):
    - `sink`/`source`: data/ctrl symbol streams (Logical Idle sent when `sink` has no data).
    - Status (see USB3PHY_STATUS): mirrored from the LTSSM.
    - Link-Layer control (see USB3PHY_LINK_CTRL): PM handshake (LGO_Ux/LAU/LXU/LPMA decoded or to
      send by the Link Layer), directed Recovery/Hot Reset/Warm Reset and Ux exit requests. The
      signals are the LTSSM ones (same Signal objects); undriven inputs keep their reset values.
    """
    def __init__(self, serdes, sys_clk_freq, role="device", with_endianness_swap=True,
        timer_scale=1, lanes=1, x2_capable=True, deskew_depth=8):
        assert role in ["device", "host"]
        assert lanes in [1, 2]
        serdes = [serdes] if lanes == 1 else serdes
        assert len(serdes) == lanes
        assert (lanes == 1) or (not with_endianness_swap)

        # PIPE(s) / LTSSM --------------------------------------------------------------------------
        if lanes == 1:
            self.pipe  = pipe  = USB3PIPE(serdes[0],
                sys_clk_freq         = sys_clk_freq,
                with_endianness_swap = with_endianness_swap,
                timer_scale          = timer_scale)
            self.pipes = [pipe]
            self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale)
            self.comb += connect_ltssm_pipe(ltssm, pipe)
            self.comb += ltssm.ds_port.eq(role == "host")
        else:
            self.add_lanes(serdes, sys_clk_freq, role, timer_scale, x2_capable)
            ltssm = self.ltssm

        # SerDes RX Re-initialization --------------------------------------------------------------
        # The transceiver RX initialization (PMA/CDR/offset calibration) does not always end in a
        # good state: on hardware (Artix7 GTP, xHCI host with SSC) ~1/12 RX resets ended with a
        # receiver unable to train or with a high error rate, recovered by another RX reset. Re-run
        # it on each training failure (and Warm Reset), when the LTSSM exits to Rx.Detect, or when
        # requested (serdes_rx_reset, ex: debug). SerDes without RX reset control: no-op.
        self.serdes_rx_reset = Signal() # i
        for s in serdes:
            if hasattr(s, "rx_reset"):
                self.comb += s.rx_reset.eq(ltssm.exit_to_rx_detect | self.serdes_rx_reset)

        # Endpoints --------------------------------------------------------------------------------
        if lanes == 1:
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

    def add_lanes(self, serdes, sys_clk_freq, role, timer_scale, x2_capable):
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
        for name in ["lfps_tx_idle", "ts_tx_enable", "ts_tx_tseq", "ts_tx_ts1",
            "ts_tx_ts2", "serdes_rx_align", "rx_ready", "tx_ready", "idle_handshake", "ts2_decode"]:
            self.comb += getattr(pipes[1], name).eq(getattr(ltssm, name))
        self.comb += [
            pipes[1].ts_tx_ts2_reset.eq(ltssm.tx_ts2_reset),
            pipes[1].ts_tx_ts2_loopback.eq(ltssm.tx_ts2_loopback),
        ]

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
            ltssm.rx_idle8.eq(pipes[0].idle_done & pipes[1].idle_done),
        ]

        # SuperSpeedPlus Polling (Lane 0) ----------------------------------------------------------
        self.ssp = ssp = SSPPolling(sys_clk_freq, timer_scale=timer_scale, x2_capable=x2_capable)
        self.x2  = Signal() # o: Gen 1x2 negotiated.
        self.comb += [
            ssp.start.eq(ltssm.ssp_polling),
            ssp.ds_port.eq(role == "host"),
            ssp.idle.eq(serdes[0].rx_idle),
            ltssm.ssp_done.eq(ssp.done),
            ltssm.ssp_fallback.eq(ssp.ss_fallback),
            # Lane 0 TX: SuperSpeedPlus Polling LFPS, else the PIPE (LFPS/electrical idle/data).
            pipes[0].lfps_tx_ssp.eq(ssp.tx_active),
            pipes[0].lfps_tx_ssp_lfps.eq(ssp.tx_lfps),
        ]
        self.sync += If(ssp.done, self.x2.eq(ssp.x2))

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
