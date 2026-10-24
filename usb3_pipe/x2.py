#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 1x2 (2x5Gbps) PHY (USB 3.2 6.13), see doc/gen2-x2-roadmap.md.

Two Gen 1 lanes (one USB3PIPE per lane: 8b/10b, SKP, word alignment, TS, scrambling) controlled by a
single LTSSM, Lane 0 is the Configuration Lane:
- SuperSpeedPlus Polling (SCD1/SCD2, LBPM PHY Capability/Ready, usb3_pipe/ssp.py) on Lane 0 before
  Polling.RxEQ (7.5.4.3-7.5.4.6), LFPS on Lane 0 only (6.13.12), Lane 1 in electrical idle.
- Ordered Sets (TSEQ/TS1/TS2/SKP) transmitted simultaneously on both lanes (6.13.6), TS1/TS2
  requirements satisfied on both lanes (rx_ts1/rx_ts2 reported when seen on both lanes).
- Per lane scramblers (Lane 0 FFFFh, Lane 1 8000h, 6.13.5), per lane polarity inversion (6.13.7: the
  LTSSM handles Lane 0, Lane 1 is inverted on TS1 received inverted), per lane SKP removal.
- Lanes deskew on the TS1/TS2 COM words (LaneDeskew, Table 6-34: 6.4ns max at the receiver, a
  fraction of a word), re-acquired on each TS1 phase (Polling.Active, Recovery.Active).
- Data striping (6.13.4): 64-bit/8-bit link words (Symbol 0 on data[0:8]), even Symbols on Lane 0,
  odd Symbols on Lane 1 (LaneStriper/LaneDestriper).

The two lanes TX paths must run in lockstep (same clock, same SKP schedule: SKP Ordered Sets inserted
simultaneously on both lanes, 6.4.3.2).

Not implemented: dynamic fallback to Gen 1x1 on Lane 0 when Gen 1x1 is negotiated (x2 status
de-asserted), SuperSpeedPlus Link Layer (Type 1/Type 2 traffic, 4-bit HSEQ...), framing alignment
of the 64-bit link words.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import COM
from usb3_pipe.core       import USB3PIPE
from usb3_pipe.ltssm      import USB3LTSSM
from usb3_pipe.phy        import connect_ltssm_pipe, USB3PHY_STATUS, USB3PHY_LINK_CTRL
from usb3_pipe.scrambling import LFSR_SEED_LANE1
from usb3_pipe.lanes      import LaneStriper, LaneDestriper, LaneDeskew, link_layout
from usb3_pipe.ssp        import SSPPolling

# Helpers ------------------------------------------------------------------------------------------

class _LFPSSerDes:
    """Lane 0 SerDes view for its USB3PIPE: same signals except the LFPS/electrical idle TX controls,
    muxed with the SuperSpeedPlus Polling LFPS by the PHY."""
    def __init__(self, serdes):
        for name in ["sink", "source", "rx_align", "rx_polarity", "rx_idle", "ready", "tx_polarity"]:
            setattr(self, name, getattr(serdes, name))
        for name in ["tx_skp_enable", "rx_errors", "rx_idle_stretch", "rx_reset"]:
            if hasattr(serdes, name):
                setattr(self, name, getattr(serdes, name))
        self.tx_idle    = Signal()
        self.tx_pattern = Signal(len(serdes.tx_pattern))
        if hasattr(serdes, "tx_lfps"):
            self.tx_lfps = Signal()

class _Both(LiteXModule):
    """Lane events combination: a (pulse) event of each lane is held for window cycles, the output is
    set while both are held (the lanes being skewed by a few words)."""
    def __init__(self, a, b, window=64):
        self.o = Signal()

        # # #

        held = []
        for e in [a, b]:
            count = Signal(max=window + 1)
            self.sync += If(e, count.eq(window)).Elif(count != 0, count.eq(count - 1))
            held.append(e | (count != 0))
        self.comb += self.o.eq(held[0] & held[1])

# USB3 X2 PHY --------------------------------------------------------------------------------------

@ResetInserter()
class USB3X2PHY(LiteXModule):
    """Gen 1x2 PHY: 2x USB3PIPE + USB3LTSSM (ssp) + SSPPolling + lanes striping/deskew.

    Link interface: as USB3PHY (status, Link-Layer control) with 64-bit/8-bit sink/source link words
    (Symbol 0 on data[0:8]). x2 is the negotiated Gen 1x2 operation (PHY Capability LBPM).
    """
    def __init__(self, serdes, sys_clk_freq, role="device", timer_scale=1, x2_capable=True,
        deskew_depth=8):
        assert role in ["device", "host"]
        assert len(serdes) == 2

        self.sink   = sink   = stream.Endpoint(link_layout())
        self.source = source = stream.Endpoint(link_layout())

        # # #

        # PIPEs / LTSSM ----------------------------------------------------------------------------
        lane0_serdes = _LFPSSerDes(serdes[0])
        self.pipes = pipes = [
            USB3PIPE(s,
                sys_clk_freq         = sys_clk_freq,
                with_endianness_swap = False,
                timer_scale          = timer_scale,
                lfsr_seed            = seed,
                x2_lane              = True)
            for s, seed in [(lane0_serdes, 0xffff), (serdes[1], LFSR_SEED_LANE1)]]
        self.submodules += pipes
        self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale, ssp=True)
        self.comb += ltssm.ds_port.eq(role == "host")

        # Lane 0 (Configuration Lane): LTSSM <-> PIPE, TS/Idle status combined with Lane 1.
        combined = {id(ltssm.ts_rx_ts1), id(ltssm.ts_rx_ts1_inv), id(ltssm.ts_rx_ts2), id(ltssm.rx_idle8)}
        self.comb += [s for s in connect_ltssm_pipe(ltssm, pipes[0]) if id(s.l) not in combined]

        # Lane 1: same TX/RX controls, no LFPS (6.13.12): electrical idle while Lane 0 sends LFPS.
        for name in ["lfps_tx_idle", "ts_rx_enable", "ts_tx_enable", "ts_tx_tseq", "ts_tx_ts1",
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
        self.both_ts1     = both_ts1     = _Both(pipes[0].ts_rx_ts1,     lane1_ts1)
        self.both_ts1_inv = both_ts1_inv = _Both(pipes[0].ts_rx_ts1_inv, lane1_ts1)
        self.both_ts2     = both_ts2     = _Both(pipes[0].ts_rx_ts2,     pipes[1].ts_rx_ts2)
        self.comb += [
            ltssm.ts_rx_ts1.eq(both_ts1.o),
            ltssm.ts_rx_ts1_inv.eq(both_ts1_inv.o),
            ltssm.ts_rx_ts2.eq(both_ts2.o),
            ltssm.rx_idle8.eq(pipes[0].idle_done & pipes[1].idle_done),
        ]

        # SuperSpeedPlus Polling (Lane 0) ----------------------------------------------------------
        self.ssp = ssp = SSPPolling(sys_clk_freq, timer_scale=timer_scale, x2_capable=x2_capable)
        self.x2  = Signal()
        self.comb += [
            ssp.start.eq(ltssm.ssp_polling),
            ssp.ds_port.eq(role == "host"),
            ssp.idle.eq(serdes[0].rx_idle),
            ltssm.ssp_done.eq(ssp.done),
            ltssm.ssp_fallback.eq(ssp.ss_fallback),
        ]
        self.sync += If(ssp.done, self.x2.eq(ssp.x2))
        # Lane 0 TX: SuperSpeedPlus Polling LFPS, else the PIPE (LFPS unit/electrical idle/data).
        with_tx_lfps = hasattr(serdes[0], "tx_lfps")
        self.comb += If(ssp.tx_active,
            serdes[0].tx_idle.eq(~ssp.tx_lfps),
            serdes[0].tx_pattern.eq(Mux(ssp.tx_lfps, ssp.tx_pattern, 0)),
            *([serdes[0].tx_lfps.eq(ssp.tx_lfps)] if with_tx_lfps else []),
        ).Else(
            serdes[0].tx_idle.eq(lane0_serdes.tx_idle),
            serdes[0].tx_pattern.eq(lane0_serdes.tx_pattern),
            *([serdes[0].tx_lfps.eq(lane0_serdes.tx_lfps)] if with_tx_lfps else []),
        )

        # SerDes RX Re-initialization (as USB3PHY) -------------------------------------------------
        for s in serdes:
            if hasattr(s, "rx_reset"):
                self.comb += s.rx_reset.eq(ltssm.exit_to_rx_detect)

        # TX: Striping -----------------------------------------------------------------------------
        self.striper = striper = LaneStriper()
        self.comb += sink.connect(striper.sink)
        for lane in range(2):
            self.comb += striper.sources[lane].connect(pipes[lane].sink)

        # RX: Deskew / Destriping ------------------------------------------------------------------
        # The deskew is re-acquired on the TS1/TS2 COM words (every 4 words) after each TSEQ/TS1 phase,
        # pairing the COM words received within 1 word on both lanes (6.4ns max lane-to-lane skew at
        # the receiver, Table 6-34: less than a word, the word alignment of each lane on COM can add
        # one word).
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

        # Status -----------------------------------------------------------------------------------
        self.ready = Signal()
        self.comb += self.ready.eq(serdes[0].ready & serdes[1].ready)
        for name in USB3PHY_STATUS:
            setattr(self, name, Signal(name=name))
            self.comb += getattr(self, name).eq(getattr(ltssm, name))
        for name, direction in USB3PHY_LINK_CTRL:
            setattr(self, name, getattr(ltssm, name))
        self.deskew_aligned = deskew.aligned
