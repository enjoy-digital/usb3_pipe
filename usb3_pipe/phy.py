#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY: USB3PIPE(s) + USB3LTSSM with the Link interface used by a USB3 Link Layer
(doc/manual): Gen 1x1 (5Gbps), Gen 2x1 (10Gbps on the Gen2PIPE with a gen2_capable SerDes) or
Gen 1x2 (two lanes, simulation only, USB 3.2 6.13, see doc/notes/gen2-x2-roadmap.md).
"""

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.fhdl.tools import insert_reset

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import COM, SHP, SDP, SLC, LTSSM_PIPE_INTERFACE, EndiannessSwap, AsyncFIFO
from usb3_pipe.common     import Buffer
from usb3_pipe.core       import USB3PIPE, Gen2PIPE, GEN2_PIPE_INTERFACE
from usb3_pipe.datapath   import GEN2_DATA_WIDTH
from usb3_pipe.ltssm      import USB3LTSSM, SSPPolling
from usb3_pipe.scrambling import LFSR_SEED_LANE0, LFSR_SEED_LANE1
from usb3_pipe.lanes      import LaneStriper, LaneDestriper, LaneDeskew, LaneSwap, lane_layout, link_layout
from usb3_pipe.datapath   import RXWordAligner
from usb3_pipe.framing    import gen2_link_layout, WordsToPairs, PairsToWords, WordsPacker
from usb3_pipe.framing    import PacketsBuffer, LogicalIdleFilter, WordsNarrower

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

# Link Words CDC (pipe_cd) --------------------------------------------------------------------------

class LinkWordsCDC(LiteXModule):
    """Gen 1 Link words (link_words=1), word pairs (2) or quads (4: Gen 2 word quads, Gen 1 words
    in pairs) between the Link interface (sys) and a Gen 1 PIPE in pipe_cd, through AsyncFIFOs
    (registered FIFO writes: timings).

    sink/source: Link interface streams (sys, Gen 1 words; USB3PHY selects the Gen2PIPE words around
    this module). pipe_rx_source -> pipe_rx_sink (pipe_cd): RX words to the RX FIFO, connected by
    USB3PHY (directly, or through the Gen 2 words selection when the Gen2PIPE RX words are in
    pipe_cd). rx_throttle: RX FIFO level above its threshold (the Link Layer then holds its Rx
    Header Buffer Credits, LCRD_x, so the partner stops after its outstanding Header Packets and
    their DPPs: bounded RX backlog with a Link Layer slower than the line rate).

    With link_words=2 and the Link Layer at half the PIPE frequency, the RX Logical Idle words
    between the packets are dropped and the RX words packed in full pairs (the Link Layer consumes
    one word per cycle outside of the DPP data), the TX packets (last: end of a Link Command, Header
    Packet or DPH + DPP, the Link Layer sends single words at the PIPE words rate/2 outside of the
    DPP data) are stored and forwarded (PacketsBuffer): no Logical Idle inside the packets. The TX
    words received by the PIPE while not ready (tx_ready, outside of U0) are dropped: no stale words
    at the next U0 entry (the Link Layer only sends words in U0, the PIPE tx_ready
    resynchronization is shorter than the FIFO latency). The RX path has no back-pressure (received
    words at the line rate: the Link Layer reads its words at least at the received words rate).

    gen2: Gen 2 capable PHY: deeper RX FIFO (the Link Layer, 64-bit at half the PIPE frequency,
    reads at about half the received rate and up to 8 Header Packets, Type 1/Type 2 credits, can be
    received after the throttling), throttling from a quarter. rx_depth: RX FIFO depth override
    (ex: devices without Bulk OUT endpoints, distributed RAM on small FPGAs).
    """
    def __init__(self, pipe, pipe_cd, link_words=2, gen2=False, fifo_depth=32, rx_depth=None):
        # layout: Gen 1 words/TX words (pairs with link_words=4), ext_layout: Link Layer words.
        layout     = [("data", 32), ("ctrl", 4)] if link_words == 1 else gen2_link_layout()
        ext_layout = layout if link_words != 4 else gen2_link_layout(4)
        self.pipe_rx_sink   = pipe_rx_sink   = stream.Endpoint(layout) # i (pipe_cd).
        self.pipe_rx_source = pipe_rx_source = stream.Endpoint(layout) # o (pipe_cd).
        self.rx_throttle    = Signal()                                   # o

        # # #

        # PIPE side (pipe_cd) ----------------------------------------------------------------------
        if link_words == 1:
            pipe_sink = stream.Endpoint(layout)
            self.comb += [
                pipe_sink.connect(pipe.sink, omit={"ready"}),
                pipe_sink.ready.eq(pipe.sink.ready | ~pipe.tx_ready),
                pipe.source.connect(pipe_rx_source, omit={"ready"}),
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
                tx_packets.drop.eq(~pipe.tx_ready),
                tx_packets.source.connect(to_words.sink),
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
                to_pairs.source.connect(pipe_rx_source),
            ]
            # Skid buffer reset registered (fanout).
            sync  = getattr(self.sync, pipe_cd)
            sync += tx_skid.reset.eq(~pipe.tx_ready)
            pipe_sink = tx_packets.sink

        # TX: sys -> pipe_cd (registered ready: the Link Layer TX path from a register) -------------
        self.tx_cdc_buffer = tx_buffer = Buffer(layout, pipe_ready=True)
        self.tx_cdc        = tx_cdc    = ClockDomainsRenamer({"write": "sys", "read": pipe_cd})(
            AsyncFIFO(layout, fifo_depth))
        tx_sink = tx_buffer.sink
        if link_words == 4:
            # Gen 1 TX words: quads -> pairs.
            self.tx_narrower = tx_narrower = WordsNarrower(4, 2)
            self.comb += tx_narrower.source.connect(tx_buffer.sink)
            tx_sink = tx_narrower.sink
        self.sink = tx_sink # i (sys, ext_layout).
        self.comb += [
            tx_buffer.source.connect(tx_cdc.sink),
            tx_cdc.source.connect(pipe_sink),
        ]

        # RX: pipe_cd -> sys -----------------------------------------------------------------------
        self.rx_cdc_buffer = rx_buffer = ClockDomainsRenamer(pipe_cd)(Buffer(layout,
            pipe_ready=True))
        # Deep (block RAM, read side at the Link Layer frequency): with word pairs at half the PIPE
        # frequency, the Link Layer consumes the Header Packets/DPP tails one word per cycle and
        # back-to-back DPH+DPPs (Bulk OUT bursts) accumulate words (~11 per 1024-byte DPP). Gen 2:
        # ~64 entries per 1024-byte DPP, deeper FIFO (see gen2).
        if rx_depth is None:
            rx_depth = fifo_depth if link_words == 1 else (1024 if gen2 else 256)
        self.rx_cdc = rx_cdc = ClockDomainsRenamer({"write": pipe_cd, "read": "sys"})(
            stream.AsyncFIFO(layout, rx_depth, buffered=True))
        self.source = source = rx_cdc.source # o (sys, Gen 1 words).
        self.comb += [
            pipe_rx_sink.connect(rx_buffer.sink, omit={"ready"}),
            pipe_rx_sink.ready.eq(1),
            rx_buffer.source.connect(rx_cdc.sink, omit={"ready"}),
            rx_buffer.source.ready.eq(1), # Dropped when the FIFO is full (Link Layer not reading).
        ]

        # RX throttling (sys): rx_cdc level >= half (a quarter with gen2). Level from a
        # resynchronized (Gray) count of the written entries and the count of the read entries.
        if link_words >= 2:
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

# USB3 PHY -----------------------------------------------------------------------------------------

# LTSSM status outputs exposed by USB3PHY.
USB3PHY_STATUS = [
    "u0", "recovery", "rx_ready", "tx_ready", "u1", "u2", "u3", "compliance", "loopback",
    "hot_reset", "ess_inactive", "polling_idle", "rx_keep",
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
    ("compliance_enable", "i"), ("directed_loopback", "i"), ("loopback_capable", "i"),
]

# Link interface attributes always defined by USB3PHY (constants/ints when the feature is absent),
# with the sink/source streams and the status/control signals above: (name, description).
USB3PHY_LINK_INTERFACE = [
    ("link_words",    "Link words per cycle: 1 (32-bit), 2 (64-bit pairs) or 4 (128-bit quads)."),
    ("with_u1u2",     "U1/U2 Link Power Management supported (LGO_U1/LGO_U2 accepted)."),
    ("ssp_capable",   "Gen 2 operation possible (SuperSpeedPlus Link Layer rules on gen2_active)."),
    ("gen2_active",   "Gen 2 (10Gbps) operation (0 if not ssp_capable)."),
    ("source_lc",     "LGOOD_n/LCRD_x Link Commands words not behind the RX words, enabled by source_lc_enable (None without)."),
    ("rx_throttle",   "RX words buffered (pipe_cd): Rx Header Buffer credits held (0 without)."),
    ("rx_t2_credits", "Type 2 credits the partner may hold: 4 (no limit) or a Signal (runtime)."),
]

# PIPE -> LTSSM status pulses (LTSSM_PIPE_INTERFACE inputs, others are levels).
LTSSM_PIPE_PULSES = [
    "lfps_rx_polling", "lfps_rx_ping", "ts_tx_done", "ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2",
]

# sys only (Gen2PIPE tx/rx and pipe_cd clock domains: resynchronized resets, see do_finalize and
# get_fragment).
@ResetInserter(["sys"])
class USB3PHY(LiteXModule):
    """USB3 PHY: USB3PIPE(s) + USB3LTSSM.

    Combines the PIPE (LFPS/TS/scrambling/datapath) with the LTSSM to provide a link-trained
    data/ctrl (K-char) symbol stream in the sys clock domain, ready for a USB3 Link Layer. The
    SerDes is not a submodule and must be added by the caller.

    - lanes=1 (Gen 1x1): serdes is a SerDes, 32-bit data/4-bit ctrl link words.
    - lanes=2 (Gen 1x2, 6.13): serdes is a list of 2 SerDes (Lane 0: Configuration Lane), 64-bit
      data/8-bit ctrl link words (Symbol 0 on data[0:8], no endianness swap). x2 is the negotiated
      Gen 1x2 operation (PHY Capability LBPM, x2_capable: advertised capability), sink/source carry
      the link words with x2. Gen 1x1 negotiated (x2 = 0: Gen 1x1 SuperSpeedPlus partner, or
      SuperSpeed partner after the SuperSpeed fallback): single lane on Lane 0 (Lane 1 in electrical
      idle), 32-bit/4-bit link words on sink_x1/source_x1 (framing aligned, as with lanes=1).
      Lanes order (USB Type-C plug orientation): lane_swap (1: Configuration Lane on serdes[1]),
      detected from the partner's Polling.LFPS with lane_swap_detect (default: Device role).
    - gen2_capable: Gen 2 (10Gbps) capability advertised in SuperSpeedPlus Polling (SCD1/SCD2, PHY
      Capability LBPM, also used with lanes=1), gen2 is the negotiated Gen 2 operation. With a
      gen2_capable SerDes (raw words, 10Gbps rate), the training continues at 10Gbps on the
      Gen2PIPE (add_gen2: Gen 2 TSEQ/TS1/TS2, SDS/Logical Idle, gen2_active), else at Gen 1 speed.
      The Link Layer switches to SuperSpeedPlus operation on gen2_active (usb3_core.link).
    - with_u1u2=False (Gen 1x1): no U1/U2 Link Power Management (less logic, see USB3LTSSM).
    - pipe_cd (Gen 1x1): USB3PIPE (and SerDes, renamed by the caller) in the pipe_cd clock domain at
      pipe_clk_freq (>= 125MHz), LTSSM and Link interface in sys (LinkWordsCDC): ex. sys at 62.5MHz
      with link_words=2 on FPGAs where the LTSSM/Link Layer can't reach 125MHz. pipe_rx_depth: RX
      CDC FIFO depth override (default: 256 word pairs with link_words=2, for Bulk OUT bursts).
      link_words=4 (pipe_cd, Gen 2 capable SerDes with raw words wider than 40 bits): Link Layer
      word quads (128-bit, Gen 2 DPP data up to 16 bytes per cycle), Gen 1 words in pairs. The
      PHY reset is resynchronized to pipe_cd and resets the pipe_cd logic of the PHY (PIPE,
      Gen2PIPE, CDC) as the sys one.

    Link interface (what a USB3 Link Layer needs from the PHY, the Link Layer does not have to know
    the PHY internals):
    - `sink`/`source`: data/ctrl symbol streams (Logical Idle sent when `sink` has no data).
    - Status (see USB3PHY_STATUS): mirrored from the LTSSM.
    - Link-Layer control (see USB3PHY_LINK_CTRL): PM handshake (LGO_Ux/LAU/LXU/LPMA decoded or to
      send by the Link Layer), directed Recovery/Hot Reset/Warm Reset and Ux exit requests,
      Compliance (host) and Loopback master controls. The signals are the LTSSM ones (same Signal
      objects); undriven inputs keep their reset values.
    - Capabilities and flow control (see USB3PHY_LINK_INTERFACE), always defined: `link_words`,
      `with_u1u2`, `ssp_capable`/`gen2_active` (Gen 2 operation), `source_lc` (fast Link Commands
      path with `source_lc_enable`, None without), `rx_throttle` (RX words buffered with pipe_cd:
      Rx Header Buffer credits held), `rx_t2_credits` (Type 2 credits the partner may hold).
    """
    def __init__(self, serdes, sys_clk_freq, role="device", with_endianness_swap=True,
        timer_scale=1, lanes=1, x2_capable=True, gen2_capable=False, deskew_depth=8, link_words=1,
        lane_swap_detect=None, pipe_cd=None, pipe_clk_freq=None, pipe_rx_depth=None, with_u1u2=True):
        assert role in ["device", "host"], f"role: device or host ({role})."
        assert lanes in [1, 2], f"lanes: 1 or 2 ({lanes})."
        serdes = [serdes] if lanes == 1 else serdes
        assert len(serdes) == lanes, "serdes: one SerDes per lane."
        assert (lanes == 1) or (not with_endianness_swap), "lanes=2: no endianness swap."
        assert link_words in [1, 2, 4], f"link_words: 1, 2 or 4 ({link_words})."
        assert (lanes == 1) or (link_words == 1), "lanes=2: link_words=1."
        assert (pipe_cd is None) or (lanes == 1), "pipe_cd: lanes=1."
        assert (pipe_cd is None) or (pipe_clk_freq is not None), "pipe_cd: pipe_clk_freq required."
        assert (pipe_cd is None) or (not gen2_capable) or (link_words in [2, 4]), \
            "pipe_cd with gen2_capable: link_words=2 or 4."
        assert (link_words != 4) or ((pipe_cd is not None) and gen2_capable), \
            "link_words=4: pipe_cd and gen2_capable."
        # Link Layer words (1: 32-bit), pairs (2: 64-bit) or quads (4: 128-bit).
        self.link_words = link_words
        self.pipe_cd    = pipe_cd
        self.with_u1u2  = with_u1u2 # U1/U2 Link Power Management (USB3LTSSM with_u1u2).
        assert with_u1u2 or (lanes == 1), "lanes=2: with_u1u2."
        # Link interface defaults (USB3PHY_LINK_INTERFACE), overridden by add_gen2 (ssp_capable,
        # gen2_active) and add_pipe_cd_datapath (rx_throttle, rx_t2_credits).
        self.ssp_capable   = False
        self.gen2_active   = Constant(0, 1)
        self.source_lc     = None
        self.rx_throttle   = Constant(0, 1)
        self.rx_t2_credits = 4 # Type 2 credits (SuperSpeedPlus) the partner may hold, 4: no limit.

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
                # PHY reset resynchronized to pipe_cd (do_finalize), inserted on the pipe_cd
                # statements of the PHY (get_fragment: PIPE, Gen2PIPE, LinkWordsCDC).
                self.pipe_reset = Signal()
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
            if lane_swap_detect is None:
                lane_swap_detect = (role == "device")
            self.add_lanes(serdes, sys_clk_freq, role, timer_scale, x2_capable, gen2_capable,
                lane_swap_detect)
            ltssm = self.ltssm

        # SerDes RX Re-initialization --------------------------------------------------------------
        # The transceiver RX initialization (PMA/CDR/offset calibration) does not always end in a
        # good state: on hardware (Artix7 GTP, xHCI host with SSC) ~1/12 RX resets ended with a
        # receiver unable to train or with a high error rate, recovered by another RX reset. Re-run
        # it on each training failure (and Warm Reset), when the LTSSM exits to Rx.Detect, or when
        # requested (serdes_rx_reset, ex: debug).
        self.serdes_rx_reset = Signal() # i
        rx_errors_reinit = [None]*len(serdes)
        if lanes == 2:
            rx_errors_reinit = self.add_rx_errors_monitors(serdes, sys_clk_freq, timer_scale)
        for s, reinit in zip(serdes, rx_errors_reinit):
            rx_reset = ltssm.exit_to_rx_detect | self.serdes_rx_reset
            if reinit is not None:
                rx_reset = rx_reset | reinit # Gen 1x2: RX re-initialization on continuous errors.
            if pipe_cd is None:
                self.comb += s.rx_reset.eq(rx_reset)
            else:
                self.specials += MultiReg(ltssm.exit_to_rx_detect | self.serdes_rx_reset,
                    s.rx_reset, pipe_cd)

        # Endpoints --------------------------------------------------------------------------------
        if lanes == 1:
            if pipe_cd is not None:
                self.add_pipe_cd_datapath(pipe, pipe_cd, rx_depth=pipe_rx_depth)
            elif self.ssp_capable:
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

    def add_rx_errors_monitors(self, serdes, sys_clk_freq, timer_scale, window=1024, holdoff=100e-6,
        reset_cycles=16):
        """RX errors monitors (Gen 1x2, one per physical SerDes): a transceiver RX initialization
        can end in a bad state (continuous decode errors, ex: Artix7 GTP RX elastic buffer or CDR)
        and the re-initialization on the exits to Rx.Detect may run while the partner is in
        electrical idle/LFPS and end bad again. On hardware (two Acorn Minis) a receiver stayed with
        decode errors on ~75% of the cycles until a RX reset during the partner's transmission.

        A lane decoding errors on more than half of the cycles of a window (window cycles, RX not
        idle, LTSSM in a state with continuous symbols: Polling.RxEQ to U0, Recovery) is
        re-initialized (rx_reset for reset_cycles), then not monitored for holdoff (CDR lock).
        rx_errors_reinit: re-initialization pulses (per lane, debug counters). Returns the per lane
        rx_reset requests."""
        ltssm = self.ltssm
        self.rx_errors_reinit = Signal(len(serdes)) # o: Re-initialization pulses.
        symbols = Signal()
        self.comb += symbols.eq(Reduce("OR", [ltssm.fsm.ongoing(state) for state in [
            "Polling.RxEQ", "Polling.Active", "Polling.Configuration", "Polling.Idle", "U0",
            "Recovery.Active", "Recovery.Configuration", "Recovery.Idle"]]))
        holdoff_cycles = max(1, int(holdoff*sys_clk_freq)//timer_scale)
        requests = []
        for lane, s in enumerate(serdes):
            count   = Signal(max=window + 1)
            errors  = Signal(max=window + 1)
            hold    = Signal(max=holdoff_cycles + 1)
            reset   = Signal(max=reset_cycles + 1)
            request = Signal()
            self.sync += [
                self.rx_errors_reinit[lane].eq(0),
                If(reset != 0,
                    reset.eq(reset - 1),
                    hold.eq(holdoff_cycles),
                ).Elif(hold != 0,
                    hold.eq(hold - 1),
                    count.eq(0),
                    errors.eq(0),
                ).Elif(~symbols | s.rx_idle,
                    count.eq(0),
                    errors.eq(0),
                ).Elif(count == window,
                    count.eq(0),
                    errors.eq(0),
                    If(errors > window//2,
                        reset.eq(reset_cycles),
                        self.rx_errors_reinit[lane].eq(1),
                    )
                ).Else(
                    count.eq(count + 1),
                    errors.eq(errors + s.rx_errors),
                )
            ]
            self.comb += request.eq(reset != 0)
            requests.append(request)
        return requests

    def add_lanes(self, serdes, sys_clk_freq, role, timer_scale, x2_capable, gen2_capable=False,
        lane_swap_detect=False):
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

        Gen 1x1 negotiated (x2 = 0): Lane 0 only, TS1/TS2/Idle handshake status from Lane 0, Lane 1
        in electrical idle (single lane fallback).

        Not implemented: SuperSpeedPlus Link Layer (Type 1/Type 2 traffic, 4-bit HSEQ...), framing
        alignment of the 64-bit link words.
        """
        # Lane Swap --------------------------------------------------------------------------------
        # Logical lanes on the physical SerDes (USB Type-C plug orientation): with
        # lane_swap_detect, the Configuration Lane is the physical lane receiving the partner's
        # Polling.LFPS (lane_swap = 1: Lane 0 on serdes[1]), else lane_swap input (default 0).
        self.lanes_phys_serdes = serdes # Physical SerDes (RX status: comma realignments).
        self.lane_swap_unit = lane_swap_unit = LaneSwap(serdes, detect=lane_swap_detect)
        self.lane_swap = lane_swap_unit.swap
        serdes = lane_swap_unit.lanes
        self.lanes_serdes = serdes # Logical lanes SerDes (Lane 0: Configuration Lane).
        # Lane electrical idle filter in U0 at Gen 1x2 (lanes datapath): 10us.
        self.lanes_idle_cycles = max(1, int(10e-6*sys_clk_freq)//timer_scale)

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
        self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale, ssp=True,
            lanes=2)
        self.comb += ltssm.ds_port.eq(role == "host")
        # Configuration Lane detection in the SuperSpeedPlus Polling, and in Compliance: a Device
        # timed out to Compliance before receiving the partner's Polling.LFPS (partner not started
        # yet) only leaves it on a Warm Reset LFPS, sent by the Host on its Configuration Lane: the
        # lanes swap follows it, the Warm Reset is then detected on (logical) Lane 0.
        self.comb += lane_swap_unit.enable.eq(ltssm.ssp_polling | ltssm.compliance)

        # Lane 0 (Configuration Lane): LTSSM <-> PIPE, TS/Idle status combined with Lane 1 below.
        self.comb += connect_ltssm_pipe(ltssm, pipes[0], with_rx_ts_status=False)

        # SuperSpeedPlus Polling (Lane 0): negotiated x2 --------------------------------------------
        self.add_ssp_polling(pipes[0], serdes[0], sys_clk_freq, role, timer_scale,
            x2_capable=x2_capable, gen2_capable=gen2_capable)
        x2 = self.x2
        self.comb += ltssm.x2.eq(x2) # x2 Link flow control timers (Table 7-8).

        # Lane 1: same TX/RX controls, no LFPS (6.13.12): electrical idle while Lane 0 sends LFPS.
        # Gen 1x1 negotiated (x2 = 0, single lane fallback): Lane 1 unused, in electrical idle.
        for name in ["lfps_tx_idle", "ts_tx_enable", "ts_tx_tseq", "ts_tx_ts1", "ts_tx_ts2",
            "ts_tx_ts2_reset", "ts_tx_ts2_loopback", "ts2_decode", "idle_handshake",
            "serdes_rx_align", "rx_ready", "tx_ready"]:
            self.comb += getattr(pipes[1], name).eq(getattr(ltssm, name) & x2)
        self.comb += If(~x2, pipes[1].lfps_tx_idle.eq(1))

        # Lane 1 polarity (6.13.7): inverted when TS1 are received inverted on Lane 1.
        lane1_polarity = Signal()
        self.comb += pipes[1].serdes_rx_polarity.eq(lane1_polarity)
        self.sync += If(pipes[1].ts_rx_ts1_inv & ~pipes[1].ts_rx_ts1,
            lane1_polarity.eq(~lane1_polarity)
        )

        # TS1/TS2 received on both lanes (6.13.6), Idle handshake on both lanes (Lane 0 only when
        # Gen 1x1 is negotiated).
        lane1_ts1 = pipes[1].ts_rx_ts1 | pipes[1].ts_rx_ts1_inv
        self.both_ts1     = both_ts1     = LanesEvent(pipes[0].ts_rx_ts1,     lane1_ts1)
        self.both_ts1_inv = both_ts1_inv = LanesEvent(pipes[0].ts_rx_ts1_inv, lane1_ts1)
        self.both_ts2     = both_ts2     = LanesEvent(pipes[0].ts_rx_ts2,     pipes[1].ts_rx_ts2)
        self.comb += [
            ltssm.ts_rx_ts1.eq(    Mux(x2, both_ts1.o,     pipes[0].ts_rx_ts1)),
            ltssm.ts_rx_ts1_inv.eq(Mux(x2, both_ts1_inv.o, pipes[0].ts_rx_ts1_inv)),
            ltssm.ts_rx_ts2.eq(    Mux(x2, both_ts2.o,     pipes[0].ts_rx_ts2)),
            ltssm.idle_handshake_done.eq(pipes[0].idle_handshake_done &
                (pipes[1].idle_handshake_done | ~x2)),
        ]

    def add_ssp_polling(self, pipe, serdes, sys_clk_freq, role, timer_scale, x2_capable,
        gen2_capable):
        """SuperSpeedPlus Polling (SCD1/SCD2, LBPM PHY Capability/Ready, SSPPolling,
        7.5.4.3-7.5.4.6) on the Configuration Lane in Polling.LFPS: x2/gen2 are the negotiated
        lanes/speed."""
        self.ssp  = ssp = SSPPolling(sys_clk_freq, timer_scale=timer_scale,
            x2_capable   = x2_capable,
            gen2_capable = gen2_capable)
        # Gen 1x2 advertised (lanes=2, runtime: 0 forces Gen 1x1 from the next SuperSpeedPlus
        # Polling, ex: tests of the Gen 1x1 operation with a partner x2 capable; x2_capable is the
        # maximum).
        if len(self.pipes) == 2:
            self.x2_enable = Signal(reset=1) # i
            self.comb += ssp.x2_capable.eq(int(x2_capable) & self.x2_enable)
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
        # Updated on each SuperSpeedPlus Polling, also on the SuperSpeed fallback (x2/gen2 = 0:
        # not kept from a previous partner).
        self.sync += If(ssp.done | ssp.ss_fallback,
            self.x2.eq(ssp.x2),
            self.gen2.eq(ssp.gen2),
        )

    @property
    def reset(self):
        """PHY reset (sys clock domain)."""
        return self.reset_sys

    def do_finalize(self):
        # Gen2PIPE tx/rx clock domains reset with the PHY reset (else, after a PHY reset, the sys
        # sides of the Gen 2 TX units/RX words buffers restart while their tx/rx sides keep their
        # state: units started before being written, Logical Idle Data Blocks inside the units).
        if hasattr(self, "gen2_pipe"):
            self.specials += [
                MultiReg(self.reset_sys, self.gen2_pipe.reset_tx, "tx"),
                MultiReg(self.reset_sys, self.gen2_pipe.reset_rx, "rx"),
            ]
        if self.pipe_cd is not None:
            self.specials += MultiReg(self.reset_sys, self.pipe_reset, self.pipe_cd)

    def get_fragment(self):
        f = LiteXModule.get_fragment(self)
        # The class ResetInserter only covers sys: with pipe_cd, the PIPE, the Gen2PIPE pipe_cd part,
        # the pipe_cd sides of the Link words CDC (AsyncFIFOs, buffers, RX throttle counter) and the
        # resynchronization registers kept their state across a PHY reset while their sys sides
        # restarted (stale FIFO entries replayed, RX throttle level offset): same reset insertion on
        # the pipe_cd statements, from the resynchronized reset.
        if self.pipe_cd is not None:
            f.sync[self.pipe_cd] = insert_reset(self.pipe_reset, f.sync[self.pipe_cd])
        return f

    def add_pairs_adapters(self, pipe):
        """Words pairs (link_words=2) <-> Gen 1 PIPE words: returns the (sink, source) pairs
        streams."""
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

    def add_pipe_cd_datapath(self, pipe, pipe_cd, rx_depth=None):
        """Link interface with the PIPE in pipe_cd: Gen 1 words through a LinkWordsCDC. With a
        Gen2PIPE, its words (Gen 2 packet framing) are selected while gen2_active: TX words to its
        framing (sys); RX words either in sys (framing at the Link Layer frequency, 32 bytes per
        cycle: selected after the Gen 1 words CDC, RX throttling from the Gen 2 words buffer level)
        or in pipe_cd (8 bytes per cycle: selected before the Gen 1 words CDC, with the gen2_active
        resynchronization, LGOOD_n/LCRD_x Link Commands through their own AsyncFIFO)."""
        gen2 = self.ssp_capable
        self.link_cdc = link_cdc = LinkWordsCDC(pipe, pipe_cd, self.link_words, gen2=gen2,
            rx_depth=rx_depth)
        if not gen2:
            self.comb += link_cdc.pipe_rx_source.connect(link_cdc.pipe_rx_sink)
            self.sink        = link_cdc.sink
            self.source      = link_cdc.source
            self.rx_throttle = link_cdc.rx_throttle
            return

        gen2_pipe   = self.gen2_pipe
        gen2_active = self.gen2_active
        layout      = gen2_link_layout(self.link_words)
        gen2_sink, gen2_source, gen2_lc = self.add_gen2_pipe_streams(layout)

        # TX: Link Layer words to the Gen2PIPE framing (sys) while gen2_active, else to the Gen 1
        # words CDC.
        self.sink = sink = stream.Endpoint(layout)
        self.comb += If(gen2_active,
            sink.connect(gen2_sink),
        ).Else(
            sink.connect(link_cdc.sink),
        )

        # LGOOD_n/LCRD_x Link Commands words (Gen 2, PHY words order), not behind the RX words, when
        # enabled by the Link Layer (source_lc_enable, else in source).
        self.source_lc        = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source_lc_enable = Signal()

        if gen2_pipe.link_rx_cd != "sys":
            # RX: Gen 2 words/Link Commands in sys, selected after the Gen 1 words CDC.
            self.comb += link_cdc.pipe_rx_source.connect(link_cdc.pipe_rx_sink)
            self.source = source = stream.Endpoint(layout)
            self.comb += [
                gen2_source.ready.eq(1),
                link_cdc.source.ready.eq(1),
                If(gen2_active,
                    gen2_source.connect(source, omit={"ready"}),
                    gen2_source.ready.eq(source.ready),
                ).Else(
                    link_cdc.source.connect(source),
                )
            ]
            self.comb += gen2_pipe.source_lc_enable.eq(self.source_lc_enable)
            self.add_gen2_source_lc(gen2_lc, source_lc)
            # RX throttling: the Gen 1 words CDC level, or at Gen 2 the words buffer level (entries
            # of up to 8 words: 512 bytes by default); Type 2 credits held by the partner
            # (rx_t2_credits, 4: no limit; limiting them bounds the DPPs buffered before a DPH but
            # an xHCI host rejected the Header Packets with it). Runtime-adjustable (bring-up).
            gen2_throttle = Signal()
            self.gen2_rx_throttle_level = Signal(8, reset=16)
            self.rx_t2_credits          = Signal(3, reset=4)
            self.rx_throttle            = Signal()
            self.sync += gen2_throttle.eq(gen2_pipe.rx_level >= self.gen2_rx_throttle_level)
            self.comb += [
                self.rx_throttle.eq(link_cdc.rx_throttle),
                If(gen2_active, self.rx_throttle.eq(gen2_throttle)),
            ]
        else:
            # RX: Gen 2 words in pipe_cd, selected before the Gen 1 words CDC (no back-pressure);
            # Link Commands through their own AsyncFIFO.
            self.comb += [
                gen2_source.ready.eq(1),
                link_cdc.pipe_rx_source.ready.eq(1),
                If(self.gen2_active_pipe,
                    gen2_source.connect(link_cdc.pipe_rx_sink, omit={"ready"}),
                ).Else(
                    link_cdc.pipe_rx_source.connect(link_cdc.pipe_rx_sink, omit={"ready"}),
                )
            ]
            self.source      = link_cdc.source
            self.rx_throttle = link_cdc.rx_throttle
            self.specials += MultiReg(self.source_lc_enable, gen2_pipe.source_lc_enable, pipe_cd)
            self.lc_cdc = lc_cdc = ClockDomainsRenamer({"write": pipe_cd, "read": "sys"})(
                stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True))
            self.comb += gen2_lc.connect(lc_cdc.sink)
            self.add_gen2_source_lc(lc_cdc.source, source_lc)

    def add_gen2_pipe_streams(self, layout):
        """Gen2PIPE Link Layer streams in the PHY words order: (sink, source, source_lc) Endpoints
        (EndiannessSwap: Gen2PIPE words Symbol 0 on data[0:8], PHY words Symbol 0 on data[24:32])."""
        gen2_pipe   = self.gen2_pipe
        gen2_sink   = stream.Endpoint(layout)
        gen2_source = stream.Endpoint(layout)
        gen2_lc     = stream.Endpoint([("data", 32), ("ctrl", 4)])
        if self.gen2_endianness_swap:
            self.submodules += EndiannessSwap(gen2_sink, gen2_pipe.sink, word_bytes=4)
            self.submodules += EndiannessSwap(gen2_pipe.source, gen2_source, word_bytes=4)
            self.submodules += EndiannessSwap(gen2_pipe.source_lc, gen2_lc)
        else:
            self.comb += [
                gen2_sink.connect(gen2_pipe.sink),
                gen2_pipe.source.connect(gen2_source),
                gen2_pipe.source_lc.connect(gen2_lc),
            ]
        return gen2_sink, gen2_source, gen2_lc

    def add_gen2_source_lc(self, lc, source_lc):
        """Gen 2 Link Commands stream (sys) on source_lc while gen2_active (consumed otherwise)."""
        self.comb += [
            lc.connect(source_lc, omit={"valid", "ready"}),
            source_lc.valid.eq(lc.valid & self.gen2_active),
            lc.ready.eq(source_lc.ready | ~self.gen2_active),
        ]

    def add_gen2_datapath(self, pipe):
        """Link Layer words: Gen2PIPE (Gen 2 packet framing) while gen2_active, else Gen 1 PIPE
        (words or word pairs: link_words)."""
        layout = [("data", 32), ("ctrl", 4)] if self.link_words == 1 else gen2_link_layout()
        self.sink   = sink   = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        if self.link_words == 1:
            pipe_sink, pipe_source = pipe.sink, pipe.source
        else:
            pipe_sink, pipe_source = self.add_pairs_adapters(pipe)
        gen2_sink, gen2_source, gen2_lc = self.add_gen2_pipe_streams(layout)
        # LGOOD_n/LCRD_x Link Commands words (Gen 2, PHY words order), not behind the RX words, when
        # enabled by the Link Layer (source_lc_enable, else in source).
        self.source_lc        = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source_lc_enable = Signal()
        self.comb += self.gen2_pipe.source_lc_enable.eq(self.source_lc_enable)
        self.add_gen2_source_lc(gen2_lc, source_lc)
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
        # tx/rx clock domains reset with the PHY (sys) reset, synchronized (do_finalize): the TX
        # units and RX words buffers have their write/read sides in different clock domains.
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
        self.ssp_capable = True
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
        # LTSSM <-> Gen 1 PIPE / Gen2PIPE. No comma alignment at Gen 2: the transceiver comma
        # aligner would slip bits on comma-like patterns of the raw 128b/132b words (block alignment
        # by the Gen 2 RX datapath).
        for name, width, direction in LTSSM_PIPE_INTERFACE:
            if name == "serdes_rx_align":
                self.comb += pipe.serdes_rx_align.eq(ltssm.serdes_rx_align & ~gen2_active)
            elif name not in GEN2_PIPE_INTERFACE:
                self.comb += connect_ltssm_pipe_signal(ltssm, pipe, name, direction)
            elif direction == "o":
                # Gen 2 RX words kept from Recovery.Idle (rx_keep, Link Layer: processed in U0).
                gen2_s = getattr(ltssm, name)
                if name == "rx_ready":
                    gen2_s = gen2_s | ltssm.rx_keep
                self.comb += [
                    getattr(pipe,      name).eq(getattr(ltssm, name) & ~gen2_active),
                    getattr(gen2_pipe, name).eq(gen2_s &  gen2_active),
                ]
            else:
                self.comb += getattr(ltssm, name).eq(
                    Mux(gen2_active, getattr(gen2_pipe, name), getattr(pipe, name)))

    def add_gen2_ltssm_cdc(self, ltssm, pipe, gen2_pipe, gen2_active, pipe_cd):
        """LTSSM (sys) <-> Gen 1 PIPE / Gen2PIPE (pipe_cd) wiring: GEN2_PIPE_INTERFACE controls
        gated by gen2_active then resynchronized to each PIPE, status resynchronized from each PIPE
        then selected by gen2_active (sys)."""
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
                # Gen 2 RX words kept from Recovery.Idle (rx_keep, Link Layer: processed in U0).
                gen2_s = (ltssm_s | ltssm.rx_keep) if name == "rx_ready" else ltssm_s
                self.ltssm_to_pipe(ltssm_s & ~gen2_active, getattr(pipe,      name), pipe_cd)
                self.ltssm_to_pipe(gen2_s  &  gen2_active, getattr(gen2_pipe, name), pipe_cd)
            else:
                gen1_s = Signal(width)
                gen2_s = Signal(width)
                self.pipe_to_ltssm(name, getattr(pipe,      name), gen1_s, pipe_cd)
                self.pipe_to_ltssm(name, getattr(gen2_pipe, name), gen2_s, pipe_cd)
                self.comb += ltssm_s.eq(Mux(gen2_active, gen2_s, gen1_s))

    def add_lanes_datapath(self, deskew_depth):
        """Gen 1x2 datapath: data striping (6.13.4): 64-bit/8-bit link words (Symbol 0 on
        data[0:8]), even Symbols on Lane 0, odd Symbols on Lane 1 (LaneStriper/LaneDestriper), lanes
        deskew on the TS1/TS2 COM words (LaneDeskew: first COM words after a gap, Table 6-34: 6.4ns
        max at the receiver), re-acquired on each TS1 phase (Polling.Active, Recovery.Active)."""
        pipes = self.pipes
        ltssm = self.ltssm
        x2    = self.x2
        self.sink      = sink      = stream.Endpoint(link_layout())
        self.source    = source    = stream.Endpoint(link_layout())
        self.sink_x1   = sink_x1   = stream.Endpoint(lane_layout())
        self.source_x1 = source_x1 = stream.Endpoint(lane_layout())
        tx_enable = Signal()
        self.comb += tx_enable.eq(ltssm.tx_ready | ltssm.idle_handshake)

        # TX: Striping -----------------------------------------------------------------------------
        # Logical Idle (7.2.3) striped as data (6.13.4): Idle words when the Link Layer has no data
        # (and during the Idle handshake), so that both lanes always transmit the same words in
        # lockstep (a lane TX held for a cycle, ex: full TX CDC FIFO, holds the other lane).
        self.striper = striper = LaneStriper()
        self.comb += If(sink.valid & ltssm.tx_ready,
            sink.connect(striper.sink),
        ).Else(
            striper.sink.valid.eq(tx_enable),
            striper.sink.data.eq(0),
            striper.sink.ctrl.eq(0),
        )

        # TX: Gen 1x1 (Lane 0) ---------------------------------------------------------------------
        x1_words = stream.Endpoint(lane_layout())
        self.comb += If(sink_x1.valid & ltssm.tx_ready,
            sink_x1.connect(x1_words),
        ).Else(
            x1_words.valid.eq(tx_enable),
            x1_words.data.eq(0),
            x1_words.ctrl.eq(0),
        )
        self.comb += If(x2,
            striper.sources[0].connect(pipes[0].sink),
            striper.sources[1].connect(pipes[1].sink),
        ).Else(
            x1_words.connect(pipes[0].sink),
        )

        # RX: Deskew / Destriping ------------------------------------------------------------------
        # The deskew is acquired on the TS1/TS2 COM words: first COM word of each lane after a gap
        # without COM words on both lanes (TSEQ in Polling.RxEQ, Logical Idle before Recovery), the
        # same TS1 Ordered Set on both lanes whatever the lanes latencies (6.4ns max lane-to-lane skew
        # at the receiver, Table 6-34, + per lane word alignment/SKP removal/RX buffers latencies).
        self.deskew    = deskew    = LaneDeskew(COM.value*0x01010101, 0b1111, depth=deskew_depth)
        self.destriper = destriper = LaneDestriper()
        self.comb += pipes[1].source.connect(deskew.sinks[1])
        for lane in range(2):
            self.comb += deskew.sources[lane].connect(destriper.sinks[lane])
        self.comb += [
            destriper.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(destriper.source.valid & ltssm.rx_ready & x2),
            destriper.source.ready.eq(source.ready | ~ltssm.rx_ready | ~x2),
        ]
        self.deskew_aligned = deskew.aligned # o
        # Deskew lost at Gen 1x2 (ex: a lane re-aligned by its RX polarity change or SKP removal
        # during the TS1/TS2 Ordered Sets): Recovery, the deskew is re-acquired on its TS1 (after a
        # gap: Logical Idle in U0). Without it, U0 without data on the x2 datapath until a Recovery.
        # A lane in electrical idle for 10us (ex: one cable/lane unplugged, the other lane still in
        # U0: the lane re-aligned on its return, the deskew is no longer valid and nothing else
        # detects it with the PRBS stand-ins) is also a deskew lost: Recovery (Rx.Detect if the
        # lane does not come back, tRecoveryActiveTimeout).
        lanes_idle = Signal()
        idle_count = Signal(max=self.lanes_idle_cycles + 1)
        self.sync += If(~Reduce("OR", [s.rx_idle for s in self.lanes_serdes]),
            idle_count.eq(0),
        ).Elif(idle_count != self.lanes_idle_cycles,
            idle_count.eq(idle_count + 1),
        )
        self.comb += lanes_idle.eq(idle_count == self.lanes_idle_cycles)
        # A lane word alignment changed in U0 (transceiver comma realignment or RX
        # re-initialization: the lane words are re-aligned on COM only, there is none in U0) also
        # invalidates the deskew: pairing cleared, Recovery (the lanes words re-aligned and
        # deskewed on the TS1). On hardware, a lane re-aligned in U0 left the link in U0 with every
        # x2 word corrupted (two Acorn Minis, about half of the bitstream loads).
        lanes_realign = Signal()
        realign = [s.rx_realign for s in self.lanes_phys_serdes if hasattr(s, "rx_realign")]
        if hasattr(self, "rx_errors_reinit"):
            realign += [self.rx_errors_reinit != 0]
        self.comb += lanes_realign.eq(Reduce("OR", realign) if realign else 0)
        self.lanes_deskew_lost      = Signal()        # o: Deskew lost trigger (x2, not deskewed).
        self.lanes_idle             = Signal()        # o: Lane idle trigger (x2, lane idle for 10us).
        self.lanes_realign          = Signal()        # o: Lane realigned in U0 (x2).
        self.comb += [
            self.lanes_deskew_lost.eq(x2 & ~deskew.aligned),
            self.lanes_idle.eq(x2 & lanes_idle),
            self.lanes_realign.eq(x2 & ltssm.u0 & lanes_realign),
            # Recovery on a deskew lost once the deskew search is armed (gap: Logical Idle received
            # in U0), the deskew re-acquired on the Recovery TS1. Not on the U0 entry: a partner still
            # in Recovery.Idle (idle handshake) would receive our TS1 instead of Idle Symbols and stay
            # there until tRecoveryIdleTimeout (2ms, eSS.Inactive), and a Recovery without a gap does
            # not re-arm the deskew (U0 entries/Recoveries loop: two Acorn Minis).
            ltssm.lanes_deskew_lost.eq((self.lanes_deskew_lost & deskew.armed) | self.lanes_idle),
            deskew.clear.eq(lanes_idle | self.lanes_realign),
        ]

        # RX: Gen 1x1 (Lane 0) ---------------------------------------------------------------------
        # Framing alignment of the Lane 0 words (as USB3PIPE in Gen 1x1, words shifted by the SKP
        # Ordered Sets removal), output gated by rx_ready.
        self.framing_aligner = framing_aligner = RXWordAligner(markers=[SHP, SDP, SLC])
        self.comb += [
            If(x2,
                pipes[0].source.connect(deskew.sinks[0]),
            ).Else(
                pipes[0].source.connect(framing_aligner.sink),
            ),
            framing_aligner.source.connect(source_x1, omit={"valid", "ready"}),
            source_x1.valid.eq(framing_aligner.source.valid & ltssm.rx_ready & ~x2),
            framing_aligner.source.ready.eq(source_x1.ready | ~ltssm.rx_ready | x2),
        ]
