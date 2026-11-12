#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Dual-lane (Gen 1x2) building blocks (USB 3.2 6.13), see doc/notes/gen2-x2-roadmap.md.

- LaneStriper   : 64-bit link words (8 symbols, symbol 0 on data[0:8]) striped on two 32-bit lanes,
                  aligned to Lane 0: even symbols on Lane 0, odd symbols on Lane 1 (6.13.4). A word
                  is given to both lanes in the same cycle (lanes ready independent of valid).
- LaneDestriper : two 32-bit lanes to 64-bit link words.
- LaneDeskew    : aligns two lanes on a marker word (ex: Ordered Set duplicated on both lanes,
                  6.13.6, first marker after a gap on both lanes), the lanes are then released in
                  lockstep (the two sources must be consumed together, ex: by a LaneDestriper).
- LaneSwap      : logical lanes on two physical SerDes (USB Type-C plug orientation), Configuration
                  Lane detection from the partner's Polling.LFPS.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

# Layouts ------------------------------------------------------------------------------------------

def lane_layout():
    return [("data", 32), ("ctrl", 4)]

def link_layout():
    return [("data", 64), ("ctrl", 8)]

# Lane Striper -------------------------------------------------------------------------------------

class LaneStriper(LiteXModule):
    """TX data striping (6.13.4): sink (64-bit link words, link_layout) -> sources[0]/sources[1]
    (32-bit lane words, lane_layout): even symbols on Lane 0, odd symbols on Lane 1, both lanes
    accepting the words in lockstep (sink.ready = both sources ready)."""
    def __init__(self):
        self.sink    = sink    = stream.Endpoint(link_layout())
        self.sources = sources = [stream.Endpoint(lane_layout()) for _ in range(2)]

        # # #

        # A word is given to both lanes or to none: presented only when both lanes are ready (the
        # lanes ready must not depend on valid).
        self.comb += sink.ready.eq(sources[0].ready & sources[1].ready)
        for lane, source in enumerate(sources):
            self.comb += [
                source.valid.eq(sink.valid & sink.ready),
                source.first.eq(sink.first), # Packets boundaries (TX SKP Ordered Sets between packets).
                source.last.eq(sink.last),
                source.data.eq(Cat(*[sink.data[8*(2*i + lane):8*(2*i + lane + 1)]
                    for i in range(4)])),
                source.ctrl.eq(Cat(*[sink.ctrl[2*i + lane] for i in range(4)])),
            ]

# Lane Destriper -----------------------------------------------------------------------------------

class LaneDestriper(LiteXModule):
    """RX data destriping (6.13.4): sinks[0]/sinks[1] (32-bit lane words, deskewed: a word is
    consumed on both lanes together) -> source (64-bit link words, link_layout, Lane 0 symbols on
    the even positions)."""
    def __init__(self):
        self.sinks  = sinks  = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.source = source = stream.Endpoint(link_layout())

        # # #

        self.comb += [
            source.valid.eq(sinks[0].valid & sinks[1].valid),
            sinks[0].ready.eq(source.valid & source.ready),
            sinks[1].ready.eq(source.valid & source.ready),
        ]
        for i in range(4):
            for lane in range(2):
                self.comb += [
                    source.data[8*(2*i + lane):8*(2*i + lane + 1)].eq(sinks[lane].data[8*i:8*(i+1)]),
                    source.ctrl[2*i + lane].eq(sinks[lane].ctrl[i]),
                ]

# Lane Deskew --------------------------------------------------------------------------------------

class LaneDeskew(LiteXModule):
    """Two lanes deskew (up to depth words of skew) on a marker word (ex: COM words of the TS1/TS2
    Ordered Sets, duplicated on both lanes, 6.13.6).

    The markers are paired independently of the lanes latencies (per lane word alignment, SKP removal
    and RX buffers fill levels: up to a few words, a periodic marker would be ambiguous): the search
    is armed after a gap of gap words without marker on both lanes (ex: TSEQ or Logical Idle before
    the TS1 Ordered Sets), then the first marker received on each lane are the same marker. The words
    before it are dropped, then both lanes are released together. Markers at the head of a single lane
    twice without paired markers in between (lanes no longer paired: ex: word alignment of a lane
    changed after the pairing, a mismatch at each Ordered Set) restart the search (armed on the next
    gap), as clear (ex: a lane in electrical idle). A single mismatch is tolerated: the last markers of
    an Ordered Sets stream one word apart on the lanes (ex: a decode error substituted at the end of
    the TS2 Ordered Sets on one lane, Logical Idle after them: two Acorn Minis), the lanes still
    paired.

    The sinks are never held (continuous lanes streams, a held lane would overflow the SerDes RX
    buffers): words received with a full FIFO are dropped.
    """
    def __init__(self, marker_data, marker_ctrl, depth=8, gap=16):
        self.sinks   = sinks   = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.sources = sources = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.aligned = aligned = Signal() # o
        self.armed   = armed   = Signal() # o: Search armed (gap seen, pairing on the next markers).
        self.clear   = Signal()           # i: Pairing cleared (search re-armed on the next gap).

        # # #

        self.fifos = fifos = [stream.SyncFIFO(lane_layout(), depth) for _ in range(2)]
        self.submodules += fifos
        heads = []
        gaps  = []
        for lane in range(2):
            head      = Signal()
            gap_count = Signal(max=gap + 1)
            marker_in = Signal()
            self.comb += [
                sinks[lane].connect(fifos[lane].sink, omit={"ready"}),
                sinks[lane].ready.eq(1),
                marker_in.eq((sinks[lane].data == marker_data) & (sinks[lane].ctrl == marker_ctrl)),
                head.eq(fifos[lane].source.valid &
                    (fifos[lane].source.data == marker_data) &
                    (fifos[lane].source.ctrl == marker_ctrl)),
            ]
            # Words without marker received on the lane (saturated to gap).
            self.sync += If(sinks[lane].valid,
                If(marker_in,
                    gap_count.eq(0)
                ).Elif(gap_count != gap,
                    gap_count.eq(gap_count + 1)
                )
            )
            heads.append(head)
            gaps.append(gap_count == gap)
        self.heads = heads

        # Search armed after a gap on both lanes (no marker in the FIFOs, dropped while searching),
        # first markers paired; aligned until the markers are no longer paired (second mismatch
        # without paired markers in between).
        both     = Signal()
        mismatch = Signal()
        self.comb += both.eq(fifos[0].source.valid & fifos[1].source.valid)
        self.sync += [
            If(self.clear,
                aligned.eq(0),
                armed.eq(0),
                mismatch.eq(0)
            ).Elif(~aligned,
                mismatch.eq(0),
                If(gaps[0] & gaps[1],
                    armed.eq(1)
                ).Elif(armed & heads[0] & heads[1],
                    aligned.eq(1),
                    armed.eq(0)
                )
            ).Elif(both & (heads[0] != heads[1]),
                If(mismatch,
                    aligned.eq(0)
                ).Else(
                    mismatch.eq(1)
                )
            ).Elif(both & heads[0] & heads[1],
                mismatch.eq(0)
            )
        ]
        for lane in range(2):
            self.comb += [
                If(aligned,
                    sources[lane].valid.eq(both),
                    sources[lane].data.eq(fifos[lane].source.data),
                    sources[lane].ctrl.eq(fifos[lane].source.ctrl),
                    fifos[lane].source.ready.eq(both & sources[0].ready & sources[1].ready),
                ).Else(
                    # Searching: drop the words (armed: the words before the first marker).
                    fifos[lane].source.ready.eq(fifos[lane].source.valid & ~(armed & heads[lane])),
                )
            ]

# Lane Swap ----------------------------------------------------------------------------------------

# SerDes signals of the Gen 1 interface used by the USB3PIPE (sys clock domain): inputs (driven by the
# PIPE) and outputs, sink/source streams.
LANE_SWAP_INPUTS  = ["tx_idle", "tx_pattern", "tx_lfps", "tx_polarity", "rx_polarity",
    "tx_skp_enable", "rx_align"]
LANE_SWAP_OUTPUTS = ["ready", "rx_idle", "rx_errors"]

class LaneSerDes:
    """Logical lane SerDes (LaneSwap): the Gen 1 SerDes interface of a physical SerDes."""
    def __init__(self, serdes):
        for name in LANE_SWAP_INPUTS + LANE_SWAP_OUTPUTS:
            setattr(self, name, Signal.like(getattr(serdes, name), name=name))
        self.sink   = stream.Endpoint(serdes.sink.description)
        self.source = stream.Endpoint(serdes.source.description)
        self.rx_idle_stretch = serdes.rx_idle_stretch
        self.xcvr_name       = None
        self.gen2_capable    = False

class LaneSwap(LiteXModule):
    """Logical lanes (sources: Lane 0 = Configuration Lane, Lane 1) on two physical SerDes: logical
    Lane i on the physical SerDes i ^ swap (USB Type-C plug orientation, 6.13.1: a flipped plug swaps
    the lanes). The Gen 1 SerDes interface of the USB3PIPE is muxed (sys clock domain); swap is
    expected to change only while the lanes do not transmit data (Rx.Detect/Polling.LFPS).

    With detect, swap follows the lane receiving the partner's Polling.LFPS (sent on its Configuration
    Lane only, 6.13.12) while enable (ex: LTSSM in Polling.LFPS): LFPS activity (rx_idle de-asserted)
    for burst_cycles on a single physical lane selects it as Configuration Lane."""
    def __init__(self, serdes, detect=False, burst_cycles=16):
        assert len(serdes) == 2
        self.lanes  = lanes = [LaneSerDes(s) for s in serdes]
        self.swap   = swap  = Signal() # i (detect=False) / o (detect=True).
        self.enable = Signal()         # i: Detection enable (detect=True).

        # # #

        for p in range(2):
            phys = serdes[p]
            for name in LANE_SWAP_INPUTS:
                self.comb += getattr(phys, name).eq(
                    Mux(swap, getattr(lanes[1 - p], name), getattr(lanes[p], name)))
            for name in LANE_SWAP_OUTPUTS:
                self.comb += getattr(lanes[p], name).eq(
                    Mux(swap, getattr(serdes[1 - p], name), getattr(serdes[p], name)))
            # TX: logical lane sink -> physical SerDes sink (ready back).
            a, b = lanes[p].sink, lanes[1 - p].sink
            for name in [n for n, *_ in phys.sink.description.payload_layout] + ["valid", "first", "last"]:
                self.comb += getattr(phys.sink, name).eq(Mux(swap, getattr(b, name), getattr(a, name)))
            self.comb += lanes[p].sink.ready.eq(Mux(swap, serdes[1 - p].sink.ready, phys.sink.ready))
            # RX: physical SerDes source -> logical lane source (ready back).
            a, b = serdes[p].source, serdes[1 - p].source
            for name in [n for n, *_ in phys.source.description.payload_layout] + ["valid", "first", "last"]:
                self.comb += getattr(lanes[p].source, name).eq(Mux(swap, getattr(b, name), getattr(a, name)))
            self.comb += phys.source.ready.eq(Mux(swap, lanes[1 - p].source.ready, lanes[p].source.ready))

        # Configuration Lane detection -------------------------------------------------------------
        if detect:
            active = [Signal(max=burst_cycles + 1) for _ in range(2)]
            for p in range(2):
                self.sync += If(serdes[p].rx_idle,
                    active[p].eq(0)
                ).Elif(active[p] != burst_cycles,
                    active[p].eq(active[p] + 1)
                )
            self.sync += If(self.enable,
                If((active[1] == burst_cycles) & serdes[0].rx_idle,
                    swap.eq(1)
                ).Elif((active[0] == burst_cycles) & serdes[1].rx_idle,
                    swap.eq(0)
                )
            )
