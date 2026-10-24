#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Dual-lane (Gen 1x2) building blocks (USB 3.2 6.13), see doc/gen2-x2-roadmap.md.

- LaneStriper   : 64-bit link words (8 symbols, symbol 0 on data[0:8]) striped on two 32-bit lanes,
                  aligned to Lane 0: even symbols on Lane 0, odd symbols on Lane 1 (6.13.4). The two
                  lanes must accept the words in lockstep.
- LaneDestriper : two 32-bit lanes to 64-bit link words.
- LaneDeskew    : aligns two lanes on a marker word (ex: Ordered Set duplicated on both lanes,
                  6.13.6), the lanes are then released in lockstep (the two sources must be consumed
                  together, ex: by a LaneDestriper).
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
    def __init__(self):
        self.sink    = sink    = stream.Endpoint(link_layout())
        self.sources = sources = [stream.Endpoint(lane_layout()) for _ in range(2)]

        # # #

        # The lanes must be consumed in lockstep (lanes valid/ready are not cross-gated: no
        # combinatorial loop with sinks whose ready depends on valid).
        self.comb += sink.ready.eq(sources[0].ready & sources[1].ready)
        for lane, source in enumerate(sources):
            self.comb += [
                source.valid.eq(sink.valid),
                source.data.eq(Cat(*[sink.data[8*(2*i + lane):8*(2*i + lane + 1)] for i in range(4)])),
                source.ctrl.eq(Cat(*[sink.ctrl[2*i + lane] for i in range(4)])),
            ]

# Lane Destriper -----------------------------------------------------------------------------------

class LaneDestriper(LiteXModule):
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
    """Two lanes deskew (up to depth words of skew): words before the marker are dropped on each lane
    until the marker is at the head of both lanes, then both lanes are released together.

    With a periodic marker (ex: COM words of TS1/TS2 Ordered Sets, every 4 words), max_skew (words)
    pairs the markers of the same period: the words are timestamped on arrival and two head markers
    received more than max_skew cycles apart are not paired, the older one is dropped (the skew must
    be below half the marker period)."""
    def __init__(self, marker_data, marker_ctrl, depth=8, max_skew=None):
        self.sinks   = sinks   = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.sources = sources = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.aligned = aligned = Signal() # o

        # # #

        fifo_layout = lane_layout() + [("time", 8)]
        fifos = [stream.SyncFIFO(fifo_layout, depth) for _ in range(2)]
        self.submodules += fifos
        time = Signal(8)
        self.sync += time.eq(time + 1)
        heads = []
        for lane in range(2):
            head = Signal()
            self.comb += [
                sinks[lane].connect(fifos[lane].sink),
                fifos[lane].sink.time.eq(time),
                head.eq(fifos[lane].source.valid &
                    (fifos[lane].source.data == marker_data) & (fifos[lane].source.ctrl == marker_ctrl)),
            ]
            heads.append(head)

        # Markers pairing.
        pair  = Signal()
        stale = [Signal() for _ in range(2)]
        if max_skew is None:
            self.comb += pair.eq(heads[0] & heads[1])
        else:
            diff = Signal((8, True)) # Lane 0 marker time - Lane 1 marker time.
            self.comb += [
                diff.eq(fifos[0].source.time - fifos[1].source.time),
                If(heads[0] & heads[1],
                    If((diff <= max_skew) & (diff >= -max_skew),
                        pair.eq(1)
                    ).Elif(diff > 0,
                        stale[1].eq(1)
                    ).Else(
                        stale[0].eq(1)
                    )
                )
            ]

        both = Signal()
        self.comb += both.eq(fifos[0].source.valid & fifos[1].source.valid)
        self.sync += If(~aligned & pair, aligned.eq(1))
        for lane in range(2):
            self.comb += [
                If(aligned,
                    sources[lane].valid.eq(both),
                    sources[lane].data.eq(fifos[lane].source.data),
                    sources[lane].ctrl.eq(fifos[lane].source.ctrl),
                    fifos[lane].source.ready.eq(both & sources[0].ready & sources[1].ready),
                ).Else(
                    # Searching: drop the words before the marker (and the stale markers).
                    fifos[lane].source.ready.eq(fifos[lane].source.valid & (~heads[lane] | stale[lane])),
                )
            ]
