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
- LanesPacketAligner : destriped 64-bit link words -> link words pairs of the packets (any packet
                  placement, each packet starting on a 32-bit word, Logical Idle removed).
- LanesTXPacker : link words pairs -> 64-bit link words for the striper (packets on 64-bit words).
- LanesTXSymbolShift : 64-bit link words delayed by 1-7 symbols (test: packets at any position).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import SHP, SDP, SLC, EPF

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
    the TS2 Ordered Sets on one lane, Logical Idle after them), the lanes still paired. A marker
    against a plain data word (no K Symbol: Logical Idle or a TS body word, not a corrupted marker)
    clears the pairing at once: the lanes were paired Ordered Sets apart (identical TS Ordered Sets
    hide it until the end of the stream).

    The sinks are never held (continuous lanes streams, a held lane would overflow the SerDes RX
    buffers): words received with a full FIFO are dropped. A word dropped while armed (a lane held on
    its first marker more than depth words: the other lane's first marker is not the same one) or
    while aligned (the lanes skewed beyond depth) clears the pairing and flushes the FIFOs: a stale
    marker would otherwise be paired, the lanes Ordered Sets apart.

    error_symbol: K Symbol substituted for the RX 8b/10b errors (K28.4, 6.3.1.4). A word with one is not
    counted in a gap and disarms an armed search (a lane not word aligned yet receives garbage: as a
    gap, its first marker would be paired with a later Ordered Set of the other lane), with these
    exceptions seen on hardware:
    - A marker word with some of its Symbols as error_symbol is a marker: the first COM Symbols of a
      TS Ordered Set after an idle period often arrive with errors, on one or both lanes.
    - An error word ending with error_symbol followed by a marker word: the Symbol before the first
      COM, or the first COM Symbols on a lane shifted by the word alignment.
    - Up to max_isolated error words once a lane reached its gap: isolated errors in a Logical Idle as
      the receiver leaves electrical idle, before the first TS Ordered Set of a Recovery (no gap again
      before its markers: the lanes would stay unpaired until a retrain).
    Garbage from a lane not word aligned yet has error words in quick succession: still disarms.
    """
    def __init__(self, marker_data, marker_ctrl, depth=8, gap=16, error_symbol=None, max_isolated=3):
        self.sinks   = sinks   = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.sources = sources = [stream.Endpoint(lane_layout()) for _ in range(2)]
        self.aligned = aligned = Signal() # o
        self.armed   = armed   = Signal() # o: Search armed (gap seen, pairing on the next markers).
        self.clear   = Signal()           # i: Pairing cleared (search re-armed on the next gap).

        # # #

        nsym = len(sinks[0].ctrl)
        assert 0 <= max_isolated <= 3

        # Marker word: the marker, or with error_symbol the marker with some of its Symbols received
        # as error_symbol (at least one marker Symbol left).
        def marker_match(data, ctrl):
            if error_symbol is None:
                return (data == marker_data) & (ctrl == marker_ctrl)
            sym = marker_data & 0xff
            assert marker_data == int.from_bytes(bytes([sym])*nsym, "little")
            assert marker_ctrl == 2**nsym - 1
            is_m = [data[8*i:8*(i+1)] == sym          for i in range(nsym)]
            is_e = [data[8*i:8*(i+1)] == error_symbol for i in range(nsym)]
            return ((ctrl == marker_ctrl) &
                Reduce("AND", [m | e for m, e in zip(is_m, is_e)]) & Reduce("OR", is_m))

        # FIFOs, with the marker/plain flags stored with the words (the heads are a single FIFO output
        # bit, not a word compare: timing), flushed on an overflow clear.
        flush = Signal()
        self.fifos = fifos = [ResetInserter()(stream.SyncFIFO(lane_layout() + [("marker", 1), ("plain", 1)],
            depth, buffered=True)) for _ in range(2)]
        self.submodules += fifos
        self.comb += [fifo.reset.eq(flush) for fifo in fifos]

        # Per lane inputs, registered with their marker/error flags (timing: the Symbol compares in
        # their own cycle).
        heads   = []
        gaps    = []
        disarms = []
        inputs  = []
        for lane in range(2):
            head       = Signal()
            gap_count  = Signal(max=gap + 1)
            marker_c   = Signal()
            error_c    = Signal()
            trail_c    = Signal()
            valid_in   = Signal()
            data_in    = Signal(len(sinks[lane].data))
            ctrl_in    = Signal(len(sinks[lane].ctrl))
            marker_in  = Signal()
            plain_in   = Signal()
            error_in   = Signal()
            trail_in   = Signal()
            trail_hold = Signal()
            tolerated  = Signal(2) # Isolated error words tolerated since the gap was reached.
            isolated   = Signal()  # Error word tolerated (gap reached, budget left).
            self.comb += [
                sinks[lane].ready.eq(1),
                marker_c.eq(marker_match(sinks[lane].data, sinks[lane].ctrl)),
            ]
            if error_symbol is not None:
                is_e = [sinks[lane].ctrl[i] & (sinks[lane].data[8*i:8*(i+1)] == error_symbol)
                    for i in range(nsym)]
                self.comb += [
                    error_c.eq(Reduce("OR", is_e)),
                    # Error Symbols only as a run ending the word.
                    trail_c.eq(is_e[-1] & Reduce("AND", [~is_e[i] | is_e[i+1] for i in range(nsym - 1)])),
                ]
            self.sync += [
                valid_in.eq(sinks[lane].valid),
                data_in.eq(sinks[lane].data),
                ctrl_in.eq(sinks[lane].ctrl),
                marker_in.eq(marker_c),
                plain_in.eq(sinks[lane].ctrl == 0),
                error_in.eq(error_c),
                trail_in.eq(trail_c),
            ]
            self.comb += [
                fifos[lane].sink.valid.eq(valid_in),
                fifos[lane].sink.data.eq(data_in),
                fifos[lane].sink.ctrl.eq(ctrl_in),
                fifos[lane].sink.marker.eq(marker_in),
                fifos[lane].sink.plain.eq(plain_in),
                head.eq(fifos[lane].source.valid & fifos[lane].source.marker),
            ]
            inputs.append(valid_in)
            heads.append(head)
            # Words without marker nor error Symbol received on the lane (saturated to gap); once the
            # gap is reached, up to max_isolated error words neither break it nor disarm the search.
            self.comb += isolated.eq((gap_count == gap) & (tolerated != max_isolated))
            self.sync += If(valid_in,
                If(marker_in,
                    gap_count.eq(0),
                    tolerated.eq(0),
                ).Elif(error_in & isolated,
                    tolerated.eq(tolerated + 1)
                ).Elif(error_in,
                    gap_count.eq(0),
                    tolerated.eq(0),
                ).Elif(gap_count != gap,
                    gap_count.eq(gap_count + 1)
                )
            )
            gaps.append(gap_count == gap)
            # Error word (not a corrupted marker word) disarming an armed search; with its error
            # Symbols ending it, decided on the next word (not followed by a marker word).
            disarm = Signal()
            self.sync += If(valid_in, trail_hold.eq(error_in & ~marker_in & trail_in & ~isolated))
            self.comb += disarm.eq(valid_in & (
                (error_in & ~marker_in & ~trail_in & ~isolated) | (trail_hold & ~marker_in)))
            disarms.append(disarm)
        self.heads = heads

        # Search armed after a gap on both lanes (no marker in the FIFOs, dropped while searching),
        # first markers paired; aligned until the markers are no longer paired (second mismatch
        # without paired markers in between, or a marker against a plain data word).
        both     = Signal()
        mismatch = Signal()
        overflow = Signal() # A word received with a full FIFO (dropped).
        plain    = [Signal() for n in range(2)]
        # Pairs released while aligned, with their head flags, registered (timing).
        pair_r   = Signal()
        head_r   = [Signal() for n in range(2)]
        plain_r  = [Signal() for n in range(2)]
        self.comb += [
            both.eq(fifos[0].source.valid & fifos[1].source.valid),
            overflow.eq(Reduce("OR", [inputs[n] & ~fifos[n].sink.ready for n in range(2)])),
            flush.eq(overflow & (armed | aligned)),
            *[plain[n].eq(fifos[n].source.valid & fifos[n].source.plain) for n in range(2)],
        ]
        self.sync += [
            If(self.clear | flush,
                aligned.eq(0),
                armed.eq(0),
                mismatch.eq(0)
            ).Elif(~aligned,
                mismatch.eq(0),
                If(disarms[0] | disarms[1],
                    armed.eq(0)
                ).Elif(gaps[0] & gaps[1],
                    armed.eq(1)
                ).Elif(armed & heads[0] & heads[1],
                    aligned.eq(1),
                    armed.eq(0)
                )
            ).Elif(pair_r & (head_r[0] != head_r[1]),
                If(mismatch | (head_r[0] & plain_r[1]) | (head_r[1] & plain_r[0]),
                    aligned.eq(0)
                ).Else(
                    mismatch.eq(1)
                )
            ).Elif(pair_r & head_r[0] & head_r[1],
                mismatch.eq(0)
            )
        ]
        self.sync += [
            pair_r.eq(aligned & both & ~self.clear & ~flush),
            *[head_r[n].eq(heads[n])  for n in range(2)],
            *[plain_r[n].eq(plain[n]) for n in range(2)],
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

# Lanes Framing Aligner ----------------------------------------------------------------------------

def link_pairs_layout():
    """Link words pairs (words: 1 or 2 valid words, word 0 first: data[0:32]/ctrl[0:4])."""
    return [("data", 64), ("ctrl", 8), ("words", 2)]

class LanesPacketAligner(LiteXModule):
    """64-bit link words (destriped lanes, Symbol 0 on data[0:8]) -> link words pairs of the
    packets only, each packet starting on a 32-bit word (as RXWordAligner + LogicalIdleFilter for
    Gen 1x1), for packets at any symbol position of the 64-bit words (they may start on either
    lane, 6.13.4) with any number of Logical Idle symbols between them (none included).

    Symbols parser (8 symbols per cycle): a packet starts on a SHP/SLC/SDP K-symbol outside of a
    packet: Header Packet: 20 symbols (framing, 12-byte header, CRC-16, Link Control Word), Link
    Command: 8 symbols, DPP: from the SDP framing to the EPF of its END/EDB framing (7.2.1,
    7.2.4). A word holds the symbols of at most two packets (the end of one, the start of the
    next). Each packet's output words are the 32-bit slices of the symbols stream from its start
    (rotation: start symbol modulo 4, words taken from a two words window), its last word
    completed with Logical Idle symbols (00h data, as received from a partner keeping its packets
    on 32-bit words): a packet always starts on a 32-bit word, Logical Idle between the packets
    removed. Up to 4 words per cycle (end of a packet, its completed last word, start of the next)
    written in rotation to 4 word FIFOs (buffer_symbols/4 words each), up to two words per cycle
    on the output (registered, no back-pressure). Timing: no per symbol placement from a buffer
    level (the previous symbols buffer did not reach 125MHz on the Tang Mega 138K Pro, t28); the
    completions add up to 3 symbols per packet, absorbed by the FIFOs (overflow: sticky flag,
    never with Logical Idle or a SKP Ordered Set every few packets).

    Not handled (as the Gen 1x1 path): a corrupted first framing symbol (the packet is then taken
    from the next symbol, misaligned: CRC error and retry by the Link Layer), a corrupted EPF at
    the end of a DPP (the DPP ends at the next EPF), a K EPF symbol right after the framing of a DPP
    starting in the same word (not its end; impossible in a valid DPP: data, CRC-32 and END first)."""
    def __init__(self, buffer_symbols=32):
        assert buffer_symbols % 4 == 0
        self.sink     = sink   = stream.Endpoint(link_layout())
        self.source   = source = stream.Endpoint(link_pairs_layout())
        self.overflow = Signal() # o: Symbols buffer overflow (sticky).

        # # #

        IDLE, FIXED, DPP_FRAMING, DPP = range(4)

        # Symbols parser (registered: symbol, keep, end of packet) ---------------------------------
        mode  = Signal(2)
        count = Signal(5)
        sym   = [Signal(9, name=f"sym{i}")  for i in range(8)] # {k, data}.
        keep  = [Signal(name=f"keep{i}")    for i in range(8)] # Packet symbol.
        last  = [Signal(name=f"last{i}")    for i in range(8)] # Last symbol of a packet.
        valid = Signal()
        self.comb += sink.ready.eq(1)

        # Input registered with the framing K-symbols decoded (timing: the 64-bit words come
        # straight from the lanes destriping/deskew FIFOs).
        in_valid = Signal()
        in_data  = Signal(64)
        in_ctrl  = Signal(8)
        in_k     = {s: [Signal(name=f"is_{n}{i}") for i in range(8)]
            for s, n in [(SHP, "shp"), (SLC, "slc"), (SDP, "sdp"), (EPF, "epf")]}
        self.sync += [
            in_valid.eq(sink.valid),
            in_data.eq(sink.data),
            in_ctrl.eq(sink.ctrl),
        ]
        for i in range(8):
            for s in in_k:
                self.sync += in_k[s][i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == s.value))

        # Parser without a chain of 8 dependent symbol steps (timing): a word holds at most the end
        # of the current packet then the start of one packet (a packet is 8 symbols or more), the
        # end found from the count (Header Packet/Link Command) or the first EPF (DPP), the start by
        # a priority encoder after it. count: symbols of the current packet (FIXED) or of its DPP
        # framing (DPP_FRAMING) from the word's symbol 0.
        def first(bits):
            """(found, index) of the first set bit (priority encoder)."""
            found, index = Signal(), Signal(3)
            self.comb += found.eq(Reduce("OR", bits))
            for i in reversed(range(8)):
                self.comb += If(bits[i], index.eq(i))
            return found, index
        is_start = [in_k[SHP][i] | in_k[SLC][i] | in_k[SDP][i] for i in range(8)]
        # End of the current packet in this word (ended, e).
        epf_from = Signal(4) # First symbol of the EPF search (DPP: 0, DPP_FRAMING: after it).
        self.comb += epf_from.eq(Mux(mode == DPP_FRAMING, count, 0))
        epf_found, epf_index = first([in_k[EPF][i] & (i >= epf_from) for i in range(8)])
        ended = Signal()
        e     = Signal(3)
        self.comb += Case(mode, {
            IDLE:        ended.eq(0),
            FIXED:       [ended.eq(count <= 8), e.eq(count - 1)],
            DPP_FRAMING: [ended.eq((count <= 8) & epf_found), e.eq(epf_index)],
            DPP:         [ended.eq(epf_found), e.eq(epf_index)],
        })
        # Start of a packet after it (or from symbol 0 out of a packet): started, s, kind.
        # The start search does not depend on the end found in this word (timing, t29: two chained
        # priority encoders): no SHP/SLC/SDP K-symbol inside a packet's remaining symbols but its
        # framing (FIXED/DPP_FRAMING: the first count symbols; a DPP: only END K-symbols before its
        # EPF).
        start_ok = [Signal(name=f"start_ok{i}") for i in range(8)]
        for i in range(8):
            self.comb += start_ok[i].eq(is_start[i] &
                ((mode == IDLE) | (mode == DPP) | (count <= i)))
        started, s_index = first(start_ok)
        s_shp = Signal()
        s_slc = Signal()
        self.comb += [
            s_shp.eq(Array(in_k[SHP])[s_index]),
            s_slc.eq(Array(in_k[SLC])[s_index]),
        ]
        in_packet = Signal()
        self.comb += in_packet.eq(mode != IDLE)
        for i in range(8):
            data, k = in_data[8*i:8*(i+1)], in_ctrl[i]
            _keep = Signal(name=f"keep_c{i}")
            _last = Signal(name=f"last_c{i}")
            self.comb += [
                _keep.eq((in_packet & (~ended | (i <= e))) | (started & (i >= s_index))),
                # Last symbol: end of the current packet, or of a Link Command starting at symbol 0.
                _last.eq((ended & (e == i)) | ((i == 7) & started & s_slc & (s_index == 0))),
            ]
            self.sync += [
                sym[i].eq(Cat(data, k)),
                keep[i].eq(in_valid & _keep),
                last[i].eq(in_valid & _last),
            ]
        # State at the end of the word.
        nm = Signal(2)
        nc = Signal(5)
        self.comb += [
            nm.eq(mode), nc.eq(count - 8),
            If(started,
                If(s_shp,
                    nm.eq(FIXED), nc.eq(12 + s_index),       # 20 symbols from s.
                ).Elif(s_slc,
                    nm.eq(Mux(s_index == 0, IDLE, FIXED)), nc.eq(s_index), # 8 symbols from s.
                ).Else(
                    nm.eq(Mux(s_index > 4, DPP_FRAMING, DPP)), nc.eq(s_index - 4), # Framing: 4.
                )
            ).Elif(ended | (mode == IDLE),
                nm.eq(IDLE),
            ).Elif(mode == DPP_FRAMING,
                nm.eq(DPP), # Framing (3 symbols at most) ended in this word.
            ),
        ]
        self.sync += [
            valid.eq(in_valid),
            If(in_valid,
                mode.eq(nm),
                count.eq(nc),
            )
        ]

        # Packets word ends (registered with the symbols) -------------------------------------------
        # A word holds the symbols of at most two packets: A, the current packet (continued, or started
        # in this word out of a packet), and B, a packet started after the end of A. A packet's output
        # words are the 32-bit slices of the symbols stream from its start (rotation r: its start
        # symbol modulo 4), its last word completed with Logical Idle symbols.
        r_cur = Signal(2) # Rotation of the current packet.
        a_act = Signal()  # A in this word.
        a_r   = Signal(2) # A rotation.
        a_new = Signal()  # A started in this word, at a_s.
        a_s   = Signal(3)
        a_end = Signal()  # A ended in this word, at a_e.
        a_e   = Signal(3)
        b_new = Signal()  # B started in this word (after the end of A), at b_s.
        b_s   = Signal(3)
        self.sync += [
            If(in_valid & started, r_cur.eq(s_index[:2])),
            If(in_packet,
                a_act.eq(in_valid),
                a_r.eq(r_cur),
                a_new.eq(0),
                a_end.eq(ended),
                a_e.eq(e),
                b_new.eq(in_valid & started),
                b_s.eq(s_index),
            ).Else(
                a_act.eq(in_valid & started),
                a_r.eq(s_index[:2]),
                a_new.eq(1),
                a_s.eq(s_index),
                # Link Command starting at symbol 0: ends in this word.
                a_end.eq(s_slc & (s_index == 0)),
                a_e.eq(7),
                b_new.eq(0),
            )
        ]
        # Previous word (window: previous word symbols -8..-1, this word 0..7).
        prev = [Signal(9, name=f"prev{i}") for i in range(8)]
        self.sync += If(valid, *[prev[i].eq(sym[i]) for i in range(8)])
        def win(p):
            return prev[p + 8] if p < 0 else (sym[p] if p < 8 else Constant(0, 9))

        # Word candidates (registered) --------------------------------------------------------------
        # A: words ending at i1 = (r+3)%4 and i1+4 in this word, and with A ending in this word, its
        # last word (ending at i1+8, completed); B: its first word when complete in this word.
        def word(syms):
            return Cat(*[x[:8] for x in syms]), Cat(*[x[8] for x in syms])
        cand_d = [Signal(32, name=f"cand_d{k}") for k in range(4)]
        cand_c = [Signal(4,  name=f"cand_c{k}") for k in range(4)]
        cand_v = [Signal(name=f"cand_v{k}")     for k in range(4)]
        cases  = {}
        for r in range(4):
            i1 = (r + 3) % 4
            stmts = []
            for k in range(3):
                ws = i1 - 3 + 4*k # Word start symbol.
                syms = []
                for p in range(ws, ws + 4):
                    x = win(p)
                    if p >= 0:
                        # Past the end of A: Logical Idle (completion).
                        m = Signal(9, name=f"m{r}_{k}_{p - ws}")
                        self.comb += m.eq(Mux(a_end & (p > a_e), 0, x) if p < 8 else 0)
                        x = m
                    syms.append(x)
                d, c = word(syms)
                v_new = (a_s <= ws) if ws >= 0 else 0
                v_end = (ws <= a_e) if ws <= 7 else 0
                v = a_act & (~a_new | v_new) & (~a_end | v_end)
                if k == 2:
                    v = v & a_end # Last word only (completed).
                stmts += [cand_d[k].eq(d), cand_c[k].eq(c), cand_v[k].eq(v)]
            cases[r] = stmts
        self.sync += Case(a_r, cases)
        b_cases = {}
        for bs in range(1, 5):
            d, c = word([sym[p] for p in range(bs, bs + 4)])
            b_cases[bs] = [cand_d[3].eq(d), cand_c[3].eq(c), cand_v[3].eq(b_new)]
        b_cases["default"] = cand_v[3].eq(0)
        self.sync += Case(b_s, b_cases)

        # Words FIFOs (4 banks written in rotation, up to 4 words per cycle) -------------------------
        word_layout = [("data", 32), ("ctrl", 4)]
        banks = [stream.SyncFIFO(word_layout, buffer_symbols//4) for _ in range(4)]
        self.submodules += banks
        wp  = Signal(2)
        pos = [Signal(2, name=f"pos{k}") for k in range(4)]
        self.comb += [
            pos[0].eq(wp),
            pos[1].eq(wp + cand_v[0]),
            pos[2].eq(wp + cand_v[0] + cand_v[1]),
            pos[3].eq(wp + cand_v[0] + cand_v[1] + cand_v[2]),
        ]
        self.sync += wp.eq(wp + cand_v[0] + cand_v[1] + cand_v[2] + cand_v[3])
        for b, bank in enumerate(banks):
            sel = If(cand_v[0] & (pos[0] == b), bank.sink.valid.eq(1), bank.sink.data.eq(cand_d[0]),
                bank.sink.ctrl.eq(cand_c[0]))
            for k in range(1, 4):
                sel = sel.Elif(cand_v[k] & (pos[k] == b), bank.sink.valid.eq(1),
                    bank.sink.data.eq(cand_d[k]), bank.sink.ctrl.eq(cand_c[k]))
            self.comb += sel
            self.sync += If(bank.sink.valid & ~bank.sink.ready, self.overflow.eq(1))

        # Output: up to two words per cycle (registered) ----------------------------------------------
        rp  = Signal(2)
        rp1 = Signal(2)
        self.comb += rp1.eq(rp + 1)
        v0  = Signal()
        v1  = Signal()
        self.comb += [
            v0.eq(Array(bk.source.valid for bk in banks)[rp]),
            v1.eq(v0 & Array(bk.source.valid for bk in banks)[rp1]),
        ]
        for b, bank in enumerate(banks):
            self.comb += bank.source.ready.eq((v0 & (rp == b)) | (v1 & (rp1 == b)))
        self.sync += [
            source.valid.eq(v0),
            source.words.eq(Mux(v1, 2, 1)),
            source.data.eq(Cat(Array(bk.source.data for bk in banks)[rp],
                               Array(bk.source.data for bk in banks)[rp1])),
            source.ctrl.eq(Cat(Array(bk.source.ctrl for bk in banks)[rp],
                               Array(bk.source.ctrl for bk in banks)[rp1])),
            rp.eq(rp + v0 + v1),
        ]

# Lanes TX Packer ----------------------------------------------------------------------------------

class LanesTXPacker(LiteXModule):
    """Link words pairs (1 or 2 words per entry, packets first/last, a pair never spans two packets)
    -> 64-bit link words for the striped lanes (two words per entry, packets first/last): a 64-bit
    word never holds words of two packets, the word after the last word of a packet is a Logical Idle
    word (both lanes end the packets together: SKP Ordered Sets inserted on both lanes between the
    packets, 6.4.3.2). Up to two words per cycle (pairs of two words, or a held word + one word)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(link_pairs_layout())
        self.source = source = stream.Endpoint(link_layout())

        # # #

        held       = Signal()
        held_first = Signal()
        held_last  = Signal()
        held_data  = Signal(32, reset_less=True)
        held_ctrl  = Signal(4,  reset_less=True)
        two = sink.words == 2

        # Outputs.
        self.comb += [
            If(held & held_last,
                # Held last word of a packet + Logical Idle word.
                source.valid.eq(1),
                source.data.eq(Cat(held_data, Constant(0, 32))),
                source.ctrl.eq(Cat(held_ctrl, Constant(0, 4))),
                source.first.eq(held_first),
                source.last.eq(1),
            ).Elif(held,
                # Held word + first word of the entry.
                source.valid.eq(sink.valid),
                source.data.eq(Cat(held_data, sink.data[0:32])),
                source.ctrl.eq(Cat(held_ctrl, sink.ctrl[0:4])),
                source.first.eq(held_first),
                source.last.eq(sink.last & ~two),
                sink.ready.eq(source.ready),
            ).Elif(two,
                source.valid.eq(sink.valid),
                source.data.eq(sink.data),
                source.ctrl.eq(sink.ctrl),
                source.first.eq(sink.first),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ).Elif(sink.last,
                # Last word of a packet alone + Logical Idle word.
                source.valid.eq(sink.valid),
                source.data.eq(Cat(sink.data[0:32], Constant(0, 32))),
                source.ctrl.eq(Cat(sink.ctrl[0:4], Constant(0, 4))),
                source.first.eq(sink.first),
                source.last.eq(1),
                sink.ready.eq(source.ready),
            ).Else(
                # Word alone (not last): held.
                sink.ready.eq(1),
            )
        ]
        # Held word.
        self.sync += [
            If(source.valid & source.ready,
                held.eq(0)
            ),
            If(sink.valid & sink.ready,
                If(held & two,
                    held.eq(1),
                    held_first.eq(0),
                    held_last.eq(sink.last),
                    held_data.eq(sink.data[32:64]),
                    held_ctrl.eq(sink.ctrl[4:8]),
                ).Elif(~held & ~two & ~sink.last,
                    held.eq(1),
                    held_first.eq(sink.first),
                    held_last.eq(0),
                    held_data.eq(sink.data[0:32]),
                    held_ctrl.eq(sink.ctrl[0:4]),
                )
            )
        ]

# Lanes TX Symbol Shift ----------------------------------------------------------------------------

class LanesTXSymbolShift(LiteXModule):
    """64-bit link words delayed by shift symbols (1-7, Logical Idle symbols before the first word,
    the end of the last word of a burst flushed with Logical Idle symbols): packets starting at
    any symbol position of the 64-bit words (on Lane 1 with an odd shift), as from a partner not
    keeping its packets on 64-bit words. Test of the receivers (sim.py --lanes-tx-shift)."""
    def __init__(self, shift):
        assert 1 <= shift <= 7
        self.sink   = sink   = stream.Endpoint(link_layout())
        self.source = source = stream.Endpoint(link_layout())

        # # #

        prev_data = Signal(64)
        prev_ctrl = Signal(8)
        pending   = Signal() # End of the previous word not sent yet.
        cur_data  = Mux(sink.valid, sink.data, 0)
        cur_ctrl  = Mux(sink.valid, sink.ctrl, 0)
        data      = Cat(prev_data, cur_data)
        ctrl      = Cat(prev_ctrl, cur_ctrl)
        self.comb += [
            source.valid.eq(sink.valid | pending),
            source.data.eq(data[8*(8 - shift):8*(16 - shift)]),
            source.ctrl.eq(ctrl[8 - shift:16 - shift]),
            source.first.eq(sink.first),
            source.last.eq(~sink.valid | sink.last),
            sink.ready.eq(source.ready),
        ]
        self.sync += If(source.valid & source.ready,
            prev_data.eq(cur_data),
            prev_ctrl.eq(cur_ctrl),
            pending.eq(sink.valid),
        )
