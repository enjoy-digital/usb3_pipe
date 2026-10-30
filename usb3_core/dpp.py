#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Data Packet Payloads reader (endpoints/EP0 memories to the Link Layer)."""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import *

# DPP Reader ---------------------------------------------------------------------------------------

class DPPReader(LiteXModule):
    """Data Packet Payloads reader: reads the jobs (memory word address, length in bytes, 0 for a
    zero-length DPP) from a memory read port (re, 1 cycle latency) to a continuous DPP stream (the
    Link Layer can't stall in a DPP: the words are prefetched)."""
    def __init__(self, rd_port, depth=16):
        dw = len(rd_port.dat_r)
        wb = dw//8 # Bytes per word.
        self.sink   = sink   = stream.Endpoint([("adr", len(rd_port.adr)), ("length", 11)])
        self.source = source = stream.Endpoint(dpp_layout(dw))

        # # #

        self.prefetch = prefetch = stream.SyncFIFO(dpp_layout(dw), depth)
        adr    = Signal(len(rd_port.adr), reset_less=True)
        rem    = Signal(11, reset_less=True)
        active = Signal()
        issue  = Signal()
        valid  = Signal()
        last   = Signal(reset_less=True)
        nbytes = Signal(4, reset_less=True)
        zlp    = Signal()
        self.comb += [
            sink.ready.eq(~active),
            issue.eq(active & (prefetch.level < (depth - 4))),
            rd_port.adr.eq(adr),
            rd_port.re.eq(issue),
        ]
        self.sync += [
            valid.eq(issue),
            last.eq(issue & (rem <= wb)),
            nbytes.eq(Mux(rem <= wb, rem, wb)),
            zlp.eq(sink.valid & sink.ready & (sink.length == 0)),
            If(sink.valid & sink.ready,
                active.eq(sink.length != 0),
                adr.eq(sink.adr),
                rem.eq(sink.length),
            ).Elif(issue,
                adr.eq(adr + 1),
                rem.eq(rem - wb),
                If(rem <= wb, active.eq(0)),
            ),
        ]
        self.comb += [
            prefetch.sink.valid.eq(valid | zlp),
            prefetch.sink.data.eq(rd_port.dat_r),
            prefetch.sink.bytes.eq(Mux(zlp, 0, nbytes)),
            prefetch.sink.last.eq(last | zlp),
            prefetch.source.connect(source),
        ]


class DPPSplitter(LiteXModule):
    """64-bit Data Packet Payload words -> full 8-byte words (two: not last) or 32-bit words (data[0:32],
    1-4 bytes) for the last bytes: the Link Layer appends the CRC-32/END on 32-bit words."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(dpp_layout(64))
        self.source = source = stream.Endpoint(dpp_layout(64) + [("two", 1)])

        # # #

        half  = Signal() # Upper half of a split word.
        full  = Signal()
        split = Signal()
        self.comb += [
            full.eq((sink.bytes == 8) & ~sink.last),
            split.eq(~full & (sink.bytes > 4)),
            source.valid.eq(sink.valid),
            source.first.eq(sink.first & ~half),
            If(half,
                source.data.eq(sink.data[32:64]),
                source.bytes.eq(sink.bytes - 4),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ).Elif(full,
                source.data.eq(sink.data),
                source.bytes.eq(8),
                source.two.eq(1),
                sink.ready.eq(source.ready),
            ).Elif(split,
                source.data.eq(sink.data[0:32]),
                source.bytes.eq(4),
            ).Else(
                source.data.eq(sink.data[0:32]),
                source.bytes.eq(sink.bytes),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ),
        ]
        self.sync += If(source.valid & source.ready,
            half.eq(~half & split)
        )


class DPPPacker(LiteXModule):
    """32-bit Data Packet Payload words -> 64-bit words (8 bytes, the last one 0-8 bytes): 32-bit
    endpoints (Control Endpoint, ...) on a 64-bit Link Layer (half rate, stored and forwarded at Gen
    2, Gen 1 PIPE at the same rate)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(dpp_layout(32))
        self.source = source = stream.Endpoint(dpp_layout(64))

        # # #

        held   = Signal(32, reset_less=True)
        held_v = Signal()
        first  = Signal(reset_less=True)
        self.comb += [
            If(~held_v,
                # First word of a pair (held) or a single last word.
                source.valid.eq(sink.valid & sink.last),
                source.data.eq(sink.data),
                source.bytes.eq(sink.bytes),
                source.first.eq(sink.first),
                source.last.eq(1),
                sink.ready.eq(Mux(sink.last, source.ready, 1)),
            ).Else(
                source.valid.eq(sink.valid),
                source.data.eq(Cat(held, sink.data)),
                source.bytes.eq(4 + sink.bytes),
                source.first.eq(first),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            )
        ]
        self.sync += If(sink.valid & sink.ready,
            If(~held_v & ~sink.last,
                held.eq(sink.data),
                first.eq(sink.first),
                held_v.eq(1),
            ).Else(
                held_v.eq(0),
            )
        )


class DPPRXAdapter(LiteXModule):
    """64-bit received Data Packet Payload words (Link Layer, not held) -> 32-bit words for a 32-bit
    endpoint (Control Endpoint, ...), buffered, with the DPP done/ok status after its words.

    sink: 64-bit DPP words of the endpoint (valid when routed to it), done/ok: DPP received/status
    (rx_dpp_done/rx_dpp_ok when routed to it), setup: SETUP DP. source/source_done/source_ok/
    source_setup: to the endpoint (rx_dpp, rx_dpp_done, rx_dpp_ok, rx_dpp_setup)."""
    def __init__(self, depth=128):
        self.sink         = sink   = stream.Endpoint(dpp_layout(64))
        self.done         = Signal() # i
        self.ok           = Signal() # i
        self.setup        = Signal() # i
        self.source       = source = stream.Endpoint(dpp_layout(32))
        self.source_done  = Signal() # o
        self.source_ok    = Signal() # o
        self.source_setup = Signal() # o

        # # #

        # Buffered: synchronous RAM read (block RAMs; asynchronous reads are silently mis-implemented in
        # block RAMs by the Gowin toolchain, the DPP words then read back as zeros on hardware).
        layout = dpp_layout(64) + [("setup", 1), ("done", 1), ("ok", 1)]
        self.fifo = fifo = stream.SyncFIFO(layout, depth, buffered=True)
        self.comb += [
            fifo.sink.valid.eq(sink.valid | self.done),
            fifo.sink.data.eq(sink.data),
            fifo.sink.bytes.eq(sink.bytes),
            fifo.sink.first.eq(sink.first),
            fifo.sink.last.eq(sink.last),
            fifo.sink.setup.eq(self.setup),
            fifo.sink.done.eq(self.done),
            fifo.sink.ok.eq(self.ok),
            sink.ready.eq(1),
        ]
        half = Signal() # Upper half of the 64-bit word.
        two  = Signal()
        self.comb += [
            two.eq(fifo.source.bytes > 4),
            source.data.eq(Mux(half, fifo.source.data[32:64], fifo.source.data[0:32])),
            source.bytes.eq(Mux(half, fifo.source.bytes - 4, Mux(two, 4, fifo.source.bytes))),
            source.first.eq(fifo.source.first & ~half),
            source.last.eq(fifo.source.last & (half | ~two)),
            self.source_setup.eq(fifo.source.setup),
            If(fifo.source.done,
                self.source_done.eq(fifo.source.valid),
                self.source_ok.eq(fifo.source.ok),
                fifo.source.ready.eq(1),
            ).Else(
                source.valid.eq(fifo.source.valid),
                fifo.source.ready.eq(half | ~two),
            ),
        ]
        self.sync += If(fifo.source.valid & ~fifo.source.done,
            half.eq(~half & two)
        )
