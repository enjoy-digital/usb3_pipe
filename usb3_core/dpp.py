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
        nbytes = Signal(bits_for(wb), reset_less=True)
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
    """64-bit Data Packet Payload words -> full 8-byte words (two: not last) or 32-bit words
    (data[0:32], 1-4 bytes) for the last bytes: the Link Layer appends the CRC-32/END on 32-bit
    words."""
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

class DPPQuadSplitter(LiteXModule):
    """128-bit Data Packet Payload words -> full 16-byte words (words=4: not last), the full 4-byte
    words of the last word (words=1-3, not last), then its last 1-4 bytes in a 32-bit word
    (data[0:32], words=1, last): the Link Layer appends the CRC-32/END on 32-bit words."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(dpp_layout(128))
        self.source = source = stream.Endpoint(dpp_layout(128) + [("words", 3)])

        # # #

        tail  = Signal() # Last bytes of a split last word.
        full  = Signal()
        split = Signal()
        k     = Signal(2) # Full 4-byte words of a split last word.
        self.comb += [
            full.eq((sink.bytes == 16) & ~sink.last),
            split.eq(sink.last & (sink.bytes > 4)),
            k.eq((sink.bytes - 1)[2:4]),
            source.valid.eq(sink.valid),
            source.first.eq(sink.first & ~tail),
            If(tail,
                Case(k, {i: source.data.eq(sink.data[32*i:]) for i in range(4)}),
                source.bytes.eq(sink.bytes - Cat(C(0, 2), k)),
                source.words.eq(1),
                source.last.eq(1),
                sink.ready.eq(source.ready),
            ).Elif(split,
                source.data.eq(sink.data),
                source.bytes.eq(Cat(C(0, 2), k)),
                source.words.eq(k),
            ).Else(
                source.data.eq(sink.data),
                source.bytes.eq(sink.bytes),
                source.words.eq(Mux(full, 4, 1)),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ),
        ]
        self.sync += If(source.valid & source.ready,
            tail.eq(~tail & split)
        )

class DPPPacker(LiteXModule):
    """32-bit Data Packet Payload words -> 64-bit (or dw-bit) words (full words, the last one 0 to
    dw/8 bytes): 32-bit endpoints (Control Endpoint, ...) on a 64/128-bit Link Layer (lower rate,
    stored and forwarded at Gen 2, Gen 1 PIPE at the same rate)."""
    def __init__(self, dw=64):
        assert dw in [64, 128]
        n = dw//32
        self.sink   = sink   = stream.Endpoint(dpp_layout(32))
        self.source = source = stream.Endpoint(dpp_layout(dw))

        # # #

        held  = [Signal(32, reset_less=True) for _ in range(n - 1)]
        count = Signal(max=n) # Held words.
        first = Signal(reset_less=True)
        out   = Signal()
        self.comb += [
            out.eq(sink.last | (count == (n - 1))),
            source.valid.eq(sink.valid & out),
            Case(count, {c: source.data.eq(Cat(*held[:c], sink.data)) for c in range(n)}),
            source.bytes.eq(Cat(C(0, 2), count) + sink.bytes),
            source.first.eq(Mux(count == 0, sink.first, first)),
            source.last.eq(sink.last),
            sink.ready.eq(Mux(out, source.ready, 1)),
        ]
        self.sync += If(sink.valid & sink.ready,
            If(out,
                count.eq(0),
            ).Else(
                Case(count, {c: held[c].eq(sink.data) for c in range(n - 1)}),
                If(count == 0, first.eq(sink.first)),
                count.eq(count + 1),
            )
        )

class DPPRXAdapter(LiteXModule):
    """64-bit (or dw-bit) received Data Packet Payload words (Link Layer, not held) -> 32-bit words
    for a 32-bit endpoint (Control Endpoint, ...), buffered, with the DPP done/ok status after its
    words.

    sink: DPP words of the endpoint (valid when routed to it), done/ok: DPP received/status
    (rx_dpp_done/rx_dpp_ok when routed to it), setup: SETUP DP. source/source_done/source_ok/
    source_setup: to the endpoint (rx_dpp, rx_dpp_done, rx_dpp_ok, rx_dpp_setup)."""
    def __init__(self, depth=128, dw=64):
        assert dw in [64, 128]
        n = dw//32
        self.sink         = sink   = stream.Endpoint(dpp_layout(dw))
        self.done         = Signal() # i
        self.ok           = Signal() # i
        self.setup        = Signal() # i
        self.source       = source = stream.Endpoint(dpp_layout(32))
        self.source_done  = Signal() # o
        self.source_ok    = Signal() # o
        self.source_setup = Signal() # o

        # # #

        # Buffered: synchronous RAM read (block RAMs; asynchronous reads are silently
        # mis-implemented in block RAMs by the Gowin toolchain, the DPP words then read back as
        # zeros on hardware).
        layout = dpp_layout(dw) + [("setup", 1), ("done", 1), ("ok", 1)]
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
        index = Signal(max=n) # 32-bit word of the dw-bit word.
        rem   = Signal(len(sink.bytes) + 1)
        more  = Signal()      # More bytes after this 32-bit word.
        self.comb += [
            rem.eq(fifo.source.bytes - Cat(C(0, 2), index)),
            more.eq(rem > 4),
            Case(index, {i: source.data.eq(fifo.source.data[32*i:32*(i + 1)]) for i in range(n)}),
            source.bytes.eq(Mux(more, 4, rem)),
            source.first.eq(fifo.source.first & (index == 0)),
            source.last.eq(fifo.source.last & ~more),
            self.source_setup.eq(fifo.source.setup),
            If(fifo.source.done,
                self.source_done.eq(fifo.source.valid),
                self.source_ok.eq(fifo.source.ok),
                fifo.source.ready.eq(1),
            ).Else(
                source.valid.eq(fifo.source.valid),
                fifo.source.ready.eq(~more),
            ),
        ]
        self.sync += If(fifo.source.valid & ~fifo.source.done,
            If(more, index.eq(index + 1)).Else(index.eq(0))
        )
