#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 byte streams helpers: 32-bit endpoint words (last/last_be) to/from bytes (CDC-ACM...)."""

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import last_be_to_bytes

# Byte Unpacker ------------------------------------------------------------------------------------

class USB3ByteUnpacker(LiteXModule):
    """32-bit words (last_be on the last word of a packet) to bytes."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("last_be", 4)])
        self.source = source = stream.Endpoint([("data", 8)])

        # # #

        index  = Signal(2)
        nbytes = Signal(3)
        self.comb += [
            If(sink.last & (sink.last_be != 0),
                nbytes.eq(last_be_to_bytes(sink.last_be))
            ).Else(
                nbytes.eq(4)
            ),
            source.valid.eq(sink.valid),
            Case(index, {i: source.data.eq(sink.data[8*i:8*(i+1)]) for i in range(4)}),
            sink.ready.eq(source.ready & (index == (nbytes - 1))),
        ]
        self.sync += If(source.valid & source.ready,
            index.eq(index + 1),
            If(sink.ready, index.eq(0))
        )

# Byte Packer ---------------------------------------------------------------------------------------

class USB3BytePacker(LiteXModule):
    """Bytes to 32-bit words. A complete word is held until the next byte (packet continues) or
    idle_cycles without bytes (packet ended: last), partial words are sent after idle_cycles (last
    with last_be); packets are also ended at max_bytes."""
    def __init__(self, max_bytes=1024, idle_cycles=64):
        self.sink   = sink   = stream.Endpoint([("data", 8)])
        self.source = source = stream.Endpoint([("data", 32), ("last_be", 4)])

        # # #

        acc       = Signal(32)
        idx       = Signal(2)
        out       = Signal(32)
        pending   = Signal() # Complete word held.
        pkt_words = Signal(max=max_bytes//4)
        at_max    = Signal()
        send_full = Signal()
        send_part = Signal()
        self.idle = idle = WaitTimer(idle_cycles)
        self.comb += [
            idle.wait.eq(~sink.valid & (pending | (idx != 0))),
            at_max.eq(pkt_words == (max_bytes//4 - 1)),
            send_full.eq(pending & (sink.valid | idle.done | at_max)),
            send_part.eq(~pending & (idx != 0) & idle.done),
            source.valid.eq(send_full | send_part),
            If(pending,
                source.data.eq(out),
                source.last.eq(~sink.valid | at_max),
                If(source.last, source.last_be.eq(0b1000)),
            ).Else(
                source.data.eq(acc),
                source.last.eq(1),
                Case(idx, {1: source.last_be.eq(0b0001), 2: source.last_be.eq(0b0010), 3: source.last_be.eq(0b0100)}),
            ),
            sink.ready.eq(~pending | (source.valid & source.ready)),
        ]
        self.sync += [
            If(source.valid & source.ready,
                pending.eq(0),
                pkt_words.eq(pkt_words + 1),
                If(source.last, pkt_words.eq(0)),
                If(send_part, idx.eq(0)),
            ),
            If(sink.valid & sink.ready,
                Case(idx, {i: acc[8*i:8*(i+1)].eq(sink.data) for i in range(4)}),
                idx.eq(idx + 1),
                If(idx == 3,
                    out.eq(Cat(acc[0:24], sink.data)),
                    pending.eq(1),
                )
            ),
        ]
