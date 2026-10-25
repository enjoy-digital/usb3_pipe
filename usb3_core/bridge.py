#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 byte streams helpers: 32-bit endpoint words (last/be) to/from bytes (CDC-ACM...)."""

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import be_to_bytes, endpoint_layout

# Byte Unpacker ------------------------------------------------------------------------------------

class USB3ByteUnpacker(LiteXModule):
    """32-bit words (be: valid bytes) to bytes."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(endpoint_layout())
        self.source = source = stream.Endpoint([("data", 8)])

        # # #

        index  = Signal(2)
        nbytes = Signal(3)
        self.comb += [
            nbytes.eq(be_to_bytes(sink.be)),
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
    with be); packets are also ended at max_bytes."""
    def __init__(self, max_bytes=1024, idle_cycles=64):
        self.sink   = sink   = stream.Endpoint([("data", 8)])
        self.source = source = stream.Endpoint(endpoint_layout())

        # # #

        acc       = Signal(32, reset_less=True)
        idx       = Signal(2)
        out       = Signal(32, reset_less=True)
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
                source.be.eq(0b1111),
            ).Else(
                source.data.eq(acc),
                source.last.eq(1),
                Case(idx, {1: source.be.eq(0b0001), 2: source.be.eq(0b0011), 3: source.be.eq(0b0111)}),
            ),
            # No byte accepted while a partial word is sent (its index is reset).
            sink.ready.eq((~pending | (source.valid & source.ready)) & ~send_part),
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
