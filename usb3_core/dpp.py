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
        self.sink   = sink   = stream.Endpoint([("adr", len(rd_port.adr)), ("length", 11)])
        self.source = source = stream.Endpoint(dpp_layout())

        # # #

        self.prefetch = prefetch = stream.SyncFIFO(dpp_layout(), depth)
        adr    = Signal(len(rd_port.adr), reset_less=True)
        rem    = Signal(11, reset_less=True)
        active = Signal()
        issue  = Signal()
        valid  = Signal()
        last   = Signal(reset_less=True)
        nbytes = Signal(3, reset_less=True)
        zlp    = Signal()
        self.comb += [
            sink.ready.eq(~active),
            issue.eq(active & (prefetch.level < (depth - 4))),
            rd_port.adr.eq(adr),
            rd_port.re.eq(issue),
        ]
        self.sync += [
            valid.eq(issue),
            last.eq(issue & (rem <= 4)),
            nbytes.eq(Mux(rem <= 4, rem, 4)),
            zlp.eq(sink.valid & sink.ready & (sink.length == 0)),
            If(sink.valid & sink.ready,
                active.eq(sink.length != 0),
                adr.eq(sink.adr),
                rem.eq(sink.length),
            ).Elif(issue,
                adr.eq(adr + 1),
                rem.eq(rem - 4),
                If(rem <= 4, active.eq(0)),
            ),
        ]
        self.comb += [
            prefetch.sink.valid.eq(valid | zlp),
            prefetch.sink.data.eq(rd_port.dat_r),
            prefetch.sink.bytes.eq(Mux(zlp, 0, nbytes)),
            prefetch.sink.last.eq(last | zlp),
            prefetch.source.connect(source),
        ]
