#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.daisho_mod.usb3.usb3_top import USB3Top


# USB3 Core Endpoints ------------------------------------------------------------------------------

# Daisho endpoints are double-buffered RAMs (2 x 1024 bytes): the data words are transmitted/received
# with their first byte on data[31:24] (as the descriptors).

USB3_CORE_EP_WORDS = 256 # 1024 bytes (SuperSpeed Bulk max packet size).

class USB3CoreEndpointIN(LiteXModule):
    """USB3 Core IN Endpoint (Device -> Host, EP1).

    Writes the packets of the sink stream (32-bit words, last: end of packet, packets longer than
    1024 bytes are split) to the endpoint buffer and commits them (one packet per buffer).
    """
    def __init__(self):
        self.sink = sink = stream.Endpoint([("data", 32)])

        # Daisho endpoint interface.
        self.buf_in_addr       = Signal(9)  # o
        self.buf_in_data       = Signal(32) # o
        self.buf_in_wren       = Signal()   # o
        self.buf_in_ready      = Signal()   # i
        self.buf_in_commit     = Signal()   # o
        self.buf_in_commit_len = Signal(11) # o
        self.buf_in_commit_ack = Signal()   # i

        # # #

        count = Signal(max=USB3_CORE_EP_WORDS + 1)

        self.fsm = fsm = FSM(reset_state="WRITE")
        fsm.act("WRITE",
            sink.ready.eq(self.buf_in_ready),
            self.buf_in_addr.eq(count),
            self.buf_in_data.eq(sink.data),
            If(sink.valid & sink.ready,
                self.buf_in_wren.eq(1),
                NextValue(count, count + 1),
                If(sink.last | (count == (USB3_CORE_EP_WORDS - 1)),
                    NextState("COMMIT")
                )
            )
        )
        fsm.act("COMMIT",
            self.buf_in_commit.eq(1),
            self.buf_in_commit_len.eq(4*count),
            If(self.buf_in_commit_ack,
                NextState("COMMIT-ACK")
            )
        )
        fsm.act("COMMIT-ACK",
            If(~self.buf_in_commit_ack,
                NextValue(count, 0),
                NextState("WRITE")
            )
        )

class USB3CoreEndpointOUT(LiteXModule):
    """USB3 Core OUT Endpoint (Host -> Device, EP2).

    Reads the received packets from the endpoint buffer to the source stream (32-bit words, last: end
    of packet) and re-arms the endpoint buffer.
    """
    def __init__(self, fifo_depth=8):
        self.source = source = stream.Endpoint([("data", 32)])

        # Daisho endpoint interface.
        self.buf_out_addr    = Signal(9)  # o
        self.buf_out_q       = Signal(32) # i
        self.buf_out_len     = Signal(11) # i
        self.buf_out_hasdata = Signal()   # i
        self.buf_out_arm     = Signal()   # o
        self.buf_out_arm_ack = Signal()   # i

        # # #

        # Words read from the endpoint buffer (2 clocks read latency) to a FIFO.
        self.fifo = fifo = stream.SyncFIFO([("data", 32)], fifo_depth)
        self.comb += fifo.source.connect(source)

        nwords   = Signal(9)
        count    = Signal(10)
        inflight = Signal(max=fifo_depth + 1)
        rd_valid = Signal(2)
        rd_last  = Signal(2)
        issue    = Signal()
        self.comb += [
            nwords.eq((self.buf_out_len + 3)[2:]),
            self.buf_out_addr.eq(count),
        ]
        self.sync += [
            rd_valid.eq(Cat(issue, rd_valid[0])),
            rd_last.eq(Cat(issue & (count == (nwords - 1)), rd_last[0])),
            inflight.eq(inflight + issue - (fifo.sink.valid & fifo.sink.ready)),
        ]
        self.comb += [
            fifo.sink.valid.eq(rd_valid[1]),
            fifo.sink.data.eq(self.buf_out_q),
            fifo.sink.last.eq(rd_last[1]),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(count, 0),
            If(self.buf_out_hasdata & (self.buf_out_len != 0),
                NextState("READ")
            ).Elif(self.buf_out_hasdata,
                NextState("ARM") # Zero-length packet.
            )
        )
        fsm.act("READ",
            issue.eq((fifo.level + inflight) < (fifo_depth - 1)),
            If(issue,
                NextValue(count, count + 1),
                If(count == (nwords - 1),
                    NextState("FLUSH")
                )
            )
        )
        fsm.act("FLUSH",
            If((rd_valid == 0),
                NextState("ARM")
            )
        )
        fsm.act("ARM",
            self.buf_out_arm.eq(1),
            If(self.buf_out_arm_ack,
                NextState("ARM-ACK")
            )
        )
        fsm.act("ARM-ACK",
            If(~self.buf_out_arm_ack,
                NextState("IDLE")
            )
        )

# USB3 Core ----------------------------------------------------------------------------------------

class USB3Core(LiteXModule):
    """USB3.0 Core

    Wrap the Daisho USB3.0 Core (https://github.com/mossmann/daisho) in a Module with small adaptations
    to make it compatible with the USB3.0 PIPE.
    - New top to only keep link/protocol layers and and endpoints.
    - Additional logic to emulate Daisho's internal ltssm states.
    - Additional TX FIFO to handle back-pressure from the SerDes. (not handled by Daisho)
    - Bulk endpoints as streams: ep1_sink (EP1 IN, Device -> Host), ep2_source (EP2 OUT, Host ->
      Device), 32-bit words (first byte on data[31:24]), last: end of packet (max 1024 bytes).
    """
    def __init__(self, platform, daisho_core="daisho"):
        self.reset  = Signal() # i: Core reset (LTSSM not in Polling.Idle/Recovery.Idle/U0).
        self.u0     = Signal() # i: LTSSM in U0.
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Daisho LTSSM state -----------------------------------------------------------------------
        # Daisho's Link Layer initializes on the LT_POLLING_IDLE -> LT_POLLING_U0 transition (Header
        # Sequence Number/Rx Header Buffer Credit Advertisements, Port Capability LMP, 7.2.4.1.1,
        # 8.4.5) and only receives in LT_POLLING_U0: the core is released from reset during the Idle
        # Symbol handshake (LT_POLLING_IDLE) and enters LT_POLLING_U0 with the LTSSM, so the partner's
        # Advertisements, sent right after its U0 entry, are not lost.
        LT_POLLING_IDLE = 15
        LT_POLLING_U0   = 16
        ltssm_state     = Signal(5)
        self.comb += [
            If(self.u0,
                ltssm_state.eq(LT_POLLING_U0)
            ).Else(
                ltssm_state.eq(LT_POLLING_IDLE)
            )
        ]

        # RX (Sink) --------------------------------------------------------------------------------

        in_data   = Signal(32)
        in_datak  = Signal(4)
        in_active = Signal()
        self.comb += [
            sink.ready.eq(1),
            in_data.eq(sink.data),
            in_datak.eq(sink.ctrl),
            in_active.eq(sink.valid),
        ]

        # TX (Source) ------------------------------------------------------------------------------
        # Daisho core does not support back-pressure (ready signal of LiteX's streams). To accomodate
        # that, we use a FIFO that absorbs the data bursts from the core and re-transmits datas to
        # the USB3 Pipe at the maximum allowed rate with back-pressure. This is a hack for our tests
        # and should be fixed correctly in the core.

        # FIFO
        self.out_fifo = out_fifo = stream.SyncFIFO([("data", 32), ("ctrl", 4)], 32)

        # Map core signals to stream, re-generate first/last delimiters from active signal.
        out_data     = Signal(32)
        out_datak    = Signal(4)
        out_stall    = Signal()
        out_active   = Signal()
        out_active_d = Signal()
        self.sync += [
            out_fifo.sink.valid.eq(out_active),
            out_fifo.sink.data.eq(out_data),
            out_fifo.sink.ctrl.eq(out_datak),
            out_active_d.eq(out_active),
            out_fifo.sink.first.eq(out_active & ~out_active_d),
        ]
        self.comb += [
            out_stall.eq(out_fifo.level > 16),
            out_fifo.sink.last.eq(~out_active & out_active_d),
        ]

        # Connect FIFO to source.
        self.comb += [
            If(out_fifo.source.valid,
                out_fifo.source.connect(source)
            ).Else(
                source.valid.eq(1),
                source.first.eq(0),
                source.last.eq(0),
                source.data.eq(0),
                source.ctrl.eq(0),
            )
        ]

        # Daisho USB3 core -------------------------------------------------------------------------
        self.usb3_top = usb3_top = USB3Top(platform)
        self.submodules += usb3_top

        self.comb += [
            usb3_top.clk.eq(ClockSignal("sys")),
            usb3_top.reset.eq(self.reset),
            usb3_top.ltssm_state.eq(ltssm_state),

            usb3_top.in_data.eq(in_data),
            usb3_top.in_datak.eq(in_datak),
            usb3_top.in_active.eq(in_active),

            out_data.eq(usb3_top.out_data),
            out_datak.eq(usb3_top.out_datak),
            out_active.eq(usb3_top.out_active),
            usb3_top.out_stall.eq(out_stall),
        ]

        # Daisho USB3 core endpoints --------------------------------------------------------------
        # EP1: IN (Device -> Host) from ep1_sink, EP2: OUT (Host -> Device) to ep2_source.
        self.ep1 = ep1 = USB3CoreEndpointIN()
        self.ep2 = ep2 = USB3CoreEndpointOUT()
        self.ep1_sink   = ep1.sink
        self.ep2_source = ep2.source
        self.comb += [
            usb3_top.buf_in_addr.eq(ep1.buf_in_addr),
            usb3_top.buf_in_data.eq(ep1.buf_in_data),
            usb3_top.buf_in_wren.eq(ep1.buf_in_wren),
            ep1.buf_in_ready.eq(usb3_top.buf_in_ready),
            usb3_top.buf_in_commit.eq(ep1.buf_in_commit),
            usb3_top.buf_in_commit_len.eq(ep1.buf_in_commit_len),
            ep1.buf_in_commit_ack.eq(usb3_top.buf_in_commit_ack),

            usb3_top.buf_out_addr.eq(ep2.buf_out_addr),
            ep2.buf_out_q.eq(usb3_top.buf_out_q),
            ep2.buf_out_len.eq(usb3_top.buf_out_len),
            ep2.buf_out_hasdata.eq(usb3_top.buf_out_hasdata),
            usb3_top.buf_out_arm.eq(ep2.buf_out_arm),
            ep2.buf_out_arm_ack.eq(usb3_top.buf_out_arm_ack),
        ]

        # Daisho USB3 sources ----------------------------------------------------------------------
        daisho_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), daisho_core)
        platform.add_verilog_include_path(os.path.join(daisho_path))
        platform.add_verilog_include_path(os.path.join(daisho_path, "usb3"))
        platform.add_source_dir(os.path.join(daisho_path, "usb3"))
