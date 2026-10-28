#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Control Endpoint (EP0) Wishbone bridge: SoC bus access over vendor requests.

Wire-compatible with LiteX CommUSB (litex/tools/remote/comm_usb.py, litex_server --usb):
- Read : bmRequestType 0xC3 (Device-to-Host, Vendor, Other), bRequest 0x00.
- Write: bmRequestType 0x43 (Host-to-Device, Vendor, Other), bRequest 0x00.
- wValue/wIndex: 32-bit byte address (low/high 16 bits), word aligned.
- wLength: 4*N bytes (N=1 for CommUSB), incrementing burst of N 32-bit words (N <= 128), data in
  little endian.

Writes are buffered (the OUT Data stage has no back-pressure) and executed in order, before any
subsequent read. Other requests (or invalid lengths) are stalled.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone

from usb3_core.control import EP0_HANDLER_WORDS

# Constants ----------------------------------------------------------------------------------------

USB3_WISHBONE_READ    = 0xc3
USB3_WISHBONE_WRITE   = 0x43
USB3_WISHBONE_REQUEST = 0x00

# USB3 EP0 Wishbone --------------------------------------------------------------------------------

class USB3EP0Wishbone(LiteXModule):
    """Wishbone master on the EP0 request handler (USB3DeviceCore(with_ep0_handler=True)).

    write_fifo_depth: buffered write words (2 full-size write requests by default), writes are
    executed at Wishbone speed while each Control transfer takes several microseconds.
    """
    def __init__(self, ep0, write_fifo_depth=2*EP0_HANDLER_WORDS):
        self.bus = bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")

        # # #

        # Request decoding -------------------------------------------------------------------------
        length_ok = Signal()
        is_read   = Signal()
        is_write  = Signal()
        self.comb += [
            length_ok.eq((ep0.req_length[:2] == 0) &
                         (ep0.req_length != 0) &
                         (ep0.req_length <= 4*EP0_HANDLER_WORDS)),
            is_read.eq( (ep0.req_type == USB3_WISHBONE_READ)  &
                        (ep0.req_request == USB3_WISHBONE_REQUEST) & length_ok),
            is_write.eq((ep0.req_type == USB3_WISHBONE_WRITE) &
                        (ep0.req_request == USB3_WISHBONE_REQUEST) & length_ok),
            ep0.req_stall.eq(ep0.req_valid & ~is_read & ~is_write),
        ]
        address = Signal(30)
        self.comb += address.eq(Cat(ep0.req_value, ep0.req_index)[2:])

        # Writes: OUT Data stage words with their address, buffered --------------------------------
        self.write_fifo = write_fifo = stream.SyncFIFO([("adr", 30), ("dat", 32)],
            depth=write_fifo_depth, buffered=True)
        write_adr = Signal(30)
        self.sync += [
            If(ep0.req_setup,
                write_adr.eq(address),
            ).Elif(write_fifo.sink.valid,
                write_adr.eq(write_adr + 1),
            )
        ]
        self.comb += [
            ep0.req_out.ready.eq(1),
            write_fifo.sink.valid.eq(ep0.req_out.valid & ep0.req_valid & is_write),
            write_fifo.sink.adr.eq(write_adr),
            write_fifo.sink.dat.eq(ep0.req_out.data),
        ]

        # Reads: pending request, cancelled by a new SETUP -----------------------------------------
        read_pending = Signal()
        read_cancel  = Signal()
        read_adr     = Signal(30)
        read_count   = Signal(max=EP0_HANDLER_WORDS + 1)
        read_data    = Signal(32)

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        self.sync += [
            If(ep0.req_setup,
                read_pending.eq(is_read),
                read_cancel.eq(~fsm.ongoing("IDLE")),
                read_adr.eq(address),
                read_count.eq(ep0.req_length[2:]),
            ),
            If(fsm.ongoing("IDLE"),
                read_cancel.eq(0),
            ),
        ]
        fsm.act("IDLE",
            # Writes first: a read requested after a write returns the written data.
            If(write_fifo.source.valid,
                NextState("WRITE")
            ).Elif(read_pending & ~ep0.req_setup,
                NextValue(read_pending, 0),
                NextState("READ")
            )
        )
        fsm.act("WRITE",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.we.eq(1),
            bus.sel.eq(0b1111),
            bus.adr.eq(write_fifo.source.adr),
            bus.dat_w.eq(write_fifo.source.dat),
            If(bus.ack,
                write_fifo.source.ready.eq(1),
                NextState("IDLE")
            )
        )
        fsm.act("READ",
            bus.stb.eq(1),
            bus.cyc.eq(1),
            bus.we.eq(0),
            bus.sel.eq(0b1111),
            bus.adr.eq(read_adr),
            If(bus.ack,
                NextValue(read_data, bus.dat_r),
                NextValue(read_adr, read_adr + 1),
                NextState("RESPONSE")
            )
        )
        fsm.act("RESPONSE",
            If(read_cancel | ep0.req_setup,
                NextState("IDLE")
            ).Else(
                ep0.req_in.valid.eq(1),
                ep0.req_in.data.eq(read_data),
                ep0.req_in.last.eq(read_count == 1),
                ep0.req_in.be.eq(0b1111),
                If(ep0.req_in.ready,
                    NextValue(read_count, read_count - 1),
                    If(read_count == 1,
                        NextState("IDLE")
                    ).Else(
                        NextState("READ")
                    )
                )
            )
        )
