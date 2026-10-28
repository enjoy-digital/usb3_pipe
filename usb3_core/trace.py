#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core Header Packets trace (debug, bench/usb3_trace.py).

Records the received (accepted by the Link Layer) and transmitted Header Packets of the Protocol
Layer (TPs/DPHs, ITPs excluded; LMPs are handled in the Link Layer and not recorded) in a memory
with a timestamp: the Protocol Layer exchanges with a real host can be observed without analyzer
(ex: flow control, NRDY/ERDY, control transfers).

Entry (64-bit): dw1[0:32], type[32:37] (DW0[4:0]), direction[37] (0: RX, 1: TX), timestamp[38:64]
(sys clock cycles/16).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from usb3_core.common import TYPE_ITP

# USB3 Core Trace ----------------------------------------------------------------------------------

class USB3CoreTrace(LiteXModule):
    def __init__(self, link, depth=512):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, reset=1, description="Trace enable (stops when full)."),
            CSRField("clear",  size=1, pulse=True, description="Trace clear (timestamp reset)."),
            CSRField("circular",   size=1, description="Circular (stopped with enable=0)."),
            CSRField("ept_filter", size=1, description="Record only the ept endpoint packets."),
            CSRField("ept",        size=4, description="Endpoint number (ept_filter)."),
        ])
        self._count   = CSRStatus(16, description="Recorded entries.")
        self._pointer = CSRStatus(16, description="Write pointer (circular: oldest entry).")
        self._index = CSRStorage(16, description="Entry to read.")
        self._data  = CSRStatus(64,  description="Entry (dw1, type, direction, timestamp).")

        # # #

        mem     = Memory(64, depth)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(async_read=True)
        self.specials += mem, wr_port, rd_port

        timestamp = Signal(30)
        count     = Signal(16)
        pointer   = Signal(max=depth)
        self.sync += timestamp.eq(timestamp + 1)

        rx = Signal()
        tx = Signal()
        # ITPs (sent every 125us by the host) are not recorded.
        self.comb += [
            rx.eq(link.rx_hp.valid & link.rx_hp.ready & (link.rx_hp.dw0[0:5] != TYPE_ITP)),
            tx.eq(link.tx_hp.valid & link.tx_hp.ready),
        ]
        # RX and TX on the same cycle: TX recorded on the next one.
        tx_pending = Signal()
        tx_dw0     = Signal(32, reset_less=True)
        tx_dw1     = Signal(32, reset_less=True)
        record     = Signal()
        entry      = Signal(64)
        self.comb += [
            If(rx,
                record.eq(1),
                entry.eq(Cat(link.rx_hp.dw1, link.rx_hp.dw0[0:5], C(0, 1), timestamp[4:30])),
            ).Elif(tx_pending,
                record.eq(1),
                entry.eq(Cat(tx_dw1, tx_dw0[0:5], C(1, 1), timestamp[4:30])),
            ).Elif(tx,
                record.eq(1),
                entry.eq(Cat(link.tx_hp.dw1, link.tx_hp.dw0[0:5], C(1, 1), timestamp[4:30])),
            ),
            wr_port.adr.eq(pointer),
            wr_port.dat_w.eq(entry),
            wr_port.we.eq(record & self._control.fields.enable &
                (self._control.fields.circular | (count < depth)) &
                (~self._control.fields.ept_filter | (entry[8:12] == self._control.fields.ept))),
        ]
        self.sync += [
            If(tx_pending & ~rx, tx_pending.eq(0)),
            If(rx & tx,
                tx_pending.eq(1),
                tx_dw0.eq(link.tx_hp.dw0),
                tx_dw1.eq(link.tx_hp.dw1),
            ),
            If(wr_port.we,
                pointer.eq(Mux(pointer == (depth - 1), 0, pointer + 1)),
                If(count != depth, count.eq(count + 1)),
            ),
            If(self._control.fields.clear,
                count.eq(0),
                pointer.eq(0),
                timestamp.eq(0),
            ),
        ]
        self.comb += [
            self._count.status.eq(count),
            self._pointer.status.eq(pointer),
            rd_port.adr.eq(self._index.storage),
            self._data.status.eq(rd_port.dat_r),
        ]

# USB3 Link Trace ----------------------------------------------------------------------------------

LINK_TRACE_EVENTS = ["rx_lc", "tx_lc", "rx_hp", "tx_hp", "u0_rise", "u0_fall", "recovery"]

class USB3LinkTrace(LiteXModule):
    """Link Layer events trace (debug, bench/usb3_trace.py link): Link Commands (RX: received and
    checked, TX: transmission started), Header Packets (RX: received and checked, LMPs included,
    TX: transmission started), U0 entry/exit and Recovery requests (with causes).

    Entry (96-bit): timestamp[0:32] (sys clock cycles), events[32:39], rx_lc_info[39:50],
    tx_lc_sel[50:61], rx_hp[61:77] (type, LMP subtype, seq, ok, accept, seq error), tx_hp[77:87]
    (seq, type, retry), recovery causes[87:92] (seq error, LGOOD, LCRD, PENDING_HP, CREDIT_HP).

    Modes: linear (stops when full) or circular (stops on a U0 exit or a Recovery request: the last entries before it are
    kept, oldest at the write pointer), keep-alive filter (LUP/LDN only entries and ITPs dropped).
    """
    def __init__(self, link, depth=1024):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, reset=1, description="Trace enable (stops when full)."),
            CSRField("clear",  size=1, pulse=True, description="Trace clear (timestamp reset)."),
            CSRField("circular", size=1, description="Circular, stops on a U0 exit."),
            CSRField("filter",   size=1, description="Drop LUP/LDN only entries and ITPs."),
        ])
        self._count   = CSRStatus(16, description="Recorded entries.")
        self._pointer = CSRStatus(16, description="Write pointer (circular: oldest entry).")
        self._index = CSRStorage(16, description="Entry to read.")
        self._data  = CSRStatus(96,  description="Entry.")

        # # #

        mem     = Memory(96, depth)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(async_read=True)
        self.specials += mem, wr_port, rd_port

        timestamp = Signal(32)
        count     = Signal(16)
        pointer   = Signal(max=depth)
        stopped   = Signal()
        u0_d      = Signal()
        self.sync += [
            timestamp.eq(timestamp + 1),
            u0_d.eq(link.dbg_u0),
        ]
        events = Signal(7)
        self.comb += events.eq(Cat(
            link.dbg_rx_lc, link.dbg_tx_lc, link.dbg_rx_hp, link.dbg_tx_hp,
            link.dbg_u0 & ~u0_d, ~link.dbg_u0 & u0_d, link.dbg_recovery))
        entry = Signal(96)
        # Keep-alive filter: LUP (TX), LDN (RX) and ITPs (with their LGOOD_n/LCRD_x) dropped.
        keepalive = Signal()
        itp       = Signal()
        self.comb += [
            itp.eq(link.dbg_rx_hp & (link.dbg_rx_hp_info[0:5] == TYPE_ITP)),
            keepalive.eq(
                (~link.dbg_rx_lc | (link.dbg_rx_lc_info == 0x580) | (link.dbg_rx_lc_info[7:11] == 0)) &
                (~link.dbg_tx_lc | (link.dbg_tx_lc_sel  == 0x400) | (link.dbg_tx_lc_sel[7:11]  == 0)) &
                (~link.dbg_rx_hp | itp) & ~link.dbg_tx_hp & (events[4:7] == 0)),
        ]
        record = Signal()
        self.comb += [
            entry.eq(Cat(timestamp, events, link.dbg_rx_lc_info, link.dbg_tx_lc_sel,
                link.dbg_rx_hp_info, link.dbg_tx_hp_info, link.dbg_recovery_cause)),
            record.eq((events != 0) & self._control.fields.enable & ~stopped &
                ~(self._control.fields.filter & keepalive)),
            wr_port.adr.eq(pointer),
            wr_port.dat_w.eq(entry),
            wr_port.we.eq(record & (self._control.fields.circular | (count < depth))),
        ]
        self.sync += [
            If(wr_port.we,
                pointer.eq(Mux(pointer == (depth - 1), 0, pointer + 1)),
                If(count != depth, count.eq(count + 1)),
                If(self._control.fields.circular & (events[5] | events[6]), stopped.eq(1)),
            ),
            If(self._control.fields.clear,
                count.eq(0),
                pointer.eq(0),
                stopped.eq(0),
                timestamp.eq(0),
            ),
        ]
        self.comb += [
            self._count.status.eq(count),
            self._pointer.status.eq(pointer),
            rd_port.adr.eq(self._index.storage),
            self._data.status.eq(rd_port.dat_r),
        ]

