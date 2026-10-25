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
        ])
        self._count = CSRStatus(16,  description="Recorded entries.")
        self._index = CSRStorage(16, description="Entry to read.")
        self._data  = CSRStatus(64,  description="Entry (dw1, type, direction, timestamp).")

        # # #

        mem     = Memory(64, depth)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(async_read=True)
        self.specials += mem, wr_port, rd_port

        timestamp = Signal(30)
        count     = Signal(16)
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
            wr_port.adr.eq(count),
            wr_port.dat_w.eq(entry),
            wr_port.we.eq(record & self._control.fields.enable & (count < depth)),
        ]
        self.sync += [
            If(tx_pending & ~rx, tx_pending.eq(0)),
            If(rx & tx,
                tx_pending.eq(1),
                tx_dw0.eq(link.tx_hp.dw0),
                tx_dw1.eq(link.tx_hp.dw1),
            ),
            If(wr_port.we, count.eq(count + 1)),
            If(self._control.fields.clear,
                count.eq(0),
                timestamp.eq(0),
            ),
        ]
        self.comb += [
            self._count.status.eq(count),
            rd_port.adr.eq(self._index.storage),
            self._data.status.eq(rd_port.dat_r),
        ]
