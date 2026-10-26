#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab DUT UART: LiteX RS232 PHY on the DUT pins, byte streams (to/from a CDC-ACM function),
baudrate from a CSR or the CDC-ACM line coding, RX/TX bytes events."""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from litex.soc.cores.uart import RS232PHYTX, RS232PHYRX

# AgentLab UART ------------------------------------------------------------------------------------

class AgentLabUART(LiteXModule):
    """DUT UART (8N1).

    sink  : bytes to the DUT (TX).
    source: bytes from the DUT (RX, dropped when the FIFO is full: no back-pressure on a UART).
    tx_event/rx_event (pulses) and tx_data/rx_data: bytes transmitted (end of the stop bit)/received
    (middle of the stop bit), for the events.
    line_coding/line_baudrate: baudrate set by the host (CDC-ACM SET_LINE_CODING), also writable
    from the baudrate CSR.

    pads: tx/rx pads, None: internal loopback (TX to RX).
    """
    def __init__(self, pads, clk_freq, baudrate=115200, fifo_depth=256):
        self.sink          = sink   = stream.Endpoint([("data", 8)]) # i
        self.source        = source = stream.Endpoint([("data", 8)]) # o
        self.tx_event      = Signal()   # o
        self.tx_data       = Signal(8)  # o
        self.rx_event      = Signal()   # o
        self.rx_data       = Signal(8)  # o
        self.line_coding   = Signal()   # i
        self.line_baudrate = Signal(32) # i
        self.connected     = Signal()   # i
        self.tx_pin        = Signal()   # o: TX line (logic analyzer probe).
        self.rx_pin        = Signal()   # o: RX line (logic analyzer probe).

        self._baudrate    = CSRStorage(32, reset=baudrate, write_from_dev=True,
            description="Baudrate (also set by the host CDC-ACM line coding).")
        self._tuning_word = CSRStatus(32, description="RS232 PHY tuning word (from the baudrate).")
        self._status      = CSRStatus(fields=[
            CSRField("connected", size=1, description="Host terminal connected (CDC-ACM DTR)."),
        ])

        # # #

        # Pads (internal loopback without pads).
        if pads is None:
            pads = Record([("tx", 1), ("rx", 1)])
            self.comb += pads.rx.eq(pads.tx)
        self.comb += [
            self.tx_pin.eq(pads.tx),
            self.rx_pin.eq(pads.rx),
        ]

        # Baudrate (CSR or line coding) to tuning word: baudrate*2^32/clk_freq, as a multiplication
        # by a constant (baudrates up to 2^24-1).
        shift       = 16
        factor      = round(2**(32 + shift)/clk_freq)
        baudrate_r  = Signal(32, reset=baudrate) # Last baudrate set (CSR or line coding).
        baud        = Signal(24)
        product     = Signal(24 + bits_for(factor))
        tuning_word = Signal(32)
        self.comb += [
            # Baudrate CSR updated by the line coding (read back).
            self._baudrate.we.eq(self.line_coding),
            self._baudrate.dat_w.eq(self.line_baudrate),
            self._tuning_word.status.eq(tuning_word),
            self._status.fields.connected.eq(self.connected),
        ]
        self.sync += [
            If(self.line_coding,
                baudrate_r.eq(self.line_baudrate)
            ).Elif(self._baudrate.re,
                baudrate_r.eq(self._baudrate.storage)
            ),
            If(baudrate_r[24:] != 0,
                baud.eq(2**24 - 1)
            ).Else(
                baud.eq(baudrate_r)
            ),
            product.eq(baud*factor),
            tuning_word.eq(product[shift:]),
        ]

        # TX.
        self.tx_fifo = tx_fifo = stream.SyncFIFO([("data", 8)], fifo_depth, buffered=True)
        self.tx      = tx      = RS232PHYTX(pads, tuning_word)
        self.comb += [
            sink.connect(tx_fifo.sink),
            tx_fifo.source.connect(tx.sink),
            self.tx_event.eq(tx.sink.valid & tx.sink.ready),
            self.tx_data.eq(tx.sink.data),
        ]

        # RX.
        self.rx      = rx      = RS232PHYRX(pads, tuning_word)
        self.rx_fifo = rx_fifo = stream.SyncFIFO([("data", 8)], fifo_depth, buffered=True)
        self.comb += [
            rx.source.connect(rx_fifo.sink),
            rx_fifo.source.connect(source),
            self.rx_event.eq(rx.source.valid),
            self.rx_data.eq(rx.source.data),
        ]
