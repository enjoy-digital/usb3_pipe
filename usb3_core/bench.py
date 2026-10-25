#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Bulk bandwidth benchmark (scripts/usb3_bench.py).

- Source (Bulk IN) : 32-bit incrementing counter words at full rate (max size packets).
- Sink   (Bulk OUT): checks the 32-bit incrementing counter words (resynchronized on errors).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect     import stream
from litex.soc.interconnect.csr import *

from usb3_core.common import endpoint_layout

# USB3 Bench ---------------------------------------------------------------------------------------

class USB3Bench(LiteXModule):
    def __init__(self, with_csr=True):
        self.source = source = stream.Endpoint(endpoint_layout()) # To Bulk IN.
        self.sink   = sink   = stream.Endpoint(endpoint_layout()) # From Bulk OUT.

        self.source_enable = Signal(reset=1) # i
        self.clear         = Signal()        # i: Counters clear (next received word not checked).
        self.source_words  = Signal(64)      # o
        self.sink_words    = Signal(64)      # o
        self.sink_errors   = Signal(32)      # o

        # # #

        # Source.
        count = Signal(32)
        self.comb += [
            source.valid.eq(self.source_enable),
            source.data.eq(count),
            source.be.eq(0b1111),
        ]
        self.sync += If(source.valid & source.ready,
            count.eq(count + 1),
            self.source_words.eq(self.source_words + 1),
        )

        # Sink.
        expected = Signal(32, reset_less=True)
        self.comb += sink.ready.eq(1)
        self.sync += [
            If(sink.valid,
                expected.eq(sink.data + 1),
                self.sink_words.eq(self.sink_words + 1),
                If((sink.data != expected) & (self.sink_words != 0),
                    self.sink_errors.eq(self.sink_errors + 1)
                )
            ),
            If(self.clear,
                self.source_words.eq(0),
                self.sink_words.eq(0),
                self.sink_errors.eq(0),
            )
        ]

        if with_csr:
            self.add_csr()

    def add_csr(self):
        self._control      = CSRStorage(fields=[
            CSRField("source_enable", size=1, reset=1, description="Bulk IN source enable."),
            CSRField("clear",         size=1, offset=1, pulse=True, description="Counters clear."),
        ])
        self._source_words = CSRStatus(64, description="Bulk IN words sent.")
        self._sink_words   = CSRStatus(64, description="Bulk OUT words received.")
        self._sink_errors  = CSRStatus(32, description="Bulk OUT words errors (discontinuities).")
        self.comb += [
            self.source_enable.eq(self._control.fields.source_enable),
            self.clear.eq(self._control.fields.clear),
            self._source_words.status.eq(self.source_words),
            self._sink_words.status.eq(self.sink_words),
            self._sink_errors.status.eq(self.sink_errors),
        ]
