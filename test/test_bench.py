#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3Bench: Bulk IN counter source words and Bulk OUT counter checker (32/64-bit)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.bench import USB3Bench

class TestUSB3Bench(unittest.TestCase):
    def check(self, data_width, n=64):
        # Source looped to the sink: consecutive 32-bit counter words, no checker errors.
        dut    = USB3Bench(with_csr=False, data_width=data_width)
        nw     = data_width//32
        words  = []
        errors = []
        def generator():
            yield dut.source.ready.eq(1)
            for _ in range(n):
                yield
                data = (yield dut.source.data)
                words.extend((data >> (32*i)) & 0xffffffff for i in range(nw))
            yield
            errors.append((yield dut.sink_errors))
        dut.comb += [
            dut.sink.valid.eq(dut.source.valid & dut.source.ready),
            dut.sink.data.eq(dut.source.data),
        ]
        run_simulation(dut, generator())
        self.assertEqual(words, list(range(len(words))))
        self.assertEqual(errors, [0])

    def test_32(self):
        self.check(32)

    def test_64(self):
        self.check(64)
