#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.common import AsyncFIFO

class TestAsyncFIFO(unittest.TestCase):
    def check(self, clocks, depth=8, n=256, seed=0):
        # Words written/read in order with random valid/ready on both sides.
        random.seed(seed)
        dut   = ClockDomainsRenamer({"write": "w", "read": "r"})(AsyncFIFO([("data", 32)], depth))
        datas = [random.getrandbits(32) for _ in range(n)]
        rx    = []
        def writer():
            for d in datas:
                while random.random() < 0.3:
                    yield dut.sink.valid.eq(0)
                    yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(d)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
        def reader():
            while len(rx) < n:
                yield dut.source.ready.eq(random.random() < 0.7)
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    rx.append((yield dut.source.data))
        run_simulation(dut, {"w": writer(), "r": reader()}, clocks={"sys": 10, **clocks})
        self.assertEqual(rx, datas)

    def test_same_freq(self):
        self.check({"w": 10, "r": 10})

    def test_write_faster(self):
        self.check({"w": 8, "r": 16})

    def test_read_faster(self):
        self.check({"w": 16, "r": 8}, depth=16)

if __name__ == "__main__":
    unittest.main()
