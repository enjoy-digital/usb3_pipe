#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.datapath import RXSKPRemover

from test.common import (words_to_symbols, symbols_to_words, strip_skp, random_symbols,
    stream_driver, stream_monitor)

# SKP Ordered Sets handling (USB 3.2 6.4.3) --------------------------------------------------------

class TestSKP(unittest.TestCase):
    def run_skp_remover(self, symbols, valid_rate, ready_rate, seed):
        rng      = random.Random(seed)
        words    = symbols_to_words(symbols)
        expected = symbols_to_words(strip_skp(symbols)[:4*(len(strip_skp(symbols))//4)])

        def checker(dut):
            dut.words = yield from stream_monitor(dut.source, len(expected),
                rng=random.Random(seed + 1), ready_rate=ready_rate, timeout=20*len(words))

        dut = RXSKPRemover()
        run_simulation(dut, [stream_driver(dut.sink, words, rng=rng, valid_rate=valid_rate), checker(dut)])
        return dut.words, expected

    def test_rx_skp_remover_backpressure(self):
        # SKP removal must not lose/corrupt symbols when the downstream applies back-pressure.
        for ready_rate in [1.0, 0.7, 0.3]:
            for valid_rate in [1.0, 0.8]:
                with self.subTest(ready_rate=ready_rate, valid_rate=valid_rate):
                    rng     = random.Random(0)
                    symbols = random_symbols(4*1024, rng, skp_rate=0.02, k_rate=0.02)
                    words, expected = self.run_skp_remover(symbols, valid_rate, ready_rate, seed=2)
                    self.assertEqual(words, expected)

    def test_rx_skp_remover_stall_without_skp(self):
        # Directed case: a few cycles of back-pressure on a stream without SKP.
        rng     = random.Random(3)
        symbols = random_symbols(4*64, rng)
        words   = symbols_to_words(symbols)

        def generator(dut):
            for d, c in words:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(d)
                yield dut.sink.ctrl.eq(c)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)

        def checker(dut):
            dut.words = []
            cycle = 0
            while len(dut.words) < len(words) and cycle < 4*len(words):
                ready = 0 if 10 <= cycle < 13 else 1
                yield dut.source.ready.eq(ready)
                yield
                if ready and (yield dut.source.valid):
                    dut.words.append(((yield dut.source.data), (yield dut.source.ctrl)))
                cycle += 1

        dut = RXSKPRemover()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.words, words)

if __name__ == "__main__":
    unittest.main()
