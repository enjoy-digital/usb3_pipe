#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.gen import *

from usb3_pipe.datapath import RXSKPRemover
from usb3_pipe.common   import SKP, K

from test.common import words_to_symbols, symbols_to_words, strip_skp, random_symbols
from test.common import stream_driver, stream_monitor

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
        run_simulation(dut, [stream_driver(dut.sink, words, rng=rng, valid_rate=valid_rate),
            checker(dut)])
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

# SKP runs parity (Gen 1x2: a lane shifted by one Symbol by its SKP removal) -----------------------

class TestSKPParityCheck(unittest.TestCase):
    def run_check(self, symbols):
        # symbols: (value, k) in time order, words of 4 (Symbol 0 first). Returns the odd pulses
        # count.
        from usb3_pipe.datapath import RXSKPParityCheck
        from litex.soc.interconnect import stream
        sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        dut  = RXSKPParityCheck(sink)
        dut.sink = sink
        odd  = []
        def gen():
            for i in range(0, len(symbols), 4):
                word = symbols[i:i+4]
                yield sink.valid.eq(1)
                yield sink.ready.eq(1)
                yield sink.data.eq(sum(v << (8*j) for j, (v, k) in enumerate(word)))
                yield sink.ctrl.eq(sum(k << j for j, (v, k) in enumerate(word)))
                yield
                odd.append((yield dut.odd))
            yield sink.valid.eq(0)
            for _ in range(4):
                yield
                odd.append((yield dut.odd))
        run_simulation(dut, gen())
        return sum(odd)

    D   = (0x00, 0)
    SKP_ = (SKP.value, 1)
    SUB_ = (K(28, 4), 1)

    def test_even_runs(self):
        # SKP Ordered Sets (2 Symbols) at any Symbol position, single or in pairs, runs spanning
        # two words: no odd run (per lane SKP counts may differ, always by whole Ordered Sets).
        D, S = self.D, self.SKP_
        for pos in range(4):
            for n in [2, 4, 6]:
                with self.subTest(pos=pos, n=n):
                    symbols = [D]*(8 + pos) + [S]*n + [D]*(16 - pos)
                    symbols += [D]*((-len(symbols)) % 4)
                    self.assertEqual(self.run_check(symbols), 0)

    def test_odd_runs(self):
        # A SKP Symbol with an error (substituted SUB, kept by the SKP removal) at any place of the
        # run, a Symbol received as a SKP: one odd run.
        D, S, E = self.D, self.SKP_, self.SUB_
        cases = {
            "first" : [E, S, S, S],
            "last"  : [S, S, S, E],
            "middle": [S, E, S, S],
            "single": [E, S],
            "phantom": [S],
        }
        for name, run in cases.items():
            for pos in range(4):
                with self.subTest(name=name, pos=pos):
                    symbols = [D]*(8 + pos) + run + [D]*(16 - pos)
                    symbols += [D]*((-len(symbols)) % 4)
                    self.assertEqual(self.run_check(symbols), 1)

if __name__ == "__main__":
    unittest.main()
