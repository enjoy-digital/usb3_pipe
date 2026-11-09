#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.gen import *

from usb3_pipe.serdes_ecp5 import ECP5DCUPLL, ECP5TXGearbox, ECP5RXGearbox, ECP5RXCommaAligner

# 10-bit symbols (LSB first): K28.5 (RD-/RD+) and comma-free data symbols (D21.5, D10.2).
K28_5   = [0x17c, 0x283]
SYMBOLS = [0x2aa, 0x155]

CLOCKS = {"full": 4, "half": 8, "sys": 8}

def bits_of(words, width):
    return [(w >> i) & 1 for w in words for i in range(width)]

def words_of(bits, width):
    return [sum(b << i for i, b in enumerate(bits[n:n + width])) for n in range(0, len(bits) - width + 1, width)]

class TestECP5DCUPLL(unittest.TestCase):
    def test_configs(self):
        pll = ECP5DCUPLL(None, refclk_freq=250e6, linerate=5e9)
        self.assertEqual((pll.m, pll.d), (20, 1))
        pll = ECP5DCUPLL(None, refclk_freq=100e6, linerate=2.5e9)
        self.assertEqual((pll.m, pll.d), (25, 1))
        with self.assertRaises(ValueError):
            ECP5DCUPLL(None, refclk_freq=100e6, linerate=5e9) # x25 max: 2.5Gbps.

class TestECP5TXGearbox(unittest.TestCase):
    def test_order(self):
        dut = ECP5TXGearbox()
        words = [random.getrandbits(40) for _ in range(64)]
        out   = []
        def half_gen():
            for w in words:
                yield dut.sink.eq(w)
                yield
        def full_gen():
            for _ in range(2*len(words) + 8):
                out.append((yield dut.source))
                yield
        run_simulation(dut, {"half": half_gen(), "full": full_gen()}, clocks=CLOCKS)
        # 20-bit words: LSBs then MSBs of each 40-bit word, in order (after the gearbox latency).
        expected = [h for w in words for h in [w & 0xfffff, w >> 20]]
        stream   = "".join(f"{w:05x}" for w in out)
        self.assertEqual(len(out), 2*len(words) + 8)
        self.assertIn("".join(f"{w:05x}" for w in expected[:100]), stream)

class RXPath(LiteXModule):
    def __init__(self):
        self.gearbox = ECP5RXGearbox()
        self.aligner = ClockDomainsRenamer("half")(ECP5RXCommaAligner())
        self.comb += self.aligner.sink.eq(self.gearbox.source)

class TestECP5RXPath(unittest.TestCase):
    def check_offset(self, offset):
        # Symbols stream: Ordered Sets like (4 COMs) and data, shifted by offset bits.
        symbols = []
        rd = 0
        for _ in range(48):
            for _ in range(4):
                symbols.append(K28_5[rd]); rd ^= 1
            symbols += [random.choice(SYMBOLS) for _ in range(12)]
        bits = [random.getrandbits(1) for _ in range(offset)] + bits_of(symbols, 10)
        rx_words = words_of(bits, 20)

        dut = RXPath()
        out = []
        def full_gen():
            for w in rx_words:
                yield dut.gearbox.sink.eq(w)
                yield
        def half_gen():
            for _ in range(len(rx_words)//2):
                out.append((yield dut.aligner.source))
                yield
        run_simulation(dut, {"full": full_gen(), "half": half_gen()}, clocks=CLOCKS)

        # After the alignment (first Ordered Sets), the output symbols are the transmitted ones.
        rx_symbols = words_of(bits_of(out[len(out)//4:], 40), 10)
        expected   = "".join(f"{s:03x}" for s in symbols)
        received   = "".join(f"{s:03x}" for s in rx_symbols[:len(rx_symbols) - 8])
        self.assertGreater(len(rx_symbols), len(symbols)//2)
        self.assertIn(received, expected)

    def test_offsets(self):
        random.seed(0)
        for offset in range(40):
            with self.subTest(offset=offset):
                self.check_offset(offset)

if __name__ == "__main__":
    unittest.main()
