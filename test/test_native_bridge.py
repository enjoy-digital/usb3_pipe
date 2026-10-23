#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_core.bridge import USB3ByteUnpacker, USB3BytePacker

def to_words(data):
    """Bytes -> (data, last, last_be) words of a packet."""
    words = []
    for i in range(0, len(data), 4):
        chunk = data[i:i+4]
        last  = (i + 4 >= len(data))
        be    = (1 << (len(chunk) - 1)) if (last and len(chunk) < 4) else (0b1000 if last else 0)
        words.append((int.from_bytes(bytes(chunk + [0]*(4 - len(chunk))), "little"), last, be))
    return words

class TestNativeBridge(unittest.TestCase):
    def test_unpacker(self):
        rng     = random.Random(1)
        packets = [[rng.randrange(256) for _ in range(n)] for n in [1, 2, 3, 4, 5, 7, 8, 13]]
        dut     = USB3ByteUnpacker()
        out     = []
        def generator(dut):
            for p in packets:
                for data, last, be in to_words(p):
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(data)
                    yield dut.sink.last.eq(last)
                    yield dut.sink.last_be.eq(be)
                    yield
                    while not (yield dut.sink.ready):
                        yield
            yield dut.sink.valid.eq(0)
        def checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(200):
                if (yield dut.source.valid):
                    out.append((yield dut.source.data))
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(out, sum(packets, []))

    def test_packer(self):
        # Bursts of bytes separated by idle: one packet per burst.
        rng    = random.Random(2)
        bursts = [[rng.randrange(256) for _ in range(n)] for n in [1, 4, 5, 8, 11, 2]]
        dut    = USB3BytePacker(idle_cycles=8)
        packets, current = [], []
        def generator(dut):
            for b in bursts:
                for byte in b:
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(byte)
                    yield
                    while not (yield dut.sink.ready):
                        yield
                yield dut.sink.valid.eq(0)
                for _ in range(32):
                    yield
        def checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(600):
                if (yield dut.source.valid):
                    data = (yield dut.source.data)
                    be   = (yield dut.source.last_be)
                    n    = 4
                    if (yield dut.source.last) and be:
                        n = {1: 1, 2: 2, 4: 3, 8: 4}[be]
                    current.extend(list(data.to_bytes(4, "little"))[:n])
                    if (yield dut.source.last):
                        packets.append(list(current))
                        current.clear()
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(packets, bursts)

if __name__ == "__main__":
    unittest.main()
