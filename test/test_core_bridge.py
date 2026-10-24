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
    """Bytes -> (data, last, be) words of a packet."""
    words = []
    for i in range(0, len(data), 4):
        chunk = data[i:i+4]
        last  = (i + 4 >= len(data))
        be    = (1 << len(chunk)) - 1
        words.append((int.from_bytes(bytes(chunk + [0]*(4 - len(chunk))), "little"), last, be))
    return words

class TestCoreBridge(unittest.TestCase):
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
                    yield dut.sink.be.eq(be)
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
                    be   = (yield dut.source.be)
                    n    = {0b0001: 1, 0b0011: 2, 0b0111: 3, 0b1111: 4}[be]
                    self.assertTrue((yield dut.source.last) or (n == 4))
                    current.extend(list(data.to_bytes(4, "little"))[:n])
                    if (yield dut.source.last):
                        packets.append(list(current))
                        current.clear()
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(packets, bursts)

    def test_packer_backpressure(self):
        # Random back-pressure (endpoint not ready) and bytes arriving right after the idle flush:
        # the byte stream is preserved (found on hardware: bytes duplicated at the partial words).
        rng    = random.Random(3)
        data   = [rng.randrange(256) for _ in range(400)]
        dut    = USB3BytePacker(idle_cycles=4)
        out    = []
        def generator(dut):
            for byte in data:
                for _ in range(rng.choice([0, 0, 0, 3, 5, 8])):
                    yield dut.sink.valid.eq(0)
                    yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(byte)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
        def checker(dut):
            for _ in range(8000):
                ready = rng.random() < 0.5
                yield dut.source.ready.eq(int(ready))
                yield
                if ready and (yield dut.source.valid):
                    word = (yield dut.source.data)
                    be   = (yield dut.source.be)
                    n    = {0b0001: 1, 0b0011: 2, 0b0111: 3, 0b1111: 4}[be]
                    self.assertTrue((yield dut.source.last) or (n == 4))
                    out.extend(list(word.to_bytes(4, "little"))[:n])
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(out, data)

if __name__ == "__main__":
    unittest.main()
