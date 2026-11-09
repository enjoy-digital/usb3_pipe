#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 endpoint user stream converters (usb3_core/bulk.py): 32-bit <-> 64/128-bit words, packets
of any length (be: valid bytes), random back-pressure."""

import random
import unittest

from migen import *

from litex.gen import *

from usb3_core.bulk import USB3EndpointUpConverter, USB3EndpointDownConverter

# Helpers ------------------------------------------------------------------------------------------

def words(packet, wb):
    """Packet bytes -> (data, be, last) words of wb bytes."""
    r = []
    for i in range(0, len(packet), wb):
        chunk = packet[i:i+wb]
        data  = int.from_bytes(bytes(chunk), "little")
        be    = 2**len(chunk) - 1
        r.append((data, be, i + wb >= len(packet)))
    return r

class DUT(LiteXModule):
    def __init__(self, dw):
        self.up   = USB3EndpointUpConverter(dw)
        self.down = USB3EndpointDownConverter(dw)
        self.comb += self.up.source.connect(self.down.sink)

def run(dw, packets, seed=0):
    dut   = DUT(dw)
    prng  = random.Random(seed)
    sent  = [w for p in packets for w in words(p, 4)]
    wide  = []
    recv  = []
    def source():
        for data, be, last in sent:
            yield dut.up.sink.valid.eq(1)
            yield dut.up.sink.data.eq(data)
            yield dut.up.sink.be.eq(be)
            yield dut.up.sink.last.eq(last)
            yield
            while not (yield dut.up.sink.ready):
                yield
            yield dut.up.sink.valid.eq(0)
            for _ in range(prng.randint(0, 2)):
                yield
    def sink():
        while len(recv) < len(sent):
            yield dut.down.source.ready.eq(prng.random() > 0.3)
            yield
            if (yield dut.down.source.valid) and (yield dut.down.source.ready):
                recv.append(((yield dut.down.source.data), (yield dut.down.source.be),
                    bool((yield dut.down.source.last))))
            if (yield dut.up.source.valid) and (yield dut.up.source.ready):
                wide.append(((yield dut.up.source.be), bool((yield dut.up.source.last))))
    run_simulation(dut, [source(), sink()])
    return sent, wide, recv

# Tests --------------------------------------------------------------------------------------------

class TestEndpointConverters(unittest.TestCase):
    def test_roundtrip(self):
        # 32-bit -> dw-bit -> 32-bit: same words, packets of 1..64 bytes.
        prng = random.Random(42)
        for dw in [64, 128]:
            with self.subTest(dw=dw):
                packets = [[prng.randrange(256) for _ in range(prng.randint(1, 64))]
                    for _ in range(40)]
                sent, wide, recv = run(dw, packets)
                self.assertEqual(recv, [(d, be, last) for d, be, last in sent])
                # Wide words: dw//8 bytes, last word of a packet with the remaining bytes.
                self.assertEqual(wide, [(be, last) for p in packets
                    for _, be, last in words(p, dw//8)])

if __name__ == "__main__":
    unittest.main()
