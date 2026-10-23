#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.gen2 import *

# Reference Models ---------------------------------------------------------------------------------

class Gen2ScramblerModel:
    """Bit-serial Gen 2 LFSR (Galois, output = LFSR[22], LSB first)."""
    def __init__(self, seed=GEN2_LFSR_SEED):
        self.lfsr = seed

    def byte(self, value, advance=True, scramble=True):
        if not advance:
            return value
        out = 0
        for i in range(8):
            bit = (self.lfsr >> 22) & 0x1
            out |= (((value >> i) & 0x1) ^ (bit if scramble else 0)) << i
            self.lfsr = (self.lfsr << 1) & 0x7fffff
            if bit:
                self.lfsr ^= GEN2_LFSR_TAPS
        return out

def bits_from_words(words):
    return [(w >> i) & 0x1 for w in words for i in range(32)]

def words_from_bits(bits):
    bits = bits + [0]*((-len(bits)) % 32)
    return [sum(bits[32*i + j] << j for j in range(32)) for i in range(len(bits)//32)]

# Tests --------------------------------------------------------------------------------------------

class TestGen2(unittest.TestCase):
    def test_scrambler_vs_model(self):
        rng   = random.Random(0)
        dut   = Gen2Scrambler(nbytes=4)
        model = Gen2ScramblerModel()
        def generator(dut):
            for _ in range(200):
                data     = [rng.randrange(256) for _ in range(4)]
                advance  = [rng.random() < 0.9 for _ in range(4)]
                scramble = [rng.random() < 0.8 for _ in range(4)]
                yield dut.ce.eq(1)
                yield dut.i.eq(int.from_bytes(bytes(data), "little"))
                yield dut.advance.eq(sum(a << i for i, a in enumerate(advance)))
                yield dut.scramble.eq(sum(s << i for i, s in enumerate(scramble)))
                yield
                out = (yield dut.o)
                ref = [model.byte(d, a, s) for d, a, s in zip(data, advance, scramble)]
                self.assertEqual(out, int.from_bytes(bytes(ref), "little"))
        run_simulation(dut, generator(dut))

    def test_scrambler_init(self):
        # init: LFSR re-initialized to the seed (SYNC Ordered Set).
        dut = Gen2Scrambler(nbytes=4)
        def generator(dut):
            yield dut.ce.eq(1)
            yield dut.advance.eq(0b1111)
            yield dut.scramble.eq(0b1111)
            yield
            first = (yield dut.o)
            for _ in range(10):
                yield
            yield dut.init.eq(1)
            yield
            yield dut.init.eq(0)
            yield
            self.assertEqual((yield dut.o), first)
        run_simulation(dut, generator(dut))

    def test_gearbox_aligner_roundtrip(self):
        # Blocks -> gearbox -> bit offset -> aligner: blocks recovered once locked, header single bit
        # error corrected.
        rng    = random.Random(1)
        blocks = [(rng.choice([BLOCK_HEADER_DATA, BLOCK_HEADER_CONTROL]), rng.getrandbits(128))
            for _ in range(64)]

        # TX gearbox.
        tx = Gen2BlockGearbox()
        words = []
        def tx_generator(dut):
            for header, payload in blocks:
                yield dut.sink.valid.eq(1)
                yield dut.sink.header.eq(header)
                yield dut.sink.data.eq(payload)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(16):
                yield
        def tx_checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(600):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield
        run_simulation(tx, [tx_generator(tx), tx_checker(tx)])
        bits = bits_from_words(words)
        self.assertEqual(bits[:132*len(blocks)],
            [b for h, p in blocks for b in [(h >> i) & 1 for i in range(4)] + [(p >> i) & 1 for i in range(128)]])

        # Channel: random bit offset, a Block Header single bit error.
        offset = 37
        bits   = [rng.getrandbits(1) for _ in range(offset)] + bits[:132*len(blocks)]
        bits[offset + 132*40 + 1] ^= 1

        # RX aligner.
        rx  = Gen2BlockAligner()
        out = []
        def rx_generator(dut):
            for w in words_from_bits(bits):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(w)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(16):
                yield
        def rx_checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(1000):
                if (yield dut.source.valid):
                    out.append(((yield dut.source.header), (yield dut.source.data),
                        (yield dut.source.header_error)))
                yield
        run_simulation(rx, [rx_generator(rx), rx_checker(rx)])
        received = [(h, p) for h, p, e in out]
        # Blocks used for the alignment are not output: the received blocks are the last ones.
        self.assertGreater(len(received), 40)
        self.assertEqual(received, blocks[len(blocks) - len(received):])
        errors = [i for i, (h, p, e) in enumerate(out) if e]
        self.assertEqual([len(blocks) - len(received) + i for i in errors], [40])

if __name__ == "__main__":
    unittest.main()
