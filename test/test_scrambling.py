#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.common import COM, SKP, gen2_skp_lfsr_symbols
from usb3_pipe.scrambling import Scrambler, Descrambler
from usb3_pipe.scrambling import GEN2_LFSR_SEED, gen2_lfsr_advance, Gen2Scrambler

from test.common import ScramblerModel, symbols_to_words
from test.model.gen2 import figure_6_12_step, Gen2ScramblerModel

scrambler_ref = [
    0x14c017ff, 0x8202e7b2, 0xa6286e72, 0x8dbf6dbe,
    0xe6a740be, 0xb2e2d32c, 0x2a770207, 0xe0be34cd,
    0xb1245da7, 0x22bda19b, 0xd31d45d4, 0xee76ead7,
    0xfa1ada2c, 0x3b362d28, 0x676f0e3a, 0x264c06cf,
    0xcd3ae9d3, 0xfc307627, 0xde038b94, 0xf65206d3,
    0x9580884f, 0xf2666ac4, 0x35a10c9f, 0x27cf41e2,
    0x9e7e4074, 0x84fe58a5, 0xa9086009, 0x626f0bf1,
    0xed5c4317, 0xd43f3948, 0xb30ef55a, 0x9b9d03c7,
    0x5c8e0d8b, 0xae779833, 0x3e0bac2d, 0x7a420bda,
    0xa8cfd17c, 0x41ee121c, 0x7a383fc2, 0x01f4690d,
    0xc57231da, 0x0e93d7a0, 0x55a4afdc, 0x1672f0e7,
    0x8438d568, 0x18cd00dd, 0x5930ca9e, 0x771b754c,
    0xcfedc531, 0x3d6e6491, 0x0429e8fe, 0xc4fc6ccf,
    0x62da5e0b, 0xdfab5bba, 0x377db759, 0xc61ae35e,
    0x4ff51488, 0xcb56c88b, 0x634210d3, 0xf7b48a04,
    0x01a00184, 0xee674983, 0xa48b2a3e, 0xd514af76
]

class TestScrambling(unittest.TestCase):
    def test_reference_model_matches_table(self):
        # The golden table is the Gen1 LFSR keystream (scrambled zeros) from FFFFh (6.3.1.3).
        symbols = ScramblerModel().process([(0x00, 0)]*4*len(scrambler_ref))
        self.assertEqual([data for data, ctrl in symbols_to_words(symbols)], scrambler_ref)

    def test_reference_model_tx_seed_after_ts2(self):
        # Scrambler(reset=0x7dbd) default: LFSR state after a TS2 (4 COM + 12 D symbols).
        model = ScramblerModel()
        model.process([(COM.value, 1)]*4 + [(0x00, 0)]*12)
        self.assertEqual(model.lfsr, 0x7dbd)

    def test_scrambler_data(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield
            yield dut.sink.data.eq(0)
            yield dut.sink.valid.eq(1)
            yield
            for i in range(64):
                self.assertEqual((yield dut.source.data), scrambler_ref[i])
                yield

        dut = Scrambler(reset=0xffff)
        run_simulation(dut, generator(dut))

    def test_scrambler_ctrl(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield
            yield dut.sink.data.eq(0)
            yield dut.sink.ctrl.eq(0b1111)
            yield dut.sink.valid.eq(1)
            yield
            for i in range(64):
                self.assertEqual((yield dut.source.data), 0)
                yield

        dut = Scrambler(reset=0xffff)
        run_simulation(dut, generator(dut))

    def test_scrambler_backpressure(self):
        def generator(dut):
            yield dut.sink.data.eq(0)
            yield dut.sink.ctrl.eq(0b0000)
            yield dut.sink.valid.eq(1)
            yield
            # Toggle ready to create stalls.
            for i in range(256):
                yield dut.source.ready.eq(i & 0x1)
                yield
            yield dut.sink.valid.eq(0)
            for i in range(16):
                yield

        def checker(dut):
            k = 0
            yield
            while k < 64:
                if (yield dut.sink.valid) and (yield dut.sink.ready):
                    self.assertEqual((yield dut.source.data), scrambler_ref[k])
                    k += 1
                yield
            self.assertEqual(k, 64)

        dut = Scrambler(reset=0xffff)
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_scrambler_mixed_ctrl(self):
        def apply_expected(data, mask, ctrl):
            r = 0
            for i in range(4):
                b = (data >> (8*i)) & 0xff
                m = (mask >> (8*i)) & 0xff
                if (ctrl >> i) & 1:
                    o = b
                else:
                    o = b ^ m
                r |= (o << (8*i))
            return r

        ctrl_pat = [0b0000, 0b0001, 0b0010, 0b0100, 0b1000, 0b0101, 0b1010, 0b1111]

        def generator(dut):
            yield dut.source.ready.eq(1)
            yield dut.enable.eq(1)
            yield dut.sink.data.eq(0x11223344)
            yield dut.sink.ctrl.eq(ctrl_pat[0])
            yield dut.sink.valid.eq(1)
            yield
            for i in range(1, len(ctrl_pat)):
                yield dut.sink.data.eq(0x11223344 + i)
                yield dut.sink.ctrl.eq(ctrl_pat[i])
                yield
            yield dut.sink.valid.eq(0)
            for i in range(16):
                yield
            dut.run = False

        def checker(dut):
            k = 0
            while dut.run:
                if (yield dut.source.valid) and (yield dut.source.ready):
                    data = 0x11223344 + k
                    ctrl = ctrl_pat[k]
                    expected = apply_expected(data, scrambler_ref[k], ctrl)
                    self.assertEqual((yield dut.source.data), expected)
                    k += 1
                yield
            self.assertEqual(k, len(ctrl_pat))

        dut = Scrambler(reset=0xffff)
        dut.run = True
        run_simulation(dut, [generator(dut), checker(dut)])


    def test_descrambler_data(self):
        def generator(dut):
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0b1111)
                yield dut.sink.data.eq(0xbcbcbcbc)
                yield
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0b0000)
                yield dut.sink.data.eq(scrambler_ref[i])
                yield
            yield dut.sink.valid.eq(0)
            for i in range(64):
                yield

        def checker(dut):
            yield dut.source.ready.eq(1)
            yield
            while (yield dut.source.ctrl == 0b1111):
                yield
            for i in range(16):
                self.assertEqual((yield dut.source.data), 0)
                yield

        dut = Descrambler(reset=0xffff)
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_descrambler_com_resync_per_lane(self):
        def generator(dut, lane):
            # Create some activity first.
            for i in range(8):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0b0000)
                yield dut.sink.data.eq(scrambler_ref[i])
                yield

            # Emit a K-word with only one COM lane (others SKP).
            data = 0
            for i in range(4):
                v = COM.value if i == lane else SKP.value
                data |= (v << (8*i))
            yield dut.sink.valid.eq(1)
            yield dut.sink.ctrl.eq(0b1111)
            yield dut.sink.data.eq(data)
            yield

            # After COM reset, feed the reference scrambler stream (scrambling of zeros).
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0b0000)
                yield dut.sink.data.eq(scrambler_ref[i])
                yield

            yield dut.sink.valid.eq(0)
            for i in range(32):
                yield

        def checker(dut):
            yield dut.source.ready.eq(1)
            yield
            # Skip initial stuff until we see the COM word.
            while not ((yield dut.source.valid) and ((yield dut.source.ctrl) == 0b1111)):
                yield
            # Next 16 data words should descramble to zero.
            for i in range(16):
                yield
                self.assertEqual((yield dut.source.data), 0)

        for lane in range(4):
            dut = Descrambler(reset=0xffff)
            run_simulation(dut, [generator(dut, lane), checker(dut)])

    def test_scrambler_descrambler_roundtrip(self):
        class DUT(Module):
            def __init__(self):
                self.submodules.scr = Scrambler(reset=0xffff)
                self.submodules.des = Descrambler(reset=0xffff)
                self.comb += self.scr.source.connect(self.des.sink)
                self.sink   = self.scr.sink
                self.source = self.des.source

        def generator(dut):
            yield dut.source.ready.eq(1)
            yield dut.sink.ctrl.eq(0b0000)
            yield dut.sink.data.eq(0x01020300)
            yield dut.sink.valid.eq(1)
            yield
            for i in range(1, 64):
                yield dut.sink.data.eq(0x01020300 + i)
                yield
            yield dut.sink.valid.eq(0)
            for i in range(16):
                yield
            dut.run = False

        def checker(dut):
            i = 0
            while dut.run:
                if (yield dut.source.valid) and (yield dut.source.ready):
                    self.assertEqual((yield dut.source.data), 0x01020300 + i)
                    i += 1
                yield
            self.assertEqual(i, 64)

        dut = DUT()
        dut.run = True
        run_simulation(dut, [generator(dut), checker(dut)])


    def test_scrambler_enable_passthrough(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield dut.enable.eq(0)
            yield dut.sink.ctrl.eq(0b0000)
            yield dut.sink.data.eq(0x11223344)
            yield dut.sink.valid.eq(1)
            yield
            for i in range(1, 32):
                yield dut.sink.data.eq(0x11223344 + i)
                yield
            yield dut.sink.valid.eq(0)
            for i in range(16):
                yield
            dut.run = False

        def checker(dut):
            i = 0
            while dut.run:
                if (yield dut.source.valid) and (yield dut.source.ready):
                    self.assertEqual((yield dut.source.data), 0x11223344 + i)
                    i += 1
                yield
            self.assertEqual(i, 32)

        dut = Scrambler(reset=0xffff)
        dut.run = True
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_descrambler_com_resync_multi_lane(self):
        # 1, 2, 3, 4 COM symbols ending the word: the LFSR is initialized after the last COM and
        # advances on the D symbols before it (6.3.1.3).
        com_patterns = [0b1000, 0b1100, 0b1110, 0b1111]
        for ctrl in com_patterns:
            with self.subTest(ctrl=bin(ctrl)):
                def generator(dut):
                    # Initial scrambled data to get out of reset state
                    for i in range(8):
                        yield dut.sink.valid.eq(1)
                        yield dut.sink.ctrl.eq(0)
                        yield dut.sink.data.eq(scrambler_ref[i])
                        yield
                    # Emit COM word with selected lanes
                    data = 0
                    for i in range(4):
                        if (ctrl >> i) & 1:
                            data |= (COM.value << (8 * i))
                        else:
                            data |= (SKP.value << (8 * i))
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.ctrl.eq(ctrl)
                    yield dut.sink.data.eq(data)
                    yield
                    # Feed fresh scrambled zeros after resync
                    for i in range(16):
                        yield dut.sink.valid.eq(1)
                        yield dut.sink.ctrl.eq(0)
                        yield dut.sink.data.eq(scrambler_ref[i])
                        yield
                    yield dut.sink.valid.eq(0)

                def checker(dut):
                    yield dut.source.ready.eq(1)
                    yield
                    # Skip initial 8 data + 1 COM word
                    for _ in range(9):
                        while not ((yield dut.source.valid) and (yield dut.source.ready)):
                            yield
                        yield
                    # Next 16 words should descramble to zero
                    for _ in range(16):
                        while not ((yield dut.source.valid) and (yield dut.source.ready)):
                            yield
                        self.assertEqual((yield dut.source.data), 0)
                        yield

                dut = Descrambler(reset=0xffff)
                run_simulation(dut, [generator(dut), checker(dut)])

    def test_scrambler_enable_toggle_midstream(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield dut.enable.eq(1)
            base = 0xA5A5A500
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.data.eq(base + i)
                yield
            yield dut.enable.eq(0)  # disable scrambling
            for i in range(16, 32):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.data.eq(base + i)
                yield
            yield dut.enable.eq(1)  # re-enable
            for i in range(32, 48):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.data.eq(base + i)
                yield
            yield dut.sink.valid.eq(0)

        def checker(dut):
            yield dut.source.ready.eq(1)
            base = 0xA5A5A500
            for i in range(16):
                while not ((yield dut.source.valid) and (yield dut.source.ready)):
                    yield
                self.assertEqual((yield dut.source.data), (base + i) ^ scrambler_ref[i])
                yield
            for i in range(16, 32):
                while not ((yield dut.source.valid) and (yield dut.source.ready)):
                    yield
                self.assertEqual((yield dut.source.data), base + i)  # passthrough
                yield
            # LFSR continues from previous state
            for i in range(32, 48):
                while not ((yield dut.source.valid) and (yield dut.source.ready)):
                    yield
                self.assertEqual((yield dut.source.data), (base + i) ^ scrambler_ref[i])
                yield

        dut = Scrambler(reset=0xffff)
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_descrambler_recovery_from_bad_state(self):
        def generator(dut):
            # Feed completely wrong (inverted) scrambled data first
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.data.eq(scrambler_ref[i] ^ 0xffffffff)
                yield
            # Send COM on all lanes to force resynchronization
            yield dut.sink.valid.eq(1)
            yield dut.sink.ctrl.eq(0b1111)
            yield dut.sink.data.eq(COM.value * 0x01010101)
            yield
            # Now send correct scrambled zeros
            for i in range(32):
                yield dut.sink.valid.eq(1)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.data.eq(scrambler_ref[i])
                yield
            yield dut.sink.valid.eq(0)

        def checker(dut):
            yield dut.source.ready.eq(1)
            yield
            # First 16 outputs + COM word may be garbage — skip them
            for _ in range(17):
                while not ((yield dut.source.valid) and (yield dut.source.ready)):
                    yield
                yield
            # After resync, should get clean zeros
            for _ in range(32):
                while not ((yield dut.source.valid) and (yield dut.source.ready)):
                    yield
                self.assertEqual((yield dut.source.data), 0)
                yield

        dut = Descrambler(reset=0xffff)
        run_simulation(dut, [generator(dut), checker(dut)])

# Gen 2 (6.3.2.3) ----------------------------------------------------------------------------------

class TestGen2Scrambling(unittest.TestCase):
    def test_lfsr_figure_6_12(self):
        # The polynomial/taps used by the hardware and gen2_lfsr_advance are the Figure 6-12 LFSR.
        rng = random.Random(0)
        for _ in range(100):
            lfsr  = rng.getrandbits(23)
            masks = gen2_lfsr_advance([1 << n for n in range(23)])
            hw    = sum((bin(lfsr & m).count("1") & 0x1) << i for i, m in enumerate(masks))
            out, ref = figure_6_12_step(lfsr)
            self.assertEqual(hw,  ref)
            self.assertEqual(out, (lfsr >> 22) & 0x1)
        # Maximal length: 2^23 - 1 period (primitive polynomial), sampled: no return before.
        lfsr = GEN2_LFSR_SEED
        for _ in range(4096):
            _, lfsr = figure_6_12_step(lfsr)
            self.assertNotEqual(lfsr, GEN2_LFSR_SEED)

    def test_skp_lfsr_symbols(self):
        # Table 6-13: 4N+1 = {~LFSR[22], LFSR[22:16]} (40h-FFh), LFSR[15:8], LFSR[7:0].
        self.assertEqual(gen2_skp_lfsr_symbols(0x400000), [0x40, 0x00, 0x00])
        self.assertEqual(gen2_skp_lfsr_symbols(0x000000), [0x80, 0x00, 0x00])
        self.assertEqual(gen2_skp_lfsr_symbols(GEN2_LFSR_SEED), [0x80 | 0x1d, 0xbf, 0xbc])
        for lfsr in range(0, 2**23, 4099):
            self.assertTrue(0x40 <= gen2_skp_lfsr_symbols(lfsr)[0] <= 0xff)

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
