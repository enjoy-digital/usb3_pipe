#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.gen import *

from usb3_pipe.lanes import *

MARKER = (0xbcbcbcbc, 0b1111) # COM x4.

class DUT(LiteXModule):
    def __init__(self):
        self.striper   = LaneStriper()
        self.deskew    = LaneDeskew(*MARKER)
        self.destriper = LaneDestriper()
        for lane in range(2):
            self.comb += self.deskew.sources[lane].connect(self.destriper.sinks[lane])

class TestLanes(unittest.TestCase):
    def test_striping(self):
        # Even symbols on Lane 0, odd symbols on Lane 1 (aligned to Lane 0).
        dut = LaneStriper()
        def generator(dut):
            yield dut.sink.valid.eq(1)
            yield dut.sink.data.eq(0x0706050403020100)
            yield dut.sink.ctrl.eq(0b00000101)
            yield dut.sources[0].ready.eq(1)
            yield dut.sources[1].ready.eq(1)
            yield
            self.assertEqual((yield dut.sources[0].data), 0x06040200)
            self.assertEqual((yield dut.sources[1].data), 0x07050301)
            self.assertEqual((yield dut.sources[0].ctrl), 0b0011)
            self.assertEqual((yield dut.sources[1].ctrl), 0b0000)
        run_simulation(dut, generator(dut))

    def test_stripe_skew_deskew_destripe(self):
        # Link words striped, lanes skewed (Lane 1 delayed by 3 words, different garbage before the
        # marker), deskewed on the marker, destriped: link words recovered.
        rng   = random.Random(0)
        words = [(rng.getrandbits(64), rng.getrandbits(8)) for _ in range(64)]
        # Marker on both lanes (as duplicated Ordered Sets): striped from a link word.
        marker_word = (0xbcbcbcbcbcbcbcbc, 0xff)
        striper = LaneStriper()
        lanes = [[], []]
        def strip_gen(dut):
            for lane in range(2):
                yield dut.sources[lane].ready.eq(1)
            for data, ctrl in [marker_word] + words:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
                for lane in range(2):
                    lanes[lane].append(((yield dut.sources[lane].data), (yield dut.sources[lane].ctrl)))
        run_simulation(striper, strip_gen(striper))
        lanes[0] = [(rng.getrandbits(32), 0) for _ in range(1)] + lanes[0]
        lanes[1] = [(rng.getrandbits(32), 0) for _ in range(4)] + lanes[1]

        dut = DUT()
        out = []
        def lane_gen(dut, lane):
            for data, ctrl in lanes[lane]:
                yield dut.deskew.sinks[lane].valid.eq(1)
                yield dut.deskew.sinks[lane].data.eq(data)
                yield dut.deskew.sinks[lane].ctrl.eq(ctrl)
                yield
                while not (yield dut.deskew.sinks[lane].ready):
                    yield
            yield dut.deskew.sinks[lane].valid.eq(0)
        def checker(dut):
            yield dut.destriper.source.ready.eq(1)
            for _ in range(300):
                if (yield dut.destriper.source.valid):
                    out.append(((yield dut.destriper.source.data), (yield dut.destriper.source.ctrl)))
                yield
        run_simulation(dut, [lane_gen(dut, 0), lane_gen(dut, 1), checker(dut)])
        self.assertEqual(out, [marker_word] + words)

    def test_deskew_periodic_marker(self):
        # Periodic marker (TS1/TS2 COM words, every 4 words), lanes skewed by up to 1 word in either
        # direction, deskew started at any point of the period: the markers of the same Ordered Set are
        # paired (max_skew=1), the link words are recovered.
        rng    = random.Random(1)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        stream = []
        for _ in range(8):
            stream += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
        stream += [(rng.getrandbits(64), rng.getrandbits(8)) for _ in range(32)]
        def stripe(word, lane):
            data, ctrl = word
            return (sum(((data >> (8*(2*i + lane))) & 0xff) << (8*i) for i in range(4)),
                    sum(((ctrl >> (2*i + lane)) & 0x1) << i for i in range(4)))
        for start in range(4, 8):
            for skew in [-1, 0, 1]:
                # Lane late by skew words: it still holds skew older words.
                lanes = [
                    [stripe(w, 0) for w in stream[start - max(0, -skew):]],
                    [stripe(w, 1) for w in stream[start - max(0,  skew):]],
                ]
                class DUT(LiteXModule):
                    def __init__(self):
                        self.deskew    = LaneDeskew(0xbcbcbcbc, 0b1111, max_skew=1)
                        self.destriper = LaneDestriper()
                        for lane in range(2):
                            self.comb += self.deskew.sources[lane].connect(self.destriper.sinks[lane])
                dut = DUT()
                out = []
                def lane_gen(dut, lane):
                    for data, ctrl in lanes[lane]:
                        yield dut.deskew.sinks[lane].valid.eq(1)
                        yield dut.deskew.sinks[lane].data.eq(data)
                        yield dut.deskew.sinks[lane].ctrl.eq(ctrl)
                        yield
                        while not (yield dut.deskew.sinks[lane].ready):
                            yield
                    yield dut.deskew.sinks[lane].valid.eq(0)
                def checker(dut):
                    yield dut.destriper.source.ready.eq(1)
                    for _ in range(200):
                        if (yield dut.destriper.source.valid):
                            out.append(((yield dut.destriper.source.data), (yield dut.destriper.source.ctrl)))
                        yield
                run_simulation(dut, [lane_gen(dut, 0), lane_gen(dut, 1), checker(dut)])
                self.assertTrue(len(out), f"start={start} skew={skew}")
                self.assertEqual(out[0], marker)
                first = stream.index(marker, start)
                self.assertIn(out, [stream[m:m + len(out)] for m in range(first - 4, first + 5)])
                self.assertGreater(len(out), len(stream) - start - 8)

if __name__ == "__main__":
    unittest.main()
