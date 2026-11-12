#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

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

    def test_striping_lockstep(self):
        # Lanes ready randomly: a word is given to both lanes in the same cycle, never to a single
        # lane (a lane held must hold the other one, the lanes would get out of step).
        rng   = random.Random(1)
        words = [rng.getrandbits(64) for _ in range(64)]
        dut   = LaneStriper()
        got   = [[], []]
        def generator(dut):
            yield dut.sink.valid.eq(1)
            i = 0
            while i < len(words):
                yield dut.sink.data.eq(words[i])
                for lane in range(2):
                    yield dut.sources[lane].ready.eq(rng.random() < 0.7)
                yield
                valid = []
                for lane in range(2):
                    valid.append((yield dut.sources[lane].valid) & (yield dut.sources[lane].ready))
                self.assertEqual(valid[0], valid[1])
                for lane in range(2):
                    if valid[lane]:
                        got[lane].append((yield dut.sources[lane].data))
                if (yield dut.sink.ready):
                    i += 1
        run_simulation(dut, generator(dut))
        for lane in range(2):
            self.assertEqual(got[lane], [
                sum(((w >> (8*(2*k + lane))) & 0xff) << (8*k) for k in range(4)) for w in words])

    def test_stripe_skew_deskew_destripe(self):
        # Link words striped, lanes skewed (Lane 1 delayed by 3 words, different garbage before the
        # marker, longer than the deskew gap), deskewed on the marker, destriped: link words recovered.
        rng   = random.Random(0)
        words = [(rng.getrandbits(64), rng.getrandbits(8)) for _ in range(64)]
        # Marker on both lanes (as duplicated Ordered Sets): striped from a link word.
        marker_word = (0xbcbcbcbcbcbcbcbc, 0xff)
        striper = LaneStriper()
        lanes   = [[], []]
        def strip_gen(dut):
            for lane in range(2):
                yield dut.sources[lane].ready.eq(1)
            for data, ctrl in [marker_word] + words:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
                for lane in range(2):
                    lanes[lane].append(((yield dut.sources[lane].data),
                        (yield dut.sources[lane].ctrl)))
        run_simulation(striper, strip_gen(striper))
        lanes[0] = [(rng.getrandbits(32), 0) for _ in range(17)] + lanes[0]
        lanes[1] = [(rng.getrandbits(32), 0) for _ in range(20)] + lanes[1]

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
                    out.append(((yield dut.destriper.source.data),
                        (yield dut.destriper.source.ctrl)))
                yield
        run_simulation(dut, [lane_gen(dut, 0), lane_gen(dut, 1), checker(dut)])
        self.assertEqual(out, [marker_word] + words)

    def run_deskew(self, lanes, cycles=300):
        class DUT(LiteXModule):
            def __init__(self):
                self.deskew    = LaneDeskew(0xbcbcbcbc, 0b1111)
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
            yield dut.deskew.sinks[lane].valid.eq(0)
        def checker(dut):
            yield dut.destriper.source.ready.eq(1)
            for _ in range(cycles):
                if (yield dut.destriper.source.valid):
                    out.append(((yield dut.destriper.source.data), (yield dut.destriper.source.ctrl)))
                yield
        run_simulation(dut, [lane_gen(dut, 0), lane_gen(dut, 1), checker(dut)])
        return out

    @staticmethod
    def stripe(word, lane):
        data, ctrl = word
        return (sum(((data >> (8*(2*i + lane))) & 0xff) << (8*i) for i in range(4)),
                sum(((ctrl >> (2*i + lane)) & 0x1) << i for i in range(4)))

    def test_deskew_first_marker_after_gap(self):
        # Gap without marker (TSEQ), then periodic marker (TS1/TS2 COM words, every 4 words), lanes
        # skewed by up to 3 words in either direction (more than half the marker period): the first
        # markers after the gap are paired, the link words are recovered from the first marker.
        rng    = random.Random(1)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        tseq   = [(rng.getrandbits(64), 0) for _ in range(24)]
        ts     = []
        for _ in range(8):
            ts += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
        data   = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(32)]
        stream = tseq + ts + data
        for skew in [-3, -2, -1, 0, 1, 2, 3]:
            # Lane late by skew words: extra garbage words before its stream.
            lanes = [
                [(0, 0)]*max(0,  skew) + [self.stripe(w, 0) for w in stream],
                [(0, 0)]*max(0, -skew) + [self.stripe(w, 1) for w in stream],
            ]
            out = self.run_deskew(lanes)
            self.assertEqual(out, ts + data, f"skew={skew}")

    def test_deskew_no_gap(self):
        # Deskew started in the middle of periodic markers (no gap): no pairing (ambiguous), until a
        # gap followed by markers.
        rng    = random.Random(2)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        first = ts(6)
        idle  = [(0, 0)]*20
        last  = ts(4)
        stream = first + idle + last
        lanes = [[self.stripe(w, 0) for w in stream], [(0, 0)]*2 + [self.stripe(w, 1) for w in stream]]
        out = self.run_deskew(lanes)
        self.assertEqual(out, last)

    def run_deskew_aligned(self, lanes):
        # Deskew aligned/armed outputs per cycle (lanes fed in parallel).
        dut = LaneDeskew(0xbcbcbcbc, 0b1111)
        trace = []
        def lane_gen(dut, lane):
            for data, ctrl in lanes[lane]:
                yield dut.sinks[lane].valid.eq(1)
                yield dut.sinks[lane].data.eq(data)
                yield dut.sinks[lane].ctrl.eq(ctrl)
                yield
            yield dut.sinks[lane].valid.eq(0)
        def checker(dut):
            for lane in range(2):
                yield dut.sources[lane].ready.eq(1)
            for _ in range(max(len(l) for l in lanes) + 16):
                trace.append(((yield dut.aligned), (yield dut.armed)))
                yield
        run_simulation(dut, [lane_gen(dut, 0), lane_gen(dut, 1), checker(dut)])
        return trace

    def test_deskew_single_mismatch_tolerated(self):
        # Last COM word of the TS stream lost on Lane 0 only (decode error substituted, K28.4), then
        # Logical Idle: a single mismatch, the lanes still paired (two Acorn Minis, end of the
        # Recovery TS2): aligned kept, the Logical Idle words then released paired.
        rng    = random.Random(3)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        sub    = (0x9c9c9c9c9c9c9c9c, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap, first, last = [(0, 0)]*20, ts(6), ts(1)
        idle = [(rng.getrandbits(64), 0) for _ in range(24)]
        lane0 = [self.stripe(w, 0) for w in gap + first] + [self.stripe(sub, 0)] + \
            [self.stripe(w, 0) for w in last[1:] + idle]
        lane1 = [self.stripe(w, 1) for w in gap + first + last + idle]
        trace = self.run_deskew_aligned([lane0, lane1])
        self.assertEqual(trace[-1][0], 1)
        self.assertEqual(self.run_deskew([lane0, lane1])[-len(idle):], idle)

    def test_deskew_lost_on_lane_shift(self):
        # Lane 0 one word late from the middle of the TS stream (word dropped on Lane 1: lanes no
        # longer paired, a mismatch at each Ordered Set): pairing lost (second mismatch), search
        # armed again after the next gap.
        rng    = random.Random(4)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap, first, more = [(0, 0)]*20, ts(4), ts(4)
        idle = [(0, 0)]*24
        lane0 = [self.stripe(w, 0) for w in gap + first + more + idle]
        lane1 = [self.stripe(w, 1) for w in gap + first + more[1:] + idle]
        trace = self.run_deskew_aligned([lane0, lane1])
        aligned = [a for a, _ in trace]
        lost = len(aligned) - aligned[::-1].index(1) # First cycle after the last aligned one.
        self.assertLess(lost, len(gap + first) + 12) # Within 2 Ordered Sets of the shift.
        self.assertEqual(trace[-1], (0, 1))          # Not aligned, armed (gap after the TS).

    def test_lane_swap(self):
        # Logical lanes on the physical SerDes (lane i on SerDes i ^ swap), swap detected from the
        # partner's LFPS on a single physical lane (detection enabled only).
        class SerDes:
            def __init__(self):
                for name in ["tx_idle", "tx_lfps", "tx_polarity", "rx_polarity", "tx_skp_enable",
                    "rx_align", "ready", "rx_idle", "rx_errors"]:
                    setattr(self, name, Signal())
                self.tx_pattern = Signal(20)
                self.sink   = stream.Endpoint(lane_layout())
                self.source = stream.Endpoint(lane_layout())
                self.rx_idle_stretch = 0
        serdes = [SerDes(), SerDes()]
        dut    = LaneSwap(serdes, detect=True, burst_cycles=4)
        lanes  = dut.lanes
        def generator():
            for p in range(2):
                yield serdes[p].rx_idle.eq(1)
                yield serdes[p].source.valid.eq(1)
                yield serdes[p].source.data.eq(0x100 + p)
                yield serdes[p].ready.eq(p)
            yield lanes[0].tx_idle.eq(1)
            yield lanes[0].sink.valid.eq(1)
            yield lanes[0].sink.data.eq(0xaa)
            yield
            # Not swapped: lane i on SerDes i.
            self.assertEqual((yield serdes[0].tx_idle), 1)
            self.assertEqual((yield serdes[1].tx_idle), 0)
            self.assertEqual((yield serdes[0].sink.data), 0xaa)
            self.assertEqual((yield lanes[0].source.data), 0x100)
            self.assertEqual((yield lanes[1].ready), 1)
            # LFPS on SerDes 1 only, detection disabled: no swap.
            yield serdes[1].rx_idle.eq(0)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.swap), 0)
            # Detection enabled: swapped (Lane 0 on SerDes 1).
            yield dut.enable.eq(1)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.swap), 1)
            self.assertEqual((yield serdes[1].tx_idle), 1)
            self.assertEqual((yield serdes[0].tx_idle), 0)
            self.assertEqual((yield serdes[1].sink.data), 0xaa)
            self.assertEqual((yield serdes[1].sink.valid), 1)
            self.assertEqual((yield serdes[0].sink.valid), 0)
            self.assertEqual((yield lanes[0].source.data), 0x101)
            self.assertEqual((yield lanes[0].ready), 1)
            # Both lanes active (data): kept. LFPS on SerDes 0 only: back to not swapped.
            yield serdes[0].rx_idle.eq(0)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.swap), 1)
            yield serdes[1].rx_idle.eq(1)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.swap), 0)
        run_simulation(dut, generator())

if __name__ == "__main__":
    unittest.main()
