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

    def run_deskew(self, lanes, cycles=300, error_symbol=None):
        class DUT(LiteXModule):
            def __init__(self):
                self.deskew    = LaneDeskew(0xbcbcbcbc, 0b1111, error_symbol=error_symbol)
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

    def test_deskew_late_lane_overflow(self):
        # Lane 1 not word aligned yet while Lane 0 already receives the TS Ordered Sets (Tang Mega
        # 138K Pro Lane 1, t16o): Lane 1 garbage (no marker: counted as a gap), its first marker
        # 10 Ordered Sets after Lane 0's. Lane 0's first marker held while armed fills its FIFO
        # (words dropped): the pairing would join Lane 1's first marker with a stale Lane 0 one, the
        # destriped words then off by Ordered Sets (markers every 4 words: never seen as a mismatch).
        # The overflow clears the search: re-armed on the next gap (Logical Idle), the next TS paired.
        rng    = random.Random(5)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap, first = [(0, 0)]*20, ts(30)
        idle = [(0, 0)]*24
        last = ts(4)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        late = 40 # Lane 1 words before its word alignment (Lane 0 already in its TS).
        garbage = [(rng.getrandbits(32) & 0x7f7f7f7f, 0) for _ in range(late)]
        lane0 = [self.stripe(w, 0) for w in gap + first + idle + last + data]
        lane1 = [self.stripe(w, 1) for w in gap] + garbage + \
            [self.stripe(w, 1) for w in first[late:] + idle + last + data]
        out = self.run_deskew([lane0, lane1], cycles=400)
        self.assertEqual(out, last + data)

    def test_deskew_late_lane_errors_no_gap(self):
        # Lane 1 not word aligned yet (garbage with decode errors substituted, K28.4) while Lane 0
        # already receives the TS Ordered Sets, its first marker one Ordered Set later than Lane 0's
        # (within the FIFOs depth: no overflow, t16s). The garbage with errors is not a gap and
        # disarms a search armed before it: no pairing until a clean gap on both lanes (Logical
        # Idle), the next TS paired.
        rng    = random.Random(6)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        sub    = (0x9c9c9c9c9c9c9c9c, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap, first = [(0, 0)]*20, ts(10)
        idle = [(0, 0)]*24
        last = ts(4)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        late = 4 # One Ordered Set.
        garbage = [self.stripe(sub, 1) if i % 3 == 0 else (rng.getrandbits(32) & 0x7f7f7f7f, 0)
            for i in range(24 + late)]
        for armed in [False, True]:
            # armed: a clean gap on both lanes first (search armed before the garbage, disarmed by
            # its error words).
            pre = [(0, 0)]*20 if armed else []
            lane0 = [self.stripe(w, 0) for w in pre + [(0, 0)]*24 + first + idle + last + data]
            lane1 = [self.stripe(w, 1) for w in pre] + garbage + \
                [self.stripe(w, 1) for w in first[late:] + idle + last + data]
            out = self.run_deskew([lane0, lane1], cycles=400, error_symbol=0x9c)
            self.assertEqual(out, last + data, f"armed={armed}")
        lane0 = [self.stripe(w, 0) for w in [(0, 0)]*24 + first + idle + last + data]
        lane1 = garbage + [self.stripe(w, 1) for w in first[late:] + idle + last + data]
        # Without the error Symbol check, the garbage is a gap: the lanes paired one Ordered Set apart,
        # seen at the end of the TS stream (a marker against Logical Idle): pairing cleared, the next
        # TS paired.
        out = self.run_deskew([lane0, lane1], cycles=400)
        self.assertEqual(out[-len(last + data):], last + data)

    def test_deskew_os_apart_end_of_stream(self):
        # Lane 1 paired one TS Ordered Set early (its first marker lost without a decode error: Tang
        # Mega 138K Pro, t25): identical TS Ordered Sets, no mismatch until the end of the stream, where
        # Lane 0's last marker meets Lane 1's Logical Idle (plain data word): pairing cleared at once,
        # re-armed on the gap (deskew lost, Recovery).
        rng    = random.Random(9)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap, first = [(0, 0)]*20, ts(8)
        idle = [(rng.getrandbits(64), 0) for _ in range(40)]
        lane0 = [self.stripe(w, 0) for w in gap + first + idle]
        lane1 = [self.stripe(w, 1) for w in gap + [(0, 0)]*4 + first[4:] + idle + [(0, 0)]*4]
        trace = self.run_deskew_aligned([lane0, lane1])
        aligned = [a for a, _ in trace]
        self.assertIn(1, aligned)                    # Paired (one Ordered Set apart).
        lost = len(aligned) - aligned[::-1].index(1)
        self.assertLess(lost, len(gap + first) + 8)  # Cleared at the end of the TS stream.
        self.assertEqual(trace[-1], (0, 1))          # Re-armed on the gap.

    def test_deskew_first_com_errors_armed(self):
        # Search armed on a gap, the first COM Symbols of the first TS Ordered Set received as K28.4
        # on both lanes (hardware, after an idle period): corrupted marker words, paired (t16u: the
        # search disarmed, the lanes were never paired again).
        rng    = random.Random(7)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        bad    = (0xbcbcbcbcbcbc9c9c, 0xff) # First 2 COMs as K28.4.
        def ts(n, first_bad=False):
            r = []
            for i in range(n):
                r += [bad if (first_bad and i == 0) else marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap  = [(0, 0)]*20
        tss  = ts(6, first_bad=True)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        stream = gap + tss + data
        lanes = [[(0, 0)] + [self.stripe(w, 0) for w in stream], [self.stripe(w, 1) for w in stream]]
        out = self.run_deskew(lanes, error_symbol=0x9c)
        self.assertEqual(len(out), len(tss + data))
        self.assertEqual(out[1:], tss[1:] + data)

    def test_deskew_first_com_errors_one_lane(self):
        # First COM Symbols of the first TS Ordered Set received as K28.4 on Lane 0 only: the
        # corrupted marker word is a marker, paired with Lane 1's first marker (not with Lane 0's
        # next one: the lanes an Ordered Set apart, t16v).
        rng    = random.Random(8)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap  = [(0, 0)]*20
        tss  = ts(6)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        stream = gap + tss + data
        lane0 = [self.stripe(w, 0) for w in stream]
        lane0[len(gap)] = (0xbcbc9c9c, 0xf) # Lane 0 first marker: 2 COMs as K28.4.
        lane1 = [self.stripe(w, 1) for w in stream]
        out = self.run_deskew([[(0, 0)] + lane0, lane1], error_symbol=0x9c)
        self.assertEqual(len(out), len(tss + data))
        self.assertEqual(out[1:], tss[1:] + data)

    def test_deskew_trailing_error_before_first_marker(self):
        # Search armed on a gap, the word before the first marker word on a lane ending with K28.4
        # (the Symbol before the first COM, or the first COM Symbols of a shifted lane, received as
        # errors): not an error word disarming the search (the next word is a marker), the first TS
        # Ordered Set paired (Tang Mega 138K Pro t45: disarmed, no gap again before U0, the lanes
        # never paired). An error word followed by a non-marker word still disarms (no pairing on
        # the TS stream without a new gap).
        rng    = random.Random(10)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        gap  = [(0, 0)]*20
        tss  = ts(6)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        stream = gap + tss + data
        for lane, word in [(0, (0x9c000000, 0b1000)), (1, (0x9c9c0000, 0b1100)), (1, (0x9c9c9c9c, 0xf))]:
            with self.subTest(lane=lane, word=hex(word[0])):
                lanes = [[self.stripe(w, n) for w in stream] for n in range(2)]
                lanes[lane][len(gap) - 1] = word
                out = self.run_deskew(lanes, error_symbol=0x9c)
                self.assertEqual(out, tss + data)
        # Error words in quick succession before the marker word (more than the isolated errors
        # budget, a lane not word aligned yet): disarms.
        lanes = [[self.stripe(w, n) for w in stream] for n in range(2)]
        for k in range(2, 6):
            lanes[1][len(gap) - k] = (0x009c0000, 0b0100)
        self.assertEqual(self.run_deskew(lanes, error_symbol=0x9c), [])

    def test_deskew_isolated_errors_in_gap(self):
        # Search armed on a gap, then isolated error words in the Logical Idle before the first TS
        # burst on both lanes (Acorn Mini TX -> Tang Mega 138K Pro: a K28.4, then K28.4 K28.4 D0.0
        # D0.0 ~50 words before each burst, t48): the search stays armed, the first TS paired (they
        # disarmed it before the first markers of every Recovery: Hot Reset loops until a retrain).
        rng    = random.Random(11)
        marker = (0xbcbcbcbcbcbcbcbc, 0xff)
        def ts(n):
            r = []
            for _ in range(n):
                r += [marker] + [(rng.getrandbits(64), 0) for _ in range(3)]
            return r
        tss  = ts(6)
        data = [(rng.getrandbits(64), rng.getrandbits(8) & 0x7f) for _ in range(16)]
        # Fewer than a gap of Logical Idle words between the error words and the burst (hardware: no
        # gap reached again before the markers, the receiver coming out of electrical idle).
        single = (0x009c0000, 0b0100)
        pair   = (0x00009c9c, 0b0011)
        for errors in [[single], [single, pair], [single, pair, single]]:
            with self.subTest(errors=len(errors)):
                stream = [(0, 0)]*20
                for w in errors:
                    stream += [w] + [(0, 0)]*5
                stream += [(0, 0)]*3 + tss + data
                lanes = [[self.stripe(w, n) for w in stream] for n in range(2)]
                out = self.run_deskew(lanes, error_symbol=0x9c)
                self.assertEqual(out, tss + data)

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

    # x2 Link interface building blocks --------------------------------------------------------------

    @staticmethod
    def x2_packets(rng, n=24):
        """Gen 1 Link Layer packets as lists of (word, ctrl) link words (Symbol 0 in the LSBs):
        Header Packets (SHP + 4 words), Link Commands (SLC + 1 word), DPHs + DPPs (zero data words
        included, END framing)."""
        def framing(k):
            return (k | (k << 8) | (k << 16) | (0xf7 << 24), 0b1111) # 3 K Symbols + EPF.
        SHP, SLC, SDP, END = 0xfb, 0xfe, 0x5c, 0xfd # K27.7, K30.7, K28.2, K29.7.
        packets = []
        for _ in range(n):
            kind = rng.choice(["hp", "lc", "dp"])
            if kind == "hp":
                packets.append([framing(SHP)] + [(rng.getrandbits(32) | 1, 0) for _ in range(4)])
            elif kind == "lc":
                packets.append([framing(SLC), (rng.getrandbits(32) | 1, 0)])
            else:
                dpp = [(rng.choice([0, rng.getrandbits(32)]), 0) for _ in range(rng.randint(1, 12))]
                packets.append([framing(SHP)] + [(rng.getrandbits(32) | 1, 0) for _ in range(4)] +
                    [framing(SDP)] + dpp + [framing(END)])
        return packets

    def test_tx_packer(self):
        # Packets as link words pairs (1 or 2 words, a pair never spans two packets), random gaps on
        # both sides: 64-bit words with the packets words in order, never two packets in a word, a
        # Logical Idle word only after the last word of a packet, first/last on the packets words.
        rng     = random.Random(3)
        packets = self.x2_packets(rng)
        pairs   = []
        for p in packets:
            i = 0
            while i < len(p):
                n = 2 if (i + 1 < len(p)) and rng.random() < 0.6 else 1
                pairs.append((p[i:i+n], i == 0, i + n == len(p)))
                i += n
        dut = LaneTXPacker()
        out = []
        def source_gen():
            for words, first, last in pairs:
                while rng.random() < 0.3:
                    yield dut.sink.valid.eq(0)
                    yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(sum(w << (32*i) for i, (w, c) in enumerate(words)))
                yield dut.sink.ctrl.eq(sum(c << (4*i) for i, (w, c) in enumerate(words)))
                yield dut.sink.words.eq(len(words))
                yield dut.sink.first.eq(first)
                yield dut.sink.last.eq(last)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
        def sink_gen():
            for _ in range(2000):
                ready = rng.random() < 0.7
                yield dut.source.ready.eq(ready)
                yield
                if ready and (yield dut.source.valid):
                    out.append(((yield dut.source.data), (yield dut.source.ctrl),
                        (yield dut.source.first), (yield dut.source.last)))
        run_simulation(dut, [source_gen(), sink_gen()])
        got, current = [], None
        for data, ctrl, first, last in out:
            words = [(data & 0xffffffff, ctrl & 0xf), (data >> 32, ctrl >> 4)]
            if first:
                self.assertIsNone(current)
                current = []
            self.assertIsNotNone(current)
            current.append(words[0])
            if last:
                # Second word: last word of the packet or a Logical Idle word after it.
                if words[1] != (0, 0) or len(current) + 1 <= len(packets[len(got)]) and \
                    packets[len(got)][len(current)] == (0, 0):
                    current.append(words[1])
                if current[-1] == (0, 0) and len(current) > len(packets[len(got)]):
                    current.pop()
                got.append(current)
                current = None
            else:
                current.append(words[1])
        self.assertEqual(got, packets)

    @staticmethod
    def x2_symbol_packets(rng, n=32):
        """Gen 1 Link Layer packets as lists of (byte, k) symbols: Header Packets (20 symbols), Link
        Commands (8 symbols), DPHs + DPPs of any length (1-40 data bytes, END or EDB framing)."""
        SHP, SLC, SDP, END, EDB, EPF = 0xfb, 0xfe, 0x5c, 0xfd, 0x7c, 0xf7
        def framing(k):
            return [(k, 1)]*3 + [(EPF, 1)]
        def data(n):
            return [(rng.getrandbits(8), 0) for _ in range(n)]
        packets = []
        for _ in range(n):
            kind = rng.choice(["hp", "lc", "dp"])
            if kind == "hp":
                packets.append(framing(SHP) + data(16))
            elif kind == "lc":
                packets.append(framing(SLC) + data(4))
            else:
                packets.append(framing(SHP) + data(16))
                packets.append(framing(SDP) + data(rng.randint(1, 40) + 4) +
                    framing(rng.choice([END, EDB])))
        return packets

    def run_packet_aligner(self, packets, symbols, gaps):
        """Symbols (8 per cycle, valid gaps every gaps cycles): output words checked against the
        packets, each packet completed with Logical Idle symbols to a multiple of 4 symbols."""
        symbols = symbols + [(0, 0)]*(-len(symbols) % 8 + 16)
        dut = LaneRXAligner()
        out = []
        overflow = []
        def gen():
            cycle = 0
            for i in range(0, len(symbols), 8):
                while gaps and (cycle % gaps == gaps - 1):
                    yield dut.sink.valid.eq(0)
                    yield
                    cycle += 1
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(sum(symbols[i+k][0] << (8*k) for k in range(8)))
                yield dut.sink.ctrl.eq(sum(symbols[i+k][1] << k for k in range(8)))
                yield
                cycle += 1
            yield dut.sink.valid.eq(0)
            for _ in range(16):
                yield
        @passive
        def mon():
            while True:
                yield
                if (yield dut.overflow):
                    overflow.append(1)
                if (yield dut.source.valid):
                    n = (yield dut.source.words)
                    data, ctrl = (yield dut.source.data), (yield dut.source.ctrl)
                    out.extend([((data >> (8*k)) & 0xff, (ctrl >> k) & 1) for k in range(4*n)])
        run_simulation(dut, [gen(), mon()])
        if packets is None:
            return out, bool(overflow)
        expected = []
        for p in packets:
            expected += p + [(0, 0)]*(-len(p) % 4)
        self.assertEqual(out, expected)
        self.assertEqual(overflow, [])

    def test_packet_aligner(self):
        # Packets at any symbol position, 0-9 Logical Idle symbols between them (DPPs of any length,
        # next packet right after the end of a DPP included), valid gaps: the packets symbols only,
        # each packet starting on a 32-bit word (last word completed with Logical Idle symbols).
        rng     = random.Random(5)
        packets = self.x2_symbol_packets(rng)
        symbols = [(0, 0)]*rng.randint(0, 7)
        for p in packets:
            symbols += [(0, 0)]*rng.randint(0, 9) + p
        self.run_packet_aligner(packets, symbols, gaps=5)

    def test_packet_aligner_back_to_back(self):
        # Packets back to back (no Logical Idle), every starting symbol position: completions of the
        # DPPs absorbed by the words FIFOs.
        for offset in range(8):
            rng     = random.Random(6 + offset)
            packets = self.x2_symbol_packets(rng, n=12)
            symbols = [(0, 0)]*offset + [s for p in packets for s in p]
            self.run_packet_aligner(packets, symbols, gaps=8)

    def test_packet_aligner_overflow(self):
        # Back to back DPHs + DPPs of odd lengths without Logical Idle nor valid gaps (completions
        # not absorbed): overflow, words FIFOs flushed. The packets received after it are complete
        # and in order (without the flush, the banks are no longer read in their write order).
        hp = self.x2_symbol_packets(random.Random(0), n=1)[0][:20]
        for offset in range(8):
            with self.subTest(offset=offset):
                rng   = random.Random(8 + offset)
                burst = []
                for _ in range(rng.randint(60, 90)):
                    burst += hp + [(0x5c, 1)]*3 + [(0xf7, 1)]
                    burst += [(0x55, 0)]*(rng.choice([1, 2, 3, 5, 6, 7]) + 4)
                    burst += [(0xfd, 1)]*3 + [(0xf7, 1)]
                packets = self.x2_symbol_packets(rng, n=16)
                symbols = [(0, 0)]*offset + burst + [(0, 0)]*64
                for p in packets:
                    symbols += [(0, 0)]*rng.randint(0, 9) + p
                expected = []
                for p in packets:
                    expected += p + [(0, 0)]*(-len(p) % 4)
                out, overflow = self.run_packet_aligner(None, symbols, gaps=0)
                self.assertTrue(overflow)
                self.assertEqual(out[-len(expected):], expected)

    def test_packet_aligner_tx_packer(self):
        # Packets of a LaneTXPacker partner (64-bit words, framing on 32-bit words, idle words):
        # unchanged.
        rng     = random.Random(7)
        packets = self.x2_symbol_packets(rng)
        symbols = []
        for p in packets:
            p = p + [(0, 0)]*(-len(p) % 4)
            symbols += [(0, 0)]*(4*rng.randint(0, 3)) + p
            symbols += [(0, 0)]*(-len(symbols) % 8)
        self.run_packet_aligner(packets, symbols, gaps=0)

if __name__ == "__main__":
    unittest.main()
