#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.common import TSEQ, TS1
from usb3_pipe.training import TSChecker, TSGenerator


class TestTraining(unittest.TestCase):
    def test_ts1_checker(self):
        ts1_length = len(TS1.to_bytes())//4
        ts1_words  = [int.from_bytes(TS1.to_bytes()[4*i:4*(i+1)], "little") for i in range(ts1_length)]
        def generator(dut, n_loops):
            yield dut.sink.valid.eq(1)
            for i in range(n_loops):
                for j in range(ts1_length):
                    yield dut.sink.ctrl.eq(0b1111 if j == 0 else 0b0000)
                    yield dut.sink.data.eq(ts1_words[j])
                    yield
            for i in range(128):
                yield
            dut.run = False

        def checker(dut, n_loops, n_ordered_sets):
            count = 0
            while dut.run:
                if (yield dut.detected):
                    self.assertEqual((yield dut.reset),      0)
                    self.assertEqual((yield dut.loopback),   0)
                    self.assertEqual((yield dut.scrambling), 1)
                    count += 1
                yield
            self.assertEqual(count, n_loops/n_ordered_sets)

        dut = TSChecker(ordered_set=TS1, n_ordered_sets=4)
        dut.run = True
        generators = [
            generator(dut, n_loops=32),
            checker(dut, n_loops=32, n_ordered_sets=4),
        ]
        run_simulation(dut, generators)

    def test_tseq_generator(self):
        tseq_length = len(TSEQ.to_bytes())//4
        tseq_words  = [int.from_bytes(TSEQ.to_bytes()[4*i:4*(i+1)], "little") for i in range(tseq_length)]
        def generator(dut, n_loops):
            for i in range(n_loops):
                yield dut.start.eq(1)
                yield
                while not (yield dut.done):
                    yield
                yield dut.start.eq(0)
                yield
            for i in range(128):
                yield
            dut.run = False

        def checker(dut, n_loops, n_ordered_sets):
            words = []
            yield dut.source.ready.eq(1)
            yield
            i = 0
            while dut.run:
                if (yield dut.source.valid):
                    length = tseq_length*n_ordered_sets
                    self.assertEqual((yield dut.source.first), int(i%length == 0))
                    self.assertEqual((yield dut.source.last),  int(i%length == (length-1)))
                    words.append((yield dut.source.data))
                i += 1
                yield dut.source.ready.eq(0)
                yield
                yield dut.source.ready.eq(1)
                yield

            self.assertEqual(words, tseq_words*n_loops*n_ordered_sets)

        dut = TSGenerator(ordered_set=TSEQ, n_ordered_sets=4)
        dut.run = True
        generators = [
            generator(dut, n_loops=4),
            checker(dut, n_loops=4, n_ordered_sets=4),
        ]
        run_simulation(dut, generators)

    def test_ts1_generator(self):
        ts1_length = len(TS1.to_bytes())//4
        ts1_words  = [int.from_bytes(TS1.to_bytes()[4*i:4*(i+1)], "little") for i in range(ts1_length)]
        def generator(dut, n_loops):
            for i in range(n_loops):
                yield dut.start.eq(1)
                yield dut.reset.eq(0)
                yield dut.loopback.eq(0)
                yield dut.scrambling.eq(1)
                yield
                yield dut.start.eq(0)
                yield
                while not (yield dut.done):
                    yield
                yield
            for i in range(128):
                yield
            dut.run = False

        def checker(dut, n_loops, n_ordered_sets):
            words = []
            yield dut.source.ready.eq(1)
            yield
            i = 0
            while dut.run:
                if (yield dut.source.valid):
                    length = ts1_length*n_ordered_sets
                    self.assertEqual((yield dut.source.first), int(i%length == 0))
                    self.assertEqual((yield dut.source.last),  int(i%length == (length-1)))
                    words.append((yield dut.source.data))
                    i += 1
                yield
            self.assertEqual(words, ts1_words*n_loops*n_ordered_sets)

        dut = TSGenerator(ordered_set=TS1, n_ordered_sets=4)
        dut.run = True
        generators = [
            generator(dut, n_loops=32),
            checker(dut, n_loops=32, n_ordered_sets=4),
        ]
        run_simulation(dut, generators)
