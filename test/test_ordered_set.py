# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe.common import TSEQ, TS1
from usb3_pipe.ordered_set import OrderedSetReceiver


class TestOrderedSet(unittest.TestCase):
    def test_tseq_receiver(self):
        tseq_length = len(TSEQ.to_bytes())//4
        tseq_words  = [int.from_bytes(TSEQ.to_bytes()[4*i:4*(i+1)], "little") for i in range(tseq_length)]
        def generator(dut, n_loops):
            yield dut.sink.valid.eq(1)
            for i in range(n_loops):
                for j in range(tseq_length):
                    yield dut.sink.ctrl.eq(j == 0)
                    yield dut.sink.data.eq(tseq_words[j])
                    yield
            for i in range(128):
                yield
            dut.run = False

        def checker(dut, n_loops, n_ordered_sets):
            count = 0
            while dut.run:
                if (yield dut.detected):
                    count += 1
                yield
            self.assertEqual(count, n_loops/n_ordered_sets)

        dut = OrderedSetReceiver(ordered_set=TSEQ, n_ordered_sets=4, data_width=32)
        dut.run = True
        generators = [
            generator(dut, n_loops=32),
            checker(dut, n_loops=32, n_ordered_sets=4),
        ]
        run_simulation(dut, generators, vcd_name="tseq.vcd")


    def test_ts1_receiver(self):
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

        dut = OrderedSetReceiver(ordered_set=TS1, n_ordered_sets=4, data_width=32)
        dut.run = True
        generators = [
            generator(dut, n_loops=32),
            checker(dut, n_loops=32, n_ordered_sets=4),
        ]
        run_simulation(dut, generators, vcd_name="ts1.vcd")