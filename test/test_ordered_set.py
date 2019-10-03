# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe.common import TSEQ
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
