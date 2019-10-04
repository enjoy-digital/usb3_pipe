# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe.scrambler import Scrambler

class TestScrambler(unittest.TestCase):
    def test_scrambler(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield
            yield dut.sink.data.eq(0)
            yield dut.sink.valid.eq(1)
            yield
            for i in range(64):
                print("{:08x}".format((yield dut.source.data)))
                yield

        dut = Scrambler()
        run_simulation(dut, generator(dut), vcd_name="scrambler.vcd")
