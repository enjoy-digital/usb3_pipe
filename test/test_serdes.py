# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest
import random

from migen import *

from usb3_pipe.serdes import SerdesTXDatapath, SerdesRXDatapath


class TestSerDes(unittest.TestCase):
    def test_datapath_loopback(self):
        prng  = random.Random(42)
        datas = [prng.randrange(2**32) for _ in range(64)]
        ctrls = [prng.randrange(2**2)  for _ in range(64)]

        class DUT(Module):
            def __init__(self):
                self.submodules.tx = SerdesTXDatapath("serdes")
                self.submodules.rx = SerdesRXDatapath("serdes")
                self.comb += self.tx.source.connect(self.rx.sink)

        def generator(dut):
            for data, ctrl in zip(datas, ctrls):
                yield dut.tx.sink.valid.eq(1)
                yield dut.tx.sink.data.eq(data)
                yield dut.tx.sink.ctrl.eq(ctrl)
                yield
                while not (yield dut.tx.sink.ready):
                    yield
                yield dut.tx.sink.valid.eq(0)

        def checker(dut):
            dut.data_errors = 0
            dut.ctrl_errors = 0
            yield dut.rx.source.ready.eq(1)
            for data, ctrl in zip(datas, ctrls):
                while not (yield dut.rx.source.valid):
                    yield
                if (yield dut.rx.source.data != data):
                    dut.data_errors += 1
                if (yield dut.rx.source.ctrl != ctrl):
                    dut.ctrl_errors += 1
                yield

        dut = DUT()
        run_simulation(dut,
            generators = [generator(dut), checker(dut)],
            clocks     = {"sys": 1e9/133e6, "serdes": 1e9/125e6}
        )
        self.assertEqual(dut.data_errors, 0)
        self.assertEqual(dut.ctrl_errors, 0)
