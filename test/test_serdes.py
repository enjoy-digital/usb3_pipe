# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest
import random

from migen import *

from usb3_pipe.serdes import SerdesTXDatapath, SerdesRXDatapath
from usb3_pipe.serdes import SerdesRXAligner


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

    def test_aligner(self):
        aligner_datas_input = [
            0x030201bc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0x0201bc07, 0x0605040b, 0x0a09080f, 0x0e0d0c13,
            0x9cbc1f01, 0x0300202f, 0x8d03a500, 0x00005352,
            0xbc878685, 0x848b8a89, 0x888f8e8d, 0x8c929190,
        ]
        aligner_ctrls_input = [
            0x1, 0x0, 0x0, 0x0,
            0x2, 0x0, 0x0, 0x0,
            0x4, 0x0, 0x0, 0x0,
            0x8, 0x0, 0x0, 0x0,
        ]
        aligner_datas_reference = [
            0x030201bc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0x0b0201bc, 0x0f060504, 0x130a0908, 0x010e0d0c,
            0x202f9cbc, 0xa5000300, 0x53528d03, 0x86850000,
            0x8b8a89bc, 0x8f8e8d84, 0x92919088, 0x9291908c,
        ]
        aligner_ctrls_reference = [
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
        ]

        def generator(dut):
            yield dut.sink.valid.eq(1)
            for data, ctrl in zip(aligner_datas_input, aligner_ctrls_input):
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield

        def checker(dut):
            dut.datas_errors = 0
            dut.ctrls_errors = 0
            yield dut.source.ready.eq(1)
            while not (yield dut.source.valid):
                yield
            for data, ctrl in zip(aligner_datas_reference, aligner_ctrls_reference):
                if (yield dut.source.data) != data:
                    dut.datas_errors += 1
                if (yield dut.source.ctrl) != ctrl:
                    dut.ctrls_errors += 1
                yield

        dut = SerdesRXAligner()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.datas_errors, 0)
        self.assertEqual(dut.ctrls_errors, 0)
