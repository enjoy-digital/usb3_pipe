# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe.scrambling import Scrambler, Descrambler

scrambler_ref = [
    0x8dbf6dbe, 0xe6a740be, 0xb2e2d32c, 0x2a770207,
    0xe0be34cd, 0xb1245da7, 0x22bda19b, 0xd31d45d4,
    0xee76ead7, 0xfa1ada2c, 0x3b362d28, 0x676f0e3a,
    0x264c06cf, 0xcd3ae9d3, 0xfc307627, 0xde038b94,
    0xf65206d3, 0x9580884f, 0xf2666ac4, 0x35a10c9f,
    0x27cf41e2, 0x9e7e4074, 0x84fe58a5, 0xa9086009,
    0x626f0bf1, 0xed5c4317, 0xd43f3948, 0xb30ef55a,
    0x9b9d03c7, 0x5c8e0d8b, 0xae779833, 0x3e0bac2d,
    0x7a420bda, 0xa8cfd17c, 0x41ee121c, 0x7a383fc2,
    0x01f4690d, 0xc57231da, 0x0e93d7a0, 0x55a4afdc,
    0x1672f0e7, 0x8438d568, 0x18cd00dd, 0x5930ca9e,
    0x771b754c, 0xcfedc531, 0x3d6e6491, 0x0429e8fe,
    0xc4fc6ccf, 0x62da5e0b, 0xdfab5bba, 0x377db759,
    0xc61ae35e, 0x4ff51488, 0xcb56c88b, 0x634210d3,
    0xf7b48a04, 0x01a00184, 0xee674983, 0xa48b2a3e,
    0xd514af76, 0xb660ac4f, 0xb762d679, 0x2ae5e743
]

class TestScrambling(unittest.TestCase):
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

        dut = Scrambler()
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

        dut = Scrambler()
        run_simulation(dut, generator(dut))

    def test_descrambler_data(self):
        def generator(dut):
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(0)
                yield
            for i in range(16):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(scrambler_ref[i])
                yield
            yield dut.sink.valid.eq(0)
            for i in range(64):
                yield

        def checker(dut):
            yield dut.source.ready.eq(1)
            while (yield dut.source.valid) == 0:
                yield
            for i in range(16):
                self.assertEqual((yield dut.source.data), 0)
                yield

        dut = Descrambler()
        run_simulation(dut, [generator(dut), checker(dut)])