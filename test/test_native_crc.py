#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_core.crc        import CRC5, CRC16, CRC32
from usb3_core.host_model import crc5, crc16, crc32

class TestNativeCRC(unittest.TestCase):
    def test_crc5(self):
        dut = CRC5()
        def generator(dut):
            for value in list(range(16)) + [0x080, 0x100, 0x280, 0x7ff]:
                yield dut.data.eq(value)
                yield
                self.assertEqual((yield dut.value), crc5(value))
        run_simulation(dut, generator(dut))

    def test_crc16(self):
        rng = random.Random(16)
        dut = CRC16()
        def generator(dut):
            for _ in range(32):
                data = [rng.randrange(256) for _ in range(12)]
                yield dut.data.eq(int.from_bytes(bytes(data), "little"))
                yield
                self.assertEqual((yield dut.value), crc16(data))
        run_simulation(dut, generator(dut))

    def test_crc32(self):
        rng = random.Random(32)
        dut = CRC32()
        def generator(dut):
            for length in [1, 2, 3, 4, 5, 18, 64, 255, 1024]:
                data = [rng.randrange(256) for _ in range(length)]
                yield dut.clear.eq(1)
                yield
                yield dut.clear.eq(0)
                for i in range(0, length, 4):
                    chunk = data[i:i+4]
                    yield dut.ce.eq(1)
                    yield dut.data.eq(int.from_bytes(bytes(chunk + [0]*(4 - len(chunk))), "little"))
                    yield dut.bytes.eq(len(chunk))
                    yield
                yield dut.ce.eq(0)
                yield
                self.assertEqual((yield dut.value), crc32(data), f"length {length}")
        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
