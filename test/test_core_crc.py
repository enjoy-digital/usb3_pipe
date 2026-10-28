#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_core.crc        import CRC5, CRC16, CRC16Words, CRC32
from usb3_core.host_model import crc5, crc16, crc32

class TestCoreCRC(unittest.TestCase):
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

    def test_crc16_words(self):
        rng = random.Random(161)
        dut = CRC16Words()
        def generator(dut):
            for _ in range(16):
                data = [rng.randrange(256) for _ in range(12)]
                yield dut.clear.eq(1)
                yield
                yield dut.clear.eq(0)
                for i in range(3):
                    yield dut.ce.eq(1)
                    yield dut.data.eq(int.from_bytes(bytes(data[4*i:4*i+4]), "little"))
                    yield
                yield dut.ce.eq(0)
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

    def test_crc32_64(self):
        # 64-bit words: 8 bytes per word, the last 1-4 bytes in 32-bit words (data[0:32]).
        rng = random.Random(64)
        dut = CRC32(data_width=64)
        def generator(dut):
            for length in [1, 4, 5, 8, 9, 12, 13, 18, 64, 255, 1024]:
                data = [rng.randrange(256) for _ in range(length)]
                yield dut.clear.eq(1)
                yield
                yield dut.clear.eq(0)
                i = 0
                while i < length:
                    n = 8 if (length - i) >= 8 else min(4, length - i)
                    chunk = data[i:i+n]
                    yield dut.ce.eq(1)
                    yield dut.data.eq(int.from_bytes(bytes(chunk + [0]*(8 - len(chunk))), "little"))
                    yield dut.bytes.eq(n)
                    yield
                    i += n
                yield dut.ce.eq(0)
                yield
                self.assertEqual((yield dut.value), crc32(data), f"length {length}")
        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
