#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SuperSpeedPlus LFPS signaling (usb3_pipe/lfps.py, 6.9.4/6.9.5): SCD1/SCD2 and LBPM generation/
decoding."""

import random
import unittest

from migen import *

from litex.gen import *

from usb3_pipe.lfps import *

SYS_CLK_FREQ = 133e6
TIMER_SCALE  = 10

# Tests --------------------------------------------------------------------------------------------

class TestSSPLFPS(unittest.TestCase):
    def test_scd_bits(self):
        # SCD1 = 0010b, SCD2 = 1101b, transmitted LSb first (6.9.4.2).
        self.assertEqual((SCD1, SCD2), (0b0010, 0b1101))
        self.assertEqual(scd_bits(SCD1), [0, 1, 0, 0])
        self.assertEqual(scd_bits(SCD2), [1, 0, 1, 1])

    def test_lbpm_values(self):
        # Table 7-13.
        self.assertEqual(lbpm_phy_capability(gen2=False, x2=False), 0b00000000)
        self.assertEqual(lbpm_phy_capability(gen2=False, x2=True),  0b00100000)
        self.assertEqual(lbpm_phy_capability(gen2=True,  x2=True),  0b00100100)
        self.assertEqual(lbpm_phy_ready(dfp=True,  x2=True), 0b01000001)
        self.assertEqual(lbpm_phy_ready(dfp=False, x2=True), 0b00000001)

    def scd_run(self, mode):
        class DUT(LiteXModule):
            def __init__(self):
                self.gen      = SCDGenerator(SYS_CLK_FREQ, TIMER_SCALE)
                self.frontend = LFPSRXFrontEnd()
                self.chk      = SCDChecker(self.frontend, SYS_CLK_FREQ, TIMER_SCALE)
                self.comb += self.frontend.idle.eq(~self.gen.tx_lfps)
        dut = DUT()
        events = {"scd1": 0, "scd2": 0, "polling": 0, "plain": 0}
        def generator(dut):
            yield dut.gen.enable.eq(1)
            yield dut.gen.mode.eq(mode)
            for _ in range(8000):
                for name in events:
                    events[name] += (yield getattr(dut.chk, name))
                yield
            events["count"] = (yield dut.gen.count)
        run_simulation(dut, generator(dut))
        return events

    def test_scd(self):
        scd1 = self.scd_run(1)
        self.assertGreaterEqual(scd1["scd1"], scd1["count"] - 2)
        self.assertEqual(scd1["scd2"], 0)
        self.assertEqual(scd1["plain"], 0)
        scd2 = self.scd_run(2)
        self.assertGreaterEqual(scd2["scd2"], scd2["count"] - 2)
        self.assertEqual(scd2["scd1"], 0)
        plain = self.scd_run(0)
        self.assertEqual(plain["scd1"] + plain["scd2"], 0)
        self.assertGreaterEqual(plain["plain"], plain["count"] - 2)
        self.assertEqual(plain["polling"], plain["plain"])

    def test_lbpm(self):
        rng      = random.Random(0)
        messages = [rng.getrandbits(8) for _ in range(8)]
        class DUT(LiteXModule):
            def __init__(self):
                self.gen      = LBPMGenerator(SYS_CLK_FREQ, TIMER_SCALE)
                self.frontend = LFPSRXFrontEnd()
                self.chk      = LBPMChecker(self.frontend, SYS_CLK_FREQ, TIMER_SCALE)
                self.comb += self.frontend.idle.eq(~self.gen.tx_lfps)
        dut      = DUT()
        received = []
        def generator(dut):
            yield dut.gen.data.eq(messages[0])
            yield
            yield dut.gen.enable.eq(1)
            for m in messages[1:]:
                # Next LBPM data, sampled at the end of the current one.
                yield dut.gen.data.eq(m)
                while not (yield dut.gen.sent):
                    yield
                yield
            for _ in range(1000):
                yield
        def checker(dut):
            for _ in range(5000):
                if (yield dut.chk.received):
                    received.append((yield dut.chk.data))
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(received[:len(messages)], messages)

if __name__ == "__main__":
    unittest.main()
