#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.lfps import LFPSBurstGenerator

# LFPS clock period (USB 3.2 6.9.1, Table 6-29: tPeriod 20-100ns) ----------------------------------

class TestLFPSClock(unittest.TestCase):
    def measure_period(self, sys_clk_freq, lfps_clk_freq):
        """Generated LFPS period in seconds (from the tx_pattern toggles during a burst)."""
        dut = LFPSBurstGenerator(sys_clk_freq=sys_clk_freq, lfps_clk_freq=lfps_clk_freq)
        def generator(dut):
            dut.toggles = []
            yield dut.length.eq(512)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            last = None
            for i in range(512):
                yield
                v = (yield dut.tx_pattern) & 1
                if last is not None and v != last:
                    dut.toggles.append(i)
                last = v
        run_simulation(dut, generator(dut))
        half_periods = [b - a for a, b in zip(dut.toggles[1:], dut.toggles[2:])]
        self.assertEqual(len(set(half_periods)), 1)
        return 2*half_periods[0]/sys_clk_freq

    def test_default_period_in_spec(self):
        for sys_clk_freq in [125e6, 133.33e6, 150e6, 200e6]:
            with self.subTest(sys_clk_freq=sys_clk_freq):
                period = self.measure_period(sys_clk_freq, 25e6)
                self.assertTrue(20e-9 <= period <= 100e-9, period)

    def test_out_of_spec_period_rejected(self):
        # 10MHz is accepted as a frequency (100ns), but the generated period is rounded to an even
        # number of 8ns cycles at 125MHz (112ns): out of tPeriod, must be rejected at build time.
        with self.assertRaises(AssertionError):
            LFPSBurstGenerator(sys_clk_freq=125e6, lfps_clk_freq=10e6)

if __name__ == "__main__":
    unittest.main()
