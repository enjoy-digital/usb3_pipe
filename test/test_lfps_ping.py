#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.lfps   import LFPSUnit

from test.model.gt import LFPSSerDesStub

SYS_CLK_FREQ = 125e6

def ns_to_cycles(t):
    return round(t*1e-9*SYS_CLK_FREQ)

# Ping.LFPS / U1 Exit classification (USB 3.2 Table 6-30, 6.9.2 implementation note) ---------------

class TestPingClassification(unittest.TestCase):
    def burst(self, burst_cycles):
        """Send a single burst of burst_cycles (raw electrical idle deasserted), return the number
        of rx_ping pulses and whether rx_exit_u1 was asserted."""
        serdes = LFPSSerDesStub(with_rx_idle_filter=True)
        dut    = LFPSUnit(serdes=serdes, sys_clk_freq=SYS_CLK_FREQ)
        dut.submodules += serdes
        dut.pings   = 0
        dut.exit_u1 = 0

        def generator(dut):
            for _ in range(64):
                yield
            yield serdes.rx_elecidle.eq(0)
            for _ in range(burst_cycles):
                yield
            yield serdes.rx_elecidle.eq(1)
            for _ in range(256):
                yield

        def monitor(dut):
            for _ in range(64): # Reset settling.
                yield
            for _ in range(burst_cycles + 256):
                yield
                dut.pings   += (yield dut.rx_ping)
                dut.exit_u1 |= (yield dut.rx_exit_u1)

        run_simulation(dut, [generator(dut), monitor(dut)])
        return dut.pings, dut.exit_u1

    def test_ping_lfps(self):
        # Ping.LFPS: tBurst 40-200ns: one Ping, no U1 exit.
        for t in [40, 100, 160, 200]:
            with self.subTest(burst_ns=t):
                self.assertEqual(self.burst(ns_to_cycles(t)), (1, 0))

    def test_glitch_is_not_ping(self):
        # Below the minimum Ping.LFPS tBurst (40ns, 2 LFPS cycles): no Ping, no U1 exit.
        for t in [8, 16, 24]:
            with self.subTest(burst_ns=t):
                self.assertEqual(self.burst(ns_to_cycles(t)), (0, 0))

    def test_u1_exit(self):
        # U1 exit is declared for bursts lasting 300ns or more, without a Ping.
        for t in [304, 600, 900]:
            with self.subTest(burst_ns=t):
                self.assertEqual(self.burst(ns_to_cycles(t)), (0, 1))

if __name__ == "__main__":
    unittest.main()
