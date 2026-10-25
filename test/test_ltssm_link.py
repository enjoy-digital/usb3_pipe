#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from test.model.ltssm import LTSSMLinkHarness


class TestLTSSMLink(unittest.TestCase):
    def wait_for(self, signal, timeout=256):
        for i in range(timeout):
            if (yield signal):
                return i
            yield
        return None

    def test_dual_ltssm_reaches_u0(self):
        dut = LTSSMLinkHarness()

        def generator():
            cycles = yield from self.wait_for(dut.host.u0 & dut.dev.u0, timeout=256)
            self.assertNotEqual(cycles, None)
            self.assertEqual((yield dut.host.tx_ready), 1)
            self.assertEqual((yield dut.dev.tx_ready), 1)

        run_simulation(dut, generator())

    def test_dual_ltssm_recovery_returns_to_u0(self):
        dut = LTSSMLinkHarness()

        def generator():
            cycles = yield from self.wait_for(dut.host.u0 & dut.dev.u0, timeout=256)
            self.assertNotEqual(cycles, None)

            yield dut.host_force_ts1.eq(1)
            yield
            yield dut.host_force_ts1.eq(0)

            cycles = yield from self.wait_for(dut.host.recovery & dut.dev.recovery, timeout=256)
            self.assertNotEqual(cycles, None)

            cycles = yield from self.wait_for(dut.host.u0 & dut.dev.u0, timeout=256)
            self.assertNotEqual(cycles, None)
            self.assertEqual((yield dut.host.rx_ready), 1)
            self.assertEqual((yield dut.dev.rx_ready), 1)

        run_simulation(dut, generator())
