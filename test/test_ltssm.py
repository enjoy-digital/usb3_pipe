#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_ltssm import USB3LTSSM


class TestLTSSM(unittest.TestCase):
    def wait_for(self, dut, signal, timeout=64):
        for _ in range(timeout):
            if (yield signal):
                return True
            yield
        return False

    def drive_to_u0(self, dut, inverted=False):
        yield dut.lfps_tx_count.eq(20)
        yield dut.lfps_rx_polling.eq(1)
        yield
        yield dut.lfps_tx_count.eq(24)
        yield

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_tseq)), True)
        yield dut.ts_tx_done.eq(1)
        yield
        yield dut.ts_tx_done.eq(0)

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts1)), True)
        if inverted:
            yield dut.ts_rx_ts1_inv.eq(1)
            yield
            yield dut.ts_rx_ts1_inv.eq(0)
        else:
            yield dut.ts_rx_ts1.eq(1)
            yield
            yield dut.ts_rx_ts1.eq(0)
        yield
        yield dut.ts_tx_done.eq(1)
        yield
        yield dut.ts_tx_done.eq(0)

        self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts2)), True)
        yield dut.ts_rx_ts2.eq(1)
        yield
        yield dut.ts_rx_ts2.eq(0)
        yield
        yield dut.ts_tx_done.eq(1)
        yield
        yield dut.ts_tx_done.eq(0)

        self.assertEqual((yield from self.wait_for(dut, dut.u0)), True)
        yield

    def test_polling_lfps_requires_minimum_bursts(self):
        def generator(dut):
            yield dut.lfps_rx_polling.eq(1)

            for count in range(20):
                yield dut.lfps_tx_count.eq(count)
                yield
                self.assertEqual((yield dut.ts_tx_tseq), 0)

            yield dut.lfps_tx_count.eq(20)
            seen_rxeq = False
            for _ in range(4):
                yield
                seen_rxeq = seen_rxeq or bool((yield dut.ts_tx_tseq))
            self.assertEqual(seen_rxeq, True)

        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
        run_simulation(dut, generator(dut))

    def test_inverted_ts1_sets_rx_polarity(self):
        def generator(dut):
            yield from self.drive_to_u0(dut, inverted=True)
            self.assertEqual((yield dut.serdes_rx_polarity), 1)
            self.assertEqual((yield dut.rx_ready), 1)
            self.assertEqual((yield dut.tx_ready), 1)

        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
        run_simulation(dut, generator(dut))

    def test_u0_ignores_spurious_lfps_polling(self):
        def generator(dut):
            yield from self.drive_to_u0(dut)
            yield dut.lfps_rx_polling.eq(0)
            yield
            self.assertEqual((yield dut.u0), 1)

            yield dut.lfps_rx_polling.eq(1)
            for _ in range(4):
                yield
                self.assertEqual((yield dut.u0), 1)
                self.assertEqual((yield dut.rx_ready), 1)
                self.assertEqual((yield dut.tx_ready), 1)

            yield dut.lfps_rx_polling.eq(0)
            for _ in range(4):
                yield
                self.assertEqual((yield dut.u0), 1)

        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
        run_simulation(dut, generator(dut))

    def test_recovery_sequence_returns_to_u0(self):
        def generator(dut):
            yield from self.drive_to_u0(dut)
            yield dut.lfps_rx_polling.eq(0)
            yield

            yield dut.ts_rx_ts1.eq(1)
            yield
            yield dut.ts_rx_ts1.eq(0)

            self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts1)), True)
            self.assertEqual((yield dut.u0), 0)
            self.assertEqual((yield dut.recovery), 1)

            yield dut.ts_rx_ts2.eq(1)
            yield
            yield dut.ts_rx_ts2.eq(0)
            yield
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)

            self.assertEqual((yield from self.wait_for(dut, dut.ts_tx_ts2)), True)
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)

            self.assertEqual((yield from self.wait_for(dut, dut.u0)), True)
            self.assertEqual((yield dut.rx_ready), 1)
            self.assertEqual((yield dut.tx_ready), 1)

        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
        run_simulation(dut, generator(dut))

    def test_exit_to_compliance_latches_request(self):
        def generator(dut):
            for _ in range(365):
                yield

            self.assertEqual((yield dut.exit_to_compliance), 1)
            self.assertEqual((yield dut.lfps_tx_idle), 1)

            yield dut.lfps_rx_polling.eq(1)
            for _ in range(4):
                yield
                self.assertEqual((yield dut.exit_to_compliance), 1)
                self.assertEqual((yield dut.lfps_tx_idle), 1)
                self.assertEqual((yield dut.ts_tx_tseq), 0)

        dut = USB3LTSSM(sys_clk_freq=1000, with_timers=True)
        run_simulation(dut, generator(dut))
