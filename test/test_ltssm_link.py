#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_ltssm import USB3LTSSM


class LTSSMLinkHarness(Module):
    def __init__(self, sys_clk_freq=int(125e6)):
        self.submodules.host = host = USB3LTSSM(sys_clk_freq, with_timers=False)
        self.submodules.dev  = dev  = USB3LTSSM(sys_clk_freq, with_timers=False)

        self.host_force_ts1 = Signal()

        self.host_lfps_count = Signal(16)
        self.dev_lfps_count  = Signal(16)

        host_ts_counter = Signal(3)
        dev_ts_counter  = Signal(3)
        host_ts_active  = Signal()
        dev_ts_active   = Signal()
        host_ts_done    = Signal()
        dev_ts_done     = Signal()

        self.comb += [
            host_ts_active.eq(host.ts_tx_enable & (host.ts_tx_tseq | host.ts_tx_ts1 | host.ts_tx_ts2)),
            dev_ts_active.eq(dev.ts_tx_enable & (dev.ts_tx_tseq | dev.ts_tx_ts1 | dev.ts_tx_ts2)),
            host.lfps_tx_count.eq(self.host_lfps_count),
            dev.lfps_tx_count.eq(self.dev_lfps_count),
            host.lfps_rx_polling.eq(dev.lfps_tx_polling & (self.dev_lfps_count >= 20)),
            dev.lfps_rx_polling.eq(host.lfps_tx_polling & (self.host_lfps_count >= 20)),
            host.ts_tx_done.eq(host_ts_done),
            dev.ts_tx_done.eq(dev_ts_done),
            host.ts_rx_ts1.eq(self.host_force_ts1 | (dev_ts_done & dev.ts_tx_ts1)),
            host.ts_rx_ts1_inv.eq(0),
            host.ts_rx_ts2.eq(dev_ts_done & dev.ts_tx_ts2),
            dev.ts_rx_ts1.eq(host_ts_done & host.ts_tx_ts1),
            dev.ts_rx_ts1_inv.eq(0),
            dev.ts_rx_ts2.eq(host_ts_done & host.ts_tx_ts2),
        ]

        self.sync += [
            If(host.lfps_tx_polling,
                self.host_lfps_count.eq(self.host_lfps_count + 1)
            ).Else(
                self.host_lfps_count.eq(0)
            ),
            If(dev.lfps_tx_polling,
                self.dev_lfps_count.eq(self.dev_lfps_count + 1)
            ).Else(
                self.dev_lfps_count.eq(0)
            ),
            host_ts_done.eq(0),
            If(host_ts_active,
                host_ts_counter.eq(host_ts_counter + 1),
                If(host_ts_counter == 3,
                    host_ts_done.eq(1),
                    host_ts_counter.eq(0)
                )
            ).Else(
                host_ts_counter.eq(0)
            ),
            dev_ts_done.eq(0),
            If(dev_ts_active,
                dev_ts_counter.eq(dev_ts_counter + 1),
                If(dev_ts_counter == 3,
                    dev_ts_done.eq(1),
                    dev_ts_counter.eq(0)
                )
            ).Else(
                dev_ts_counter.eq(0)
            ),
        ]


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
