#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_ltssm import USB3LTSSM


class TestLTSSM(unittest.TestCase):
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
        def wait_for(dut, signal, timeout=32):
            for _ in range(timeout):
                if (yield signal):
                    return True
                yield
            return False

        def generator(dut):
            yield dut.lfps_tx_count.eq(20)
            yield dut.lfps_rx_polling.eq(1)
            yield
            yield dut.lfps_tx_count.eq(24)
            yield

            self.assertEqual((yield from wait_for(dut, dut.ts_tx_tseq)), True)
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)

            self.assertEqual((yield from wait_for(dut, dut.ts_tx_ts1)), True)
            yield dut.ts_rx_ts1_inv.eq(1)
            yield
            yield dut.ts_rx_ts1_inv.eq(0)
            yield
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)

            self.assertEqual((yield from wait_for(dut, dut.ts_tx_ts2)), True)
            self.assertEqual((yield dut.serdes_rx_polarity), 1)

            yield dut.ts_rx_ts2.eq(1)
            yield
            yield dut.ts_rx_ts2.eq(0)
            yield
            yield dut.ts_tx_done.eq(1)
            yield
            yield dut.ts_tx_done.eq(0)

            self.assertEqual((yield from wait_for(dut, dut.u0)), True)
            self.assertEqual((yield dut.serdes_rx_polarity), 1)
            self.assertEqual((yield dut.rx_ready), 1)
            self.assertEqual((yield dut.tx_ready), 1)

        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
        run_simulation(dut, generator(dut))
