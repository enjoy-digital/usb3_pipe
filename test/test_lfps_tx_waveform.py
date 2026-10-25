#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

import pytest

from migen import *

from usb3_pipe.core import USB3PIPE

from sim import USB3SerDesModel

SYS_CLK_FREQ = int(125e6)

# LFPS TX waveform (USB 3.2 6.9.1, Table 6-29) -----------------------------------------------------

class TestLFPSTXWaveform(unittest.TestCase):
    def check_square_wave(self, wire):
        # LFPS is a square wave (6.9.1): during a burst, the line only toggles between low and high
        # levels at the LFPS frequency (tPeriod 20-100ns), no 8b/10b symbols.
        bursts = [w for w in wire if w is not None]
        self.assertTrue(len(bursts) > 0)
        self.assertEqual(set(bursts) - {0x00000, 0xfffff}, set())
        # Full periods (from half periods in 8ns cycles) within each burst, excluding the burst edges.
        runs, last, n = [], None, 0
        for w in wire:
            if w != last:
                if last is not None and w is not None:
                    runs.append(n)
                last, n = w, 0
            n += 1
        runs = runs[1:]
        period_ns = [2*r*8 for r in runs]
        self.assertTrue(len(period_ns) > 0)
        self.assertTrue(all(20 <= p <= 100 for p in period_ns), period_ns)

    def square_wave_test(self, cases):
        serdes = USB3SerDesModel(phy_dw=20)
        dut    = USB3PIPE(serdes=serdes, sys_clk_freq=SYS_CLK_FREQ, timer_scale=1)
        dut.submodules += serdes

        def generator(dut):
            dut.wires = {}
            for tx_signal, n_cycles in cases:
                wire = []
                yield getattr(dut, tx_signal).eq(1)
                for _ in range(n_cycles):
                    yield
                    wire.append(None if (yield serdes.tx_idle) else (yield serdes.tx.data))
                yield getattr(dut, tx_signal).eq(0)
                for _ in range(64):
                    yield
                dut.wires[tx_signal] = wire

        run_simulation(dut, generator(dut))
        for tx_signal, _ in cases:
            with self.subTest(tx_signal):
                self.check_square_wave(dut.wires[tx_signal])

    def test_lfps_square_wave(self):
        # Short: first Polling.LFPS burst.
        self.square_wave_test([("lfps_tx_polling", 200)])

    @pytest.mark.slow
    def test_lfps_square_wave_all(self):
        self.square_wave_test([("lfps_tx_polling", 400), ("lfps_tx_ping", 64), ("lfps_tx_exit_u1", 200)])

if __name__ == "__main__":
    unittest.main()
