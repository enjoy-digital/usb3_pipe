#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.serdes import RXIdleFilter

SYS_CLK_FREQ = 125e6

# Helpers ------------------------------------------------------------------------------------------

class SerDesModel(Module):
    """LFPS face of the SerDes: raw electrical idle detection through the RX idle filter."""
    def __init__(self):
        self.rx_elecidle = Signal(reset=1)
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)
        self.submodules.rx_idle_filter = RXIdleFilter(self.rx_elecidle)
        self.rx_idle         = self.rx_idle_filter.idle
        self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles

def detections(t_burst, t_repeat, n=4):
    serdes = SerDesModel()
    dut    = LFPSUnit(serdes=serdes, sys_clk_freq=SYS_CLK_FREQ)
    dut.submodules += serdes
    b, r = int(round(t_burst*SYS_CLK_FREQ)), int(round(t_repeat*SYS_CLK_FREQ))
    def generator(dut):
        dut.detections = 0
        for _ in range(n):
            for i in range(r):
                yield serdes.rx_elecidle.eq(i >= b)
                yield
                dut.detections += (yield dut.rx_polling)
    run_simulation(dut, generator(dut))
    return dut.detections

# Polling.LFPS reception (USB 3.2 6.9, Table 6-30: tBurst 0.6-1.4us, tRepeat 6-14us) --------------

class TestLFPSPollingRX(unittest.TestCase):
    def test_polling_lfps_spec_corners_detected(self):
        # Polling.LFPS from a partner anywhere within the Table 6-30 limits must be detected, the
        # SerDes electrical idle filter stretching the bursts.
        for t_burst, t_repeat in [(0.6e-6, 6e-6), (0.6e-6, 14e-6), (1.4e-6, 6e-6), (1.4e-6, 14e-6),
            (1.0e-6, 10e-6)]:
            with self.subTest(t_burst=t_burst, t_repeat=t_repeat):
                self.assertGreaterEqual(detections(t_burst, t_repeat, n=3), 1)

    def test_out_of_spec_polling_lfps_not_detected(self):
        for t_burst, t_repeat in [(0.4e-6, 10e-6), (1.8e-6, 10e-6), (1.0e-6, 20e-6)]:
            with self.subTest(t_burst=t_burst, t_repeat=t_repeat):
                self.assertEqual(detections(t_burst, t_repeat, n=3), 0)

if __name__ == "__main__":
    unittest.main()
