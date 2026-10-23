#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.phy import USB3PHY

from test.test_ts2_link_config import GTModel

# Helpers ------------------------------------------------------------------------------------------

class GTResetModel(GTModel):
    def __init__(self):
        GTModel.__init__(self)
        self.rx_reset = Signal()

# SerDes RX Re-initialization ----------------------------------------------------------------------

class TestPHYRXReset(unittest.TestCase):
    def test_rx_reset_on_exit_to_rx_detect(self):
        # No partner: Polling.LFPS times out and the (Downstream) port exits to Rx.Detect, where the
        # SerDes RX is re-initialized (and only there).
        gt  = GTResetModel()
        dut = USB3PHY(serdes=gt, sys_clk_freq=int(125e6), role="host", with_endianness_swap=False,
            timer_scale=100000)
        dut.submodules += gt
        exit_to_rx_detect = dut.ltssm_states["Polling.ExitToRxDetect"]

        def generator(dut):
            dut.samples = []
            for _ in range(4000):
                dut.samples.append(((yield dut.ltssm_state), (yield gt.rx_reset)))
                yield

        run_simulation(dut, generator(dut))
        resets = [state for state, rx_reset in dut.samples if rx_reset]
        self.assertGreater(len(resets), 0)
        self.assertEqual(set(resets), {exit_to_rx_detect})
        self.assertTrue(all(rx_reset for state, rx_reset in dut.samples if state == exit_to_rx_detect))

if __name__ == "__main__":
    unittest.main()
