#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3PHY SuperSpeedPlus Polling results (x2/gen2): updated on each SuperSpeedPlus Polling, the
SuperSpeed fallback included (a Gen 2 capable PHY moved from a Gen 2 partner to a SuperSpeed one)."""

import unittest

from migen import *

from usb3_pipe.phy import USB3PHY

from sim import USB3SerDesModel

# Test ---------------------------------------------------------------------------------------------

class TestPHYSSPStatus(unittest.TestCase):
    def test_gen2_cleared_on_ss_fallback(self):
        # Gen 2 negotiated with a first partner (SSPPolling DONE, gen2 = 1), then a SuperSpeed
        # partner (SS-FALLBACK, gen2 = 0): the PHY gen2 (Gen 2 operation after the Polling) must
        # follow, else the PHY would switch to 10Gbps with a SuperSpeed partner.
        dut = USB3PHY(serdes=USB3SerDesModel(phy_dw=40), sys_clk_freq=int(125e6),
            with_endianness_swap=False, gen2_capable=True)
        ssp = dut.ssp
        def generator():
            # SSPPolling forced in DONE/SS-FALLBACK for a cycle (start from the LTSSM, not in
            # Polling: back to IDLE on the next cycle).
            # Gen 2 partner.
            yield ssp.gen2.eq(1)
            yield ssp.fsm.state.eq(ssp.fsm.encoding["DONE"])
            yield
            yield
            self.assertEqual((yield dut.gen2), 1)
            for _ in range(4):
                yield
            self.assertEqual((yield dut.gen2), 1) # Kept until the next SuperSpeedPlus Polling.
            # SuperSpeed partner.
            yield ssp.fsm.state.eq(ssp.fsm.encoding["SS-FALLBACK"])
            yield
            yield
            self.assertEqual((yield dut.gen2), 0)
            self.assertEqual((yield dut.x2), 0)
        ssp.finalize() # FSM encoding.
        run_simulation(dut, generator())

if __name__ == "__main__":
    unittest.main()
