#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM

from test.model.ltssm import LTSSMTestCase

SYS_CLK_FREQ = int(125e6)

def us_to_cycles(t):
    return int(t*1e-6*SYS_CLK_FREQ)

# LFPS Exit Handshake (USB 3.2 6.9.2, Table 6-31) --------------------------------------------------

class TestLTSSMExitHandshake(LTSSMTestCase):

    def enter_u1(self, dut):
        yield from self.drive_to_u0(dut)
        yield dut.lfps_rx_polling.eq(0) # Partner Polling.LFPS ended.
        # Partner LGO_U1 accepted (LAU), U1 entered on PM_ENTRY_TIMER timeout (7.2.4.2).
        yield dut.lgo_u1.eq(1)
        yield
        yield dut.lgo_u1.eq(0)
        self.assertTrue((yield from self.wait_for(dut, dut.u1, timeout=us_to_cycles(20))))
        for _ in range(us_to_cycles(4)): # U1_MIN_RESIDENCY.
            yield

    def test_u1_exit_responder_responds_while_partner_lfps_ongoing(self):
        # The initiator transmits LFPS until the handshake succeeds or fails: the responder must
        # respond while the initiator LFPS is still received (t11-t10 <= 2us for U1 exit).
        def generator(dut):
            yield from self.enter_u1(dut)
            yield dut.lfps_rx_exit_u1.eq(1) # Partner U1 exit LFPS (detected, ongoing).
            responded = yield from self.wait_for(dut, dut.lfps_tx_exit_u1, timeout=us_to_cycles(2))
            self.assertTrue(responded)
            for _ in range(us_to_cycles(1)): # Partner sees the response and stops.
                yield
            yield dut.lfps_rx_exit_u1.eq(0)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=us_to_cycles(20))))

        dut = USB3LTSSM(sys_clk_freq=SYS_CLK_FREQ)
        run_simulation(dut, generator(dut))

    def test_u1_exit_initiator_transmits_until_response(self):
        # Link partner 1 shall continue LFPS transmission until the handshake is declared
        # successful or failed (6.9.2): a late response (here 16us) must still see our LFPS.
        def generator(dut):
            yield from self.enter_u1(dut)
            yield dut.exit_u1_req.eq(1)
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_exit_u1, timeout=us_to_cycles(2))))
            yield dut.exit_u1_req.eq(0)
            for _ in range(us_to_cycles(16)):
                yield
            self.assertEqual((yield dut.lfps_tx_exit_u1), 1)
            yield dut.lfps_rx_exit_u1.eq(1) # Partner response.
            for _ in range(us_to_cycles(1)):
                yield
            yield dut.lfps_rx_exit_u1.eq(0)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=us_to_cycles(20))))

        dut = USB3LTSSM(sys_clk_freq=SYS_CLK_FREQ)
        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
