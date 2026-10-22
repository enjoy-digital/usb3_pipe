#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM

from test import test_ltssm

SYS_CLK_FREQ = int(125e6)

# TX Electrical Idle (USB 3.2 7.5.4.x, 7.5.6, 7.5.7) -----------------------------------------------

class TestLTSSMTXElectricalIdle(unittest.TestCase):
    wait_for = test_ltssm.TestLTSSM.wait_for

    def test_tx_not_idle_while_transmitting(self):
        # TSEQ (Polling.RxEQ), TS1 (Polling.Active), TS2 (Polling.Configuration), Idle Symbols
        # (Polling.Idle) and U0 data are SuperSpeed transmissions: the transmitter must not be in
        # electrical idle. In U1 the transmitter is in electrical idle.
        def generator(dut):
            yield dut.lfps_tx_count.eq(20)
            yield dut.lfps_rx_polling.eq(1)
            yield
            yield dut.lfps_tx_count.eq(24)
            yield
            for tx_signal, rx_signal in [
                (dut.ts_tx_tseq, None),
                (dut.ts_tx_ts1,  dut.ts_rx_ts1),
                (dut.ts_tx_ts2,  dut.ts_rx_ts2)]:
                self.assertTrue((yield from self.wait_for(dut, tx_signal)))
                yield
                self.assertEqual((yield dut.lfps_tx_idle), 0)
                if rx_signal is not None:
                    yield rx_signal.eq(1)
                    yield
                    yield rx_signal.eq(0)
                    yield
                yield dut.ts_tx_done.eq(1)
                yield
                yield dut.ts_tx_done.eq(0)
            self.assertTrue((yield from self.wait_for(dut, dut.u0)))
            yield dut.lfps_rx_polling.eq(0)
            yield
            self.assertEqual((yield dut.lfps_tx_idle), 0)
            # U1 (partner LGO_U1 accepted).
            yield dut.lgo_u1.eq(1)
            yield
            yield dut.lgo_u1.eq(0)
            self.assertTrue((yield from self.wait_for(dut, dut.u1, timeout=4000)))
            yield
            self.assertEqual((yield dut.lfps_tx_idle), 1)

        dut = USB3LTSSM(sys_clk_freq=SYS_CLK_FREQ)
        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
