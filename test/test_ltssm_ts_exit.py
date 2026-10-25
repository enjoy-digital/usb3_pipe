#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM

from test.model.ltssm import LTSSMTestCase, wait_for, pulse

# Helpers ------------------------------------------------------------------------------------------

def in_state(dut, name):
    return (yield dut.fsm.state) == dut.fsm.encoding[name]

def run(generator):
    dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)
    dut.finalize()
    run_simulation(dut, generator(dut))

def drive_to_polling_active(dut):
    yield dut.lfps_tx_count.eq(20)
    yield dut.lfps_rx_polling.eq(1)
    yield
    yield dut.lfps_tx_count.eq(24)
    yield
    assert (yield from wait_for(dut, dut.ts_tx_tseq))
    yield from pulse(dut.ts_tx_done)
    assert (yield from wait_for(dut, dut.ts_tx_ts1))
    yield dut.lfps_rx_polling.eq(0)

# TS exit conditions (USB 3.2 7.5.4.8.2, 7.5.4.9.2, 7.5.10.4.2) ------------------------------------

class TestLTSSMTSExit(LTSSMTestCase):
    def test_polling_active_exits_on_ts2(self):
        # The port shall transition to Polling.Configuration upon receiving eight consecutive and
        # identical TS1 or TS2 ordered sets (7.5.4.8.2): the partner may already be in
        # Polling.Configuration, sending TS2.
        def generator(dut):
            yield from drive_to_polling_active(dut)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield
            self.assertTrue((yield from in_state(dut, "Polling.Configuration")))
        run(generator)

    def check_configuration_exit(self, dut, config_state, idle_state):
        # In Configuration, a burst of 16 TS2 is being sent. 8 consecutive TS2 are detected near
        # the end of this burst: 16 TS2 must still be sent after the first of them (7.5.4.9.2
        # condition 2), so the end of the current burst is too early.
        self.assertTrue((yield from in_state(dut, config_state)))
        yield from pulse(dut.ts_tx_done)  # TS2 burst without partner TS2.
        yield from pulse(dut.ts_rx_ts2)   # 8 consecutive TS2 received...
        yield from pulse(dut.ts_tx_done)  # ...at the end of a burst: only a few TS2 sent after.
        yield
        self.assertTrue((yield from in_state(dut, config_state)))
        yield from pulse(dut.ts_tx_done)  # 16 more TS2 sent.
        yield
        self.assertTrue((yield from in_state(dut, idle_state)))

    def test_polling_configuration_sends_16_ts2_after_receiving(self):
        def generator(dut):
            yield dut.rx_idle8.eq(0)
            yield from drive_to_polling_active(dut)
            yield from pulse(dut.ts_rx_ts1)
            yield from pulse(dut.ts_tx_done)
            yield
            yield from self.check_configuration_exit(dut, "Polling.Configuration", "Polling.Idle")
        run(generator)

    def test_polling_configuration_ts2_received_in_active(self):
        # 8 consecutive TS2 already received in Polling.Active: the whole first TS2 burst of
        # Polling.Configuration is sent after them.
        def generator(dut):
            yield dut.rx_idle8.eq(0)
            yield from drive_to_polling_active(dut)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield
            self.assertTrue((yield from in_state(dut, "Polling.Configuration")))
            yield from pulse(dut.ts_tx_done)
            yield
            self.assertTrue((yield from in_state(dut, "Polling.Idle")))
        run(generator)

    def test_recovery_configuration_sends_16_ts2_after_receiving(self):
        def generator(dut):
            yield from self.drive_to_u0(dut)
            yield dut.rx_idle8.eq(0)
            yield from pulse(dut.directed_recovery)
            self.assertTrue((yield from self.wait_for(dut, dut.ts_tx_ts1)))
            yield from pulse(dut.ts_rx_ts1)
            yield from pulse(dut.ts_tx_done)
            yield
            yield from self.check_configuration_exit(dut, "Recovery.Configuration", "Recovery.Idle")
        run(generator)

if __name__ == "__main__":
    unittest.main()
