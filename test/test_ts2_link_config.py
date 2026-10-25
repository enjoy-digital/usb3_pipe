#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.common import TS1, TS2, LinkConfig
from usb3_pipe.core import USB3PIPE
from usb3_pipe.phy import USB3PHY

from test.common   import ordered_set_words
from test.model.gt import GTModel

# Helpers ------------------------------------------------------------------------------------------

def rx_words(*bursts):
    """Words of consecutive Ordered Set bursts: (ordered_set, link_config, n)."""
    words = []
    for ordered_set, link_config, n in bursts:
        words += ordered_set_words(ordered_set, link_config=link_config)*n
    return words

def drive_rx(gt, words, probe):
    """Drive the received words, return probe() sampled after the last one."""
    for data, ctrl in words:
        yield gt.source.valid.eq(1)
        yield gt.source.data.eq(data)
        yield gt.source.ctrl.eq(ctrl)
        yield
    yield gt.source.valid.eq(0)
    for _ in range(4):
        yield
    return (yield from probe())

# TS2 Link Functionality (USB 3.2 6.4.1.1.2 Table 6-6, 7.5.4.10.2, 7.5.12) -------------------------

RESET    = LinkConfig(reset=1, loopback=0, scrambling=1)
LOOPBACK = LinkConfig(reset=0, loopback=1, scrambling=1)
NORMAL   = LinkConfig(reset=0, loopback=0, scrambling=1)

class TestTS2LinkConfig(unittest.TestCase):
    def pipe_rx(self, words):
        gt  = GTModel(tx_ready=False, with_ready=True)
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt
        def probe():
            return ((yield dut.ts_rx_ts2_reset), (yield dut.ts_rx_ts2_loopback))
        def generator(dut):
            dut.result = yield from drive_rx(gt, words, probe)
        run_simulation(dut, generator(dut))
        return dut.result

    def test_pipe_reports_received_ts2_reset(self):
        # An upstream port shall transition to Hot Reset when the Reset bit is asserted in the
        # received TS2 (7.5.4.10.2): the PIPE must report it to the LTSSM.
        self.assertEqual(self.pipe_rx(rx_words((TS2, RESET, 8))), (1, 0))

    def test_pipe_reports_received_ts2_loopback(self):
        # The port shall transition to Loopback as a slave if the Loopback bit is asserted in the
        # received TS2 (7.5.4.10.2).
        self.assertEqual(self.pipe_rx(rx_words((TS2, LOOPBACK, 8))), (0, 1))

    def test_pipe_reports_ts2_reset_deassertion(self):
        # Hot Reset.Active exits on TS2 received with the Reset bit de-asserted (7.5.12.3.2).
        self.assertEqual(self.pipe_rx(rx_words((TS2, RESET, 8), (TS2, NORMAL, 4))), (0, 0))

    def test_ts1_does_not_change_ts2_link_config(self):
        # The Link Functionality bits are the TS2 ones (7.5.4.10.2): a TS1 (same 4 COM symbols as
        # a TS2) must not change them.
        self.assertEqual(self.pipe_rx(rx_words((TS2, NORMAL, 8), (TS1, RESET, 8))), (0, 0))

    def test_pipe_transmits_ts2_link_config(self):
        # A downstream port directed to Hot Reset sets the Reset bit in the TS2 ordered set, the
        # port directed to Loopback sets the Loopback bit (7.5.4.9.1, 7.5.12.3.1).
        for name, reset, loopback, expected in [
            ("reset",    1, 0, 0x01),
            ("loopback", 0, 1, 0x04),
            ("normal",   0, 0, 0x00)]:
            with self.subTest(name):
                gt  = GTModel(tx_ready=False, with_ready=True)
                dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
                dut.submodules += gt
                def generator(dut):
                    dut.link_config = []
                    yield gt.sink.ready.eq(1)
                    yield dut.ts_tx_enable.eq(1)
                    yield dut.ts_tx_ts2.eq(1)
                    yield dut.ts_tx_ts2_reset.eq(reset)
                    yield dut.ts_tx_ts2_loopback.eq(loopback)
                    for _ in range(4*16):
                        yield
                        if (yield gt.sink.valid) and (yield gt.sink.ctrl) == 0b1111:
                            yield
                            dut.link_config.append(((yield gt.sink.data) >> 8) & 0xff)
                run_simulation(dut, generator(dut))
                self.assertTrue(len(dut.link_config) >= 8)
                self.assertEqual(set(dut.link_config), {expected})

    def test_phy_connects_ts2_link_config_to_ltssm(self):
        # The LTSSM Hot Reset/Loopback transitions use the decoded TS2 bits: USB3PHY must connect
        # them (rx: PIPE -> LTSSM, tx: LTSSM -> PIPE).
        gt  = GTModel(tx_ready=False, with_ready=True)
        dut = USB3PHY(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt
        def probe():
            return ((yield dut.ltssm.rx_ts2_reset), (yield dut.ltssm.rx_ts2_loopback))
        def generator(dut):
            dut.rx_reset = yield from drive_rx(gt, rx_words((TS2, RESET, 8)), probe)
            dut.rx_loopback = yield from drive_rx(gt, rx_words((TS2, LOOPBACK, 8)), probe)
        run_simulation(dut, generator(dut))
        self.assertEqual(dut.rx_reset,    (1, 0))
        self.assertEqual(dut.rx_loopback, (0, 1))

        # TX: the ts_tx_ts2_reset/loopback PIPE inputs must be driven by the LTSSM outputs.
        dut  = USB3PHY(serdes=GTModel(tx_ready=False, with_ready=True), sys_clk_freq=int(125e6), with_endianness_swap=False)
        comb = dut.get_fragment().comb
        for name in ["reset", "loopback"]:
            pipe_input   = getattr(dut.pipe,  "ts_tx_ts2_" + name)
            ltssm_output = getattr(dut.ltssm, "tx_ts2_" + name)
            self.assertTrue(any(getattr(s, "l", None) is pipe_input and s.r is ltssm_output
                for s in comb), name)

if __name__ == "__main__":
    unittest.main()
