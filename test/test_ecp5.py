#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.gen import *

from liteiclink.serdes import serdes_ecp5

from usb3_pipe.serdes import ECP5USB3SerDes

# Helpers ------------------------------------------------------------------------------------------

class Pads:
    def __init__(self):
        self.p = Signal()
        self.n = Signal()

class Platform:
    def __init__(self):
        self.period_constraints = []

    def add_period_constraint(self, clk, period):
        self.period_constraints.append((clk, period))

    def add_false_path_constraints(self, *clks):
        pass

def ecp5_usb3_serdes(platform=None, **kwargs):
    return ECP5USB3SerDes(platform or Platform(),
        sys_clk      = Signal(),
        sys_clk_freq = 125e6,
        refclk       = Signal(),
        refclk_freq  = 250e6,
        tx_pads      = Pads(),
        rx_pads      = Pads(),
        **kwargs)

# ECP5USB3SerDes -----------------------------------------------------------------------------------

@unittest.skipUnless(hasattr(serdes_ecp5, "SerDesECP5SymbolAligner"),
    "LiteICLink SerDesECP5 without 40-bit support.")
class TestECP5USB3SerDes(unittest.TestCase):
    def test_dcu(self):
        platform = Platform()
        serdes   = ecp5_usb3_serdes(platform)
        dcu      = serdes.dcu
        # 5Gbps from 250MHz x20, 40-bit words at 125MHz, DCU clocks constrained at 250MHz.
        self.assertEqual((serdes.pll.config["m"], serdes.pll.config["d"]), (20, 1))
        self.assertEqual(dcu.nwords, 4)
        self.assertEqual(dcu.tx_clk_freq, 125e6)
        self.assertEqual(dcu.rx_clk_freq, 125e6)
        (tx_clk, tx_period), (rx_clk, rx_period) = platform.period_constraints
        self.assertIs(tx_clk, dcu.txoutclk)
        self.assertIs(rx_clk, dcu.rxoutclk)
        self.assertEqual((tx_period, rx_period), (4.0, 4.0))
        # Separate TX/RX initialization and TX Electrical Idle from FFC_EI_EN (OOB/LFPS).
        self.assertEqual(dcu.serdes_params["p_CHX_LSM_DISABLE"], "0b1")
        self.assertIn("i_CHX_FFC_EI_EN", dcu.serdes_params)

    def test_rx_settings(self):
        dcu = ecp5_usb3_serdes().dcu
        for k, v in ECP5USB3SerDes.rx_settings.items():
            with self.subTest(param=k):
                self.assertIs(dcu.serdes_params[k], v)

    def test_pipe_cd(self):
        # DCU control logic in its own clock domain (PIPE on the DCU TX clock, USB3PHY pipe_cd).
        serdes = ecp5_usb3_serdes(xcvr_cd="dcu_ctrl", tx_cdc=False)
        self.assertEqual(serdes.dcu.tx_clk_freq, 125e6)

if __name__ == "__main__":
    unittest.main()
