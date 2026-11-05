#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GTR12 USB3 register configurations (usb3_pipe/gw5): defaults, GW5AT-15 overrides, CPLL
5G/10G configurations and rate change writes."""

import re
import unittest

from usb3_pipe.gw5 import gw5_usb3_csr_config, gw5_usb3_rate_writes, gw5_csr_overrides
from usb3_pipe.gw5 import GW5AT15_Q0_QPLL0_OVERRIDES

def writes(csr_config):
    return {int(a, 16): int(v, 16) for a, v in re.findall(
        r"upar_write_driver\((0x[0-9A-Fa-f]+),(0x[0-9A-Fa-f]+)\)", csr_config)}

class TestGW5Config(unittest.TestCase):
    def test_gw5ast138_default(self):
        # Q1 (GW5AST-138B): QPLL1 by default, no overrides.
        self.assertEqual(gw5_usb3_csr_config(0, "5G", "100M"),
            gw5_usb3_csr_config(0, "5G", "100M", pll="QPLL1"))
        with self.assertRaises(AssertionError):
            gw5_usb3_csr_config(0, "5G", "100M", pll="QPLL0")

    def test_gw5at15_default(self):
        # Q0 (GW5AT-15): QPLL0 by default with Gowin's USB 3.0 PHY IP settings.
        c = writes(gw5_usb3_csr_config(0, "5G", "125M", position="Q0"))
        for addr, value in GW5AT15_Q0_QPLL0_OVERRIDES.items():
            if value is None:
                self.assertNotIn(addr, c)
            else:
                self.assertEqual(c[addr], value)
        # QPLL1 (no link on hardware): generated configuration, no overrides.
        c1 = gw5_usb3_csr_config(0, "5G", "125M", position="Q0", pll="QPLL1")
        self.assertNotIn("PHY IP", c1)

    def test_cpll(self):
        # CPLL (Q1 lane 0, 125MHz REFCLK0): CDR VCO bias override at both rates, manual RX equalizer
        # at 10G only.
        c5  = writes(gw5_usb3_csr_config(0, "5G",  "125M", pll="CPLL"))
        c10 = writes(gw5_usb3_csr_config(0, "10G", "125M", pll="CPLL"))
        self.assertEqual(c5[0x90025c], 0x7d)
        self.assertEqual(c10[0x90025c], 0x7d)
        self.assertNotIn(0x9082d0, c5)
        self.assertEqual(c10[0x9082d0], 0xcccc1)
        with self.assertRaises(AssertionError):
            gw5_usb3_csr_config(0, "10G", "100M", pll="CPLL")

    def test_cpll_rate_writes(self):
        # CPLL rate change: CPLL reset, the differing writes, RX equalizer mode, signal detect
        # toggle, CPLL released after 50us, 200us for its lock (short: Polling.RxEQ of the partner).
        w = gw5_usb3_rate_writes(0, "125M", pll="CPLL", clk_freq=100e6)
        self.assertEqual(w[0], (0x90a004, 0x22322, 0x22322))
        self.assertIn((0x90a00c, 0x10140108, 0x10280108), w)
        self.assertEqual([x[3] for x in w if len(x) == 4], [5000, 20000])
        self.assertEqual(w[-2][:3], (0x90a004, 0x22233, 0x22233))
        self.assertEqual(w[-1], (0x90033e, 0, 0))
        # Equalizer registers back to 0 at 5G (as after the 5G configuration), attenuation 8 at 10G.
        eq = {a: (v5, v10) for a, v5, v10, *_ in w if a in (0x9082d0, 0x9082d4, 0x9003a2)}
        self.assertEqual(eq,
            {0x9082d0: (0, 0xcccc1), 0x9082d4: (0, 0x88881), 0x9003a2: (0, 0x10000)})

    def test_overrides(self):
        c = ("upar_write_driver(0x000010,0x00000001) # a\n"
            "upar_write_driver(0x000020,0x00000002) # b\n")
        self.assertEqual(writes(gw5_csr_overrides(c, {0x10: 0x5, 0x20: None})), {0x10: 0x5})
        with self.assertRaises(AssertionError):
            gw5_csr_overrides(c, {0x30: 0x1}) # Not written: refused.

if __name__ == "__main__":
    unittest.main()
