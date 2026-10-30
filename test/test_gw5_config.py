#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GTR12 USB3 register configurations (usb3_pipe/gw5): defaults and GW5AT-15 overrides."""

import re
import unittest

from usb3_pipe.gw5 import gw5_usb3_csr_config, gw5_csr_overrides, GW5AT15_Q0_QPLL0_OVERRIDES

def writes(csr_config):
    return {int(a, 16): int(v, 16) for a, v in re.findall(r"upar_write_driver\((0x[0-9A-Fa-f]+),(0x[0-9A-Fa-f]+)\)",
        csr_config)}

class TestGW5Config(unittest.TestCase):
    def test_gw5ast138_default(self):
        # Q1 (GW5AST-138B): QPLL1 by default, no overrides.
        self.assertEqual(gw5_usb3_csr_config(0, "5G", "100M"), gw5_usb3_csr_config(0, "5G", "100M", pll="QPLL1"))
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

    def test_overrides(self):
        c = "upar_write_driver(0x000010,0x00000001) # a\nupar_write_driver(0x000020,0x00000002) # b\n"
        self.assertEqual(writes(gw5_csr_overrides(c, {0x10: 0x5, 0x20: None})), {0x10: 0x5})
        with self.assertRaises(AssertionError):
            gw5_csr_overrides(c, {0x30: 0x1}) # Not written: refused.

if __name__ == "__main__":
    unittest.main()
