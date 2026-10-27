#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.fhdl.specials import Instance

from litex.build.xilinx import XilinxPlatform

from usb3_pipe import A7USB3SerDes

# Helpers ------------------------------------------------------------------------------------------

def _pads(*names):
    return Record([(name, 1) for name in names])

def _instance(fragment, of):
    instances = [s for s in fragment.specials if isinstance(s, Instance) and s.of == of]
    assert len(instances) == 1
    return instances[0]

def _input(instance, name):
    for item in instance.items:
        if isinstance(item, Instance.Input) and item.name == name:
            return item.expr
    raise KeyError(name)

# Test SerDes --------------------------------------------------------------------------------------

class TestSerDes(unittest.TestCase):
    def a7_common(self, refclk_pads):
        platform = XilinxPlatform("xc7a200t-fbg484-3", io=[])
        serdes   = A7USB3SerDes(platform,
            sys_clk      = Signal(),
            sys_clk_freq = 125e6,
            refclk_pads  = refclk_pads,
            refclk_freq  = 200e6 if isinstance(refclk_pads, ClockSignal) else 100e6,
            tx_pads      = _pads("p", "n"),
            rx_pads      = _pads("p", "n"))
        return _instance(serdes.get_fragment(), "GTPE2_COMMON")

    def test_a7_refclk_from_pads(self):
        # Dedicated pads (IBUFDS_GTE2): GTREFCLK0.
        common = self.a7_common(_pads("p", "n"))
        self.assertIsInstance(_input(common, "GTREFCLK0"), Signal)
        self.assertEqual(_input(common, "GTGREFCLK0").value, 0)
        self.assertEqual(_input(common, "PLL0REFCLKSEL").value, 0b001)

    def test_a7_refclk_from_fabric(self):
        # Fabric clock: GTGREFCLK0 (clock network), not GTREFCLK0 (general interconnect).
        common = self.a7_common(ClockSignal("clk200"))
        self.assertIsInstance(_input(common, "GTGREFCLK0"), ClockSignal)
        self.assertEqual(_input(common, "GTREFCLK0").value, 0)
        self.assertEqual(_input(common, "PLL0REFCLKSEL").value, 0b111)

    def test_a7_comma_alignment_any_byte(self):
        # USB3 Ordered Sets start with consecutive COMs: commas aligned on any byte.
        platform = XilinxPlatform("xc7a200t-fbg484-3", io=[])
        serdes   = A7USB3SerDes(platform,
            sys_clk      = Signal(),
            sys_clk_freq = 125e6,
            refclk_pads  = _pads("p", "n"),
            refclk_freq  = 100e6,
            tx_pads      = _pads("p", "n"),
            rx_pads      = _pads("p", "n"))
        channel = _instance(serdes.get_fragment(), "GTPE2_CHANNEL")
        word = [item.value for item in channel.items
            if isinstance(item, Instance.Parameter) and item.name == "ALIGN_COMMA_WORD"]
        self.assertEqual([w.value if hasattr(w, "value") else w for w in word], [1])

if __name__ == "__main__":
    unittest.main()
