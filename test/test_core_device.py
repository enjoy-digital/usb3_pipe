#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core (Link/Protocol Layers, endpoints) against the host model scripts, in a
Migen simulation with a PHY stand-in (U0, data/ctrl words)."""

import unittest

import pytest

from migen import *

from litex.gen import *

from usb3_core.host_model import *

from test.model.device import BulkLoopbackDUT as DUT, check_core_enumeration

class TestCoreDevice(unittest.TestCase):
    def test_link_init(self):
        # Link initialization with the reactive host: Advertisements (LGOOD_7, LCRD_A-D), Port
        # Capability and Port Configuration Response LMPs (fast: no enumeration).
        dut  = DUT(max_burst=2)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            for _ in range(64):
                yield
            self.assertEqual(host.lcs[:5], [LGOOD(7)] + [LCRD(x) for x in range(4)])
            self.assertEqual([(hp[0] >> 5) & 0xf for hp in host.hps[:2]], [LMP_SUB_PORT_CAP, LMP_SUB_PORT_CFG_RESP])
        run_pyhost(dut, host, scenario())

    @pytest.mark.slow
    def test_enumeration(self):
        # Migen simulation (PHY stand-in, no SKP).
        script   = enumeration_script(skp=False)
        out      = play_script(DUT(), script)
        check_core_enumeration(self, parse_sim_output(out, script))

if __name__ == "__main__":
    unittest.main()
