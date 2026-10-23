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

from litex.soc.interconnect import stream

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self, max_burst=8):
        self.phy = phy = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors = bulk_descriptors(max_burst=max_burst),
            endpoints   = [USB3BulkIN(ept=1, max_burst=max_burst), USB3BulkOUT(ept=2, max_burst=max_burst)])
        # Bulk loopback: EP2 OUT -> EP1 IN.
        self.loopback = loopback = stream.SyncFIFO([("data", 32), ("last_be", 4)], 2*256, buffered=True)
        self.comb += [
            core.endpoints[1].source.connect(loopback.sink),
            loopback.source.connect(core.endpoints[0].sink),
        ]

# Helpers ------------------------------------------------------------------------------------------

def check_core_enumeration(test, sections):
    """Enumeration + Bulk loopback checks of the USB3 Device Core (bulk_descriptors)."""
    # Packets integrity (CRCs).
    for name, packets in sections.items():
        for packet in packets:
            test.assertTrue(packet[-1], f"{name}: bad CRC {packet}")

    # Link initialization: Advertisements, Port Capability, Port Configuration Response.
    link_init = sections["link_init"]
    lcs = [p[1] for p in link_init if p[0] == "LC"]
    test.assertEqual(lcs[:5], [LGOOD(7)] + [LCRD(x) for x in range(4)])
    test.assertEqual(lmps(link_init), [LMP_SUB_PORT_CAP, LMP_SUB_PORT_CFG_RESP])

    # GET_DESCRIPTOR (Device).
    descriptors = bulk_descriptors()
    packets = sections["get_device_descriptor"]
    test.assertEqual(tps(packets), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])
    test.assertEqual(dpps(packets), [descriptors.device()])

    # GET_DESCRIPTOR (Configuration, 9 bytes).
    test.assertEqual(dpps(sections["get_configuration_descriptor"]), [descriptors.configuration()[:9]])

    # GET_DESCRIPTOR (String 1).
    test.assertEqual(dpps(sections["get_string_descriptor"]), [descriptors.string(1)])

    # SET_CONFIGURATION.
    test.assertEqual(tps(sections["set_configuration"]), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])

    # Bulk OUT -> loopback -> Bulk IN.
    for i, payload in enumerate(bulk_payloads()):
        acks = tps(sections[f"bulk_out_{i}"])
        test.assertEqual(len(acks), 1)
        test.assertEqual((acks[0][0], acks[0][2]), (TP_SUB_ACK, i + 1))
        test.assertEqual(dpps(sections[f"bulk_in_{i}"]), [payload])


class TestCoreDevice(unittest.TestCase):
    @pytest.mark.slow
    def test_enumeration(self):
        # Migen simulation (PHY stand-in, no SKP).
        script   = enumeration_script(skp=False)
        out      = play_script(DUT(), script)
        check_core_enumeration(self, parse_sim_output(out, script))

if __name__ == "__main__":
    unittest.main()
