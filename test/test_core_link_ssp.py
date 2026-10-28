#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Link Layer in SuperSpeedPlus operation (PHY stand-in at Gen 2): Type 1/Type 2 Tx Header
Buffers (7.2.4.1: 4 unacknowledged Type 1 Header Packets and 4 unacknowledged Type 2 DPHs)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.common     import *
from usb3_core.host_model import *
from usb3_core.link       import USB3LinkLayer

# DUT ----------------------------------------------------------------------------------------------

class LinkDUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        phy.gen2_active = Signal(reset=1)
        self.link = USB3LinkLayer(phy, sys_clk_freq=125e6, ssp_tt={1: TT_BULK})

# Test ---------------------------------------------------------------------------------------------

class TestCoreLinkSSP(unittest.TestCase):
    def test_tx_header_buffers_types(self):
        # The host advertises its credits but never acknowledges (no LGOOD_n): the Port Capability
        # LMP and 3 ACK TPs fill the Type 1 buffers, 4 DPHs (TT Bulk) are still accepted in the Type 2
        # buffers, the 5th Type 1/Type 2 Header Packets are not.
        dut  = LinkDUT()
        link = dut.link
        phy  = dut.phy
        accepted = []

        def host():
            words = link_command(LGOOD(15))
            for x in range(4):
                words += link_command(LCRD(x)) + link_command(LCRD2(x))
            yield phy.polling_idle.eq(1)
            yield
            yield phy.polling_idle.eq(0)
            yield phy.u0.eq(1)
            yield phy.sink.ready.eq(1)
            for data, ctrl in words:
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)

        def push(dw0, dw1, dpp=0, timeout=256):
            yield link.tx_hp.valid.eq(1)
            yield link.tx_hp.dw0.eq(dw0)
            yield link.tx_hp.dw1.eq(dw1)
            yield link.tx_hp.dpp.eq(dpp)
            yield
            for _ in range(timeout):
                if (yield link.tx_hp.ready):
                    yield link.tx_hp.valid.eq(0)
                    yield
                    return True
                yield
            yield link.tx_hp.valid.eq(0)
            yield
            return False

        def dpps():
            # Zero-length DPPs for the DPHs.
            yield "passive"
            yield link.tx_dpp.valid.eq(1)
            yield link.tx_dpp.bytes.eq(0)
            yield link.tx_dpp.last.eq(1)
            while True:
                yield

        def scenario():
            for _ in range(128):
                yield
            ack = tp_dw1(TP_SUB_ACK, ept=2, nump=1)
            dph = dph_dw1(seq=0, ept=1, direction=1, length=0)
            for _ in range(3):
                accepted.append(("ack", (yield from push(TYPE_TP, ack))))
            for _ in range(4):
                accepted.append(("dph", (yield from push(TYPE_DP, dph, dpp=1))))
            accepted.append(("ack", (yield from push(TYPE_TP, ack))))
            accepted.append(("dph", (yield from push(TYPE_DP, dph, dpp=1))))

        run_simulation(dut, [host(), dpps(), scenario()])
        self.assertEqual(accepted, [("ack", True)]*3 + [("dph", True)]*4 + [("ack", False), ("dph", False)])

if __name__ == "__main__":
    unittest.main()
