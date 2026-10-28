#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core test designs and checks (Migen simulations with a PHY stand-in, Verilator
simulations with the host model)."""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.host_model  import *
from usb3_core.common      import endpoint_layout
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT

from test.model.host import tps, lmps, dpps

# Bulk Loopback Device -----------------------------------------------------------------------------

class PairsPHY:
    """PHY stand-in in words pairs (USB3PHY link_words=2) for the core, words on the host side
    (phy.source/phy.sink driven/monitored by the host models)."""
    def __init__(self, phy, module):
        from usb3_pipe.framing import WordsToPairs, PairsToWords
        for k, v in vars(phy).items():
            setattr(self, k, v)
        to_pairs = WordsToPairs()
        to_words = PairsToWords()
        module.submodules += to_pairs, to_words
        module.comb += [
            phy.source.connect(to_pairs.sink),
            to_words.source.connect(phy.sink),
        ]
        self.source = to_pairs.source
        self.sink   = to_words.sink

class BulkLoopbackDUT(LiteXModule):
    """USB3 Device Core (PHY stand-in) with Bulk IN/OUT endpoints (bulk_descriptors), EP2 OUT looped
    to EP1 IN. link_words=2: core on PHY words pairs, 64-bit Bulk endpoints."""
    def __init__(self, max_burst=8, link_words=1):
        self.phy = phy = PHYStandIn()
        dw = 32*link_words
        self.core = core = USB3DeviceCore(phy if link_words == 1 else PairsPHY(phy, self),
            sys_clk_freq = 125e6,
            descriptors  = bulk_descriptors(max_burst=max_burst),
            endpoints    = [
                USB3BulkIN(ept=1,  max_burst=max_burst, data_width=dw),
                USB3BulkOUT(ept=2, max_burst=max_burst, data_width=dw)])
        # Bulk loopback: EP2 OUT -> EP1 IN.
        self.loopback = loopback = stream.SyncFIFO(endpoint_layout(dw), 2*256, buffered=True)
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
