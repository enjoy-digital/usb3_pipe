#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Control Endpoint Wishbone bridge: CommUSB requests (single/burst reads and writes, invalid
requests stalled) on a Wishbone SRAM (Migen simulation, reactive host)."""

import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore
from usb3_core.wishbone    import USB3EP0Wishbone

from test.model.host import *

# DUT (Device + Wishbone bridge + SRAM) ------------------------------------------------------------

SRAM_INIT = [(0x01020304*i + 0x11223344) & 0xffffffff for i in range(256)]

class DUT(LiteXModule):
    def __init__(self):
        self.phy  = phy  = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors      = bulk_descriptors(),
            with_ep0_handler = True)
        self.bridge = bridge = USB3EP0Wishbone(core.ep0)
        self.sram   = sram   = wishbone.SRAM(4*len(SRAM_INIT), init=SRAM_INIT)
        self.comb += bridge.bus.connect(sram.bus)

# Helpers ------------------------------------------------------------------------------------------

def to_bytes(words):
    return [b for w in words for b in w.to_bytes(4, "little")]

def to_words(data):
    return [int.from_bytes(bytes(data[i:i+4]), "little") for i in range(0, len(data), 4)]

def wb_read(host, addr, n=1):
    data = yield from pyhost_control_in(host,
        setup_packet(0xc3, 0x00, addr & 0xffff, (addr >> 16) & 0xffff, 4*n))
    return None if data is None else to_words(data)

def wb_write(host, addr, words):
    ok = yield from pyhost_control_out(host,
        setup_packet(0x43, 0x00, addr & 0xffff, (addr >> 16) & 0xffff, 4*len(words)),
        to_bytes(words))
    return ok

# Tests --------------------------------------------------------------------------------------------

class TestCoreWishbone(unittest.TestCase):
    def run_scenario(self, scenario):
        dut  = DUT()
        host = PyHost(dut.phy)
        def wrapper():
            yield from pyhost_link_init(host)
            yield from scenario(host)
        run_simulation(dut, [host.driver(), host.monitor(), wrapper()])

    def test_single_read(self):
        # CommUSB read: 4 bytes at a byte address.
        def scenario(host):
            self.assertEqual((yield from wb_read(host, 0x0000)), [SRAM_INIT[0]])
            self.assertEqual((yield from wb_read(host, 0x0014)), [SRAM_INIT[5]])
        self.run_scenario(scenario)

    def test_single_write_read(self):
        # CommUSB write then read back (writes executed before the following read).
        def scenario(host):
            self.assertTrue((yield from wb_write(host, 0x0020, [0xdeadbeef])))
            self.assertEqual((yield from wb_read(host, 0x0020)), [0xdeadbeef])
            self.assertEqual((yield from wb_read(host, 0x0024)), [SRAM_INIT[9]])
        self.run_scenario(scenario)

    def test_burst(self):
        # Incrementing bursts: full size (128 words) write/read, then a short read.
        words = [(0x9e3779b9*i) & 0xffffffff for i in range(128)]
        def scenario(host):
            self.assertTrue((yield from wb_write(host, 0x0100, words)))
            self.assertEqual((yield from wb_read(host, 0x0100, 128)), words)
            self.assertEqual((yield from wb_read(host, 0x0000, 3)), SRAM_INIT[:3])
        self.run_scenario(scenario)

    def test_invalid_requests_stalled(self):
        # Unaligned length, oversized burst and unknown bRequest stalled, bridge still usable.
        def scenario(host):
            data = yield from pyhost_control_in(host, setup_packet(0xc3, 0x00, 0, 0, 6))
            self.assertIsNone(data)
            data = yield from pyhost_control_in(host, setup_packet(0xc3, 0x00, 0, 0, 516))
            self.assertIsNone(data)
            data = yield from pyhost_control_in(host, setup_packet(0xc3, 0x01, 0, 0, 4))
            self.assertIsNone(data)
            self.assertEqual((yield from wb_read(host, 0x0008)), [SRAM_INIT[2]])
        self.run_scenario(scenario)

if __name__ == "__main__":
    unittest.main()
