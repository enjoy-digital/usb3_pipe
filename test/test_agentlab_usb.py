#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab USB3 composite device: descriptors consistency (interfaces, IADs, endpoints), and
(Migen simulation, reactive host) enumeration, CDC-ACM class requests answered by the function of
their interface, DUT UART bytes through the CDC-ACM data endpoints (internal loopback UART) and
events records on the events Bulk IN endpoint."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.descriptors import *

from agentlab.gateware.usb        import *
from agentlab.gateware.instrument import AgentLab
from agentlab.gateware.events     import *

from test.model.host import *

# Helpers ------------------------------------------------------------------------------------------

def parse_descriptors(data):
    """Configuration descriptor to a list of descriptors (bytes lists)."""
    r = []
    i = 0
    while i < len(data):
        length = data[i]
        assert length >= 2
        r.append(data[i:i + length])
        i += length
    assert i == len(data)
    return r

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self):
        self.phy      = phy = PHYStandIn()
        self.usb      = usb = AgentLabUSB(phy, sys_clk_freq=125e6)
        self.agentlab = agentlab = AgentLab(sys_clk_freq=125e6, uart_pads=[None]*4)
        self.comb += agentlab_usb_connect(usb, agentlab)

# Tests --------------------------------------------------------------------------------------------

class TestAgentLabDescriptors(unittest.TestCase):
    def test_descriptors(self):
        m           = agentlab_usb_map()
        descriptors = agentlab_descriptors()
        device      = descriptors.device()
        self.assertEqual(device[4:7], [0xef, 0x02, 0x01]) # IADs.
        config = descriptors.configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertLessEqual(len(config), 512) # One EP0 packet.
        descs = parse_descriptors(config)[1:]
        iads       = [d for d in descs if d[1] == DESC_IAD]
        interfaces = [d for d in descs if d[1] == DESC_INTERFACE]
        endpoints  = [d for d in descs if d[1] == DESC_ENDPOINT]
        # Interfaces: 0-10, contiguous, bNumInterfaces.
        self.assertEqual([d[2] for d in interfaces], list(range(11)))
        self.assertEqual(config[4], 11)
        # IADs: one per CDC-ACM function (control + 4 DUT UARTs), 2 interfaces each.
        self.assertEqual([(d[2], d[3], d[4]) for d in iads],
            [(2*n, 2, 0x02) for n in range(5)])
        # Interfaces endpoints count matches the endpoints descriptors.
        self.assertEqual(sum(d[4] for d in interfaces), len(endpoints))
        # Endpoints: unique addresses, <= 15 IN/OUT.
        addresses = [d[2] for d in endpoints]
        self.assertEqual(len(addresses), len(set(addresses)))
        self.assertLessEqual(len([a for a in addresses if a & 0x80]), 15)
        self.assertLessEqual(len([a for a in addresses if not a & 0x80]), 15)
        # Map (doc/agentlab.md).
        self.assertEqual(m["control"], {"interface": 0, "notif_ept": 1, "in_ept": 2, "out_ept": 2})
        for n in range(4):
            self.assertEqual(m[f"uart{n}"], {"interface": 2 + 2*n, "notif_ept": 3 + n,
                "in_ept": 7 + n, "out_ept": 7 + n})
        self.assertEqual(m["events"], {"interface": 10, "in_ept": 11})
        events = [d for d in interfaces if d[2] == 10][0]
        self.assertEqual(events[5], 0xff) # Vendor.
        self.assertIn(0x80 | 11, addresses)

    def test_core_endpoints(self):
        # Every endpoint descriptor has its core endpoint (and conversely).
        for with_la in [False, True]:
            dut = AgentLabUSB(PHYStandIn(), sys_clk_freq=125e6, with_la=with_la)
            descs = parse_descriptors(agentlab_descriptors(with_la=with_la).configuration())
            addresses = sorted(d[2] for d in descs if d[1] == DESC_ENDPOINT)
            core      = sorted(ep.ept | (ep.direction << 7) for ep in dut.core.endpoints)
            self.assertEqual(addresses, core)

    def test_descriptors_la(self):
        # Logic analyzer: vendor interface 11, EP12 IN bulk, Max Burst 16.
        m = agentlab_usb_map(with_la=True)
        self.assertEqual(m["la"], {"interface": 11, "in_ept": 12})
        config = agentlab_descriptors(with_la=True).configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertLessEqual(len(config), 512)
        self.assertEqual(config[4], 12)
        descs      = parse_descriptors(config)[1:]
        interfaces = [d for d in descs if d[1] == DESC_INTERFACE]
        self.assertEqual([d[2] for d in interfaces], list(range(12)))
        la = descs.index([d for d in interfaces if d[2] == 11][0])
        self.assertEqual((descs[la][4], descs[la][5]), (1, 0xff))
        ep, comp = descs[la + 1], descs[la + 2]
        self.assertEqual((ep[1], ep[2], ep[3]), (DESC_ENDPOINT, 0x80 | 12, 0x02))
        self.assertEqual(comp[2], 16 - 1) # bMaxBurst.

    def test_descriptors_la_eyes(self):
        # Logic analyzer and Eyes: LA interface 11/EP12, Eyes (VC+VS) 12-13/EP14, contiguous.
        m = agentlab_usb_map(with_la=True, with_eyes=True)
        self.assertEqual(m["la"],   {"interface": 11, "in_ept": 12})
        self.assertEqual(m["eyes"], {"interface": 12, "in_ept": 14})
        config = agentlab_descriptors(with_la=True, with_eyes=True).configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertEqual(config[4], 14)
        descs      = parse_descriptors(config)[1:]
        interfaces = [d for d in descs if d[1] == DESC_INTERFACE]
        self.assertEqual(sorted(set(d[2] for d in interfaces)), list(range(14)))
        dut = AgentLabUSB(PHYStandIn(), sys_clk_freq=125e6, with_la=True, with_eyes=True)
        addresses = sorted(d[2] for d in descs if d[1] == DESC_ENDPOINT)
        core      = sorted(ep.ept | (ep.direction << 7) for ep in dut.core.endpoints)
        self.assertEqual(addresses, core)

class TestAgentLabUSB(unittest.TestCase):
    def test_enumeration_requests_uart_events(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            # Descriptors.
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, agentlab_descriptors().device())
            config = agentlab_descriptors().configuration()
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0200, 0,
                len(config)))
            self.assertEqual(data, config)
            ok = yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))
            self.assertTrue(ok)

            # DUT UART0 (interface 2): 12.5Mbaud (sys_clk/10), DTR.
            coding = list((12500000).to_bytes(4, "little")) + [0, 0, 8]
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x20, 0, 2, 7), coding)
            self.assertTrue(ok)
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x22, 0x0001, 2, 0))
            self.assertTrue(ok)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.usb.uarts[0].baudrate), 12500000)
            self.assertEqual((yield dut.usb.uarts[1].baudrate), 115200)
            self.assertEqual((yield dut.usb.uarts[0].connect), 1)
            self.assertEqual((yield dut.usb.uarts[1].connect), 0)
            self.assertEqual((yield dut.usb.control.connect), 0)
            # Line coding of the other functions untouched.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 4, 7))
            self.assertEqual(data, list((115200).to_bytes(4, "little")) + [0, 0, 8])
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 2, 7))
            self.assertEqual(data, coding)
            # Class request to the events (vendor) interface: stalled.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 10, 7))
            self.assertIsNone(data)

            # Marker event.
            yield dut.agentlab._marker.storage.eq(0x12345678)
            yield dut.agentlab._marker.re.eq(1)
            yield
            yield dut.agentlab._marker.re.eq(0)

            # DUT UART0: OUT bytes looped back (internal loopback UART), IN.
            message = list(b"Hi!")
            host.send_hp(dph, length=len(message), ept=7, dp_seq=0)
            host.send(data_packet_payload(message))
            # (Bytes 100 cycles apart: one short packet per byte, CDC-ACM packer idle timeout).
            received = []
            seq      = 0
            while len(received) < len(message):
                packets, seq = yield from pyhost_bulk_in(host, ept=7, count=1, seq=seq, nump=2)
                received += packets[0]
            self.assertEqual(received, message)

            # Events: marker, UART0 TX/RX bytes (records of 16 bytes).
            records = []
            seq     = 0
            while len(records) < 1 + 2*len(message):
                packets, seq = yield from pyhost_bulk_in(host, ept=11, count=1, seq=seq, nump=4)
                data = bytes(packets[0])
                self.assertEqual(len(data) % EVENT_RECORD_BYTES, 0)
                for i in range(0, len(data), EVENT_RECORD_BYTES):
                    r = data[i:i + EVENT_RECORD_BYTES]
                    records.append((r[0], r[1], int.from_bytes(r[2:8], "little"),
                        int.from_bytes(r[8:16], "little")))
            self.assertEqual(records[0][:3], (EVENT_MARKER, 0, 0x12345678))
            tx = [r[2] for r in records if r[0] == EVENT_UART_TX and r[1] == 0]
            rx = [r[2] for r in records if r[0] == EVENT_UART_RX and r[1] == 0]
            self.assertEqual(tx, message)
            self.assertEqual(rx, message)
            self.assertEqual([r[3] for r in records], sorted(r[3] for r in records))
        run_pyhost(dut, host, scenario())

if __name__ == "__main__":
    unittest.main()
