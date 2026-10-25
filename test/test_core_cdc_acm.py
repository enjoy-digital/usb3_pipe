#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-ACM serial port (Migen simulation, reactive host): connection (SET_CONFIGURATION,
SET_CONTROL_LINE_STATE), line coding requests, echo. Composite device with two CDC-ACM functions on
a USB3ControlDispatcher: class requests answered by the function of their interface."""

import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.host_model import *
from usb3_core.cdc_acm     import USB3CDCACM, USB3CDCACMFunction, cdc_acm_interfaces
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.control     import USB3ControlDispatcher
from usb3_core.descriptors import USB3Descriptors, DEVICE_CLASS_IAD

from test.model.host import *

# DUT (CDC-ACM with echo) --------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self):
        self.phy  = phy  = PHYStandIn()
        self.acm  = acm  = USB3CDCACM(phy, sys_clk_freq=125e6)
        self.echo = echo = stream.SyncFIFO([("data", 8)], 64)
        self.comb += [
            acm.source.connect(echo.sink),
            echo.source.connect(acm.sink),
        ]

# DUT (two CDC-ACM functions) ----------------------------------------------------------------------

class DualDUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD, interfaces=
            cdc_acm_interfaces(comm_interface=0, notif_ept=3, in_ept=1, out_ept=2, max_burst=2) +
            cdc_acm_interfaces(comm_interface=2, notif_ept=6, in_ept=4, out_ept=5, max_burst=2))
        self.core = core = USB3DeviceCore(phy, 125e6, descriptors,
            endpoints = [
                USB3BulkIN(ept=1,  max_burst=2),
                USB3BulkOUT(ept=2, max_burst=2),
                USB3BulkIN(ept=3,  max_burst=2),
                USB3BulkIN(ept=4,  max_burst=2),
                USB3BulkOUT(ept=5, max_burst=2),
                USB3BulkIN(ept=6,  max_burst=2),
            ],
            with_ep0_handler = True)
        self.dispatcher = dispatcher = USB3ControlDispatcher(core.ep0, [[0, 1], [2, 3]])
        eps = core.endpoints
        self.acm0 = USB3CDCACMFunction(dispatcher.ports[0], eps[0], eps[1], interface=0,
            with_line_coding=True)
        self.acm1 = USB3CDCACMFunction(dispatcher.ports[1], eps[3], eps[4], interface=2,
            with_line_coding=True)

# Tests --------------------------------------------------------------------------------------------

class TestCoreCDCACM(unittest.TestCase):
    def test_connect_line_coding_echo(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            # Not connected before SET_CONFIGURATION/DTR.
            self.assertEqual((yield dut.acm.connect), 0)
            ok = yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))
            self.assertTrue(ok)
            # GET_LINE_CODING (115200 8N1), SET_LINE_CODING, SET_CONTROL_LINE_STATE (DTR).
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 0, 7))
            self.assertEqual(data, list((115200).to_bytes(4, "little")) + [0, 0, 8])
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x20, 0, 0, 7), data)
            self.assertTrue(ok)
            self.assertEqual((yield dut.acm.connect), 0)
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x22, 0x0001, 0, 0))
            self.assertTrue(ok)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.acm.connect), 1)

            # Echo: OUT bytes (odd length) looped back on the Bulk IN endpoint.
            message = list(b"Hello USB3 CDC-ACM!")
            n0 = len(host.dpps)
            host.send_hp(dph, length=len(message), ept=2, dp_seq=0)
            host.send(data_packet_payload(message))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            for _ in range(512):
                yield
            # The IN request may be NRDY'd before the echo: request again after the ERDY.
            if len(host.dpps) == n0:
                host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) > n0)
            self.assertEqual(sum([d[0] for d in host.dpps[n0:]], []), message)

            # Short packet acknowledged with NumP=0 (as xHCI): next echo after an ERDY.
            n1 = len(host.dpps)
            seq = (len(host.dpps) - n0) % 32
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=seq)
            h1 = len(host.hps)
            message = list(b"litex> ident")
            host.send_hp(dph, length=len(message), ept=2, dp_seq=1)
            host.send(data_packet_payload(message))
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_ERDY
                for hp in host.hps[h1:] if (hp[0] & 0x1f) == 0x04))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=seq, pp=1)
            yield from host.wait(lambda: len(host.dpps) > n1)
            self.assertEqual(sum([d[0] for d in host.dpps[n1:]], []), message)
        run_pyhost(dut, host, scenario())

    def test_dual_line_coding_dispatch(self):
        dut  = DualDUT()
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            ok = yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))
            self.assertTrue(ok)
            # SET_LINE_CODING on interface 2 (second function): 3Mbaud 8N1.
            coding = list((3000000).to_bytes(4, "little")) + [0, 0, 8]
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x20, 0, 2, 7), coding)
            self.assertTrue(ok)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.acm1.baudrate), 3000000)
            self.assertEqual((yield dut.acm0.baudrate), 115200)
            # GET_LINE_CODING of each function.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 2, 7))
            self.assertEqual(data, coding)
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 0, 7))
            self.assertEqual(data, list((115200).to_bytes(4, "little")) + [0, 0, 8])
            # SET_CONTROL_LINE_STATE (DTR) on interface 0 only.
            ok = yield from pyhost_control_out(host, setup_packet(0x21, 0x22, 0x0001, 0, 0))
            self.assertTrue(ok)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.acm0.connect), 1)
            self.assertEqual((yield dut.acm1.connect), 0)
            # Class request to an interface without function: stalled; unsupported class request
            # of a function: stalled.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 4, 7))
            self.assertIsNone(data)
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x7f, 0, 0, 7))
            self.assertIsNone(data)
            # Requests still answered after the stalls.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 2, 7))
            self.assertEqual(data, coding)
        run_pyhost(dut, host, scenario())

if __name__ == "__main__":
    unittest.main()
