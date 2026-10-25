#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Control Endpoint: standard requests and class/vendor request handler (Migen simulation,
reactive host)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore

from test.model.host import *

# DUT (Device + test request handler) --------------------------------------------------------------

GET_CUR  = 0x81
SET_CUR  = 0x01
RESPONSE = [(3*i + 1) & 0xff for i in range(34)]

class DUT(LiteXModule):
    def __init__(self):
        self.phy  = phy  = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors      = bulk_descriptors(),
            with_ep0_handler = True)
        ep0 = core.ep0

        # Handler: GET_CUR (34 bytes response), SET_CUR (data recorded), others stalled.
        self.out_words = []
        words = [int.from_bytes(bytes(RESPONSE[i:i+4] + [0]*(4 - len(RESPONSE[i:i+4]))), "little")
            for i in range(0, len(RESPONSE), 4)]
        index  = Signal(max=len(words) + 1)
        active = Signal()
        self.sync += [
            If(ep0.req_setup,
                index.eq(0),
                active.eq(ep0.req_request == GET_CUR),
            ).Elif(ep0.req_in.valid & ep0.req_in.ready,
                index.eq(index + 1),
                If(ep0.req_in.last, active.eq(0)),
            )
        ]
        self.comb += [
            ep0.req_stall.eq(ep0.req_valid & (ep0.req_request != GET_CUR) & (ep0.req_request != SET_CUR)),
            ep0.req_in.valid.eq(active),
            ep0.req_in.data.eq(Array(words)[index]),
            ep0.req_in.last.eq(index == len(words) - 1),
            ep0.req_in.be.eq(Mux(index == len(words) - 1, 0b0011, 0b1111)), # 34 bytes: 2 bytes in the last word.
            ep0.req_out.ready.eq(1),
        ]

# Tests --------------------------------------------------------------------------------------------

class TestCoreControl(unittest.TestCase):
    def run_scenario(self, scenario):
        dut  = DUT()
        host = PyHost(dut.phy)
        out  = []
        @passive
        def out_monitor():
            while True:
                if (yield dut.core.ep0.req_out.valid):
                    data    = (yield dut.core.ep0.req_out.data)
                    be      = (yield dut.core.ep0.req_out.be)
                    n       = {0b0001: 1, 0b0011: 2, 0b0111: 3, 0b1111: 4}[be]
                    out.extend(list(data.to_bytes(4, "little"))[:n])
                yield
        def wrapper():
            yield from pyhost_link_init(host)
            yield from scenario(host, out)
        run_simulation(dut, [host.driver(), host.monitor(), wrapper(), out_monitor()])

    def test_standard_request(self):
        def scenario(host, out):
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, bulk_descriptors().device())
        self.run_scenario(scenario)

    def test_handler_in(self):
        # Class GET_CUR (34 bytes), host asking for less (wLength 26) and for more (64).
        def scenario(host, out):
            data = yield from pyhost_control_in(host, setup_packet(0xa1, GET_CUR, 0x0100, 1, 34))
            self.assertEqual(data, RESPONSE)
            data = yield from pyhost_control_in(host, setup_packet(0xa1, GET_CUR, 0x0100, 1, 26))
            self.assertEqual(data, RESPONSE[:26])
            data = yield from pyhost_control_in(host, setup_packet(0xa1, GET_CUR, 0x0100, 1, 64))
            self.assertEqual(data, RESPONSE)
        self.run_scenario(scenario)

    def test_handler_out(self):
        # Class SET_CUR with a 7 bytes Data stage: data forwarded to the handler.
        payload = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70]
        def scenario(host, out):
            ok = yield from pyhost_control_out(host, setup_packet(0x21, SET_CUR, 0x0100, 1, 7), payload)
            self.assertTrue(ok)
            self.assertEqual(out, payload)
        self.run_scenario(scenario)

    def test_handler_stall(self):
        # Unsupported class request: stalled by the handler, then a standard request works.
        def scenario(host, out):
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x99, 0, 0, 8))
            self.assertIsNone(data)
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, bulk_descriptors().device())
        self.run_scenario(scenario)

if __name__ == "__main__":
    unittest.main()
