#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 GUD (Generic USB Display) function: requests as the Linux gud driver sends them (vendor,
interface), SET_BUFFER start pulse, frame data on the Bulk OUT endpoint (Migen simulation, reactive
host)."""

import struct
import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model import *
from usb3_core.device     import USB3DeviceCore
from usb3_core.bulk       import USB3BulkOUT
from usb3_core.gud        import *
from usb3_core.descriptors import USB3Descriptors

from test.model.host import *

# 720p60 (LiteX video_timings["1280x720@60Hz"]).
TIMINGS = {
    "pix_clk"       : 74.25e6,
    "h_active"      : 1280,
    "h_blanking"    : 370,
    "h_sync_offset" : 220,
    "h_sync_width"  : 40,
    "v_active"      : 720,
    "v_blanking"    : 30,
    "v_sync_offset" : 5,
    "v_sync_width"  : 5,
}

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        descriptors = USB3Descriptors(vid=GUD_VID, pid=GUD_PID,
            interfaces=gud_interfaces(0, out_ept=1, max_burst=2))
        self.frame_size = add_gud_requests(descriptors, TIMINGS)
        self.core = core = USB3DeviceCore(phy, 125e6, descriptors,
            endpoints=[USB3BulkOUT(ept=1, max_burst=2)])
        self.gud = gud = USB3GUDFunction(core.ep0, core.endpoints[0])
        self.starts = Signal(8)
        self.sync += If(gud.start, self.starts.eq(self.starts + 1))
        self.comb += gud.source.ready.eq(1)

class TestCoreGUD(unittest.TestCase):
    def test_gud(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        rx   = []

        def get(request, length, value=0):
            # Vendor, interface 0 (gud driver: wIndex = interface number).
            return (yield from pyhost_control_in(host,
                setup_packet(GUD_REQ_TYPE_IN, request, value, 0, length)))

        @passive
        def monitor():
            while True:
                if (yield dut.gud.source.valid):
                    rx.append(((yield dut.gud.source.data), (yield dut.gud.source.be),
                        (yield dut.gud.source.last)))
                yield

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # Probe (gud_probe): display descriptor, formats, properties, connectors.
            desc = yield from get(GUD_REQ_GET_DESCRIPTOR, 30)
            magic, version, flags, compression, max_buffer, minw, maxw, minh, maxh = \
                struct.unpack("<IBIBIIIII", bytes(desc))
            self.assertEqual((magic, version, flags, compression),
                (GUD_DISPLAY_MAGIC, 1, GUD_DISPLAY_FLAG_FULL_UPDATE, 0))
            self.assertEqual((max_buffer, minw, maxw, minh, maxh),
                (1280*720*2, 1280, 1280, 720, 720))
            self.assertEqual((yield from get(GUD_REQ_GET_FORMATS, 32)), [GUD_PIXEL_FORMAT_RGB565])
            self.assertEqual((yield from get(GUD_REQ_GET_CONNECTORS, 32*5)),
                [GUD_CONNECTOR_TYPE_HDMI, 0, 0, 0, 0])
            # Connector (wValue: connector index): status, modes (wLength larger than the response).
            self.assertEqual((yield from get(GUD_REQ_GET_CONNECTOR_STATUS, 1)),
                [GUD_CONNECTOR_STATUS_CONNECTED])
            mode = yield from get(GUD_REQ_GET_CONNECTOR_MODES, 128*24)
            self.assertEqual(struct.unpack("<IHHHHHHHHI", bytes(mode)),
                (74250, 1280, 1500, 1540, 1650, 720, 725, 730, 750,
                 GUD_DISPLAY_MODE_FLAG_PHSYNC | GUD_DISPLAY_MODE_FLAG_PVSYNC |
                 GUD_DISPLAY_MODE_FLAG_PREFERRED))
            self.assertEqual((yield from get(GUD_REQ_GET_STATUS, 1)), [GUD_STATUS_OK])
            # SET_STATE_CHECK/COMMIT, SET_CONTROLLER/DISPLAY_ENABLE accepted.
            state = list(mode) + [GUD_PIXEL_FORMAT_RGB565, 0]
            for request, data in [(GUD_REQ_SET_STATE_CHECK, state), (GUD_REQ_SET_STATE_COMMIT, None),
                (GUD_REQ_SET_CONTROLLER_ENABLE, [1]), (GUD_REQ_SET_DISPLAY_ENABLE, [1])]:
                ok = yield from pyhost_control_out(host,
                    setup_packet(GUD_REQ_TYPE_OUT, request, 0, 0, len(data) if data else 0), data)
                self.assertTrue(ok)
            self.assertEqual((yield dut.starts), 0)
            # SET_BUFFER (struct gud_set_buffer_req, 25 bytes): start pulse.
            req = list(struct.pack("<IIIIIBI", 0, 0, 1280, 720, 1280*720*2, 0, 0))
            ok = yield from pyhost_control_out(host,
                setup_packet(GUD_REQ_TYPE_OUT, GUD_REQ_SET_BUFFER, 0, 0, len(req)), req)
            self.assertTrue(ok)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.starts), 1)
            # Frame data on the Bulk OUT endpoint.
            payload = [(7*i + 3) & 0xff for i in range(1024)]
            host.send_hp(dph, length=1024, ept=1, dp_seq=0)
            host.send(data_packet_payload(payload))
            yield from host.wait(lambda: len(rx) >= 256)
            got = sum([list(d.to_bytes(4, "little")) for d, be, last in rx[:256]], [])
            self.assertEqual(got, payload)

        run_simulation(dut, [host.driver(), host.monitor(1.0), scenario(), monitor()])

    def test_empty_responses(self):
        # GET_PROPERTIES/GET_CONNECTOR_PROPERTIES/GET_CONNECTOR_EDID: zero-length data stage.
        dut  = DUT()
        host = PyHost(dut.phy)
        res  = {}
        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            for request, length in [(GUD_REQ_GET_PROPERTIES, 320), (GUD_REQ_GET_CONNECTOR_PROPERTIES, 320),
                (GUD_REQ_GET_CONNECTOR_EDID, 2048)]:
                res[request] = yield from pyhost_control_in(host,
                    setup_packet(GUD_REQ_TYPE_IN, request, 0, 0, length))
        run_simulation(dut, [host.driver(), host.monitor(1.0), scenario()])
        for request, data in res.items():
            self.assertEqual(data, [], f"request 0x{request:02x}")

if __name__ == "__main__":
    unittest.main()
