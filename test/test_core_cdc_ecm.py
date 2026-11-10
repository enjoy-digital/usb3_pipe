#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-ECM function: frames from the host echoed back (frame of two packets, frame of a max
size packet ended by a zero-length packet, short frame), zero-length packet after a frame of a
multiple of the max packet size, NETWORK_CONNECTION/CONNECTION_SPEED_CHANGE notifications (Migen
simulation, reactive host)."""

import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.host_model  import *
from usb3_core.common      import endpoint_layout
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT, bulk_endpoint_32
from usb3_core.descriptors import USB3Descriptors, DEVICE_CLASS_IAD
from usb3_core.cdc_ecm     import *

from test.model.host import *

# DUT (CDC-ECM with echo) --------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD)
        mac = add_cdc_ecm_strings(descriptors, 0x02eccc000001)
        descriptors.interfaces = cdc_ecm_interfaces(0, notif_ept=3, in_ept=1, out_ept=2,
            mac_string=mac, max_burst=2)
        add_cdc_ecm_requests(descriptors)
        self.core = core = USB3DeviceCore(phy, 125e6, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=2),
            USB3BulkOUT(ept=2, max_burst=8, with_zlp=True), # 5 OUT packets sent at once.
            USB3BulkIN(ept=3,  max_burst=2),
        ], with_ep0_handler=True)
        eps = core.endpoints
        self.ecm  = ecm  = USB3CDCECMFunction(core.ep0, eps[0], eps[1], eps[2])
        self.echo = echo = stream.SyncFIFO(endpoint_layout(32), 1024, buffered=True)
        self.comb += [
            ecm.source.connect(echo.sink),
            echo.source.connect(ecm.sink),
        ]

# Tests --------------------------------------------------------------------------------------------

class TestCoreCDCECM(unittest.TestCase):
    def test_frames_notifications(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        frames = [
            [(3*i + 1) & 0xff for i in range(1514)], # 1024 + 490 bytes.
            [(5*i + 2) & 0xff for i in range(1024)], # Max size packet + zero-length packet.
            [(7*i + 3) & 0xff for i in range(60)],
        ]
        res = {}

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # SET_INTERFACE (data interface, alternate setting 1), SET_ETHERNET_PACKET_FILTER.
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, 1, 1, 0))))
            self.assertTrue((yield from pyhost_control_out(host,
                setup_packet(0x21, CDC_SET_ETHERNET_PACKET_FILTER, 0x0f, 0, 0))))
            # Notifications: NETWORK_CONNECTION (connected), CONNECTION_SPEED_CHANGE.
            packets, _ = yield from pyhost_bulk_in(host, ept=3, count=2, seq=0)
            res["notifications"] = packets
            # Frames (OUT), a zero-length packet after the max size packet one.
            seq = 0
            for frame in frames:
                for i in range(0, len(frame), 1024):
                    chunk = frame[i:i+1024]
                    host.send_hp(dph, length=len(chunk), ept=2, dp_seq=seq)
                    host.send(data_packet_payload(chunk))
                    seq = (seq + 1) % 32
                if len(frame) % 1024 == 0:
                    host.send_hp(dph, length=0, ept=2, dp_seq=seq)
                    host.send(data_packet_payload([]))
                    seq = (seq + 1) % 32
            # Echoed frames (IN): 2 + 2 (with the zero-length packet) + 1 packets.
            packets, _ = yield from pyhost_bulk_in(host, ept=1, count=5, seq=0)
            res["in"] = packets
        run_pyhost(dut, host, scenario())

        conn, speed = res["notifications"]
        self.assertEqual(conn, [0xa1, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.assertEqual(speed[:8], [0xa1, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00])
        self.assertEqual(speed[8:], list((10**9).to_bytes(4, "little"))*2)
        p = res["in"]
        self.assertEqual([len(x) for x in p], [1024, 490, 1024, 0, 60])
        self.assertEqual(p[0] + p[1], frames[0])
        self.assertEqual(p[2], frames[1])
        self.assertEqual(p[4], frames[2])

if __name__ == "__main__":
    unittest.main()
