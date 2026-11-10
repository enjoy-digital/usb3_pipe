#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SET_INTERFACE resets the Sequence Numbers of the endpoints of the interface (USB 3.2 9.4.10):
CDC-ECM frames echoed before and after a SET_INTERFACE, the host restarting at Sequence Number 0 on
both directions after it (Migen simulation, reactive host)."""

import unittest

from migen import *

from usb3_core.host_model import *

from test.model.host import *
from test.test_core_cdc_ecm import DUT

class TestCoreSetInterface(unittest.TestCase):
    def test_set_interface_resets_sequence_numbers(self):
        dut    = DUT()
        host   = PyHost(dut.phy)
        frames = [[(3*i + 1) & 0xff for i in range(100)], [(5*i + 2) & 0xff for i in range(60)]]
        res    = {}

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            res["in"] = []
            for frame in frames:
                # SET_INTERFACE (data interface 1, alternate setting 1): Sequence Numbers from 0.
                self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, 1, 1, 0))))
                host.send_hp(dph, length=len(frame), ept=2, dp_seq=0)
                host.send(data_packet_payload(frame))
                packets, _ = yield from pyhost_bulk_in(host, ept=1, count=1, seq=0)
                res["in"] += packets
        run_pyhost(dut, host, scenario())
        self.assertEqual(res["in"], frames)

if __name__ == "__main__":
    unittest.main()
