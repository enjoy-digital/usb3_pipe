#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-ACM serial port (Migen simulation, reactive host): connection (SET_CONFIGURATION,
SET_CONTROL_LINE_STATE), line coding requests, echo."""

import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.host_model import *
from usb3_core.cdc_acm    import USB3CDCACM

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

if __name__ == "__main__":
    unittest.main()
