#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core: bursts, flow control and error recovery (Migen simulation, reactive Python
host acknowledging the device Header Packets like an xHCI host)."""

import unittest

import pytest

from migen import *

from usb3_core.host_model import *
from usb3_core.common     import LCMD_LBAD

from test.model.host   import *
from test.model.device import BulkLoopbackDUT as DUT

# Helpers ------------------------------------------------------------------------------------------

def payload(i, n=1024):
    return [(i*31 + j) & 0xff for j in range(n)]

# Tests --------------------------------------------------------------------------------------------

@pytest.mark.slow
class TestCoreBulk(unittest.TestCase):
    def out_burst(self, host, start, n, seq0=0, corrupt=None):
        for i in range(n):
            data = data_packet_payload(payload(start + i))
            if corrupt == i:
                data = list(data)
                data[5] = (data[5][0] ^ 0x1, data[5][1]) # Corrupted payload: CRC-32 error.
            host.send_hp(dph, length=1024, ept=2, dp_seq=(seq0 + i) % 32)
            host.send(data)

    def acks(self, host, start=0):
        return [tp_fields(hp) for hp in host.hps[start:] if (hp[0] & 0x1f) == 0x04 and (hp[1] & 0xf) == 1]

    def test_bulk_out_burst_in_burst(self):
        # 4 OUT packets back-to-back (burst), then 4 IN packets requested with a single ACK TP.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            n0 = len(host.hps)
            self.out_burst(host, 0, 4)
            yield from host.wait(lambda: len(self.acks(host, n0)) >= 4)
            acks = self.acks(host, n0)
            self.assertEqual([a["seq"] for a in acks], [1, 2, 3, 4])
            self.assertEqual([a["retry"] for a in acks], [0]*4)
            for a in acks:
                self.assertGreaterEqual(a["nump"], 1)
            # IN burst (looped back data available).
            for _ in range(2048):
                yield
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=4, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 4)
            self.assertEqual([d[0] for d in host.dpps[:4]], [payload(i) for i in range(4)])
            self.assertTrue(all(d[1] for d in host.dpps[:4]))
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([d["seq"] for d in dphs], [0, 1, 2, 3])
            # 4 requested, 4 sent: no EOB.
            self.assertEqual([d["eob"] for d in dphs], [0, 0, 0, 0])
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=4)
            for _ in range(256):
                yield
        run_pyhost(dut, host, scenario())

    def test_bulk_out_crc_error_retry(self):
        # DPP with a CRC-32 error: ACK TP with retry for this packet, then accepted when resent.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            n0 = len(host.hps)
            self.out_burst(host, 0, 2, corrupt=1)
            yield from host.wait(lambda: len(self.acks(host, n0)) >= 2)
            acks = self.acks(host, n0)
            self.assertEqual([(a["seq"], a["retry"]) for a in acks], [(1, 0), (1, 1)])
            n1 = len(host.hps)
            self.out_burst(host, 1, 1, seq0=1)
            yield from host.wait(lambda: len(self.acks(host, n1)) >= 1)
            self.assertEqual([(a["seq"], a["retry"]) for a in self.acks(host, n1)], [(2, 0)])
            # Packets looped back: 0 and 1 (the corrupted one dropped).
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            self.assertEqual([d[0] for d in host.dpps[:2]], [payload(0), payload(1)])
        run_pyhost(dut, host, scenario())

    def test_header_crc_error_lbad_lrty(self):
        # Header Packet with a CRC-16 error: LBAD, ignored until LRTY + resent Header Packet (same
        # Header Sequence Number).
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            n0  = len(host.hps)
            seq = host.tx_seq
            bad = list(dph(seq=seq, length=1024, ept=2, dp_seq=0))
            bad[2] = (bad[2][0] ^ 0x100, bad[2][1])
            host.send(bad)
            host.send(data_packet_payload(payload(0)))
            yield from host.wait(lambda: LCMD_LBAD in host.lcs)
            host.send(link_command(LRTY()))
            host.send(dph(seq=seq, length=1024, ept=2, dp_seq=0))
            host.send(data_packet_payload(payload(0)))
            host.tx_seq = (seq + 1) % 8
            yield from host.wait(lambda: len(self.acks(host, n0)) >= 1)
            self.assertEqual([(a["seq"], a["retry"]) for a in self.acks(host, n0)], [(1, 0)])
        run_pyhost(dut, host, scenario())

    def test_bulk_in_nrdy_erdy(self):
        # IN request without data: NRDY, then ERDY when data is available.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=4, tp_seq=0, pp=1)
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_NRDY
                for hp in host.hps if (hp[0] & 0x1f) == 0x04))
            n0 = len(host.hps)
            self.out_burst(host, 0, 1)
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_ERDY
                for hp in host.hps[n0:] if (hp[0] & 0x1f) == 0x04))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=4, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 1)
            self.assertEqual(host.dpps[0][0], payload(0))
        run_pyhost(dut, host, scenario())

    def test_bulk_in_eob(self):
        # Fewer packets available than requested: EOB on the last one, flow control (the ACK TPs of
        # the packets sent before the EOB do not reopen the window), ERDY and new request.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            self.out_burst(host, 0, 2)
            for _ in range(2048):
                yield
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=8, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            for _ in range(256):
                yield
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([(d["seq"], d["eob"]) for d in dphs], [(0, 0), (1, 1)])
            # ACK TP of DP0 (window still open, sent before the EOB DP was received), then of the EOB DP.
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=8, tp_seq=1)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=2)
            n0 = len(host.hps)
            self.out_burst(host, 2, 1, seq0=2)
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_ERDY
                for hp in host.hps[n0:] if (hp[0] & 0x1f) == 0x04))
            # No DP sent before the host request.
            self.assertEqual(len(host.dpps), 2)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=8, tp_seq=2, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 3)
            self.assertEqual(host.dpps[2][0], payload(2))
        run_pyhost(dut, host, scenario())

    def test_bulk_pointers_wrap(self):
        # max_burst=2 (pointers modulo 4): 10 packets looped back, bursts of 2, pointers wrapping.
        dut  = DUT(max_burst=2)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            for k in range(0, 10, 2):
                n0 = len(host.hps)
                self.out_burst(host, k, 2, seq0=k)
                yield from host.wait(lambda: len(self.acks(host, n0)) >= 2)
                acks = self.acks(host, n0)
                self.assertEqual([(a["seq"], a["retry"]) for a in acks], [((k + 1) % 32, 0), ((k + 2) % 32, 0)])
                for _ in range(1024):
                    yield
                host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=k, pp=1)
                yield from host.wait(lambda: len(host.dpps) >= k + 2)
                host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=k + 2)
            self.assertEqual([d[0] for d in host.dpps], [payload(i) for i in range(10)])
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([d["seq"] for d in dphs], list(range(10)))
        run_pyhost(dut, host, scenario())

    def test_set_address(self):
        # SET_ADDRESS: Status stage ACK TP from the default address (0), then from the new address.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            n0 = len(host.hps)
            host.send_hp(dph, length=8, setup=1)
            host.send(data_packet_payload(setup_packet(0x00, 0x05, 0x0002, 0, 0)))
            yield from host.wait(lambda: len(host.hps) >= n0 + 1)
            host.send_hp(tp, TP_SUB_STATUS)
            yield from host.wait(lambda: len(host.hps) >= n0 + 2)
            host.send_hp(dph, length=8, setup=1, addr=2)
            host.send(data_packet_payload(setup_packet(0x80, 0x06, 0x0100, 0, 18)))
            yield from host.wait(lambda: len(host.hps) >= n0 + 3)
            addrs = [hp[0] >> 25 for hp in host.hps[n0:n0 + 3]]
            self.assertEqual(addrs, [0, 0, 2])
        run_pyhost(dut, host, scenario())

    def test_bulk_in_short_packet_nump0(self):
        # Short packet acknowledged with NumP=0 (host closing the window, as xHCI after a short
        # packet), then new data: ERDY, new host request, data sent.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            host.send_hp(dph, length=64, ept=2, dp_seq=0)
            host.send(data_packet_payload(payload(0, 64)))
            for _ in range(512):
                yield
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=1, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 1)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=1)
            n0 = len(host.hps)
            host.send_hp(dph, length=64, ept=2, dp_seq=1)
            host.send(data_packet_payload(payload(1, 64)))
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_ERDY
                for hp in host.hps[n0:] if (hp[0] & 0x1f) == 0x04))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=1, tp_seq=1, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            self.assertEqual([d[0] for d in host.dpps[:2]], [payload(0, 64), payload(1, 64)])
        run_pyhost(dut, host, scenario())

    def test_bulk_in_short_packets(self):
        # Two short packets available: only the first one is sent (a short packet ends the transfer),
        # the second one after the host ACK TP (NumP=0, ERDY, new request), with the next Sequence
        # Number (found on hardware with a CDC-ACM console: the host dropped the second packet).
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            for i in range(2):
                host.send_hp(dph, length=64, ept=2, dp_seq=i)
                host.send(data_packet_payload(payload(i, 64)))
            for _ in range(1024):
                yield
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 1)
            for _ in range(256):
                yield
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([d["seq"] for d in dphs], [0])
            n0 = len(host.hps)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=1)
            yield from host.wait(lambda: any(tp_fields(hp)["subtype"] == TP_SUB_ERDY
                for hp in host.hps[n0:] if (hp[0] & 0x1f) == 0x04))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=1, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([d["seq"] for d in dphs], [0, 1])
            self.assertEqual([d[0] for d in host.dpps[:2]], [payload(0, 64), payload(1, 64)])
        run_pyhost(dut, host, scenario())

    def test_tx_backpressure(self):
        # PHY TX ready deasserted randomly (SKP insertion): packets integrity.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            self.out_burst(host, 0, 2)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            self.assertEqual([d for d in host.dpps[:2]], [(payload(0), True), (payload(1), True)])
        run_pyhost(dut, host, scenario(), tx_ready_rate=0.9)

if __name__ == "__main__":
    unittest.main()
