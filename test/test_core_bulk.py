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
from usb3_core.common     import LCMD_LBAD, TYPE_TP, TYPE_DP, TP_SUB_ACK, TP_SUB_NRDY, TP_SUB_ERDY
from usb3_core.endpoints  import MAX_PACKET_SIZE, MAX_PACKET_WORDS
from usb3_core.bulk       import USB3BulkIN

from test.model.host   import *
from test.model.device import BulkLoopbackDUT as DUT

# Helpers ------------------------------------------------------------------------------------------

def payload(i, n=1024):
    return [(i*31 + j) & 0xff for j in range(n)]

# Tests --------------------------------------------------------------------------------------------

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

    @pytest.mark.slow
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

    @pytest.mark.slow
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

    @pytest.mark.slow
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

    @pytest.mark.slow
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

    @pytest.mark.slow
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

    @pytest.mark.slow
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

# Bulk IN Flush (Endpoint level) -------------------------------------------------------------------

class EndpointHost:
    """xHCI-like host on the Protocol Layer interface of a Bulk IN Endpoint: requests (ACK TP, NumP
    = n) when a transfer is pending (after an ERDY when flow controlled), checks the DPs (Sequence
    Number, in the window, DPH length), ACKs each DP (ack_latency cycles after its DPP, NumP=0 after
    a short packet: transfer complete)."""
    def __init__(self, ep, n, ack_latency=200):
        self.ep, self.n, self.ack_latency = ep, n, ack_latency
        self.seq       = 0     # Next expected Sequence Number.
        self.window    = 0
        self.pending   = False # Transfer pending.
        self.wait_erdy = False
        self.data      = []    # Received words.
        self.dphs      = 0     # DPHs received.
        self.dps       = []    # (seq, length).
        self.errors    = []
        self.acks      = []    # (cycle, nump, seq).
        self.cycle     = 0

    def ack_dw1(self, nump, seq):
        return TP_SUB_ACK | (1 << 7) | (self.ep.ept << 8) | (nump << 16) | ((seq % 32) << 21)

    def request(self):
        self.pending = True
        if not self.wait_erdy:
            self.window = self.n
            self.acks.append((self.cycle, self.n, self.seq))

    @passive
    def generator(self):
        ep   = self.ep
        dphs = []
        dpp  = []
        yield ep.tx_hp.ready.eq(1)
        yield ep.tx_dpp.ready.eq(1)
        while True:
            strobe = len(self.acks) > 0 and self.acks[0][0] <= self.cycle
            if strobe:
                _, nump, seq = self.acks.pop(0)
                yield ep.hdr_dw0.eq(TYPE_TP)
                yield ep.hdr_dw1.eq(self.ack_dw1(nump, seq))
            yield ep.hdr_strobe.eq(strobe)
            yield
            self.cycle += 1
            if (yield ep.tx_hp.valid):
                dw0, dw1 = (yield ep.tx_hp.dw0), (yield ep.tx_hp.dw1)
                if (dw0 & 0x1f) == TYPE_DP:
                    dphs.append((dw1 & 0x1f, dw1 >> 16))
                    self.dphs += 1
                elif (dw1 & 0xf) == TP_SUB_NRDY:
                    self.window    = 0
                    self.wait_erdy = True
                elif (dw1 & 0xf) == TP_SUB_ERDY:
                    self.wait_erdy = False
                    if self.pending:
                        self.request()
            if (yield ep.tx_dpp.valid):
                dpp.append((yield ep.tx_dpp.data))
                if (yield ep.tx_dpp.last):
                    seq, length = dphs.pop(0)
                    self.dps.append((seq, length))
                    if length != 4*len(dpp):
                        self.errors.append("DP {}: length {}, DPP {} bytes".format(seq, length, 4*len(dpp)))
                    elif seq != self.seq or self.window == 0:
                        self.errors.append("DP {}: expected {} (window {})".format(seq, self.seq, self.window))
                    else:
                        self.seq   += 1
                        self.data  += dpp
                        short       = length < MAX_PACKET_SIZE
                        self.window = 0 if short else self.n
                        self.pending &= not short
                        self.acks.append((self.cycle + self.ack_latency, self.window, self.seq))
                    dpp = []

class TestCoreBulkINFlush(unittest.TestCase):
    def flush_test(self, sent):
        # Flush (stream restarted) after `sent` DPs sent (not acknowledged yet), the old stream
        # continuing: the packets not sent are dropped, the new stream is sent next.
        n    = 2
        dut  = USB3BulkIN(ept=1, max_burst=n)
        host = EndpointHost(dut, n)
        old  = [[0xa000_0000 | (p << 16) | i for i in range(MAX_PACKET_WORDS)] for p in range(2*n)]
        new  = [0xb000_0000 | i for i in range(300)]
        state = {"flushed": False}
        def write(words, last):
            # Old stream (not last) interrupted by the flush (as the LA packer reset on arm).
            stop = lambda: state["flushed"] and not last
            for i, w in enumerate(words):
                if stop():
                    break
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(w)
                yield dut.sink.be.eq(0b1111)
                yield dut.sink.last.eq(last and (i == len(words) - 1))
                yield
                while not (yield dut.sink.ready) and not stop():
                    yield
            yield dut.sink.valid.eq(0)
        def writer():
            for packet in old:
                yield from write(packet, last=False)
            while not state["flushed"]:
                yield
            for _ in range(64):
                yield
            yield from write(new, last=True)
        def scenario():
            if sent:
                host.request()
                while host.dphs < sent:
                    yield
            else:
                for _ in range(1024): # Slots full, nothing requested.
                    yield
            state["flushed"] = True
            yield dut.flush.eq(1)
            yield
            yield dut.flush.eq(0)
            for _ in range(2000):
                if not host.pending:
                    host.request()
                yield
        run_simulation(dut, [scenario(), writer(), host.generator()])
        self.assertEqual(host.errors, [])
        self.assertEqual([seq for seq, _ in host.dps], list(range(len(host.dps))))
        self.assertEqual(host.data, sum(old[:sent], []) + new)

    def test_bulk_in_flush_buffered(self):
        self.flush_test(sent=0)

    def test_bulk_in_flush_inflight(self):
        # Flush while DPs are sent and not acknowledged yet: the host ACK TPs of the sent DPs are
        # still accepted (Sequence Numbers kept in sync with the host). Previously the pointers
        # jumped over them: ACK TPs rejected, new stream sent with already used Sequence Numbers,
        # NRDY/ERDY loop (endpoint wedged).
        self.flush_test(sent=2)

if __name__ == "__main__":
    unittest.main()
