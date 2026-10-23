#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Native USB3 Device Core: bursts, flow control and error recovery (Migen simulation, reactive Python
host acknowledging the device Header Packets like an xHCI host)."""

import random
import unittest

from collections import deque

import pytest

from migen import *

from usb3_core.host_model import *
from usb3_core.common     import LCMD_LBAD

from test.test_native_core import DUT

# Reactive Host ------------------------------------------------------------------------------------

FRAMING = {0xfbfbfbf7: "HP", 0xfefefef7: "LC", 0x5c5c5cf7: "DPP"}

class PyHost:
    """Host Link Layer: sends packets (atomic), acknowledges the device Header Packets (LGOOD_n +
    LCRD_x), records the received packets."""
    def __init__(self, phy, corrupt_lgood=False):
        self.phy     = phy
        self.queue   = deque() # Packets (lists of words) to send.
        self.urgent  = deque() # Link Commands (sent first, between packets).
        self.tx_seq  = 0
        self.lcrd    = 0
        self.hps     = []      # (dw0, dw1, dw2, seq, crc_ok).
        self.dpps    = []      # (payload, crc_ok).
        self.lcs     = []
        self.cycles  = 0

    def send_hp(self, builder, *args, **kwargs):
        self.queue.append(builder(*args, seq=self.tx_seq, **kwargs))
        self.tx_seq = (self.tx_seq + 1) % 8

    def send(self, words):
        self.queue.append(words)

    def driver(self):
        yield "passive"
        phy = self.phy
        yield phy.polling_idle.eq(1)
        yield
        yield phy.polling_idle.eq(0)
        yield phy.u0.eq(1)
        yield phy.source.valid.eq(1)
        current = deque()
        while True:
            if not current:
                if self.urgent:
                    current = deque(self.urgent.popleft())
                elif self.queue:
                    current = deque(self.queue.popleft())
            if current:
                data, ctrl = current.popleft()
            else:
                data, ctrl = 0, 0
            yield phy.source.data.eq(data)
            yield phy.source.ctrl.eq(ctrl)
            yield
            self.cycles += 1

    def monitor(self, tx_ready_rate=1.0, seed=0):
        yield "passive"
        phy   = self.phy
        rng   = random.Random(seed)
        state = None
        words = []
        while True:
            ready = rng.random() < tx_ready_rate
            yield phy.sink.ready.eq(int(ready))
            yield
            if not (ready and (yield phy.sink.valid)):
                continue
            w = ((yield phy.sink.data), (yield phy.sink.ctrl))
            if state is None:
                if w[1] == 0b1111 and w[0] in FRAMING:
                    state = FRAMING[w[0]]
                    words = [w]
                continue
            words.append(w)
            if state == "LC" and len(words) == 2:
                self.lcs.append(decode_packets(words)[0][1])
                state = None
            elif state == "HP" and len(words) == 5:
                packet = decode_packets(words)[0]
                self.hps.append(packet[1:])
                if packet[-1]:
                    self.urgent.append(link_command(LGOOD(packet[4])))
                    self.urgent.append(link_command(LCRD(self.lcrd)))
                    self.lcrd = (self.lcrd + 1) % 4
                state = None
            elif state == "DPP" and any(c & (1 << (3 - i)) and ((d >> 8*(3 - i)) & 0xff) == 0xf7
                for d, c in [w] for i in range(4)) and len(words) > 1:
                packets = decode_packets(words + [(0, 0)])
                if packets:
                    self.dpps.append(tuple(packets[0][1:]))
                state = None

    def wait(self, cond, timeout=200000):
        for _ in range(timeout):
            if cond():
                return
            yield
        raise TimeoutError("PyHost wait timeout")

# Helpers ------------------------------------------------------------------------------------------

def tp_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "subtype": dw1 & 0xf, "nump": (dw1 >> 16) & 0x1f,
        "seq": (dw1 >> 21) & 0x1f, "retry": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf}

def dph_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "seq": dw1 & 0x1f, "eob": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf,
        "length": dw1 >> 16}

def run(dut, host, scenario, tx_ready_rate=1.0):
    run_simulation(dut, [host.driver(), host.monitor(tx_ready_rate), scenario], vcd_name=None)

def link_init(host):
    host.send(link_command(LGOOD(7)))
    for x in range(4):
        host.send(link_command(LCRD(x)))
    host.send_hp(lmp_port_capability)
    host.send_hp(lmp_port_configuration)
    yield from host.wait(lambda: len(host.hps) >= 2)

def payload(i, n=1024):
    return [(i*31 + j) & 0xff for j in range(n)]

# Tests --------------------------------------------------------------------------------------------

@pytest.mark.slow
class TestNativeBursts(unittest.TestCase):
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
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_bulk_out_crc_error_retry(self):
        # DPP with a CRC-32 error: ACK TP with retry for this packet, then accepted when resent.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_header_crc_error_lbad_lrty(self):
        # Header Packet with a CRC-16 error: LBAD, ignored until LRTY + resent Header Packet (same
        # Header Sequence Number).
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_bulk_in_nrdy_erdy(self):
        # IN request without data: NRDY, then ERDY when data is available.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_bulk_in_eob(self):
        # Fewer packets available than requested: EOB on the last one, then ERDY and new request.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
            self.out_burst(host, 0, 2)
            for _ in range(2048):
                yield
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=8, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            for _ in range(256):
                yield
            dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
            self.assertEqual([(d["seq"], d["eob"]) for d in dphs], [(0, 0), (1, 1)])
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
        run(dut, host, scenario())

    def test_bulk_pointers_wrap(self):
        # max_burst=2 (pointers modulo 4): 10 packets looped back, bursts of 2, pointers wrapping.
        dut  = DUT(max_burst=2)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_set_address(self):
        # SET_ADDRESS: Status stage ACK TP from the default address (0), then from the new address.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
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
        run(dut, host, scenario())

    def test_tx_backpressure(self):
        # PHY TX ready deasserted randomly (SKP insertion): packets integrity.
        dut  = DUT(max_burst=8)
        host = PyHost(dut.phy)
        def scenario():
            yield from link_init(host)
            self.out_burst(host, 0, 2)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=2, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) >= 2)
            self.assertEqual([d for d in host.dpps[:2]], [(payload(0), True), (payload(1), True)])
        run(dut, host, scenario(), tx_ready_rate=0.9)

if __name__ == "__main__":
    unittest.main()
