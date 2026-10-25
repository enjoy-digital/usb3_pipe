#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core: Isochronous IN endpoint (USB 3.2 8.12.6), descriptors/alternate settings
(Migen simulation, reactive Python host acting as an isochronous host)."""

import unittest

import pytest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.descriptors import *
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3IsoIN

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self, max_burst=4):
        self.phy  = phy  = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors = iso_in_descriptors(ept=1, max_burst=max_burst),
            endpoints   = [USB3IsoIN(ept=1, max_burst=max_burst)])
        self.iso = core.endpoints[0]

# Helpers ------------------------------------------------------------------------------------------

def payload(i, n):
    return [(i*37 + j) & 0xff for j in range(n)]

def write_transfer(ep, data):
    """Write a service interval transfer to the endpoint sink."""
    words = [data[i:i+4] for i in range(0, len(data), 4)]
    for i, w in enumerate(words):
        yield ep.sink.valid.eq(1)
        yield ep.sink.data.eq(int.from_bytes(bytes(w + [0]*(4 - len(w))), "little"))
        yield ep.sink.be.eq((1 << len(w)) - 1)
        yield ep.sink.last.eq(i == len(words) - 1)
        yield
        while not (yield ep.sink.ready):
            yield
    yield ep.sink.valid.eq(0)
    yield

def wait(n):
    for _ in range(n):
        yield

def configure(host, alt=1):
    """SET_CONFIGURATION(1), SET_INTERFACE(0, alt)."""
    ok = yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))
    assert ok
    ok = yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, alt, 0, 0))
    assert ok

def run(dut, host, generators):
    run_simulation(dut, [host.driver(), host.monitor()] + generators, vcd_name=None)

# Tests --------------------------------------------------------------------------------------------

class TestCoreIso(unittest.TestCase):
    def check_transfer(self, dps, data, max_burst):
        """DPs of a service interval: Seq Num 0..N-1 (no retry), 1024 bytes packets except the last,
        LPF on the last one only, data/CRCs."""
        n = max(1, -(-len(data)//1024))
        self.assertEqual(len(dps), n)
        self.assertLessEqual(n, max_burst)
        self.assertEqual([d["seq"] for d, p, crc in dps], list(range(n)))
        self.assertEqual([d["lpf"] for d, p, crc in dps], [0]*(n - 1) + [1])
        self.assertEqual([d["length"] for d, p, crc in dps], [len(p) for d, p, crc in dps])
        self.assertTrue(all(len(p) == 1024 for d, p, crc in dps[:-1]))
        self.assertTrue(all(crc for d, p, crc in dps))
        self.assertEqual(sum([p for d, p, crc in dps], []), data)

    def test_descriptors(self):
        d      = iso_in_descriptors(ept=1, max_burst=4, interval=1)
        config = d.configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertEqual(config[4], 1) # One interface (2 alternate settings).
        descs = []
        while config:
            descs.append(config[:config[0]])
            config = config[config[0]:]
        interfaces = [x for x in descs if x[1] == DESC_INTERFACE]
        self.assertEqual([(x[2], x[3], x[4]) for x in interfaces], [(0, 0, 0), (0, 1, 1)])
        # Isochronous IN, Asynchronous, 1024 bytes, bInterval 1.
        ep = [x for x in descs if x[1] == DESC_ENDPOINT][0]
        self.assertEqual(ep[2:7], [0x81, 0x05, 0x00, 0x04, 1])
        # Companion: bMaxBurst 3, Mult 0, wBytesPerInterval 4096.
        comp = [x for x in descs if x[1] == DESC_SS_EP_COMP][0]
        self.assertEqual(comp, [6, DESC_SS_EP_COMP, 3, 0, 0x00, 0x10])
        self.assertEqual(d.alt_settings(), {0: 2})

    def test_iso_in(self):
        max_burst = 4
        dut  = DUT(max_burst=max_burst)
        host = PyHost(dut.phy)
        transfers = [payload(0, 4096), payload(1, 2500), payload(2, 1024), payload(3, 102), payload(4, 3072)]
        state = {"written": 0, "read": -1}
        def writer():
            # One transfer written per service interval, once the previous one read.
            for i, t in enumerate(transfers):
                while state["read"] < i:
                    yield
                yield from write_transfer(dut.iso, t)
                state["written"] = i + 1
        def scenario():
            yield from pyhost_link_init(host)
            yield from configure(host, alt=1)
            # GET_INTERFACE: alternate setting 1.
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0x81, 0x0a, 0, 0, 1))), [1])
            # PING -> PING_RESPONSE (Endpoint Number/Direction copied).
            f = yield from pyhost_ping(host, ept=1)
            self.assertEqual((f["subtype"], f["ept"]), (TP_SUB_PING_RESP, 1))
            # No data: zero-length DP, Seq Num 0, LPF.
            dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst])
            self.assertEqual([(d["seq"], d["lpf"], d["length"], p) for d, p, crc in dps], [(0, 1, 0, [])])
            # Transfers: one ACK TP per interval, then several ACK TPs per interval (Seq Num + NumP).
            bursts = [[max_burst], [2, 2], [1, 1, 1, 1], [max_burst], [1, 3]]
            state["read"] = 0
            for i, t in enumerate(transfers):
                yield from host.wait(lambda: state["written"] > i)
                yield from wait(16)
                dps = yield from pyhost_iso_in(host, ept=1, bursts=bursts[i])
                self.check_transfer(dps, t, max_burst)
                state["read"] = i + 1
            # No more data: zero-length DP.
            dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst], ping=False)
            self.assertEqual([(d["seq"], d["lpf"], d["length"]) for d, p, crc in dps], [(0, 1, 0)])
            # Isochronous: no TP response from the device (only PING_RESPONSEs and EP0 ACKs).
            yield from wait(32)
            iso_tps = [tp_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x04 and tp_fields(hp)["ept"] == 1]
            self.assertTrue(all(f["subtype"] == TP_SUB_PING_RESP for f in iso_tps))
            # Each DP sent once (no retry).
            iso_dps = [d for d, p, crc in pyhost_dps(host) if d["ept"] == 1]
            self.assertEqual(len(iso_dps), 2 + sum(max(1, -(-len(t)//1024)) for t in transfers))
            self.assertEqual((yield dut.iso.n_transfers), len(transfers))
            self.assertEqual((yield dut.iso.n_zlps), 2)
        run(dut, host, [writer(), scenario()])

    @pytest.mark.slow
    def test_iso_in_buffered(self):
        # Transfers written ahead (buffer full): sent one per service interval, in order.
        max_burst = 4
        dut  = DUT(max_burst=max_burst)
        host = PyHost(dut.phy)
        transfers = [payload(i, 1024 + 700*i) for i in range(3)]
        def writer():
            for t in transfers:
                yield from write_transfer(dut.iso, t)
        def scenario():
            yield from pyhost_link_init(host)
            yield from configure(host)
            yield from wait(1100)
            for t in transfers:
                dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst])
                self.check_transfer(dps, t, max_burst)
                yield from wait(400) # Next transfer completed in the freed slots.
        run(dut, host, [writer(), scenario()])

    @pytest.mark.slow
    def test_iso_in_interrupted(self):
        # Service interval ended before the end of the transfer (host requested 1 packet out of 3):
        # the remaining packets are dropped, the next interval gets the next transfer.
        max_burst = 4
        dut  = DUT(max_burst=max_burst)
        host = PyHost(dut.phy)
        transfers = [payload(0, 3000), payload(1, 2048)]
        def writer():
            for t in transfers:
                yield from write_transfer(dut.iso, t)
        def scenario():
            yield from pyhost_link_init(host)
            yield from configure(host)
            yield from wait(900)
            d0 = len(pyhost_dps(host))
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=1, tp_seq=0)
            yield from host.wait(lambda: len(pyhost_dps(host)) > d0)
            yield from wait(300)
            dps = pyhost_dps(host)[d0:]
            self.assertEqual([(d["seq"], d["lpf"], p) for d, p, crc in dps], [(0, 0, transfers[0][:1024])])
            dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst])
            self.check_transfer(dps, transfers[1], max_burst)
            self.assertEqual((yield dut.iso.n_drops), 2)
        run(dut, host, [writer(), scenario()])

    @pytest.mark.slow
    def test_iso_in_flush(self):
        # Flush: buffered transfers dropped, next interval: zero-length DP, then new data.
        max_burst = 2
        dut  = DUT(max_burst=max_burst)
        host = PyHost(dut.phy)
        state = {"flushed": False}
        def writer():
            yield from write_transfer(dut.iso, payload(0, 2048))
            yield from wait(16)
            yield dut.iso.flush.eq(1)
            yield
            yield dut.iso.flush.eq(0)
            yield from wait(64)
            state["flushed"] = True
        def scenario():
            yield from pyhost_link_init(host)
            yield from configure(host)
            yield from host.wait(lambda: state["flushed"])
            dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst])
            self.assertEqual([(d["seq"], d["lpf"], d["length"]) for d, p, crc in dps], [(0, 1, 0)])
            yield from write_transfer(dut.iso, payload(1, 1500))
            yield from wait(64)
            dps = yield from pyhost_iso_in(host, ept=1, bursts=[max_burst])
            self.check_transfer(dps, payload(1, 1500), max_burst)
        run(dut, host, [writer(), scenario()])

if __name__ == "__main__":
    unittest.main()
