# This file is part of USB3-PIPE.
# SPDX-License-Identifier: BSD-2-Clause

"""Bulk IN burst coalescing, final-packet release and short-packet latency."""

import unittest

from migen import run_simulation, passive

from usb3_core.bulk import USB3BulkIN
from usb3_core.common import TYPE_DP
from test.test_core_bulk import EndpointHost
from test.model.host import dph_fields


class TestBulkCoalescing(unittest.TestCase):
    def packets(self, idle_cycles, lengths, gap):
        dut = USB3BulkIN(ept=1, max_burst=4, wr_idle_cycles=idle_cycles)
        host = EndpointHost(dut, 4)
        headers = []
        written = []

        def writer():
            for p, length in enumerate(lengths):
                for i in range(length//4):
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq((p << 16) | i)
                    yield dut.sink.be.eq(15)
                    yield dut.sink.last.eq(i == length//4 - 1)
                    yield
                    while not (yield dut.sink.ready):
                        yield
                yield dut.sink.valid.eq(0)
                written.append(host.cycle)
                if p == 0:
                    host.request()
                for _ in range(gap):
                    yield
            for _ in range(idle_cycles + 4096):
                if len(host.dps) == len(lengths):
                    break
                yield

        @passive
        def monitor():
            while True:
                if (yield dut.tx_hp.valid) and (yield dut.tx_hp.ready):
                    dw0, dw1 = (yield dut.tx_hp.dw0), (yield dut.tx_hp.dw1)
                    if (dw0 & 31) == TYPE_DP:
                        headers.append((host.cycle, dph_fields([dw0, dw1, 0, 0, 0])))
                yield

        run_simulation(dut, [writer(), host.generator(), monitor()])
        self.assertEqual(host.errors, [])
        self.assertEqual([length for seq, length in host.dps], lengths)
        self.assertEqual(host.data, [((p << 16) | i) for p, length in enumerate(lengths)
            for i in range(length//4)])
        return headers, written

    def test_coalesces_gap_but_eventually_releases_final_full_packet(self):
        old, _ = self.packets(64, [1024, 1024], gap=100)
        new, written = self.packets(1024, [1024, 1024], gap=100)
        self.assertEqual([h['eob'] for cycle, h in old], [1, 1])
        self.assertEqual([h['eob'] for cycle, h in new], [0, 1])
        self.assertLessEqual(new[-1][0] - written[-1], 1024 + 32)

    def test_short_packet_does_not_wait_for_coalescing_timeout(self):
        headers, written = self.packets(1024, [4], gap=0)
        self.assertLess(headers[0][0] - written[0], 64)

    def test_invalid_idle_cycles(self):
        for value in [0, -1, None, 1.5]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                USB3BulkIN(ept=1, wr_idle_cycles=value)
