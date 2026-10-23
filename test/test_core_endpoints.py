#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *
from migen.sim import passive

from usb3_core.core import USB3CoreEndpointIN, USB3CoreEndpointOUT, USB3_CORE_EP_WORDS

# Daisho endpoint buffer model (usb3_ep.v handshakes: commit/arm acknowledged after a few cycles,
# 2 clocks RAM read latency) ------------------------------------------------------------------------

@passive
def ep_in_model(dut, packets, ready_rate=1.0, rng=random.Random(0)):
    """IN side: buffer written by the DUT, committed packets returned in `packets`."""
    mem = {}
    while True:
        yield dut.buf_in_ready.eq(int(rng.random() < ready_rate))
        yield
        if (yield dut.buf_in_wren):
            mem[(yield dut.buf_in_addr)] = (yield dut.buf_in_data)
        if (yield dut.buf_in_commit):
            length = (yield dut.buf_in_commit_len)
            packets.append([mem[i] for i in range(length//4)])
            mem = {}
            for _ in range(4):
                yield dut.buf_in_commit_ack.eq(1)
                yield
            yield dut.buf_in_commit_ack.eq(0)

def ep_out_model(dut, packets):
    """OUT side: buffer filled with the packets, read by the DUT."""
    for packet in packets:
        pipe = [0, 0]
        yield dut.buf_out_len.eq(4*len(packet))
        yield dut.buf_out_hasdata.eq(1)
        while True:
            addr = (yield dut.buf_out_addr)
            pipe = [packet[addr] if addr < len(packet) else 0xdeadbeef, pipe[0]]
            yield dut.buf_out_q.eq(pipe[1])
            yield
            if (yield dut.buf_out_arm):
                break
        yield dut.buf_out_hasdata.eq(0)
        for _ in range(4):
            yield dut.buf_out_arm_ack.eq(1)
            yield
        yield dut.buf_out_arm_ack.eq(0)
        for _ in range(4):
            yield

# Endpoints ----------------------------------------------------------------------------------------

class TestCoreEndpoints(unittest.TestCase):
    def test_endpoint_in(self):
        rng     = random.Random(1)
        packets = [[rng.randrange(2**32) for _ in range(n)] for n in [1, 10, 256, 3]]
        # A 300 words stream without last is split in 256 + 44 words packets.
        long    = [rng.randrange(2**32) for _ in range(300)]
        dut = USB3CoreEndpointIN()

        def generator(dut):
            for packet in packets + [long]:
                for i, data in enumerate(packet):
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(data)
                    yield dut.sink.last.eq((i == len(packet) - 1) and (packet is not long))
                    yield
                    while not (yield dut.sink.ready):
                        yield
            yield dut.sink.valid.eq(0)
            for _ in range(64):
                yield

        dut.packets = []
        run_simulation(dut, [generator(dut), ep_in_model(dut, dut.packets, ready_rate=0.8)])
        self.assertEqual(dut.packets[:4], packets)
        self.assertEqual(dut.packets[4], long[:USB3_CORE_EP_WORDS])

    def test_endpoint_out(self):
        rng     = random.Random(2)
        packets = [[rng.randrange(2**32) for _ in range(n)] for n in [1, 10, 256, 7]]
        dut = USB3CoreEndpointOUT()

        def receiver(dut):
            dut.received, packet = [], []
            for _ in range(4000):
                yield dut.source.ready.eq(int(rng.random() < 0.7))
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    packet.append((yield dut.source.data))
                    if (yield dut.source.last):
                        dut.received.append(packet)
                        packet = []

        run_simulation(dut, [ep_out_model(dut, packets), receiver(dut)])
        self.assertEqual(dut.received, packets)

if __name__ == "__main__":
    unittest.main()
