#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Packet Slots/Writer (Migen simulation): write/read, lengths, split at Max Packet Size,
slots wrap/full, flush, Last Packet Flag."""

import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common    import endpoint_layout
from usb3_core.endpoints import MAX_PACKET_SIZE, MAX_PACKET_WORDS
from usb3_core.slots     import USB3PacketSlots, USB3PacketWriter

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self, n=4, max_packets=None, with_enable=False):
        self.sink   = stream.Endpoint(endpoint_layout())
        self.rd_ptr = Signal(log2_int(n) + 1)
        self.flush  = Signal()
        self.enable = Signal(reset=1)
        self.slots  = slots = USB3PacketSlots(n)
        self.writer = USB3PacketWriter(slots, self.sink,
            rd_ptr      = self.rd_ptr,
            flush       = self.flush,
            enable      = self.enable if with_enable else None,
            max_packets = max_packets,
        )

# Helpers ------------------------------------------------------------------------------------------

def write_packet(dut, data, last=True):
    """Write a packet (bytes) to the sink."""
    words = [data[i:i+4] for i in range(0, len(data), 4)]
    for i, w in enumerate(words):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(int.from_bytes(bytes(w + [0]*(4 - len(w))), "little"))
        yield dut.sink.be.eq((1 << len(w)) - 1)
        yield dut.sink.last.eq(last & (i == len(words) - 1))
        yield
        while not (yield dut.sink.ready):
            yield
    yield dut.sink.valid.eq(0)
    yield

def read_slot(dut, slot, length):
    """Read a slot (length bytes) from the read port."""
    data = []
    for i in range((length + 3)//4):
        yield dut.slots.rd_port.adr.eq(slot*MAX_PACKET_WORDS + i)
        yield dut.slots.rd_port.re.eq(1)
        yield
        yield dut.slots.rd_port.re.eq(0)
        yield
        data += list(((yield dut.slots.rd_port.dat_r)).to_bytes(4, "little"))
    return data[:length]

def payload(i, n):
    return [(i*37 + j) & 0xff for j in range(n)]

# Test ---------------------------------------------------------------------------------------------

class TestPacketSlots(unittest.TestCase):
    def test_adr(self):
        slots = USB3PacketSlots(8)
        self.assertEqual(slots.wbits, log2_int(MAX_PACKET_WORDS))
        self.assertEqual(len(slots.wr_port.adr), slots.sbits + slots.wbits)
        self.assertEqual(len(slots.adr(Signal(3))), slots.sbits + slots.wbits)
        self.assertEqual(len(slots.adr(Signal(3), Signal(9))), slots.sbits + slots.wbits)

    def test_write_read_lengths(self):
        # Short packets (1..4 trailing bytes), a max size packet, a packet split at Max Packet Size.
        dut = DUT(n=4)
        packets = [payload(0, 13), payload(1, 4), payload(2, 1), payload(3, MAX_PACKET_SIZE)]
        def gen():
            for k, p in enumerate(packets):
                yield from write_packet(dut, p)
                self.assertEqual((yield dut.writer.ptr), k + 1)
                self.assertEqual((yield dut.slots.lengths[k]), len(p))
                self.assertEqual((yield from read_slot(dut, k, len(p))), p)
            yield dut.rd_ptr.eq(4)
            yield
            # Split: 1024 + 6 bytes packet -> 2 slots (1024, 6).
            p = payload(4, MAX_PACKET_SIZE + 6)
            yield from write_packet(dut, p)
            self.assertEqual((yield dut.writer.ptr), 6)
            self.assertEqual((yield dut.slots.lengths[0]), MAX_PACKET_SIZE)
            self.assertEqual((yield dut.slots.lengths[1]), 6)
            self.assertEqual((yield from read_slot(dut, 0, MAX_PACKET_SIZE)), p[:MAX_PACKET_SIZE])
            self.assertEqual((yield from read_slot(dut, 1, 6)), p[MAX_PACKET_SIZE:])
        run_simulation(dut, gen())

    def test_full_wrap(self):
        # Full when n slots used, wrap once slots freed (rd_ptr).
        dut = DUT(n=2)
        def gen():
            yield from write_packet(dut, payload(0, 8))
            yield from write_packet(dut, payload(1, 8))
            self.assertEqual((yield dut.writer.ptr), 2)
            yield dut.sink.valid.eq(1)
            yield
            self.assertEqual((yield dut.sink.ready), 0)
            yield dut.sink.valid.eq(0)
            yield dut.rd_ptr.eq(1)
            yield
            self.assertEqual((yield dut.sink.ready), 1)
            for k in range(2, 6):
                yield from write_packet(dut, payload(k, 4*k + 1))
                yield dut.rd_ptr.eq((yield dut.writer.ptr) - 1)
                yield
                slot = k % 2
                self.assertEqual((yield dut.writer.ptr), (k + 1) % 4)
                self.assertEqual((yield dut.slots.lengths[slot]), 4*k + 1)
                self.assertEqual((yield from read_slot(dut, slot, 4*k + 1)), payload(k, 4*k + 1))
        run_simulation(dut, gen())

    def test_flush(self):
        # Packet being written dropped on flush, next packet written from word 0 of the same slot.
        dut = DUT(n=4)
        def gen():
            yield from write_packet(dut, payload(0, 40), last=False)
            yield dut.flush.eq(1)
            yield
            yield dut.flush.eq(0)
            yield
            self.assertEqual((yield dut.writer.ptr), 0)
            p = payload(1, 10)
            yield from write_packet(dut, p)
            self.assertEqual((yield dut.writer.ptr), 1)
            self.assertEqual((yield dut.slots.lengths[0]), 10)
            self.assertEqual((yield from read_slot(dut, 0, 10)), p)
        run_simulation(dut, gen())

    def test_enable_lpf(self):
        # Transfers of up to 3 packets: LPF on last or 3rd packet, sink gated by enable.
        dut = DUT(n=8, max_packets=3, with_enable=True)
        def gen():
            yield dut.enable.eq(0)
            yield dut.sink.valid.eq(1)
            yield
            self.assertEqual((yield dut.sink.ready), 0)
            yield dut.sink.valid.eq(0)
            yield dut.enable.eq(1)
            yield
            # 4 x 1024 + 8 bytes: LPF on the 3rd packet, then on the last (short) one.
            yield from write_packet(dut, payload(0, 4*MAX_PACKET_SIZE + 8))
            # 2 packets transfer: LPF on the last one.
            yield from write_packet(dut, payload(1, MAX_PACKET_SIZE + 4))
            lpfs = []
            for k in range(7):
                lpfs.append((yield dut.writer.lpfs[k]))
            self.assertEqual(lpfs, [0, 0, 1, 0, 1, 0, 1])
            self.assertEqual((yield dut.writer.ptr), 7)
        run_simulation(dut, gen())

if __name__ == "__main__":
    unittest.main()
