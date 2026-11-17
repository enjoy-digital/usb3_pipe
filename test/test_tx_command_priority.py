# This file is part of USB3-PIPE.
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 1 TX command latency, packet atomicity, fences and recovery/drop."""

import random
import unittest

from migen import run_simulation

from usb3_pipe.framing import LinkCommandPacketsBuffer, gen2_link_layout, SLC, EPF
from usb3_core.common import LCMD_LRTY, LCMD_LGO_U, LCMD_LUP


def packet(tag, length=20):
    return [(tag + i, 0, 2) for i in range(length)]


def command(info):
    word = info | (info << 16)
    wire = int.from_bytes(word.to_bytes(4, "big"), "little")
    return [(SLC | (SLC << 8) | (SLC << 16) | (EPF << 24), 15, 1), (wire, 0, 1)]


class TestTXCommandPriority(unittest.TestCase):
    def check_order(self, incoming, expected, initial_stall=True):
        dut = LinkCommandPacketsBuffer(gen2_link_layout(), depth=128)
        received = []
        queued = [False]
        rng = random.Random(42)

        def writer():
            for frame in incoming:
                for i, (data, ctrl, words) in enumerate(frame):
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(data)
                    yield dut.sink.ctrl.eq(ctrl)
                    yield dut.sink.words.eq(words)
                    yield dut.sink.first.eq(i == 0)
                    yield dut.sink.last.eq(i == len(frame) - 1)
                    yield
                    while not (yield dut.sink.ready):
                        yield
            yield dut.sink.valid.eq(0)
            queued[0] = True

        def reader():
            # A data packet is selected and held before the command is queued.
            yield dut.source.ready.eq(0)
            while initial_stall and not queued[0]:
                yield
            current = []
            held = None
            total = sum(map(len, expected))
            count = 0
            for _ in range(max(2000, total*8)):
                ready = rng.randrange(4) != 0
                yield dut.source.ready.eq(ready)
                yield
                if not (yield dut.source.valid):
                    continue
                entry = ((yield dut.source.data), (yield dut.source.ctrl),
                    (yield dut.source.words), (yield dut.source.first), (yield dut.source.last))
                if held is not None:
                    self.assertEqual(entry, held, "output changed while back-pressured")
                if not ready:
                    held = entry
                    continue
                held = None
                if not current:
                    self.assertEqual(entry[3], 1)
                current.append(entry[:3])
                count += 1
                if entry[4]:
                    received.append(current)
                    current = []
                if count == total:
                    break
            self.assertEqual(received, expected)

        run_simulation(dut, [writer(), reader()])

    def test_acknowledgement_overtakes_queued_data_without_preemption(self):
        a, b = packet(0x100), packet(0x200)
        good, credit, bad = command(0), command(0x80), command(0x180)
        self.check_order([a, b, good, credit, bad], [a, good, credit, bad, b])

    def test_retry_fence_keeps_older_packets_before_lrty_and_newer_packets_after(self):
        a, b, c = packet(0x100), packet(0x200), packet(0x300)
        good = command(0)
        for info in [LCMD_LRTY, LCMD_LGO_U | 1, LCMD_LGO_U | 2, LCMD_LGO_U | 3, LCMD_LUP]:
            with self.subTest(command=info):
                barrier = command(info)
                self.check_order([a, b, barrier, c, good], [a, b, barrier, good, c])

    def test_fences_and_descriptors_across_counter_wrap(self):
        frames = []
        for i in range(600):
            frames += [packet(0x1000 + i, length=1), command(LCMD_LRTY)]
        self.check_order(frames, frames, initial_stall=False)

    def test_drop_discards_partial_packet_and_resynchronizes_at_first(self):
        dut = LinkCommandPacketsBuffer(gen2_link_layout(), depth=32)
        got = []

        def writer():
            yield dut.source.ready.eq(0)
            # Drop with a selected packet, a queued command/descriptor, and an
            # incomplete subsequent packet. All three queues must be cleared.
            for frame in [packet(0x100, length=1), command(0)]:
                for i, (data, ctrl, words) in enumerate(frame):
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(data)
                    yield dut.sink.ctrl.eq(ctrl)
                    yield dut.sink.words.eq(words)
                    yield dut.sink.first.eq(i == 0)
                    yield dut.sink.last.eq(i == len(frame) - 1)
                    yield
                    while not (yield dut.sink.ready):
                        yield
            yield dut.sink.first.eq(1)
            yield dut.sink.last.eq(0)
            yield dut.sink.ctrl.eq(0)
            yield dut.sink.data.eq(0xdead)
            yield
            yield dut.drop.eq(1)
            for _ in range(4):
                yield
            yield dut.drop.eq(0)
            yield dut.sink.first.eq(0)
            yield dut.sink.last.eq(1)
            yield dut.sink.data.eq(0xbeef)
            yield
            yield dut.sink.first.eq(1)
            yield dut.sink.data.eq(0x1234)
            yield
            while not (yield dut.sink.ready):
                yield
            yield dut.sink.valid.eq(0)
            yield dut.source.ready.eq(1)
            for _ in range(20):
                if (yield dut.source.valid):
                    got.append((yield dut.source.data))
                yield

        run_simulation(dut, writer())
        self.assertEqual(got, [0x1234])
