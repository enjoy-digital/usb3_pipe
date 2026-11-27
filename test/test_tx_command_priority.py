# This file is part of USB3-PIPE.
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 1 TX command latency, packet atomicity, fences and recovery/drop, lc_sink (Link Commands
written concurrently with the packets)."""

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


def push(endpoint, frame, rng=None):
    """Write a frame (list of (data, ctrl, words) entries) to a stream endpoint."""
    for i, (data, ctrl, words) in enumerate(frame):
        yield endpoint.valid.eq(1)
        yield endpoint.data.eq(data)
        yield endpoint.ctrl.eq(ctrl)
        yield endpoint.words.eq(words)
        yield endpoint.first.eq(i == 0)
        yield endpoint.last.eq(i == len(frame) - 1)
        yield
        while not (yield endpoint.ready):
            yield
        if rng is not None and rng.randrange(4) == 0:
            yield endpoint.valid.eq(0)
            for _ in range(rng.randrange(1, 4)):
                yield
    yield endpoint.valid.eq(0)


def collect(dut, cycles, rng=None):
    """Output frames (lists of (data, ctrl, words)) read for `cycles` cycles."""
    frames = []
    current = []
    for _ in range(cycles):
        ready = rng is None or rng.randrange(4) != 0
        yield dut.source.ready.eq(ready)
        yield
        if (yield dut.source.valid) and ready:
            current.append(((yield dut.source.data), (yield dut.source.ctrl), (yield dut.source.words)))
            if (yield dut.source.last):
                frames.append(current)
                current = []
    return frames


class TestTXLinkCommandsSink(unittest.TestCase):
    def run_scenario(self, main, side, cycles=600, rng=None, drop_at=None):
        """main: frames written to sink, side: (delay, frame) written to lc_sink."""
        dut = LinkCommandPacketsBuffer(gen2_link_layout(), depth=128, with_lc_sink=True)
        out = []

        def main_writer():
            for frame in main:
                yield from push(dut.sink, frame, rng)

        def side_writer():
            for delay, frame in side:
                for _ in range(delay):
                    yield
                yield from push(dut.lc_sink, frame, rng)

        def reader():
            out.extend((yield from collect(dut, cycles, rng)))

        run_simulation(dut, [main_writer(), side_writer(), reader()])
        return out

    def test_command_written_while_a_packet_is_stored_is_output_before_it(self):
        a = packet(0x100, 40)
        good, credit = command(0), command(0x80)
        # lc_sink commands queued while the packet is still being written (40 entries): first out,
        # as the packet is selected once complete. Through sink they would be behind it.
        out = self.run_scenario([a], [(5, good), (0, credit)])
        self.assertEqual(out, [good, credit, a])
        out = self.run_scenario([a, command(0), command(0x80)], [])
        self.assertEqual(out[0], a)

    def test_command_written_after_the_packet_is_selected_waits_for_it(self):
        a, b = packet(0x100, 20), packet(0x200, 20)
        good = command(0)
        # The packet in output is never preempted; the command goes before the next packet.
        out = self.run_scenario([a, b], [(30, good)], cycles=400)
        self.assertEqual(out[0], a)
        self.assertIn(good, out)
        self.assertLess(out.index(good), out.index(b))

    def test_commands_of_both_inputs_are_whole_and_ordered(self):
        for seed in range(8):
            rng = random.Random(seed)
            a, b = packet(0x100, 30), packet(0x200, 17)
            main = [a, command(LCMD_LRTY), b, command(0), command(LCMD_LUP)]
            side = [(rng.randrange(0, 60), command(0x80 + i)) for i in range(4)]
            side = [(d if i == 0 else rng.randrange(0, 6), f) for i, (d, f) in enumerate(side)]
            out = self.run_scenario(main, side, cycles=1500, rng=rng)
            sent = main + [f for _, f in side]
            # Every frame once, entries not mixed: the output frames are exactly the written frames.
            self.assertEqual(sorted(map(tuple, out)), sorted(map(tuple, sent)), f"seed {seed}")
            # Order kept inside each input (packets, fenced commands, side commands).
            side_frames = [f for _, f in side]
            self.assertEqual([f for f in out if f in side_frames], side_frames, f"seed {seed}")
            # Packets and fenced commands keep their order (an unfenced LGOOD of sink may overtake
            # the queued packets).
            ordered = [f for f in main if f != command(0)]
            self.assertEqual([f for f in out if f in ordered], ordered, f"seed {seed}")

    def test_drop_clears_both_inputs(self):
        dut = LinkCommandPacketsBuffer(gen2_link_layout(), depth=128, with_lc_sink=True)
        out = []

        def writers():
            yield from push(dut.lc_sink, command(0))
            yield dut.drop.eq(1)
            for _ in range(4):
                yield
            yield dut.drop.eq(0)
            for _ in range(4):
                yield
            yield from push(dut.lc_sink, command(0x80))

        def reader():
            out.extend((yield from collect(dut, 200)))

        run_simulation(dut, [writers(), reader()])
        # The command written before the drop (output stalled, not ready) is gone, the next one works.
        self.assertEqual(out[-1], command(0x80))
        self.assertNotIn(command(0), out[1:])
