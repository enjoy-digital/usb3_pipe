#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab gateware (Migen simulation): timebase, events records/arbitration/overflow/packets,
GPIO edges, power changes, DUT UART (internal loopback) byte path and events, markers."""

import unittest

from migen import *

from litex.gen import *

from agentlab.gateware.timebase   import AgentLabTimebase
from agentlab.gateware.events     import *
from agentlab.gateware.gpio       import AgentLabGPIO, AgentLabPower
from agentlab.gateware.uart       import AgentLabUART
from agentlab.gateware.instrument import AgentLab

# Helpers ------------------------------------------------------------------------------------------

def decode_records(words):
    """32-bit words (little-endian) to records [(type, source, value, timestamp)]."""
    data = b"".join(w.to_bytes(4, "little") for w in words)
    assert len(data) % EVENT_RECORD_BYTES == 0
    records = []
    for i in range(0, len(data), EVENT_RECORD_BYTES):
        r = data[i:i + EVENT_RECORD_BYTES]
        records.append((r[0], r[1], int.from_bytes(r[2:8], "little"),
            int.from_bytes(r[8:16], "little")))
    return records

class Collector:
    """Collects the words (and last flags) of a 32-bit words stream."""
    def __init__(self, source, ready=lambda cycle: 1):
        self.source = source
        self.ready  = ready
        self.words  = []
        self.lasts  = []

    @passive
    def generator(self):
        cycle = 0
        while True:
            ready = self.ready(cycle)
            yield self.source.ready.eq(ready)
            yield
            if ready and (yield self.source.valid):
                self.words.append((yield self.source.data))
                self.lasts.append((yield self.source.last))
            cycle += 1

# Events -------------------------------------------------------------------------------------------

class EventsDUT(LiteXModule):
    def __init__(self, n_ports=3, fifo_depth=16, port_depth=4):
        self.timebase = AgentLabTimebase()
        ts = self.timebase.timestamp
        self.ports = [AgentLabEventPort(ts, EVENT_MARKER, source=i, depth=port_depth)
            for i in range(n_ports)]
        for i, port in enumerate(self.ports):
            self.add_module(name=f"port{i}", module=port)
        self.events = AgentLabEvents(ts, [p.source for p in self.ports],
            [p.overflow for p in self.ports], fifo_depth=fifo_depth, idle_cycles=16)

def pulse(port, value):
    yield port.valid.eq(1)
    yield port.value.eq(value)
    yield
    yield port.valid.eq(0)

class TestAgentLabEvents(unittest.TestCase):
    def test_timebase_latch(self):
        dut = AgentLabTimebase()
        def gen():
            for _ in range(10):
                yield
            yield dut._latch.fields.latch.eq(1)
            yield
            ts = (yield dut.timestamp)
            yield dut._latch.fields.latch.eq(0)
            yield
            self.assertEqual((yield dut._value.status), ts)
        run_simulation(dut, gen())

    def test_records_arbitration(self):
        dut = EventsDUT()
        col = Collector(dut.events.source)
        state = {}
        def gen():
            for _ in range(8):
                yield
            # Simultaneous events on the 3 ports (same timestamp), then sequential ones.
            for p in dut.ports:
                yield p.valid.eq(1)
            for i, p in enumerate(dut.ports):
                yield p.value.eq(0x100 + i)
            yield
            state["ts0"] = (yield dut.timebase.timestamp) # Sampled at the end of this cycle.
            for p in dut.ports:
                yield p.valid.eq(0)
            yield
            yield dut.ports[1].valid.eq(1)
            yield dut.ports[1].value.eq(0x123456789abc)
            yield
            state["ts1"] = (yield dut.timebase.timestamp)
            yield dut.ports[1].valid.eq(0)
            for _ in range(64):
                yield
        run_simulation(dut, [gen(), col.generator()])
        records = decode_records(col.words)
        self.assertEqual(records, [
            (EVENT_MARKER, 0, 0x100, state["ts0"]),
            (EVENT_MARKER, 1, 0x101, state["ts0"]),
            (EVENT_MARKER, 2, 0x102, state["ts0"]),
            (EVENT_MARKER, 1, 0x123456789abc, state["ts1"]),
        ])
        # Packet ended (last) after the idle timeout, on the last word.
        self.assertEqual(col.lasts[-1], 1)
        self.assertEqual(sum(col.lasts), 1)

    def test_round_robin(self):
        # Two ports always requesting: interleaved records.
        dut = EventsDUT(n_ports=2, fifo_depth=64)
        col = Collector(dut.events.source)
        def gen():
            for i in range(8):
                for p in dut.ports:
                    yield p.valid.eq(1)
                    yield p.value.eq(i)
                yield
            for p in dut.ports:
                yield p.valid.eq(0)
            for _ in range(256):
                yield
        run_simulation(dut, [gen(), col.generator()])
        records = decode_records(col.words)
        sources = [r[1] for r in records]
        lost    = [r for r in records if r[0] == EVENT_OVERFLOW]
        # Ports FIFOs (4) absorb the burst partially: the merged records alternate.
        self.assertEqual(sources[:6], [0, 1]*3)
        for s in [0, 1]:
            values = [r[2] for r in records if r[1] == s and r[0] == EVENT_MARKER]
            self.assertEqual(values, sorted(values))
        self.assertEqual(len(records) - len(lost) + sum(r[2] for r in lost), 16)

    def test_overflow(self):
        # Host not reading: main FIFO (8) + ports FIFOs fill, lost events counted, overflow record
        # inserted when space is available.
        dut = EventsDUT(n_ports=1, fifo_depth=8, port_depth=2)
        col = Collector(dut.events.source, ready=lambda cycle: cycle > 400)
        def gen():
            for _ in range(4):
                yield
            for i in range(32):
                yield from pulse(dut.ports[0], i)
                yield
            self.assertGreater((yield dut.events._lost.status), 0)
            for _ in range(600):
                yield
        run_simulation(dut, [gen(), col.generator()])
        records = decode_records(col.words)
        markers  = [r for r in records if r[0] == EVENT_MARKER]
        overflow = [r for r in records if r[0] == EVENT_OVERFLOW]
        self.assertEqual(len(overflow), 1)
        self.assertEqual(len(markers) + overflow[0][2], 32)
        # Markers before the loss in order, overflow record after them.
        self.assertEqual([r[2] for r in markers], list(range(len(markers))))
        self.assertEqual(records[-1][0], EVENT_OVERFLOW)

    def test_max_packet(self):
        # Continuous records: packets ended at 1024 bytes (64 records).
        dut = EventsDUT(n_ports=1, fifo_depth=128, port_depth=128)
        col = Collector(dut.events.source)
        def gen():
            for i in range(100):
                yield from pulse(dut.ports[0], i)
            for _ in range(600):
                yield
        run_simulation(dut, [gen(), col.generator()])
        self.assertEqual(len(col.words), 400)
        ends = [i for i, last in enumerate(col.lasts) if last]
        self.assertEqual(ends, [255, 399])
        self.assertEqual([r[2] for r in decode_records(col.words)], list(range(100)))

    def test_packer_min_period(self):
        # Bursts of 10 words every 60 cycles (idle in between): packets ended at each burst, or
        # with min_period, at most every min_period cycles (bursts merged in the host transfers).
        for min_period, ends in [(0, [9, 19, 29, 39, 49, 59, 69, 79]), (200, [9, 39, 79])]:
            dut = AgentLabWordPacker(idle_cycles=8, min_period=min_period)
            col = Collector(dut.source)
            def gen():
                for burst in range(8):
                    for i in range(10):
                        yield dut.sink.valid.eq(1)
                        yield dut.sink.data.eq(10*burst + i)
                        yield
                        while not (yield dut.sink.ready):
                            yield
                    yield dut.sink.valid.eq(0)
                    for _ in range(50):
                        yield
                for _ in range(400):
                    yield
            run_simulation(dut, [gen(), col.generator()])
            self.assertEqual(col.words, list(range(80)))
            self.assertEqual([i for i, last in enumerate(col.lasts) if last], ends)

# GPIO / Power -------------------------------------------------------------------------------------

class VectorDUT(LiteXModule):
    def __init__(self):
        self.timebase = AgentLabTimebase()
        ts = self.timebase.timestamp
        self.gpio  = AgentLabGPIO(n=4)
        self.power = AgentLabPower(n=2)
        self.gpio_event  = AgentLabVectorEvents(ts, 4, EVENT_GPIO)
        self.power_event = AgentLabVectorEvents(ts, 2, EVENT_POWER)
        self.comb += [
            self.gpio_event.changes.eq(self.gpio.changes),
            self.gpio_event.levels.eq(self.gpio.levels),
            self.power_event.changes.eq(self.power.changes),
            self.power_event.levels.eq(self.power.levels),
        ]
        self.events = AgentLabEvents(ts, [self.gpio_event.source, self.power_event.source],
            idle_cycles=16)

class TestAgentLabGPIO(unittest.TestCase):
    def test_gpio_edges_power(self):
        dut = VectorDUT()
        col = Collector(dut.events.source)
        def gen():
            # Outputs/OE from CSRs.
            yield dut.gpio._out.storage.eq(0b1010)
            yield dut.gpio._oe.storage.eq(0b0011)
            yield
            yield
            self.assertEqual((yield dut.gpio.o),  0b1010)
            self.assertEqual((yield dut.gpio.oe), 0b0011)
            # Edges of masked GPIOs only (bit 3 not masked).
            yield dut.gpio._edge_mask.storage.eq(0b0111)
            for _ in range(4):
                yield
            yield dut.gpio.i.eq(0b1101) # Rising 0, 2, 3.
            for _ in range(8):
                yield
            self.assertEqual((yield dut.gpio._in.status), 0b1101)
            yield dut.gpio.i.eq(0b1100) # Falling 0.
            for _ in range(8):
                yield
            # Power switch 1 on.
            yield dut.power._out.storage.eq(0b10)
            for _ in range(8):
                yield
            self.assertEqual((yield dut.power._status.status), 0b10)
            for _ in range(64):
                yield
        run_simulation(dut, [gen(), col.generator()])
        records = decode_records(col.words)
        self.assertEqual([(r[0], r[1], r[2]) for r in records], [
            (EVENT_GPIO,  0, 1 | (0b1101 << 8)),
            (EVENT_GPIO,  2, 1 | (0b1101 << 8)),
            (EVENT_GPIO,  0, 0 | (0b1100 << 8)),
            (EVENT_POWER, 1, 1 | (0b10   << 8)),
        ])
        # Rising edges of the same cycle: same timestamp.
        self.assertEqual(records[0][3], records[1][3])
        self.assertLess(records[1][3], records[2][3])

# UART ---------------------------------------------------------------------------------------------

class TestAgentLabUART(unittest.TestCase):
    def test_uart_loopback_baudrate(self):
        # Internal loopback, 1/16 sys_clk baudrate (from the line coding).
        clk_freq = 16e6
        dut = AgentLabUART(None, clk_freq, baudrate=115200, fifo_depth=16)
        # Baudrate CSR (written from the line coding): finalized (CSR bank in a SoC).
        dut._baudrate.finalize(32, "big")
        dut.submodules += dut._baudrate
        message = list(b"AgentLab")
        received, tx_events, rx_events = [], [], []
        def gen():
            yield dut.line_baudrate.eq(int(clk_freq//16))
            yield dut.line_coding.eq(1)
            yield
            yield dut.line_coding.eq(0)
            for _ in range(4):
                yield
            self.assertEqual((yield dut._baudrate.storage), int(clk_freq//16))
            self.assertAlmostEqual((yield dut._tuning_word.status), 2**32//16, delta=2**12)
            for b in message:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(b)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(16*12*len(message) + 256):
                yield
        @passive
        def monitor():
            yield dut.source.ready.eq(1)
            while True:
                yield
                if (yield dut.source.valid):
                    received.append((yield dut.source.data))
                if (yield dut.tx_event):
                    tx_events.append((yield dut.tx_data))
                if (yield dut.rx_event):
                    rx_events.append((yield dut.rx_data))
        run_simulation(dut, [gen(), monitor()])
        self.assertEqual(received,  message)
        self.assertEqual(tx_events, message)
        self.assertEqual(rx_events, message)

# Instrument ---------------------------------------------------------------------------------------

class TestAgentLab(unittest.TestCase):
    def test_instrument_markers_uart_events(self):
        dut = AgentLab(sys_clk_freq=16e6, uart_pads=[None]*2, n_gpios=2, n_power=1)
        col = Collector(dut.source)
        def gen():
            # UART1 at sys_clk/16.
            yield dut.uarts[1].line_baudrate.eq(int(1e6))
            yield dut.uarts[1].line_coding.eq(1)
            yield
            yield dut.uarts[1].line_coding.eq(0)
            # Marker.
            yield dut._marker.storage.eq(0xdeadbeef)
            yield dut._marker.re.eq(1)
            yield
            yield dut._marker.re.eq(0)
            for _ in range(8):
                yield
            yield dut.uarts[1].sink.valid.eq(1)
            yield dut.uarts[1].sink.data.eq(0x5a)
            yield
            while not (yield dut.uarts[1].sink.ready):
                yield
            yield dut.uarts[1].sink.valid.eq(0)
            yield dut.uarts[1].source.ready.eq(1)
            for _ in range(16*12 + 256):
                yield
        run_simulation(dut, [gen(), col.generator()])
        records = decode_records(col.words)
        self.assertEqual([(r[0], r[1], r[2]) for r in records], [
            (EVENT_MARKER,  0, 0xdeadbeef),
            (EVENT_UART_RX, 1, 0x5a), # Middle of the stop bit (loopback).
            (EVENT_UART_TX, 1, 0x5a), # End of the stop bit.
        ])
        self.assertEqual(records, sorted(records, key=lambda r: r[3]))

if __name__ == "__main__":
    unittest.main()
