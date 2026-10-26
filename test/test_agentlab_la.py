#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab logic analyzer gateware (Migen simulation, LiteDRAM native ports memory model):
capture/stream modes, triggers (immediate, pattern, edge, external), pre-trigger, divider, start/
trigger timestamps, ring buffer full/overflow, stop; stream parsed with the host parser."""

import random
import unittest
import collections

from migen import *

from litex.gen import *

from litedram.common import LiteDRAMNativePort

from agentlab.gateware.timebase   import AgentLabTimebase
from agentlab.gateware.la         import *
from agentlab.gateware.events     import EVENT_LA_TRIGGER, EVENT_MARKER
from agentlab.gateware.instrument import AgentLab

from agentlab.host.la import LAStreamParser

# LiteDRAM Native Ports Model ----------------------------------------------------------------------

class DRAMModel:
    """Write/read native ports on a word memory: commands executed in order (as a LiteDRAM bank
    machine: a read sees the writes accepted before it), read data after `latency` cycles,
    commands accepted when cmd_ready(cycle)."""
    def __init__(self, wport, rport, latency=10, cmd_ready=lambda cycle: 1):
        self.wport     = wport
        self.rport     = rport
        self.latency   = latency
        self.cmd_ready = cmd_ready
        self.mem       = {}
        self.writes    = 0
        self.reads     = 0

    @passive
    def generator(self):
        wport, rport = self.wport, self.rport
        ops   = collections.deque() # ("w"/"r", addr), in acceptance order.
        rdata = collections.deque() # (cycle, data).
        cycle = 0
        while True:
            w_ready  = self.cmd_ready(cycle)
            r_ready  = self.cmd_ready(cycle + 3)
            wd_ready = int(len(ops) > 0 and ops[0][0] == "w")
            rd_valid = int(len(rdata) > 0 and rdata[0][0] <= cycle)
            yield wport.cmd.ready.eq(w_ready)
            yield rport.cmd.ready.eq(r_ready)
            yield wport.wdata.ready.eq(wd_ready)
            yield rport.rdata.valid.eq(rd_valid)
            if rd_valid:
                yield rport.rdata.data.eq(rdata[0][1])
            yield
            # Transfers of this cycle.
            exec_read = len(ops) > 0 and ops[0][0] == "r"
            if wd_ready and (yield wport.wdata.valid):
                self.mem[ops[0][1]] = (yield wport.wdata.data)
                ops.popleft()
                self.writes += 1
            elif exec_read:
                rdata.append((cycle + self.latency, self.mem.get(ops[0][1], 0)))
                ops.popleft()
                self.reads += 1
            if w_ready and (yield wport.cmd.valid):
                assert (yield wport.cmd.we)
                ops.append(("w", (yield wport.cmd.addr)))
            if r_ready and (yield rport.cmd.valid):
                ops.append(("r", (yield rport.cmd.addr)))
            if rd_valid and (yield rport.rdata.ready):
                rdata.popleft()
            cycle += 1

# DUT ----------------------------------------------------------------------------------------------

TS_BITS = 12 # Channels 0-11: timestamp bits (sample values follow the timebase), 12-15: ctrl.

class LADUT(LiteXModule):
    def __init__(self, dw=64, ring_size=1024, block_samples=16, ring_base=0x400, **kwargs):
        self.timebase = AgentLabTimebase()
        self.ctrl     = Signal(4)
        inputs        = Signal(16)
        self.comb += inputs.eq(Cat(self.timebase.timestamp[:TS_BITS], self.ctrl))
        self.wport = LiteDRAMNativePort("write", 24, dw)
        self.rport = LiteDRAMNativePort("read",  24, dw)
        self.la = AgentLabLA(inputs, self.timebase.timestamp, self.wport, self.rport,
            ring_base     = ring_base,
            ring_size     = ring_size,
            block_samples = block_samples,
            idle_cycles   = 16,
            packet_period = 512,
            **{"sample_fifo_depth": 64, "output_fifo_depth": 64, "dma_fifo_depth": 8, **kwargs})

class Reader:
    """Bulk IN side: reads the LA source words (ready pattern), feeds the host parser."""
    def __init__(self, source, ready=lambda cycle: 1):
        self.source = source
        self.ready  = ready
        self.parser = LAStreamParser(sys_clk_freq=125e6)
        self.words  = []
        self.bes    = []
        self.lasts  = []
        self.errors = 0

    @passive
    def generator(self):
        cycle = 0
        while True:
            ready = self.ready(cycle)
            yield self.source.ready.eq(ready)
            yield
            if ready and (yield self.source.valid):
                w = (yield self.source.data)
                self.words.append(w)
                self.bes.append((yield self.source.be))
                self.lasts.append((yield self.source.last))
                try:
                    self.parser.feed(w.to_bytes(4, "little"))
                except ValueError: # Aborted capture (re-arm): restarted on the next header.
                    self.parser = LAStreamParser(sys_clk_freq=125e6)
                    self.errors += 1
            cycle += 1

def csr_set(csr, value):
    """CSRStorage value (and its fields: CSRs not simulated as submodules)."""
    yield csr.storage.eq(value)
    if hasattr(csr, "fields"):
        for f in csr.fields.fields:
            yield getattr(csr.fields, f.name).eq((value >> f.offset) & (2**f.size - 1))

def configure(la, mode=LA_MODE_CAPTURE, divider=1, source=LA_TRIGGER_IMMEDIATE, mask=0, value=0,
    edge=None, polarity=LA_EDGE_RISING, ext=0, pre=0, length=64):
    yield from csr_set(la._mode, mode)
    yield from csr_set(la._divider, divider)
    yield from csr_set(la._trigger_source, source)
    yield from csr_set(la._trigger_mask, mask)
    yield from csr_set(la._trigger_value, value)
    yield from csr_set(la._trigger_edge, 0 if edge is None else (1 | (edge << 1) | (polarity << 6)))
    yield from csr_set(la._trigger_ext, ext)
    yield from csr_set(la._pre_trigger, pre)
    yield from csr_set(la._length, length)
    yield

def control(la, arm=0, stop=0):
    yield la._control.fields.arm.eq(arm)
    yield la._control.fields.stop.eq(stop)
    yield
    yield la._control.fields.arm.eq(0)
    yield la._control.fields.stop.eq(0)

def status(la):
    f = la._status.fields
    return {"state": (yield f.state), "triggered": (yield f.triggered),
        "overflow": (yield f.overflow)}

def wait_parser(reader, timeout=20000):
    for _ in range(timeout):
        if reader.parser.done:
            return
        yield
    raise AssertionError("LA stream not complete ({} samples).".format(
        len(reader.parser.samples)))

def check_timebase(test, capture, divider):
    """Channels 0-11 are the timestamp (2 cycles of input synchronizer): every sample matches its
    timestamp in the shared timebase."""
    mask = (1 << TS_BITS) - 1
    for k, s in enumerate(capture.samples):
        test.assertEqual(s & mask, (capture.timestamp(k) - 2) & mask, "sample {}".format(k))
    test.assertEqual(capture.divider, divider)
    test.assertEqual(capture.trigger_timestamp,
        capture.start_timestamp + capture.trigger_offset*divider)

def run(dut, scenario, reader_ready=lambda cycle: 1, cmd_ready=lambda cycle: 1, vcd=None):
    dram   = DRAMModel(dut.wport, dut.rport, cmd_ready=cmd_ready)
    reader = Reader(dut.la.source, ready=reader_ready)
    run_simulation(dut, [scenario(dram, reader), dram.generator(), reader.generator()],
        vcd_name=vcd)
    return dram, reader

# Tests --------------------------------------------------------------------------------------------

class TestAgentLabLA(unittest.TestCase):
    def capture(self, dw=64, mode=LA_MODE_CAPTURE, divider=1, pre=0, length=40, trigger={},
        ctrl=None,
        reader_ready=lambda cycle: 1, cmd_ready=lambda cycle: 1, delay=0, **kwargs):
        dut = LADUT(dw=dw, **kwargs)
        result = {}
        def scenario(dram, reader):
            yield from configure(dut.la, mode=mode, divider=divider, pre=pre, length=length,
                **trigger)
            for _ in range(delay):
                yield
            yield from control(dut.la, arm=1)
            if ctrl is not None:
                yield from ctrl(dut)
            yield from wait_parser(reader)
            result["status"] = (yield from status(dut.la))
            result["count"]  = (yield dut.la._count.status)
        dram, reader = run(dut, scenario, reader_ready=reader_ready, cmd_ready=cmd_ready)
        return reader.parser.capture, result

    def test_immediate(self):
        for dw in [32, 64, 128]:
            capture, r = self.capture(dw=dw, length=41)
            self.assertEqual(len(capture.samples), 41)
            self.assertEqual(capture.header.count, 41)
            self.assertEqual(capture.trigger_offset, 0)
            self.assertEqual(capture.channels, 16)
            self.assertFalse(capture.overflow)
            self.assertEqual(r["status"]["state"], LA_STATE_DONE)
            self.assertEqual(r["count"], 41)
            check_timebase(self, capture, 1)

    def test_divider(self):
        for divider in [2, 3, 7]:
            capture, _ = self.capture(divider=divider, length=33)
            self.assertEqual(len(capture.samples), 33)
            check_timebase(self, capture, divider)

    def test_pattern_pre_trigger(self):
        # Trigger on timestamp bits 4-11 == 0x13 (several samples match: first one).
        mask, value = 0xff0, 0x130
        for dw, pre, divider in [(64, 20, 1), (128, 37, 2), (32, 5, 3)]:
            capture, _ = self.capture(dw=dw, pre=pre, length=pre + 30, divider=divider,
                trigger=dict(source=LA_TRIGGER_PATTERN, mask=mask, value=value), delay=7)
            self.assertEqual(len(capture.samples), pre + 30)
            self.assertEqual(capture.trigger_offset, pre)
            t = capture.samples[pre]
            self.assertEqual(t & mask, value)
            self.assertNotEqual(capture.samples[pre - 1] & mask, value)
            check_timebase(self, capture, divider)

    def test_pre_trigger_partial(self):
        # Trigger before the pre-trigger depth is reached: offset = samples before the trigger.
        def ctrl(dut):
            for _ in range(30):
                yield
            yield dut.ctrl.eq(0b0001)
        capture, _ = self.capture(pre=100, length=150, ctrl=ctrl,
            trigger=dict(source=LA_TRIGGER_PATTERN, mask=1 << 12, value=1 << 12))
        off = capture.trigger_offset
        self.assertTrue(20 < off < 100)
        self.assertEqual(len(capture.samples), off + 50)
        self.assertEqual((capture.samples[off] >> 12) & 1, 1)
        self.assertEqual((capture.samples[off - 1] >> 12) & 1, 0)
        check_timebase(self, capture, 1)

    def test_edge(self):
        # Falling edge on channel 13, qualified by channel 14 high (pattern + edge).
        def ctrl(dut):
            yield dut.ctrl.eq(0b0010) # ch13 high, ch14 low.
            for _ in range(40):
                yield
            yield dut.ctrl.eq(0b0000) # Falling edge, ch14 low: no trigger.
            for _ in range(40):
                yield
            yield dut.ctrl.eq(0b0110)
            for _ in range(40):
                yield
            yield dut.ctrl.eq(0b0100) # Falling edge, ch14 high: trigger.
        capture, _ = self.capture(pre=10, length=30, divider=2, ctrl=ctrl,
            trigger=dict(source=LA_TRIGGER_PATTERN, mask=1 << 14, value=1 << 14, edge=13,
                polarity=LA_EDGE_FALLING))
        s = capture.samples
        self.assertEqual(capture.trigger_offset, 10)
        self.assertEqual(((s[9] >> 13) & 1, (s[10] >> 13) & 1), (1, 0))
        self.assertEqual((s[10] >> 14) & 1, 1)
        check_timebase(self, capture, 2)

    def test_external_trigger_event(self):
        dut = LADUT()
        result = {}
        def scenario(dram, reader):
            yield from configure(dut.la, source=LA_TRIGGER_EXTERNAL, ext=1 << LA_EXT_MARKER,
                pre=8, length=24, divider=5)
            yield from control(dut.la, arm=1)
            for _ in range(100):
                yield
            # GPIO event: not selected.
            yield dut.la.ext_trigger.eq(1 << LA_EXT_GPIO)
            yield
            yield dut.la.ext_trigger.eq(0)
            for _ in range(50):
                yield
            self.assertEqual((yield from status(dut.la))["state"], LA_STATE_ARMED)
            # Marker event: trigger (on the next sample), trigger event pulse.
            yield dut.la.ext_trigger.eq(1 << LA_EXT_MARKER)
            ts = (yield dut.timebase.timestamp)
            yield
            yield dut.la.ext_trigger.eq(0)
            for _ in range(6):
                if (yield dut.la.trigger_event):
                    result["event_ts"]    = (yield dut.timebase.timestamp)
                    result["event_value"] = (yield dut.la.trigger_value)
                yield
            result["marker_ts"] = ts
            yield from wait_parser(reader)
        dram, reader = run(dut, scenario)
        capture = reader.parser.capture
        self.assertEqual(len(capture.samples), 24)
        self.assertEqual(capture.trigger_offset, 8)
        self.assertEqual(capture.trigger_timestamp, result["event_ts"])
        self.assertEqual(capture.samples[8], result["event_value"])
        self.assertTrue(0 <= capture.trigger_timestamp - result["marker_ts"] <= 5)
        check_timebase(self, capture, 5)

    def test_capture_stalls(self):
        # DRAM commands and USB host stalls: same samples (DRAM absorbs, no overflow at /4).
        rng = random.Random(1)
        rd  = [rng.random() < 0.6 for _ in range(4096)]
        cmd = [rng.random() < 0.7 for _ in range(4096)]
        capture, r = self.capture(dw=128, divider=4, pre=17, length=200,
            trigger=dict(source=LA_TRIGGER_PATTERN, mask=0x0f0, value=0x050),
            reader_ready=lambda cycle: rd[cycle % 4096], cmd_ready=lambda cycle: cmd[cycle % 4096])
        self.assertEqual(len(capture.samples), 200)
        self.assertFalse(capture.overflow)
        check_timebase(self, capture, 4)

    def test_capture_full_rate_dram_stalls(self):
        # Full rate (divider 1), periodic DRAM stalls (as refreshes): the stalls are absorbed and
        # caught up (FIFO on DRAM words, drained faster than the samplerate), no overflow.
        capture, r = self.capture(dw=128, ring_size=16384, length=3000,
            cmd_ready=lambda cycle: (cycle % 1000) >= 60)
        self.assertEqual(len(capture.samples), 3000)
        self.assertFalse(capture.overflow)
        check_timebase(self, capture, 1)

    def test_capture_ring_full(self):
        # Capture longer than the ring (256 samples): ends when full (overflow), samples intact.
        capture, r = self.capture(dw=64, ring_size=1024, pre=10, length=400, divider=1)
        self.assertTrue(capture.overflow)
        self.assertTrue(r["status"]["overflow"])
        n = len(capture.samples)
        self.assertEqual(capture.header.count, n)
        self.assertTrue(256 < n <= 256 + 64 + 8, n) # Ring + sample FIFO (+ converter).
        check_timebase(self, capture, 1)

    def test_stream(self):
        # Stream mode: 4 x the ring, host keeping up (with stalls): no overflow.
        rng = random.Random(2)
        rd  = [rng.random() < 0.7 for _ in range(4096)]
        capture, r = self.capture(dw=64, ring_size=1024, mode=LA_MODE_STREAM, pre=30,
            length=1024, divider=2, trigger=dict(source=LA_TRIGGER_PATTERN, mask=0xf00,
            value=0x300), reader_ready=lambda cycle: rd[cycle % 4096])
        self.assertEqual(capture.header.mode, LA_MODE_STREAM)
        self.assertEqual(capture.header.count, 1024)
        self.assertEqual(len(capture.samples), 1024)
        self.assertFalse(capture.overflow)
        check_timebase(self, capture, 2)

    def test_stream_overflow(self):
        # Host too slow: ring full, capture ended (overflow), all the captured samples received.
        capture, r = self.capture(dw=64, ring_size=1024, mode=LA_MODE_STREAM, pre=0,
            length=4000, divider=1, reader_ready=lambda cycle: (cycle % 4) == 0)
        n = len(capture.samples)
        self.assertTrue(capture.overflow)
        self.assertEqual(capture.header.count, 4000)
        self.assertEqual(r["count"], n)
        self.assertTrue(256 < n < 4000, n)
        check_timebase(self, capture, 1)

    def test_stream_stop(self):
        dut = LADUT()
        def scenario(dram, reader):
            yield from configure(dut.la, mode=LA_MODE_STREAM, length=0, divider=3)
            yield from control(dut.la, arm=1)
            for _ in range(1500):
                yield
            self.assertEqual((yield from status(dut.la))["state"], LA_STATE_TRIGGERED)
            yield from control(dut.la, stop=1)
            yield from wait_parser(reader)
        dram, reader = run(dut, scenario)
        capture = reader.parser.capture
        self.assertEqual(capture.header.count, 0)
        self.assertTrue(450 < len(capture.samples) < 520)
        self.assertFalse(capture.overflow)
        check_timebase(self, capture, 3)

    def test_armed_overflow(self):
        # DRAM writes stalled while armed (no trigger): samples lost, capture ended with the samples
        # before the overflow (up to pre-trigger, also more than captured), overflow flagged. The
        # stream is well formed (blocks matching the header count, a single last block, all bytes
        # valid, ended by a packet end) and the next capture works.
        for pre in [10, 100, 1000]:
            dut    = LADUT(ring_size=4096)
            result = {}
            def scenario(dram, reader):
                yield from configure(dut.la, source=LA_TRIGGER_EXTERNAL, ext=1 << LA_EXT_MARKER,
                    pre=pre, length=pre + 50)
                yield from control(dut.la, arm=1)
                yield from wait_parser(reader)
                result["status"] = (yield from status(dut.la))
                result["count"]  = (yield dut.la._count.status)
                result["first"]  = reader.parser.capture
                for _ in range(64):
                    yield
                result["words"] = len(reader.words)
                # Next capture.
                reader.parser = LAStreamParser(sys_clk_freq=125e6)
                yield from configure(dut.la, length=40)
                yield from control(dut.la, arm=1)
                yield from wait_parser(reader)
            dram, reader = run(dut, scenario, cmd_ready=lambda cycle: not (100 <= cycle < 3100))
            capture = result["first"]
            n = len(capture.samples)
            self.assertEqual(result["status"], {"state": LA_STATE_DONE, "triggered": 0,
                "overflow": 1})
            self.assertTrue(capture.overflow)
            self.assertEqual(capture.header.count, n)
            self.assertEqual(result["count"], n)
            self.assertEqual(capture.trigger_offset, n)
            self.assertTrue(0 < n <= pre, n)
            check_timebase(self, capture, 1)
            # Stream: header, blocks (last one last), nothing after, packet ended.
            words = reader.words[:result["words"]]
            self.assertEqual(words[0], LA_MAGIC)
            i, total, lasts = LA_HEADER_WORDS, 0, 0
            while i < len(words):
                self.assertEqual(words[i] >> 24, LA_BLOCK_MAGIC)
                count = words[i] & 0xffff
                total += count
                i     += 1 + count
                if (words[i - 1 - count] >> 16) & LA_BLOCK_LAST:
                    lasts += 1
                    break
            self.assertEqual((i, total, lasts), (len(words), n, 1))
            self.assertEqual(set(reader.bes[:len(words)]), {0b1111})
            self.assertEqual(reader.lasts[len(words) - 1], 1)
            # Next capture.
            capture = reader.parser.capture
            self.assertEqual(len(capture.samples), 40)
            self.assertFalse(capture.overflow)
            check_timebase(self, capture, 1)

    def test_rearm_abort(self):
        # Stop while armed (no readout), re-arm during a capture: only the last capture sent.
        dut = LADUT()
        def scenario(dram, reader):
            yield from configure(dut.la, source=LA_TRIGGER_PATTERN, mask=1 << 15, value=1 << 15,
                length=20)
            yield from control(dut.la, arm=1)
            for _ in range(50):
                yield
            yield from control(dut.la, stop=1)
            yield
            self.assertEqual((yield from status(dut.la))["state"], LA_STATE_IDLE)
            yield from configure(dut.la, mode=LA_MODE_STREAM, length=0)
            yield from control(dut.la, arm=1)
            for _ in range(200):
                yield
            yield from configure(dut.la, length=30, pre=5,
                source=LA_TRIGGER_PATTERN, mask=0xf00, value=0x200)
            yield from control(dut.la, arm=1)
            for _ in range(1500):
                yield
        dram, reader = run(dut, scenario)
        # Aborted stream capture (truncated: the parser locks on the last capture header).
        words  = reader.words
        last   = max(i for i, w in enumerate(words) if w == LA_MAGIC)
        self.assertEqual(words.count(LA_MAGIC), 2)
        parser = LAStreamParser(sys_clk_freq=125e6)
        parser.feed(b"".join(w.to_bytes(4, "little") for w in words[last:]))
        capture = parser.capture
        self.assertIsNotNone(capture)
        self.assertEqual(len(capture.samples), 30)
        self.assertEqual(capture.trigger_offset, 5)
        self.assertEqual(capture.samples[5] & 0xf00, 0x200)
        check_timebase(self, capture, 1)

# Instrument ---------------------------------------------------------------------------------------

class InstrumentDUT(LiteXModule):
    def __init__(self):
        self.wport = LiteDRAMNativePort("write", 24, 64)
        self.rport = LiteDRAMNativePort("read",  24, 64)
        self.agentlab = AgentLab(125e6, uart_pads=[None]*4, n_gpios=2, n_power=1,
            la_inputs = "probes",
            la_kwargs = dict(write_port=self.wport, read_port=self.rport, ring_base=0,
                ring_size=4096, block_samples=16, idle_cycles=16, packet_period=512))

class TestAgentLabLAInstrument(unittest.TestCase):
    def test_probes_marker_trigger_event(self):
        # Marker (external trigger), LA trigger event in the events stream at the trigger
        # timestamp, probes: power switch 0 on channel 12, timestamp on channels 16-31.
        dut = InstrumentDUT()
        la  = dut.agentlab.la
        words = []
        @passive
        def events_reader():
            source = dut.agentlab.source
            yield source.ready.eq(1)
            while True:
                yield
                if (yield source.valid):
                    words.append((yield source.data))
        def scenario(dram, reader):
            yield from configure(la, source=LA_TRIGGER_EXTERNAL, ext=1 << LA_EXT_MARKER, pre=4,
                length=16, divider=2)
            yield from csr_set(dut.agentlab.power._out, 1)
            yield from control(la, arm=1)
            for _ in range(40):
                yield
            yield dut.agentlab._marker.storage.eq(0x55)
            yield dut.agentlab._marker.re.eq(1)
            yield
            yield dut.agentlab._marker.re.eq(0)
            yield from wait_parser(reader)
            for _ in range(200):
                yield
        dram   = DRAMModel(dut.wport, dut.rport)
        reader = Reader(la.source)
        run_simulation(dut, [scenario(dram, reader), dram.generator(), reader.generator(),
            events_reader()])
        capture = reader.parser.capture
        self.assertEqual((len(capture.samples), capture.trigger_offset), (16, 4))
        for k, sample in enumerate(capture.samples):
            self.assertEqual(sample >> 16, (capture.timestamp(k) - 2) & 0xffff)
            self.assertEqual((sample >> 12) & 1, 1)
        data    = b"".join(w.to_bytes(4, "little") for w in words)
        records = [(data[i], int.from_bytes(data[i + 2:i + 8], "little"),
            int.from_bytes(data[i + 8:i + 16], "little")) for i in range(0, len(data), 16)]
        marker  = [r for r in records if r[0] == EVENT_MARKER]
        trigger = [r for r in records if r[0] == EVENT_LA_TRIGGER]
        self.assertEqual(len(marker), 1)
        self.assertEqual(len(trigger), 1)
        self.assertEqual(trigger[0][1], capture.samples[4])
        self.assertEqual(trigger[0][2], capture.trigger_timestamp)
        self.assertTrue(0 <= trigger[0][2] - marker[0][2] <= 2)

if __name__ == "__main__":
    unittest.main()
