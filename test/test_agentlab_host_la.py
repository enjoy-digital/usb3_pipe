#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab logic analyzer host software: configuration parsing, stream format (round trip,
gateware constants), VCD/sigrok writers (parsed back), summaries, Instrument/CLI/MCP captures on
fake backends."""

import io
import os
import json
import array
import random
import unittest
import tempfile
import contextlib

from agentlab.host import constants as c
from agentlab.host.la         import *
from agentlab.host.cli        import main
from agentlab.host.mcp_server import AgentLabTools

from test.agentlab_host_fake import *

# Helpers ------------------------------------------------------------------------------------------

def make_capture(samples, divider=2, start=1000, pre=3, channels=8, block=16, overflow=False):
    header = LAHeader(version=c.LA_VERSION, channels=channels, mode=0, flags=0, divider=divider,
        block_samples=block, trigger_offset=pre, count=len(samples), start_timestamp=start,
        trigger_timestamp=start + pre*divider)
    return header, pack_stream(header, samples, overflow=overflow)

# Configuration ------------------------------------------------------------------------------------

class TestLAConfig(unittest.TestCase):
    def test_numbers(self):
        self.assertEqual(parse_number("25M"), 25e6)
        self.assertEqual(parse_number("2.5k"), 2500)
        self.assertEqual(parse_number("10MHz"), 10e6)
        self.assertEqual(parse_samples("4k", 1e6), 4000)
        self.assertEqual(parse_samples("10ms", 1e6), 10000)
        self.assertEqual(parse_samples("2us", 125e6), 250)
        self.assertEqual(parse_channels("0-3,7"), [0, 1, 2, 3, 7])
        self.assertEqual(parse_channels(None, 4), [0, 1, 2, 3])
        with self.assertRaises(ValueError):
            parse_channels("40")

    def test_trigger(self):
        t = parse_trigger("immediate")
        self.assertEqual(t.source, c.LA_TRIGGER_IMMEDIATE)
        t = parse_trigger("ch3=1,ch4=0,falling:2")
        self.assertEqual((t.source, t.mask, t.value), (c.LA_TRIGGER_PATTERN, 0b11000, 0b01000))
        self.assertEqual(t.edge_csr, 1 | (2 << 1) | (c.LA_EDGES["falling"] << 6))
        t = parse_trigger("mask=0xf0,value=0x50")
        self.assertEqual((t.mask, t.value), (0xf0, 0x50))
        t = parse_trigger("ext:marker+gpio")
        self.assertEqual((t.source, t.ext), (c.LA_TRIGGER_EXTERNAL, 0b011))
        self.assertEqual(parse_trigger("power").ext, 0b100)
        for bad in ["ch3=2", "foo", "marker,ch1=1"]:
            with self.assertRaises(ValueError):
                parse_trigger(bad)

    def test_gateware_constants(self):
        # Host constants match the gateware (stream format, CSR fields).
        from agentlab.gateware import la
        self.assertEqual(c.LA_MAGIC,           la.LA_MAGIC)
        self.assertEqual(c.LA_VERSION,         la.LA_VERSION)
        self.assertEqual(c.LA_HEADER_WORDS,    la.LA_HEADER_WORDS)
        self.assertEqual(c.LA_BLOCK_MAGIC,     la.LA_BLOCK_MAGIC)
        self.assertEqual(c.LA_BLOCK_LAST,      la.LA_BLOCK_LAST)
        self.assertEqual(c.LA_BLOCK_OVERFLOW,  la.LA_BLOCK_OVERFLOW)
        self.assertEqual(c.LA_HEADER_OVERFLOW, la.LA_HEADER_OVERFLOW)
        self.assertEqual(c.LA_MODES, {"capture": la.LA_MODE_CAPTURE, "stream": la.LA_MODE_STREAM})
        self.assertEqual(c.LA_EDGES, {"rising": la.LA_EDGE_RISING,
            "falling": la.LA_EDGE_FALLING, "edge": la.LA_EDGE_ANY})
        self.assertEqual(c.LA_EXT_SOURCES, {"marker": la.LA_EXT_MARKER, "gpio": la.LA_EXT_GPIO,
            "power": la.LA_EXT_POWER})
        self.assertEqual((c.LA_TRIGGER_IMMEDIATE, c.LA_TRIGGER_PATTERN, c.LA_TRIGGER_EXTERNAL),
            (la.LA_TRIGGER_IMMEDIATE, la.LA_TRIGGER_PATTERN, la.LA_TRIGGER_EXTERNAL))
        self.assertEqual(c.LA_STATES[la.LA_STATE_DONE], "done")
        self.assertEqual(c.LA_STATES[la.LA_STATE_ARMED], "armed")

# Stream Format ------------------------------------------------------------------------------------

class TestLAStream(unittest.TestCase):
    def test_round_trip(self):
        rng = random.Random(0)
        for n in [0, 1, 15, 16, 17, 100]:
            samples = [rng.getrandbits(32) for _ in range(n)]
            header, data = make_capture(samples)
            self.assertEqual(len(data), 48 + 4*n + 4*max(1, -(-n//16))) # Header, samples, blocks.
            # Arbitrary splits, stale data before the header.
            parser = LAStreamParser()
            stream = b"\x01\x02\x03\x04" + data + b"\xff"*8
            i = 0
            while i < len(stream) and not parser.done:
                j = i + rng.randint(1, 50)
                parser.feed(stream[i:j])
                i = j
            capture = parser.capture
            self.assertEqual(parser.skipped, 4)
            self.assertEqual(list(capture.samples), samples)
            self.assertEqual(capture.header, header)
            self.assertFalse(capture.overflow)

    def test_overflow_timing(self):
        header, data = make_capture(list(range(40)), divider=5, start=10**12, pre=7,
            overflow=True)
        capture = parse_stream(data, sys_clk_freq=125e6)
        self.assertTrue(capture.overflow)
        self.assertEqual(capture.samplerate, 25e6)
        self.assertEqual(capture.timestamp(7), capture.trigger_timestamp)
        self.assertAlmostEqual(capture.time(1) - capture.time(0), 40e-9)
        with self.assertRaises(ValueError):
            parse_stream(data[:-4])
        bad = bytearray(data)
        bad[48 + 3] = 0x00 # Block magic.
        with self.assertRaises(ValueError):
            parse_stream(bytes(bad))

# Writers ------------------------------------------------------------------------------------------

def counter_capture(n=64, divider=4, start=1000, channels=4):
    _, data = make_capture([k & ((1 << channels) - 1) for k in range(n)], divider=divider,
        start=start, channels=channels)
    return parse_stream(data, sys_clk_freq=125e6)

class TestLAWriters(unittest.TestCase):
    def test_vcd(self):
        capture = counter_capture()
        f = io.StringIO()
        write_vcd(capture, f, channels=[0, 2], names=["clk", "a", "b", "c"])
        f.seek(0)
        vcd = read_vcd(f)
        self.assertEqual(vcd["timescale"], "1ns")
        self.assertEqual(set(vcd["vars"]), {"clk", "b"})
        # Times: device ns (start_timestamp 1000 ticks = 8000ns, 4 ticks/sample = 32ns).
        clk = vcd["vars"]["clk"]
        self.assertEqual(clk[0], (8000, 0))
        self.assertEqual(clk[1], (8032, 1))
        self.assertEqual(len(clk), 64)
        b = vcd["vars"]["b"]
        self.assertEqual([t for t, v in b][:3], [8000, 8000 + 4*32, 8000 + 8*32])
        self.assertEqual(vcd["end"], 8000 + 64*32)

    def test_sr(self):
        capture = counter_capture(n=100, channels=4)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "c.sr")
            write_sr(capture, path, chunk_samples=32)
            sr = read_sr(path)
            self.assertEqual(sr["version"], "2")
            self.assertEqual(sr["samplerate"], 125e6/4)
            self.assertEqual(sr["probes"], ["ch0", "ch1", "ch2", "ch3"])
            self.assertEqual(sr["unitsize"], 1)
            self.assertEqual(sr["samples"], list(capture.samples))
            self.assertEqual(sr["agentlab"]["start_timestamp"], 1000)
            # Channels subset: packed.
            write_sr(capture, path, channels=[1, 3], names=["a", "b", "c", "d"])
            sr = read_sr(path)
            self.assertEqual(sr["probes"], ["b", "d"])
            self.assertEqual(sr["samples"], [((k >> 1) & 1) | (((k >> 3) & 1) << 1)
                for k in range(100)])
            # 32 channels: unitsize 4.
            capture = parse_stream(make_capture([0xdeadbeef, 0x12345678], channels=32)[1])
            write_sr(capture, path)
            sr = read_sr(path)
            self.assertEqual((sr["unitsize"], sr["samples"]), (4, [0xdeadbeef, 0x12345678]))

    def test_summary(self):
        capture = counter_capture(n=64, divider=1, channels=4)
        s = summarize(capture, [0, 3])
        self.assertEqual(s["samples"], 64)
        ch0 = s["channels"]["ch0"]
        self.assertEqual((ch0["initial"], ch0["final"], ch0["edges"]), (0, 1, 63))
        self.assertEqual((ch0["rising"], ch0["falling"]), (32, 31))
        self.assertAlmostEqual(ch0["frequency_hz"], 125e6/2)
        self.assertAlmostEqual(ch0["duty"], 0.5)
        self.assertAlmostEqual(ch0["first_edges"][0]["time_s"], 1001/125e6)
        self.assertAlmostEqual(s["channels"]["ch3"]["frequency_hz"], 125e6/16)
        self.assertNotIn("ch1", s["channels"])

# Instrument / CLI / MCP ---------------------------------------------------------------------------

def csr_writes(bus, name):
    return [v for t, n, v in bus.log if n == name]

class TestLAInstrument(unittest.TestCase):
    def test_capture(self):
        i, f = make_instrument()
        capture = i.la_capture(rate="25M", trigger="ch1=1,rising:0", pre=10, length="2us")
        bus = f["bus"]
        self.assertEqual(csr_writes(bus, c.CSR_LA_DIVIDER), [5])
        self.assertEqual(csr_writes(bus, c.CSR_LA_LENGTH), [50])
        self.assertEqual(csr_writes(bus, c.CSR_LA_PRE_TRIGGER), [10])
        self.assertEqual(csr_writes(bus, c.CSR_LA_TRIGGER_SOURCE), [c.LA_TRIGGER_PATTERN])
        self.assertEqual(csr_writes(bus, c.CSR_LA_TRIGGER_MASK), [0b10])
        self.assertEqual(csr_writes(bus, c.CSR_LA_TRIGGER_EDGE), [1])
        self.assertEqual(csr_writes(bus, c.CSR_LA_CONTROL), [1])
        self.assertEqual(len(capture.samples), 50)
        self.assertEqual(capture.trigger_offset, 10)
        self.assertEqual(i.la_status()["state"], "done")
        # Validation.
        for kwargs in [dict(pre=10, length=10), dict(length=2**21), dict(length=0),
            dict(mode="foo")]:
            with self.assertRaises(ValueError):
                i.la_capture(**kwargs)

    def test_timeout(self):
        i, f = make_instrument()
        f["la"].trigger = False
        with self.assertRaises(TimeoutError):
            i.la_capture(trigger="ch0=1", timeout=1.0)
        self.assertEqual(f["la"].stops, 1)

    def test_cli(self):
        i, f = make_instrument()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "x.vcd")
            out  = io.StringIO()
            with contextlib.redirect_stdout(out):
                r = main(["la", "capture", "--channels", "0-1", "--names", "clk,data",
                    "--length", "100", "--out", path], instrument_factory=lambda args: i)
            self.assertEqual(r, 0)
            s = json.loads(out.getvalue())
            self.assertEqual(s["file"], path)
            self.assertEqual(set(s["channels"]), {"clk", "data"})
            self.assertEqual(s["samples"], 100)
            self.assertEqual(set(read_vcd(path)["vars"]), {"clk", "data"})
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            main(["la", "status"], instrument_factory=lambda args: i)
        self.assertTrue(json.loads(out.getvalue())["dram_ready"])
        with contextlib.redirect_stderr(io.StringIO()):
            r = main(["la", "capture", "--pre", "10", "--length", "5"],
                instrument_factory=lambda args: i)
        self.assertEqual(r, 2)

    def test_mcp(self):
        i, f = make_instrument()
        tools = AgentLabTools(lambda: i)
        self.assertIn("la_capture", tools.TOOLS)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "x.sr")
            s = tools.la_capture(channels="0-3", rate_hz=1e6, length="1ms", out=path)
            self.assertEqual(s["samples"], 1000)
            self.assertEqual(s["samplerate_hz"], 1e6)
            self.assertEqual(len(read_sr(path)["samples"]), 1000)
            self.assertEqual(s["channels"]["ch1"]["edges"], 499) # Counter: toggles every 2.
        s = tools.la_capture(length="100")
        self.assertTrue(s["file"].endswith(".vcd") and os.path.exists(s["file"]))
        os.remove(s["file"])

if __name__ == "__main__":
    unittest.main()
