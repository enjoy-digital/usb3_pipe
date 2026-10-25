#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab CLI and MCP tools tests (fake backends, no hardware)."""

import io
import json
import asyncio
import unittest
import contextlib

from agentlab.host import constants as c
from agentlab.host.cli        import build_parser, main, run
from agentlab.host.events     import Event
from agentlab.host.mcp_server import AgentLabTools, build_server

from test.agentlab_host_fake import *

def run_cli(argv, instrument):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        r = main(argv, instrument_factory=lambda args: instrument)
    return r, out.getvalue()

# CLI ----------------------------------------------------------------------------------------------

class TestCLI(unittest.TestCase):
    def test_parser(self):
        p = build_parser()
        a = p.parse_args(["--csr-csv", "x.csv", "power", "cycle", "1", "--off-time", "2.5"])
        self.assertEqual((a.csr_csv, a.cmd, a.action, a.index, a.off_time),
            ("x.csv", "power", "cycle", 1, 2.5))
        a = p.parse_args(["uart", "wait", "0", "login:", "--timeout", "30"])
        self.assertEqual((a.action, a.pattern, a.timeout), ("wait", "login:", 30.0))
        a = p.parse_args(["events", "--follow", "--types", "uart_rx,gpio"])
        self.assertTrue(a.follow)
        a = p.parse_args(["marker", "0x10"])
        self.assertEqual(a.value, 16)
        for bad in [[], ["gpio", "set", "1", "2"], ["power", "toggle", "0"], ["uart", "read"]]:
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                p.parse_args(bad)

    def test_power_gpio(self):
        i, f = make_instrument()
        self.assertEqual(run_cli(["power", "on", "1"], i)[0], 0)
        self.assertEqual(run_cli(["power", "get", "1"], i), (0, "1\n"))
        run_cli(["power", "cycle", "1", "--off-time", "0.2"], i)
        writes = [(t, v) for t, n, v in f["bus"].log if n == c.CSR_POWER_OUT]
        self.assertEqual(writes[-2:], [(0.0, 0), (0.2, 0b10)])
        run_cli(["gpio", "set", "2", "0"], i)
        self.assertEqual(run_cli(["gpio", "get", "2"], i), (0, "0\n"))

    def test_uart(self):
        i, f = make_instrument(uart_chunks={1: [(0.1, b"hello\r\n# ")]})
        r, out = run_cli(["uart", "wait", "1", "# ", "--baudrate", "57600"], i)
        self.assertEqual((r, out), (0, "hello\r\n# "))
        self.assertEqual(f["ports"][1].baudrate, 57600)
        run_cli(["uart", "write", "1", "reboot\\r"], i)
        self.assertEqual(f["ports"][1].tx, b"reboot\r")
        r, out = run_cli(["uart", "wait", "1", "never", "--timeout", "0.5"], i)
        self.assertEqual(r, 1)
        self.assertIn("timeout", out)

    def test_timestamp_marker_info(self):
        i, f = make_instrument()
        f["clock"].now = 1.0
        r, out = run_cli(["marker", "7"], i)
        self.assertIn("125000000 ticks", out)
        r, out = run_cli(["info"], i)
        self.assertIn(c.CSR_POWER_OUT, json.loads(out)["csrs"])

    def test_events(self):
        i, f = make_instrument(events=uart_events("ok\n", source=2) +
            [Event(c.EVENT_GPIO_EDGE, 1, 1, 9000)])
        r, out = run_cli(["events", "--duration", "0.2", "--types", "gpio"], i)
        self.assertEqual(len(out.splitlines()), 1)
        self.assertIn("gpio", out)
        i, f = make_instrument(events=uart_events("ok\n", source=2))
        r, out = run_cli(["events", "--duration", "0.2", "--lines"], i)
        self.assertIn("uart2 ok", out)

    def test_errors(self):
        from agentlab.host.instrument import Instrument
        r, out = run_cli(["power", "on", "0"], Instrument())
        self.assertEqual(r, 2)
        def fail(args):
            raise RuntimeError("No AgentLab device found")
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(main(["info"], instrument_factory=fail), 2)
        self.assertIn("No AgentLab", err.getvalue())

# MCP Tools ----------------------------------------------------------------------------------------

class TestMCPTools(unittest.TestCase):
    def make(self, **kwargs):
        i, f = make_instrument(**kwargs)
        return AgentLabTools(lambda: i), i, f

    def test_docstrings(self):
        for name in AgentLabTools.TOOLS:
            doc = getattr(AgentLabTools, name).__doc__
            self.assertTrue(doc and len(doc.strip()) > 40, name)

    def test_uart_tools(self):
        tools, i, f = self.make(uart_chunks={0: [(0.2, b"Linux login: ")]})
        r = tools.uart_wait_for(0, r"(\w+) login:", timeout_s=5)
        self.assertTrue(r["matched"])
        self.assertEqual(r["groups"], ["Linux"])
        json.dumps(r)
        self.assertEqual(tools.uart_write(0, "root", newline=True), {"bytes": 6})
        self.assertEqual(f["ports"][0].tx, b"root\r\n")
        f["ports"][0].feed(b"# ")
        # " " after the match kept for the next read.
        self.assertEqual(tools.uart_read(0, timeout_s=0.2), {"text": " # ", "bytes": 3})

    def test_power_gpio_tools(self):
        tools, i, f = self.make()
        f["clock"].now = 1.0
        r = tools.power_cycle(0, off_time_s=2.0)
        self.assertEqual((r["index"], r["off_time_s"]), (0, 2.0))
        self.assertAlmostEqual(r["time_s"], 3.0)
        self.assertTrue(tools.power_set(1, False)["time_s"] > 0)
        self.assertEqual(tools.gpio_set(4, 0), {"index": 4, "value": 0})
        self.assertEqual(tools.gpio_get(4)["value"], 0)
        tools.gpio_set(4, None)
        self.assertEqual(tools.gpio_get(4)["value"], 1)

    def test_events_tools(self):
        t0     = int(1.0*SYS_CLK_FREQ)
        events = uart_events("a\nb\n", source=0, t0=t0) + [Event(c.EVENT_MARKER, 0, 5, 2*t0)]
        tools, i, f = self.make(events=events)
        try:
            i.events.wait_for(lambda e: e.type == c.EVENT_MARKER, timeout=2.0)
            r = tools.events(types=["marker"])
            self.assertEqual(len(r["events"]), 1)
            self.assertAlmostEqual(r["events"][0]["time_s"], 2.0)
            self.assertIsNone(r["error"])
            r = tools.events(since_s=0.5, until_s=1.5, limit=2)
            self.assertTrue(r["truncated"])
            self.assertEqual(len(r["events"]), 2)
            json.dumps(r)
            r = tools.uart_log(uart=0)
            self.assertEqual([l["text"] for l in r["lines"]], ["a", "b"])
        finally:
            tools.close()

    def test_timestamp_marker_tools(self):
        tools, i, f = self.make()
        f["clock"].now = 0.5
        self.assertAlmostEqual(tools.timestamp()["time_s"], 0.5)
        r = tools.marker(3)
        self.assertEqual(r["value"], 3)
        self.assertIn((0.5, c.CSR_MARKER, 3), f["bus"].log)

    def test_build_server(self):
        tools, i, f = self.make()
        try:
            server = build_server(tools)
        except RuntimeError:
            self.skipTest("mcp package not installed")
        listed = asyncio.run(server.list_tools())
        self.assertEqual(sorted(t.name for t in listed), sorted(AgentLabTools.TOOLS))
        tool   = {t.name: t for t in listed}["uart_wait_for"]
        schema = getattr(tool, "input_schema", None) or tool.inputSchema # mcp 2.x / 1.x.
        self.assertEqual(sorted(schema["required"]), ["pattern", "uart"])

if __name__ == "__main__":
    unittest.main()
