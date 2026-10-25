#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""`agentlab-mcp`: MCP server (stdio) exposing the AgentLab instrument as tools for LLM agents.

The tools are plain methods of `AgentLabTools` (testable without the `mcp` package); the official
`mcp` Python SDK (FastMCP) is only imported to serve them.
"""

import sys
import argparse
from typing import List, Optional

from agentlab.host.events import uart_lines

# Tools --------------------------------------------------------------------------------------------

class AgentLabTools:
    """AgentLab MCP tools (instrument opened lazily on first use)."""

    TOOLS = [
        "uart_read", "uart_write", "uart_wait_for", "uart_log",
        "power_set", "power_cycle",
        "gpio_set", "gpio_get",
        "events", "timestamp", "marker",
    ]

    def __init__(self, instrument_factory):
        self._factory    = instrument_factory
        self._instrument = None

    @property
    def instrument(self):
        if self._instrument is None:
            self._instrument = self._factory()
        return self._instrument

    def close(self):
        if self._instrument is not None:
            self._instrument.close()
            self._instrument = None

    def _now(self) -> dict:
        if self.instrument.csrs is None: # No CSR map: no device time.
            return {"timestamp": None, "time_s": None}
        ticks = self.instrument.timestamp()
        return {"timestamp": ticks, "time_s": self.instrument.ticks_to_s(ticks)}

    # DUT UARTs ------------------------------------------------------------------------------------

    def uart_read(self, uart: int, timeout_s: float = 1.0, max_bytes: Optional[int] = None) -> dict:
        """Read text received on a DUT serial console.

        Args:
            uart: DUT UART index (0-3).
            timeout_s: listening time in seconds (returns earlier if max_bytes are received).
            max_bytes: optional maximum number of bytes to return.
        Returns:
            {"text": received text (UTF-8, invalid bytes replaced), "bytes": byte count}.
            Text consumed here is not returned again by later reads.
        """
        data = self.instrument.uart(uart).read(timeout=timeout_s, size=max_bytes)
        return {"text": data.decode(errors="replace"), "bytes": len(data)}

    def uart_write(self, uart: int, text: str, newline: bool = False) -> dict:
        """Send text to a DUT serial console (e.g. a shell command).

        Args:
            uart: DUT UART index (0-3).
            text: text to send (UTF-8). Use "\\r" or "\\n" explicitly, or set newline.
            newline: append "\\r\\n" (Enter key) after the text.
        Returns:
            {"bytes": number of bytes written}.
        """
        data = text + ("\r\n" if newline else "")
        return {"bytes": self.instrument.uart(uart).write(data)}

    def uart_wait_for(self, uart: int, pattern: str, timeout_s: float = 10.0) -> dict:
        """Wait until a regex appears on a DUT serial console (boot prompt, log line, ...).

        Args:
            uart: DUT UART index (0-3).
            pattern: Python regular expression searched in the received text, e.g. "login:" or
                r"IP address: (\\S+)".
            timeout_s: maximum wait in seconds.
        Returns:
            {"matched": bool, "match": matched text or null, "groups": regex groups,
             "text": all text received up to the end of the match (or until timeout),
             "elapsed_s": wait time in seconds}. Text after the match is kept for the next
            uart_read/uart_wait_for.
        """
        return self.instrument.uart(uart).wait_for(pattern, timeout=timeout_s).to_dict()

    def uart_log(self, uart: Optional[int] = None, since_s: Optional[float] = None,
        until_s: Optional[float] = None, limit: int = 200) -> dict:
        """DUT console history as timestamped lines, rebuilt from the events stream.

        Args:
            uart: DUT UART index (0-3), or null for all UARTs.
            since_s / until_s: device-time window in seconds (see the timestamp tool).
            limit: maximum number of lines (most recent kept).
        Returns:
            {"lines": [{"time_s", "timestamp", "uart", "text"}], "now_s": current device time}.
        """
        i      = self.instrument
        events = i.events_between(since=since_s, until=until_s, types="uart_rx", source=uart)
        lines  = [l.to_dict(i.sys_clk_freq) for l in uart_lines(events)][-limit:]
        return {"lines": lines, "now_s": self._now()["time_s"]}

    # Power ----------------------------------------------------------------------------------------

    def power_set(self, index: int, on: bool) -> dict:
        """Switch a DUT power output on or off.

        Args:
            index: power switch index.
            on: true to power the DUT, false to cut power.
        Returns:
            {"index", "on", "time_s": device time after the change}.
        """
        self.instrument.power_set(index, on)
        return {"index": index, "on": on, **self._now()}

    def power_cycle(self, index: int, off_time_s: float = 1.0) -> dict:
        """Power cycle the DUT: switch power off, wait off_time_s seconds, switch it back on.

        Args:
            index: power switch index.
            off_time_s: off duration in seconds.
        Returns:
            {"index", "off_time_s", "time_s": device time when power was restored}.
            Use uart_wait_for afterwards to wait for the DUT boot.
        """
        self.instrument.power_cycle(index, off_time=off_time_s)
        return {"index": index, "off_time_s": off_time_s, **self._now()}

    # GPIOs ----------------------------------------------------------------------------------------

    def gpio_set(self, index: int, value: Optional[int]) -> dict:
        """Drive a DUT GPIO (reset/boot pins, buttons, ...) or release it.

        Args:
            index: GPIO index.
            value: 1 (high), 0 (low) or null to release the pin (input, high impedance).
        Returns:
            {"index", "value"}.
        """
        if value is None:
            self.instrument.gpio_release(index)
        else:
            self.instrument.gpio_set(index, bool(value))
        return {"index": index, "value": value}

    def gpio_get(self, index: int) -> dict:
        """Read the level of a DUT GPIO.

        Args:
            index: GPIO index.
        Returns:
            {"index", "value": 0 or 1}.
        """
        return {"index": index, "value": int(self.instrument.gpio_get(index))}

    # Events / Timebase ----------------------------------------------------------------------------

    def events(self, since_s: Optional[float] = None, until_s: Optional[float] = None,
        types: Optional[List[str]] = None, source: Optional[int] = None,
        limit: int = 500) -> dict:
        """Timestamped hardware events (UART bytes, GPIO edges, power changes, markers).

        All events share the device timebase (seconds since FPGA start, ns resolution).
        Args:
            since_s / until_s: device-time window in seconds (from timestamp/marker results).
            types: subset of "uart_rx", "uart_tx", "gpio", "power", "la_trigger", "marker",
                "overflow" (null: all).
            source: UART/GPIO/power index filter (null: all).
            limit: maximum number of events (most recent kept).
        Returns:
            {"events": [{"time_s", "timestamp", "type", "source", "value", "char"?}],
             "truncated": bool, "now_s": current device time,
             "error": events reader error (device unplugged, ...) or null}.
        """
        i      = self.instrument
        events = i.events_between(since=since_s, until=until_s, types=types, source=source)
        return {
            "events"    : [e.to_dict(i.sys_clk_freq) for e in events[-limit:]],
            "truncated" : len(events) > limit,
            "now_s"     : self._now()["time_s"],
            "error"     : None if i.events.error is None else str(i.events.error),
        }

    def timestamp(self) -> dict:
        """Current device time.

        Returns:
            {"timestamp": sys_clk ticks, "time_s": seconds}. Use it as since_s for events.
        """
        return self._now()

    def marker(self, value: int = 0) -> dict:
        """Insert a marker event in the events stream (to tag an agent action in time).

        Args:
            value: marker value (up to 32 bits) to recognize it in the events.
        Returns:
            {"value", "timestamp", "time_s"}: device time just after the marker.
        """
        ticks = self.instrument.marker(value)
        return {"value": value, "timestamp": ticks, "time_s": self.instrument.ticks_to_s(ticks)}

# Server -------------------------------------------------------------------------------------------

def build_server(tools: AgentLabTools, name="agentlab"):
    """Build an MCP server with the AgentLab tools (needs the `mcp` package, 2.x or 1.x)."""
    try:
        from mcp.server.mcpserver import MCPServer as Server # mcp >= 2.
    except ImportError:
        try:
            from mcp.server.fastmcp import FastMCP as Server # mcp 1.x.
        except ImportError as e:
            raise RuntimeError("agentlab-mcp needs the mcp package: pip install mcp") from e
    server = Server(name)
    for tool in tools.TOOLS:
        fn = getattr(tools, tool)
        server.add_tool(fn, name=tool, description=fn.__doc__)
    return server

def main(argv=None):
    from agentlab.host.cli        import add_connection_args
    from agentlab.host.instrument import Instrument
    parser = argparse.ArgumentParser(prog="agentlab-mcp",
        description="AgentLab MCP server (stdio).")
    add_connection_args(parser)
    args  = parser.parse_args(argv)
    tools = AgentLabTools(lambda: Instrument.open(csr_csv=args.csr_csv, serial=args.serial,
        control_port=args.control_port, server=args.server, sys_clk_freq=args.sys_clk_freq,
        start_events=True))
    try:
        server = build_server(tools)
    except RuntimeError as e:
        print("agentlab-mcp: {}".format(e), file=sys.stderr)
        return 2
    try:
        server.run()
    finally:
        tools.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
