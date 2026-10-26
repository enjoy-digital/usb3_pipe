#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""`agentlab` command line interface."""

import os
import sys
import json
import time
import argparse

from agentlab.host.events import filter_events, format_events, uart_lines
from agentlab.host.eyes   import GRAB_BACKENDS

# Arguments ----------------------------------------------------------------------------------------

def add_connection_args(parser):
    parser.add_argument("--csr-csv",      default=os.environ.get("AGENTLAB_CSR_CSV"),
        help="Gateware csr.csv/csr.json (default: $AGENTLAB_CSR_CSV).")
    parser.add_argument("--serial",       default=None, help="Device USB serial number.")
    parser.add_argument("--control-port", default=None, help="Control tty override.")
    parser.add_argument("--server",       default=None, help="litex_server host:port.")
    parser.add_argument("--sys-clk-freq", default=None, type=float, help="Timebase override (Hz).")

def build_parser():
    parser = argparse.ArgumentParser(prog="agentlab", description="AgentLab instrument CLI.")
    add_connection_args(parser)
    sub = parser.add_subparsers(dest="cmd", required=True)

    # Info.
    sub.add_parser("info", help="Device and CSR information.")

    # UART.
    uart     = sub.add_parser("uart", help="DUT UARTs.")
    uart_sub = uart.add_subparsers(dest="action", required=True)
    for name in ["read", "write", "wait"]:
        p = uart_sub.add_parser(name)
        p.add_argument("index",      type=int, help="DUT UART index.")
        p.add_argument("--baudrate", type=int, default=None)
        if name == "read":
            p.add_argument("--timeout", type=float, default=1.0, help="Seconds.")
        if name == "write":
            p.add_argument("data", help="Text (escapes such as \\n and \\r are interpreted).")
        if name == "wait":
            p.add_argument("pattern",   help="Python regex.")
            p.add_argument("--timeout", type=float, default=10.0, help="Seconds.")

    # Power.
    power     = sub.add_parser("power", help="Power switches.")
    power_sub = power.add_subparsers(dest="action", required=True)
    for name in ["on", "off", "cycle", "get"]:
        p = power_sub.add_parser(name)
        p.add_argument("index", type=int)
        if name == "cycle":
            p.add_argument("--off-time", type=float, default=1.0, help="Seconds.")

    # GPIO.
    gpio     = sub.add_parser("gpio", help="GPIOs.")
    gpio_sub = gpio.add_subparsers(dest="action", required=True)
    p = gpio_sub.add_parser("set")
    p.add_argument("index", type=int)
    p.add_argument("value", type=int, choices=[0, 1])
    p = gpio_sub.add_parser("get")
    p.add_argument("index", type=int)
    p = gpio_sub.add_parser("release")
    p.add_argument("index", type=int)

    # Events.
    events = sub.add_parser("events", help="Events stream.")
    events.add_argument("--follow",   action="store_true", help="Print events until Ctrl-C.")
    events.add_argument("--duration", type=float, default=1.0, help="Capture time (seconds).")
    events.add_argument("--types",    default=None, help="Comma separated (uart_rx,gpio,...).")
    events.add_argument("--source",   type=int, default=None)
    events.add_argument("--lines",    action="store_true", help="Rebuild UART RX text lines.")

    # Timestamp / Marker.
    sub.add_parser("timestamp", help="Read the device timestamp.")
    marker = sub.add_parser("marker", help="Emit a marker event.")
    marker.add_argument("value", type=lambda x: int(x, 0), nargs="?", default=0)

    # Logic Analyzer.
    la     = sub.add_parser("la", help="Logic analyzer.")
    la_sub = la.add_subparsers(dest="action", required=True)
    p = la_sub.add_parser("capture", help="Capture (prints a JSON summary).")
    p.add_argument("--channels", default=None,        help="Channels saved/summarized (ex: 0-7,12; default: all).")
    p.add_argument("--names",    default=None,        help="Channel names (comma separated, one per channel).")
    p.add_argument("--rate",     default=None,        help="Samplerate in Hz (k/M suffixes; default: sys_clk).")
    p.add_argument("--trigger",  default="immediate", help="immediate, chN=0|1, rising:N, falling:N, edge:N (comma separated), marker, gpio, power.")
    p.add_argument("--pre",      default="0",         help="Pre-trigger samples (or duration: 10us).")
    p.add_argument("--length",   default="4096",      help="Samples (or duration: 10ms), pre-trigger included; stream mode: 0, until --duration.")
    p.add_argument("--mode",     default="capture",   choices=["capture", "stream"])
    p.add_argument("--duration", type=float, default=None, help="Stream mode with length 0: stop after (seconds).")
    p.add_argument("--timeout",  type=float, default=10.0, help="Trigger/capture timeout (seconds).")
    p.add_argument("--out",      default=None,        help="Output file: .vcd, .sr (sigrok/PulseView) or .json.")
    la_sub.add_parser("status", help="Logic analyzer status.")

    # Eyes.
    shot = sub.add_parser("screenshot", help="Grab one Eyes frame (UVC) to a PNG file.")
    shot.add_argument("path", help="Output PNG file.")
    shot.add_argument("--refresh", action="store_true", help="Redraw the status panel first.")
    shot.add_argument("--backend", default="auto", choices=["auto"] + GRAB_BACKENDS)
    shot.add_argument("--video",   default=None, help="UVC node override (/dev/videoN).")
    display     = sub.add_parser("display", help="Eyes display (text grid, status, waveforms).")
    display_sub = display.add_subparsers(dest="action", required=True)
    p = display_sub.add_parser("status", help="Draw the status panel.")
    p.add_argument("--follow", action="store_true", help="Refresh until Ctrl-C.")
    p.add_argument("--period", type=float, default=1.0, help="Refresh period (seconds).")
    p = display_sub.add_parser("text", help="Write text (\\n: next row).")
    p.add_argument("text")
    p.add_argument("--col",   type=int, default=0)
    p.add_argument("--row",   type=int, default=0)
    p.add_argument("--fg",    default="white", help="Color name or index (0-15).")
    p.add_argument("--bg",    default="black", help="Color name or index (0-15).")
    p.add_argument("--clear", action="store_true", help="Clear the screen first.")
    display_sub.add_parser("clear", help="Clear the screen.")
    p = display_sub.add_parser("waves", help="Waveform lanes configuration.")
    p.add_argument("--lanes",     type=int,   default=None, help="Number of lanes (0: off).")
    p.add_argument("--first",     type=int,   default=None, help="Sample bit of the first lane.")
    p.add_argument("--period-us", type=float, default=None, help="Sample period (us).")
    p.add_argument("--run",       action=argparse.BooleanOptionalAction, default=None,
        help="Continuous sampling (roll mode).")
    p.add_argument("--single",    action="store_true", help="Capture one window.")

    return parser

# Commands -----------------------------------------------------------------------------------------

def _unescape(s):
    return s.encode().decode("unicode_escape").encode("latin-1")

def run(args, instrument, out=None):
    out = out or sys.stdout
    p   = lambda *a: print(*a, file=out)
    if args.cmd == "info":
        p(json.dumps(instrument.info(), indent=2))
    elif args.cmd == "uart":
        if args.baudrate is not None:
            instrument.uart_set_baudrate(args.index, args.baudrate)
        uart = instrument.uart(args.index)
        if args.action == "read":
            out.write(uart.read(timeout=args.timeout).decode(errors="replace"))
        elif args.action == "write":
            uart.write(_unescape(args.data))
        elif args.action == "wait":
            r = uart.wait_for(args.pattern, timeout=args.timeout)
            out.write(r.text)
            if not r.matched:
                p("\n[timeout after {:.3f}s]".format(r.elapsed))
                return 1
    elif args.cmd == "power":
        if args.action == "on":
            instrument.power_set(args.index, True)
        elif args.action == "off":
            instrument.power_set(args.index, False)
        elif args.action == "cycle":
            instrument.power_cycle(args.index, off_time=args.off_time)
        elif args.action == "get":
            p(int(instrument.power_get(args.index)))
    elif args.cmd == "gpio":
        if args.action == "set":
            instrument.gpio_set(args.index, bool(args.value))
        elif args.action == "get":
            p(int(instrument.gpio_get(args.index)))
        elif args.action == "release":
            instrument.gpio_release(args.index)
    elif args.cmd == "events":
        return run_events(args, instrument, p)
    elif args.cmd == "timestamp":
        t = instrument.timestamp()
        p("{} ticks ({:.9f}s)".format(t, instrument.ticks_to_s(t)))
    elif args.cmd == "la":
        if args.action == "status":
            p(json.dumps(instrument.la_status(), indent=2))
        elif args.action == "capture":
            summary = instrument.la_capture_summary(
                channels = args.channels,
                names    = args.names,
                out      = args.out,
                rate     = args.rate,
                trigger  = args.trigger,
                pre      = args.pre,
                length   = args.length,
                mode     = args.mode,
                duration = args.duration,
                timeout  = args.timeout)
            p(json.dumps(summary, indent=2))
    elif args.cmd == "marker":
        t = instrument.marker(args.value)
        p("{} ticks ({:.9f}s)".format(t, instrument.ticks_to_s(t)))
    elif args.cmd == "screenshot":
        r = instrument.screenshot(args.path, refresh=args.refresh, backend=args.backend,
            video_device=args.video)
        p("{path}: {width}x{height}, {bytes} bytes (from {device})".format(**r))
    elif args.cmd == "display":
        return run_display(args, instrument, p)
    return 0

def _color(s):
    return int(s) if s.isdigit() else s

def run_display(args, instrument, p):
    if args.action == "status":
        instrument.display_status()
        if args.follow:
            try:
                while True:
                    time.sleep(args.period)
                    instrument.display_status()
            except KeyboardInterrupt:
                pass
    elif args.action == "text":
        instrument.display_text(args.text.replace("\\n", "\n"), col=args.col,
            row=args.row, fg=_color(args.fg), bg=_color(args.bg), clear=args.clear)
    elif args.action == "clear":
        instrument.display_clear()
    elif args.action == "waves":
        period = None if args.period_us is None else args.period_us*1e-6
        instrument.display_waves(lanes=args.lanes, first=args.first, period_s=period,
            run=args.run, single=args.single)
    return 0

def run_events(args, instrument, p):
    types  = None if args.types is None else args.types.split(",")
    reader = instrument.events
    seen   = 0
    def dump():
        nonlocal seen
        events = reader.snapshot()
        new, seen = events[seen:], len(events)
        new = filter_events(new, types=types, source=args.source)
        if args.lines:
            for l in uart_lines(new):
                p("{:14.9f}s uart{} {}".format(instrument.ticks_to_s(l.timestamp), l.source,
                    l.text))
        else:
            for line in format_events(new, instrument.sys_clk_freq):
                p(line)
    try:
        if args.follow:
            while True:
                time.sleep(0.1)
                dump()
        else:
            time.sleep(args.duration)
            dump()
    except KeyboardInterrupt:
        pass
    return 0

# Main ---------------------------------------------------------------------------------------------

def open_instrument(args):
    from agentlab.host.instrument import Instrument
    return Instrument.open(csr_csv=args.csr_csv, serial=args.serial,
        control_port=args.control_port, server=args.server, sys_clk_freq=args.sys_clk_freq)

def main(argv=None, instrument_factory=open_instrument):
    args = build_parser().parse_args(argv)
    try:
        instrument = instrument_factory(args)
    except (RuntimeError, OSError) as e:
        print("agentlab: {}".format(e), file=sys.stderr)
        return 2
    try:
        return run(args, instrument)
    except (RuntimeError, OSError, KeyError, ValueError, TimeoutError) as e:
        print("agentlab: {}".format(e), file=sys.stderr)
        return 2
    finally:
        instrument.close()

if __name__ == "__main__":
    sys.exit(main())
