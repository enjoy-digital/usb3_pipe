#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab events: 16-byte record parsing, background reader with ring buffer, correlation."""

import time
import struct
import threading
import collections
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence

from agentlab.host import constants as c

# Event --------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Event:
    type      : int
    source    : int
    value     : int
    timestamp : int # sys_clk ticks.

    @property
    def name(self) -> str:
        return c.EVENT_NAMES.get(self.type, "unknown_0x{:02x}".format(self.type))

    def time(self, sys_clk_freq) -> float:
        return self.timestamp/sys_clk_freq

    def to_dict(self, sys_clk_freq=None) -> dict:
        d = {
            "type"      : self.name,
            "source"    : self.source,
            "value"     : self.value,
            "timestamp" : self.timestamp,
        }
        if sys_clk_freq:
            d["time_s"] = self.time(sys_clk_freq)
        if self.type in [c.EVENT_UART_RX, c.EVENT_UART_TX]:
            d["char"] = chr(self.value & 0xff)
        return d

    def __str__(self):
        return "{:>16d} {:<10s} src={:<3d} value=0x{:x}".format(
            self.timestamp, self.name, self.source, self.value)

# Record Parsing -----------------------------------------------------------------------------------

_RECORD = struct.Struct("<BB6sQ")

def pack_event(event: Event) -> bytes:
    """Pack an Event into a 16-byte record (inverse of parse_record, used by tests/simulation)."""
    return _RECORD.pack(event.type, event.source, event.value.to_bytes(6, "little"),
        event.timestamp)

def parse_record(record: bytes) -> Event:
    """Parse one 16-byte little-endian record."""
    if len(record) != c.EVENT_RECORD_SIZE:
        raise ValueError("Event record must be {} bytes (got {}).".format(
            c.EVENT_RECORD_SIZE, len(record)))
    typ, source, value, timestamp = _RECORD.unpack(record)
    return Event(type=typ, source=source, value=int.from_bytes(value, "little"),
        timestamp=timestamp)

class EventParser:
    """Incremental parser: feed USB transfers of any size, get complete Events."""
    def __init__(self):
        self._buffer = b""

    def feed(self, data: bytes) -> List[Event]:
        self._buffer += bytes(data)
        size   = c.EVENT_RECORD_SIZE
        n      = len(self._buffer)//size
        events = [parse_record(self._buffer[i*size:(i + 1)*size]) for i in range(n)]
        self._buffer = self._buffer[n*size:]
        return events

def parse_records(data: bytes) -> List[Event]:
    """Parse a buffer of complete 16-byte records."""
    if len(data) % c.EVENT_RECORD_SIZE:
        raise ValueError("Events buffer length {} is not a multiple of {}.".format(
            len(data), c.EVENT_RECORD_SIZE))
    return EventParser().feed(data)

# Filtering / Correlation --------------------------------------------------------------------------

def _type_codes(types) -> Optional[set]:
    if types is None:
        return None
    if isinstance(types, (str, int)):
        types = [types]
    codes = set()
    for t in types:
        if isinstance(t, str):
            if t not in c.EVENT_TYPES:
                raise ValueError("Unknown event type {} (valid: {}).".format(
                    t, ", ".join(sorted(c.EVENT_TYPES))))
            t = c.EVENT_TYPES[t]
        codes.add(t)
    return codes

def filter_events(events: Iterable[Event], since=None, until=None, types=None, source=None,
    sys_clk_freq=None) -> List[Event]:
    """Filter events on [since, until] (seconds if sys_clk_freq is given, else ticks), types
    (names or codes) and source index."""
    codes = _type_codes(types)
    scale = 1 if sys_clk_freq is None else sys_clk_freq
    t0    = None if since is None else since*scale
    t1    = None if until is None else until*scale
    r     = []
    for e in events:
        if t0 is not None and e.timestamp < t0:
            continue
        if t1 is not None and e.timestamp > t1:
            continue
        if codes is not None and e.type not in codes:
            continue
        if source is not None and e.source != source:
            continue
        r.append(e)
    return r

def first_after(events: Iterable[Event], timestamp, types=None, source=None) -> Optional[Event]:
    """First event at or after `timestamp` (ticks) matching types/source."""
    for e in filter_events(events, since=timestamp, types=types, source=source):
        return e
    return None

@dataclass
class UARTLine:
    timestamp : int # Ticks of the line's first byte.
    source    : int
    text      : str

    def to_dict(self, sys_clk_freq=None) -> dict:
        d = {"timestamp": self.timestamp, "uart": self.source, "text": self.text}
        if sys_clk_freq:
            d["time_s"] = self.timestamp/sys_clk_freq
        return d

def uart_lines(events: Iterable[Event], source=None, direction=c.EVENT_UART_RX,
    flush_partial=True) -> List[UARTLine]:
    """Rebuild UART text lines from UART byte events, each line stamped with its first byte."""
    lines   = []
    pending = {} # source -> (timestamp, [chars]).
    for e in events:
        if e.type != direction or (source is not None and e.source != source):
            continue
        ch = chr(e.value & 0xff)
        if ch == "\r":
            continue
        if e.source not in pending:
            pending[e.source] = (e.timestamp, [])
        if ch == "\n":
            t, chars = pending.pop(e.source)
            lines.append(UARTLine(t, e.source, "".join(chars)))
        else:
            pending[e.source][1].append(ch)
    if flush_partial:
        for src, (t, chars) in pending.items():
            lines.append(UARTLine(t, src, "".join(chars)))
        lines.sort(key=lambda l: l.timestamp)
    return lines

# Background Reader --------------------------------------------------------------------------------

class EventReader:
    """Background thread reading the events endpoint into a ring buffer.

    `read_fn(size, timeout_ms)` returns bytes (empty on timeout), e.g. a pyusb endpoint read.
    """
    def __init__(self, read_fn: Callable[[int, int], bytes], depth=1_000_000,
        transfer_size=16384, timeout_ms=100):
        self.read_fn       = read_fn
        self.transfer_size = transfer_size
        self.timeout_ms    = timeout_ms
        self.parser        = EventParser()
        self.buffer        = collections.deque(maxlen=depth)
        self.overflows     = 0     # Overflow records received (device side drops).
        self.dropped       = 0     # Events dropped from the host ring buffer.
        self.error         = None  # Exception that stopped the thread.
        self._lock         = threading.Lock()
        self._cond         = threading.Condition(self._lock)
        self._stop         = threading.Event()
        self._thread       = None

    def start(self):
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="agentlab-events", daemon=True)
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def poll(self) -> List[Event]:
        """Read one transfer synchronously (used by the thread, or directly without thread)."""
        data   = self.read_fn(self.transfer_size, self.timeout_ms)
        events = self.parser.feed(data) if data else []
        if events:
            with self._cond:
                for e in events:
                    if e.type == c.EVENT_OVERFLOW:
                        self.overflows += 1
                    if len(self.buffer) == self.buffer.maxlen:
                        self.dropped += 1
                    self.buffer.append(e)
                self._cond.notify_all()
        return events

    def _run(self):
        while not self._stop.is_set():
            try:
                self.poll()
            except Exception as e: # Report to users (device unplugged, ...).
                self.error = e
                with self._cond:
                    self._cond.notify_all()
                return

    def snapshot(self) -> List[Event]:
        with self._lock:
            return list(self.buffer)

    def clear(self):
        with self._lock:
            self.buffer.clear()

    def wait_for(self, predicate: Callable[[Event], bool], timeout: float,
        since: Optional[int] = None) -> Optional[Event]:
        """Wait for an event matching `predicate` (with timestamp >= since ticks)."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for e in self.buffer:
                    if (since is None or e.timestamp >= since) and predicate(e):
                        return e
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self.error is not None:
                    return None
                self._cond.wait(remaining)

def usb_read_fn(usb_device, interface=c.EVENTS_INTERFACE, endpoint=c.EVENTS_ENDPOINT):
    """Return a read_fn for EventReader on the events bulk endpoint (pyusb)."""
    import usb.core
    import usb.util
    usb.util.claim_interface(usb_device, interface)
    def read_fn(size, timeout_ms):
        try:
            return bytes(usb_device.read(endpoint, size, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            return b""
    return read_fn

def format_events(events: Sequence[Event], sys_clk_freq) -> List[str]:
    return ["{:14.9f}s {:<10s} src={:<3d} value=0x{:x}{}".format(
        e.time(sys_clk_freq), e.name, e.source, e.value,
        " {!r}".format(chr(e.value & 0xff)) if e.type in [c.EVENT_UART_RX, c.EVENT_UART_TX] else "")
        for e in events]
