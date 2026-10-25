#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""DUT UARTs: CDC-ACM ttys (pyserial), read/write/wait_for and baudrate (SET_LINE_CODING)."""

import re
import time
from dataclasses import dataclass
from typing import Callable, Optional, Union

def _clean(s):
    """Bytes/escaped str -> printable str (invalid UTF-8 replaced)."""
    if s is None:
        return None
    if isinstance(s, str):
        s = s.encode(errors="surrogateescape")
    return s.decode(errors="replace")

# Wait Result --------------------------------------------------------------------------------------

@dataclass
class WaitResult:
    matched : bool
    text    : str            # Text captured until the end of the match (or timeout).
    match   : Optional[str]  # Matched text.
    groups  : tuple          # Regex groups.
    elapsed : float          # Seconds.

    def to_dict(self) -> dict:
        return {
            "matched"   : self.matched,
            "match"     : self.match,
            "groups"    : list(self.groups),
            "text"      : self.text,
            "elapsed_s" : round(self.elapsed, 6),
        }

# DUT UART -----------------------------------------------------------------------------------------

class DUTUart:
    """A DUT UART on a CDC-ACM tty.

    `port` is a pyserial-like object (read/write/baudrate/timeout/in_waiting) or a tty path.
    Bytes received but not consumed by `wait_for` (after the match) are kept for the next read.
    """
    def __init__(self, port, index=0, baudrate=115200, clock: Callable[[], float] = time.monotonic):
        if isinstance(port, str):
            import serial
            port = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        self.port     = port
        self.index    = index
        self.clock    = clock
        self._pending = b""

    @property
    def baudrate(self) -> int:
        return self.port.baudrate

    def set_baudrate(self, baudrate: int):
        """Set the DUT UART baudrate (pyserial/termios: the kernel sends SET_LINE_CODING)."""
        self.port.baudrate = int(baudrate)

    def write(self, data: Union[bytes, str]) -> int:
        if isinstance(data, str):
            data = data.encode()
        n = self.port.write(data)
        self.port.flush()
        return len(data) if n is None else n

    def _read_chunk(self) -> bytes:
        return self.port.read(max(1, getattr(self.port, "in_waiting", 0) or 1))

    def read(self, timeout: float = 0.5, size: Optional[int] = None) -> bytes:
        """Read during `timeout` seconds (returns early when `size` bytes are received)."""
        data     = self._pending
        deadline = self.clock() + timeout
        while (size is None or len(data) < size) and self.clock() < deadline:
            data += self._read_chunk()
        if size is not None:
            data, self._pending = data[:size], data[size:]
        else:
            self._pending = b""
        return data

    def flush_input(self):
        """Drop pending and buffered received bytes."""
        self._pending = b""
        self.port.reset_input_buffer()

    def wait_for(self, pattern: str, timeout: float = 10.0, flags=0) -> WaitResult:
        """Wait for `pattern` (regex) on the received text (decoded as UTF-8, errors replaced).

        Returns a WaitResult with the text captured up to the end of the match; bytes received
        after the match are kept for the next read/wait_for.
        """
        regex = re.compile(pattern, flags)
        start = self.clock()
        data  = self._pending
        self._pending = b""
        while True:
            # surrogateescape: exact byte offsets even on invalid UTF-8.
            text = data.decode(errors="surrogateescape")
            m    = regex.search(text)
            if m is not None:
                end = len(text[:m.end()].encode(errors="surrogateescape"))
                self._pending = data[end:]
                return WaitResult(True, _clean(data[:end]), _clean(m.group(0)),
                    tuple(_clean(g) for g in m.groups()), self.clock() - start)
            if self.clock() - start >= timeout:
                return WaitResult(False, _clean(data), None, (), self.clock() - start)
            data += self._read_chunk()

    def close(self):
        self.port.close()
