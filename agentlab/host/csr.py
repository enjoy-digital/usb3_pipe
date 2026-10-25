#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""CSR access: CSR map (LiteX csr.csv/csr.json) and Wishbone buses (direct UARTBone, RemoteClient).

The bus interface is minimal: `read(addr, length)` returns a list of 32-bit words and
`write(addr, datas)` writes a list of 32-bit words (byte addresses, word aligned).
"""

import os
import csv
import json
import time
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# CSR Map ------------------------------------------------------------------------------------------

@dataclass
class CSRRegister:
    name : str
    addr : int
    size : int  # Number of CSR words.
    mode : str  # "rw", "ro" or "wo".

@dataclass
class CSRMap:
    registers : Dict[str, CSRRegister] = field(default_factory=dict)
    constants : Dict[str, object]      = field(default_factory=dict)

    @property
    def data_width(self) -> int:
        return int(self.constants.get("config_csr_data_width", 32))

    @property
    def sys_clk_freq(self) -> Optional[float]:
        freq = self.constants.get("config_clock_frequency", None)
        return None if freq is None else float(freq)

    def __contains__(self, name):
        return name in self.registers

    def __getitem__(self, name) -> CSRRegister:
        try:
            return self.registers[name]
        except KeyError:
            raise KeyError("CSR {} not found in CSR map.".format(name)) from None

    @classmethod
    def from_file(cls, filename) -> "CSRMap":
        """Load a LiteX csr.csv or csr.json (selected on extension)."""
        with open(filename, "r") as f:
            content = f.read()
        if os.path.splitext(filename)[1] == ".json":
            return cls.from_json(content)
        return cls.from_csv(content)

    @classmethod
    def from_csv(cls, content) -> "CSRMap":
        csr_map = cls()
        lines   = [l for l in content.splitlines() if l and not l.startswith("#")]
        for row in csv.reader(lines):
            if len(row) < 3:
                continue
            kind, name, value = row[0], row[1], row[2]
            if kind == "csr_register":
                csr_map.registers[name] = CSRRegister(
                    name = name,
                    addr = int(value, 0),
                    size = int(row[3]),
                    mode = row[4],
                )
            elif kind == "constant":
                csr_map.constants[name] = _parse_constant(value)
        return csr_map

    @classmethod
    def from_json(cls, content) -> "CSRMap":
        d       = json.loads(content)
        csr_map = cls()
        for name, reg in d.get("csr_registers", {}).items():
            csr_map.registers[name] = CSRRegister(
                name = name,
                addr = int(reg["addr"]),
                size = int(reg["size"]),
                mode = reg["type"],
            )
        csr_map.constants.update(d.get("constants", {}))
        return csr_map

def _parse_constant(value):
    for base in [0, 10]:
        try:
            return int(value, base)
        except ValueError:
            pass
    try:
        return float(value)
    except ValueError:
        return value

# CSR Access ---------------------------------------------------------------------------------------

class CSRs:
    """Named CSR access on a bus (multi-word registers: most significant word first, as LiteX)."""
    def __init__(self, bus, csr_map: CSRMap):
        self.bus     = bus
        self.csr_map = csr_map

    def __contains__(self, name):
        return name in self.csr_map

    def read(self, name) -> int:
        reg   = self.csr_map[name]
        dw    = self.csr_map.data_width
        value = 0
        for data in self.bus.read(reg.addr, reg.size):
            value = (value << dw) | (data & (2**dw - 1))
        return value

    def write(self, name, value: int):
        reg   = self.csr_map[name]
        dw    = self.csr_map.data_width
        datas = [(value >> ((reg.size - 1 - i)*dw)) & (2**dw - 1) for i in range(reg.size)]
        self.bus.write(reg.addr, datas)

    def close(self):
        if hasattr(self.bus, "close"):
            self.bus.close()

# UARTBone Bus (direct, no server) -----------------------------------------------------------------

UARTBONE_CMD_WRITE_BURST_INCR = 0x01
UARTBONE_CMD_READ_BURST_INCR  = 0x02
UARTBONE_MAX_BURST_LENGTH     = 255

class UARTBoneBus:
    """Direct LiteX UARTBone client (Stream2Wishbone protocol) over a serial port.

    Frame: cmd (1 byte), length in words (1 byte), word address (big-endian), then for writes
    `length` 32-bit big-endian words; reads return `length` 32-bit big-endian words.
    """
    def __init__(self, port, addr_width=32, timeout=1.0):
        # `port` is a pyserial-like object (read/write/reset_input_buffer) or a device path.
        if isinstance(port, str):
            import serial
            port = serial.Serial(port, baudrate=115200, timeout=timeout)
        self.port       = port
        self.addr_bytes = addr_width//8
        self.timeout    = timeout

    def _header(self, cmd, addr, length):
        return bytes([cmd, length]) + (addr//4).to_bytes(self.addr_bytes, "big")

    def _read_exact(self, length):
        data     = b""
        deadline = time.monotonic() + self.timeout
        while len(data) < length:
            data += self.port.read(length - len(data))
            if len(data) < length and time.monotonic() > deadline:
                raise TimeoutError("UARTBone read timeout ({}/{} bytes).".format(len(data), length))
        return data

    def read(self, addr, length=1) -> List[int]:
        self.port.reset_input_buffer()
        datas = []
        while length:
            size = min(length, UARTBONE_MAX_BURST_LENGTH)
            self.port.write(self._header(UARTBONE_CMD_READ_BURST_INCR, addr, size))
            raw = self._read_exact(4*size)
            datas += [int.from_bytes(raw[4*i:4*(i + 1)], "big") for i in range(size)]
            addr   += 4*size
            length -= size
        return datas

    def write(self, addr, datas):
        datas = datas if isinstance(datas, list) else [datas]
        while datas:
            chunk = datas[:UARTBONE_MAX_BURST_LENGTH]
            frame = self._header(UARTBONE_CMD_WRITE_BURST_INCR, addr, len(chunk))
            frame += b"".join(d.to_bytes(4, "big") for d in chunk)
            self.port.write(frame)
            addr  += 4*len(chunk)
            datas  = datas[len(chunk):]

    def close(self):
        self.port.close()

# RemoteClient Bus (litex_server) ------------------------------------------------------------------

class RemoteClientBus:
    """CSR bus through `litex_server` (optionally spawned on the control ttyACM)."""
    def __init__(self, host="localhost", port=1234, spawn_uart=None, spawn_timeout=5.0):
        self.process = None
        if spawn_uart is not None:
            self.process = self._spawn(spawn_uart, port, spawn_timeout)
        from litex import RemoteClient
        self.client = RemoteClient(host=host, port=port)
        self.client.open()

    @staticmethod
    def _spawn(uart, port, timeout):
        exe = shutil.which("litex_server")
        if exe is None:
            raise RuntimeError("litex_server not found in PATH.")
        cmd     = [exe, "--uart", "--uart-port", uart, "--bind-port", str(port)]
        process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(min(timeout, 1.0)) # Let the server bind its port.
        if process.poll() is not None:
            raise RuntimeError("litex_server exited (port {} busy?).".format(port))
        return process

    def read(self, addr, length=1) -> List[int]:
        datas = self.client.read(addr, length=length)
        return datas if isinstance(datas, list) else [datas]

    def write(self, addr, datas):
        self.client.write(addr, datas if isinstance(datas, list) else [datas])

    def close(self):
        self.client.close()
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
