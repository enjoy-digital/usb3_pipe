#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Instrument: one object for CSRs, DUT UARTs, power, GPIOs, timestamp/markers, events."""

import time
from typing import Callable, Dict, List, Optional

from agentlab.host import constants as c
from agentlab.host.csr       import CSRMap, CSRs, UARTBoneBus, RemoteClientBus
from agentlab.host.uart      import DUTUart
from agentlab.host.events    import EventReader, filter_events, uart_lines, usb_read_fn
from agentlab.host.discovery import DeviceInfo, find_device, find_usb_device

# Instrument ---------------------------------------------------------------------------------------

class Instrument:
    """AgentLab instrument.

    Built from its backends (CSRs, UART factory, events read function) so tests can inject fakes;
    use `Instrument.open()` to connect to a real device.
    """
    def __init__(self, csrs: Optional[CSRs] = None,
        uart_factory : Optional[Callable[[int], DUTUart]] = None,
        events_read  : Optional[Callable[[int, int], bytes]] = None,
        sys_clk_freq : Optional[float] = None,
        device       : Optional[DeviceInfo] = None,
        sleep        : Callable[[float], None] = time.sleep):
        self.csrs          = csrs
        self.device        = device
        self.sleep         = sleep
        self._uart_factory = uart_factory
        self._uarts        = {}
        self._events_read  = events_read
        self._events       = None
        if sys_clk_freq is None and csrs is not None:
            sys_clk_freq = csrs.csr_map.sys_clk_freq
        self.sys_clk_freq  = float(sys_clk_freq or c.DEFAULT_SYS_CLK_FREQ)

    # Open / Close ---------------------------------------------------------------------------------

    @classmethod
    def open(cls, csr_csv=None, serial=None, control_port=None, server=None, spawn_server=False,
        server_port=1234, sys_clk_freq=None, sysfs_root=None, start_events=False) -> "Instrument":
        """Connect to an AgentLab device.

        csr_csv      : LiteX csr.csv/csr.json of the gateware (required for CSR features).
        serial       : USB serial number (when several devices are connected).
        control_port : control tty override (default: discovered /dev/ttyACMx of interface 0).
        server       : "host:port" of a running litex_server (RemoteClient) instead of UARTBone.
        spawn_server : spawn `litex_server --uart` on the control tty and use RemoteClient.
        start_events : start the events reader now (else on first use of `events`).
        """
        kwargs = {} if sysfs_root is None else {"sysfs_root": sysfs_root}
        device = None
        try:
            device = find_device(serial=serial, **kwargs)
        except RuntimeError:
            if control_port is None and server is None:
                raise
        csrs = None
        if csr_csv is not None:
            csr_map = CSRMap.from_file(csr_csv)
            port    = control_port or (device.control_tty if device else None)
            if server is not None:
                host, _, p = server.partition(":")
                bus = RemoteClientBus(host=host or "localhost", port=int(p or 1234))
            elif spawn_server:
                bus = RemoteClientBus(port=server_port, spawn_uart=port)
            else:
                if port is None:
                    raise RuntimeError("Control tty not found (CDC-ACM interface {}).".format(
                        c.CONTROL_INTERFACE))
                bus = UARTBoneBus(port)
            csrs = CSRs(bus, csr_map)

        def uart_factory(index):
            tty = device.uart_tty(index) if device else None
            if tty is None:
                raise RuntimeError("DUT UART{} tty not found.".format(index))
            return DUTUart(tty, index=index)

        def events_read(size, timeout_ms):
            # Bound lazily: only claims the vendor interface when events are used.
            if not hasattr(events_read, "fn"):
                if device is None:
                    raise RuntimeError("Events need the USB device (not found).")
                events_read.fn = usb_read_fn(find_usb_device(device))
            return events_read.fn(size, timeout_ms)

        instrument = cls(csrs=csrs, uart_factory=uart_factory, events_read=events_read,
            sys_clk_freq=sys_clk_freq, device=device)
        if start_events and device is not None:
            instrument.events
        return instrument

    def close(self):
        if self._events is not None:
            self._events.stop()
        for uart in self._uarts.values():
            uart.close()
        self._uarts.clear()
        if self.csrs is not None:
            self.csrs.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # Helpers --------------------------------------------------------------------------------------

    def _csrs(self) -> CSRs:
        if self.csrs is None:
            raise RuntimeError("CSR access needs a CSR map (csr.csv/csr.json).")
        return self.csrs

    def _set_bit(self, name, index, value):
        csrs = self._csrs()
        reg  = csrs.read(name)
        reg  = (reg | (1 << index)) if value else (reg & ~(1 << index))
        csrs.write(name, reg)

    def ticks_to_s(self, ticks: int) -> float:
        return ticks/self.sys_clk_freq

    def info(self) -> Dict[str, object]:
        d = {"sys_clk_freq": self.sys_clk_freq}
        if self.device is not None:
            d.update(self.device.describe())
        if self.csrs is not None:
            d["csrs"] = sorted(n for n in self.csrs.csr_map.registers if n.startswith("agentlab_"))
        return d

    # Timestamp / Marker ---------------------------------------------------------------------------

    def timestamp(self) -> int:
        """Current device timestamp (sys_clk ticks)."""
        csrs = self._csrs()
        if c.CSR_TIMESTAMP_LATCH in csrs:
            csrs.write(c.CSR_TIMESTAMP_LATCH, 1)
        return csrs.read(c.CSR_TIMESTAMP)

    def time(self) -> float:
        """Current device time (seconds)."""
        return self.ticks_to_s(self.timestamp())

    def marker(self, value: int = 0) -> int:
        """Emit a marker event (value) and return the device timestamp read just after."""
        self._csrs().write(c.CSR_MARKER, value)
        return self.timestamp()

    # Power ----------------------------------------------------------------------------------------

    def power_set(self, index: int, on: bool):
        self._set_bit(c.CSR_POWER_OUT, index, on)

    def power_get(self, index: int) -> bool:
        """Power switch state (sense when available, else the commanded state)."""
        csrs = self._csrs()
        name = c.CSR_POWER_STATUS if c.CSR_POWER_STATUS in csrs else c.CSR_POWER_OUT
        return bool((csrs.read(name) >> index) & 1)

    def power_cycle(self, index: int, off_time: float = 1.0):
        """Switch power `index` off, wait `off_time` seconds, switch it on."""
        self.power_set(index, False)
        self.sleep(off_time)
        self.power_set(index, True)

    # GPIOs ----------------------------------------------------------------------------------------

    def gpio_dir(self, index: int, output: bool):
        self._set_bit(c.CSR_GPIO_OE, index, output)

    def gpio_set(self, index: int, value: bool):
        """Drive GPIO `index` to `value` (and enable its output)."""
        self._set_bit(c.CSR_GPIO_OUT, index, value)
        self.gpio_dir(index, True)

    def gpio_release(self, index: int):
        """Release GPIO `index` (input, high impedance)."""
        self.gpio_dir(index, False)

    def gpio_get(self, index: int) -> bool:
        return bool((self._csrs().read(c.CSR_GPIO_IN) >> index) & 1)

    def gpio_edge_events(self, index: int, enable: bool = True):
        self._set_bit(c.CSR_GPIO_EDGE_MASK, index, enable)

    # DUT UARTs ------------------------------------------------------------------------------------

    def uart(self, index: int) -> DUTUart:
        if index not in self._uarts:
            if self._uart_factory is None:
                raise RuntimeError("No DUT UART backend.")
            self._uarts[index] = self._uart_factory(index)
        return self._uarts[index]

    def uart_set_baudrate(self, index: int, baudrate: int):
        """Set the DUT UART baudrate (SET_LINE_CODING, plus the CSR when present)."""
        self.uart(index).set_baudrate(baudrate)
        name = c.csr_uart_baudrate(index)
        if self.csrs is not None and name in self.csrs:
            self.csrs.write(name, int(baudrate))

    # Events ---------------------------------------------------------------------------------------

    @property
    def events(self) -> EventReader:
        """Events reader (background thread started on first access)."""
        if self._events is None:
            if self._events_read is None:
                raise RuntimeError("No events backend.")
            self._events = EventReader(self._events_read).start()
        return self._events

    def events_between(self, since=None, until=None, types=None, source=None) -> List:
        """Events received in [since, until] (device seconds), filtered by types/source."""
        return filter_events(self.events.snapshot(), since=since, until=until, types=types,
            source=source, sys_clk_freq=self.sys_clk_freq)

    def uart_lines(self, index=None, since=None, until=None):
        """DUT UART RX text lines (with device timestamps) from the events stream."""
        events = self.events_between(since=since, until=until, types="uart_rx", source=index)
        return uart_lines(events)
