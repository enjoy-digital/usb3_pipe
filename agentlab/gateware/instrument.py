#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab instrument (phase 1): timebase, DUT UARTs, GPIOs, power outputs, markers and their
events stream. CSR names (agentlab_*, doc/agentlab.md) are the host API contract."""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from agentlab.gateware.timebase import AgentLabTimebase
from agentlab.gateware.events   import *
from agentlab.gateware.gpio     import AgentLabGPIO, AgentLabPower
from agentlab.gateware.uart     import AgentLabUART

# AgentLab -----------------------------------------------------------------------------------------

class AgentLab(LiteXModule):
    """AgentLab instrument.

    uart_pads : DUT UARTs pads (tx/rx), None entries: internal loopback.
    gpio_pads : DUT GPIOs pads (tristate), None: n_gpios internal GPIOs (simulation/no pins).
    power_pads: DUT power switches pads, None: n_power CSR only switches.

    uarts[n].sink/source: DUT UARTs byte streams (to/from the CDC-ACM functions).
    source: events stream (32-bit words packets, to the events Bulk IN endpoint).
    """
    def __init__(self, sys_clk_freq, uart_pads=[None]*4, gpio_pads=None, n_gpios=0,
        power_pads=None, n_power=0, events_fifo_depth=512):
        self.source = stream.Endpoint([("data", 32), ("be", 4)]) # o

        self._marker = CSRStorage(32, description="Marker: write to insert a marker event (value).")

        self.n_uarts = len(uart_pads)
        self.n_gpios = len(gpio_pads)  if gpio_pads  is not None else n_gpios
        self.n_power = len(power_pads) if power_pads is not None else n_power

        # # #

        # Timebase ---------------------------------------------------------------------------------
        self.timestamp = timebase = AgentLabTimebase()
        timestamp = timebase.timestamp

        ports     = []
        overflows = []
        def add_port(port):
            ports.append(port.source)
            overflows.append(port.overflow)

        # DUT UARTs --------------------------------------------------------------------------------
        self.uarts = []
        rx_ports   = []
        tx_ports   = []
        for n, pads in enumerate(uart_pads):
            uart = AgentLabUART(pads, sys_clk_freq)
            self.add_module(name=f"uart{n}", module=uart)
            self.uarts.append(uart)
            rx_port = AgentLabEventPort(timestamp, EVENT_UART_RX, source=n)
            tx_port = AgentLabEventPort(timestamp, EVENT_UART_TX, source=n)
            self.add_module(name=f"uart{n}_rx_event", module=rx_port)
            self.add_module(name=f"uart{n}_tx_event", module=tx_port)
            self.comb += [
                rx_port.valid.eq(uart.rx_event),
                rx_port.value.eq(uart.rx_data),
                tx_port.valid.eq(uart.tx_event),
                tx_port.value.eq(uart.tx_data),
            ]
            rx_ports.append(rx_port)
            tx_ports.append(tx_port)
        for port in rx_ports + tx_ports:
            add_port(port)

        # GPIOs ------------------------------------------------------------------------------------
        if self.n_gpios > 0:
            self.gpio = gpio = AgentLabGPIO(gpio_pads, n_gpios)
            self.gpio_event = gpio_event = AgentLabVectorEvents(timestamp, gpio.n, EVENT_GPIO)
            self.comb += [
                gpio_event.changes.eq(gpio.changes),
                gpio_event.levels.eq(gpio.levels),
            ]
            add_port(gpio_event)

        # Power ------------------------------------------------------------------------------------
        if self.n_power > 0:
            self.power = power = AgentLabPower(power_pads, n_power)
            self.power_event = power_event = AgentLabVectorEvents(timestamp, power.n, EVENT_POWER)
            self.comb += [
                power_event.changes.eq(power.changes),
                power_event.levels.eq(power.levels),
            ]
            add_port(power_event)

        # Marker -----------------------------------------------------------------------------------
        self.marker_event = marker_event = AgentLabEventPort(timestamp, EVENT_MARKER)
        self.comb += [
            marker_event.valid.eq(self._marker.re),
            marker_event.value.eq(self._marker.storage),
        ]
        add_port(marker_event)

        # Events -----------------------------------------------------------------------------------
        self.events = events = AgentLabEvents(timestamp, ports, overflows,
            fifo_depth = events_fifo_depth)
        self.comb += events.source.connect(self.source)
