#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab instrument: timebase, DUT UARTs, GPIOs, power outputs, markers and their events
stream (phase 1), logic analyzer (phase 2), Eyes text-mode renderer (phase 4). CSR names
(agentlab_*, doc/agentlab.md) are the host API contract."""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from agentlab.gateware.timebase import AgentLabTimebase
from agentlab.gateware.events   import *
from agentlab.gateware.gpio     import AgentLabGPIO, AgentLabPower
from agentlab.gateware.uart     import AgentLabUART
from agentlab.gateware.la       import AgentLabLA, LA_EXT_MARKER, LA_EXT_GPIO, LA_EXT_POWER
from agentlab.gateware.eyes     import AgentLabEyes

# AgentLab -----------------------------------------------------------------------------------------

class AgentLab(LiteXModule):
    """AgentLab instrument.

    uart_pads : DUT UARTs pads (tx/rx), None entries: internal loopback.
    gpio_pads : DUT GPIOs pads (tristate), None: n_gpios internal GPIOs (simulation/no pins).
    power_pads: DUT power switches pads, None: n_power CSR only switches.
    la_inputs : logic analyzer channels (pads/signal, <= 32), "probes": internal probes (see
                la_probes), None: no logic analyzer.
    la_kwargs : logic analyzer DRAM ring buffer/parameters (AgentLabLA: write_port, read_port,
                ring_base, ring_size...).

    uarts[n].sink/source: DUT UARTs byte streams (to/from the CDC-ACM functions).
    source: events stream (32-bit words packets, to the events Bulk IN endpoint).
    la.source: logic analyzer stream (32-bit words packets, to the LA Bulk IN endpoint).
    eyes  : Eyes renderer (with_eyes, eyes_kwargs: AgentLabEyes arguments), waveform lanes
            sources: GPIOs, power switches, then DUT UARTs TX/RX lines (bit 0: GPIO0).
    """
    def __init__(self, sys_clk_freq, uart_pads=[None]*4, gpio_pads=None, n_gpios=0,
        power_pads=None, n_power=0, events_fifo_depth=512, la_inputs=None, la_kwargs={},
        with_eyes=False, eyes_kwargs={}):
        self.source = stream.Endpoint([("data", 32), ("be", 4)]) # o

        self._marker = CSRStorage(32, description="Marker: write to insert a marker event (value).")

        self.n_uarts = len(uart_pads)
        self.n_gpios = len(gpio_pads)  if gpio_pads  is not None else n_gpios
        self.n_power = len(power_pads) if power_pads is not None else n_power
        self.with_la = la_inputs is not None

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

        # Logic Analyzer ---------------------------------------------------------------------------
        if self.with_la:
            if isinstance(la_inputs, str):
                assert la_inputs == "probes"
                la_inputs = self.la_probes()
            self.la = la = AgentLabLA(la_inputs, timestamp, **la_kwargs)
            self.la_event = la_event = AgentLabEventPort(timestamp, EVENT_LA_TRIGGER)
            self.comb += [
                la_event.valid.eq(la.trigger_event),
                la_event.value.eq(la.trigger_value),
                la.ext_trigger[LA_EXT_MARKER].eq(self._marker.re),
            ]
            if self.n_gpios > 0:
                self.comb += la.ext_trigger[LA_EXT_GPIO].eq(gpio.changes != 0)
            if self.n_power > 0:
                self.comb += la.ext_trigger[LA_EXT_POWER].eq(power.changes != 0)
            add_port(la_event)

        # Events -----------------------------------------------------------------------------------
        self.events = events = AgentLabEvents(timestamp, ports, overflows,
            fifo_depth = events_fifo_depth)
        self.comb += events.source.connect(self.source)

        # Eyes -------------------------------------------------------------------------------------
        if with_eyes:
            self.eyes = eyes = AgentLabEyes(**eyes_kwargs)
            lanes = []
            if self.n_gpios > 0:
                lanes.append(gpio.levels)
            if self.n_power > 0:
                lanes.append(power.levels)
            for uart in self.uarts:
                lanes += [uart.tx_line, uart.rx_line]
            self.eyes_lanes = len(Cat(*lanes))
            assert self.eyes_lanes <= eyes.wave_width
            self.comb += eyes.samples.eq(Cat(*lanes))

    def la_probes(self):
        """Logic analyzer internal probes (32 channels, validation without DUT pins):

        | Channels | Probe                                                  |
        |----------|--------------------------------------------------------|
        | 0-7      | UART0 TX, UART0 RX, UART1 TX, UART1 RX, ... UART3 RX   |
        | 8-11     | GPIO0-3 (pins levels)                                  |
        | 12-15    | Power switches 0-3                                     |
        | 16-31    | Timestamp bits 0-15 (counter: data/timebase integrity) |
        """
        probes = Signal(32)
        uarts  = []
        for uart in self.uarts[:4]:
            uarts += [uart.tx_line, uart.rx_line]
        self.comb += [
            probes[0:2*len(self.uarts[:4])].eq(Cat(*uarts)),
            probes[16:32].eq(self.timestamp.timestamp[:16]),
        ]
        if self.n_gpios > 0:
            n = min(self.n_gpios, 4)
            self.comb += probes[8:8 + n].eq(self.gpio.i[:n])
        if self.n_power > 0:
            n = min(self.n_power, 4)
            self.comb += probes[12:12 + n].eq(self.power.levels[:n])
        return probes
