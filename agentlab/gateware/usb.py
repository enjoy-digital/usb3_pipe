#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab USB3 composite device (doc/agentlab.md), phase 1:

| Interface(s) | Class         | Endpoints                              | Function           |
|--------------|---------------|----------------------------------------|--------------------|
| 0-1          | CDC-ACM       | EP1 IN notif, EP2 IN/OUT data          | Control (UARTBone) |
| 2-9          | CDC-ACM x4    | EP3-EP6 IN notif, EP7-EP10 IN/OUT data | DUT UARTs 0-3      |
| 10           | Vendor (0xff) | EP11 IN bulk                           | Events             |

Interfaces 11+/EP12+ reserved for the next phases (logic analyzer, JTAG/SWD, UVC).
"""

from migen import *

from litex.gen import *

from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.control     import USB3ControlDispatcher
from usb3_core.descriptors import *
from usb3_core.cdc_acm     import cdc_acm_interfaces, USB3CDCACMFunction

# Constants ----------------------------------------------------------------------------------------

AGENTLAB_VID     = 0x1d50
AGENTLAB_PID     = 0x605a
AGENTLAB_PRODUCT = "AgentLab"

CDC_MAX_BURST    = 2
EVENTS_MAX_BURST = 4

# Interfaces/Endpoints Map -------------------------------------------------------------------------

def agentlab_usb_map(n_uarts=4):
    """Interfaces/endpoints numbers: {function: {"interface", "notif_ept", "in_ept", "out_ept"}}."""
    m = {}
    m["control"] = {"interface": 0, "notif_ept": 1, "in_ept": 2, "out_ept": 2}
    for n in range(n_uarts):
        data_ept = 3 + n_uarts + n
        m[f"uart{n}"] = {"interface": 2 + 2*n, "notif_ept": 3 + n, "in_ept": data_ept,
            "out_ept": data_ept}
    m["events"] = {"interface": 2 + 2*n_uarts, "in_ept": 3 + 2*n_uarts}
    return m

# Descriptors --------------------------------------------------------------------------------------

def agentlab_descriptors(n_uarts=4, **kwargs):
    """AgentLab composite device descriptors (CDC-ACM class requests handled by the functions)."""
    kwargs.setdefault("vid",     AGENTLAB_VID)
    kwargs.setdefault("pid",     AGENTLAB_PID)
    kwargs.setdefault("product", AGENTLAB_PRODUCT)
    m = agentlab_usb_map(n_uarts)
    interfaces = []
    for name in ["control"] + [f"uart{n}" for n in range(n_uarts)]:
        f = m[name]
        interfaces += cdc_acm_interfaces(
            comm_interface = f["interface"],
            notif_ept      = f["notif_ept"],
            in_ept         = f["in_ept"],
            out_ept        = f["out_ept"],
            max_burst      = CDC_MAX_BURST)
    interfaces.append(USB3InterfaceDescriptor(m["events"]["interface"], cls=0xff, endpoints=[
        USB3EndpointDescriptor(m["events"]["in_ept"], direction=1, max_burst=EVENTS_MAX_BURST),
    ]))
    return USB3Descriptors(device_class=DEVICE_CLASS_IAD, interfaces=interfaces, **kwargs)

# AgentLab USB -------------------------------------------------------------------------------------

class AgentLabUSB(LiteXModule):
    """AgentLab USB3 composite device on a USB3 PHY.

    control     : CDC-ACM function (byte streams sink/source) for UARTBone.
    uarts[n]    : DUT UARTs CDC-ACM functions (sink/source, connect, baudrate/line_coding).
    events      : events Bulk IN endpoint sink (32-bit words packets).
    """
    def __init__(self, phy, sys_clk_freq, n_uarts=4, timer_scale=1, **kwargs):
        self.map = m = agentlab_usb_map(n_uarts)
        names = ["control"] + [f"uart{n}" for n in range(n_uarts)]

        # # #

        # Endpoints.
        endpoints = {}
        for name in names:
            f = m[name]
            endpoints[name] = (
                USB3BulkIN(ept=f["notif_ept"], max_burst=CDC_MAX_BURST), # Notifications (unused).
                USB3BulkIN(ept=f["in_ept"],    max_burst=CDC_MAX_BURST),
                USB3BulkOUT(ept=f["out_ept"],  max_burst=CDC_MAX_BURST),
            )
        events_in = USB3BulkIN(ept=m["events"]["in_ept"], max_burst=EVENTS_MAX_BURST)
        eps = sum([list(endpoints[name]) for name in names], []) + [events_in]
        assert len([ep for ep in eps if ep.direction == 1]) <= 15
        assert len([ep for ep in eps if ep.direction == 0]) <= 15

        # USB3 Device Core.
        descriptors = agentlab_descriptors(n_uarts, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
            endpoints        = sorted(eps, key=lambda ep: (ep.ept, ep.direction)),
            timer_scale      = timer_scale,
            with_ep0_handler = True)

        # EP0 class requests dispatched to the CDC-ACM functions (by interface).
        self.dispatcher = dispatcher = USB3ControlDispatcher(core.ep0,
            interfaces = [[m[name]["interface"], m[name]["interface"] + 1] for name in names])

        # CDC-ACM functions.
        functions = []
        for name, port in zip(names, dispatcher.ports):
            _, bulk_in, bulk_out = endpoints[name]
            function = USB3CDCACMFunction(port, bulk_in, bulk_out,
                interface        = m[name]["interface"],
                with_line_coding = True)
            self.add_module(name=name, module=function)
            functions.append(function)
        self.control = functions[0]
        self.uarts   = functions[1:]

        # Events.
        self.events = events_in.sink

# AgentLab USB <-> Instrument ----------------------------------------------------------------------

def agentlab_usb_connect(usb, agentlab):
    """Connections of an AgentLab instrument to its USB3 composite device (DUT UARTs <-> CDC-ACM
    functions with their line coding/connection, events -> Bulk IN): comb statements."""
    r = []
    for uart, function in zip(agentlab.uarts, usb.uarts):
        r += [
            function.source.connect(uart.sink),
            uart.source.connect(function.sink),
            uart.line_coding.eq(function.line_coding),
            uart.line_baudrate.eq(function.baudrate),
            uart.connected.eq(function.connect),
        ]
    r += [agentlab.source.connect(usb.events)]
    return r
