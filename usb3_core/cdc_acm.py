#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-ACM serial port (USB CDC 1.2, ACM subclass).

A USB3 device with a CDC-ACM function, exposed as a LiteX UART PHY (8-bit sink/source streams, as
LiteX's LunaCDCACM): usable as the SoC console (BIOS/Linux on /dev/ttyACMx, litex_term) or to carry
the UARTBone protocol (litex_server --uart).

    self.usb3_acm = USB3CDCACM(usb3_phy, sys_clk_freq)
    self.uart     = UART(self.usb3_acm)
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import cdc_acm_descriptors
from usb3_core.bridge      import USB3ByteUnpacker, USB3BytePacker

# Constants ----------------------------------------------------------------------------------------

CDC_SET_CONTROL_LINE_STATE = 0x22

# USB3 CDC-ACM -------------------------------------------------------------------------------------

class USB3CDCACM(LiteXModule):
    """USB3 CDC-ACM serial port.

    sink  : bytes to the host (dropped while no terminal is connected: no back-pressure on the SoC
            console before the host opens the port).
    source: bytes from the host.
    connect: host terminal connected (device configured and DTR set by SET_CONTROL_LINE_STATE).
    """
    def __init__(self, phy, sys_clk_freq, product="USB3-PIPE Serial", max_burst=2, **kwargs):
        self.sink    = sink    = stream.Endpoint([("data", 8)]) # i
        self.source  = source  = stream.Endpoint([("data", 8)]) # o
        self.connect = connect = Signal()                       # o

        # # #

        # USB3 Device: CDC-ACM Data Bulk IN/OUT (EP1/EP2), Notification Interrupt IN (EP3, unused).
        descriptors = cdc_acm_descriptors(max_burst=max_burst, product=product, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
            endpoints = [
                USB3BulkIN(ept=1,  max_burst=max_burst),
                USB3BulkOUT(ept=2, max_burst=max_burst),
                USB3BulkIN(ept=3,  max_burst=max_burst),
            ],
            with_ep0_handler = True)
        bulk_in, bulk_out, _ = core.endpoints
        ep0 = core.ep0

        # Connection (DTR from SET_CONTROL_LINE_STATE).
        dtr = Signal()
        self.sync += [
            If(ep0.req_done & (ep0.req_type == 0x21) & (ep0.req_request == CDC_SET_CONTROL_LINE_STATE),
                dtr.eq(ep0.req_value[0])
            ),
            If(~core.configured, dtr.eq(0)),
        ]
        self.comb += connect.eq(core.configured & dtr)

        # Host -> Device.
        self.unpacker = unpacker = USB3ByteUnpacker()
        self.comb += [
            bulk_out.source.connect(unpacker.sink),
            unpacker.source.connect(source),
        ]

        # Device -> Host (dropped while not connected).
        self.packer = packer = USB3BytePacker()
        self.comb += [
            If(connect,
                sink.connect(packer.sink)
            ).Else(
                sink.ready.eq(1)
            ),
            packer.source.connect(bulk_in.sink),
        ]
