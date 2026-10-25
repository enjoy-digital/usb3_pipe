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
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import *
from usb3_core.bridge      import USB3ByteUnpacker, USB3BytePacker

# Constants ----------------------------------------------------------------------------------------

CDC_SET_LINE_CODING        = 0x20
CDC_GET_LINE_CODING        = 0x21
CDC_SET_CONTROL_LINE_STATE = 0x22

# Descriptors --------------------------------------------------------------------------------------

def cdc_acm_interfaces(comm_interface=0, notif_ept=3, in_ept=1, out_ept=2, max_burst=8):
    """CDC-ACM (Abstract Control Model) function: IAD, Communication Interface (notification
    Interrupt IN endpoint, never sending notifications) and Data Interface (comm_interface + 1, Bulk
    IN/OUT)."""
    data_interface = comm_interface + 1
    iad = [8, DESC_IAD, comm_interface, 2, 0x02, 0x02, 0x01, 0]
    cs  = []
    cs += [5, DESC_CS_INTERFACE, 0x00] + le16(0x0110)                 # Header.
    cs += [5, DESC_CS_INTERFACE, 0x01, 0x00, data_interface]          # Call Management.
    cs += [4, DESC_CS_INTERFACE, 0x02, 0x02]                          # ACM (Line Coding/State).
    cs += [5, DESC_CS_INTERFACE, 0x06, comm_interface, data_interface] # Union.
    comm = USB3InterfaceDescriptor(comm_interface, cls=0x02, subclass=0x02, protocol=0x01, extra=cs,
        endpoints=[USB3EndpointDescriptor(notif_ept, direction=1, kind=EP_INTERRUPT, max_packet=16,
            interval=9)])
    data = USB3InterfaceDescriptor(data_interface, cls=0x0a, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    return [iad, comm, data]

def add_cdc_acm_requests(descriptors):
    """CDC-ACM class requests: SET/GET_LINE_CODING (115200 8N1), SET_CONTROL_LINE_STATE."""
    descriptors.add_class_request(0x21, CDC_SET_LINE_CODING)
    descriptors.add_class_request(0xa1, CDC_GET_LINE_CODING, le32(115200) + [0, 0, 8])
    descriptors.add_class_request(0x21, CDC_SET_CONTROL_LINE_STATE)

def cdc_acm_descriptors(notif_ept=3, in_ept=1, out_ept=2, max_burst=8, **kwargs):
    """CDC-ACM device."""
    kwargs.setdefault("device_class", DEVICE_CLASS_IAD)
    interfaces  = cdc_acm_interfaces(0, notif_ept, in_ept, out_ept, max_burst)
    descriptors = USB3Descriptors(interfaces=interfaces, **kwargs)
    add_cdc_acm_requests(descriptors)
    return descriptors

# USB3 CDC-ACM Function ----------------------------------------------------------------------------

class USB3CDCACMFunction(LiteXModule):
    """CDC-ACM function of a USB3 device (EP0 with request handler, Bulk IN/OUT data endpoints).

    sink  : bytes to the host (dropped while no terminal is connected: no back-pressure on the SoC
            console before the host opens the port).
    source: bytes from the host.
    connect: host terminal connected (device configured and DTR set by SET_CONTROL_LINE_STATE).
    """
    def __init__(self, ep0, bulk_in, bulk_out):
        self.sink    = sink    = stream.Endpoint([("data", 8)]) # i
        self.source  = source  = stream.Endpoint([("data", 8)]) # o
        self.connect = connect = Signal()                       # o

        # # #

        # Connection (DTR from SET_CONTROL_LINE_STATE).
        dtr = Signal()
        self.sync += [
            If(ep0.req_done & (ep0.req_type == 0x21) &
                (ep0.req_request == CDC_SET_CONTROL_LINE_STATE),
                dtr.eq(ep0.req_value[0])
            ),
            If(~ep0.configured, dtr.eq(0)),
        ]
        self.comb += connect.eq(ep0.configured & dtr)

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

# USB3 CDC-ACM -------------------------------------------------------------------------------------

class USB3CDCACM(LiteXModule):
    """USB3 CDC-ACM serial port device (see USB3CDCACMFunction)."""
    def __init__(self, phy, sys_clk_freq, product="USB3-PIPE Serial", max_burst=2, **kwargs):
        # USB3 Device: CDC-ACM Data Bulk IN/OUT (EP1/EP2), Notification Interrupt IN (EP3, unused).
        descriptors = cdc_acm_descriptors(max_burst=max_burst, product=product, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
            endpoints = [
                USB3BulkIN(ept=1,  max_burst=max_burst),
                USB3BulkOUT(ept=2, max_burst=max_burst),
                USB3BulkIN(ept=3,  max_burst=max_burst),
            ],
            with_ep0_handler = True)
        function = USB3CDCACMFunction(core.ep0, core.endpoints[0], core.endpoints[1])
        self.function = function
        self.sink    = function.sink
        self.source  = function.source
        self.connect = function.connect
