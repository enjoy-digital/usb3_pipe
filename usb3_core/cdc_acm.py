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

    interface: Communication Interface number: class requests of this interface only (wIndex),
    several CDC-ACM functions on the same EP0 (else any interface).
    with_line_coding: the class requests (SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE) are handled
    by the function (request handler, not in the descriptors requests table: no
    add_cdc_acm_requests, see USB3ControlDispatcher with several handlers) and the line coding set
    by the host is exposed (baudrate, stop_bits, parity, data_bits, line_coding: update pulse).
    """
    def __init__(self, ep0, bulk_in, bulk_out, interface=None, with_line_coding=False):
        self.sink    = sink    = stream.Endpoint([("data", 8)]) # i
        self.source  = source  = stream.Endpoint([("data", 8)]) # o
        self.connect = connect = Signal()                       # o
        if with_line_coding:
            self.baudrate    = baudrate    = Signal(32, reset=115200) # o: dwDTERate.
            self.stop_bits   = stop_bits   = Signal(8)                # o: bCharFormat.
            self.parity      = parity      = Signal(8)                # o: bParityType.
            self.data_bits   = data_bits   = Signal(8, reset=8)       # o: bDataBits.
            self.line_coding = line_coding = Signal()                 # o: SET_LINE_CODING (pulse).

        # # #

        # Requests of this function (interface).
        req_match = Signal()
        if interface is None:
            self.comb += req_match.eq(1)
        else:
            self.comb += req_match.eq(ep0.req_index[0:8] == interface)
        set_line_coding        = Signal()
        get_line_coding        = Signal()
        set_control_line_state = Signal()
        self.comb += [
            set_line_coding.eq(req_match & (ep0.req_type == 0x21) &
                (ep0.req_request == CDC_SET_LINE_CODING)),
            get_line_coding.eq(req_match & (ep0.req_type == 0xa1) &
                (ep0.req_request == CDC_GET_LINE_CODING)),
            set_control_line_state.eq(req_match & (ep0.req_type == 0x21) &
                (ep0.req_request == CDC_SET_CONTROL_LINE_STATE)),
        ]

        # Connection (DTR from SET_CONTROL_LINE_STATE).
        dtr = Signal()
        self.sync += [
            If(ep0.req_done & set_control_line_state,
                dtr.eq(ep0.req_value[0])
            ),
            If(~ep0.configured, dtr.eq(0)),
        ]
        self.comb += connect.eq(ep0.configured & dtr)

        # Line Coding (request handler).
        if with_line_coding:
            self.comb += ep0.req_stall.eq(ep0.req_valid &
                ~(set_line_coding | get_line_coding | set_control_line_state))

            # GET_LINE_CODING: 7 bytes response.
            get_active = Signal()
            get_word   = Signal()
            self.sync += [
                If(ep0.req_setup,
                    get_active.eq(get_line_coding),
                    get_word.eq(0),
                ).Elif(ep0.req_in.valid & ep0.req_in.ready,
                    get_word.eq(1),
                    If(ep0.req_in.last, get_active.eq(0)),
                )
            ]
            self.comb += [
                ep0.req_in.valid.eq(get_active),
                If(get_word == 0,
                    ep0.req_in.data.eq(baudrate),
                    ep0.req_in.be.eq(0b1111),
                ).Else(
                    ep0.req_in.data.eq(Cat(stop_bits, parity, data_bits)),
                    ep0.req_in.last.eq(1),
                    ep0.req_in.be.eq(0b0111),
                ),
            ]

            # SET_LINE_CODING: 7 bytes OUT Data stage, applied on the Status stage.
            set_word      = Signal(2)
            set_baudrate  = Signal(32, reset_less=True)
            set_format    = Signal(24, reset_less=True)
            self.comb += ep0.req_out.ready.eq(1)
            self.sync += [
                If(ep0.req_setup, set_word.eq(0)),
                If(ep0.req_out.valid & set_line_coding,
                    If(set_word == 0, set_baudrate.eq(ep0.req_out.data)),
                    If(set_word == 1, set_format.eq(ep0.req_out.data[0:24])),
                    If(set_word != 2, set_word.eq(set_word + 1)),
                ),
                line_coding.eq(0),
                If(ep0.req_done & set_line_coding & (set_word == 2),
                    baudrate.eq(set_baudrate),
                    stop_bits.eq(set_format[0:8]),
                    parity.eq(set_format[8:16]),
                    data_bits.eq(set_format[16:24]),
                    line_coding.eq(1),
                ),
            ]

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
