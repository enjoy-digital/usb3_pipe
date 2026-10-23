#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Descriptors builder (USB 3.2 chapter 9).

Builds the Device, Configuration (Interfaces, Endpoints, SuperSpeed Endpoint Companions, class
specific descriptors), BOS and String descriptors as bytes, and the control requests table used by
the Control Endpoint (see usb3_core.control).
"""

# Constants ----------------------------------------------------------------------------------------

DESC_DEVICE        = 0x01
DESC_CONFIGURATION = 0x02
DESC_STRING        = 0x03
DESC_INTERFACE     = 0x04
DESC_ENDPOINT      = 0x05
DESC_BOS           = 0x0f
DESC_DEVICE_CAP    = 0x10
DESC_SS_EP_COMP    = 0x30
DESC_IAD           = 0x0b
DESC_CS_INTERFACE  = 0x24

# Device class of devices with Interface Association Descriptors (Miscellaneous, Common Class, IAD).
DEVICE_CLASS_IAD = (0xef, 0x02, 0x01)

EP_BULK      = 0x02
EP_INTERRUPT = 0x03

def le16(v):
    return [v & 0xff, (v >> 8) & 0xff]

def le32(v):
    return le16(v & 0xffff) + le16(v >> 16)

# Endpoint/Interface -------------------------------------------------------------------------------

class USB3EndpointDescriptor:
    def __init__(self, ept, direction, kind=EP_BULK, max_burst=1, max_packet=1024, interval=0):
        self.ept        = ept
        self.direction  = direction # 0: OUT, 1: IN.
        self.kind       = kind
        self.max_burst  = max_burst
        self.max_packet = max_packet
        self.interval   = interval

    def bytes(self):
        address = self.ept | (self.direction << 7)
        ep   = [7, DESC_ENDPOINT, address, self.kind] + le16(self.max_packet) + [self.interval]
        bpi  = self.max_packet*self.max_burst if self.kind == EP_INTERRUPT else 0
        comp = [6, DESC_SS_EP_COMP, self.max_burst - 1, 0] + le16(bpi)
        return ep + comp

class USB3InterfaceDescriptor:
    def __init__(self, number, cls=0xff, subclass=0x00, protocol=0x00, endpoints=[], extra=[], string=0):
        self.number    = number
        self.cls       = cls
        self.subclass  = subclass
        self.protocol  = protocol
        self.endpoints = endpoints
        self.extra     = extra # Class specific descriptors (bytes).
        self.string    = string

    def bytes(self):
        r  = [9, DESC_INTERFACE, self.number, 0, len(self.endpoints), self.cls, self.subclass,
            self.protocol, self.string]
        r += self.extra
        for ep in self.endpoints:
            r += ep.bytes()
        return r

# Descriptors --------------------------------------------------------------------------------------

class USB3Descriptors:
    """USB3 device descriptors.

    interfaces: list of USB3InterfaceDescriptor (or raw bytes for IADs/class descriptors).
    """
    def __init__(self, vid=0x1d50, pid=0x605a, bcd_device=0x0100,
        manufacturer = "Enjoy-Digital",
        product      = "USB3-PIPE",
        serial       = "0001",
        interfaces   = [],
        max_power_ma = 0,
        device_class = (0x00, 0x00, 0x00)):
        self.vid          = vid
        self.pid          = pid
        self.bcd_device   = bcd_device
        self.strings      = [manufacturer, product, serial]
        self.interfaces   = interfaces
        self.max_power_ma = max_power_ma
        self.device_class = device_class # (bDeviceClass, bDeviceSubClass, bDeviceProtocol).
        self.class_requests = [] # (bmRequestType, bRequest, IN response bytes or None).

    def device(self):
        return [18, DESC_DEVICE] + le16(0x0320) + list(self.device_class) + [9] + \
            le16(self.vid) + le16(self.pid) + le16(self.bcd_device) + [1, 2, 3, 1]

    def configuration(self):
        body = []
        n_interfaces = 0
        for interface in self.interfaces:
            if isinstance(interface, USB3InterfaceDescriptor):
                body += interface.bytes()
                n_interfaces += 1
            else:
                body += list(interface)
        total = 9 + len(body)
        # Self powered, max power in 8mA units (SuperSpeed).
        return [9, DESC_CONFIGURATION] + le16(total) + [n_interfaces, 1, 0, 0xc0,
            self.max_power_ma//8] + body

    def bos(self):
        # USB 2.0 Extension (no LPM) and SuperSpeed USB Device Capability: SuperSpeed only, U1/U2
        # exit latencies of 0 (U1/U2 not supported: LGO_U1/LGO_U2 are rejected).
        usb2_ext = [7, DESC_DEVICE_CAP, 0x02] + le32(0)
        ss_cap   = [10, DESC_DEVICE_CAP, 0x03, 0x00] + le16(0x0008) + [3, 0x00] + le16(0x0000)
        caps     = usb2_ext + ss_cap
        return [5, DESC_BOS] + le16(5 + len(caps)) + [2] + caps

    def string(self, index):
        if index == 0:
            return [4, DESC_STRING] + le16(0x0409)
        s = self.strings[index - 1].encode("utf-16-le")
        return [2 + len(s), DESC_STRING] + list(s)

    def add_class_request(self, bmRequestType, bRequest, response=None):
        """Class/vendor request accepted by the Control Endpoint (IN: constant response bytes)."""
        self.class_requests.append((bmRequestType, bRequest, response))

    def get_requests(self):
        """Control requests table: list of (bmRequestType, bRequest, wValue, response bytes or None,
        kind). wValue None: any. kind: "in", "out", "set_address", "set_configuration",
        "clear_feature"."""
        r = []
        # GET_DESCRIPTOR.
        r.append((0x80, 0x06, (DESC_DEVICE        << 8) | 0, self.device(),        "in"))
        r.append((0x80, 0x06, (DESC_CONFIGURATION << 8) | 0, self.configuration(), "in"))
        r.append((0x80, 0x06, (DESC_BOS           << 8) | 0, self.bos(),           "in"))
        for i in range(len(self.strings) + 1):
            r.append((0x80, 0x06, (DESC_STRING << 8) | i, self.string(i), "in"))
        # GET_STATUS (Device/Interface/Endpoint), GET_CONFIGURATION/GET_INTERFACE.
        for bmRequestType in [0x80, 0x81, 0x82]:
            r.append((bmRequestType, 0x00, None, [0x00, 0x00], "in"))
        r.append((0x80, 0x08, None, [0x01], "in"))
        r.append((0x81, 0x0a, None, [0x00], "in"))
        # SET_ADDRESS, SET_CONFIGURATION, SET/CLEAR_FEATURE, SET_INTERFACE, SET_SEL, SET_ISOCH_DELAY.
        r.append((0x00, 0x05, None, None, "set_address"))
        r.append((0x00, 0x09, None, None, "set_configuration"))
        for bmRequestType in [0x00, 0x01, 0x02]:
            r.append((bmRequestType, 0x03, None, None, "out"))
        r.append((0x00, 0x01, None, None, "out"))
        r.append((0x01, 0x01, None, None, "out"))
        r.append((0x02, 0x01, None, None, "clear_feature"))
        r.append((0x01, 0x0b, None, None, "out"))
        r.append((0x00, 0x30, None, None, "out"))
        r.append((0x00, 0x31, None, None, "out"))
        # Class/vendor requests.
        for bmRequestType, bRequest, response in self.class_requests:
            r.append((bmRequestType, bRequest, None, response, "in" if response is not None else "out"))
        return r

# Configurations -----------------------------------------------------------------------------------

def bulk_descriptors(in_ept=1, out_ept=2, max_burst=8, **kwargs):
    """Vendor class interface with a Bulk IN and a Bulk OUT endpoint."""
    interface = USB3InterfaceDescriptor(0, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    return USB3Descriptors(interfaces=[interface], **kwargs)

def cdc_acm_descriptors(notif_ept=3, in_ept=1, out_ept=2, max_burst=8, **kwargs):
    """CDC-ACM (Abstract Control Model) function: Communication Interface (notification Interrupt
    IN endpoint, never sending notifications) and Data Interface (Bulk IN/OUT)."""
    iad = [8, DESC_IAD, 0, 2, 0x02, 0x02, 0x01, 0]
    cs  = []
    cs += [5, DESC_CS_INTERFACE, 0x00] + le16(0x0110) # Header.
    cs += [5, DESC_CS_INTERFACE, 0x01, 0x00, 1]       # Call Management.
    cs += [4, DESC_CS_INTERFACE, 0x02, 0x02]          # ACM (Line Coding/Serial State).
    cs += [5, DESC_CS_INTERFACE, 0x06, 0, 1]          # Union.
    comm = USB3InterfaceDescriptor(0, cls=0x02, subclass=0x02, protocol=0x01, extra=cs, endpoints=[
        USB3EndpointDescriptor(notif_ept, direction=1, kind=EP_INTERRUPT, max_packet=16, interval=9),
    ])
    data = USB3InterfaceDescriptor(1, cls=0x0a, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    kwargs.setdefault("device_class", DEVICE_CLASS_IAD)
    descriptors = USB3Descriptors(interfaces=[iad, comm, data], **kwargs)
    # SET_LINE_CODING/GET_LINE_CODING (115200 8N1)/SET_CONTROL_LINE_STATE.
    descriptors.add_class_request(0x21, 0x20)
    descriptors.add_class_request(0xa1, 0x21, le32(115200) + [0, 0, 8])
    descriptors.add_class_request(0x21, 0x22)
    return descriptors
