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

# Standard Requests (9.4, Table 9-5).
REQ_GET_STATUS        = 0x00
REQ_CLEAR_FEATURE     = 0x01
REQ_SET_FEATURE       = 0x03
REQ_SET_ADDRESS       = 0x05
REQ_GET_DESCRIPTOR    = 0x06
REQ_GET_CONFIGURATION = 0x08
REQ_SET_CONFIGURATION = 0x09
REQ_GET_INTERFACE     = 0x0a
REQ_SET_INTERFACE     = 0x0b
REQ_SET_SEL           = 0x30
REQ_SET_ISOCH_DELAY   = 0x31

# Standard Requests bmRequestType (9.3.1, Table 9-2): direction, recipient.
REQ_TYPE_DEVICE_OUT    = 0x00
REQ_TYPE_INTERFACE_OUT = 0x01
REQ_TYPE_ENDPOINT_OUT  = 0x02
REQ_TYPE_DEVICE_IN     = 0x80
REQ_TYPE_INTERFACE_IN  = 0x81
REQ_TYPE_ENDPOINT_IN   = 0x82

# Standard Feature Selectors (9.4, Table 9-7).
FEATURE_ENDPOINT_HALT = 0
FEATURE_U1_ENABLE     = 48
FEATURE_U2_ENABLE     = 49

# Device class of devices with Interface Association Descriptors (Miscellaneous, Common Class, IAD).
DEVICE_CLASS_IAD = (0xef, 0x02, 0x01)

EP_ISOCHRONOUS = 0x01
EP_BULK        = 0x02
EP_INTERRUPT   = 0x03

# Isochronous endpoints synchronization types (bmAttributes[3:2]).
EP_SYNC_NONE     = 0x00
EP_SYNC_ASYNC    = 0x04
EP_SYNC_ADAPTIVE = 0x08
EP_SYNC_SYNC     = 0x0c

# U1/U2 Device Exit Latencies advertised with LPM (SuperSpeed USB Device Capability, 9.6.2.2): maximum
# values (bU1DevExitLat: 10us, wU2DevExitLat: 2047us).
U1_DEV_EXIT_LAT = 0x0a
U2_DEV_EXIT_LAT = 0x07ff

def le16(v):
    return [v & 0xff, (v >> 8) & 0xff]

def le32(v):
    return le16(v & 0xffff) + le16(v >> 16)

# Endpoint/Interface -------------------------------------------------------------------------------

class USB3EndpointDescriptor:
    """Endpoint descriptor and its SuperSpeed Endpoint Companion (9.6.6/9.6.7).

    Periodic (Isochronous/Interrupt) endpoints: interval (bInterval, service interval of
    2^(interval - 1) x 125us), wBytesPerInterval of max_packet x max_burst x (mult + 1). Isochronous:
    sync (bmAttributes synchronization type), mult (companion bmAttributes Mult, bursts per service
    interval - 1).
    """
    def __init__(self, ept, direction, kind=EP_BULK, max_burst=1, max_packet=1024, interval=0,
        sync=EP_SYNC_NONE, mult=0):
        assert 1 <= max_burst <= 16
        assert mult == 0 or kind == EP_ISOCHRONOUS
        self.ept        = ept
        self.direction  = direction # 0: OUT, 1: IN.
        self.kind       = kind
        self.max_burst  = max_burst
        self.max_packet = max_packet
        self.interval   = interval
        self.sync       = sync if kind == EP_ISOCHRONOUS else 0
        self.mult       = mult

    def bytes_per_interval(self):
        if self.kind in [EP_ISOCHRONOUS, EP_INTERRUPT]:
            return self.max_packet*self.max_burst*(self.mult + 1)
        return 0

    def bytes(self):
        address = self.ept | (self.direction << 7)
        ep   = [7, DESC_ENDPOINT, address, self.kind | self.sync] + le16(self.max_packet) + [self.interval]
        comp = [6, DESC_SS_EP_COMP, self.max_burst - 1, self.mult] + le16(self.bytes_per_interval())
        return ep + comp

class USB3InterfaceDescriptor:
    """Interface descriptor (alt: alternate setting), class specific descriptors and endpoints."""
    def __init__(self, number, cls=0xff, subclass=0x00, protocol=0x00, endpoints=[], extra=[], string=0,
        alt=0):
        self.number    = number
        self.alt       = alt
        self.cls       = cls
        self.subclass  = subclass
        self.protocol  = protocol
        self.endpoints = endpoints
        self.extra     = extra # Class specific descriptors (bytes).
        self.string    = string

    def bytes(self):
        r  = [9, DESC_INTERFACE, self.number, self.alt, len(self.endpoints), self.cls, self.subclass,
            self.protocol, self.string]
        r += self.extra
        for ep in self.endpoints:
            r += ep.bytes()
        return r

# Descriptors --------------------------------------------------------------------------------------

class USB3Descriptors:
    """USB3 device descriptors.

    interfaces: list of USB3InterfaceDescriptor (or raw bytes for IADs/class descriptors).
    with_lpm  : Link Power Management (U1/U2) supported: non-zero U1/U2 Device Exit Latencies in the
                BOS (hosts enable U1/U2 only with non-zero exit latencies), see enable_lpm().
    """
    def __init__(self, vid=0x1d50, pid=0x605a, bcd_device=0x0100,
        manufacturer = "Enjoy-Digital",
        product      = "USB3-PIPE",
        serial       = "0001",
        interfaces   = [],
        max_power_ma = 0,
        device_class = (0x00, 0x00, 0x00),
        with_lpm     = False):
        self.vid          = vid
        self.pid          = pid
        self.bcd_device   = bcd_device
        self.strings      = [manufacturer, product, serial]
        self.interfaces   = interfaces
        self.max_power_ma = max_power_ma
        self.device_class = device_class # (bDeviceClass, bDeviceSubClass, bDeviceProtocol).
        self.with_lpm     = with_lpm
        self.class_requests = [] # (bmRequestType, bRequest, IN response bytes or None).

    def enable_lpm(self):
        """U1/U2 supported (Link Layer accepting LGO_U1/LGO_U2): non-zero exit latencies."""
        self.with_lpm = True

    def device(self):
        return [18, DESC_DEVICE] + le16(0x0320) + list(self.device_class) + [9] + \
            le16(self.vid) + le16(self.pid) + le16(self.bcd_device) + [1, 2, 3, 1]

    def configuration(self):
        body = []
        n_interfaces = 0
        for interface in self.interfaces:
            if isinstance(interface, USB3InterfaceDescriptor):
                body += interface.bytes()
                n_interfaces += int(interface.alt == 0)
            else:
                body += list(interface)
        total = 9 + len(body)
        # Self powered, max power in 8mA units (SuperSpeed).
        return [9, DESC_CONFIGURATION] + le16(total) + [n_interfaces, 1, 0, 0xc0,
            self.max_power_ma//8] + body

    def bos(self):
        # USB 2.0 Extension (no USB 2.0 LPM) and SuperSpeed USB Device Capability: SuperSpeed only,
        # U1/U2 exit latencies of 0 without LPM (U1/U2 not supported: LGO_U1/LGO_U2 are rejected).
        u1_exit_lat = U1_DEV_EXIT_LAT if self.with_lpm else 0
        u2_exit_lat = U2_DEV_EXIT_LAT if self.with_lpm else 0
        usb2_ext = [7, DESC_DEVICE_CAP, 0x02] + le32(0)
        ss_cap   = [10, DESC_DEVICE_CAP, 0x03, 0x00] + le16(0x0008) + [3, u1_exit_lat] + le16(u2_exit_lat)
        caps     = usb2_ext + ss_cap
        return [5, DESC_BOS] + le16(5 + len(caps)) + [2] + caps

    def string(self, index):
        if index == 0:
            return [4, DESC_STRING] + le16(0x0409)
        s = self.strings[index - 1].encode("utf-16-le")
        return [2 + len(s), DESC_STRING] + list(s)

    def alt_settings(self):
        """Alternate settings: {interface number: number of alternate settings}."""
        r = {}
        for interface in self.interfaces:
            if isinstance(interface, USB3InterfaceDescriptor):
                r[interface.number] = max(r.get(interface.number, 0), interface.alt + 1)
        return r

    def add_class_request(self, bmRequestType, bRequest, response=None):
        """Class/vendor request accepted by the Control Endpoint (IN: constant response bytes)."""
        self.class_requests.append((bmRequestType, bRequest, response))

    def get_requests(self):
        """Control requests table: list of (bmRequestType, bRequest, wValue, response bytes or None,
        kind). wValue None: any. kind: "in", "out", "set_address", "set_configuration",
        "clear_feature", "device_feature" (SET/CLEAR_FEATURE(U1_ENABLE/U2_ENABLE)), "device_status"
        (GET_STATUS(Device)), "get_interface" (response: one word per alternate setting)."""
        r = []
        # GET_DESCRIPTOR.
        get_descriptor = (REQ_TYPE_DEVICE_IN, REQ_GET_DESCRIPTOR)
        r.append((*get_descriptor, (DESC_DEVICE        << 8) | 0, self.device(),        "in"))
        r.append((*get_descriptor, (DESC_CONFIGURATION << 8) | 0, self.configuration(), "in"))
        r.append((*get_descriptor, (DESC_BOS           << 8) | 0, self.bos(),           "in"))
        for i in range(len(self.strings) + 1):
            r.append((*get_descriptor, (DESC_STRING << 8) | i, self.string(i), "in"))
        # GET_STATUS (Device: U1/U2 Enable bits inserted by the Control Endpoint, Interface/Endpoint),
        # GET_CONFIGURATION/GET_INTERFACE.
        r.append((REQ_TYPE_DEVICE_IN, REQ_GET_STATUS, None, [0x00, 0x00], "device_status"))
        for bmRequestType in [REQ_TYPE_INTERFACE_IN, REQ_TYPE_ENDPOINT_IN]:
            r.append((bmRequestType, REQ_GET_STATUS, None, [0x00, 0x00], "in"))
        r.append((REQ_TYPE_DEVICE_IN, REQ_GET_CONFIGURATION, None, [0x01], "in"))
        n_alts = max(self.alt_settings().values(), default=1)
        alts   = sum([[alt, 0, 0, 0] for alt in range(n_alts)], [])
        r.append((REQ_TYPE_INTERFACE_IN, REQ_GET_INTERFACE, None, alts, "get_interface"))
        # SET_ADDRESS, SET_CONFIGURATION, SET/CLEAR_FEATURE, SET_INTERFACE, SET_SEL, SET_ISOCH_DELAY.
        r.append((REQ_TYPE_DEVICE_OUT, REQ_SET_ADDRESS,       None, None, "set_address"))
        r.append((REQ_TYPE_DEVICE_OUT, REQ_SET_CONFIGURATION, None, None, "set_configuration"))
        for bRequest in [REQ_SET_FEATURE, REQ_CLEAR_FEATURE]:
            for feature in [FEATURE_U1_ENABLE, FEATURE_U2_ENABLE]:
                r.append((REQ_TYPE_DEVICE_OUT, bRequest, feature, None, "device_feature"))
        for bmRequestType in [REQ_TYPE_DEVICE_OUT, REQ_TYPE_INTERFACE_OUT, REQ_TYPE_ENDPOINT_OUT]:
            r.append((bmRequestType, REQ_SET_FEATURE, None, None, "out"))
        r.append((REQ_TYPE_DEVICE_OUT,    REQ_CLEAR_FEATURE,   None, None, "out"))
        r.append((REQ_TYPE_INTERFACE_OUT, REQ_CLEAR_FEATURE,   None, None, "out"))
        r.append((REQ_TYPE_ENDPOINT_OUT,  REQ_CLEAR_FEATURE,   None, None, "clear_feature"))
        r.append((REQ_TYPE_INTERFACE_OUT, REQ_SET_INTERFACE,   None, None, "out"))
        r.append((REQ_TYPE_DEVICE_OUT,    REQ_SET_SEL,         None, None, "out"))
        r.append((REQ_TYPE_DEVICE_OUT,    REQ_SET_ISOCH_DELAY, None, None, "out"))
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

def iso_in_descriptors(ept=1, max_burst=1, interval=1, **kwargs):
    """Vendor class interface with an Isochronous IN endpoint: alternate setting 0 without endpoint
    (zero bandwidth), alternate setting 1 with the Isochronous IN endpoint (Asynchronous, Mult of 0,
    service interval of 2^(interval - 1) x 125us)."""
    alt0 = USB3InterfaceDescriptor(0, alt=0)
    alt1 = USB3InterfaceDescriptor(0, alt=1, endpoints=[
        USB3EndpointDescriptor(ept, direction=1, kind=EP_ISOCHRONOUS, max_burst=max_burst,
            interval=interval, sync=EP_SYNC_ASYNC),
    ])
    return USB3Descriptors(interfaces=[alt0, alt1], **kwargs)

def cdc_acm_interfaces(comm_interface=0, notif_ept=3, in_ept=1, out_ept=2, max_burst=8):
    """CDC-ACM (Abstract Control Model) function: IAD, Communication Interface (notification Interrupt
    IN endpoint, never sending notifications) and Data Interface (comm_interface + 1, Bulk IN/OUT)."""
    data_interface = comm_interface + 1
    iad = [8, DESC_IAD, comm_interface, 2, 0x02, 0x02, 0x01, 0]
    cs  = []
    cs += [5, DESC_CS_INTERFACE, 0x00] + le16(0x0110)                 # Header.
    cs += [5, DESC_CS_INTERFACE, 0x01, 0x00, data_interface]          # Call Management.
    cs += [4, DESC_CS_INTERFACE, 0x02, 0x02]                          # ACM (Line Coding/Serial State).
    cs += [5, DESC_CS_INTERFACE, 0x06, comm_interface, data_interface] # Union.
    comm = USB3InterfaceDescriptor(comm_interface, cls=0x02, subclass=0x02, protocol=0x01, extra=cs,
        endpoints=[USB3EndpointDescriptor(notif_ept, direction=1, kind=EP_INTERRUPT, max_packet=16, interval=9)])
    data = USB3InterfaceDescriptor(data_interface, cls=0x0a, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    return [iad, comm, data]

def add_cdc_acm_requests(descriptors):
    """CDC-ACM class requests: SET_LINE_CODING, GET_LINE_CODING (115200 8N1), SET_CONTROL_LINE_STATE."""
    descriptors.add_class_request(0x21, 0x20)
    descriptors.add_class_request(0xa1, 0x21, le32(115200) + [0, 0, 8])
    descriptors.add_class_request(0x21, 0x22)

def cdc_acm_descriptors(notif_ept=3, in_ept=1, out_ept=2, max_burst=8, **kwargs):
    """CDC-ACM device."""
    kwargs.setdefault("device_class", DEVICE_CLASS_IAD)
    descriptors = USB3Descriptors(interfaces=cdc_acm_interfaces(0, notif_ept, in_ept, out_ept, max_burst), **kwargs)
    add_cdc_acm_requests(descriptors)
    return descriptors
