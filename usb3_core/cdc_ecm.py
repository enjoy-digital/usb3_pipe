#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-ECM (Ethernet Control Model, CDC ECM 1.2) function: a USB network adapter (Linux
cdc_ether, macOS), Ethernet frames on 32-bit user streams (ex: bridged to an Ethernet MAC).

- Communication Interface (IAD): Header/Union/Ethernet Networking functional descriptors (MAC
  address string, 1514-byte segments), notification Interrupt IN endpoint: NETWORK_CONNECTION and
  CONNECTION_SPEED_CHANGE when configured and on link changes.
- Data Interface: alternate setting 0 without endpoints, alternate setting 1 with Bulk IN/OUT. One
  Ethernet frame (without FCS) per transfer, ended by a short packet or a zero-length packet.
- Requests: SET_ETHERNET_PACKET_FILTER (accepted, no filtering).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import endpoint_layout
from usb3_core.endpoints   import MAX_PACKET_SIZE
from usb3_core.descriptors import *

# Constants ----------------------------------------------------------------------------------------

CDC_SET_ETHERNET_PACKET_FILTER = 0x43

CDC_NOTIFY_NETWORK_CONNECTION = 0x00
CDC_NOTIFY_SPEED_CHANGE       = 0x2a

ECM_MAX_SEGMENT_SIZE = 1514

# Descriptors --------------------------------------------------------------------------------------

def cdc_ecm_interfaces(comm_interface=0, notif_ept=3, in_ept=1, out_ept=2, mac_string=4,
    max_burst=8):
    """CDC-ECM function: IAD, Communication Interface (notification Interrupt IN endpoint) and Data
    Interface (comm_interface + 1: alternate setting 0 without endpoints, 1 with Bulk IN/OUT).
    mac_string: index of the MAC address string (12 hexadecimal digits, see add_cdc_ecm_strings)."""
    data_interface = comm_interface + 1
    iad = [8, DESC_IAD, comm_interface, 2, 0x02, 0x06, 0x00, 0]
    cs  = []
    cs += [5, DESC_CS_INTERFACE, 0x00] + le16(0x0110)                  # Header.
    cs += [5, DESC_CS_INTERFACE, 0x06, comm_interface, data_interface] # Union.
    cs += [13, DESC_CS_INTERFACE, 0x0f, mac_string] + le32(0) + \
        le16(ECM_MAX_SEGMENT_SIZE) + le16(0) + [0]                     # Ethernet Networking.
    comm = USB3InterfaceDescriptor(comm_interface, cls=0x02, subclass=0x06, protocol=0x00, extra=cs,
        endpoints=[USB3EndpointDescriptor(notif_ept, direction=1, kind=EP_INTERRUPT, max_packet=16,
            interval=9)])
    data_alt0 = USB3InterfaceDescriptor(data_interface, cls=0x0a, alt=0)
    data_alt1 = USB3InterfaceDescriptor(data_interface, cls=0x0a, alt=1, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    return [iad, comm, data_alt0, data_alt1]

def add_cdc_ecm_strings(descriptors, mac_address):
    """MAC address string (12 hexadecimal digits) added to the descriptors: returns its index."""
    descriptors.strings.append("{:012X}".format(mac_address))
    return len(descriptors.strings)

def add_cdc_ecm_requests(descriptors):
    """CDC-ECM class requests: SET_ETHERNET_PACKET_FILTER."""
    descriptors.add_class_request(0x21, CDC_SET_ETHERNET_PACKET_FILTER)

# Helpers ------------------------------------------------------------------------------------------

def be_bytes(be):
    """Valid bytes of a contiguous 4-bit byte enable."""
    return Mux(be[3], 4, Mux(be[2], 3, Mux(be[1], 2, Mux(be[0], 1, 0))))

# Bulk Transfers ---------------------------------------------------------------------------------

class USB3BulkTransferRX(LiteXModule):
    """Bulk OUT transfers (32-bit user side, with_zlp endpoint) -> stream of transfers: a transfer
    ends with a short packet or a zero-length packet (marker word, be=0). One word held: a transfer
    ending with a max size packet is only ended by the following zero-length packet."""
    def __init__(self, bulk_out):
        self.source = source = stream.Endpoint(endpoint_layout(32)) # o

        # # #

        rx        = bulk_out.source
        rx_bytes  = Signal(max=MAX_PACKET_SIZE + 1) # Bytes of the current packet.
        rx_n      = Signal(3)
        rx_zlp    = Signal()
        rx_end    = Signal() # Word ending a transfer (last of a short packet).
        held      = Signal()
        held_data = Signal(32)
        held_be   = Signal(4)
        held_end  = Signal()
        self.comb += [
            rx_n.eq(be_bytes(rx.be)),
            rx_zlp.eq(rx.be == 0),
            rx_end.eq(rx.last & ((rx_bytes + rx_n) < MAX_PACKET_SIZE)),
            source.valid.eq(held & (held_end | rx.valid)),
            source.data.eq(held_data),
            source.be.eq(held_be),
            source.last.eq(held_end | (rx.valid & rx_zlp)),
            rx.ready.eq(~held | (source.valid & source.ready)),
        ]
        self.sync += [
            If(source.valid & source.ready,
                held.eq(0)
            ),
            If(rx.valid & rx.ready,
                If(rx_zlp | rx.last,
                    rx_bytes.eq(0)
                ).Else(
                    rx_bytes.eq(rx_bytes + rx_n)
                ),
                If(~rx_zlp,
                    held.eq(1),
                    held_data.eq(rx.data),
                    held_be.eq(rx.be),
                    held_end.eq(rx_end),
                )
            ),
        ]

class USB3BulkTransferTX(LiteXModule):
    """Stream of transfers -> Bulk IN (32-bit user side): one transfer per stream packet, zero-length
    packet after a transfer of a multiple of the max packet size."""
    def __init__(self, bulk_in):
        self.sink = sink = stream.Endpoint(endpoint_layout(32)) # i

        # # #

        tx        = bulk_in.sink
        tx_bytes  = Signal(max=MAX_PACKET_SIZE) # Transfer bytes, modulo the max packet size.
        tx_next   = Signal(max=2*MAX_PACKET_SIZE)
        zlp       = Signal()
        self.comb += [
            tx_next.eq(tx_bytes + be_bytes(sink.be)),
            If(zlp,
                tx.valid.eq(1),
                tx.be.eq(0),
                tx.last.eq(1),
            ).Else(
                sink.connect(tx),
            ),
        ]
        self.sync += [
            If(zlp,
                If(tx.ready, zlp.eq(0))
            ).Elif(sink.valid & sink.ready,
                tx_bytes.eq(tx_next[:log2_int(MAX_PACKET_SIZE)]),
                If(sink.last,
                    tx_bytes.eq(0),
                    zlp.eq(tx_next[:log2_int(MAX_PACKET_SIZE)] == 0),
                )
            ),
        ]

# CDC Notifications --------------------------------------------------------------------------------

class CDCNetworkNotifications(LiteXModule):
    """NETWORK_CONNECTION + CONNECTION_SPEED_CHANGE notifications (CDC ECM/NCM), when configured and
    on link changes."""
    def __init__(self, ep0, notif_in, link, comm_interface=0, speed=1_000_000_000):
        def word(b):
            return int.from_bytes(bytes(b), "little")
        speed_bytes = list(speed.to_bytes(4, "little"))
        notification = Array([
            Cat(C(0xa1, 8), C(CDC_NOTIFY_NETWORK_CONNECTION, 8), link, C(0, 15)),
            C(word([comm_interface, 0, 0, 0]), 32),
            C(word([0xa1, CDC_NOTIFY_SPEED_CHANGE, 0, 0]), 32),
            C(word([comm_interface, 0, 8, 0]), 32),
            C(word(speed_bytes), 32),
            C(word(speed_bytes), 32),
        ])
        notify  = Signal()
        link_d  = Signal()
        conf_d  = Signal()
        index   = Signal(3)
        self.sync += [
            link_d.eq(link),
            conf_d.eq(ep0.configured),
            If((ep0.configured & ~conf_d) | (ep0.configured & (link != link_d)),
                notify.eq(1),
                index.eq(0),
            ).Elif(notify & notif_in.sink.ready,
                index.eq(index + 1),
                If(index == 5, notify.eq(0)),
            ),
            If(~ep0.configured, notify.eq(0)),
        ]
        self.comb += [
            notif_in.sink.valid.eq(notify),
            notif_in.sink.data.eq(notification[index]),
            notif_in.sink.be.eq(0b1111),
            notif_in.sink.last.eq((index == 1) | (index == 5)), # 8-byte, then 16-byte notification.
        ]

# USB3 CDC-ECM Function ----------------------------------------------------------------------------

class USB3CDCECMFunction(LiteXModule):
    """CDC-ECM function of a USB3 device (Bulk IN/OUT data endpoints: 32-bit user sides, Bulk OUT
    with_zlp; notification endpoint): one Ethernet frame per transfer.

    sink  : frames to the host (data: 32-bit words, first byte on data[0:8], be: valid bytes, last:
            end of frame).
    source: frames from the host (same layout).
    link  : network connection status (notified to the host).
    """
    def __init__(self, ep0, bulk_in, bulk_out, notif_in, comm_interface=0, speed=1_000_000_000):
        self.sink   = sink   = stream.Endpoint(endpoint_layout(32)) # i
        self.source = source = stream.Endpoint(endpoint_layout(32)) # o
        self.link   = Signal(reset=1)                               # i

        # # #

        self.rx     = rx = USB3BulkTransferRX(bulk_out)
        self.tx     = tx = USB3BulkTransferTX(bulk_in)
        self.notify = CDCNetworkNotifications(ep0, notif_in, self.link, comm_interface, speed)
        self.comb += [
            rx.source.connect(source),
            sink.connect(tx.sink),
        ]
