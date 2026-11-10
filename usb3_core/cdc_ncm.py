#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-NCM (Network Control Model, CDC NCM 1.0) function: a USB network adapter (Linux
cdc_ncm, Windows 10+ UsbNcm, macOS), Ethernet frames on 32-bit user streams (same interface as
USB3CDCECMFunction).

- Communication Interface (IAD): Header/Union/Ethernet Networking/NCM functional descriptors,
  notification Interrupt IN endpoint (NETWORK_CONNECTION, CONNECTION_SPEED_CHANGE).
- Data Interface (NTB protocol): alternate setting 0 without endpoints, 1 with Bulk IN/OUT, one NCM
  Transfer Block (NTB16, no CRC) per transfer.
- Host -> Device: NTB buffered (block RAM, ntb_out_max_size), NTH16/NDP16 walked, datagrams sent as
  frames (datagrams 4-byte aligned: wNdpOutDivisor 4, wNdpOutPayloadRemainder 2 after the Ethernet
  header).
- Device -> Host: each frame stored, then sent as one NTB (NTH16, NDP16 with one datagram).
- Requests: GET_NTB_PARAMETERS, GET/SET_NTB_FORMAT, GET/SET_NTB_INPUT_SIZE,
  SET_ETHERNET_PACKET_FILTER (accepted, no filtering).
"""

import struct

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import endpoint_layout, bytes_to_be
from usb3_core.endpoints   import MAX_PACKET_SIZE
from usb3_core.descriptors import *
from usb3_core.cdc_ecm     import ECM_MAX_SEGMENT_SIZE, CDC_SET_ETHERNET_PACKET_FILTER, be_bytes
from usb3_core.cdc_ecm     import USB3BulkTransferRX, USB3BulkTransferTX, CDCNetworkNotifications

# Constants ----------------------------------------------------------------------------------------

NCM_GET_NTB_PARAMETERS = 0x80
NCM_GET_NTB_FORMAT     = 0x83
NCM_SET_NTB_FORMAT     = 0x84
NCM_GET_NTB_INPUT_SIZE = 0x85
NCM_SET_NTB_INPUT_SIZE = 0x86

NTH16_SIGNATURE = 0x484d434e # "NCMH".
NDP16_SIGNATURE = 0x304d434e # "NCM0" (no CRC).
NTH16_LENGTH    = 12
NDP16_LENGTH    = 16         # Header (8) + one datagram entry (4) + terminator (4).

NCM_NTB_IN_MAX_SIZE  = 16384
NCM_NTB_OUT_MAX_SIZE = 16384

# Descriptors --------------------------------------------------------------------------------------

def cdc_ncm_interfaces(comm_interface=0, notif_ept=3, in_ept=1, out_ept=2, mac_string=4,
    max_burst=8):
    """CDC-NCM function: IAD, Communication Interface (notification Interrupt IN endpoint) and Data
    Interface (comm_interface + 1, NTB protocol: alternate setting 0 without endpoints, 1 with Bulk
    IN/OUT). mac_string: index of the MAC address string (see add_cdc_ecm_strings)."""
    data_interface = comm_interface + 1
    iad = [8, DESC_IAD, comm_interface, 2, 0x02, 0x0d, 0x00, 0]
    cs  = []
    cs += [5, DESC_CS_INTERFACE, 0x00] + le16(0x0110)                  # Header.
    cs += [5, DESC_CS_INTERFACE, 0x06, comm_interface, data_interface] # Union.
    cs += [13, DESC_CS_INTERFACE, 0x0f, mac_string] + le32(0) + \
        le16(ECM_MAX_SEGMENT_SIZE) + le16(0) + [0]                     # Ethernet Networking.
    cs += [6, DESC_CS_INTERFACE, 0x1a] + le16(0x0100) + [0x00]          # NCM (no capabilities).
    comm = USB3InterfaceDescriptor(comm_interface, cls=0x02, subclass=0x0d, protocol=0x00, extra=cs,
        endpoints=[USB3EndpointDescriptor(notif_ept, direction=1, kind=EP_INTERRUPT, max_packet=16,
            interval=9)])
    data_alt0 = USB3InterfaceDescriptor(data_interface, cls=0x0a, protocol=0x01, alt=0)
    data_alt1 = USB3InterfaceDescriptor(data_interface, cls=0x0a, protocol=0x01, alt=1, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    return [iad, comm, data_alt0, data_alt1]

def ncm_ntb_parameters(ntb_in_max_size=NCM_NTB_IN_MAX_SIZE, ntb_out_max_size=NCM_NTB_OUT_MAX_SIZE):
    """GET_NTB_PARAMETERS response (NTB16 only, 4-byte divisors/alignments). Payload remainders of 2:
    the payload after the 14-byte Ethernet header at 2 modulo 4, so the datagrams start 4-byte
    aligned (Linux cdc_ncm: datagram remainder = (wNdpOutPayloadRemainder - 14) & 3)."""
    return list(struct.pack("<HHIHHHHIHHHH",
        28,               # wLength.
        0x0001,           # bmNtbFormatsSupported: NTB16.
        ntb_in_max_size,  # dwNtbInMaxSize.
        4, 2, 4,          # wNdpInDivisor, wNdpInPayloadRemainder, wNdpInAlignment.
        0,                # Reserved.
        ntb_out_max_size, # dwNtbOutMaxSize.
        4, 2, 4,          # wNdpOutDivisor, wNdpOutPayloadRemainder, wNdpOutAlignment.
        0))               # wNtbOutMaxDatagrams: no limit.

def add_cdc_ncm_requests(descriptors, ntb_in_max_size=NCM_NTB_IN_MAX_SIZE,
    ntb_out_max_size=NCM_NTB_OUT_MAX_SIZE):
    """CDC-NCM class requests (constant responses)."""
    descriptors.add_class_request(0xa1, NCM_GET_NTB_PARAMETERS,
        ncm_ntb_parameters(ntb_in_max_size, ntb_out_max_size))
    descriptors.add_class_request(0xa1, NCM_GET_NTB_FORMAT,     [0, 0]) # NTB16.
    descriptors.add_class_request(0x21, NCM_SET_NTB_FORMAT)
    descriptors.add_class_request(0xa1, NCM_GET_NTB_INPUT_SIZE, le32(ntb_in_max_size))
    descriptors.add_class_request(0x21, NCM_SET_NTB_INPUT_SIZE)
    descriptors.add_class_request(0x21, CDC_SET_ETHERNET_PACKET_FILTER)

# NTB Transfers (Host -> Device) ------------------------------------------------------------------

class NCMTransferRX(LiteXModule):
    """Bulk OUT (32-bit user side, with_zlp endpoint) -> NTB transfers: a transfer ends with a short
    packet, a zero-length packet (marker word, be=0, sent with last) or at max_size bytes (no
    zero-length packet after an NTB of dwNtbOutMaxSize: Linux cdc_ncm pads its NTBs to it)."""
    def __init__(self, bulk_out, max_size=NCM_NTB_OUT_MAX_SIZE):
        self.source = source = stream.Endpoint(endpoint_layout(32)) # o

        # # #

        rx          = bulk_out.source
        n           = Signal(3)
        pkt_bytes   = Signal(max=MAX_PACKET_SIZE + 1)   # Bytes of the current packet.
        total_bytes = Signal(max=max_size + 1)          # Bytes of the current transfer.
        short       = Signal()
        full        = Signal()
        self.comb += [
            n.eq(be_bytes(rx.be)),
            short.eq(rx.last & ((pkt_bytes + n) < MAX_PACKET_SIZE)),
            full.eq((total_bytes + n) >= max_size),
            rx.connect(source, omit={"last"}),
            source.last.eq((rx.be == 0) | short | full),
        ]
        self.sync += If(rx.valid & rx.ready,
            pkt_bytes.eq(Mux(rx.last, 0, pkt_bytes + n)),
            total_bytes.eq(Mux(source.last, 0, total_bytes + n)),
        )

# NTB Parser (Host -> Device) ----------------------------------------------------------------------

class NCMNTBParser(LiteXModule):
    """NTB16 transfers (32-bit words) -> Ethernet frames: NTB buffered, NTH16/NDP16 walked, each
    datagram (4-byte aligned) sent as a frame. Invalid NTBs (signatures, too large) are dropped."""
    def __init__(self, max_size=NCM_NTB_OUT_MAX_SIZE):
        self.sink   = sink   = stream.Endpoint(endpoint_layout(32)) # i: NTB transfers.
        self.source = source = stream.Endpoint(endpoint_layout(32)) # o: frames.
        self.errors = Signal(16)                                    # o: dropped NTBs/datagrams.
        # Debug: last NTB words count, NTH16 words 0/2, NDP16 word 0, last drop reason (1: overflow,
        # 2: NTH16 signature, 3: NDP16 index, 4: NDP16 signature, 5: entry index, 6: datagram).
        self.dbg_words = Signal(16)
        self.dbg_nth0  = Signal(32)
        self.dbg_nth2  = Signal(32)
        self.dbg_ndp0  = Signal(32)
        self.dbg_drop  = Signal(3)

        # # #

        words = max_size//4
        mem   = Memory(32, words)
        wr    = mem.get_port(write_capable=True)
        rd    = mem.get_port(async_read=False) # Synchronous read (block RAM).
        self.specials += mem, wr, rd

        wr_adr   = Signal(max=words + 1)
        overflow = Signal()
        ndp      = Signal(16) # NDP16 byte index.
        entry    = Signal(16) # Datagram entry byte index.
        dg_index = Signal(16)
        dg_len   = Signal(16)
        dg_adr   = Signal(16) # Datagram word address.
        dg_left  = Signal(16) # Datagram bytes left.
        next_ndp = Signal(16)

        self.fsm = fsm = FSM(reset_state="RECEIVE")
        # Receive the NTB transfer into the buffer.
        self.comb += [
            wr.adr.eq(wr_adr),
            wr.dat_w.eq(sink.data),
        ]
        fsm.act("RECEIVE",
            sink.ready.eq(1),
            # Words (zero-length packet markers ignored).
            If(sink.valid & (sink.be != 0),
                If(wr_adr < words,
                    wr.we.eq(1),
                    NextValue(wr_adr, wr_adr + 1),
                ).Else(
                    NextValue(overflow, 1),
                ),
            ),
            If(sink.valid & sink.last,
                NextState("NTH-READ")
            )
        )
        def drop(reason):
            return [
                NextValue(self.errors, self.errors + 1),
                NextValue(self.dbg_drop, reason),
                NextState("DONE"),
            ]
        # NTH16: signature, wBlockLength/wNdpIndex (word 2).
        fsm.act("NTH-READ",
            rd.adr.eq(0),
            NextValue(self.dbg_words, wr_adr),
            If(overflow,
                *drop(1)
            ).Else(
                NextState("NTH-SIG")
            )
        )
        fsm.act("NTH-SIG",
            rd.adr.eq(2),
            NextValue(self.dbg_nth0, rd.dat_r),
            If(rd.dat_r != NTH16_SIGNATURE,
                *drop(2)
            ).Else(
                NextState("NTH-NDP")
            )
        )
        fsm.act("NTH-NDP",
            NextValue(self.dbg_nth2, rd.dat_r),
            NextValue(ndp, rd.dat_r[16:32]),
            NextState("NDP-READ")
        )
        # NDP16: signature, wNextNdpIndex (word 1), entries from ndp + 8.
        fsm.act("NDP-READ",
            rd.adr.eq(ndp[2:]),
            If((ndp[0:2] != 0) | (ndp == 0) | (ndp[2:] >= wr_adr),
                *drop(3)
            ).Else(
                NextState("NDP-SIG")
            )
        )
        fsm.act("NDP-SIG",
            rd.adr.eq(ndp[2:] + 1),
            NextValue(self.dbg_ndp0, rd.dat_r),
            If(rd.dat_r != NDP16_SIGNATURE,
                *drop(4)
            ).Else(
                NextState("NDP-NEXT")
            )
        )
        fsm.act("NDP-NEXT",
            NextValue(next_ndp, rd.dat_r[16:32]),
            NextValue(entry, ndp + 8),
            NextState("ENTRY-READ")
        )
        # Datagram entries: (wDatagramIndex, wDatagramLength), terminated by (0, 0).
        fsm.act("ENTRY-READ",
            rd.adr.eq(entry[2:]),
            If(entry[2:] >= wr_adr,
                *drop(5)
            ).Else(
                NextState("ENTRY")
            )
        )
        fsm.act("ENTRY",
            NextValue(dg_index, rd.dat_r[0:16]),
            NextValue(dg_len,   rd.dat_r[16:32]),
            NextValue(entry, entry + 4),
            If((rd.dat_r[0:16] == 0) | (rd.dat_r[16:32] == 0),
                # End of this NDP: next NDP or done.
                If(next_ndp != 0,
                    NextValue(ndp, next_ndp),
                    NextState("NDP-READ")
                ).Else(
                    NextState("DONE")
                )
            ).Else(
                NextState("DATAGRAM-CHECK")
            )
        )
        fsm.act("DATAGRAM-CHECK",
            NextValue(dg_adr,  dg_index[2:]),
            NextValue(dg_left, dg_len),
            If((dg_index[0:2] != 0) | (((dg_index + dg_len + 3) >> 2) > wr_adr),
                # Unaligned or out of the NTB: skipped.
                NextValue(self.errors, self.errors + 1),
                NextValue(self.dbg_drop, 6),
                NextState("ENTRY-READ")
            ).Else(
                NextState("DATAGRAM-READ")
            )
        )
        # Datagram words (one word per 2 cycles: read, then output).
        last = Signal()
        self.comb += last.eq(dg_left <= 4)
        fsm.act("DATAGRAM-READ",
            rd.adr.eq(dg_adr),
            NextState("DATAGRAM")
        )
        fsm.act("DATAGRAM",
            rd.adr.eq(dg_adr),
            source.valid.eq(1),
            source.data.eq(rd.dat_r),
            source.be.eq(Mux(last, bytes_to_be(dg_left[0:3]), 0b1111)),
            source.last.eq(last),
            If(source.ready,
                NextValue(dg_adr,  dg_adr + 1),
                NextValue(dg_left, dg_left - 4),
                If(last,
                    NextState("ENTRY-READ")
                ).Else(
                    NextState("DATAGRAM-READ")
                )
            )
        )
        fsm.act("DONE",
            NextValue(wr_adr, 0),
            NextValue(overflow, 0),
            NextState("RECEIVE")
        )

# NTB Builder (Device -> Host) ---------------------------------------------------------------------

class NCMNTBBuilder(LiteXModule):
    """Ethernet frames (32-bit words) -> NTB16 transfers: each frame stored (length), then sent as
    one NTB: NTH16, NDP16 (one datagram at byte 28, 4-byte aligned), datagram."""
    def __init__(self, fifo_depth=1024):
        self.sink   = sink   = stream.Endpoint(endpoint_layout(32)) # i: frames.
        self.source = source = stream.Endpoint(endpoint_layout(32)) # o: NTB transfers.

        # # #

        datagram = NTH16_LENGTH + NDP16_LENGTH # Datagram byte index (28).

        # Frames FIFO and lengths FIFO (frame stored before its NTB: wBlockLength in the NTH16).
        self.data_fifo   = data_fifo   = stream.SyncFIFO(endpoint_layout(32), fifo_depth, buffered=True)
        self.length_fifo = length_fifo = stream.SyncFIFO([("length", 16)], 4)
        length = Signal(16)
        self.comb += [
            sink.connect(data_fifo.sink, omit={"ready", "valid"}),
            data_fifo.sink.valid.eq(sink.valid & length_fifo.sink.ready),
            sink.ready.eq(data_fifo.sink.ready & length_fifo.sink.ready),
            length_fifo.sink.valid.eq(sink.valid & sink.ready & sink.last),
            length_fifo.sink.length.eq(length + be_bytes(sink.be)),
        ]
        self.sync += If(sink.valid & sink.ready,
            length.eq(Mux(sink.last, 0, length + be_bytes(sink.be)))
        )

        # NTB.
        seq    = Signal(16)
        flen   = Signal(16)
        blen   = Signal(16) # wBlockLength.
        index  = Signal(3)
        header = Array([
            C(NTH16_SIGNATURE, 32),
            Cat(C(NTH16_LENGTH, 16), seq),
            Cat(blen, C(NTH16_LENGTH, 16)),            # wBlockLength, wNdpIndex.
            C(NDP16_SIGNATURE, 32),
            C(NDP16_LENGTH, 32),                       # wLength, wNextNdpIndex (0).
            Cat(C(datagram, 16), flen),                # Datagram entry.
            C(0, 32),                                  # Terminator.
        ])
        self.comb += blen.eq(flen + datagram)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(length_fifo.source.valid,
                NextValue(flen, length_fifo.source.length),
                NextValue(index, 0),
                length_fifo.source.ready.eq(1),
                NextState("HEADER")
            )
        )
        fsm.act("HEADER",
            source.valid.eq(1),
            source.data.eq(header[index]),
            source.be.eq(0b1111),
            If(source.ready,
                NextValue(index, index + 1),
                If(index == 6,
                    NextState("DATAGRAM")
                )
            )
        )
        fsm.act("DATAGRAM",
            data_fifo.source.connect(source),
            If(data_fifo.source.valid & data_fifo.source.ready & data_fifo.source.last,
                NextValue(seq, seq + 1),
                NextState("IDLE")
            )
        )

# USB3 CDC-NCM Function ----------------------------------------------------------------------------

class USB3CDCNCMFunction(LiteXModule):
    """CDC-NCM function of a USB3 device (Bulk IN/OUT data endpoints: 32-bit user sides, Bulk OUT
    with_zlp; notification endpoint), same user interface as USB3CDCECMFunction.

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

        self.rx      = rx      = NCMTransferRX(bulk_out)
        self.parser  = parser  = NCMNTBParser()
        self.builder = builder = NCMNTBBuilder()
        self.tx      = tx      = USB3BulkTransferTX(bulk_in)
        self.notify  = CDCNetworkNotifications(ep0, notif_in, self.link, comm_interface, speed)
        self.comb += [
            rx.source.connect(parser.sink),
            parser.source.connect(source),
            sink.connect(builder.sink),
            builder.source.connect(tx.sink),
        ]
