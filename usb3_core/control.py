#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Control Endpoint (EP0, USB 3.2 8.12.2/chapter 9).

Setup stage (SETUP DP, acknowledged), optional Data stage (IN: responses from a ROM built from the
descriptors/requests table; OUT: data acknowledged and ignored), Status stage (STATUS TP,
acknowledged, SET_ADDRESS/SET_CONFIGURATION/CLEAR_FEATURE(ENDPOINT_HALT)/SET_FEATURE and
CLEAR_FEATURE(U1_ENABLE/U2_ENABLE) applied). GET_STATUS(Device) reports the U1/U2 Enable bits.
Unsupported requests are stalled (STALL TP).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common    import *
from usb3_core.endpoints import USB3Endpoint, DPPReader

# Constants ----------------------------------------------------------------------------------------

EP0_MAX_PACKET = 512

KIND_STALL             = 0
KIND_IN                = 1
KIND_OUT               = 2
KIND_SET_ADDRESS       = 3
KIND_SET_CONFIGURATION = 4
KIND_CLEAR_FEATURE     = 5
KIND_HANDLER           = 6 # Delegated to the user request handler.
KIND_DEVICE_FEATURE    = 7 # SET/CLEAR_FEATURE(U1_ENABLE/U2_ENABLE) (Device).

USB_REQ_CLEAR_FEATURE = 0x01
USB_REQ_SET_FEATURE   = 0x03

FEATURE_U1_ENABLE = 48
FEATURE_U2_ENABLE = 49

EP0_HANDLER_WORDS = 128 # Request handler IN response buffer (512 bytes).

KINDS = {
    "in"                : KIND_IN,
    "out"               : KIND_OUT,
    "set_address"       : KIND_SET_ADDRESS,
    "set_configuration" : KIND_SET_CONFIGURATION,
    "clear_feature"     : KIND_CLEAR_FEATURE,
    "device_feature"    : KIND_DEVICE_FEATURE,
    "device_status"     : KIND_IN,
}

# Memory Read Port Proxy ---------------------------------------------------------------------------

class _ReadPortProxy:
    """Memory read port interface (adr/re/dat_r), to modify the data read by a DPPReader."""
    def __init__(self, port):
        self.adr   = Signal(len(port.adr))
        self.re    = Signal()
        self.dat_r = Signal(len(port.dat_r))

# USB3 Control Endpoint ----------------------------------------------------------------------------

class USB3EndpointControl(USB3Endpoint):
    """Control Endpoint.

    with_handler: class/vendor requests not in the requests table are delegated to a user request
    handler (UVC/MSC/CDC controls...):
    - req_setup (pulse)/req_valid (level, until the next request): request fields (req_type,
      req_request, req_value, req_index, req_length).
    - req_stall: the handler stalls the request (to set within a few cycles after req_setup).
    - req_in   : IN Data stage response (32-bit words, first byte on data[7:0], last/be), up to
                 512 bytes, written by the handler after req_setup (sent when complete).
    - req_out  : OUT Data stage data (32-bit words, last/be), no back-pressure.
    - req_done : Status stage completed (pulse), for all requests (ex: SET_INTERFACE tracking).
    """
    def __init__(self, descriptors, with_handler=False):
        USB3Endpoint.__init__(self, ept=0, direction=0)
        self.rx_dpp_setup = Signal()  # i: Received DPP is a SETUP DP.
        self.dev_addr     = Signal(7) # o: Device Address (SET_ADDRESS).
        self.configured   = Signal()  # o: Configured (SET_CONFIGURATION != 0).
        self.reset_seqs   = Signal()  # o: SET_CONFIGURATION: endpoints Sequence Numbers reset (pulse).
        self.clear_halt   = Signal()  # o: CLEAR_FEATURE(ENDPOINT_HALT) (pulse)...
        self.clear_ept    = Signal(4) # o: ...on this endpoint...
        self.clear_dir    = Signal()  # o: ...and direction.
        self.u1_enable    = Signal()  # o: U1 Enable (SET/CLEAR_FEATURE(U1_ENABLE), 9.4.5).
        self.u2_enable    = Signal()  # o: U2 Enable (SET/CLEAR_FEATURE(U2_ENABLE), 9.4.5).

        # Request handler interface.
        self.req_setup   = Signal()   # o
        self.req_valid   = Signal()   # o
        self.req_type    = Signal(8)  # o
        self.req_request = Signal(8)  # o
        self.req_value   = Signal(16) # o
        self.req_index   = Signal(16) # o
        self.req_length  = Signal(16) # o
        self.req_stall   = Signal()   # i
        self.req_done    = Signal()   # o
        self.req_in      = stream.Endpoint([("data", 32), ("be", 4)]) # i
        self.req_out     = stream.Endpoint([("data", 32), ("be", 4)]) # o

        # # #

        # Requests Table / ROM ---------------------------------------------------------------------
        requests = descriptors.get_requests()
        rom      = []
        entries  = []
        status_offset = None # GET_STATUS(Device) response word.
        for bmRequestType, bRequest, wValue, response, kind in requests:
            offset = len(rom)
            if kind == "device_status":
                status_offset = offset
            length = 0
            if response is not None:
                data    = list(response) + [0]*((-len(response)) % 4)
                rom    += [int.from_bytes(bytes(data[i:i+4]), "little") for i in range(0, len(data), 4)]
                length  = len(response)
            entries.append((bmRequestType, bRequest, wValue, offset, length, KINDS[kind]))
        self.rom_data = rom
        # ROM (descriptors/constant responses) followed by the request handler response buffer.
        handler_base = len(rom)
        mem_depth    = len(rom) + (EP0_HANDLER_WORDS if with_handler else 0)
        mem     = Memory(32, max(mem_depth, 1), init=rom)
        rd_port = mem.get_port(has_re=True)
        self.specials += mem, rd_port

        # Setup Packet -----------------------------------------------------------------------------
        setup_w0 = Signal(32)
        setup_w1 = Signal(32)
        setup_count = Signal(2)
        self.sync += If(self.rx_dpp.valid & self.rx_dpp_setup,
            If(self.rx_dpp.first,
                setup_w0.eq(self.rx_dpp.data),
                setup_count.eq(1),
            ).Elif(setup_count == 1,
                setup_w1.eq(self.rx_dpp.data),
                setup_count.eq(2),
            )
        )
        bmRequestType = setup_w0[0:8]
        bRequest      = setup_w0[8:16]
        wValue        = setup_w0[16:32]
        wIndex        = setup_w1[0:16]
        wLength       = setup_w1[16:32]

        # Request matching (first match, registered: the request is decoded when the SETUP lands and
        # used from its Data/Status stages).
        kind   = Signal(3)
        offset = Signal(max=max(mem_depth, 2), reset_less=True)
        length = Signal(16, reset_less=True)
        match_kind   = Signal(3)
        match_offset = Signal(max=max(mem_depth, 2))
        match_length = Signal(16)
        cases  = None
        for _bmRequestType, _bRequest, _wValue, e_offset, e_length, e_kind in reversed(entries):
            match = (bmRequestType == _bmRequestType) & (bRequest == _bRequest)
            if _wValue is not None:
                match = match & (wValue == _wValue)
            action = [match_kind.eq(e_kind), match_offset.eq(e_offset), match_length.eq(e_length)]
            cases = If(match, *action) if cases is None else If(match, *action).Else(cases)
        default = [match_kind.eq(KIND_STALL)]
        if with_handler:
            # Class/vendor requests not in the table: request handler.
            default = [If(bmRequestType[5:7] != 0, match_kind.eq(KIND_HANDLER)).Else(match_kind.eq(KIND_STALL))]
        self.comb += [*default, cases]

        total = Signal(16, reset_less=True)
        self.sync += [
            kind.eq(match_kind),
            offset.eq(match_offset),
            length.eq(match_length),
            total.eq(Mux(wLength < match_length, wLength, match_length)),
        ]
        stall = Signal()
        self.comb += stall.eq((kind == KIND_STALL) | ((kind == KIND_HANDLER) & self.req_stall))

        # Headers ----------------------------------------------------------------------------------
        h = self.hdr
        setup_dph = Signal()
        data_dph  = Signal()
        status_tp = Signal()
        in_ack    = Signal()
        self.comb += [
            setup_dph.eq((h.type == TYPE_DP) & (h.ept == 0) & (h.setup == 1)),
            data_dph.eq( (h.type == TYPE_DP) & (h.ept == 0) & (h.setup == 0)),
            status_tp.eq((h.type == TYPE_TP) & (h.ept == 0) & (h.subtype == TP_SUB_STATUS)),
            # Host ACK TPs on EP0 only request/acknowledge IN data (Direction not checked).
            in_ack.eq(   (h.type == TYPE_TP) & (h.ept == 0) & (h.subtype == TP_SUB_ACK)),
        ]
        ok = Signal()
        self.comb += ok.eq(self.dpp_ok)
        stall_dw1 = tp_dw1(TP_SUB_STALL, ept=0, direction=0)
        self.comb += [
            If(setup_dph,
                self.resp_valid.eq(1),
                self.resp_dw1.eq(tp_dw1(TP_SUB_ACK, ept=0, direction=0, nump=1,
                    seq=Mux(ok, 1, 0), retry=~ok)),
            ).Elif(data_dph,
                self.resp_valid.eq(1),
                If(stall,
                    self.resp_dw1.eq(stall_dw1)
                ).Else(
                    self.resp_dw1.eq(tp_dw1(TP_SUB_ACK, ept=0, direction=0, nump=1,
                        seq=Mux(ok, h.dp_seq + 1, h.dp_seq), retry=~ok))
                )
            ).Elif(status_tp,
                self.resp_valid.eq(1),
                If(stall,
                    self.resp_dw1.eq(stall_dw1)
                ).Else(
                    self.resp_dw1.eq(tp_dw1(TP_SUB_ACK, ept=0, direction=0, nump=0, seq=1))
                )
            ).Elif(in_ack & stall & (h.nump != 0),
                self.resp_valid.eq(1),
                self.resp_dw1.eq(stall_dw1),
            )
        ]

        # Status stage: requests effects.
        self.sync += [
            If(self.hdr_strobe & status_tp & ~stall,
                If(kind == KIND_SET_ADDRESS,
                    self.dev_addr.eq(wValue[0:7])
                ),
                If(kind == KIND_SET_CONFIGURATION,
                    self.configured.eq(wValue != 0)
                ),
                If(kind == KIND_DEVICE_FEATURE,
                    If(wValue == FEATURE_U1_ENABLE,
                        self.u1_enable.eq(bRequest == USB_REQ_SET_FEATURE)
                    ),
                    If(wValue == FEATURE_U2_ENABLE,
                        self.u2_enable.eq(bRequest == USB_REQ_SET_FEATURE)
                    ),
                ),
            )
        ]
        self.comb += [
            self.reset_seqs.eq(self.hdr_strobe & status_tp & (kind == KIND_SET_CONFIGURATION)),
            self.clear_halt.eq(self.hdr_strobe & status_tp & (kind == KIND_CLEAR_FEATURE) & (wValue == 0)),
            self.clear_ept.eq(wIndex[0:4]),
            self.clear_dir.eq(wIndex[7]),
        ]

        # Request handler ----------------------------------------------------------------------------
        new_setup = Signal()
        self.comb += new_setup.eq(self.hdr_strobe & setup_dph & ok)
        resp_ready = Signal()                    # Handler IN response complete.
        resp_len   = Signal(11, reset_less=True) # Handler IN response length (bytes).
        resp_words = Signal(max=EP0_HANDLER_WORDS + 1)
        if with_handler:
            wr_port = mem.get_port(write_capable=True)
            self.specials += wr_port
            setup_d = Signal()
            self.sync += [
                setup_d.eq(new_setup), # kind registered one cycle after the SETUP.
                self.req_setup.eq(setup_d & (kind == KIND_HANDLER)),
                If(new_setup,
                    self.req_valid.eq(0),
                ).Elif(setup_d,
                    self.req_valid.eq(kind == KIND_HANDLER),
                ),
            ]
            self.comb += [
                self.req_type.eq(bmRequestType),
                self.req_request.eq(bRequest),
                self.req_value.eq(wValue),
                self.req_index.eq(wIndex),
                self.req_length.eq(wLength),
                self.req_in.ready.eq(self.req_valid & ~resp_ready & (resp_words != EP0_HANDLER_WORDS)),
                wr_port.adr.eq(handler_base + resp_words),
                wr_port.dat_w.eq(self.req_in.data),
                wr_port.we.eq(self.req_in.valid & self.req_in.ready),
            ]
            self.sync += [
                If(new_setup,
                    resp_ready.eq(0),
                    resp_words.eq(0),
                ).Elif(self.req_in.valid & self.req_in.ready,
                    resp_words.eq(resp_words + 1),
                    If(self.req_in.last,
                        resp_ready.eq(1),
                        resp_len.eq(4*resp_words + be_to_bytes(self.req_in.be)),
                    )
                )
            ]
            # OUT Data stage: forwarded.
            self.comb += [
                self.req_out.valid.eq(self.rx_dpp.valid & ~self.rx_dpp_setup & (kind == KIND_HANDLER)),
                self.req_out.data.eq(self.rx_dpp.data),
                self.req_out.last.eq(self.rx_dpp.last),
                self.req_out.be.eq(Mux(self.rx_dpp.last, bytes_to_be(self.rx_dpp.bytes), 0b1111)),
            ]
        self.comb += self.req_done.eq(self.hdr_strobe & status_tp & ~stall)

        # Data stage IN ----------------------------------------------------------------------------
        # IN data available: from the ROM (table responses) or the handler response (once complete).
        in_total = Signal(16)
        in_ready = Signal()
        self.comb += [
            If(kind == KIND_HANDLER,
                in_ready.eq(resp_ready),
                in_total.eq(Mux(wLength < resp_len, wLength, resp_len)),
            ).Else(
                in_ready.eq(kind == KIND_IN),
                in_total.eq(total),
            )
        ]
        in_req       = Signal()                   # Host request (ACK TP with NumP) pending...
        in_req_seq   = Signal(5, reset_less=True)  # ...for this Sequence Number.
        send_pending = Signal()
        send_seq     = Signal(5, reset_less=True)
        send_offset  = Signal(16, reset_less=True) # Bytes.
        send_len     = Signal(11, reset_less=True)
        req_offset   = Signal(16)
        self.comb += req_offset.eq(in_req_seq*EP0_MAX_PACKET)
        self.sync += [
            If(self.hdr_strobe & setup_dph,
                in_req.eq(0),
                send_pending.eq(0),
            ),
            If(self.hdr_strobe & in_ack & (h.nump != 0) & ~stall,
                in_req.eq(1),
                in_req_seq.eq(h.tp_seq),
            ),
            If(in_req & in_ready & ~send_pending,
                in_req.eq(0),
                If((req_offset < in_total) | ((in_total == 0) & (in_req_seq == 0)),
                    send_pending.eq(1),
                    send_seq.eq(in_req_seq),
                    send_offset.eq(req_offset),
                    If(in_total - req_offset > EP0_MAX_PACKET,
                        send_len.eq(EP0_MAX_PACKET)
                    ).Else(
                        send_len.eq(in_total - req_offset)
                    )
                )
            )
        ]
        self.jobs = jobs = stream.SyncFIFO([("adr", len(offset)), ("length", 11)], 2)
        self.comb += [
            self.tx_hp.valid.eq(send_pending & jobs.sink.ready),
            self.tx_hp.dw0.eq(TYPE_DP),
            self.tx_hp.dw1.eq(dph_dw1(seq=send_seq, ept=0, direction=1, length=send_len)),
            self.tx_hp.dpp.eq(1),
            jobs.sink.valid.eq(self.tx_hp.valid & self.tx_hp.ready),
            jobs.sink.adr.eq(Mux(kind == KIND_HANDLER, handler_base, offset) + send_offset[2:]),
            jobs.sink.length.eq(send_len),
        ]
        self.sync += If(self.tx_hp.valid & self.tx_hp.ready, send_pending.eq(0))

        # Data stage IN in progress: U1/U2 entry rejected.
        self.comb += self.pm_busy.eq(in_req | send_pending | jobs.source.valid)

        # ROM Reader (GET_STATUS(Device) response: U1/U2 Enable bits D2/D3 inserted, 9.4.5).
        rd_proxy = _ReadPortProxy(rd_port)
        rd_adr   = Signal(len(rd_port.adr))
        self.sync += If(rd_proxy.re, rd_adr.eq(rd_proxy.adr))
        self.comb += [
            rd_port.adr.eq(rd_proxy.adr),
            rd_port.re.eq(rd_proxy.re),
            rd_proxy.dat_r.eq(rd_port.dat_r),
        ]
        if status_offset is not None:
            self.comb += If(rd_adr == status_offset,
                rd_proxy.dat_r.eq(rd_port.dat_r | Cat(C(0, 2), self.u1_enable, self.u2_enable))
            )
        self.reader = reader = DPPReader(rd_proxy, depth=8)
        self.comb += [
            jobs.source.connect(reader.sink),
            reader.source.connect(self.tx_dpp),
        ]

        # Data stage OUT: DPPs landed (and ignored).
        self.comb += [
            self.rx_dpp.ready.eq(1),
            self.landed.eq(self.rx_dpp_done),
        ]
