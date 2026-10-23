#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Control Endpoint (EP0, USB 3.2 8.12.2/chapter 9).

Setup stage (SETUP DP, acknowledged), optional Data stage (IN: responses from a ROM built from the
descriptors/requests table; OUT: data acknowledged and ignored), Status stage (STATUS TP,
acknowledged, SET_ADDRESS/SET_CONFIGURATION/CLEAR_FEATURE(ENDPOINT_HALT) applied). Unsupported
requests are stalled (STALL TP).
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

KINDS = {
    "in"                : KIND_IN,
    "out"               : KIND_OUT,
    "set_address"       : KIND_SET_ADDRESS,
    "set_configuration" : KIND_SET_CONFIGURATION,
    "clear_feature"     : KIND_CLEAR_FEATURE,
}

# USB3 Control Endpoint ----------------------------------------------------------------------------

class USB3EndpointControl(USB3Endpoint):
    def __init__(self, descriptors):
        USB3Endpoint.__init__(self, ept=0, direction=0)
        self.rx_dpp_setup = Signal()  # i: Received DPP is a SETUP DP.
        self.dev_addr     = Signal(7) # o: Device Address (SET_ADDRESS).
        self.configured   = Signal()  # o: Configured (SET_CONFIGURATION != 0).
        self.reset_seqs   = Signal()  # o: SET_CONFIGURATION: endpoints Sequence Numbers reset (pulse).
        self.clear_halt   = Signal()  # o: CLEAR_FEATURE(ENDPOINT_HALT) (pulse)...
        self.clear_ept    = Signal(4) # o: ...on this endpoint...
        self.clear_dir    = Signal()  # o: ...and direction.

        # # #

        # Requests Table / ROM ---------------------------------------------------------------------
        requests = descriptors.get_requests()
        rom      = []
        entries  = []
        for bmRequestType, bRequest, wValue, response, kind in requests:
            offset = len(rom)
            length = 0
            if response is not None:
                data    = list(response) + [0]*((-len(response)) % 4)
                rom    += [int.from_bytes(bytes(data[i:i+4]), "little") for i in range(0, len(data), 4)]
                length  = len(response)
            entries.append((bmRequestType, bRequest, wValue, offset, length, KINDS[kind]))
        self.rom_data = rom
        mem     = Memory(32, max(len(rom), 1), init=rom)
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
        offset = Signal(max=max(len(rom), 2))
        length = Signal(16)
        match_kind   = Signal(3)
        match_offset = Signal(max=max(len(rom), 2))
        match_length = Signal(16)
        cases  = None
        for _bmRequestType, _bRequest, _wValue, e_offset, e_length, e_kind in reversed(entries):
            match = (bmRequestType == _bmRequestType) & (bRequest == _bRequest)
            if _wValue is not None:
                match = match & (wValue == _wValue)
            action = [match_kind.eq(e_kind), match_offset.eq(e_offset), match_length.eq(e_length)]
            cases = If(match, *action) if cases is None else If(match, *action).Else(cases)
        self.comb += [match_kind.eq(KIND_STALL), cases]

        total = Signal(16)
        self.sync += [
            kind.eq(match_kind),
            offset.eq(match_offset),
            length.eq(match_length),
            total.eq(Mux(wLength < match_length, wLength, match_length)),
        ]
        stall = Signal()
        self.comb += stall.eq(kind == KIND_STALL)

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
            )
        ]
        self.comb += [
            self.reset_seqs.eq(self.hdr_strobe & status_tp & (kind == KIND_SET_CONFIGURATION)),
            self.clear_halt.eq(self.hdr_strobe & status_tp & (kind == KIND_CLEAR_FEATURE) & (wValue == 0)),
            self.clear_ept.eq(wIndex[0:4]),
            self.clear_dir.eq(wIndex[7]),
        ]

        # Data stage IN ----------------------------------------------------------------------------
        send_pending = Signal()
        send_seq     = Signal(5)
        send_offset  = Signal(16) # Bytes.
        send_len     = Signal(11)
        req_offset   = Signal(16)
        self.comb += req_offset.eq(h.tp_seq*EP0_MAX_PACKET)
        self.sync += [
            If(self.hdr_strobe & setup_dph, send_pending.eq(0)),
            If(self.hdr_strobe & in_ack & (h.nump != 0) & (kind == KIND_IN) &
                ((req_offset < total) | ((total == 0) & (h.tp_seq == 0))),
                send_pending.eq(1),
                send_seq.eq(h.tp_seq),
                send_offset.eq(req_offset),
                If(total - req_offset > EP0_MAX_PACKET,
                    send_len.eq(EP0_MAX_PACKET)
                ).Else(
                    send_len.eq(total - req_offset)
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
            jobs.sink.adr.eq(offset + send_offset[2:]),
            jobs.sink.length.eq(send_len),
        ]
        self.sync += If(self.tx_hp.valid & self.tx_hp.ready, send_pending.eq(0))

        # ROM Reader.
        self.reader = reader = DPPReader(rd_port, depth=8)
        self.comb += [
            jobs.source.connect(reader.sink),
            reader.source.connect(self.tx_dpp),
        ]

        # Data stage OUT: DPPs landed (and ignored).
        self.comb += [
            self.rx_dpp.ready.eq(1),
            self.landed.eq(self.rx_dpp_done),
        ]
