#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Protocol Layer (USB 3.2 chapter 8, Device).

- RX: received Header Packets dispatched (in order) to the endpoints (broadcast + strobe), DPHs
  processed with the status of their Data Packet Payload, TP responses (ACK/STALL) queued, PING
  answered (PING_RESPONSE). Data Packet Payloads routed to the endpoints by Endpoint Number.
- TX: Header Packets arbitration (TP responses first, then endpoints by priority), Device Address
  inserted (Route String 0), Data Packet Payloads sent in the DPHs order (DPP jobs).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import *
from usb3_core.dpp    import DPPPacker, DPPRXAdapter

# Layouts ------------------------------------------------------------------------------------------

def dpp_status_layout():
    """Received DPP status (ok: received without error, landed: stored by its endpoint)."""
    return [("ok", 1), ("landed", 1)]

def tp_response_layout():
    """TP response (DWord 1, Device Address when generated)."""
    return [("dw1", 32), ("addr", 7)]

def dpp_job_layout(n_endpoints):
    """DPP to transmit (endpoint index, in the DPHs order)."""
    return [("ep", bits_for(n_endpoints))]

# USB3 Protocol Layer ------------------------------------------------------------------------------

class USB3ProtocolLayer(LiteXModule):
    def __init__(self, link, endpoints, dev_addr):
        self.pm_busy = Signal() # o: Pending Header Packets/DPPs or busy endpoints (U1/U2 rejected).

        # # #

        # DPP words width: Link Layer data_width (32/64-bit), 32-bit endpoints on a 64-bit Link
        # Layer through DPPPacker/DPPRXAdapter (Control Endpoint, ...: always landed DPPs).
        dw = getattr(link, "data_width", 32)
        adapted = {ep: ep.data_width != dw for ep in endpoints}
        for ep in endpoints:
            assert (ep.data_width == dw) or (ep.data_width == 32 and dw == 64)

        # Received Header Packets (registered: timings).
        self.rx_hp = rx_hp = stream.PipeValid(header_layout())
        self.comb += link.rx_hp.connect(rx_hp.sink)
        rx_hp = rx_hp.source
        h     = HeaderFields(rx_hp.dw0, rx_hp.dw1, rx_hp.dw2)

        # DPP status (one per accepted DPH, in order) ----------------------------------------------
        landed = Signal()
        self.comb += landed.eq(Reduce("OR", [
            (link.rx_dpp.ept == ep.ept) if adapted[ep] else ep.landed for ep in endpoints]))
        self.status = status = stream.SyncFIFO(dpp_status_layout(), 8)
        self.comb += [
            status.sink.valid.eq(link.rx_dpp_done),
            status.sink.ok.eq(link.rx_dpp_ok),
            status.sink.landed.eq(landed),
        ]

        # RX DPP routing ---------------------------------------------------------------------------
        for ep in endpoints:
            if ep.direction == 1 and ep.ept != 0:
                continue
            sel = Signal()
            self.comb += sel.eq(link.rx_dpp.ept == ep.ept)
            if adapted[ep]:
                rx_adapter = DPPRXAdapter()
                self.submodules += rx_adapter
                self.comb += [
                    rx_adapter.sink.valid.eq(link.rx_dpp.valid & sel),
                    rx_adapter.sink.first.eq(link.rx_dpp.first),
                    rx_adapter.sink.last.eq(link.rx_dpp.last),
                    rx_adapter.sink.data.eq(link.rx_dpp.data),
                    rx_adapter.sink.bytes.eq(link.rx_dpp.bytes),
                    rx_adapter.setup.eq(link.rx_dpp.setup),
                    rx_adapter.done.eq(link.rx_dpp_done & sel),
                    rx_adapter.ok.eq(link.rx_dpp_ok),
                    rx_adapter.source.connect(ep.rx_dpp),
                    ep.rx_dpp_done.eq(rx_adapter.source_done),
                    ep.rx_dpp_ok.eq(rx_adapter.source_ok),
                ]
                if ep.ept == 0:
                    self.comb += ep.rx_dpp_setup.eq(rx_adapter.source_setup)
                continue
            self.comb += [
                ep.rx_dpp.valid.eq(link.rx_dpp.valid & sel),
                ep.rx_dpp.first.eq(link.rx_dpp.first),
                ep.rx_dpp.last.eq(link.rx_dpp.last),
                ep.rx_dpp.data.eq(link.rx_dpp.data),
                ep.rx_dpp.bytes.eq(link.rx_dpp.bytes),
                ep.rx_dpp_done.eq(link.rx_dpp_done & sel),
                ep.rx_dpp_ok.eq(link.rx_dpp_ok),
            ]
            # SETUP DPs: Control Endpoint only.
            if ep.ept == 0:
                self.comb += ep.rx_dpp_setup.eq(link.rx_dpp.setup)
        self.comb += link.rx_dpp.ready.eq(1)

        # RX Header Packets dispatch ---------------------------------------------------------------
        # TP responses, with the Device Address when generated (STATUS ACK of a SET_ADDRESS: old
        # one).
        self.resp = resp = stream.SyncFIFO(tp_response_layout(), 4)
        is_dp    = Signal()
        is_ping  = Signal()
        resp_any = Signal()
        resp_dw1 = Signal(32)
        strobe   = Signal()
        self.comb += [
            is_dp.eq(h.type == TYPE_DP),
            is_ping.eq((h.type == TYPE_TP) & (h.subtype == TP_SUB_PING)),
            resp_any.eq(is_ping | Reduce("OR", [ep.resp_valid for ep in endpoints])),
            If(is_ping,
                resp_dw1.eq(tp_dw1(TP_SUB_PING_RESP, ept=h.ept, direction=h.direction))
            ).Else(
                resp_dw1.eq(Reduce("OR", [Mux(ep.resp_valid, ep.resp_dw1, 0) for ep in endpoints]))
            ),
            strobe.eq(rx_hp.valid & (~is_dp | status.source.valid) & (~resp_any | resp.sink.ready)),
            rx_hp.ready.eq(strobe),
            status.source.ready.eq(strobe & is_dp),
            resp.sink.valid.eq(strobe & resp_any),
            resp.sink.dw1.eq(resp_dw1),
            resp.sink.addr.eq(dev_addr),
        ]
        for ep in endpoints:
            self.comb += [
                ep.hdr_dw0.eq(rx_hp.dw0),
                ep.hdr_dw1.eq(rx_hp.dw1),
                ep.hdr_dw2.eq(rx_hp.dw2),
                ep.hdr_strobe.eq(strobe),
                ep.dpp_ok.eq(status.source.ok),
                ep.dpp_landed.eq(status.source.landed),
            ]

        # TX Header Packets/DPPs to the Link Layer (registered: timings).
        self.tx_hp  = tx_hp  = stream.Buffer(tx_header_layout(), pipe_valid=True, pipe_ready=True)
        self.tx_dpp = tx_dpp = stream.PipeValid(dpp_layout(dw))
        self.comb += [
            tx_hp.source.connect(link.tx_hp),
            tx_dpp.source.connect(link.tx_dpp),
        ]

        # TX Header Packets arbitration ------------------------------------------------------------
        self.jobs = jobs = stream.SyncFIFO(dpp_job_layout(len(endpoints)), 8)
        sources = [(None, resp.source)] + [(i, ep.tx_hp) for i, ep in enumerate(endpoints)]
        cases = None
        for i, source in reversed(sources):
            if i is None:
                grant = [
                    tx_hp.sink.valid.eq(1),
                    tx_hp.sink.dw0.eq(Cat(C(TYPE_TP, 25), source.addr)),
                    tx_hp.sink.dw1.eq(source.dw1),
                    source.ready.eq(tx_hp.sink.ready),
                ]
            else:
                grant = [
                    tx_hp.sink.valid.eq(source.valid & (~source.dpp | jobs.sink.ready)),
                    tx_hp.sink.dw0.eq(Cat(source.dw0[0:25], dev_addr)),
                    tx_hp.sink.dw1.eq(source.dw1),
                    tx_hp.sink.dw2.eq(source.dw2),
                    tx_hp.sink.dpp.eq(source.dpp),
                    source.ready.eq(tx_hp.sink.ready & (~source.dpp | jobs.sink.ready)),
                    jobs.sink.valid.eq(source.valid & source.ready & source.dpp),
                    jobs.sink.ep.eq(i),
                ]
            cases = If(source.valid, *grant) if cases is None else \
                If(source.valid, *grant).Else(cases)
        self.comb += cases
        # TX DPP (in the DPHs order).
        dpp_cases = {}
        for i, ep in enumerate(endpoints):
            ep_tx_dpp = ep.tx_dpp
            if adapted[ep]:
                packer = DPPPacker()
                self.submodules += packer
                self.comb += ep.tx_dpp.connect(packer.sink)
                ep_tx_dpp = packer.source
            dpp_cases[i] = [
                ep_tx_dpp.connect(tx_dpp.sink),
            ]
        self.comb += [
            If(jobs.source.valid,
                Case(jobs.source.ep, dpp_cases)
            ),
            jobs.source.ready.eq(tx_dpp.sink.valid & tx_dpp.sink.ready & tx_dpp.sink.last),
        ]

        # Link Power Management --------------------------------------------------------------------
        # Busy (U1/U2 entry rejected, 7.2.4.2.2): received Header Packets being processed, Header
        # Packets/DPPs to transmit, endpoints busy (ex: ACK TP expected for a sent DP).
        self.comb += self.pm_busy.eq(rx_hp.valid | status.source.valid | resp.source.valid |
            tx_hp.source.valid | jobs.source.valid | tx_dpp.source.valid |
            Reduce("OR", [ep.tx_hp.valid | ep.pm_busy for ep in endpoints]))
