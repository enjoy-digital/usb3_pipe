#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Bulk Endpoints (USB 3.2 8.10/8.11).

Endpoints buffers are rings of packet slots (Max Packet Size: 1024 bytes), one slot per packet of a
burst (max_burst slots):
- IN : the user writes packets (stream, last: short packet), the endpoint sends them in bursts when
  requested by the host (ACK TP with NumP), keeps them until acknowledged (retries), sets EOB/sends
  NRDY and ERDY (flow control).
- OUT: the Data Packet Payloads are written to the slots as received, the endpoint acknowledges them
  (ACK TP with the next sequence number and the free slots as NumP, retry on errors), the user reads
  the packets (stream, last: end of packet), ERDY sent when slots are freed after a NumP of 0.

Endpoints interface with the Protocol Layer (see USB3ProtocolLayer):
- hdr: received Header Packet (broadcast) and hdr_strobe (header processed), with the DPP status
  (dpp_ok/dpp_landed) for DPHs.
- resp_valid/resp_dw1: TP response to the current header (when addressed).
- tx_hp/tx_dpp: Header Packets (DPH/ERDY/NRDY) and Data Packet Payloads to transmit.
- rx_dpp/rx_dpp_done/rx_dpp_ok: received Data Packet Payloads (routed by Endpoint Number).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import *

# Constants ----------------------------------------------------------------------------------------

MAX_PACKET_SIZE  = 1024
MAX_PACKET_WORDS = MAX_PACKET_SIZE//4

# Header Fields ------------------------------------------------------------------------------------

class HeaderFields:
    """Received Header Packet fields (TP/DPH)."""
    def __init__(self, dw0, dw1, dw2):
        self.type      = dw0[0:5]
        # TP.
        self.subtype   = dw1[0:4]
        self.retry     = dw1[6]
        self.direction = dw1[7]
        self.ept       = dw1[8:12]
        self.nump      = dw1[16:21]
        self.tp_seq    = dw1[21:26]
        # DPH.
        self.dp_seq    = dw1[0:5]
        self.setup     = dw1[15]
        self.length    = dw1[16:32]

def last_be_to_bytes(last_be):
    """LiteX last_be (one-hot last valid byte, 0: full word) to valid bytes (1-4)."""
    return Mux(last_be[0], 1, Mux(last_be[1], 2, Mux(last_be[2], 3, 4)))

# USB3 Endpoint Base -------------------------------------------------------------------------------

class USB3Endpoint(LiteXModule):
    def __init__(self, ept, direction):
        self.ept       = ept
        self.direction = direction # 0: OUT, 1: IN.

        # Protocol Layer interface.
        self.hdr_dw0     = Signal(32) # i
        self.hdr_dw1     = Signal(32) # i
        self.hdr_dw2     = Signal(32) # i
        self.hdr_strobe  = Signal()   # i
        self.dpp_ok      = Signal()   # i
        self.dpp_landed  = Signal()   # i
        self.resp_valid  = Signal()   # o
        self.resp_dw1    = Signal(32) # o
        self.tx_hp       = stream.Endpoint(header_layout() + [("dpp", 1)]) # o
        self.tx_dpp      = stream.Endpoint(dpp_layout())                   # o
        self.rx_dpp      = stream.Endpoint(dpp_layout())                   # i
        self.rx_dpp_done = Signal() # i
        self.rx_dpp_ok   = Signal() # i
        self.landed      = Signal() # o: rx DPP landed in a slot (with rx_dpp_done).

        # Control.
        self.reset_seq   = Signal() # i: Sequence Numbers reset (SET_CONFIGURATION, CLEAR_FEATURE(HALT)).

        self.hdr = HeaderFields(self.hdr_dw0, self.hdr_dw1, self.hdr_dw2)

    def tp_addressed(self, subtype):
        h = self.hdr
        return (h.type == TYPE_TP) & (h.subtype == subtype) & (h.ept == self.ept) & \
            (h.direction == self.direction)

    def dp_addressed(self):
        h = self.hdr
        return (h.type == TYPE_DP) & (h.ept == self.ept) & (h.setup == 0)

# USB3 Bulk IN Endpoint ----------------------------------------------------------------------------

class USB3BulkIN(USB3Endpoint):
    """Bulk IN Endpoint (Device -> Host).

    sink: packets to send (data: 32-bit words, first byte on data[0:8], last: end of packet with
    last_be the last valid byte (0: full word)), packets longer than 1024 bytes are split.
    """
    def __init__(self, ept, max_burst=8):
        assert max_burst in [2, 4, 8, 16]
        USB3Endpoint.__init__(self, ept, direction=1)
        self.max_burst = max_burst
        self.sink = sink = stream.Endpoint([("data", 32), ("last_be", 4)])

        # # #

        n     = max_burst
        pbits = log2_int(n) + 1 # Pointers: one more bit than the slot index (full/empty).

        # Packet Slots -----------------------------------------------------------------------------
        mem = Memory(32, n*MAX_PACKET_WORDS)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(has_re=True)
        self.specials += mem, wr_port, rd_port
        lengths = Array(Signal(11) for _ in range(n))

        wr_ptr  = Signal(pbits) # Next slot written by the user.
        ack_ptr = Signal(pbits) # Oldest slot not acknowledged by the host.
        snd_ptr = Signal(pbits) # Next slot to send.
        ack_seq = Signal(5)     # Sequence Number of ack_ptr's packet.
        snd_seq = Signal(5)
        self.comb += snd_seq.eq(ack_seq + (snd_ptr - ack_ptr))

        # Writer (user) ----------------------------------------------------------------------------
        wr_count = Signal(9)
        wr_full  = Signal()
        self.comb += [
            wr_full.eq((wr_ptr - ack_ptr) == n),
            sink.ready.eq(~wr_full),
            wr_port.adr.eq(Cat(wr_count[0:8], wr_ptr[0:pbits-1])),
            wr_port.dat_w.eq(sink.data),
            wr_port.we.eq(sink.valid & sink.ready),
        ]
        wr_commit = Signal()
        self.comb += wr_commit.eq(sink.valid & sink.ready & (sink.last | (wr_count == MAX_PACKET_WORDS - 1)))
        self.sync += [
            If(sink.valid & sink.ready,
                wr_count.eq(wr_count + 1),
                If(wr_commit,
                    wr_count.eq(0),
                    lengths[wr_ptr[0:pbits-1]].eq(4*wr_count +
                        Mux(sink.last, last_be_to_bytes(sink.last_be), 4)),
                    wr_ptr.eq(wr_ptr + 1),
                )
            )
        ]

        # Host requests (ACK TP) -------------------------------------------------------------------
        win_seq   = Signal(5) # Host window: Sequence Number of the next packet requested...
        win_nump  = Signal(5) # ...and number of packets requested from win_seq.
        wait_erdy = Signal()  # Flow control: NRDY/EOB sent, ERDY to send when data available.
        req       = Signal()  # Host ACK TP request with NumP != 0 (strobe).
        h = self.hdr
        ack = Signal()
        self.comb += ack.eq(self.hdr_strobe & self.tp_addressed(TP_SUB_ACK))
        acked = Signal(5)
        self.comb += acked.eq(h.tp_seq - ack_seq)

        # Sending ----------------------------------------------------------------------------------
        avail    = Signal() # Packet available to send.
        in_win   = Signal() # snd_seq in the host window.
        send     = Signal()
        snd_len  = Signal(11)
        snd_eob  = Signal()
        self.comb += [
            avail.eq(snd_ptr != wr_ptr),
            in_win.eq(((snd_seq - win_seq) & 0x1f) < win_nump),
            snd_len.eq(lengths[snd_ptr[0:pbits-1]]),
            # EOB: last available packet, window not exhausted and not a short packet (8.6.3).
            snd_eob.eq(((snd_ptr + 1) == wr_ptr) & ((((snd_seq + 1) - win_seq) & 0x1f) < win_nump) &
                (snd_len == MAX_PACKET_SIZE)),
        ]

        # DPP jobs (slots of the DPHs pushed, in order).
        self.jobs = jobs = stream.SyncFIFO([("slot", pbits - 1)], 2*n)

        # NRDY on a request without available packet.
        nrdy_pending = Signal()
        erdy_pending = Signal()

        self.comb += [
            If(nrdy_pending,
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_TP),
                self.tx_hp.dw1.eq(tp_dw1(TP_SUB_NRDY, ept=ept, direction=1)),
            ).Elif(erdy_pending,
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_TP),
                self.tx_hp.dw1.eq(tp_dw1(TP_SUB_ERDY, ept=ept, direction=1, nump=n)),
            ).Elif(avail & in_win & ~wait_erdy & jobs.sink.ready,
                send.eq(self.tx_hp.ready),
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_DP),
                self.tx_hp.dw1.eq(dph_dw1(seq=snd_seq, ept=ept, direction=1, length=snd_len, eob=snd_eob)),
                self.tx_hp.dpp.eq(1),
            ),
            jobs.sink.valid.eq(send),
            jobs.sink.slot.eq(snd_ptr[0:pbits-1]),
        ]

        self.sync += [
            If(send,
                snd_ptr.eq(snd_ptr + 1),
                # EOB: flow control, the host requests again after our ERDY.
                If(snd_eob,
                    wait_erdy.eq(1),
                    win_nump.eq(0),
                ),
            ),
            If(self.tx_hp.valid & self.tx_hp.ready & nrdy_pending, nrdy_pending.eq(0)),
            If(self.tx_hp.valid & self.tx_hp.ready & ~nrdy_pending & erdy_pending, erdy_pending.eq(0)),
            If(wait_erdy & ~erdy_pending & ~nrdy_pending & (wr_ptr != snd_ptr),
                erdy_pending.eq(1),
                wait_erdy.eq(0),
            ),
            If(ack,
                # Acknowledged packets: free slots.
                If(acked <= (snd_ptr - ack_ptr),
                    ack_ptr.eq(ack_ptr + acked[0:pbits]),
                    ack_seq.eq(h.tp_seq),
                ),
                win_seq.eq(h.tp_seq),
                win_nump.eq(h.nump),
                # Retry: resend from the requested packet.
                If(h.retry,
                    snd_ptr.eq(ack_ptr + acked[0:pbits]),
                ),
                # Request without available packet: NRDY, ERDY when data available.
                If((h.nump != 0) & ~h.retry & ((ack_ptr + acked[0:pbits]) == wr_ptr),
                    nrdy_pending.eq(1),
                    wait_erdy.eq(1),
                    win_nump.eq(0),
                ),
            ),
            If(self.reset_seq,
                ack_seq.eq(0),
                snd_ptr.eq(ack_ptr),
                win_nump.eq(0),
                wait_erdy.eq(0),
                nrdy_pending.eq(0),
                erdy_pending.eq(0),
            ),
        ]

        # DPP Reader -------------------------------------------------------------------------------
        self.prefetch = prefetch = stream.SyncFIFO(dpp_layout(), 16)
        rd_count  = Signal(9)
        rd_len    = Signal(11)
        rd_words  = Signal(9)
        rd_active = Signal()
        rd_valid  = Signal()
        rd_last   = Signal()
        rd_bytes  = Signal(3)
        rd_slot   = Signal(pbits - 1)
        self.comb += [
            rd_words.eq((rd_len + 3)[2:]),
            rd_port.adr.eq(Cat(rd_count[0:8], rd_slot)),
        ]
        rd_issue = Signal()
        self.comb += [
            jobs.source.ready.eq(~rd_active),
            rd_issue.eq(rd_active & (prefetch.level < 12)),
            rd_port.re.eq(rd_issue),
        ]
        self.sync += [
            rd_valid.eq(rd_issue),
            rd_last.eq(rd_issue & ((rd_count + 1 >= rd_words))),
            rd_bytes.eq(Mux((rd_count + 1 >= rd_words) & (rd_len[0:2] != 0), rd_len[0:2], 4)),
            If(jobs.source.valid & jobs.source.ready,
                rd_active.eq(1),
                rd_slot.eq(jobs.source.slot),
                rd_len.eq(lengths[jobs.source.slot]),
                rd_count.eq(0),
            ).Elif(rd_issue,
                rd_count.eq(rd_count + 1),
                If(rd_count + 1 >= rd_words, rd_active.eq(0)),
            ),
        ]
        # Zero-length packets: a single word with 0 valid bytes.
        zlp = Signal()
        self.sync += zlp.eq(jobs.source.valid & jobs.source.ready & (lengths[jobs.source.slot] == 0))
        self.comb += [
            prefetch.sink.valid.eq(rd_valid | zlp),
            prefetch.sink.data.eq(rd_port.dat_r),
            prefetch.sink.bytes.eq(Mux(zlp, 0, rd_bytes)),
            prefetch.sink.last.eq(rd_last | zlp),
            prefetch.source.connect(self.tx_dpp),
        ]
        # A zero-length packet does not issue reads.
        self.comb += If(rd_len == 0, rd_issue.eq(0))
        self.sync += If(rd_active & (rd_len == 0), rd_active.eq(0))

# USB3 Bulk OUT Endpoint ---------------------------------------------------------------------------

class USB3BulkOUT(USB3Endpoint):
    """Bulk OUT Endpoint (Host -> Device).

    source: received packets (data: 32-bit words, first byte on data[0:8], last: end of packet with
    last_be the last valid byte (0: full word)). Zero-length packets are dropped.
    """
    def __init__(self, ept, max_burst=8):
        assert max_burst in [2, 4, 8, 16]
        USB3Endpoint.__init__(self, ept, direction=0)
        self.max_burst = max_burst
        self.source = source = stream.Endpoint([("data", 32), ("last_be", 4)])

        # # #

        n     = max_burst
        pbits = log2_int(n) + 1

        # Packet Slots -----------------------------------------------------------------------------
        mem = Memory(32, n*MAX_PACKET_WORDS)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(has_re=True)
        self.specials += mem, wr_port, rd_port
        lengths = Array(Signal(11) for _ in range(n))
        valids  = Array(Signal()   for _ in range(n))

        land_ptr   = Signal(pbits) # Next slot to land a DPP.
        decide_ptr = Signal(pbits) # Next landed slot to accept/discard (Protocol Layer, in order).
        rd_ptr     = Signal(pbits) # Next slot read by the user.
        exp_seq    = Signal(5)     # Next expected Sequence Number.
        free       = Signal(pbits)
        self.comb += free.eq(n - (land_ptr - rd_ptr))

        # Landing (DPPs written as received) -------------------------------------------------------
        land_count = Signal(9)
        land_bytes = Signal(11)
        land_en    = Signal() # Slot available at the DPP start (latched).
        land_data  = Signal() # DPP data received.
        landing    = Signal()
        self.comb += [
            If(land_data,
                landing.eq(land_en)
            ).Else(
                landing.eq(free != 0)
            ),
            wr_port.adr.eq(Cat(land_count[0:8], land_ptr[0:pbits-1])),
            wr_port.dat_w.eq(self.rx_dpp.data),
            wr_port.we.eq(self.rx_dpp.valid & landing & (land_count < MAX_PACKET_WORDS)),
            self.rx_dpp.ready.eq(1),
            self.landed.eq(self.rx_dpp_done & landing),
        ]
        self.sync += [
            If(self.rx_dpp.valid,
                land_data.eq(1),
                If(self.rx_dpp.first,
                    land_en.eq(free != 0),
                    land_count.eq(1),
                    land_bytes.eq(self.rx_dpp.bytes),
                ).Else(
                    land_count.eq(land_count + 1),
                    land_bytes.eq(land_bytes + self.rx_dpp.bytes),
                )
            ),
            If(self.rx_dpp_done,
                land_data.eq(0),
                land_count.eq(0),
                land_bytes.eq(0),
                If(landing,
                    lengths[land_ptr[0:pbits-1]].eq(land_bytes),
                    land_ptr.eq(land_ptr + 1),
                )
            ),
        ]

        # Decision (DPH processed by the Protocol Layer) -------------------------------------------
        h = self.hdr
        dph    = Signal()
        accept = Signal()
        self.comb += [
            dph.eq(self.dp_addressed()),
            accept.eq(self.dpp_ok & self.dpp_landed & (h.dp_seq == exp_seq)),
        ]
        nump = Signal(5)
        self.comb += nump.eq(Mux(accept, free, free + self.dpp_landed)) # Discarded slot is freed.
        self.comb += [
            self.resp_valid.eq(dph),
            self.resp_dw1.eq(tp_dw1(TP_SUB_ACK, ept=ept, direction=0,
                nump  = nump,
                seq   = Mux(accept, exp_seq + 1, exp_seq),
                retry = ~accept & (h.dp_seq == exp_seq))),
        ]
        need_erdy = Signal()
        self.sync += [
            If(self.hdr_strobe & dph,
                If(self.dpp_landed,
                    valids[decide_ptr[0:pbits-1]].eq(accept),
                    decide_ptr.eq(decide_ptr + 1),
                ),
                If(accept, exp_seq.eq(exp_seq + 1)),
                If(nump == 0, need_erdy.eq(1)),
            ),
            If(self.reset_seq, exp_seq.eq(0)),
        ]

        # ERDY when slots are freed after a NumP of 0.
        self.comb += [
            If(need_erdy & (free != 0),
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_TP),
                self.tx_hp.dw1.eq(tp_dw1(TP_SUB_ERDY, ept=ept, direction=0, nump=free)),
            )
        ]
        self.sync += If(self.tx_hp.valid & self.tx_hp.ready, need_erdy.eq(0))

        # Reader (user) ----------------------------------------------------------------------------
        self.fifo = fifo = stream.SyncFIFO([("data", 32), ("last_be", 4)], 16)
        rd_count  = Signal(9)
        rd_len    = Signal(11)
        rd_words  = Signal(9)
        rd_active = Signal()
        rd_valid  = Signal()
        rd_last   = Signal()
        rd_be     = Signal(4)
        rd_issue  = Signal()
        rd_slot   = Signal(pbits - 1)
        self.comb += [
            rd_slot.eq(rd_ptr[0:pbits-1]),
            rd_len.eq(lengths[rd_slot]),
            rd_words.eq((rd_len + 3)[2:]),
            rd_port.adr.eq(Cat(rd_count[0:8], rd_slot)),
            rd_issue.eq(rd_active & (fifo.level < 12)),
            rd_port.re.eq(rd_issue),
        ]
        self.sync += [
            rd_valid.eq(rd_issue),
            rd_last.eq(rd_issue & (rd_count + 1 >= rd_words)),
            rd_be.eq(Mux(rd_count + 1 >= rd_words,
                Mux(rd_len[0:2] == 1, 0b0001, Mux(rd_len[0:2] == 2, 0b0010, Mux(rd_len[0:2] == 3, 0b0100, 0b1000))),
                0)),
            If(~rd_active,
                # Next decided slot: read it (valid, not empty) or skip it.
                If(rd_ptr != decide_ptr,
                    If(valids[rd_slot] & (rd_len != 0),
                        rd_active.eq(1),
                        rd_count.eq(0),
                    ).Else(
                        rd_ptr.eq(rd_ptr + 1),
                    )
                )
            ).Elif(rd_issue,
                rd_count.eq(rd_count + 1),
                If(rd_count + 1 >= rd_words,
                    rd_active.eq(0),
                    rd_ptr.eq(rd_ptr + 1),
                )
            ),
        ]
        self.comb += [
            fifo.sink.valid.eq(rd_valid),
            fifo.sink.data.eq(rd_port.dat_r),
            fifo.sink.last.eq(rd_last),
            fifo.sink.last_be.eq(rd_be),
            fifo.source.connect(source),
        ]
