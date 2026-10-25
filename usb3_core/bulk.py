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
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common    import *
from usb3_core.dpp       import DPPReader
from usb3_core.endpoints import USB3Endpoint, MAX_PACKET_SIZE, MAX_PACKET_WORDS

# Constants ----------------------------------------------------------------------------------------

WR_IDLE_CYCLES = 64 # Bulk IN: writer idle cycles before a last max size packet is sent (burst end).

# USB3 Bulk IN Endpoint ----------------------------------------------------------------------------

class USB3BulkIN(USB3Endpoint):
    """Bulk IN Endpoint (Device -> Host).

    sink: packets to send (data: 32-bit words, first byte on data[0:8], be: valid bytes, last: end of
    packet), packets longer than 1024 bytes are split.
    flush: drop the buffered packets (and the packet being written), ex: stream restarted.
    """
    def __init__(self, ept, max_burst=8):
        assert max_burst in [2, 4, 8, 16]
        USB3Endpoint.__init__(self, ept, direction=1)
        self.max_burst = max_burst
        self.sink  = sink = stream.Endpoint(endpoint_layout())
        self.flush = Signal() # i

        # # #

        n     = max_burst
        pbits = log2_int(n) + 1 # Pointers: one more bit than the slot index (full/empty).

        # Packet Slots -----------------------------------------------------------------------------
        mem = Memory(32, n*MAX_PACKET_WORDS)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(has_re=True)
        self.specials += mem, wr_port, rd_port
        lengths = Array(Signal(11, reset_less=True) for _ in range(n))

        wr_ptr  = Signal(pbits) # Next slot written by the user.
        ack_ptr = Signal(pbits) # Oldest slot not acknowledged by the host.
        snd_ptr = Signal(pbits) # Next slot to send.
        ack_seq = Signal(5)     # Sequence Number of ack_ptr's packet.
        snd_seq = Signal(5)
        # Pointers differences/increments (modulo 2*n: explicit widths, Migen differences are signed).
        inflight = Signal(pbits) # Sent, not acknowledged.
        used     = Signal(pbits) # Written, not acknowledged.
        snd_next = Signal(pbits)
        self.comb += [
            inflight.eq(snd_ptr - ack_ptr),
            used.eq(wr_ptr - ack_ptr),
            snd_next.eq(snd_ptr + 1),
            snd_seq.eq(ack_seq + inflight),
        ]

        # Writer (user) ----------------------------------------------------------------------------
        wr_count = Signal(9)
        wr_full  = Signal()
        self.comb += [
            wr_full.eq(used == n),
            sink.ready.eq(~wr_full),
            wr_port.adr.eq(Cat(wr_count[0:8], wr_ptr[0:pbits-1])),
            wr_port.dat_w.eq(sink.data),
            wr_port.we.eq(sink.valid & sink.ready),
        ]
        wr_commit = Signal()
        self.comb += wr_commit.eq(sink.valid & sink.ready & (sink.last | (wr_count == MAX_PACKET_WORDS - 1)))
        self.sync += [
            If(self.flush,
                wr_count.eq(0),
            ).Elif(sink.valid & sink.ready,
                wr_count.eq(wr_count + 1),
                If(wr_commit,
                    wr_count.eq(0),
                    lengths[wr_ptr[0:pbits-1]].eq(4*wr_count +
                        Mux(sink.last, be_to_bytes(sink.be), 4)),
                    wr_ptr.eq(wr_ptr + 1),
                )
            )
        ]

        # Host requests (ACK TP) -------------------------------------------------------------------
        win_seq   = Signal(5, reset_less=True) # Host window: Sequence Number of the next packet requested...
        win_nump  = Signal(5)                  # ...and number of packets requested from win_seq.
        h = self.hdr
        ack = Signal()
        self.comb += ack.eq(self.hdr_strobe & self.tp_addressed(TP_SUB_ACK))
        acked     = Signal(5)
        acked_ptr = Signal(pbits)
        self.comb += [
            acked.eq(h.tp_seq - ack_seq),
            acked_ptr.eq(ack_ptr + acked),
        ]

        # Flow control (8.10.1/8.10.2): after an EOB DP, a NRDY or a window closed by the host (NumP
        # of 0), an ERDY is sent when data is available and the host requests again. ACK TPs
        # received for packets sent before an EOB (with a NumP still open) do not reopen the window.
        fc_eob  = Signal() # EOB sent: waiting for the acknowledgement of the sent packets.
        fc_data = Signal() # Waiting for data: ERDY when a packet is available.
        fc_req  = Signal() # ERDY sent: waiting for the host request.
        fc      = Signal()
        self.comb += fc.eq(fc_eob | fc_data | fc_req)

        # Sending ----------------------------------------------------------------------------------
        avail    = Signal() # Packet available to send.
        in_win   = Signal() # snd_seq in the host window.
        send     = Signal()
        snd_len  = Signal(11)
        snd_eob  = Signal()
        snd_last = Signal() # Last written packet.
        # Writer activity: while the user is writing, the last written max size packet is held until
        # the next one is written (burst continued instead of an EOB and a flow control round trip).
        wr_idle  = Signal(max=WR_IDLE_CYCLES + 1)
        wr_busy  = Signal()
        self.sync += If(sink.valid, wr_idle.eq(0)).Elif(wr_idle != WR_IDLE_CYCLES,
            wr_idle.eq(wr_idle + 1))
        self.comb += [
            wr_busy.eq(sink.valid | (wr_idle != WR_IDLE_CYCLES)),
            snd_last.eq(snd_next == wr_ptr),
            avail.eq((snd_ptr != wr_ptr) & ~(snd_last & wr_busy & (snd_len == MAX_PACKET_SIZE))),
            in_win.eq(((snd_seq - win_seq) & 0x1f) < win_nump),
            snd_len.eq(lengths[snd_ptr[0:pbits-1]]),
            # EOB: last available packet, window not exhausted and not a short packet (8.6.3).
            snd_eob.eq(snd_last & ((((snd_seq + 1) - win_seq) & 0x1f) < win_nump) &
                (snd_len == MAX_PACKET_SIZE)),
        ]

        # DPP jobs (slots of the DPHs pushed, in order).
        self.jobs = jobs = stream.SyncFIFO([("slot", pbits - 1)], 2*n)

        # Sent packets not acknowledged by the host (ACK TP expected): U1/U2 entry rejected.
        self.comb += self.pm_busy.eq(inflight != 0)

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
            ).Elif(avail & in_win & ~fc & jobs.sink.ready,
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
                If(snd_eob,
                    fc_eob.eq(1),
                    win_nump.eq(0),
                ),
                # A short packet ends the transfer/burst (8.10.2): the host discards the next ones,
                # wait for its ACK TP (new request or NumP=0 and ERDY).
                If(snd_len != MAX_PACKET_SIZE,
                    win_nump.eq(0),
                ),
            ),
            If(self.tx_hp.valid & self.tx_hp.ready & nrdy_pending, nrdy_pending.eq(0)),
            If(self.tx_hp.valid & self.tx_hp.ready & ~nrdy_pending & erdy_pending,
                erdy_pending.eq(0),
                fc_req.eq(1),
            ),
            # EOB packet (and the previous ones) acknowledged: wait for data.
            If(fc_eob & (inflight == 0),
                fc_eob.eq(0),
                fc_data.eq(1),
            ),
            # Data available: ERDY.
            If(fc_data & ~nrdy_pending & ~erdy_pending & (wr_ptr != snd_ptr),
                fc_data.eq(0),
                erdy_pending.eq(1),
            ),
            If(ack,
                # Acknowledged packets: free slots.
                If(acked <= inflight,
                    ack_ptr.eq(acked_ptr),
                    ack_seq.eq(h.tp_seq),
                ),
                If(h.retry,
                    # Retry: resend from the requested packet (re-requested: flow control exited).
                    snd_ptr.eq(acked_ptr),
                    win_seq.eq(h.tp_seq),
                    win_nump.eq(h.nump),
                    fc_eob.eq(0),
                    fc_data.eq(0),
                    fc_req.eq(0),
                ).Elif(~fc_eob,
                    win_seq.eq(h.tp_seq),
                    win_nump.eq(h.nump),
                    fc_data.eq(0),
                    fc_req.eq(0),
                    If(h.nump == 0,
                        # Window closed by the host (ex: after a short packet, 8.10.2): ERDY when data
                        # is available (the host may also request again on its own).
                        fc_data.eq(1),
                    ).Elif(acked_ptr == wr_ptr,
                        # Request without available packet: NRDY, ERDY when data available.
                        nrdy_pending.eq(1),
                        fc_data.eq(1),
                        win_nump.eq(0),
                    ),
                ),
            ),
            If(self.reset_seq,
                ack_seq.eq(0),
                snd_ptr.eq(ack_ptr),
                win_nump.eq(0),
                fc_eob.eq(0),
                fc_data.eq(0),
                fc_req.eq(0),
                nrdy_pending.eq(0),
                erdy_pending.eq(0),
            ),
            If(self.flush,
                ack_ptr.eq(wr_ptr),
                snd_ptr.eq(wr_ptr),
            ),
        ]

        # DPP Reader -------------------------------------------------------------------------------
        self.reader = reader = DPPReader(rd_port, depth=16)
        self.comb += [
            jobs.source.connect(reader.sink, omit={"slot"}),
            reader.sink.adr.eq(Cat(C(0, 8), jobs.source.slot)),
            reader.sink.length.eq(lengths[jobs.source.slot]),
            reader.source.connect(self.tx_dpp),
        ]

# USB3 Bulk OUT Endpoint ---------------------------------------------------------------------------

class USB3BulkOUT(USB3Endpoint):
    """Bulk OUT Endpoint (Host -> Device).

    source: received packets (data: 32-bit words, first byte on data[0:8], be: valid bytes, last: end of
    packet). Zero-length packets are dropped.
    """
    def __init__(self, ept, max_burst=8):
        assert max_burst in [2, 4, 8, 16]
        USB3Endpoint.__init__(self, ept, direction=0)
        self.max_burst = max_burst
        self.source = source = stream.Endpoint(endpoint_layout())

        # # #

        n     = max_burst
        pbits = log2_int(n) + 1

        # Packet Slots -----------------------------------------------------------------------------
        mem = Memory(32, n*MAX_PACKET_WORDS)
        wr_port = mem.get_port(write_capable=True)
        rd_port = mem.get_port(has_re=True)
        self.specials += mem, wr_port, rd_port
        lengths = Array(Signal(11, reset_less=True) for _ in range(n))
        valids  = Array(Signal(reset_less=True)     for _ in range(n))

        land_ptr   = Signal(pbits) # Next slot to land a DPP.
        decide_ptr = Signal(pbits) # Next landed slot to accept/discard (Protocol Layer, in order).
        rd_ptr     = Signal(pbits) # Next slot read by the user.
        exp_seq    = Signal(5)     # Next expected Sequence Number.
        free       = Signal(pbits)     # Slots available to land DPPs.
        occupied   = Signal(pbits)
        decided    = Signal(pbits)     # Slots of decided (accepted/discarded) packets not read yet.
        self.comb += [
            occupied.eq(land_ptr - rd_ptr),
            decided.eq(decide_ptr - rd_ptr),
            free.eq(n - occupied),
        ]

        # Landing (DPPs written as received) -------------------------------------------------------
        land_count = Signal(9)
        land_bytes = Signal(11)
        land_en    = Signal(reset_less=True) # Slot available at the DPP start (latched).
        land_data  = Signal()                # DPP data received.
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
        # NumP: packets the host can send after this one: the slots not holding older packets (the
        # packets already landed after this one are counted by the host, NumP decrements by one at
        # most per ACK TP, 8.10.2).
        nump = Signal(5)
        self.comb += nump.eq(n - decided - accept)
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
            If(need_erdy & (decided != n) & (free != 0),
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_TP),
                self.tx_hp.dw1.eq(tp_dw1(TP_SUB_ERDY, ept=ept, direction=0, nump=n - decided)),
            )
        ]
        self.sync += If(self.tx_hp.valid & self.tx_hp.ready, need_erdy.eq(0))

        # Reader (user) ----------------------------------------------------------------------------
        self.fifo = fifo = stream.SyncFIFO(endpoint_layout(), 16)
        rd_count  = Signal(9, reset_less=True)
        rd_len    = Signal(11)
        rd_words  = Signal(9)
        rd_active = Signal()
        rd_valid  = Signal()
        rd_last   = Signal(reset_less=True)
        rd_be     = Signal(4, reset_less=True)
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
            rd_be.eq(Mux(rd_count + 1 >= rd_words, bytes_to_be(rd_len), 0b1111)),
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
            fifo.sink.be.eq(rd_be),
            fifo.source.connect(source),
        ]
