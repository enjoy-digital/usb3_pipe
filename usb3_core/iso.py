#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Isochronous Endpoints (USB 3.2 8.12.6).

Isochronous IN: the user writes service interval transfers (stream, last: end of the interval data)
in packet slots (Max Packet Size: 1024 bytes), sent when requested by the host during a service
interval (ACK TPs), without retries/flow control (see USB3IsoIN).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common    import *
from usb3_core.dpp       import DPPReader
from usb3_core.endpoints import USB3Endpoint
from usb3_core.slots     import USB3PacketSlots, USB3PacketWriter

# USB3 Isochronous IN Endpoint ---------------------------------------------------------------------

class USB3IsoIN(USB3Endpoint):
    """Isochronous IN Endpoint (Device -> Host, USB 3.2 8.12.6, Mult of 0).

    sink: service interval transfers (data: 32-bit words, first byte on data[0:8], be: valid bytes,
    last: end of the transfer) of up to max_burst x 1024 bytes (a transfer is ended after max_burst
    packets), each sent in one service interval: 1024 bytes packets, the last one (short or not)
    with the Last Packet Flag (LPF).
    flush: drop the buffered transfers (and the transfer being written), ex: stream restarted.

    Host requests (ACK TPs, 8.12.6.1): the first ACK TP of a service interval has a Sequence Number
    of 0 and NumP packets requested, the next ones (same interval) Seq Num = previous Seq Num +
    previous NumP. On the first ACK TP, a complete buffered transfer is sent from Sequence Number 0
    (packets sent within the requested window), or a zero-length DP (Seq Num 0, LPF) if no complete
    transfer is available (8.12.6.5, Table 8-33). No retries, no flow control (NRDY/ERDY), no TP
    response. The remaining packets of a transfer interrupted by the next service interval are
    dropped. PING TPs are answered by the Protocol Layer (PING_RESPONSE, 8.5.7).
    """
    def __init__(self, ept, max_burst=1):
        assert max_burst in [1, 2, 4, 8, 16]
        USB3Endpoint.__init__(self, ept, direction=1)
        self.max_burst = max_burst
        self.sink  = sink = stream.Endpoint(endpoint_layout())
        self.flush = Signal() # i

        # Statistics.
        self.n_transfers = Signal(32) # o: Transfers sent (service intervals with data).
        self.n_zlps      = Signal(32) # o: Zero-length DPs sent (no complete transfer available).
        self.n_drops     = Signal(32) # o: Packets dropped (interrupted transfers, flush).

        # # #

        n     = max(max_burst, 2) # Packet slots (a complete transfer at most).
        pbits = log2_int(n) + 1   # Pointers: one more bit than the slot index (full/empty).

        # Packet Slots -----------------------------------------------------------------------------
        self.slots = slots = USB3PacketSlots(n)
        lengths = slots.lengths

        snd_ptr  = Signal(pbits) # Next slot to send (or drop).
        free_ptr = Signal(pbits) # Oldest slot not read yet (DPP).

        # Complete transfers (LPF packet written) not sent.
        transfers     = Signal(max=n + 1)
        transfers_inc = Signal()
        transfers_dec = Signal()
        self.sync += transfers.eq(transfers + transfers_inc - transfers_dec)

        # Writer (user) ----------------------------------------------------------------------------
        # Transfers of up to max_burst packets (LPF on the last one), slots freed once read, sink
        # not ready while flushing.
        flushing = Signal() # Dropping the buffered packets.
        self.writer = writer = USB3PacketWriter(slots, sink,
            rd_ptr      = free_ptr,
            flush       = self.flush,
            enable      = ~self.flush & ~flushing,
            max_packets = max_burst,
        )
        wr_ptr = writer.ptr  # Next slot written by the user.
        lpfs   = writer.lpfs # Last Packet Flag of the slots.
        self.comb += transfers_inc.eq(writer.commit & writer.lpf)

        # Service Interval State -------------------------------------------------------------------
        h = self.hdr
        ack = Signal()
        self.comb += ack.eq(self.hdr_strobe & self.tp_addressed(TP_SUB_ACK))
        start   = Signal()                   # Service interval start (Seq Num 0): transfer or ZLP.
        active  = Signal()                   # Sending a transfer...
        snd_seq = Signal(5)                  # ...next Sequence Number...
        win_end = Signal(5, reset_less=True) # ...up to this one (excluded, Seq Num + NumP).
        skip    = Signal()                   # Dropping the end of an interrupted transfer.

        # Sending ----------------------------------------------------------------------------------
        self.jobs = jobs = stream.SyncFIFO([("slot", slots.sbits), ("zlp", 1), ("drop", 1)], 2*n)
        snd_slot = Signal(slots.sbits)
        snd_lpf  = Signal()
        send     = Signal() # DPH of a slot sent.
        zlp      = Signal() # Zero-length DPH sent.
        drop     = Signal() # Slot dropped (no DPH).
        ending   = Signal() # Last packet of the transfer sent.
        self.comb += [
            snd_slot.eq(snd_ptr[0:pbits-1]),
            snd_lpf.eq(lpfs[snd_slot]),
            If(skip | flushing,
                drop.eq((snd_ptr != wr_ptr) & jobs.sink.ready),
            ).Elif(start,
                If(transfers == 0,
                    self.tx_hp.valid.eq(jobs.sink.ready),
                    self.tx_hp.dw0.eq(TYPE_DP),
                    self.tx_hp.dw1.eq(dph_dw1(seq=0, ept=ept, direction=1, length=0, eob=1)),
                    self.tx_hp.dpp.eq(1),
                    zlp.eq(self.tx_hp.valid & self.tx_hp.ready),
                )
            ).Elif(active & (snd_seq != win_end) & jobs.sink.ready,
                self.tx_hp.valid.eq(1),
                self.tx_hp.dw0.eq(TYPE_DP),
                self.tx_hp.dw1.eq(dph_dw1(seq=snd_seq, ept=ept, direction=1,
                    length = lengths[snd_slot],
                    eob    = snd_lpf)),
                self.tx_hp.dpp.eq(1),
                send.eq(self.tx_hp.ready),
            ),
            jobs.sink.valid.eq(send | zlp | drop),
            jobs.sink.slot.eq(snd_slot),
            jobs.sink.zlp.eq(zlp),
            jobs.sink.drop.eq(drop),
            ending.eq(send & snd_lpf),
            transfers_dec.eq((send | drop) & snd_lpf),
        ]
        self.sync += [
            If(send | drop,
                snd_ptr.eq(snd_ptr + 1),
            ),
            If(send,
                snd_seq.eq(snd_seq + 1),
            ),
            If(ending,
                active.eq(0),
                self.n_transfers.eq(self.n_transfers + 1),
            ),
            If(drop,
                self.n_drops.eq(self.n_drops + 1),
                If(snd_lpf, skip.eq(0)),
            ),
            If(flushing & (snd_ptr == wr_ptr), flushing.eq(0)),
            # Service interval start: complete transfer available (sent from the next cycle) or ZLP.
            If(start & ~skip & ~flushing,
                If(transfers != 0,
                    start.eq(0),
                    active.eq(1),
                    snd_seq.eq(0),
                ).Elif(zlp,
                    start.eq(0),
                    self.n_zlps.eq(self.n_zlps + 1),
                )
            ),
            If(ack,
                If(h.tp_seq == 0,
                    # First ACK TP of a service interval (NumP of 0: no request), a transfer still
                    # being sent is interrupted.
                    start.eq(h.nump != 0),
                    win_end.eq(h.nump),
                    active.eq(0),
                    If(active & ~ending, skip.eq(1)),
                ).Elif(active | start,
                    # Next ACK TP of the service interval: requested window extended.
                    win_end.eq(h.tp_seq + h.nump),
                )
            ),
            If(self.reset_seq,
                start.eq(0),
                active.eq(0),
                If(active & ~ending, skip.eq(1)),
            ),
            If(self.flush,
                flushing.eq(1),
                start.eq(0),
                active.eq(0),
                skip.eq(0),
            ),
        ]

        # DPP Reader -------------------------------------------------------------------------------
        # Jobs read in order, slots freed once read (end of their DPP). Dropped slots are read as
        # zero-length DPPs, not forwarded.
        self.reader = reader = DPPReader(slots.rd_port, depth=16)
        self.pend   = pend   = stream.SyncFIFO([("free", 1), ("drop", 1)], 32)
        self.comb += [
            reader.sink.valid.eq(jobs.source.valid & pend.sink.ready),
            reader.sink.adr.eq(slots.adr(jobs.source.slot)),
            reader.sink.length.eq(Mux(jobs.source.zlp | jobs.source.drop, 0,
                lengths[jobs.source.slot])),
            jobs.source.ready.eq(reader.sink.ready & pend.sink.ready),
            pend.sink.valid.eq(jobs.source.valid & jobs.source.ready),
            pend.sink.free.eq(~jobs.source.zlp),
            pend.sink.drop.eq(jobs.source.drop),
            self.tx_dpp.valid.eq(reader.source.valid & pend.source.valid & ~pend.source.drop),
            self.tx_dpp.data.eq(reader.source.data),
            self.tx_dpp.bytes.eq(reader.source.bytes),
            self.tx_dpp.last.eq(reader.source.last),
            reader.source.ready.eq(pend.source.valid & (pend.source.drop | self.tx_dpp.ready)),
            pend.source.ready.eq(reader.source.valid & reader.source.ready & reader.source.last),
        ]
        self.sync += If(pend.source.valid & pend.source.ready & pend.source.free,
            free_ptr.eq(free_ptr + 1)
        )
