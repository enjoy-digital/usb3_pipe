#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Packet Slots: packet buffers of the Bulk (bulk.py) and Isochronous (iso.py) Endpoints.

- USB3PacketSlots : memory of n slots of Max Packet Size (1024 bytes) with write/read ports and the
  per-slot lengths (bytes), slot addressing (slot, word in slot).
- USB3PacketWriter: user-side writer of the IN Endpoints: packets (stream) written to the slots,
  split at Max Packet Size, slot committed (length, write pointer) at the end of each packet.
"""

from migen import *

from litex.gen import *

from usb3_core.common    import *
from usb3_core.endpoints import MAX_PACKET_SIZE, MAX_PACKET_WORDS

# USB3 Packet Slots --------------------------------------------------------------------------------

class USB3PacketSlots(LiteXModule):
    """Packet Slots: n slots x Max Packet Size (1024 bytes) in data_width words (32/64-bit).

    wr_port: write port (adr/dat_w/we), rd_port: read port (adr/re/dat_r, 1 cycle latency).
    lengths: per-slot packet length in bytes (Array, indexed by slot).
    adr(slot, count): memory word address of word count of slot (word 0 when count is None).
    """
    def __init__(self, n, data_width=32):
        assert n >= 2
        assert data_width in [32, 64]
        self.n          = n
        self.data_width = data_width
        self.words      = MAX_PACKET_SIZE//(data_width//8) # Words per slot.
        self.sbits      = log2_int(n)                     # Slot index bits.
        self.wbits      = log2_int(self.words)            # Word in slot bits.

        # # #

        # Memory (not an attribute: not collected as a CSR memory).
        mem = Memory(data_width, n*self.words)
        self.wr_port = mem.get_port(write_capable=True)
        self.rd_port = mem.get_port(has_re=True)
        self.specials += mem

        # Lengths.
        self.lengths = Array(Signal(bits_for(MAX_PACKET_SIZE), reset_less=True) for _ in range(n))

    def adr(self, slot, count=None):
        word = C(0, self.wbits) if count is None else count[0:self.wbits]
        return Cat(word, slot)

# USB3 Packet Writer -------------------------------------------------------------------------------

class USB3PacketWriter(LiteXModule):
    """Packet Writer (user side of the IN Endpoints).

    sink: packets (data: slots words, first byte on data[0:8], be: valid bytes, last: end of
    packet) written to the slot of ptr, packets longer than Max Packet Size split. The slot is
    committed (commit: length from be, ptr incremented) on last or when full.
    rd_ptr: oldest slot in use (endpoint side), sink not ready when the n slots are used.
    flush: write restarted (packet being written dropped).
    enable: optional extra sink.ready condition.
    max_packets: optional transfers of up to max_packets packets (Isochronous): lpf (Last Packet
    Flag: sink.last or max_packets-th packet of the transfer) stored per slot in lpfs.
    """
    def __init__(self, slots, sink, rd_ptr, flush, enable=None, max_packets=None):
        n     = slots.n
        sbits = slots.sbits
        self.ptr    = ptr    = Signal(sbits + 1) # o: Next slot written (one more bit: full/empty).
        self.commit = commit = Signal()          # o: Packet committed (with sink handshake).
        if max_packets is not None:
            self.lpf  = lpf  = Signal()          # o: Last Packet Flag of the packet written.
            self.lpfs = lpfs = Array(Signal(reset_less=True) for _ in range(n))

        # # #

        slot  = ptr[0:sbits]
        words = slots.words
        wb    = slots.data_width//8                # Bytes per word.
        count = Signal(bits_for(words))            # Words of the packet written.
        used  = Signal(sbits + 1)                  # Written slots not freed (modulo 2*n).
        full  = Signal()
        self.comb += [
            used.eq(ptr - rd_ptr),
            full.eq(used == n),
        ]

        # Sink.
        self.comb += [
            sink.ready.eq(~full if enable is None else ~full & enable),
            slots.wr_port.adr.eq(slots.adr(slot, count)),
            slots.wr_port.dat_w.eq(sink.data),
            slots.wr_port.we.eq(sink.valid & sink.ready),
            commit.eq(sink.valid & sink.ready & (sink.last | (count == words - 1))),
        ]

        # Count/Commit.
        on_flush  = [count.eq(0)]
        on_commit = [
            count.eq(0),
            slots.lengths[slot].eq(wb*count + Mux(sink.last, be_to_bytes(sink.be), wb)),
            ptr.eq(ptr + 1),
        ]
        if max_packets is not None:
            npkts = Signal(max=max(max_packets, 2)) # Packets of the transfer written.
            self.comb += lpf.eq(sink.last | (npkts == max_packets - 1))
            on_flush  += [npkts.eq(0)]
            on_commit += [
                npkts.eq(Mux(lpf, 0, npkts + 1)),
                lpfs[slot].eq(lpf),
            ]
        self.sync += [
            If(flush,
                *on_flush
            ).Elif(sink.valid & sink.ready,
                count.eq(count + 1),
                If(commit,
                    *on_commit
                )
            )
        ]
