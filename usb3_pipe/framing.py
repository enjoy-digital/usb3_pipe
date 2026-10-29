#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 2 packet framing (USB 3.2 7.2.1, 7.2.1.3, Table 6-2): Data Blocks bytes <-> Link Layer words.

In Gen 2 operation, packets and Link Commands are placed in the Data Blocks at any Symbol position
(no word alignment) with the Gen 1 framing Symbols as scrambled bytes, Idle Symbols (5Ah) between
them. The Link Layer interface is kept as in Gen 1 operation: 32-bit words with K Symbols (ctrl),
framing Ordered Sets at word boundaries.

- Header Packet: HPSTART (SHP x3, EPF), 12-byte header, CRC-16, Link Control Word (20 bytes).
- Non-deferred Gen 2 DPH: DPHSTART (DPHP x3, EPF), header, CRC-16, Link Control Word, Length Field
  Replica x2 (24 bytes), followed by its DPP without spacing.
- DPP: DPPSTART (SDP x3, EPF), data (length from the DPH), CRC-32, DPPEND (END x3, EPF) or
  DPPABORT (EDB x3, EPF). Nullified DPP: DPPABORT right after the DPH.
- Link Command: LCSTART (SLC x3, EPF), Link Command Word x2 (8 bytes).
"""

from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import SHP as _SHP, SDP as _SDP, EDB as _EDB, END as _END, SLC as _SLC
from usb3_pipe.common import EPF as _EPF
from usb3_pipe.common import GEN2_IDLE, GEN2_SHP, GEN2_DPHP, GEN2_SDP, GEN2_EDB, GEN2_END
from usb3_pipe.common import GEN2_SLC, GEN2_EPF

# Gen 1 K Symbols (Link Layer words).
SHP, SDP, EDB, END, SLC, EPF = [s.value for s in [_SHP, _SDP, _EDB, _END, _SLC, _EPF]]

# Packet kinds.
KIND_HP   = 0 # Header Packet (HPSTART).
KIND_DPH  = 1 # Non-deferred Gen 2 DPH (DPHSTART).
KIND_LC   = 2 # Link Command (LCSTART).
KIND_DPP  = 3 # Data Packet Payload (DPPSTART).
KIND_EDB  = 4 # Nullified DPP (DPPABORT).

def byte_offset(n):
    """n bytes in bits (n*8) as a sized expression: in Verilog, a shift amount is self-determined and
    8*n would be evaluated on max(width(8), width(n)) bits (truncated)."""
    return Cat(C(0, 3), n)

def gen2_link_layout():
    """Link words pairs (Symbol 0 on data[0:8]), words: 1 or 2 valid words."""
    return [("data", 64), ("ctrl", 8), ("words", 2)]

class WordsToPairs(LiteXModule):
    """Link words -> words pairs (gen2_link_layout, one word per entry)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint(gen2_link_layout())

        # # #

        self.comb += [
            sink.connect(source, omit={"data", "ctrl"}),
            source.data.eq(sink.data),
            source.ctrl.eq(sink.ctrl),
            source.words.eq(1),
        ]

class PairsToWords(LiteXModule):
    """Words pairs (gen2_link_layout, 1 or 2 words per entry) -> link words (packets first/last on
    their first/last words)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(gen2_link_layout())
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        second = Signal()
        self.comb += [
            source.valid.eq(sink.valid),
            source.first.eq(sink.first & ~second),
            source.last.eq(sink.last & (second | (sink.words == 1))),
            source.data.eq(Mux(second, sink.data[32:64], sink.data[0:32])),
            source.ctrl.eq(Mux(second, sink.ctrl[4:8],   sink.ctrl[0:4])),
            sink.ready.eq(source.ready & (second | (sink.words == 1))),
        ]
        self.sync += If(source.valid & source.ready,
            second.eq(~second & (sink.words == 2))
        )

class WordsPacker(LiteXModule):
    """Link words -> full words pairs (gen2_link_layout, two words per entry; a lone word is output
    alone when no word follows it): words pairs at the words rate/2 (ex: Link Layer in a clock domain
    at half the PHY frequency)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint(gen2_link_layout())

        # # #

        held      = Signal()
        held_data = Signal(32, reset_less=True)
        held_ctrl = Signal(4,  reset_less=True)
        self.comb += [
            source.valid.eq(held),
            source.data.eq(Cat(held_data, sink.data)),
            source.ctrl.eq(Cat(held_ctrl, sink.ctrl)),
            source.words.eq(Mux(sink.valid, 2, 1)),
            sink.ready.eq(~held | source.ready),
        ]
        self.sync += [
            If(source.valid & source.ready,
                held.eq(0)
            ),
            If(sink.valid & sink.ready & ~(held & sink.valid),
                held.eq(1),
                held_data.eq(sink.data),
                held_ctrl.eq(sink.ctrl),
            )
        ]

class LogicalIdleFilter(LiteXModule):
    """Gen 1 Link words: Logical Idle words (00h data symbols) between the packets dropped, the packets
    words kept (Header Packet: SHP framing word and 4 words, Link Command: SLC framing word and 1 word,
    DPP: from the SDP framing word to the EPF of its END/EDB framing). Ex: Link Layer in a clock domain
    at half the PHY frequency, consuming one word per cycle outside of the DPP data. The framing
    Symbols/Logical Idle are decoded in a register stage (timings, one cycle of latency)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Decode stage.
        flags = ["idle", "shp", "slc", "sdp", "epf"]
        self.decode = decode = stream.Buffer([("data", 32), ("ctrl", 4)] + [(f, 1) for f in flags])
        def has(symbol):
            return Reduce("OR", [sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == symbol) for i in range(4)])
        self.comb += [
            sink.connect(decode.sink),
            decode.sink.idle.eq((sink.data == 0) & (sink.ctrl == 0)),
            decode.sink.shp.eq(has(SHP)),
            decode.sink.slc.eq(has(SLC)),
            decode.sink.sdp.eq(has(SDP)),
            decode.sink.epf.eq(has(EPF)),
        ]
        d = decode.source

        # Filter.
        count = Signal(3)
        drop  = Signal()
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            drop.eq(d.idle),
            If(d.valid & d.ready,
                If(d.shp,
                    NextValue(count, 4 - 1),
                    NextState("HP-LC")
                ).Elif(d.slc,
                    NextValue(count, 1 - 1),
                    NextState("HP-LC")
                ).Elif(d.sdp,
                    NextState("DPP")
                )
            )
        )
        fsm.act("HP-LC",
            If(d.valid & d.ready,
                NextValue(count, count - 1),
                If(count == 0, NextState("IDLE"))
            )
        )
        fsm.act("DPP",
            If(d.valid & d.ready & d.epf,
                NextState("IDLE")
            )
        )
        self.comb += [
            d.connect(source, omit={"valid", "ready", *flags}),
            source.valid.eq(d.valid & ~drop),
            d.ready.eq(source.ready | drop),
        ]

class PacketsBuffer(LiteXModule):
    """Store-and-forward of the packets (last: last entry of a packet): the entries of a packet are
    output once the whole packet is buffered, without gaps (ex: Link Layer words from a clock domain
    at half the PHY frequency: no Logical Idle inside the packets). depth: entries of the largest
    packet (DPH + 1024 bytes DPP)."""
    def __init__(self, layout, depth=256):
        self.sink   = sink   = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        self.drop   = Signal() # i: Drop the buffered entries.

        # # #

        self.fifo = fifo = stream.SyncFIFO(layout, depth, buffered=True)
        packets = Signal(max=depth + 1)
        packet_in  = Signal()
        packet_out = Signal()
        self.comb += [
            sink.connect(fifo.sink),
            packet_in.eq(sink.valid & sink.ready & sink.last),
            packet_out.eq(fifo.source.valid & fifo.source.ready & fifo.source.last),
            fifo.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(fifo.source.valid & (packets != 0) & ~self.drop),
            fifo.source.ready.eq((source.ready & (packets != 0)) | self.drop),
        ]
        self.sync += packets.eq(packets + packet_in - packet_out)

class WordsWindow(LiteXModule):
    """Link words pairs (gen2_link_layout, word 0 first) -> window of the next two words (w0/c0,
    w1/c1, n: 0-2 available words), consume: words consumed this cycle (0-n)."""
    def __init__(self):
        self.sink    = sink = stream.Endpoint(gen2_link_layout())
        self.n       = Signal(2)  # o
        self.w0      = Signal(32) # o
        self.c0      = Signal(4)  # o
        self.w1      = Signal(32) # o
        self.c1      = Signal(4)  # o
        self.consume = Signal(2)  # i

        # # #

        data  = [Signal(32, reset_less=True) for _ in range(4)]
        ctrl  = [Signal(4,  reset_less=True) for _ in range(4)]
        level = Signal(3)
        rem   = Signal(3)
        self.comb += [
            self.n.eq(Mux(level >= 2, 2, level)),
            self.w0.eq(data[0]), self.c0.eq(ctrl[0]),
            self.w1.eq(data[1]), self.c1.eq(ctrl[1]),
            rem.eq(level - self.consume),
            sink.ready.eq(rem <= 2),
        ]
        accept = Signal()
        self.comb += accept.eq(sink.valid & sink.ready)
        for i in range(4):
            shifted_d = Signal(32)
            shifted_c = Signal(4)
            self.comb += Case(self.consume, {c: [
                shifted_d.eq(data[i + c] if (i + c) < 4 else 0),
                shifted_c.eq(ctrl[i + c] if (i + c) < 4 else 0)] for c in range(3)})
            second = (accept & (sink.words == 2) & (rem == (i - 1))) if i > 0 else 0
            self.sync += If(accept & (rem == i),
                data[i].eq(sink.data[0:32]), ctrl[i].eq(sink.ctrl[0:4])
            ).Elif(second,
                data[i].eq(sink.data[32:64]), ctrl[i].eq(sink.ctrl[4:8])
            ).Else(
                data[i].eq(shifted_d), ctrl[i].eq(shifted_c)
            )
        self.sync += level.eq(rem + Mux(accept, sink.words, 0))

# Gen 2 RX Framing ---------------------------------------------------------------------------------

class Gen2RXFraming(LiteXModule):
    """Data Blocks bytes (8 per cycle, descrambled, bytes in transmission order from data[0:8]) to
    Link Layer words pairs: Idle Symbols between packets dropped, packets re-aligned on words, framing
    Symbols as Gen 1 K Symbols (ctrl), DPHSTART as HPSTART and Length Field Replicas removed, DPP
    words with the CRC-32/DPPEND (padded to the word).

    A packet is identified by its first framing Symbol (in Idle) and its length (DPP: Length Field
    Replica of the preceding DPH), a byte pointer re-aligns it in a 16-byte window: 8 bytes per
    cycle in a packet, 1 cycle per packet start. Unknown packet starts are skipped (errors).

    With lc_enable, LGOOD_n/LCRD_x Link Commands (both copies identical) are output on source_lc
    (Link Command word, Symbol 0 on data[0:8]) instead of source: they only concern the transmitter
    (Header Packets acknowledgements and credits), the Link Layer gets them without waiting behind
    the received packets (RX buffering, 7.2.4.1: Header Packets held in the Tx Header Buffers until
    LGOOD_n).
    """
    def __init__(self):
        self.sink      = sink      = stream.Endpoint([("data", 64)])
        self.source    = source    = stream.Endpoint(gen2_link_layout())
        self.source_lc = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.errors    = Signal(16) # o: Unknown packet starts (skipped bytes).
        self.lc_enable = Signal()   # i: LGOOD_n/LCRD_x on source_lc.

        # # #

        # Window: R (oldest 8 bytes) + incoming word -----------------------------------------------
        r       = Signal(64)
        r_valid = Signal()
        shift   = Signal() # Consume R (R <- incoming word).
        win     = Signal(128)
        avail   = Signal()
        self.comb += [
            win.eq(Cat(r, sink.data)),
            avail.eq(r_valid & sink.valid),
            sink.ready.eq(~r_valid | shift),
        ]
        self.sync += If(sink.valid & sink.ready,
            r.eq(sink.data),
            r_valid.eq(1),
        )

        # Packet start search (Idle) ---------------------------------------------------------------
        p     = Signal(3) # Byte pointer in R.
        q     = Signal(3) # First non-Idle byte at/after p.
        found = Signal()
        first = Signal(8)
        nonidle = Signal(8)
        for i in range(8):
            self.comb += nonidle[i].eq((win[8*i:8*(i+1)] != GEN2_IDLE) & (i >= p))
        self.comb += [
            found.eq(nonidle != 0),
            q.eq(7),
        ]
        for i in reversed(range(8)):
            self.comb += If(nonidle[i], q.eq(i))
        self.comb += first.eq((win >> byte_offset(q))[0:8])

        # Packet -----------------------------------------------------------------------------------
        kind    = Signal(3)
        length  = Signal(11) # Packet length (bytes).
        count   = Signal(11) # Packet bytes already output.
        rem     = Signal(11)
        last    = Signal()
        dpp_len = Signal(16) # Last DPH Length Field Replica.
        out     = Signal(64)
        self.comb += [
            out.eq((win >> byte_offset(p))[0:64]),
            rem.eq(length - count),
            last.eq(rem <= 8),
        ]

        # Output words: framing Symbols as K Symbols, bytes after the packet end padded (00h).
        start_k = Signal(8)
        self.comb += Case(kind, {
            KIND_HP  : start_k.eq(SHP),
            KIND_DPH : start_k.eq(SHP),
            KIND_LC  : start_k.eq(SLC),
            KIND_DPP : start_k.eq(SDP),
            KIND_EDB : start_k.eq(EDB),
        })
        for j in range(8):
            i      = count + j
            byte   = out[8*j:8*(j+1)]
            data   = source.data[8*j:8*(j+1)]
            ctrl   = source.ctrl[j]
            is_end = Signal(name=f"is_end{j}") # DPPEND/DPPABORT.
            self.comb += is_end.eq((kind == KIND_DPP) & (i >= (length - 4)))
            self.comb += If(i >= length,
                data.eq(0x00),
            ).Elif(i < 3,
                ctrl.eq(1), data.eq(start_k),
            ).Elif(i == 3,
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end & (i == (length - 1)),
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end,
                ctrl.eq(1), data.eq(Mux(byte == GEN2_EDB, EDB, END)),
            ).Else(
                data.eq(byte),
            )

        # LGOOD_n/LCRD_x Link Commands (Link Command Word copies identical, 7.2.2): source_lc.
        lc_word = out[32:64]
        lc_fast = Signal()
        self.comb += [
            lc_fast.eq(self.lc_enable & (kind == KIND_LC) & (lc_word[0:16] == lc_word[16:32]) & (
                ((lc_word[0:11] & 0x7f0) == 0x000) |  # LGOOD_n.
                ((lc_word[0:11] & 0x7f8) == 0x080))), # LCRD_x/LCRD1_x/LCRD2_x.
            source_lc.data.eq(lc_word),
        ]
        out_ready = Signal()
        self.comb += out_ready.eq(Mux(lc_fast, source_lc.ready, source.ready))

        # FSM --------------------------------------------------------------------------------------
        p_next = Signal(4)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(avail,
                If(found,
                    NextValue(p, q),
                    NextValue(count, 0),
                    If((first == GEN2_SHP),
                        NextValue(kind, KIND_HP),   NextValue(length, 20),
                        NextState("PACKET")
                    ).Elif(first == GEN2_DPHP,
                        NextValue(kind, KIND_DPH),  NextValue(length, 24),
                        NextState("PACKET")
                    ).Elif(first == GEN2_SLC,
                        NextValue(kind, KIND_LC),   NextValue(length, 8),
                        NextState("PACKET")
                    ).Elif(first == GEN2_SDP,
                        NextValue(kind, KIND_DPP),  NextValue(length, dpp_len + 12),
                        NextState("PACKET")
                    ).Elif(first == GEN2_EDB,
                        NextValue(kind, KIND_EDB),  NextValue(length, 4),
                        NextState("PACKET")
                    ).Else(
                        # Unknown start: skip the byte.
                        NextValue(self.errors, self.errors + 1),
                        If(q == 7,
                            shift.eq(1),
                            NextValue(p, 0)
                        ).Else(
                            NextValue(p, q + 1)
                        )
                    )
                ).Else(
                    shift.eq(1),
                    NextValue(p, 0)
                )
            )
        )
        self.comb += p_next.eq(p + rem[0:4])
        fsm.act("PACKET",
            If(avail,
                source.valid.eq(~lc_fast),
                source_lc.valid.eq(lc_fast),
                source.words.eq(Mux(rem > 4, 2, 1)),
                If((kind == KIND_DPH) & last,
                    source.words.eq(1), # Length Field Replicas removed.
                ),
                If(out_ready,
                    If((kind == KIND_DPH) & last,
                        NextValue(dpp_len, out[32:48]),
                    ),
                    NextValue(count, count + 8),
                    If(~last,
                        shift.eq(1)
                    ).Else(
                        NextState("IDLE"),
                        If(p_next >= 8,
                            shift.eq(1),
                            NextValue(p, p_next - 8)
                        ).Else(
                            NextValue(p, p_next)
                        )
                    )
                )
            )
        )

# Gen 2 TX Framing ---------------------------------------------------------------------------------

# Gen 1 K Symbol -> Gen 2 framing byte.
GEN2_FRAMING_BYTES = {SHP: GEN2_SHP, SDP: GEN2_SDP, EDB: GEN2_EDB, END: GEN2_END, SLC: GEN2_SLC,
    EPF: GEN2_EPF}

TYPE_DP = 0x08 # Data Packet Header Type (8.2).

def gen2_tx_unit_layout():
    """Transmission units bytes (8 per entry, bytes: 1-8 valid bytes, last: end of the unit)."""
    return [("data", 64), ("bytes", 4)]

class Gen2TXFramer(LiteXModule):
    """Link Layer words pairs (gen2_link_layout: Symbol 0 on data[0:8], K Symbols with ctrl, framing
    Ordered Sets at word boundaries, 1 or 2 words per entry) to transmission units bytes (8 per
    entry): Gen 2 framing bytes, DPHSTART and Length Field Replicas for the DPHs (non-deferred: a
    device does not defer), bytes after the DPPEND/DPPABORT removed. Units: Header Packet, Link
    Command, DPH with its DPP (no spacing allowed between them, 7.2.1.2.3).

    One word per cycle, two in the DPPs (data words): 8 bytes per cycle."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(gen2_link_layout())
        self.source = source = stream.Endpoint(gen2_tx_unit_layout())

        # # #

        # Words window -----------------------------------------------------------------------------
        self.window = window = WordsWindow()
        self.comb += sink.connect(window.sink)
        valid0 = Signal()
        data0  = window.w0
        ctrl0  = window.c0
        ready0 = Signal() # Word 0 consumed (when valid0).
        two    = Signal() # Words 0 and 1 consumed (DPP data, 8-byte chunk).
        self.comb += [
            valid0.eq(window.n != 0),
            window.consume.eq(Mux(two, 2, Mux(valid0 & ready0, 1, 0))),
        ]

        # Words -> bytes chunks (4 bytes, last chunk 1-4, 8 bytes in the DPPs) ---------------------
        chunk = stream.Endpoint([("data", 64), ("bytes", 4)])

        def mapped(data, ctrl, dphp=0):
            """Gen 2 bytes of a word (K Symbols to framing bytes)."""
            r = []
            for j in range(4):
                b = data[8*j:8*(j+1)]
                k = Signal(8)
                cases = {key: k.eq(value) for key, value in GEN2_FRAMING_BYTES.items()}
                cases["default"] = k.eq(b)
                self.comb += Case(b, cases)
                if j < 3:
                    k = Mux(dphp & (b == SHP), GEN2_DPHP, k)
                r.append(Mux(ctrl[j], k, b))
            return Cat(*r)

        is_dph   = Signal()
        dph_len  = Signal(16)
        n        = Signal(3) # Word index in the Header Packet.
        word0    = Signal(32)
        word0_c  = Signal(4)
        hpstart  = Signal()
        lcstart  = Signal()
        self.comb += [
            hpstart.eq((ctrl0 == 0b1111) & (data0[0:8] == SHP)),
            lcstart.eq((ctrl0 == 0b1111) & (data0[0:8] == SLC)),
        ]
        # DPPEND/DPPABORT end: EPF K Symbol after the DPPSTART word.
        epf  = Signal(4)
        for j in range(4):
            self.comb += epf[j].eq(ctrl0[j] & (data0[8*j:8*(j+1)] == EPF))
        epf_bytes = Signal(3)
        self.comb += If(epf[0], epf_bytes.eq(1)).Elif(epf[1], epf_bytes.eq(2)).Elif(epf[2],
            epf_bytes.eq(3)).Else(epf_bytes.eq(4))

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(valid0,
                If(hpstart,
                    # HPSTART held until DW0 (DPH or not).
                    ready0.eq(1),
                    NextValue(word0,   data0),
                    NextValue(word0_c, ctrl0),
                    NextState("HP-START")
                ).Elif(lcstart,
                    chunk.valid.eq(1),
                    chunk.data.eq(mapped(data0, ctrl0)),
                    chunk.bytes.eq(4),
                    ready0.eq(chunk.ready),
                    If(chunk.ready, NextState("LC"))
                ).Else(
                    # Unexpected word: dropped.
                    ready0.eq(1)
                )
            )
        )
        fsm.act("LC",
            chunk.valid.eq(valid0),
            chunk.data.eq(data0),
            chunk.bytes.eq(4),
            chunk.last.eq(1),
            ready0.eq(chunk.ready),
            If(valid0 & chunk.ready, NextState("IDLE"))
        )
        fsm.act("HP-START",
            If(valid0,
                chunk.valid.eq(1),
                chunk.data.eq(mapped(word0, word0_c, dphp=(data0[0:5] == TYPE_DP))),
                chunk.bytes.eq(4),
                If(chunk.ready,
                    NextValue(is_dph, data0[0:5] == TYPE_DP),
                    NextValue(n, 1),
                    NextState("HP")
                )
            )
        )
        fsm.act("HP",
            chunk.valid.eq(valid0),
            chunk.data.eq(data0),
            chunk.bytes.eq(4),
            chunk.last.eq((n == 4) & ~is_dph),
            ready0.eq(chunk.ready),
            If(valid0 & chunk.ready,
                NextValue(n, n + 1),
                If(n == 2, NextValue(dph_len, data0[16:32])), # DW1 Data Length.
                If(n == 4,
                    If(is_dph,
                        NextState("REPLICA")
                    ).Else(
                        NextState("IDLE")
                    )
                )
            )
        )
        fsm.act("REPLICA",
            # Length Field Replica x2.
            chunk.valid.eq(1),
            chunk.data.eq(Cat(dph_len, dph_len)),
            chunk.bytes.eq(4),
            If(chunk.ready, NextState("DPP-START"))
        )
        fsm.act("DPP-START",
            chunk.valid.eq(valid0),
            chunk.data.eq(mapped(data0, ctrl0)),
            chunk.bytes.eq(4),
            # Nullified DPP: DPPABORT right after the DPH.
            chunk.last.eq(data0[0:8] == EDB),
            ready0.eq(chunk.ready),
            If(valid0 & chunk.ready,
                If(data0[0:8] == EDB,
                    NextState("IDLE")
                ).Else(
                    NextState("DPP")
                )
            )
        )
        dpp_two = Signal() # Two data words available (no K Symbols): 8-byte chunk.
        self.comb += dpp_two.eq((window.n == 2) & (ctrl0 == 0) & (window.c1 == 0))
        fsm.act("DPP",
            chunk.valid.eq(valid0),
            If(dpp_two,
                chunk.data.eq(Cat(data0, window.w1)),
                chunk.bytes.eq(8),
                two.eq(chunk.ready),
            ).Else(
                chunk.data.eq(mapped(data0, ctrl0)),
                chunk.bytes.eq(epf_bytes),
                chunk.last.eq(epf != 0),
                ready0.eq(chunk.ready),
            ),
            If(valid0 & chunk.ready & ~dpp_two & (epf != 0), NextState("IDLE"))
        )

        # Chunks -> 8-byte entries (4-byte chunks paired, 8-byte chunks never last) ----------------
        held   = Signal(32)
        held_v = Signal()
        wide   = Signal()
        self.comb += wide.eq(chunk.bytes == 8)
        self.comb += [
            If(~held_v,
                # 8-byte chunk, first 4-byte chunk of a pair (held), or a single last chunk.
                If(wide | chunk.last,
                    source.valid.eq(chunk.valid),
                    source.data.eq(chunk.data),
                    source.bytes.eq(chunk.bytes),
                    source.last.eq(chunk.last),
                    chunk.ready.eq(source.ready),
                ).Else(
                    chunk.ready.eq(1),
                )
            ).Else(
                source.valid.eq(chunk.valid),
                source.data.eq(Cat(held, chunk.data[0:32])),
                source.bytes.eq(Mux(wide, 8, 4 + chunk.bytes)),
                source.last.eq(chunk.last),
                chunk.ready.eq(source.ready),
            )
        ]
        self.sync += If(chunk.valid & chunk.ready,
            If(held_v & wide,
                held.eq(chunk.data[32:64]), # Upper half held (held_v kept).
            ).Elif(~held_v & ~chunk.last & ~wide,
                held.eq(chunk.data[0:32]),
                held_v.eq(1),
            ).Else(
                held_v.eq(0),
            )
        )

class Gen2TXPacker(LiteXModule):
    """Transmission units bytes (8 per entry) to Data Blocks payloads (16 bytes, bytes in
    transmission order from data[0:8]) for Gen2TX: units packed back to back, Idle Symbols (5Ah)
    after a unit when the next one is not available (last: Data Block ending a unit, SKP Ordered
    Sets allowed after it). A unit must be fully available when started (units_available: the
    Data Blocks of a unit are transmitted without Idle Symbols, 7.2.1.2.2).

    Up to 8 bytes read per cycle, a Data Block every 3 cycles at most (Gen2TX at 40-bit)."""
    def __init__(self):
        self.sink            = sink   = stream.Endpoint(gen2_tx_unit_layout())
        self.source          = source = stream.Endpoint([("data", 128)])
        self.units_available = Signal() # i: A complete unit is available on sink.
        self.unit_end        = Signal() # o: A unit is read (pulse, last entry).
        self.flush           = Signal() # i: Discard the sink entries (link not in U0).
        self.skp_pending     = Signal() # i: SKP Ordered Set pending (Gen2TX): the Data Block ending
                                        #    a unit ends there (Idle Symbols, last) instead of packing
                                        #    the next unit (SKP Ordered Sets only between packets).

        # # #

        buf     = Signal(8*24) # Bytes (oldest from buf[0:8]).
        fill    = Signal(5)    # Bytes in buf (0-24).
        in_unit = Signal()     # Reading a unit.
        ending  = Signal()     # Block with a unit end and Idle Symbols.
        hold    = Signal()     # Unit ended with a SKP Ordered Set pending: next unit after the block.

        # Output: a block when 16 bytes are buffered, or a unit end padded with Idle Symbols.
        out_fire = Signal()
        idle     = Signal(8*16)
        self.comb += idle.eq(int.from_bytes(bytes([GEN2_IDLE])*16, "little"))
        pad_mask = Signal(8*16)
        for i in range(16):
            self.comb += pad_mask[8*i:8*(i+1)].eq(Replicate(i >= fill, 8))
        self.comb += [
            If(fill >= 16,
                source.valid.eq(1),
                source.data.eq(buf[0:128]),
            ).Elif(ending & (fill != 0),
                source.valid.eq(1),
                source.data.eq((buf[0:128] & ~pad_mask) | (idle & pad_mask)),
                source.last.eq(1),
            ),
            # Block ending exactly at a unit end.
            If(ending & (fill == 16), source.last.eq(1)),
            out_fire.eq(source.valid & source.ready),
        ]

        # Input: entries placed at the fill level (after the output shift).
        # A unit is started in a partially filled block only when its first entry completes the
        # block: else, a block requested before its next entry (Gen2TX, a block every 3 cycles or
        # more) would find no block and send a Logical Idle Data Block inside the unit. The padded
        # block (Idle Symbols) is sent first.
        fill_eff = Signal(5)
        start_ok = Signal()
        rd       = Signal()
        self.comb += [
            fill_eff.eq(Mux(out_fire, Mux(fill >= 16, fill - 16, 0), fill)),
            start_ok.eq((fill_eff == 0) | ((fill_eff + sink.bytes) >= 16)),
            rd.eq(sink.valid & ~self.flush &
                (in_unit | (self.units_available & ~hold & start_ok)) &
                (fill_eff <= 16)),
            sink.ready.eq(rd | self.flush),
            self.unit_end.eq(sink.valid & sink.ready & sink.last),
        ]
        buf_shift = Signal(8*24)
        self.comb += buf_shift.eq(Mux(out_fire, buf >> 128, buf))
        placed = Signal(8*24)
        self.comb += Case(fill_eff, {f: placed.eq((buf_shift & ((1 << (8*f)) - 1)) | (sink.data << (8*f)))
            for f in range(17)})
        self.sync += [
            If(self.flush,
                fill.eq(0),
                in_unit.eq(0),
            ).Elif(rd,
                buf.eq(placed),
                fill.eq(fill_eff + sink.bytes),
                in_unit.eq(~sink.last),
            ).Elif(out_fire,
                buf.eq(buf_shift),
                fill.eq(fill_eff),
            ),
            # A unit ended: its last block is padded with Idle Symbols unless the next unit follows.
            If(self.flush | (out_fire & (fill <= 16)),
                hold.eq(0)
            ).Elif(rd & sink.last & self.skp_pending,
                hold.eq(1)
            ),
            If(self.flush,
                ending.eq(0)
            ).Elif(rd & sink.last,
                ending.eq(1)
            ).Elif(rd,
                ending.eq(0)
            ).Elif(out_fire & (fill < 16),
                ending.eq(0)
            ),
        ]

# Gen 2 Link TX ------------------------------------------------------------------------------------

class Gen2LinkTX(LiteXModule):
    """Link Layer words (sys clock domain) -> Gen2TXFramer -> units buffer (sys -> tx) ->
    Gen2TXPacker -> Data Blocks payloads (tx clock domain, to Gen2TX).

    Units are stored and forwarded: a unit is started once completely written in the buffer and
    visible on its read side. Flushed while the link is not in U0 (stale units of a previous U0
    period, units cut by a link state change). The units written count is moved to the tx clock domain (gray coded)
    and delayed to cover the buffer CDC latency (else the packer could start a unit whose last
    entries are not yet readable: Idle Symbols inside a packet).
    """
    def __init__(self, buffer_depth=512, units_delay=8, link_words=1):
        assert link_words in [1, 2]
        # Link Layer words (link_words=1) or words pairs (link_words=2, gen2_link_layout).
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)] if link_words == 1 else
            gen2_link_layout())
        self.source = source = stream.Endpoint([("data", 128)])
        self.flush  = Signal() # i: Flush (sys clock domain, link not in U0): framer reset, buffer drained.

        # # #

        self.framer = framer = ResetInserter()(Gen2TXFramer())
        flush_tx = Signal()
        self.specials += MultiReg(self.flush, flush_tx, "tx")
        self.comb += framer.reset.eq(self.flush)
        self.buffer = buffer = ClockDomainsRenamer({"write": "sys", "read": "tx"})(
            stream.AsyncFIFO(gen2_tx_unit_layout(), buffer_depth, buffered=True))
        self.packer = packer = ClockDomainsRenamer("tx")(Gen2TXPacker())
        self.comb += [
            sink.connect(framer.sink, omit={"words"}),
            framer.sink.words.eq(sink.words if link_words == 2 else 1),
            framer.source.connect(buffer.sink),
            buffer.source.connect(packer.sink),
            packer.source.connect(source),
        ]

        # Complete units in the buffer: units written (sys) vs read (tx), units cut by a flush are not
        # counted (no last entry).
        units_written = GrayCounter(16)
        units_read    = ClockDomainsRenamer("tx")(GrayCounter(16))
        self.submodules += units_written, units_read
        units_written_tx = Signal(16)
        self.specials += MultiReg(units_written.q, units_written_tx, "tx")
        units_written_d = [Signal(16) for _ in range(units_delay)]
        self.sync.tx += [units_written_d[0].eq(units_written_tx)] + [
            units_written_d[i].eq(units_written_d[i-1]) for i in range(1, units_delay)]
        self.comb += [
            units_written.ce.eq(buffer.sink.valid & buffer.sink.ready & buffer.sink.last),
            units_read.ce.eq(packer.unit_end),
            packer.units_available.eq(units_written_d[-1] != units_read.q),
            packer.flush.eq(flush_tx),
        ]

# Gen 2 Link RX ------------------------------------------------------------------------------------

class Gen2LinkRX(LiteXModule):
    """Data Blocks (rx clock domain, one every 3.3 cycles at 40-bit, not held) -> blocks buffer ->
    8 bytes per cycle -> Gen2RXFraming (reset out of the Data Stream) -> words pairs buffer (rx ->
    sys) -> Link Layer words (sys clock domain, gated by rx_ready: U0). LGOOD_n/LCRD_x Link Commands
    on source_lc (Link Command words buffer, rx -> sys) with lc_enable."""
    def __init__(self, buffer_depth=1024, link_words=1):
        assert link_words in [1, 2]
        self.sink        = sink   = stream.Endpoint([("data", 128)])         # rx clock domain.
        # Link Layer words (link_words=1) or words pairs (link_words=2, gen2_link_layout).
        self.source      = source = stream.Endpoint([("data", 32), ("ctrl", 4)] if link_words == 1
            else gen2_link_layout())
        self.source_lc   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.lc_enable   = Signal() # i: LGOOD_n/LCRD_x on source_lc (sys clock domain, static).
        self.data_stream = Signal() # i: Receiving the Data Stream (rx clock domain).
        self.rx_ready    = Signal() # i: U0 (sys clock domain).
        self.overflow    = Signal() # o: Blocks buffer overflow (sticky, rx clock domain).

        # # #

        fifo      = ClockDomainsRenamer("rx")(stream.SyncFIFO([("data", 128)], 8))
        converter = ClockDomainsRenamer("rx")(stream.Converter(128, 64))
        framing   = ClockDomainsRenamer("rx")(ResetInserter()(Gen2RXFraming()))
        buffer    = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
            stream.AsyncFIFO(gen2_link_layout(), buffer_depth, buffered=True))
        self.submodules += fifo, converter, framing, buffer
        self.framing = framing
        self.buffer  = buffer
        self.comb += [
            fifo.sink.valid.eq(sink.valid),
            fifo.sink.data.eq(sink.data),
            sink.ready.eq(1),
            fifo.source.connect(converter.sink),
            converter.source.connect(framing.sink),
            framing.reset.eq(~self.data_stream),
            framing.source.connect(buffer.sink),
        ]
        self.sync.rx += If(fifo.sink.valid & ~fifo.sink.ready, self.overflow.eq(1))

        # Link Commands words (rx -> sys), gated by rx_ready.
        self.specials += MultiReg(self.lc_enable, framing.lc_enable, "rx")
        self.lc_buffer = lc_buffer = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
            stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 16, buffered=True))
        self.comb += [
            framing.source_lc.connect(lc_buffer.sink),
            self.source_lc.valid.eq(lc_buffer.source.valid & self.rx_ready),
            self.source_lc.data.eq(lc_buffer.source.data),
            lc_buffer.source.ready.eq(~self.rx_ready | self.source_lc.ready),
        ]

        # Words pairs, gated by rx_ready.
        if link_words == 2:
            self.comb += [
                buffer.source.connect(source, omit={"valid", "ready"}),
                source.valid.eq(buffer.source.valid & self.rx_ready),
                buffer.source.ready.eq(~self.rx_ready | source.ready),
            ]
            return

        # Words pairs -> words, gated by rx_ready.
        second = Signal()
        self.comb += [
            source.valid.eq(buffer.source.valid & self.rx_ready),
            source.data.eq(Mux(second, buffer.source.data[32:64], buffer.source.data[0:32])),
            source.ctrl.eq(Mux(second, buffer.source.ctrl[4:8],   buffer.source.ctrl[0:4])),
            buffer.source.ready.eq(~self.rx_ready |
                (source.ready & (second | (buffer.source.words == 1)))),
        ]
        self.sync += If(~self.rx_ready,
            second.eq(0)
        ).Elif(source.valid & source.ready,
            second.eq(~second & (buffer.source.words == 2))
        )

