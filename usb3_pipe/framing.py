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

def gen2_link_layout(n=2):
    """Link words pairs (Symbol 0 on data[0:8]), words: 1 or 2 valid words (n words: 1-n)."""
    return [("data", 32*n), ("ctrl", 4*n), ("words", bits_for(n))]

class WordsNarrower(LiteXModule):
    """Link words entries (gen2_link_layout(n_in), 1-n_in words) -> entries of up to n_out words
    (gen2_link_layout(n_out), or link words with n_out=1 and words=False)."""
    def __init__(self, n_in, n_out, words=True):
        assert n_in % n_out == 0
        self.sink   = sink   = stream.Endpoint(gen2_link_layout(n_in))
        self.source = source = stream.Endpoint(gen2_link_layout(n_out) if words else
            [("data", 32), ("ctrl", 4)])

        # # #

        steps = n_in//n_out
        index = Signal(max=max(steps, 2))
        rem   = Signal(bits_for(n_in))
        done  = Signal()
        self.comb += [
            rem.eq(sink.words - index*n_out),
            done.eq(rem <= n_out),
            source.valid.eq(sink.valid),
            Case(index, {i: [
                source.data.eq(sink.data[32*n_out*i:32*n_out*(i+1)]),
                source.ctrl.eq(sink.ctrl[4*n_out*i:4*n_out*(i+1)]),
            ] for i in range(steps)}),
            sink.ready.eq(source.ready & done),
        ]
        if words:
            self.comb += source.words.eq(Mux(done, rem, n_out))
        self.sync += If(source.valid & source.ready,
            If(done, index.eq(0)).Else(index.eq(index + 1))
        )

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
    w1/c1, n: 0-2 available words), consume: words consumed this cycle (0-n).

    decode: optional function (data, ctrl) -> nflags-bit flags of a word, computed on the incoming
    words and stored with them (f0/f1: registered decodes for the window consumer, timings)."""
    def __init__(self, decode=None, nflags=0):
        self.sink    = sink = stream.Endpoint(gen2_link_layout())
        self.n       = Signal(2)  # o
        self.w0      = Signal(32) # o
        self.c0      = Signal(4)  # o
        self.w1      = Signal(32) # o
        self.c1      = Signal(4)  # o
        self.f0      = Signal(max(nflags, 1)) # o
        self.f1      = Signal(max(nflags, 1)) # o
        self.consume = Signal(2)  # i

        # # #

        cw    = 4 + nflags # ctrl + flags.
        data  = [Signal(32, reset_less=True) for _ in range(4)]
        ctrl  = [Signal(cw, reset_less=True) for _ in range(4)]
        level = Signal(3)
        rem   = Signal(3)
        self.comb += [
            self.w0.eq(data[0]), self.c0.eq(ctrl[0][:4]),
            self.w1.eq(data[1]), self.c1.eq(ctrl[1][:4]),
            rem.eq(level - self.consume),
            sink.ready.eq(rem <= 2),
        ]
        if nflags:
            self.comb += [
                self.f0.eq(ctrl[0][4:]),
                self.f1.eq(ctrl[1][4:]),
            ]
        in_ctrl = []
        for w in range(2):
            d, c = sink.data[32*w:32*(w+1)], sink.ctrl[4*w:4*(w+1)]
            in_ctrl.append(Cat(c, decode(d, c)) if nflags else c)
        accept = Signal()
        self.comb += accept.eq(sink.valid & sink.ready)
        for i in range(4):
            shifted_d = Signal(32)
            shifted_c = Signal(cw)
            self.comb += Case(self.consume, {c: [
                shifted_d.eq(data[i + c] if (i + c) < 4 else 0),
                shifted_c.eq(ctrl[i + c] if (i + c) < 4 else 0)] for c in range(3)})
            second = (accept & (sink.words == 2) & (rem == (i - 1))) if i > 0 else 0
            self.sync += If(accept & (rem == i),
                data[i].eq(sink.data[0:32]), ctrl[i].eq(in_ctrl[0])
            ).Elif(second,
                data[i].eq(sink.data[32:64]), ctrl[i].eq(in_ctrl[1])
            ).Else(
                data[i].eq(shifted_d), ctrl[i].eq(shifted_c)
            )
        level_next = Signal(3)
        self.comb += level_next.eq(rem + Mux(accept, sink.words, 0))
        self.sync += [
            level.eq(level_next),
            # Available words registered (timings).
            self.n.eq(Mux(level_next >= 2, 2, level_next)),
        ]

# Gen 2 RX Framing ---------------------------------------------------------------------------------

class Gen2RXFraming(LiteXModule):
    """Data Blocks bytes (nbytes per cycle: 8 or 16, descrambled, bytes in transmission order from
    data[0:8]) to Link Layer words entries (gen2_link_layout(nbytes//4): words pairs at 8 bytes per
    cycle): Idle Symbols between packets dropped, packets re-aligned on words, framing
    Symbols as Gen 1 K Symbols (ctrl), DPHSTART as HPSTART and Length Field Replicas removed, DPP
    words with the CRC-32/DPPEND (padded to the word).

    A packet is identified by its first framing Symbol (in Idle) and its length (DPP: Length Field
    Replica of the preceding DPH), a byte pointer re-aligns it in a 2*nbytes window: nbytes per
    cycle in a packet, 1 cycle per packet start. Unknown packet starts are skipped (errors). 16
    bytes per cycle for wider transceiver words (ex: 80-bit at 125MHz, a Data Block every 1.65
    cycles).

    With lc_enable, LGOOD_n/LCRD_x Link Commands (both copies identical) are output on source_lc
    (Link Command word, Symbol 0 on data[0:8]) instead of source: they only concern the transmitter
    (Header Packets acknowledgements and credits), the Link Layer gets them without waiting behind
    the received packets (RX buffering, 7.2.4.1: Header Packets held in the Tx Header Buffers until
    LGOOD_n).
    """
    def __init__(self, nbytes=8):
        assert nbytes in [8, 16]
        pw = log2_int(nbytes)
        self.sink      = sink      = stream.Endpoint([("data", 8*nbytes)])
        self.source    = source    = stream.Endpoint(gen2_link_layout(nbytes//4))
        self.source_lc = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.errors    = Signal(16) # o: Unknown packet starts (skipped bytes).
        self.lc_enable = Signal()   # i: LGOOD_n/LCRD_x on source_lc.

        # # #

        # Input stage: words with their bytes start codes (packet kind + 1, 0: Idle, CODE_UNKNOWN) ---
        CODE_UNKNOWN = 7
        in_valid = Signal()
        in_data  = Signal(8*nbytes, reset_less=True)
        in_codes = Signal(3*nbytes, reset_less=True)
        in_ready = Signal()
        codes = Signal(3*nbytes)
        for i in range(nbytes):
            b = sink.data[8*i:8*(i+1)]
            self.comb += Case(b, {
                GEN2_IDLE : codes[3*i:3*(i+1)].eq(0),
                GEN2_SHP  : codes[3*i:3*(i+1)].eq(KIND_HP  + 1),
                GEN2_DPHP : codes[3*i:3*(i+1)].eq(KIND_DPH + 1),
                GEN2_SLC  : codes[3*i:3*(i+1)].eq(KIND_LC  + 1),
                GEN2_SDP  : codes[3*i:3*(i+1)].eq(KIND_DPP + 1),
                GEN2_EDB  : codes[3*i:3*(i+1)].eq(KIND_EDB + 1),
                "default" : codes[3*i:3*(i+1)].eq(CODE_UNKNOWN),
            })
        self.comb += sink.ready.eq(~in_valid | in_ready)
        self.sync += If(sink.ready,
            in_valid.eq(sink.valid),
            in_data.eq(sink.data),
            in_codes.eq(codes),
        )

        # Window: R (oldest nbytes) + incoming word ------------------------------------------------
        # For each position i of R, the first non-Idle byte at/after i (found, index, start code) is
        # computed on the incoming word and registered with R: the packet start search is then a
        # selection by the byte pointer.
        r       = Signal(8*nbytes, reset_less=True)
        r_edb   = Signal(nbytes, reset_less=True) # EDB bytes (DPPABORT).
        r_valid = Signal()
        shift   = Signal() # Consume R (R <- incoming word).
        win     = Signal(16*nbytes)
        avail   = Signal()
        self.comb += [
            win.eq(Cat(r, in_data)),
            avail.eq(r_valid & in_valid),
            in_ready.eq(~r_valid | shift),
        ]
        nxt_found = [Signal(name=f"nxt_found{i}") for i in range(nbytes)]
        nxt_q     = [Signal(pw, name=f"nxt_q{i}") for i in range(nbytes)]
        nxt_code  = [Signal(3, name=f"nxt_code{i}") for i in range(nbytes)]
        r_found   = [Signal(name=f"r_found{i}", reset_less=True) for i in range(nbytes)]
        r_q       = [Signal(pw, name=f"r_q{i}", reset_less=True) for i in range(nbytes)]
        r_code    = [Signal(3, name=f"r_code{i}", reset_less=True) for i in range(nbytes)]
        code = lambda j: in_codes[3*j:3*(j+1)]
        in_edb = Signal(nbytes)
        self.comb += [in_edb[j].eq(code(j) == (KIND_EDB + 1)) for j in range(nbytes)]
        for i in range(nbytes):
            self.comb += [
                nxt_found[i].eq(Reduce("OR", [code(j) != 0 for j in range(i, nbytes)])),
                nxt_q[i].eq(nbytes - 1),
                nxt_code[i].eq(code(nbytes - 1)),
            ]
            for j in reversed(range(i, nbytes)):
                self.comb += If(code(j) != 0, nxt_q[i].eq(j), nxt_code[i].eq(code(j)))
        self.sync += If(in_valid & in_ready,
            r.eq(in_data),
            r_edb.eq(in_edb),
            r_valid.eq(1),
            [r_found[i].eq(nxt_found[i]) for i in range(nbytes)],
            [r_q[i].eq(nxt_q[i]) for i in range(nbytes)],
            [r_code[i].eq(nxt_code[i]) for i in range(nbytes)],
        )

        # Packet start search (Idle) ---------------------------------------------------------------
        p     = Signal(pw) # Byte pointer in R.
        q     = Signal(pw) # First non-Idle byte at/after p.
        found = Signal()
        first = Signal(3)  # Start code of the byte at q.
        self.comb += [
            found.eq(Array(r_found)[p]),
            q.eq(Array(r_q)[p]),
            first.eq(Array(r_code)[p]),
        ]

        # Packet -----------------------------------------------------------------------------------
        kind      = Signal(3)
        rem       = Signal(11) # Packet bytes remaining (from the current chunk).
        start     = Signal()   # First chunk of the packet.
        last      = Signal()
        dpp_len12 = Signal(11, reset=12) # Last DPH Length Field Replica + 12 (DPP length).
        out       = Signal(8*nbytes)
        out_edb   = Signal(nbytes)
        self.comb += [
            out.eq((win >> byte_offset(p))[0:8*nbytes]),
            out_edb.eq((Cat(r_edb, in_edb) >> p)[0:nbytes]),
            last.eq(rem <= nbytes),
        ]

        # Output words (registered with 16 bytes per cycle, see the Link Commands): framing Symbols as
        # K Symbols, bytes after the packet end padded (00h).
        if nbytes == 16:
            output = stream.Endpoint(gen2_link_layout(nbytes//4) + [("lc", 1)])
        else:
            output = source
        start_k = Signal(8)
        self.comb += Case(kind, {
            KIND_HP  : start_k.eq(SHP),
            KIND_DPH : start_k.eq(SHP),
            KIND_LC  : start_k.eq(SLC),
            KIND_DPP : start_k.eq(SDP),
            KIND_EDB : start_k.eq(EDB),
        })
        # Byte j of the chunk is packet byte i: i >= length <-> j >= rem (comparisons with constants).
        for j in range(nbytes):
            byte   = out[8*j:8*(j+1)]
            data   = output.data[8*j:8*(j+1)]
            ctrl   = output.ctrl[j]
            is_end = Signal(name=f"is_end{j}") # DPPEND/DPPABORT.
            self.comb += is_end.eq((kind == KIND_DPP) & ((j + 4) >= rem))
            self.comb += If(j >= rem,
                data.eq(0x00),
            ).Elif(start & (j < 3),
                ctrl.eq(1), data.eq(start_k),
            ).Elif(start & (j == 3),
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end & ((j + 1) == rem),
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end,
                ctrl.eq(1), data.eq(Mux(out_edb[j], EDB, END)),
            ).Else(
                data.eq(byte),
            )

        # LGOOD_n/LCRD_x Link Commands (Link Command Word copies identical, 7.2.2): source_lc. With
        # 16 bytes per cycle, decoded on the registered output (timings).
        def is_fast_lc(lc_word):
            return (lc_word[0:16] == lc_word[16:32]) & (
                ((lc_word[0:11] & 0x7f0) == 0x000) |  # LGOOD_n.
                ((lc_word[0:11] & 0x7f8) == 0x080))   # LCRD_x/LCRD1_x/LCRD2_x.
        lc_fast   = Signal()
        out_ready = Signal()
        if nbytes == 16:
            self.out_buffer = out_buffer = stream.Buffer(gen2_link_layout(nbytes//4) + [("lc", 1)],
                pipe_valid=True, pipe_ready=True)
            fast = Signal()
            self.comb += [
                output.lc.eq(kind == KIND_LC),
                output.connect(out_buffer.sink),
                out_ready.eq(output.ready),
                fast.eq(self.lc_enable & out_buffer.source.lc &
                    is_fast_lc(out_buffer.source.data[32:64])),
                out_buffer.source.connect(source, omit={"valid", "ready", "lc"}),
                source.valid.eq(out_buffer.source.valid & ~fast),
                source_lc.valid.eq(out_buffer.source.valid & fast),
                source_lc.data.eq(out_buffer.source.data[32:64]),
                out_buffer.source.ready.eq(Mux(fast, source_lc.ready, source.ready)),
            ]
        else:
            lc_word = out[32:64]
            self.comb += [
                lc_fast.eq(self.lc_enable & (kind == KIND_LC) & is_fast_lc(lc_word)),
                source_lc.data.eq(lc_word),
                # Both outputs ready (not depending on the Link Command decode: timings).
                out_ready.eq(source_lc.ready & source.ready),
            ]

        # FSM --------------------------------------------------------------------------------------
        p_next = Signal(pw + 1)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(avail,
                If(found,
                    NextValue(p, q),
                    NextValue(start, 1),
                    NextValue(kind, first - 1),
                    Case(first, {
                        KIND_HP  + 1 : NextValue(rem, 20),
                        KIND_DPH + 1 : NextValue(rem, 24),
                        KIND_LC  + 1 : NextValue(rem, 8),
                        KIND_DPP + 1 : NextValue(rem, dpp_len12),
                        KIND_EDB + 1 : NextValue(rem, 4),
                    }),
                    If(first != CODE_UNKNOWN,
                        NextState("PACKET")
                    ).Else(
                        # Unknown start: skip the byte.
                        NextValue(self.errors, self.errors + 1),
                        If(q == (nbytes - 1),
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
        self.comb += p_next.eq(p + rem[0:pw + 1])
        fsm.act("PACKET",
            If(avail,
                output.valid.eq(~lc_fast),
                source_lc.valid.eq(lc_fast) if nbytes != 16 else [],
                # Valid words: rem bytes (up to nbytes) rounded up to words.
                output.words.eq(Mux(last, (rem[0:pw + 1] + 3) >> 2, nbytes//4)),
                If((kind == KIND_DPH) & last,
                    output.words.eq(1), # Length Field Replicas removed.
                ),
                If(out_ready,
                    If((kind == KIND_DPH) & last,
                        NextValue(dpp_len12, out[32:48] + 12),
                    ),
                    NextValue(rem, rem - nbytes),
                    NextValue(start, 0),
                    If(~last,
                        shift.eq(1)
                    ).Else(
                        NextState("IDLE"),
                        If(p_next >= nbytes,
                            shift.eq(1),
                            NextValue(p, p_next - nbytes)
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

def gen2_tx_unit_layout(nbytes=8):
    """Transmission units bytes (nbytes per entry, bytes: 1-nbytes valid bytes, last: end of the
    unit)."""
    return [("data", 8*nbytes), ("bytes", bits_for(nbytes))]

class Gen2TXUnitsWidener(LiteXModule):
    """Transmission units entries: 8 bytes -> 16 bytes (entries of a unit paired, the entries before
    the last one of a unit are full: 8 bytes)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(gen2_tx_unit_layout(8))
        self.source = source = stream.Endpoint(gen2_tx_unit_layout(16))

        # # #

        held   = Signal()
        held_d = Signal(64, reset_less=True)
        self.comb += [
            If(held,
                # Held full entry + entry: 16-byte entry.
                source.valid.eq(sink.valid),
                source.data.eq(Cat(held_d, sink.data)),
                source.bytes.eq(8 + sink.bytes),
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ).Elif(sink.last,
                # Single last entry.
                source.valid.eq(sink.valid),
                source.data.eq(sink.data),
                source.bytes.eq(sink.bytes),
                source.last.eq(1),
                sink.ready.eq(source.ready),
            ).Else(
                sink.ready.eq(1),
            )
        ]
        self.sync += If(sink.valid & sink.ready,
            held.eq(~held & ~sink.last),
            held_d.eq(sink.data),
        )

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
        # Words decodes stored with the words (timings): HPSTART, LCSTART, EPF K Symbols, Symbol 0
        # EDB, DP Type (DW0), data word.
        def decode(data, ctrl):
            return Cat(
                (ctrl == 0b1111) & (data[0:8] == SHP),
                (ctrl == 0b1111) & (data[0:8] == SLC),
                *[ctrl[j] & (data[8*j:8*(j+1)] == EPF) for j in range(4)],
                data[0:8] == EDB,
                data[0:5] == TYPE_DP,
                ctrl == 0)
        self.window = window = WordsWindow(decode, nflags=9)
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
        hpstart  = window.f0[0]
        lcstart  = window.f0[1]
        # DPPEND/DPPABORT end: EPF K Symbol after the DPPSTART word.
        epf      = window.f0[2:6]
        edb0     = window.f0[6] # Symbol 0 EDB.
        type_dp  = window.f0[7] # DW0 Type: DP.
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
                chunk.data.eq(mapped(word0, word0_c, dphp=type_dp)),
                chunk.bytes.eq(4),
                If(chunk.ready,
                    NextValue(is_dph, type_dp),
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
            chunk.last.eq(edb0),
            ready0.eq(chunk.ready),
            If(valid0 & chunk.ready,
                If(edb0,
                    NextState("IDLE")
                ).Else(
                    NextState("DPP")
                )
            )
        )
        dpp_two = Signal() # Two data words available (no K Symbols): 8-byte chunk.
        self.comb += dpp_two.eq((window.n == 2) & window.f0[8] & window.f1[8])
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
        # Registered valid/ready (timings: window/FSM decisions to the pairing).
        self.chunk_buffer = chunk_buffer = stream.Buffer([("data", 64), ("bytes", 4)],
            pipe_valid=True, pipe_ready=True)
        self.comb += chunk.connect(chunk_buffer.sink)
        chunk  = chunk_buffer.source
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
    """Transmission units bytes (nbytes per entry: 8 or 16) to Data Blocks payloads (16 bytes, bytes in
    transmission order from data[0:8]) for Gen2TX: units packed back to back, Idle Symbols (5Ah)
    after a unit when the next one is not available (last: Data Block ending a unit, SKP Ordered
    Sets allowed after it). A unit must be fully available when started (units_available: the
    Data Blocks of a unit are transmitted without Idle Symbols, 7.2.1.2.2).

    Up to nbytes read per cycle: a Data Block every 3 cycles at most with 8 (Gen2TX at 40-bit),
    every cycle with 16 (Gen2TX at 80-bit: a Data Block every 1.65 cycles)."""
    def __init__(self, nbytes=8):
        assert nbytes in [8, 16]
        self.sink            = sink   = stream.Endpoint(gen2_tx_unit_layout(nbytes))
        self.source          = source = stream.Endpoint([("data", 128)])
        self.units_available = Signal() # i: A complete unit is available on sink.
        self.unit_end        = Signal() # o: A unit is read (pulse, last entry).
        self.flush           = Signal() # i: Discard the sink entries (link not in U0).
        self.skp_pending     = Signal() # i: SKP Ordered Set pending (Gen2TX): the Data Block ending
                                        #    a unit ends there (Idle Symbols, last) instead of packing
                                        #    the next unit (SKP Ordered Sets only between packets).

        # # #

        buf     = Signal(8*(16 + nbytes)) # Bytes (oldest from buf[0:8]).
        fill    = Signal(bits_for(16 + nbytes)) # Bytes in buf (0-(16 + nbytes)).
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
        # Both cases (output block taken this cycle or not) computed from the registered state, then
        # selected with out_fire (timings: placement/read decisions not after the output handshake).
        fill_eff = Signal(len(fill))
        start_ok = Signal()
        rd       = Signal()
        fill_fire = Signal(len(fill)) # Fill level after an output block.
        self.comb += fill_fire.eq(Mux(fill >= 16, fill - 16, 0))
        def read_ok(f):
            start = (f == 0) | ((f + sink.bytes) >= 16)
            return sink.valid & ~self.flush & (in_unit | (self.units_available & ~hold & start)) & (f <= 16)
        rd_keep = Signal()
        rd_fire = Signal()
        self.comb += [
            rd_keep.eq(read_ok(fill)),
            rd_fire.eq(read_ok(fill_fire)),
            fill_eff.eq(Mux(out_fire, fill_fire, fill)),
            rd.eq(Mux(out_fire, rd_fire, rd_keep)),
            sink.ready.eq(rd | self.flush),
            self.unit_end.eq(sink.valid & sink.ready & sink.last),
        ]
        buf_shift = Signal(len(buf))
        self.comb += buf_shift.eq(Mux(out_fire, buf >> 128, buf))
        placed_keep = Signal(len(buf))
        placed_fire = Signal(len(buf))
        self.comb += [
            Case(fill, {f: placed_keep.eq((buf & ((1 << (8*f)) - 1)) | (sink.data << (8*f)))
                for f in range(17)}),
            Case(fill_fire, {f: placed_fire.eq(((buf >> 128) & ((1 << (8*f)) - 1)) | (sink.data << (8*f)))
                for f in range(17)}),
        ]
        placed = Signal(len(buf))
        self.comb += placed.eq(Mux(out_fire, placed_fire, placed_keep))
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

class Gen2TXPacker16(LiteXModule):
    """Gen2TXPacker with 16 bytes entries (Gen2TX at 80-bit: a Data Block every 1.65 cycles) for
    slower fabrics (ex: GW5A at 125MHz).

    The entries before the last one of a unit are full (16 bytes): the fill level of the partial
    block only changes at the units ends. Blocks are a byte selection of the partial block and the
    entry by the registered fill level (no per entry placement in a wider buffer), registered output.
    A unit is started in a partial block when its first entry completes the block or is its last
    entry (the unit then fits in the block). A partial block with a unit end is output padded with
    Idle Symbols as soon as the output register is free (never an empty output register at a block
    request between the blocks of a unit) and replaced while not taken when the next unit is packed
    in it. Same interface as Gen2TXPacker."""
    def __init__(self):
        self.sink            = sink   = stream.Endpoint(gen2_tx_unit_layout(16))
        self.source          = source = stream.Endpoint([("data", 128)])
        self.units_available = Signal() # i: A complete unit is available on sink.
        self.unit_end        = Signal() # o: A unit is read (pulse, last entry).
        self.flush           = Signal() # i: Discard the sink entries (link not in U0).
        self.skp_pending     = Signal() # i: SKP Ordered Set pending (Gen2TX, see Gen2TXPacker).

        # # #

        acc     = Signal(128, reset_less=True) # Partial block (bytes acc[0:8*fill], oldest first).
        fill    = Signal(4)   # Bytes in the partial block (0-15).
        in_unit = Signal()    # Reading a unit.
        hold    = Signal()    # Unit ended with a SKP Ordered Set pending: next unit after the block.
        o_pad   = Signal()    # Output register: partial block padded (replaceable while not taken).

        # Entry placed at the fill level: block (16 bytes, the entry bytes after its last one as Idle
        # Symbols: padded block when ending the unit) and remaining bytes.
        total  = Signal(5) # Bytes in the partial block + entry bytes.
        data   = Signal(128)
        block  = Signal(128)
        rest   = Signal(128)
        padded = Signal(128) # Partial block padded with Idle Symbols.
        byte   = lambda v, i: v[8*i:8*(i+1)]
        self.comb += total.eq(fill + sink.bytes)
        for j in range(16):
            self.comb += [
                byte(data, j).eq(Mux(j >= sink.bytes, GEN2_IDLE, byte(sink.data, j))),
                Case(fill, {f: byte(block, j).eq(byte(acc, j) if j < f else byte(data, j - f))
                    for f in range(16)}),
                Case(fill, {f: byte(rest, j).eq(byte(sink.data, 16 + j - f) if j < f else 0)
                    for f in range(16)}),
                byte(padded, j).eq(Mux(j < fill, byte(acc, j), GEN2_IDLE)),
            ]

        # Read/output decisions.
        sent     = Signal() # Padded partial block taken: the partial block is sent.
        free     = Signal() # Output register free (empty or block taken).
        wr       = Signal() # Output register writable (free or padded partial block not taken).
        load     = Signal() # Output register loaded.
        start_ok = Signal()
        take     = Signal()
        pad      = Signal() # Partial block with a unit end output padded.
        self.comb += [
            sent.eq(source.valid & o_pad & source.ready),
            free.eq(~source.valid | (source.ready & ~o_pad)),
            wr.eq(free | (o_pad & ~source.ready)),
            start_ok.eq(self.units_available & ~hold & ((fill == 0) | (total >= 16) | sink.last)),
            take.eq(sink.valid & ~self.flush & ~sent & (in_unit | start_ok)),
            sink.ready.eq(self.flush | (take & (wr | (total < 16)))),
            pad.eq(~in_unit & (fill != 0) & ~take & free),
            load.eq(pad | (take & wr)),
            self.unit_end.eq(sink.valid & sink.ready & sink.last),
        ]
        self.sync += [
            If(load,
                source.data.eq(Mux(pad, padded, block)),
            ),
            If(source.ready,
                source.valid.eq(0),
                o_pad.eq(0),
            ),
            If(self.flush,
                source.valid.eq(0),
                o_pad.eq(0),
                fill.eq(0),
                in_unit.eq(0),
                hold.eq(0),
            ).Elif(sent,
                fill.eq(0),
                hold.eq(0),
            ).Elif(sink.valid & sink.ready,
                in_unit.eq(~sink.last),
                If(total >= 16,
                    source.valid.eq(1),
                    o_pad.eq(0),
                    # Block ending exactly at a unit end.
                    source.last.eq(sink.last & (total == 16)),
                    acc.eq(rest),
                    fill.eq(total - 16),
                    hold.eq(sink.last & self.skp_pending & (total != 16)),
                ).Else(
                    # Unit end in the partial block: output padded when writable.
                    If(wr,
                        source.valid.eq(1),
                        o_pad.eq(1),
                        source.last.eq(1),
                    ),
                    acc.eq(block),
                    fill.eq(total),
                    hold.eq(sink.last & self.skp_pending),
                )
            ).Elif(pad,
                source.valid.eq(1),
                o_pad.eq(1),
                source.last.eq(1),
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

    nbytes: units bytes per buffer entry, read by the packer per cycle (16: framer entries paired).
    """
    def __init__(self, buffer_depth=512, units_delay=8, link_words=1, nbytes=8):
        assert link_words in [1, 2]
        assert nbytes in [8, 16]
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
            stream.AsyncFIFO(gen2_tx_unit_layout(nbytes), buffer_depth, buffered=True))
        self.packer = packer = ClockDomainsRenamer("tx")(Gen2TXPacker16() if nbytes == 16 else
            Gen2TXPacker(nbytes))
        if nbytes == 16:
            # Registered valid/ready between the buffer and the packer (timings: packer read
            # decision to the buffer read side). Not reset: flushed entries drained by the packer
            # (units read count).
            self.read_buffer = read_buffer = ClockDomainsRenamer("tx")(
                stream.Buffer(gen2_tx_unit_layout(16), pipe_valid=True, pipe_ready=True))
            self.comb += [
                buffer.source.connect(read_buffer.sink),
                read_buffer.source.connect(packer.sink),
            ]
        else:
            self.comb += buffer.source.connect(packer.sink)
        if nbytes == 16:
            # Registered valid/ready between the packer and Gen2TX (timings: Gen2TX block requests
            # to the packer read/placement).
            self.packer_buffer = packer_buffer = ClockDomainsRenamer("tx")(ResetInserter()(
                stream.Buffer([("data", 128)], pipe_valid=True, pipe_ready=True)))
            self.comb += [
                packer_buffer.reset.eq(flush_tx),
                packer.source.connect(packer_buffer.sink),
                packer_buffer.source.connect(source),
            ]
        else:
            self.comb += packer.source.connect(source)
        if nbytes == 16:
            # Registered valid/ready around the framer (timings: wider transceiver words with the
            # framer at the PIPE frequency) and framer entries paired.
            framer_sink = stream.Endpoint(gen2_link_layout())
            self.comb += [
                sink.connect(framer_sink, omit={"words"}),
                framer_sink.words.eq(sink.words if link_words == 2 else 1),
            ]
            self.sink_buffer = sink_buffer = ResetInserter()(stream.Buffer(gen2_link_layout(),
                pipe_valid=True, pipe_ready=True))
            self.widener     = widener     = ResetInserter()(Gen2TXUnitsWidener())
            self.unit_buffer = unit_buffer = ResetInserter()(stream.Buffer(gen2_tx_unit_layout(16),
                pipe_valid=True, pipe_ready=True))
            self.comb += [
                sink_buffer.reset.eq(self.flush),
                widener.reset.eq(self.flush),
                unit_buffer.reset.eq(self.flush),
                framer_sink.connect(sink_buffer.sink),
                sink_buffer.source.connect(framer.sink),
                framer.source.connect(widener.sink),
                widener.source.connect(unit_buffer.sink),
                unit_buffer.source.connect(buffer.sink),
            ]
        else:
            self.comb += [
                sink.connect(framer.sink, omit={"words"}),
                framer.sink.words.eq(sink.words if link_words == 2 else 1),
                framer.source.connect(buffer.sink),
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
            packer.flush.eq(flush_tx),
        ]
        if nbytes == 16:
            # Registered (timings): units read counted from the registered unit ends, available
            # flag not set on the 2 cycles after a unit is read (count not updated in the flag yet).
            unit_end_d = Signal()
            self.sync.tx += [
                unit_end_d.eq(packer.unit_end),
                packer.units_available.eq((units_written_d[-1] != units_read.q) &
                    ~packer.unit_end & ~unit_end_d),
            ]
            self.comb += units_read.ce.eq(unit_end_d)
        else:
            self.comb += [
                units_read.ce.eq(packer.unit_end),
                packer.units_available.eq(units_written_d[-1] != units_read.q),
            ]

# Gen 2 Link RX ------------------------------------------------------------------------------------

class Gen2LinkRX(LiteXModule):
    """Data Blocks (rx clock domain, one every 3.3 cycles at 40-bit, 1.65 at 80-bit, not held) ->
    blocks buffer -> nbytes per cycle (8 or 16) -> Gen2RXFraming (reset out of the Data Stream) ->
    words buffer (nbytes/4 words entries, rx -> sys) -> Link Layer words or words pairs (sys clock
    domain, gated by rx_ready: U0). LGOOD_n/LCRD_x Link Commands on source_lc (Link Command words
    buffer, rx -> sys) with lc_enable."""
    def __init__(self, buffer_depth=1024, link_words=1, nbytes=8):
        assert link_words in [1, 2]
        assert nbytes in [8, 16]
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

        # Registered read with 16 bytes per cycle (timings). 16 bytes per cycle: 32 blocks (a Data
        # Block every 1.65 cycles, 2 cycles per short packet ending in the RX framing window: bursts
        # of short packets at the line rate, ex: units stored and forwarded behind a long DPP).
        fifo      = ClockDomainsRenamer("rx")(stream.SyncFIFO([("data", 128)], 32 if nbytes == 16 else 8,
            buffered=(nbytes == 16)))
        framing   = ClockDomainsRenamer("rx")(ResetInserter()(Gen2RXFraming(nbytes)))
        buffer    = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
            stream.AsyncFIFO(gen2_link_layout(nbytes//4), buffer_depth, buffered=True))
        self.submodules += fifo, framing, buffer
        self.framing = framing
        self.buffer  = buffer
        self.comb += [
            fifo.sink.valid.eq(sink.valid),
            fifo.sink.data.eq(sink.data),
            sink.ready.eq(1),
            framing.reset.eq(~self.data_stream),
        ]
        # Link Commands words buffer (rx -> sys).
        self.lc_buffer = lc_buffer = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
            stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 16, buffered=True))
        if nbytes == 16:
            # Registered framing outputs (valid/ready) before the buffers (timings: buffers write side).
            out_buffer = ClockDomainsRenamer("rx")(stream.Buffer(gen2_link_layout(4),
                pipe_valid=True, pipe_ready=True))
            lc_out_buffer = ClockDomainsRenamer("rx")(stream.Buffer([("data", 32), ("ctrl", 4)],
                pipe_valid=True, pipe_ready=True))
            self.submodules += out_buffer, lc_out_buffer
            self.comb += [
                framing.source.connect(out_buffer.sink),
                out_buffer.source.connect(buffer.sink),
                framing.source_lc.connect(lc_out_buffer.sink),
                lc_out_buffer.source.connect(lc_buffer.sink),
            ]
        else:
            self.comb += [
                framing.source.connect(buffer.sink),
                framing.source_lc.connect(lc_buffer.sink),
            ]
        if nbytes == 8:
            converter = ClockDomainsRenamer("rx")(stream.Converter(128, 64))
            self.submodules += converter
            self.comb += [
                fifo.source.connect(converter.sink),
                converter.source.connect(framing.sink),
            ]
        else:
            # Registered valid/ready between the blocks buffer and the RX framing (timings: RX framing
            # decisions to the buffer read side).
            in_buffer = ClockDomainsRenamer("rx")(stream.Buffer([("data", 128)], pipe_valid=True,
                pipe_ready=True))
            self.submodules += in_buffer
            self.comb += [
                fifo.source.connect(in_buffer.sink),
                in_buffer.source.connect(framing.sink),
            ]
        self.sync.rx += If(fifo.sink.valid & ~fifo.sink.ready, self.overflow.eq(1))

        # Link Commands words (rx -> sys), gated by rx_ready.
        self.specials += MultiReg(self.lc_enable, framing.lc_enable, "rx")
        self.comb += [
            self.source_lc.valid.eq(lc_buffer.source.valid & self.rx_ready),
            self.source_lc.data.eq(lc_buffer.source.data),
            lc_buffer.source.ready.eq(~self.rx_ready | self.source_lc.ready),
        ]

        # Words entries -> words pairs/words, gated by rx_ready.
        words = buffer.source
        if (nbytes//4) != link_words:
            self.narrower = narrower = ResetInserter()(WordsNarrower(nbytes//4, link_words,
                words=(link_words != 1)))
            self.comb += [
                narrower.reset.eq(~self.rx_ready),
                buffer.source.connect(narrower.sink, omit={"valid", "ready"}),
                narrower.sink.valid.eq(buffer.source.valid & self.rx_ready),
                buffer.source.ready.eq(~self.rx_ready | narrower.sink.ready),
            ]
            words = narrower.source
        self.comb += [
            words.connect(source, omit={"valid", "ready", "words"}),
            source.valid.eq(words.valid & self.rx_ready),
            words.ready.eq(~self.rx_ready | source.ready),
        ]
        if link_words == 2:
            self.comb += source.words.eq(words.words)
