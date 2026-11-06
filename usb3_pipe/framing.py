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
from usb3_pipe.common import Buffer, AsyncFIFOPipelinedRead
from usb3_pipe.crc    import CRC5

# Gen 1 K Symbols (Link Layer words).
SHP, SDP, EDB, END, SLC, EPF = [s.value for s in [_SHP, _SDP, _EDB, _END, _SLC, _EPF]]

# Packet kinds.
KIND_HP   = 0 # Header Packet (HPSTART).
KIND_DPH  = 1 # Non-deferred Gen 2 DPH (DPHSTART).
KIND_LC   = 2 # Link Command (LCSTART).
KIND_DPP  = 3 # Data Packet Payload (DPPSTART).
KIND_EDB  = 4 # Nullified DPP (DPPABORT).

def byte_offset(n):
    """n bytes in bits (n*8) as a sized expression: in Verilog, a shift amount is self-determined
    and 8*n would be evaluated on max(width(8), width(n)) bits (truncated)."""
    return Cat(C(0, 3), n)

def gen2_link_layout(n=2):
    """Link word pairs (Symbol 0 on data[0:8]), words: 1 or 2 valid words (n words: 1-n, ex: quads
    with n=4)."""
    return [("data", 32*n), ("ctrl", 4*n), ("words", bits_for(n))]

class WordsNarrower(LiteXModule):
    """Link words entries (gen2_link_layout(n_in), 1-n_in words) -> entries of up to n_out words
    (gen2_link_layout(n_out), or link words with n_out=1 and words=False), packets first/last on
    their first/last entries."""
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
            source.first.eq(sink.first & (index == 0)),
            source.last.eq(sink.last & done),
        ]
        if words:
            self.comb += source.words.eq(Mux(done, rem, n_out))
        self.sync += If(source.valid & source.ready,
            If(done, index.eq(0)).Else(index.eq(index + 1))
        )

class WordsToPairs(LiteXModule):
    """Link words -> word pairs (gen2_link_layout, one word per entry; n: words entries of n
    words)."""
    def __init__(self, n=2):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint(gen2_link_layout(n))

        # # #

        self.comb += [
            sink.connect(source, omit={"data", "ctrl"}),
            source.data.eq(sink.data),
            source.ctrl.eq(sink.ctrl),
            source.words.eq(1),
        ]

class PairsToWords(LiteXModule):
    """Words pairs (gen2_link_layout, 1 or 2 words per entry; n: entries of 1-n words) -> link words
    (packets first/last on their first/last words)."""
    def __init__(self, n=2):
        self.sink   = sink   = stream.Endpoint(gen2_link_layout(n))
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        index = Signal(max=n) # Word of the entry.
        done  = Signal()      # Last word of the entry.
        self.comb += [
            done.eq(sink.words <= (index + 1)),
            source.valid.eq(sink.valid),
            source.first.eq(sink.first & (index == 0)),
            source.last.eq(sink.last & done),
            Case(index, {i: [
                source.data.eq(sink.data[32*i:32*(i + 1)]),
                source.ctrl.eq(sink.ctrl[4*i:4*(i + 1)]),
            ] for i in range(n)}),
            sink.ready.eq(source.ready & done),
        ]
        self.sync += If(source.valid & source.ready,
            If(done, index.eq(0)).Else(index.eq(index + 1))
        )

class WordsPacker(LiteXModule):
    """Link words -> full word pairs (gen2_link_layout, two words per entry; a lone word is output
    alone when no word follows it): word pairs at the words rate/2 (ex: Link Layer in a clock
    domain at half the PHY frequency)."""
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
    """Gen 1 Link words: Logical Idle words (00h data symbols) between the packets dropped, the
    packets words kept (Header Packet: SHP framing word and 4 words, Link Command: SLC framing word
    and 1 word, DPP: from the SDP framing word to the EPF of its END/EDB framing). Ex: Link Layer in
    a clock domain at half the PHY frequency, consuming one word per cycle outside of the DPP data.
    The framing Symbols/Logical Idle are decoded in a register stage (timings, one cycle of
    latency)."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Decode stage.
        flags = ["idle", "shp", "slc", "sdp", "epf"]
        self.decode = decode = stream.Buffer([("data", 32), ("ctrl", 4)] + [(f, 1) for f in flags])
        def has(symbol):
            return Reduce("OR", [sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == symbol)
                for i in range(4)])
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
        # Buffered packets: packets written/read counters (timings: no up/down counter on the
        # buffer output path).
        packets_in  = Signal(bits_for(depth))
        packets_out = Signal(bits_for(depth))
        packet_in   = Signal()
        packet_out  = Signal()
        pending     = Signal()
        self.comb += [
            sink.connect(fifo.sink),
            packet_in.eq(sink.valid & sink.ready & sink.last),
            packet_out.eq(fifo.source.valid & fifo.source.ready & fifo.source.last),
            pending.eq(packets_in != packets_out),
            fifo.source.connect(source, omit={"valid", "ready"}),
            source.valid.eq(fifo.source.valid & pending & ~self.drop),
            fifo.source.ready.eq((source.ready & pending) | self.drop),
        ]
        self.sync += [
            If(packet_in,  packets_in.eq(packets_in + 1)),
            If(packet_out, packets_out.eq(packets_out + 1)),
        ]

class WordsWindow(LiteXModule):
    """Link word pairs (gen2_link_layout, word 0 first) -> window of the next two words (w0/c0,
    w1/c1, n: 0-2 available words), consume: words consumed this cycle (0-n). nwords: entries and
    window of nwords words (w[i]/c[i]/f[i], n: 0-nwords).

    decode: optional function (data, ctrl) -> nflags-bit flags of a word, computed on the incoming
    words and stored with them (f0/f1: registered decodes for the window consumer, timings).

    Timings: the sink ready and the entry words placement are from the registered level only (the
    consumed words of the cycle select between candidates: consume is late, from the window
    consumer decisions): 3*nwords slots, an entry is accepted when 2*nwords slots or less are used
    (the words consumed this cycle not counted: full rate kept with the level at any phase)."""
    def __init__(self, decode=None, nflags=0, nwords=2):
        self.sink    = sink = stream.Endpoint(gen2_link_layout(nwords))
        self.n       = Signal(bits_for(nwords))                        # o
        self.w       = [Signal(32) for _ in range(nwords)]             # o
        self.c       = [Signal(4)  for _ in range(nwords)]             # o
        self.f       = [Signal(max(nflags, 1)) for _ in range(nwords)] # o
        self.consume = Signal(bits_for(nwords))                        # i
        self.w0, self.c0, self.f0 = self.w[0], self.c[0], self.f[0]
        self.w1, self.c1, self.f1 = self.w[1], self.c[1], self.f[1]

        # # #

        size  = 3*nwords
        cw    = 4 + nflags # ctrl + flags.
        data  = [Signal(32, reset_less=True) for _ in range(size)]
        ctrl  = [Signal(cw, reset_less=True) for _ in range(size)]
        level = Signal(bits_for(size))
        self.comb += [
            [self.w[i].eq(data[i]) for i in range(nwords)],
            [self.c[i].eq(ctrl[i][:4]) for i in range(nwords)],
            sink.ready.eq(level <= 2*nwords),
        ]
        if nflags:
            self.comb += [self.f[i].eq(ctrl[i][4:]) for i in range(nwords)]
        in_data = []
        in_ctrl = []
        for w in range(nwords):
            d, c = sink.data[32*w:32*(w+1)], sink.ctrl[4*w:4*(w+1)]
            in_data.append(d)
            in_ctrl.append(Cat(c, decode(d, c)) if nflags else c)
        accept = Signal()
        self.comb += accept.eq(sink.valid & sink.ready)
        # Entry words at the window positions p >= level (word p - level, by the registered level),
        # shared by the slots/consumed counts with i + c == p.
        npos   = size + nwords
        in_d   = [Signal(32, name=f"in_d{p}") for p in range(npos)]
        in_c   = [Signal(cw, name=f"in_c{p}") for p in range(npos)]
        beyond = [Signal(name=f"beyond{p}") for p in range(npos)] # level <= p.
        for p in range(npos):
            cases = {l: [in_d[p].eq(in_data[p - l]), in_c[p].eq(in_ctrl[p - l])]
                for l in range(size + 1) if 0 <= (p - l) < nwords}
            cases["default"] = [in_d[p].eq(0), in_c[p].eq(0)]
            self.comb += [
                Case(level, cases),
                beyond[p].eq(level <= p),
            ]
        # Slot i: slot i + consume (shift) or the entry word at i + consume, selected by the
        # consumed count last.
        for i in range(size):
            cand_d = []
            cand_c = []
            for c in range(nwords + 1):
                p = i + c
                d = Signal(32, name=f"cand_d{i}_{c}")
                k = Signal(cw, name=f"cand_c{i}_{c}")
                if p < size:
                    self.comb += [
                        d.eq(Mux(beyond[p], in_d[p], data[p])),
                        k.eq(Mux(beyond[p], in_c[p], ctrl[p])),
                    ]
                else:
                    self.comb += d.eq(in_d[p]), k.eq(in_c[p])
                cand_d.append(d)
                cand_c.append(k)
            self.sync += Case(self.consume, {c: [data[i].eq(cand_d[c]), ctrl[i].eq(cand_c[c])]
                for c in range(nwords + 1)})
        # Level and available words (registered) candidates per consumed count (timings: the late
        # consume selects last).
        base = Signal(bits_for(size) + 1)
        self.comb += base.eq(level + Mux(accept, sink.words, 0))
        self.sync += Case(self.consume, {c: [
            level.eq(base - c),
            self.n.eq(Mux((base - c) >= nwords, nwords, (base - c)[:bits_for(nwords)])),
        ] for c in range(nwords + 1)})

# Gen 2 RX Framing ---------------------------------------------------------------------------------

class Gen2RXFraming(LiteXModule):
    """Data Blocks bytes (nbytes per cycle: 8 or 16, descrambled, bytes in transmission order from
    data[0:8]) to Link Layer words entries (gen2_link_layout(nbytes//4): word pairs at 8 bytes per
    cycle): Idle Symbols between packets dropped, packets re-aligned on words, framing
    Symbols as Gen 1 K Symbols (ctrl), DPHSTART as HPSTART and Length Field Replicas removed, DPP
    words with the CRC-32/DPPEND (padded to the word).

    A packet is identified by its first framing Symbol (in Idle) and its length (DPP: Length Field
    Replica of the preceding DPH), a byte pointer re-aligns it in a 2*nbytes window: nbytes per
    cycle in a packet, 1 cycle per packet start. Unknown packet starts are skipped (errors). 16
    bytes per cycle for wider transceiver words (ex: 80-bit at 125MHz, a Data Block every 1.65
    cycles).

    With lc_enable, LGOOD_n/LCRD_x/LBAD Link Commands (16+ bytes per cycle: a copy with a valid
    CRC-5, 8 bytes: both copies identical) are output on source_lc
    (Link Command word, Symbol 0 on data[0:8]) instead of source: they only concern the transmitter
    (Header Packets acknowledgements, credits and retransmission requests), the Link Layer gets them
    without waiting behind the received packets (RX buffering, 7.2.4.1: Header Packets held in the
    Tx Header Buffers until LGOOD_n; an LBAD held behind received DPPs would delay the LRTY and the
    retransmissions while the partner rejects the Header Packets sent in the meantime). LRTY stays
    in order (it delimits the retransmitted Header Packets).
    """
    def __init__(self, nbytes=8):
        assert nbytes in [8, 16, 32]
        pw = log2_int(nbytes)
        self.sink      = sink      = stream.Endpoint([("data", 8*nbytes)])
        self.source    = source    = stream.Endpoint(gen2_link_layout(nbytes//4))
        self.source_lc = source_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.errors    = Signal(16) # o: Unknown packet starts (skipped bytes).
        self.corrected = Signal(16) # o: Packet starts with a framing Symbol 0 error (3/4 valid).
        self.lc_enable = Signal()   # i: LGOOD_n/LCRD_x on source_lc.
        self.dbg_error        = Signal()         # o: Unknown packet start skipped (pulse,
                                                 #    bring-up).
        self.dbg_error_window = Signal(8*nbytes) # o: Its bytes (the skipped byte first), with
                                                 #    dbg_error.

        # # #

        # Input stage: words with their bytes start codes (packet kind + 1, 0: Idle, CODE_UNKNOWN) ---
        CODE_UNKNOWN = 7
        in_valid = Signal()
        in_data  = Signal(8*nbytes, reset_less=True)
        in_codes = Signal(3*nbytes, reset_less=True)
        in_ready = Signal()
        codes    = Signal(3*nbytes)
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
        r_skip    = [Signal(name=f"r_skip{i}", reset_less=True) for i in range(nbytes)]
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
            # R consumed from position i in Idle (no packet start at/after i): registered (timings:
            # window shift). An unknown start is checked in RESYNC (R kept).
            [r_skip[i].eq(~nxt_found[i]) for i in range(nbytes)],
        )

        # Packet start search (Idle) ---------------------------------------------------------------
        p     = Signal(pw) # Byte pointer in R.
        p_out = Signal(pw, attr={("syn_preserve", 1)}) # Copy for the output bytes (fanout).
        q     = Signal(pw) # First non-Idle byte at/after p.
        found = Signal()
        first = Signal(3)  # Start code of the byte at q.
        skip  = Signal()   # R consumed (Idle).
        self.comb += [
            skip.eq(Array(r_skip)[p]),
            found.eq(Array(r_found)[p]),
            q.eq(Array(r_q)[p]),
            first.eq(Array(r_code)[p]),
        ]

        # Packet -----------------------------------------------------------------------------------
        kind        = Signal(3)
        rem         = Signal(11) # Packet bytes remaining (from the current chunk).
        start       = Signal()   # First chunk of the packet.
        last        = Signal()
        dpp_len12   = Signal(11, reset=12) # Last DPH Length Field Replica + 12 (DPP length).
        dpp_len_sel = Signal(11)           # DPP length for the DPP start (set_rem, see below).
        out         = Signal(8*nbytes)
        out_edb     = Signal(nbytes)
        self.comb += [
            out.eq((win >> byte_offset(p_out))[0:8*nbytes]),
            out_edb.eq((Cat(r_edb, in_edb) >> p_out)[0:nbytes]),
            last.eq(rem <= nbytes),
        ]

        # Output words (registered with 16+ bytes per cycle, see the Link Commands): framing Symbols
        # as K Symbols, bytes after the packet end padded (00h).
        if nbytes >= 16:
            # Chunk registered first (PipeValid: the window shift), the framing Symbols/padding then
            # applied from the registers and the Link Command decoded (PipeReady registers after):
            # timings (the shift and the Symbols selection in two stages).
            raw = stream.Endpoint([
                ("data", 8*nbytes), ("edb", nbytes), ("words", bits_for(nbytes//4)),
                ("rem", 11), ("start", 1), ("kind", 3), ("fin", 1)])
            self.out_pipe_valid = out_pipe_valid = stream.PipeValid(raw.description)
            cooked = stream.Endpoint(gen2_link_layout(nbytes//4) + [("lc", 1)])
            self.comb += [
                raw.data.eq(out),
                raw.edb.eq(out_edb),
                raw.rem.eq(rem),
                raw.start.eq(start),
                raw.kind.eq(kind),
                raw.fin.eq(last),
                raw.connect(out_pipe_valid.sink),
            ]
            output = raw
            r = out_pipe_valid.source
            s_data, s_edb, s_rem, s_start, s_kind, dst = r.data, r.edb, r.rem, r.start, r.kind, cooked
        else:
            output = source
            s_data, s_edb, s_rem, s_start, s_kind, dst = out, out_edb, rem, start, kind, source
        start_k = Signal(8)
        self.comb += Case(s_kind, {
            KIND_HP  : start_k.eq(SHP),
            KIND_DPH : start_k.eq(SHP),
            KIND_LC  : start_k.eq(SLC),
            KIND_DPP : start_k.eq(SDP),
            KIND_EDB : start_k.eq(EDB),
        })
        # Byte j of the chunk is packet byte i: i >= length <-> j >= rem (comparisons with
        # constants).
        for j in range(nbytes):
            byte   = s_data[8*j:8*(j+1)]
            data   = dst.data[8*j:8*(j+1)]
            ctrl   = dst.ctrl[j]
            is_end = Signal(name=f"is_end{j}") # DPPEND/DPPABORT.
            self.comb += is_end.eq((s_kind == KIND_DPP) & ((j + 4) >= s_rem))
            self.comb += If(j >= s_rem,
                data.eq(0x00),
            ).Elif(s_start & (j < 3),
                ctrl.eq(1), data.eq(start_k),
            ).Elif(s_start & (j == 3),
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end & ((j + 1) == s_rem),
                ctrl.eq(1), data.eq(EPF),
            ).Elif(is_end,
                ctrl.eq(1), data.eq(Mux(s_edb[j], EDB, END)),
            ).Else(
                data.eq(byte),
            )

        # LGOOD_n/LCRD_x/LBAD Link Commands (7.2.2): source_lc. Fast Link Commands: identical
        # copies, or a copy of a fast kind with a valid CRC-5 (the Link Layer votes between the
        # copies): a Link Command with a bit error in one copy is not sent behind the RX words,
        # where the following fast Link Commands would overtake it (LGOOD_n/LCRD_x out of order:
        # Recovery, seen on hardware at 10Gbps). With 16+ bytes per cycle, decoded on the registered
        # chunk (timings).
        def fast_kind(c):
            return (((c[0:11] & 0x7f0) == 0x000) |  # LGOOD_n.
                    ((c[0:11] & 0x7f8) == 0x080) |  # LCRD_x/LCRD1_x/LCRD2_x.
                    ( c[0:11]          == 0x180))   # LBAD.
        lc_w    = r.data[32:64] if nbytes >= 16 else out[32:64]
        crc5    = [CRC5() for _ in range(2)]
        fast_lc = Signal()
        self.submodules += crc5
        self.comb += [
            crc5[0].data.eq(lc_w[0:11]),
            crc5[1].data.eq(lc_w[16:27]),
            fast_lc.eq((fast_kind(lc_w[0:16])  & ((lc_w[0:16] == lc_w[16:32]) |
                                                  (crc5[0].value == lc_w[11:16]))) |
                       (fast_kind(lc_w[16:32]) & (crc5[1].value == lc_w[27:32]))),
        ]
        lc_fast   = Signal()
        out_ready = Signal()
        if nbytes >= 16:
            self.out_buffer = out_buffer = stream.PipeReady(cooked.description)
            fast = Signal()
            self.comb += [
                cooked.valid.eq(r.valid),
                r.ready.eq(cooked.ready),
                cooked.words.eq(r.words),
                cooked.lc.eq((r.kind == KIND_LC) & fast_lc),
                cooked.connect(out_buffer.sink),
                out_ready.eq(raw.ready),
                fast.eq(self.lc_enable & out_buffer.source.lc),
                out_buffer.source.connect(source, omit={"valid", "ready", "lc"}),
                source.valid.eq(out_buffer.source.valid & ~fast),
                source_lc.valid.eq(out_buffer.source.valid & fast),
                source_lc.data.eq(out_buffer.source.data[32:64]),
                out_buffer.source.ready.eq(Mux(fast, source_lc.ready, source.ready)),
            ]
        else:
            self.comb += [
                lc_fast.eq(self.lc_enable & (kind == KIND_LC) & fast_lc),
                source_lc.data.eq(lc_w),
                # Both outputs ready (not depending on the Link Command decode: timings).
                out_ready.eq(source_lc.ready & source.ready),
            ]

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        def set_rem(code):
            return Case(code, {
                KIND_HP  + 1 : NextValue(rem, 20),
                KIND_DPH + 1 : NextValue(rem, 24),
                KIND_LC  + 1 : NextValue(rem, 8),
                KIND_DPP + 1 : NextValue(rem, dpp_len_sel),
                KIND_EDB + 1 : NextValue(rem, 4),
            })
        # Unknown packet start at p (Symbol 0 of a framing Ordered Set with a bit error?): Symbols
        # 1/2 identical start Symbols and Symbol 3 EPF (3 of the 4 framing Symbols valid) -> packet
        # start at p, else the byte is skipped (error).
        resync_code = Signal(3)
        self.comb += Case(out[8:16], {
            GEN2_SHP  : resync_code.eq(KIND_HP  + 1),
            GEN2_DPHP : resync_code.eq(KIND_DPH + 1),
            GEN2_SLC  : resync_code.eq(KIND_LC  + 1),
            GEN2_SDP  : resync_code.eq(KIND_DPP + 1),
            GEN2_EDB  : resync_code.eq(KIND_EDB + 1),
            "default" : resync_code.eq(CODE_UNKNOWN),
        })
        resync_ok = Signal()
        self.comb += resync_ok.eq((resync_code != CODE_UNKNOWN) & (out[16:24] == out[8:16]) &
            (out[24:32] == GEN2_EPF))
        # Checks registered (timings: the window shift to the decision), the window is held in
        # RESYNC (no shift).
        resync_ok_r   = Signal()
        resync_code_r = Signal(3)
        fsm.act("RESYNC",
            If(avail,
                NextValue(resync_ok_r,   resync_ok),
                NextValue(resync_code_r, resync_code),
                NextState("RESYNC-CHECK")
            )
        )
        fsm.act("RESYNC-CHECK",
            If(avail,
                If(resync_ok_r,
                    NextValue(self.corrected, self.corrected + 1),
                    NextValue(kind, resync_code_r - 1),
                    set_rem(resync_code_r),
                    NextState("PACKET")
                ).Else(
                    NextValue(self.errors, self.errors + 1),
                    self.dbg_error.eq(1),
                    self.dbg_error_window.eq(out),
                    NextState("IDLE"),
                    If(p == (nbytes - 1),
                        shift.eq(1),
                        NextValue(p, 0), NextValue(p_out, 0)
                    ).Else(
                        NextValue(p, p + 1), NextValue(p_out, p + 1)
                    )
                )
            )
        )
        # DPP length: the DPH Length Field Replicas (packet bytes 20-21 and 22-23) when identical,
        # else the DPH Data Length field (packet bytes 10-11, checked later by the Header Packet
        # CRC-16): a bit error in a replica does not lose the DPP framing.
        hlen_rem   = 24 - nbytes*(10//nbytes)  # rem of the chunk with the DPH bytes 10-11.
        hlen       = Signal(16, reset_less=True)
        dph_len12  = Signal(11) # DPP length + 12: candidates +12, selected (timings).
        if nbytes >= 16:
            # From the registered chunk (PipeValid) of the DPH last chunk (timings: not from the
            # window shift): the DPP start follows the DPH, found while the chunk is registered or
            # later (dpp_len12).
            hlen_now = r.data[8*(10 % nbytes):8*(10 % nbytes) + 16]
            replica0 = r.data[8*(20 % nbytes):8*(20 % nbytes) + 16]
            replica1 = r.data[8*(20 % nbytes) + 16:8*(20 % nbytes) + 32]
            r_dph_last = Signal()
            self.comb += r_dph_last.eq(r.valid & (r.kind == KIND_DPH) & r.fin)
            self.sync += [
                If(r.valid & r.ready & (r.kind == KIND_DPH) & (r.rem == hlen_rem),
                    hlen.eq(hlen_now)
                ),
                If(r_dph_last & r.ready, dpp_len12.eq(dph_len12)),
            ]
            self.comb += dpp_len_sel.eq(Mux(r_dph_last, dph_len12, dpp_len12))
        else:
            self.comb += dpp_len_sel.eq(dpp_len12)
            hlen_now = out[8*(10 % nbytes):8*(10 % nbytes) + 16]
            replica0 = out[8*(20 % nbytes):8*(20 % nbytes) + 16]
            replica1 = out[8*(20 % nbytes) + 16:8*(20 % nbytes) + 32]
            self.sync += If(fsm.ongoing("PACKET") & avail & out_ready & (kind == KIND_DPH) &
                (rem == hlen_rem), hlen.eq(hlen_now))
        hlen_sel = hlen_now if hlen_rem <= nbytes else hlen
        self.comb += dph_len12.eq(Mux(replica0 == replica1, replica0[0:11] + 12,
            hlen_sel[0:11] + 12))
        p_next = Signal(pw + 1)
        fsm.act("IDLE",
            If(avail,
                If(found,
                    NextValue(p, q), NextValue(p_out, q),
                    NextValue(start, 1),
                    NextValue(kind, first - 1),
                    set_rem(first),
                    If(first != CODE_UNKNOWN,
                        NextState("PACKET")
                    ).Else(
                        # Unknown start: framing Symbol 0 error? (RESYNC).
                        NextState("RESYNC")
                    )
                ).Else(
                    NextValue(p, 0), NextValue(p_out, 0)
                ),
                shift.eq(skip),
            )
        )
        self.comb += p_next.eq(p + rem[0:pw + 1])
        fsm.act("PACKET",
            If(avail,
                output.valid.eq(~lc_fast),
                source_lc.valid.eq(lc_fast) if nbytes == 8 else [],
                # Valid words: rem bytes (up to nbytes) rounded up to words.
                output.words.eq(Mux(last, (rem[0:pw + 1] + 3) >> 2, nbytes//4)),
                If((kind == KIND_DPH) & last,
                    output.words.eq((20 % nbytes)//4), # Length Field Replicas removed.
                ),
                If(out_ready,
                    If((kind == KIND_DPH) & last,
                        NextValue(dpp_len12, dph_len12) if nbytes < 16 else [],
                    ),
                    NextValue(rem, rem - nbytes),
                    NextValue(start, 0),
                    If(~last,
                        shift.eq(1)
                    ).Else(
                        NextState("IDLE"),
                        If(p_next >= nbytes,
                            shift.eq(1),
                            NextValue(p, p_next - nbytes), NextValue(p_out, p_next - nbytes)
                        ).Else(
                            NextValue(p, p_next), NextValue(p_out, p_next)
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
    """Transmission units entries: 8 bytes (or in_bytes: 16) -> nbytes (16 or 32: entries of a unit
    grouped, the entries before the last one of a unit are full). The bytes after the last one of
    an entry are Idle Symbols (Gen2TXBlocksPacker padding)."""
    def __init__(self, nbytes=32, in_bytes=8):
        assert nbytes in [16, 32]
        assert in_bytes in [8, 16]
        n  = nbytes//in_bytes
        ib = in_bytes
        self.sink   = sink   = stream.Endpoint(gen2_tx_unit_layout(ib))
        self.source = source = stream.Endpoint(gen2_tx_unit_layout(nbytes))

        # # #

        count  = Signal(max=max(n, 2))                     # Held entries.
        held_d = [Signal(8*ib, reset_less=True) for _ in range(n - 1)]
        data   = Signal(8*ib)
        idle   = int.from_bytes(bytes([GEN2_IDLE])*ib, "little")
        self.comb += [data[8*j:8*(j+1)].eq(Mux(j >= sink.bytes, GEN2_IDLE, sink.data[8*j:8*(j+1)]))
            for j in range(ib)]
        out = Case(count, {c: [
            source.data.eq(Cat(*held_d[:c], data, *[Constant(idle, 8*ib)]*(n - 1 - c))),
            source.bytes.eq(ib*c + sink.bytes)] for c in range(n)})
        self.comb += [
            If(sink.last | (count == (n - 1)),
                # Last entry of a unit or entries completing the output entry.
                source.valid.eq(sink.valid),
                out,
                source.last.eq(sink.last),
                sink.ready.eq(source.ready),
            ).Else(
                sink.ready.eq(1),
            )
        ]
        self.sync += If(sink.valid & sink.ready,
            If(sink.last | (count == (n - 1)),
                count.eq(0),
            ).Else(
                count.eq(count + 1),
            ),
            Case(count, {c: held_d[c].eq(sink.data) for c in range(n - 1)}),
        )

class Gen2TXFramer(LiteXModule):
    """Link Layer word pairs (gen2_link_layout: Symbol 0 on data[0:8], K Symbols with ctrl, framing
    Ordered Sets at word boundaries, 1 or 2 words per entry) to transmission units bytes (8 per
    entry): Gen 2 framing bytes, DPHSTART and Length Field Replicas for the DPHs (non-deferred: a
    device does not defer), bytes after the DPPEND/DPPABORT removed. Units: Header Packet, Link
    Command, DPH with its DPP (no spacing allowed between them, 7.2.1.2.3).

    One word per cycle, two in the DPPs (data words): 8 bytes per cycle. nwords=4: word quads
    entries, four words in the DPPs (16 bytes per cycle), units entries of 16 bytes."""
    def __init__(self, nwords=2):
        assert nwords in [2, 4]
        eb = 4*nwords # Units entries bytes.
        self.sink   = sink   = stream.Endpoint(gen2_link_layout(nwords))
        self.source = source = stream.Endpoint(gen2_tx_unit_layout(eb))

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
        self.window = window = WordsWindow(decode, nflags=9, nwords=nwords)
        self.comb += sink.connect(window.sink)
        valid0 = Signal()
        data0  = window.w0
        ctrl0  = window.c0
        ready0 = Signal() # Word 0 consumed (when valid0).
        two    = Signal() # Words 0 and 1 consumed (DPP data, 8-byte chunk).
        four   = Signal() # Words 0 to 3 consumed (DPP data, 16-byte chunk, nwords=4).
        self.comb += [
            valid0.eq(window.n != 0),
            window.consume.eq(Mux(four, 4, Mux(two, 2, Mux(valid0 & ready0, 1, 0)))),
        ]

        # Words -> bytes chunks (4 bytes, last chunk 1-4, 8 or 16 bytes in the DPPs) ---------------
        chunk_layout = [("data", 8*eb), ("bytes", bits_for(eb))]
        chunk = stream.Endpoint(chunk_layout)

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

        is_dph    = Signal()
        dph_len   = Signal(16)
        n         = Signal(3) # Word index in the Header Packet.
        word0     = Signal(32)
        word0_c   = Signal(4)
        hpstart   = window.f0[0]
        lcstart   = window.f0[1]
        # DPPEND/DPPABORT end: EPF K Symbol after the DPPSTART word.
        epf       = window.f0[2:6]
        edb0      = window.f0[6] # Symbol 0 EDB.
        type_dp   = window.f0[7] # DW0 Type: DP.
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
        dpp_two  = Signal() # Two data words available (no K Symbols): 8-byte chunk.
        dpp_four = Signal() # Four data words available (nwords=4): 16-byte chunk.
        self.comb += dpp_two.eq((window.n >= 2) & window.f0[8] & window.f1[8])
        if nwords == 4:
            self.comb += dpp_four.eq((window.n == 4) & window.f[0][8] & window.f[1][8] &
                window.f[2][8] & window.f[3][8])
        fsm.act("DPP",
            chunk.valid.eq(valid0),
            # Chunk words from the window (the first mapped: a data word is unchanged, the words
            # after the chunk bytes unused): the chunk size selection not on the data (fanout).
            chunk.data.eq(Cat(mapped(data0, ctrl0), *window.w[1:])),
            If(dpp_four,
                chunk.bytes.eq(16),
                four.eq(chunk.ready),
            ).Elif(dpp_two,
                chunk.bytes.eq(8),
                two.eq(chunk.ready),
            ).Else(
                chunk.bytes.eq(epf_bytes),
                chunk.last.eq(epf != 0),
                ready0.eq(chunk.ready),
            ),
            If(valid0 & chunk.ready & ~dpp_two & ~dpp_four & (epf != 0), NextState("IDLE"))
        )
        if nwords == 4:
            self.add_entries16(chunk, chunk_layout, source)
            return

        # Chunks -> 8-byte entries (4-byte chunks paired, 8-byte chunks never last) ----------------
        # Registered valid/ready (timings: window/FSM decisions to the pairing).
        self.chunk_buffer = chunk_buffer = Buffer([("data", 64), ("bytes", 4)],
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

    def add_entries16(self, chunk, chunk_layout, source):
        # Chunks -> 16-byte entries (4/8/16-byte chunks packed in full entries, the last chunk of a
        # unit 1-4 bytes), held words: 0-3 (multiples of 4 bytes).
        # Registered valid/ready (timings: window/FSM decisions to the packing).
        self.chunk_buffer = chunk_buffer = Buffer(chunk_layout, pipe_valid=True, pipe_ready=True)
        self.comb += chunk.connect(chunk_buffer.sink)
        chunk = chunk_buffer.source
        held  = Signal(96, reset_less=True)
        h     = Signal(2) # Held words.
        cw    = Signal(3) # Chunk words.
        t     = Signal(4) # Held + chunk words.
        out   = Signal()
        self.comb += [
            cw.eq(Mux(chunk.bytes == 16, 4, Mux(chunk.bytes == 8, 2, 1))),
            t.eq(h + cw),
            out.eq(chunk.last | (t >= 4)),
            source.valid.eq(chunk.valid & out),
            Case(h, {c: source.data.eq(Cat(held[:32*c], chunk.data)[:128]) for c in range(4)}),
            source.bytes.eq(Mux(chunk.last, Cat(C(0, 2), h) + chunk.bytes, 16)),
            source.last.eq(chunk.last),
            chunk.ready.eq(Mux(out, source.ready, 1)),
        ]
        self.sync += If(chunk.valid & chunk.ready,
            If(chunk.last,
                h.eq(0),
            ).Elif(out,
                # Chunk words after the entry held.
                h.eq(t - 4),
                Case(h, {c: held.eq(chunk.data[32*(4 - c):] if c else 0) for c in range(4)}),
            ).Else(
                h.eq(t),
                Case(h, {c: held.eq(Cat(held[:32*c], chunk.data)[:96]) for c in range(4)}),
            )
        )

class Gen2TXPacker(LiteXModule):
    """Transmission units bytes (nbytes per entry: 8 or 16) to Data Blocks payloads (16 bytes, bytes
    in transmission order from data[0:8]) for Gen2TX: units packed back to back, Idle Symbols (5Ah)
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
                                        #    a unit ends there (Idle Symbols, last) instead of
                                        #    packing the next unit (SKP Ordered Sets only between
                                        #    packets).

        # # #

        buf     = Signal(8*(16 + nbytes)) # Bytes (oldest from buf[0:8]).
        fill    = Signal(bits_for(16 + nbytes)) # Bytes in buf (0-(16 + nbytes)).
        in_unit = Signal()     # Reading a unit.
        ending  = Signal()     # Block with a unit end and Idle Symbols.
        # Unit ended with a SKP Ordered Set pending: next unit after the block.
        hold    = Signal()

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
        fill_eff  = Signal(len(fill))
        start_ok  = Signal()
        rd        = Signal()
        fill_fire = Signal(len(fill)) # Fill level after an output block.
        self.comb += fill_fire.eq(Mux(fill >= 16, fill - 16, 0))
        def read_ok(f):
            start = (f == 0) | ((f + sink.bytes) >= 16)
            return (sink.valid & ~self.flush & (in_unit | (self.units_available & ~hold & start)) &
                (f <= 16))
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
            Case(fill_fire, {f: placed_fire.eq(((buf >> 128) & ((1 << (8*f)) - 1)) |
                (sink.data << (8*f))) for f in range(17)}),
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

def gen2_tx_blocks_layout(nblocks=2):
    """Data Blocks payloads entries (nblocks blocks, blocks: 1-nblocks valid blocks, last: the last
    valid block ends a units chain: Idle Symbols padding or unit end, SKP Ordered Sets allowed after
    it)."""
    return [("data", 128*nblocks), ("blocks", bits_for(nblocks))]

class Gen2TXBlocksPacker(LiteXModule):
    """Transmission units bytes (32 bytes entries: entries before the last one of a unit full, bytes
    after the last one Idle Symbols, Gen2TXUnitsWidener) to Data Blocks payloads entries (up to 2
    blocks per cycle) at the Link Layer frequency, for a blocks buffer read by Gen2TX
    (Gen2TXBlocksReader).

    Units are packed in chains: a unit is started in the partial block of the previous one (unit end
    in the block) when available, else the partial block is output padded with Idle Symbols and ends
    the chain (last). Chains also end at a unit end with a SKP Ordered Set pending (Gen2TX) or after
    max_chain blocks (the reader waits for complete chains: bounded buffering). The fill level of
    the partial block only changes at the units ends: blocks are a byte selection of the partial
    block and the entry by the registered fill level. Registered output."""
    def __init__(self, max_chain=32):
        self.sink            = sink   = stream.Endpoint(gen2_tx_unit_layout(32))
        self.source          = source = stream.Endpoint(gen2_tx_blocks_layout(2))
        self.units_available = Signal() # i: A complete unit is available on sink.
        self.unit_end        = Signal() # o: A unit is read (pulse, last entry).
        self.flush           = Signal() # i: Discard the sink entries (link not in U0).
        self.skp_pending     = Signal() # i: SKP Ordered Set pending: chain ended at a unit end.

        # # #

        acc     = Signal(128, reset_less=True) # Partial block (bytes acc[0:8*fill]).
        fill    = Signal(4)  # Bytes in the partial block (0-15).
        fill_b  = Signal(4, attr={("syn_preserve", 1)}) # Copy for the bytes selection (fanout).
        in_unit = Signal()   # Reading a unit.
        chain   = Signal(max=max_chain + 3) # Blocks of the current chain.

        # Entry placed at the fill level: 2 blocks and remaining bytes, padded partial block.
        byte   = lambda v, i: v[8*i:8*(i+1)]
        total  = Signal(6) # Bytes in the partial block + entry bytes (1-47).
        blocks = Signal(256)
        rest   = Signal(128)
        padded = Signal(128)
        self.comb += total.eq(fill + sink.bytes)
        for j in range(32):
            self.comb += Case(fill_b, {f: byte(blocks, j).eq(byte(acc, j) if j < f else
                byte(sink.data, j - f)) for f in range(16)})
        for j in range(16):
            self.comb += [
                Case(fill_b, {f: byte(rest, j).eq(byte(sink.data, 32 + j - f) if j < f else 0)
                    for f in range(16)}),
                byte(padded, j).eq(Mux(j < fill_b, byte(acc, j), GEN2_IDLE)),
            ]

        # Decisions.
        adv      = Signal() # Output register free (or taken).
        pending  = Signal() # Partial block with a unit end.
        can_pack = Signal() # Next unit packed in the partial block.
        take     = Signal()
        pad      = Signal()
        nout     = Signal(2) # Blocks of the entry.
        self.comb += [
            adv.eq(~source.valid | source.ready),
            pending.eq(~in_unit & (fill != 0)),
            can_pack.eq(self.units_available & ~self.skp_pending & (chain < max_chain)),
            take.eq(sink.valid & ~self.flush & (in_unit | (pending & can_pack) |
                ((fill == 0) & self.units_available))),
            pad.eq(pending & ~can_pack & ~self.flush & adv),
            If(total >= 32,
                nout.eq(2)
            ).Elif(total >= 16,
                nout.eq(1)
            ),
            sink.ready.eq(self.flush | (take & (adv | (nout == 0)))),
            self.unit_end.eq(sink.valid & sink.ready & sink.last),
        ]
        last = Signal() # Unit ending exactly at the end of the output blocks.
        self.comb += last.eq(sink.last & ((total == 16) | (total == 32)))
        self.sync += [
            If(source.ready, source.valid.eq(0)),
            If(self.flush,
                source.valid.eq(0),
                fill.eq(0), fill_b.eq(0),
                in_unit.eq(0),
                chain.eq(0),
            ).Elif(sink.valid & sink.ready,
                in_unit.eq(~sink.last),
                If(nout != 0,
                    source.valid.eq(1),
                    source.data.eq(blocks),
                    source.blocks.eq(nout),
                    source.last.eq(last),
                    chain.eq(Mux(last, 0, chain + nout)),
                ),
                If(nout == 2,
                    acc.eq(rest),
                    fill.eq(total - 32), fill_b.eq(total - 32),
                ).Elif(nout == 1,
                    acc.eq(blocks[128:]),
                    fill.eq(total - 16), fill_b.eq(total - 16),
                ).Else(
                    acc.eq(blocks[:128]),
                    fill.eq(total), fill_b.eq(total),
                )
            ).Elif(pad,
                source.valid.eq(1),
                source.data.eq(padded),
                source.blocks.eq(1),
                source.last.eq(1),
                chain.eq(0),
                fill.eq(0), fill_b.eq(0),
            ),
        ]

class Gen2TXBlocksReader(LiteXModule):
    """Data Blocks payloads entries (Gen2TXBlocksPacker, through a blocks buffer) to blocks for
    Gen2TX (last: SKP Ordered Sets allowed after the block). A chain is started once complete in
    the buffer (chains_available: never a Logical Idle Data Block inside a unit)."""
    def __init__(self):
        self.sink             = sink   = stream.Endpoint(gen2_tx_blocks_layout(2))
        self.source           = source = stream.Endpoint([("data", 128)])
        self.chains_available = Signal() # i: A complete chain is available on sink.
        self.chain_end        = Signal() # o: A chain is read (pulse).
        self.flush            = Signal() # i: Discard the sink entries.

        # # #

        in_chain = Signal()
        half     = Signal() # Second block of the entry.
        final    = Signal() # Last block of the entry.
        self.comb += [
            final.eq(half | (sink.blocks == 1)),
            source.valid.eq(sink.valid & ~self.flush & (in_chain | self.chains_available)),
            source.data.eq(Mux(half, sink.data[128:], sink.data[:128])),
            source.last.eq(sink.last & final),
            sink.ready.eq(self.flush | (source.valid & source.ready & final)),
            self.chain_end.eq(sink.valid & sink.ready & sink.last),
        ]
        self.sync += [
            If(self.flush,
                half.eq(0),
                in_chain.eq(0),
            ).Elif(source.valid & source.ready,
                half.eq(~final),
                If(final, in_chain.eq(~sink.last)),
            )
        ]

# Gen 2 Link TX ------------------------------------------------------------------------------------

class Gen2LinkTX(LiteXModule):
    """Link Layer words (sys clock domain) -> Gen2TXFramer -> Data Blocks payloads (tx clock domain,
    to Gen2TX).

    nbytes=8: units buffer (sys -> tx) -> Gen2TXPacker (tx). Units are stored and forwarded: a unit
    is started once completely written in the buffer and visible on its read side. The units
    written count is moved to the tx clock domain (gray coded) and delayed to cover the buffer CDC
    latency (else the packer could start a unit whose last entries are not yet readable: Idle
    Symbols inside a packet).

    nbytes=32 (wider transceiver words, ex: 80-bit at 125MHz: a Data Block every 1.65 cycles):
    framer entries grouped by 32 bytes, units buffer -> Gen2TXBlocksPacker (sys, up to 2 blocks
    per cycle) -> blocks buffer (sys -> tx) -> Gen2TXBlocksReader (tx): units stored and forwarded
    in the sys clock domain, chains of blocks read once complete in the blocks buffer (chains
    written count moved to the tx clock domain as above).

    Flushed while the link is not in U0 (stale units of a previous U0 period, units cut by a link
    state change).
    """
    def __init__(self, buffer_depth=512, units_delay=8, link_words=1, nbytes=8):
        assert link_words in [1, 2, 4]
        assert nbytes in [8, 32]
        assert (link_words != 4) or (nbytes == 32)
        # Link Layer words (link_words=1), word pairs (link_words=2, gen2_link_layout) or quads
        # (link_words=4: framer on quads, 16 bytes per cycle in the DPPs).
        self.sink        = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)] if link_words == 1
            else gen2_link_layout(link_words))
        self.source      = source = stream.Endpoint([("data", 128)])
        self.flush       = Signal() # i: Flush (sys, link not in U0): framer reset, buffers drained.
        self.skp_pending = Signal() # i: SKP Ordered Set pending (Gen2TX, tx clock domain).

        # # #

        self.framer = framer = ResetInserter()(Gen2TXFramer(nwords=4 if link_words == 4 else 2))
        flush_tx = Signal()
        self.specials += MultiReg(self.flush, flush_tx, "tx")
        self.comb += framer.reset.eq(self.flush)

        def gray_count_cdc(ce_write, delay):
            # Gray counter (sys), resynchronized and delayed (tx).
            written    = GrayCounter(16)
            written_tx = Signal(16)
            written_d  = [Signal(16) for _ in range(delay)]
            self.submodules += written
            self.specials += MultiReg(written.q, written_tx, "tx")
            self.sync.tx += [written_d[0].eq(written_tx)] + [
                written_d[i].eq(written_d[i-1]) for i in range(1, delay)]
            self.comb += written.ce.eq(ce_write)
            return written_d[-1]

        if nbytes == 8:
            self.buffer = buffer = ClockDomainsRenamer({"write": "sys", "read": "tx"})(
                stream.AsyncFIFO(gen2_tx_unit_layout(8), buffer_depth, buffered=True))
            self.packer = packer = ClockDomainsRenamer("tx")(Gen2TXPacker(8))
            units_read = ClockDomainsRenamer("tx")(GrayCounter(16))
            self.submodules += units_read
            units_written = gray_count_cdc(buffer.sink.valid & buffer.sink.ready & buffer.sink.last,
                units_delay)
            self.comb += [
                sink.connect(framer.sink, omit={"words"}),
                framer.sink.words.eq(sink.words if link_words == 2 else 1),
                framer.source.connect(buffer.sink),
                buffer.source.connect(packer.sink),
                packer.source.connect(source),
                packer.flush.eq(flush_tx),
                packer.skp_pending.eq(self.skp_pending),
                units_read.ce.eq(packer.unit_end),
                packer.units_available.eq(units_written != units_read.q),
            ]
            return

        # 32 bytes: framing/packing (sys) ----------------------------------------------------------
        # Registered valid/ready around the framer and the widener (timings).
        fw          = 4 if link_words == 4 else 2 # Framer words.
        framer_sink = stream.Endpoint(gen2_link_layout(fw))
        if link_words == 4:
            self.comb += sink.connect(framer_sink)
        else:
            self.comb += [
                sink.connect(framer_sink, omit={"words"}),
                framer_sink.words.eq(sink.words if link_words == 2 else 1),
            ]
        self.sink_buffer = sink_buffer = ResetInserter()(Buffer(gen2_link_layout(fw),
            pipe_valid=True, pipe_ready=True))
        self.widener     = widener     = ResetInserter()(Gen2TXUnitsWidener(32, in_bytes=4*fw))
        self.unit_buffer = unit_buffer = ResetInserter()(Buffer(gen2_tx_unit_layout(32),
            pipe_valid=True, pipe_ready=True))
        # Units buffer (store and forward, units bytes up to 1060: 34 entries; 128: block RAM),
        # its output registered to the packer (timings: block RAM output to the packer decisions).
        self.units     = units     = stream.SyncFIFO(gen2_tx_unit_layout(32), 128, buffered=True)
        self.units_reg = units_reg = stream.PipeValid(gen2_tx_unit_layout(32))
        self.packer    = packer    = Gen2TXBlocksPacker()
        skp_pending = Signal()
        self.specials += MultiReg(self.skp_pending, skp_pending)
        self.comb += [
            sink_buffer.reset.eq(self.flush),
            widener.reset.eq(self.flush),
            unit_buffer.reset.eq(self.flush),
            framer_sink.connect(sink_buffer.sink),
            sink_buffer.source.connect(framer.sink),
            framer.source.connect(widener.sink),
            widener.source.connect(unit_buffer.sink),
            unit_buffer.source.connect(units.sink),
            units.source.connect(units_reg.sink),
            units_reg.source.connect(packer.sink),
            packer.flush.eq(self.flush),
            packer.skp_pending.eq(skp_pending),
        ]
        # Complete units in the units buffer, registered (timings): not set on the 2 cycles after a
        # unit is read (count not updated in the flag yet).
        units_written = Signal(16)
        units_read    = Signal(16)
        unit_end_d    = Signal()
        self.sync += [
            If(units.sink.valid & units.sink.ready & units.sink.last,
                units_written.eq(units_written + 1)),
            unit_end_d.eq(packer.unit_end),
            If(unit_end_d, units_read.eq(units_read + 1)),
            packer.units_available.eq((units_written != units_read) & ~packer.unit_end &
                ~unit_end_d),
        ]

        # Blocks buffer (sys -> tx), chains read once complete (tx) --------------------------------
        # 4 chains (timings: pointers), block RAM read at the registered read pointer.
        self.blocks = blocks = ClockDomainsRenamer({"write": "sys", "read": "tx"})(
            AsyncFIFOPipelinedRead(gen2_tx_blocks_layout(2), 64))
        self.reader = reader = ClockDomainsRenamer("tx")(Gen2TXBlocksReader())
        # Registered valid/ready between the reader and Gen2TX (timings: PipeValid first, its
        # payload registers enable from the PipeReady registered valid, not from the Gen2TX block
        # selection/ready: fanout).
        self.reader_buffer = reader_buffer = ClockDomainsRenamer("tx")(ResetInserter()(
            Buffer([("data", 128)], pipe_valid=True, pipe_ready=True)))
        chains_read = ClockDomainsRenamer("tx")(GrayCounter(16))
        self.submodules += chains_read
        chains_written = gray_count_cdc(blocks.sink.valid & blocks.sink.ready & blocks.sink.last,
            units_delay)
        # Registered valid/ready between the blocks buffer and the reader (timings: reader
        # decisions to the buffer read side only on the PipeReady valid, not on the enable of the
        # payload registers). Not reset: flushed entries drained by the reader (chains read count).
        self.read_buffer = read_buffer = ClockDomainsRenamer("tx")(Buffer(gen2_tx_blocks_layout(2),
            pipe_valid=True, pipe_ready=True))
        chain_end_d = Signal()
        self.comb += [
            packer.source.connect(blocks.sink),
            blocks.source.connect(read_buffer.sink),
            read_buffer.source.connect(reader.sink),
            reader.flush.eq(flush_tx),
            reader_buffer.reset.eq(flush_tx),
            reader.source.connect(reader_buffer.sink),
            reader_buffer.source.connect(source),
            chains_read.ce.eq(chain_end_d),
        ]
        self.sync.tx += [
            chain_end_d.eq(reader.chain_end),
            reader.chains_available.eq((chains_written != chains_read.q) & ~reader.chain_end &
                ~chain_end_d),
        ]

# Gen 2 Link RX ------------------------------------------------------------------------------------

class Gen2LinkRX(LiteXModule):
    """Data Blocks (rx clock domain, one every 3.3 cycles at 40-bit, 1.65 at 80-bit, not held) ->
    Gen2RXFraming -> Link Layer words or word pairs (sys clock domain, gated by rx_ready: U0).
    LGOOD_n/LCRD_x Link Commands on source_lc with lc_enable.

    nbytes=8: blocks buffer -> 8 bytes per cycle -> Gen2RXFraming (rx, reset out of the Data
    Stream) -> words buffer (rx -> sys), Link Command words buffer (rx -> sys).

    nbytes=32 (wider transceiver words): Data Stream blocks pairs buffer (rx -> sys, a pair per
    Link Layer cycle: blocks rate up to 2x the Link Layer frequency, the pair starting the Data
    Stream flagged) -> Gen2RXFraming (sys, 32 bytes per cycle, reset by the first pair) -> words
    buffer (sys, level: RX throttling), Link Command words buffer.
    """
    def __init__(self, buffer_depth=1024, link_words=1, nbytes=8):
        assert link_words in [1, 2, 4]
        assert nbytes in [8, 32]
        assert (link_words != 4) or (nbytes == 32)
        self.sink        = sink   = stream.Endpoint([("data", 128)])         # rx clock domain.
        # Link Layer words (link_words=1), word pairs (link_words=2, gen2_link_layout) or quads
        # (link_words=4).
        self.source      = source = stream.Endpoint([("data", 32), ("ctrl", 4)] if link_words == 1
            else gen2_link_layout(link_words))
        self.source_lc   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.lc_enable   = Signal() # i: LGOOD_n/LCRD_x on source_lc (sys clock domain, static).
        self.data_stream = Signal() # i: Receiving the Data Stream (rx clock domain).
        self.rx_ready    = Signal() # i: U0 (sys clock domain).
        self.overflow    = Signal() # o: Blocks buffer overflow (sticky, rx clock domain).

        # # #

        self.comb += sink.ready.eq(1)
        if nbytes == 8:
            fifo      = ClockDomainsRenamer("rx")(stream.SyncFIFO([("data", 128)], 8))
            converter = ClockDomainsRenamer("rx")(stream.Converter(128, 64))
            framing   = ClockDomainsRenamer("rx")(ResetInserter()(Gen2RXFraming(8)))
            buffer    = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
                stream.AsyncFIFO(gen2_link_layout(2), buffer_depth, buffered=True))
            self.lc_buffer = lc_buffer = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
                stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 16, buffered=True))
            self.submodules += fifo, converter, framing, buffer
            self.specials += MultiReg(self.lc_enable, framing.lc_enable, "rx")
            self.comb += [
                fifo.sink.valid.eq(sink.valid),
                fifo.sink.data.eq(sink.data),
                framing.reset.eq(~self.data_stream),
                fifo.source.connect(converter.sink),
                converter.source.connect(framing.sink),
                framing.source.connect(buffer.sink),
                framing.source_lc.connect(lc_buffer.sink),
            ]
            self.sync.rx += If(fifo.sink.valid & ~fifo.sink.ready, self.overflow.eq(1))
        else:
            # Data Stream blocks pairs (32 bytes, rx -> sys: a pair per Link Layer cycle), the pair
            # starting the Data Stream flagged.
            pairs = ClockDomainsRenamer({"write": "rx", "read": "sys"})(
                AsyncFIFOPipelinedRead([("data", 256), ("start", 1)], 128))
            held       = Signal(128, reset_less=True)
            has        = Signal()
            has_c      = [Signal(name="has", attr={("syn_preserve", 1)}) for _ in range(4)] # Copies.
            started    = Signal()
            held_start = Signal()
            self.sync.rx += [
                If(~self.data_stream,
                    has.eq(0),
                    [c.eq(0) for c in has_c],
                    started.eq(0),
                ).Elif(sink.valid,
                    [If(~has_c[i], held[32*i:32*(i+1)].eq(sink.data[32*i:32*(i+1)]))
                        for i in range(4)],
                    If(~has,
                        held_start.eq(~started),
                        started.eq(1),
                    ),
                    has.eq(~has),
                    [c.eq(~has) for c in has_c],
                ),
                If(pairs.sink.valid & ~pairs.sink.ready, self.overflow.eq(1)),
            ]
            self.comb += [
                pairs.sink.valid.eq(sink.valid & self.data_stream & has),
                pairs.sink.data.eq(Cat(held, sink.data)),
                pairs.sink.start.eq(held_start),
            ]
            # RX framing (and its input buffer) reset by the first pair (one cycle, pair then read).
            in_buffer  = ResetInserter()(Buffer([("data", 256)], pipe_valid=True, pipe_ready=True,
                ready_first=True))
            framing    = ResetInserter()(Gen2RXFraming(32))
            out_buffer = Buffer(gen2_link_layout(8), pipe_valid=True, pipe_ready=True)
            buffer     = stream.SyncFIFO(gen2_link_layout(8), buffer_depth//2, buffered=True)
            self.lc_buffer = lc_buffer = stream.SyncFIFO([("data", 32), ("ctrl", 4)], 16,
                buffered=True)
            self.submodules += pairs, in_buffer, framing, out_buffer, buffer
            self.pairs = pairs
            self.level = buffer.level # o: Words buffer level (entries of up to 8 words).
            reset      = Signal()
            reset_done = Signal()
            self.comb += [
                reset.eq(pairs.source.valid & pairs.source.start & ~reset_done),
                in_buffer.reset.eq(reset),
                framing.reset.eq(reset),
                in_buffer.sink.valid.eq(pairs.source.valid & ~reset),
                in_buffer.sink.data.eq(pairs.source.data),
                pairs.source.ready.eq(in_buffer.sink.ready & ~reset),
                in_buffer.source.connect(framing.sink),
                framing.lc_enable.eq(self.lc_enable),
                framing.source.connect(out_buffer.sink),
                out_buffer.source.connect(buffer.sink),
                framing.source_lc.connect(lc_buffer.sink),
            ]
            self.sync += If(reset,
                reset_done.eq(1)
            ).Elif(pairs.source.valid & pairs.source.ready,
                reset_done.eq(0)
            )
        self.framing = framing
        self.buffer  = buffer

        # Link Commands words, gated by rx_ready.
        self.comb += [
            self.source_lc.valid.eq(lc_buffer.source.valid & self.rx_ready),
            self.source_lc.data.eq(lc_buffer.source.data),
            lc_buffer.source.ready.eq(~self.rx_ready | self.source_lc.ready),
        ]

        # Words entries -> word pairs/words, gated by rx_ready.
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
        if link_words >= 2:
            self.comb += source.words.eq(words.words)
