#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Link-Layer stand-ins (PRBS packets): PHY tests without a Link Layer (sim.py --no-core, Gen 1x2
hardware link tests: bench/acorn.py --lanes 2).

- PRBSPacketGenerator : random length packets of PRBS data words framed by K-code start/end markers.
- PRBSPacketChecker   : checks the PRBSPacketGenerator packets (words/packets/errors counters).
- PRBSLinkTester      : generator + checker on a USB3PHY in U0, with CSRs (hardware link tests).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect     import stream
from litex.soc.interconnect.csr import *

# Link-Layer Stand-In (PRBS Packets) ---------------------------------------------------------------

PRBS_START = 0xfb # K27.7 on all the Symbols of the word: Packet start marker.
PRBS_END   = 0xfd # K29.7 on all the Symbols of the word: Packet end marker.

def prbs_marker(symbol, data_width):
    """Word with symbol (K-code) on all its Symbols, as (data, ctrl)."""
    n = data_width//8
    return (int.from_bytes(bytes([symbol])*n, "little"), 2**n - 1)

def xorshift(x, name):
    """Next state of a 32/64-bit xorshift PRBS (combinatorial), as (statements, signal)."""
    n = len(x)
    s = {32: (13, 17, 5), 64: (13, 7, 17)}[n]
    a = Signal(n, name=name + "_a")
    b = Signal(n, name=name + "_b")
    c = Signal(n, name=name + "_c")
    return [a.eq(x ^ (x << s[0])), b.eq(a ^ (a >> s[1])), c.eq(b ^ (b << s[2]))], c

class PRBSPacketGenerator(LiteXModule):
    """Link-Layer stand-in (TX): random length packets of PRBS data words, framed by K-code start/end
    markers (first/last) and separated by random gaps of 0 to 2**gap_bits - 1 cycles (Logical Idle
    sent by the PIPE).
    """
    def __init__(self, data_width=32, gap_bits=4):
        self.enable = Signal()   # i
        self.source = source = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.words  = Signal(32) # o: Data words sent.

        # # #

        start = prbs_marker(PRBS_START, data_width)
        end   = prbs_marker(PRBS_END,   data_width)

        prbs = Signal(data_width, reset=1)
        prbs_next_stmts, prbs_next = xorshift(prbs, "prbs_gen")
        self.comb += prbs_next_stmts

        count = Signal(5)
        self.fsm = fsm = FSM(reset_state="GAP")
        fsm.act("GAP",
            If(count != 0,
                NextValue(count, count - 1)
            ).Elif(self.enable,
                NextValue(count, prbs[0:5]), # 1-32 data words.
                NextState("START")
            )
        )
        fsm.act("START",
            source.valid.eq(1),
            source.first.eq(1),
            source.data.eq(start[0]),
            source.ctrl.eq(start[1]),
            If(source.ready,
                NextState("DATA")
            )
        )
        fsm.act("DATA",
            source.valid.eq(1),
            source.data.eq(prbs),
            source.ctrl.eq(0),
            If(source.ready,
                NextValue(prbs, prbs_next),
                NextValue(self.words, self.words + 1),
                NextValue(count, count - 1),
                If(count == 0,
                    NextState("END")
                )
            )
        )
        fsm.act("END",
            source.valid.eq(1),
            source.last.eq(1),
            source.data.eq(end[0]),
            source.ctrl.eq(end[1]),
            If(source.ready,
                NextValue(count, prbs[8:8 + gap_bits]), # Gap.
                NextState("GAP")
            )
        )

class PRBSPacketChecker(LiteXModule):
    """Link-Layer stand-in (RX): checks the packets of PRBSPacketGenerator.

    Inside packets, each data word must be the PRBS successor of the previous one (self-synchronizing:
    a corrupted/lost word only gives a bounded number of errors). Outside packets, only Logical Idle
    (descrambled 00h data symbols) is expected.
    """
    def __init__(self, data_width=32):
        self.enable  = Signal()   # i
        self.sink    = sink = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.words   = Signal(32) # o: Data words checked.
        self.packets = Signal(32) # o: Packets received.
        self.errors  = Signal(32) # o: Errors.

        # # #

        start = prbs_marker(PRBS_START, data_width)
        end   = prbs_marker(PRBS_END,   data_width)

        prev      = Signal(data_width)
        prev_ok   = Signal()
        in_packet = Signal()
        prev_next_stmts, prev_next = xorshift(prev, "prbs_chk")
        self.comb += prev_next_stmts
        self.comb += sink.ready.eq(1)

        is_start = (sink.data == start[0]) & (sink.ctrl == start[1])
        is_end   = (sink.data == end[0])   & (sink.ctrl == end[1])
        self.sync += If(self.enable & sink.valid,
            If(is_start,
                If(in_packet, self.errors.eq(self.errors + 1)),
                in_packet.eq(1)
            ).Elif(is_end,
                If(~in_packet, self.errors.eq(self.errors + 1)),
                in_packet.eq(0),
                self.packets.eq(self.packets + 1)
            ).Elif(in_packet,
                self.words.eq(self.words + 1),
                prev.eq(sink.data),
                prev_ok.eq(1),
                If((sink.ctrl != 0) | (prev_ok & (sink.data != prev_next)),
                    self.errors.eq(self.errors + 1)
                )
            ).Elif((sink.data != 0) | (sink.ctrl != 0),
                self.errors.eq(self.errors + 1)
            )
        )

# Link Tester (PRBS Packets, CSRs) -----------------------------------------------------------------

class PRBSLinkTester(LiteXModule):
    """PRBS Link-Layer stand-ins on a USB3PHY (no Link Layer: Gen 1x2 hardware link tests between two
    USB3PHYs, Host and Device roles), packets sent and checked in U0 when enabled.

    data_width: PHY link words width (32: Gen 1x1, 64: Gen 1x2), gap_bits: gaps between packets of 0
    to 2**gap_bits - 1 cycles (Gen 1x2: 2, see doc/gen2-x2-roadmap.md). The counters are free running
    (no clear: clearing the checker inside a packet would count errors), rates/errors are measured
    from their differences.
    """
    def __init__(self, phy, data_width=32, gap_bits=4):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, reset=1, description="Send/check packets in U0."),
        ])
        self._tx_words   = CSRStatus(32, description="Data words sent.")
        self._rx_words   = CSRStatus(32, description="Data words checked.")
        self._rx_packets = CSRStatus(32, description="Packets received.")
        self._rx_errors  = CSRStatus(32, description="Errors.")

        # # #

        self.generator = generator = PRBSPacketGenerator(data_width, gap_bits)
        self.checker   = checker   = PRBSPacketChecker(data_width)
        enable = Signal()
        self.comb += [
            enable.eq(self._control.fields.enable & phy.u0),
            generator.enable.eq(enable),
            checker.enable.eq(enable),
            generator.source.connect(phy.sink),
            phy.source.connect(checker.sink),
            self._tx_words.status.eq(generator.words),
            self._rx_words.status.eq(checker.words),
            self._rx_packets.status.eq(checker.packets),
            self._rx_errors.status.eq(checker.errors),
        ]
