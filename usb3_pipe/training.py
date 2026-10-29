#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Training Sequences (USB 3.2 6.4.1): TSEQ/TS1/TS2 generation and detection.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import Symbol, TSEQ, TS1, TS1_INV, TS2

# Helpers ------------------------------------------------------------------------------------------

def ordered_set_words(ordered_set):
    """Ordered Set as 32-bit words (first symbol on bits 0-7), used as memory init."""
    data = ordered_set.to_bytes()
    return [int.from_bytes(data[4*i:4*(i+1)], "little") for i in range(len(data)//4)]

# TSEQ Ordered Sets transmitted in Polling.RxEQ (7.5.4.7).
TSEQ_COUNT = 65536

# Training Sequence Checker ------------------------------------------------------------------------

class OrderedSetROM:
    """Ordered Set words ROM (adr/dat_r, asynchronous read) as constants lookup: no Memory with
    init contents and asynchronous read (not kept by some toolchains, ex: GowinSynthesis)."""
    def __init__(self, module, init):
        self.adr   = Signal(max=max(len(init), 2))
        self.dat_r = Signal(32)
        module.comb += self.dat_r.eq(Array(Constant(v, 32) for v in init)[self.adr])

class TSChecker(LiteXModule):
    """Training Sequence Checker

    Generic Training Sequence Ordered Set checker.

    This module detects a specific Training Sequence Ordered Set by analyzing the RX stream. When N
    consecutive Ordered Sets have been detected without errors (N configured by n_ordered_sets), a
    detection is reported on detected signals for one cycle. Training config is also reported for
    TS1/TS2 Ordered Sets. The input words and the detection are registered (timings, 2 cycles of
    latency).
    """
    def __init__(self, ordered_set, n_ordered_sets=1):
        self.sink     = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.detected = Signal() # o (Pulse)
        self.error    = Signal() # o (Pulse)

        self.reset      = Signal()        # o (Level)
        self.loopback   = Signal()        # o (Level)
        self.scrambling = Signal(reset=1) # o (Level)

        # # #

        self.comb += self.sink.ready.eq(1)

        # Input register (timings) -----------------------------------------------------------------
        sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.sync += [
            sink.valid.eq(self.sink.valid),
            sink.data.eq(self.sink.data),
            sink.ctrl.eq(self.sink.ctrl),
        ]

        # Memory -----------------------------------------------------------------------------------
        mem_init  = ordered_set_words(ordered_set)
        mem_depth = len(mem_init)
        port      = OrderedSetROM(self, mem_init)

        # Ctrl check -------------------------------------------------------------------------------
        ctrl_init = []
        for i in range(mem_depth):
            ctrl = 0
            for b in range(4):
                ctrl |= (isinstance(ordered_set[4*i + b], Symbol) << b)
            ctrl_init.append(ctrl)

        # Data check -------------------------------------------------------------------------------
        # The Link Functionality Symbol (Symbol 5, Table 6-4/6-5) of TS1/TS2 is variable and
        # excluded from the check.
        has_link_config = ordered_set.has_link_config
        mask_init = [0xffffffff]*mem_depth
        if has_link_config:
            mask_init[1] = 0xffff00ff
        # Received word compared to each Ordered Set word (constants) and the comparison of the
        # expected word selected (shorter than comparing to the selected word: timings).
        matches = Signal(mem_depth)
        for i in range(mem_depth):
            self.comb += matches[i].eq((sink.ctrl == ctrl_init[i]) &
                ((sink.data & mask_init[i]) == (mem_init[i] & mask_init[i])))
        error = Signal()
        self.comb += [
            error.eq(sink.valid & ~Array(matches)[port.adr]),
            self.error.eq(error)
        ]

        # Link Config (Table 6-6) ------------------------------------------------------------------
        # Bit 0 and bit 2 set simultaneously: all training control bits shall be ignored
        # (6.4.1.1.2). Captured on the Link Functionality word and only reported once the whole
        # Ordered Set has been received without error: TS1/TS2 share the same first symbols, so a
        # TS1 (or corrupted TS) must not change the reported TS2 config.
        link_config   = Signal(8)
        link_config_r = Signal(8, reset_less=True)
        self.comb += link_config.eq(sink.data[8:16])
        if has_link_config:
            self.sync += [
                If(sink.valid & (port.adr == 1),
                    link_config_r.eq(link_config)
                ),
                If(sink.valid & ~error & (port.adr == (mem_depth - 1)),
                    If(link_config_r[0] & link_config_r[2],
                        self.reset.eq(0),
                        self.loopback.eq(0),
                        self.scrambling.eq(1),
                    ).Else(
                        self.reset.eq(      link_config_r[0]),
                        self.loopback.eq(   link_config_r[2]),
                        self.scrambling.eq(~link_config_r[3]),
                    )
                )
            ]

        # Memory address generation ----------------------------------------------------------------
        self.sync += [
            If(sink.valid,
                If(~error,
                    If(port.adr == (mem_depth - 1),
                        port.adr.eq(0)
                    ).Else(
                        port.adr.eq(port.adr + 1)
                    )
                ).Else(
                    port.adr.eq(0)
                )
            )
        ]

        # Count ------------------------------------------------------------------------------------
        count      = Signal(max=mem_depth*n_ordered_sets)
        count_done = (count == (mem_depth*n_ordered_sets - 1))
        self.sync += [
            If(sink.valid,
                If(~error & ~count_done,
                    count.eq(count + 1)
                ).Else(
                    count.eq(0)
                )
            )
        ]

        # Result -----------------------------------------------------------------------------------
        # The last word of the last Ordered Set must also be received without error.
        self.sync += self.detected.eq(sink.valid & ~error & count_done)

# Training Sequence Generator ----------------------------------------------------------------------

class TSGenerator(LiteXModule):
    """Training Sequence Generator

    Generic Training Sequence Ordered Set generator.

    This module generates a specific Training Sequence Ordered Set to the TX stream. For each start,
    N consecutive Ordered Sets are generated (N configured by n_ordered_sets). Done signal is
    asserted on the last cycle of the generation. Training config can also be transmitted for
    TS1/TS2 Ordered Sets.
    """
    def __init__(self, ordered_set, n_ordered_sets=1):
        self.start  = Signal() # i (Level)
        self.abort  = Signal() # i (Level): Abort the generation (next one restarts with COM).
        self.done   = Signal() # o (Pulse)
        self.busy   = Signal() # o (Level): Inside an Ordered Set (not on its first word).
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        has_link_config = ordered_set.has_link_config
        if has_link_config:
            self.reset      = Signal()        # i (Level)
            self.loopback   = Signal()        # i (Level)
            self.scrambling = Signal(reset=1) # i (Level)

        # # #

        run = Signal()

        # Memory -----------------------------------------------------------------------------------
        mem_init  = ordered_set_words(ordered_set)
        mem_depth = len(mem_init)
        port      = OrderedSetROM(self, mem_init)

        # Memory address generation ----------------------------------------------------------------
        self.sync += [
            If(self.source.valid,
                If(self.source.ready,
                    If(port.adr == (mem_depth - 1),
                        port.adr.eq(0)
                    ).Else(
                        port.adr.eq(port.adr + 1)
                    )
                )
            ).Else(
                port.adr.eq(0)
            )
        ]

        self.comb += self.busy.eq(port.adr != 0)

        # Link Config ------------------------------------------------------------------------------
        link_config = Signal(8)
        if has_link_config:
            self.comb += [
                link_config[0].eq(self.reset),
                link_config[2].eq(self.loopback),
                link_config[3].eq(~self.scrambling)
            ]

        # Data generation --------------------------------------------------------------------------
        if has_link_config:
            first_ctrl = 0b1111
        else:
            first_ctrl = 0b0001
        self.comb += [
            self.source.valid.eq(self.start | run),
            If(port.adr == 0,
                self.source.ctrl.eq(first_ctrl),
            ).Else(
                self.source.ctrl.eq(0)
            ),
            self.source.data.eq(port.dat_r)
        ]
        if has_link_config:
            self.comb += If(port.adr == 1, self.source.data[8:16].eq(link_config))

        # Count ------------------------------------------------------------------------------------
        count      = Signal(max=mem_depth*n_ordered_sets)
        count_done = (count == (mem_depth*n_ordered_sets - 1))
        self.sync += [
            If(run,
                If(self.source.ready,
                    count.eq(count + 1),
                    If(count_done,
                        run.eq(0),
                        count.eq(0)
                    )
                )
            ).Elif(self.start,
                If(self.source.ready,
                    run.eq(1),
                    count.eq(1)
                )
            ),
            If(self.abort,
                run.eq(0),
                count.eq(0)
            )
        ]

        # Delimiters -------------------------------------------------------------------------------
        self.comb += [
            self.source.first.eq(count == 0),
            self.source.last.eq(count_done),
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.done.eq(run & self.source.valid & self.source.last & self.source.ready)

# Training Sequence Unit ---------------------------------------------------------------------------

class TSUnit(LiteXModule):
    """Training Sequence Unit

    Detect/generate the Training Sequence Ordered Sets required for a USB3.0 link with simple
    control/status signals.

    The detection is always enabled: TS1 must also be detected in U0 (partner entering Recovery,
    7.5.6.2).

    The TSEQ/TS1/TS2 transmission requests are taken into account on the Ordered Sets boundaries: an
    Ordered Set is always completed (tx_active while transmitting), including when the requests come
    from a LTSSM in another clock domain and change after the end of a burst (tx_done).
    """
    def __init__(self, serdes):
        self.rx_ts1            = Signal()        # o (Pulse)
        self.rx_ts1_inv        = Signal()        # o (Pulse)
        self.rx_ts2            = Signal()        # o (Pulse)
        self.rx_ts2_reset      = Signal()        # o (Level)
        self.rx_ts2_loopback   = Signal()        # o (Level)
        self.rx_ts2_scrambling = Signal(reset=1) # o (Level)

        self.tx_enable         = Signal()        # i (Level)
        self.tx_tseq           = Signal()        # i (Level)
        self.tx_ts1            = Signal()        # i (Level)
        self.tx_ts2            = Signal()        # i (Level)
        self.tx_ts2_reset      = Signal()        # i (Level)
        self.tx_ts2_loopback   = Signal()        # i (Level)
        self.tx_ts2_scrambling = Signal(reset=1) # i (Level)
        self.tx_done           = Signal()        # o (Pulse)
        self.tx_active         = Signal()        # o (Level): Ordered Sets transmitted on serdes.sink.

        # # #

        # Ordered Set Checkers ---------------------------------------------------------------------
        self.ts1_checker     = ts1_checker     = TSChecker(ordered_set=TS1,     n_ordered_sets=8)
        self.ts1_inv_checker = ts1_inv_checker = TSChecker(ordered_set=TS1_INV, n_ordered_sets=8)
        self.ts2_checker     = ts2_checker     = TSChecker(ordered_set=TS2,     n_ordered_sets=8)
        self.comb += [
            serdes.source.connect(ts1_checker.sink,     omit={"ready"}),
            serdes.source.connect(ts1_inv_checker.sink, omit={"ready"}),
            serdes.source.connect(ts2_checker.sink,     omit={"ready"}),
            self.rx_ts1.eq(ts1_checker.detected),
            self.rx_ts1_inv.eq(ts1_inv_checker.detected),
            self.rx_ts2.eq(ts2_checker.detected),
            # Link Functionality of the last received TS2 (6.4.1.1.2, Table 6-6).
            self.rx_ts2_reset.eq(ts2_checker.reset),
            self.rx_ts2_loopback.eq(ts2_checker.loopback),
            self.rx_ts2_scrambling.eq(ts2_checker.scrambling),
        ]

        # Ordered Set Generators -------------------------------------------------------------------
        # Note: No arbitration between generators. Only one of tx_tseq/tx_ts1/tx_ts2 must be
        # asserted at a time, otherwise multiple sources will connect to serdes.sink simultaneously.
        self.tseq_generator = tseq_generator = TSGenerator(TSEQ, n_ordered_sets=TSEQ_COUNT)
        self.ts1_generator  = ts1_generator  = TSGenerator(TS1,  n_ordered_sets=16)
        self.ts2_generator  = ts2_generator  = TSGenerator(TS2,  n_ordered_sets=16)
        # Requests selection, kept inside the Ordered Sets.
        generators = [tseq_generator, ts1_generator, ts2_generator]
        requests   = [self.tx_tseq, self.tx_ts1, self.tx_ts2]
        busy       = Signal()
        sel        = Signal(3)
        sel_r      = Signal(3)
        self.comb += [
            busy.eq(Reduce("OR", [g.busy for g in generators])),
            If(busy,
                sel.eq(sel_r)
            ).Else(
                sel.eq(Cat(*[self.tx_enable & r for r in requests]))
            ),
            self.tx_active.eq(sel != 0),
        ]
        self.sync += sel_r.eq(sel)
        # Generations are aborted when no longer requested (tx_enable deasserted or other TS
        # requested by the LTSSM): an interrupted burst restarts on an Ordered Set boundary (with
        # COM, 6.4.1.1) and with a full burst.
        for i, g in enumerate(generators):
            self.comb += [
                g.abort.eq(~sel[i]),
                If(sel[i],
                    g.start.eq(1),
                    g.source.connect(serdes.sink),
                ),
            ]
        self.comb += [
            self.tx_done.eq(tseq_generator.done | ts1_generator.done | ts2_generator.done),
            # Link Functionality of the transmitted TS2 (Hot Reset/Loopback, 7.5.4.9.1).
            ts2_generator.reset.eq(self.tx_ts2_reset),
            ts2_generator.loopback.eq(self.tx_ts2_loopback),
            ts2_generator.scrambling.eq(self.tx_ts2_scrambling),
        ]
