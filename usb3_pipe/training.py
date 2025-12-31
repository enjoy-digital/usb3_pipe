#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import Symbol, TSEQ, TS1, TS1_INV, TS2

# Training Sequence Checker ------------------------------------------------------------------------

class TSChecker(LiteXModule):
    """Training Sequence Checker

    Generic Training Sequence Ordered Set checker.

    This module detects a specific Training Sequence Ordered Set by analyzing the RX stream. When N
    consecutive Ordered Sets have been detected without errors (N configured by n_ordered_sets), a
    detection is reported on detected signals for one cycle. Training config is also reported for
    TS1/TS2 Ordered Sets.
    """
    def __init__(self, ordered_set, n_ordered_sets=1):
        self.sink     = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.detected = Signal() # o
        self.error    = Signal() # o

        self.reset      = Signal()        # o
        self.loopback   = Signal()        # o
        self.scrambling = Signal(reset=1) # o

        # # #

        self.comb += self.sink.ready.eq(1)

        # Memory -----------------------------------------------------------------------------------
        mem_depth = len(ordered_set.to_bytes())//4
        mem_init  = [int.from_bytes(ordered_set.to_bytes()[4*i:4*(i+1)], "little") for i in range(mem_depth)]
        mem       = Memory(32, mem_depth, mem_init)
        port      = mem.get_port(async_read=True)
        self.specials += mem, port


        # Ctrl check -------------------------------------------------------------------------------
        ctrl_init = []
        for i in range(mem_depth):
            ctrl = 0
            for b in range(4):
                ctrl |= (isinstance(ordered_set[4*i + b], Symbol) << b)
            ctrl_init.append(ctrl)
        ctrl_expected = Signal(4)
        self.comb += ctrl_expected.eq(Array(ctrl_init)[port.adr])

        # Data check -------------------------------------------------------------------------------
        error = Signal()
        self.comb += [
            If(self.sink.valid,
                # Check Ctrl
                If(self.sink.ctrl != ctrl_expected,
                    error.eq(1)
                ),
                # Check Word
                If(self.sink.data != port.dat_r,
                    error.eq(1)
                )
            ),
            self.error.eq(error)
        ]

        # Link Config ------------------------------------------------------------------------------
        self.sync += [
            If(self.sink.valid & (port.adr == 1),
                self.reset.eq(      self.sink.data[ 8]),
                self.loopback.eq(   self.sink.data[10]),
                self.scrambling.eq(~self.sink.data[11])
            )
        ]

        # Memory address generation ----------------------------------------------------------------
        self.sync += [
            If(self.sink.valid,
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
            If(self.sink.valid,
                If(~error & ~count_done,
                    count.eq(count + 1)
                ).Else(
                    count.eq(0)
                )
            )
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.detected.eq(self.sink.valid & count_done)

# Training Sequence Generator ----------------------------------------------------------------------

class TSGenerator(LiteXModule):
    """Training Sequence Generator

    Generic Training Sequence Ordered Set generator.

    This module generates a specific Training Sequence Ordered Set to the TX stream. For each start,
    N consecutive Ordered Sets are generated (N configured by n_ordered_sets). Done signal is assert
    on the last cycle of the generation. Training config can also be transmitted for TS1/TS2 Ordered
    Sets.
    """
    def __init__(self, ordered_set, n_ordered_sets=1):
        self.start  = Signal() # i
        self.done   = Signal() # o
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        if ordered_set.name in ["TS1", "TS2"]:
            self.reset      = Signal()        # i
            self.loopback   = Signal()        # i
            self.scrambling = Signal(reset=1) # i

        # # #

        run         = Signal()

        # Memory --------------------------------------------------------------------------------
        mem_depth = len(ordered_set.to_bytes())//4
        mem_init  = [int.from_bytes(ordered_set.to_bytes()[4*i:4*(i+1)], "little") for i in range(mem_depth)]
        mem       = Memory(32, mem_depth, mem_init)
        port      = mem.get_port(async_read=True)
        self.specials += mem, port

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

        # Link Config ------------------------------------------------------------------------------
        link_config = Signal(8)
        if ordered_set.name in ["TS1", "TS2"]:
            self.comb += [
                link_config[0].eq(self.reset),
                link_config[2].eq(self.loopback),
                link_config[3].eq(~self.scrambling)
            ]

        # Data generation --------------------------------------------------------------------------
        if ordered_set.name in ["TS1", "TS2"]:
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
        if ordered_set.name in ["TS1", "TS2"]:
            self.comb += If(port.adr == 1, self.source.data[8:16].eq(link_config))

        # Count ------------------------------------------------------------------------------------
        count = Signal(max=mem_depth*n_ordered_sets)
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
            )
        ]
        # Delimiters -------------------------------------------------------------------------------
        self.comb += [
            self.source.first.eq(count == 0),
            self.source.last.eq(count_done),
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.done.eq(self.source.ready & run & count_done)

# Training Sequence Unit ---------------------------------------------------------------------------

class TSUnit(LiteXModule):
    """Training Sequence Unit

    Detect/generate the Training Sequence Ordered Sets required for a USB3.0 link with simple
    control/status signals.
    """
    def __init__(self, serdes):
        self.rx_enable    = Signal() # i
        self.rx_ts1       = Signal() # o
        self.rx_ts1_inv   = Signal() # o
        self.rx_ts2       = Signal() # o

        self.tx_enable    = Signal() # i
        self.tx_tseq      = Signal() # i
        self.tx_ts1       = Signal() # i
        self.tx_ts2       = Signal() # i
        self.tx_done      = Signal() # o

        # # #

        # Ordered Set Checkers ---------------------------------------------------------------------
        self.ts1_checker       =  ts1_checker       = TSChecker(ordered_set=TS1,     n_ordered_sets=8)
        self.ts1_inv_checker   =  ts1_inv_checker   = TSChecker(ordered_set=TS1_INV, n_ordered_sets=8)
        self.ts2_checker       =  ts2_checker       = TSChecker(ordered_set=TS2,     n_ordered_sets=8)
        self.comb += [
            serdes.source.connect(ts1_checker.sink,       omit={"ready"}),
            serdes.source.connect(ts1_inv_checker.sink,   omit={"ready"}),
            serdes.source.connect(ts2_checker.sink,       omit={"ready"}),
            If(self.rx_enable, serdes.source.ready.eq(1)),
            self.rx_ts1.eq(ts1_checker.detected),
            self.rx_ts1_inv.eq(ts1_inv_checker.detected),
            self.rx_ts2.eq(ts2_checker.detected),
        ]

        # Ordered Set Generators -------------------------------------------------------------------
        # Note: No arbitration between generators. Only one of tx_tseq/tx_ts1/tx_ts2 must be asserted
        # at a time, otherwise multiple sources will connect to serdes.sink simultaneously.
        self.tseq_generator = tseq_generator = TSGenerator(ordered_set=TSEQ, n_ordered_sets=65536)
        self.ts1_generator  =  ts1_generator = TSGenerator(ordered_set=TS1,  n_ordered_sets=16)
        self.ts2_generator  =  ts2_generator = TSGenerator(ordered_set=TS2,  n_ordered_sets=16)
        self.comb += [
            If(self.tx_enable,
                If(self.tx_tseq,
                    tseq_generator.start.eq(1),
                    tseq_generator.source.connect(serdes.sink),
                ),
                If(self.tx_ts1,
                    ts1_generator.start.eq(1),
                    ts1_generator.source.connect(serdes.sink),
                ),
                If(self.tx_ts2,
                    ts2_generator.start.eq(1),
                    ts2_generator.source.connect(serdes.sink),
                ),
            ),
            self.tx_done.eq(tseq_generator.done | ts1_generator.done | ts2_generator.done),
        ]
