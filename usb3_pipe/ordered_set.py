# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import TSEQ, TS1, TS2

# Ordered Set Receiver -----------------------------------------------------------------------------

class OrderedSetReceiver(Module):
    def __init__(self, ordered_set, n_ordered_sets, data_width):
        assert data_width in [16, 32]
        self.sink     = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.detected = Signal() # o

        if ordered_set.name in ["TS1", "TS2"]:
            self.reset      = Signal() # o
            self.loopback   = Signal() # o
            self.scrambling = Signal() # o

        # # #

        self.comb += self.sink.ready.eq(1)

        # Memory --------------------------------------------------------------------------------
        mem_depth = len(ordered_set.to_bytes())//(data_width//8)
        mem_init  = [int.from_bytes(ordered_set.to_bytes()[4*i:4*(i+1)], "little") for i in range(mem_depth)]
        mem       = Memory(data_width, mem_depth, mem_init)
        port      = mem.get_port(async_read=True)
        self.specials += mem, port

        # Data check -------------------------------------------------------------------------------
        error      = Signal()
        error_mask = Signal(data_width, reset=2**data_width-1)
        if ordered_set.name in ["TS1", "TS2"]:
            first_ctrl = 2**(data_width//8) - 1
            if data_width == 32:
                self.comb += If(port.adr == 1, error_mask.eq(0xffff00ff))
            else:
                self.comb += If(port.adr == 2, error_mask.eq(0x00ff))
        else:
            first_ctrl = 1
        self.comb += [
            If(self.sink.valid,
                # Check Comma
                If((port.adr == 0) & (self.sink.ctrl != first_ctrl),
                    error.eq(1)
                ),
                If((port.adr != 0) & (self.sink.ctrl != 0),
                    error.eq(1)
                ),
                # Check Word
                If((self.sink.data & error_mask) != (port.dat_r & error_mask),
                    error.eq(1)
                )
            )
        ]

        # Link Config ------------------------------------------------------------------------------
        if ordered_set.name in ["TS1", "TS2"]:
            if data_width == 32:
                self.sync += [
                    If(self.sink.valid & (port.adr == 1),
                        self.reset.eq(      self.sink.data[ 8]),
                        self.loopback.eq(   self.sink.data[10]),
                        self.scrambling.eq(~self.sink.data[11])
                    )
                ]
            else:
                self.sync += [
                    If(self.sink.valid & (port.adr == 2),
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
        count = Signal(max=mem_depth*n_ordered_sets)
        self.sync += [
            If(self.sink.valid,
                If(~error & ~self.detected,
                    count.eq(count + 1)
                ).Else(
                    count.eq(0)
                )
            )
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.detected.eq(count == (mem_depth*n_ordered_sets - 1))

# Ordered Set Transmitter --------------------------------------------------------------------------

class OrderedSetTransmitter(Module):
    def __init__(self, ordered_set, n_ordered_sets, data_width):
        assert data_width in [16, 32]
        self.send = Signal() # i
        self.done = Signal() # i
        self.source = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])

        if ordered_set.name in ["TS1", "TS2"]:
            self.reset      = Signal() # i
            self.loopback   = Signal() # i
            self.scrambling = Signal() # i

        # # #

        run = Signal()

        # Memory --------------------------------------------------------------------------------
        mem_depth = len(ordered_set.to_bytes())//(data_width//8)
        mem_init  = [int.from_bytes(ordered_set.to_bytes()[4*i:4*(i+1)], "little") for i in range(mem_depth)]
        mem       = Memory(data_width, mem_depth, mem_init)
        port      = mem.get_port(async_read=True)
        self.specials += mem, port

        # Memory address generation ----------------------------------------------------------------
        self.sync += [
            If(self.source.valid & self.source.ready,
                If(port.adr == (mem_depth - 1),
                    port.adr.eq(0)
                ).Else(
                    port.adr.eq(port.adr + 1)
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
                link_config[1].eq(self.loopback),
                link_config[2].eq(~self.scrambling)
            ]

        # Data generation --------------------------------------------------------------------------
        if ordered_set.name in ["TS1", "TS2"]:
            first_ctrl = 2**(data_width//8) - 1
        else:
            first_ctrl = 1
        self.comb += [
            self.source.valid.eq(self.send | ~self.done),
            If(port.adr == 0,
                self.source.ctrl.eq(first_ctrl),
            ).Else(
                self.source.ctrl.eq(0)
            ),
            self.source.data.eq(port.dat_r)
        ]
        if ordered_set.name in ["TS1", "TS2"]:
            if data_width == 32:
                self.comb += If(port.adr == 1, self.source.data[8:16].eq(link_config))
            else:
                self.comb += If(port.adr == 2, self.source.data[8:16].eq(link_config))

        # Count ------------------------------------------------------------------------------------
        count = Signal(max=mem_depth*n_ordered_sets)
        self.sync += [
            If(self.send,
                run.eq(1),
                count.eq(0),
            ).Elif(self.done,
                run.eq(0),
            ).Else(
                count.eq(count + 1)
            )
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.done.eq(count == (mem_depth*n_ordered_sets - 1))
