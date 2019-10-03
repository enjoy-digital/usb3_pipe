from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import TSEQ, TS1, TS2

# Ordered Set Receiver -----------------------------------------------------------------------------

class OrderedSetReceiver(Module):
    def __init__(self, ordered_set, n_ordered_sets, data_width):
        self.sink     = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.detected = Signal()

        # # #

        self.comb += self.sink.ready.eq(1)

        # Memory --------------------------------------------------------------------------------
        mem_depth = len(ordered_set.to_bytes())//(data_width//8)
        mem_init  = [int.from_bytes(ordered_set.to_bytes()[4*i:4*(i+1)], "little") for i in range(mem_depth)]
        mem       = Memory(data_width, mem_depth, mem_init)
        port      = mem.get_port(async_read=True)
        self.specials += mem, port

        # Error detection --------------------------------------------------------------------------
        error = Signal()
        self.comb += [
            If(self.sink.valid,
                # Check COM
                If((port.adr == 0) & (self.sink.ctrl[0] != 1),
                    error.eq(1)
                ),
                If((port.adr != 0) & (self.sink.ctrl[0] == 1),
                    error.eq(1)
                ),
                # Check Word
                If(self.sink.data != port.dat_r,
                    error.eq(1)
                )
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
            If(self.sink.valid & ~error & ~self.detected,
                count.eq(count + 1)
            ).Else(
                count.eq(0)
            )
        ]

        # Result -----------------------------------------------------------------------------------
        self.comb += self.detected.eq(count == (mem_depth*n_ordered_sets - 1))
