from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM


class RX16to32(Module):
    def __init__(self):
        self.sink   =   sink = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        converter = stream.StrideConverter(
            [("data", 16), ("ctrl", 2)],
            [("data", 32), ("ctrl", 4)],
            reverse=False)
        converter = ClockDomainsRenamer("rx")(converter)
        self.submodules += converter
        self.comb += [
            sink.connect(converter.sink),
            converter.source.connect(source)
        ]


class RXAligner(stream.PipelinedActor):
    def __init__(self):
        self.enable = Signal()
        self.sink = sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        stream.PipelinedActor.__init__(self, 1)

        # # #

        alignment = Signal(2)

        last_data = Signal(32)
        last_ctrl = Signal(4)

        # Register last data/ctrl
        self.sync += [
            If(self.pipe_ce,
                last_data.eq(sink.data),
                last_ctrl.eq(sink.ctrl)
            )
        ]

        # Alignment detection
        self.sync += [
            If(self.enable & sink.valid & self.pipe_ce,
                If(sink.ctrl[0] & (sink.data[0:8] == COM.value),
                    alignment.eq(0)
                ),
                If(sink.ctrl[1] & (sink.data[8:16] == COM.value),
                    alignment.eq(1)
                ),
                If(sink.ctrl[2] & (sink.data[16:24] == COM.value),
                    alignment.eq(2)
                ),
                If(sink.ctrl[3] & (sink.data[24:32] == COM.value),
                    alignment.eq(3)
                )
            )
        ]

        # Do the alignment
        cases = {}
        cases[0] = [
            source.data.eq(last_data),
            source.ctrl.eq(last_ctrl)
        ]
        cases[1] = [
            source.data.eq(Cat(last_data[8:32], sink.data[0:8])),
            source.ctrl.eq(Cat(last_ctrl[1:4], sink.ctrl[0:1])),
        ]
        cases[2] = [
            source.data.eq(Cat(last_data[16:32], sink.data[0:16])),
            source.ctrl.eq(Cat(last_ctrl[2:4], sink.ctrl[0:2])),
        ]
        cases[3] = [
            source.data.eq(Cat(last_data[24:32], sink.data[0:24])),
            source.ctrl.eq(Cat(last_ctrl[3:4], sink.ctrl[0:3])),
        ]
        self.comb += Case(alignment, cases)
