#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe.serdes import *
from usb3_pipe import USB3PIPE
from usb3_core.core import USB3Core

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    default_clk_name = "sys_clk"

    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Simulation Serializer/Deserializer Model ---------------------------------------------------------

class USB3SerDesModel(LiteXModule):
    def __init__(self, phy_dw=20, rx_word_shift=0):
        assert phy_dw in [20, 40]
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.tx     = stream.Endpoint([("data", phy_dw)])
        self.rx     = stream.Endpoint([("data", phy_dw)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()       # i
        self.tx_idle     = Signal()       # i
        self.tx_pattern  = Signal(phy_dw) # i

        self.rx_polarity = Signal() # i
        self.rx_idle     = Signal() # o
        self.rx_align    = Signal() # i

        # # #

        nwords = phy_dw//10

        # Control.
        self.comb += self.ready.eq(self.enable) # Ready when enabled

        # Datapath.
        tx_datapath = TXDatapath(phy_dw=nwords*8)
        rx_datapath = RXDatapath(phy_dw=nwords*8)
        self.submodules += tx_datapath, rx_datapath
        self.comb += [
            self.sink.connect(tx_datapath.sink),
            rx_datapath.word_aligner.enable.eq(self.rx_align),
            rx_datapath.source.connect(self.source)
        ]

        # 8b10b Encoders/Decoders.
        encoder  = Encoder(nwords, True)
        decoders = [Decoder(True) for _ in range(nwords)]
        self.submodules += encoder, decoders
        self.comb += tx_datapath.source.ready.eq(1)
        self.comb += rx_datapath.sink.valid.eq(1)
        for i in range(nwords):
            self.comb += [
                encoder.k[i].eq(tx_datapath.source.ctrl[i]),
                encoder.d[i].eq(tx_datapath.source.data[8*i:8*(i+1)]),
                rx_datapath.sink.ctrl[i].eq(decoders[i].k),
                rx_datapath.sink.data[8*i:8*(i+1)].eq(decoders[i].d),
            ]

        # Pattern/Polarity/Shift emulation.
        tx_data    = Signal(phy_dw)
        rx_data    = Signal(phy_dw)
        rx_data_sr = Signal(2*phy_dw)
        self.comb += [
            If(self.tx_pattern != 0,
                tx_data.eq(self.tx_pattern)
            ).Else(
                tx_data.eq(Cat(*[encoder.output[i] for i in range(nwords)])),
            ),
            If(self.tx_polarity,
                self.tx.data.eq(~tx_data)
            ).Else(
                self.tx.data.eq(tx_data)
            )
        ]
        self.comb += [
            If(self.rx_polarity,
                rx_data.eq(~self.rx.data)
            ).Else(
                rx_data.eq(self.rx.data)
            )
        ]
        self.sync += rx_data_sr.eq(Cat(rx_data, rx_data_sr))
        for i in range(nwords):
            self.comb += decoders[i].input.eq(rx_data_sr[10*(rx_word_shift+i):10*(rx_word_shift+i+1)])

    def connect(self, serdes):
        self.comb += [
            self.tx.connect(serdes.rx),
            serdes.tx.connect(self.rx),
            self.rx_idle.eq(serdes.tx_idle),
            serdes.rx_idle.eq(self.tx_idle),
        ]

# USB3PIPESim --------------------------------------------------------------------------------------

class USB3PIPESim(SoCMini):
    def __init__(self, phy_dw=20):
        sys_clk_freq = int(133e6)

        # Platform.
        platform = Platform()
        self.comb += platform.trace.eq(1)

        # SoC Mini.
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # USB3 Host.
        host_usb3_serdes = USB3SerDesModel(phy_dw=phy_dw)
        host_usb3_pipe   = USB3PIPE(
            serdes          = host_usb3_serdes,
            sys_clk_freq    = sys_clk_freq)
        self.submodules += host_usb3_serdes, host_usb3_pipe
        host_usb3_pipe.finalize()
        host_usb3_core = USB3Core(platform)
        self.host_usb3_core = host_usb3_core
        self.comb += [
            host_usb3_serdes.tx_polarity.eq(1), # Inverse TX polarity to test RX auto-polarity
            host_usb3_pipe.source.connect(host_usb3_core.sink),
            host_usb3_core.source.connect(host_usb3_pipe.sink),
            host_usb3_core.reset.eq(~host_usb3_pipe.ready),
        ]

        # USB3 Device.
        dev_usb3_serdes = USB3SerDesModel(phy_dw=phy_dw)
        dev_usb3_pipe   = USB3PIPE(
            serdes          = dev_usb3_serdes,
            sys_clk_freq    = sys_clk_freq)
        self.submodules += dev_usb3_serdes, dev_usb3_pipe
        dev_usb3_pipe.finalize()
        dev_usb3_core = USB3Core(platform)
        self.dev_usb3_core = dev_usb3_core
        self.comb += [
            dev_usb3_serdes.tx_polarity.eq(1), # Inverse TX polarity to test RX auto-polarity
            dev_usb3_pipe.source.connect(dev_usb3_core.sink),
            dev_usb3_core.source.connect(dev_usb3_pipe.sink),
            dev_usb3_core.reset.eq(~dev_usb3_pipe.ready),
        ]

        # Connect Host <--> Device.
        self.comb += host_usb3_serdes.connect(dev_usb3_serdes)

        # Simulation Timer.
        timer = Signal(32)
        self.sync += timer.eq(timer + 1)

        # Simulation Status.
        for pipe, fsm in [
            ["host",  host_usb3_pipe.ltssm.polling.fsm],
            ["dev ",  dev_usb3_pipe.ltssm.polling.fsm]]:
            for state, value in fsm.encoding.items():
                self.sync += [
                    If(fsm.next_state != fsm.state,
                        If(fsm.next_state == value,
                            Display("[%08d] {} entering {} state".format(pipe.upper(), state), timer)
                        )
                    )
                ]

        # Simulation End.
        end_timer = WaitTimer(2**16)
        self.submodules += end_timer
        self.comb += end_timer.wait.eq(host_usb3_pipe.ready & dev_usb3_pipe.ready)
        self.sync += If(end_timer.done, Finish())

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PIPE Simulation")
    parser.add_argument("--trace",       action="store_true", help="Enable VCD tracing.")
    parser.add_argument("--trace-start", default=0,           help="Cycle to start VCD tracing.")
    parser.add_argument("--trace-end",   default=-1,          help="Cycle to end VCD tracing.")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
    os.system("cp usb3_core/daisho/usb3/*.init build/sim/gateware/")

    soc = USB3PIPESim()
    builder = Builder(soc)
    builder.build(sim_config=sim_config,
        opt_level   = "O0",
        interactive = False,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))

if __name__ == "__main__":
    main()
