#!/usr/bin/env python3

# This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex_boards.platforms import ecpix5

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from litescope import LiteScopeAnalyzer

from usb3_pipe import ECP5USB3SerDes, USB3PIPE
from usb3_core.core import USB3Core

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    ("usb_en",    0, Pins("C23"), IOStandard("LVCMOS33")),
    ("usb_amsel", 0, Pins("B26"), IOStandard("LVCMOS33")),
    ("usb_dir",   0, Pins("B23"), IOStandard("LVCMOS33")),
    ("usb_pol",   0, Pins("D26"), IOStandard("LVCMOS33")),
    ("usb_tx", 0,
        Subsignal("p", Pins("AD7")),
        Subsignal("n", Pins("AD8")),
    ),
    ("usb_rx", 0,
        Subsignal("p", Pins("AF6")),
        Subsignal("n", Pins("AF7")),
    ),

    ("usb_tx", 1,
        Subsignal("p", Pins("AD10")),
        Subsignal("n", Pins("AD11")),
    ),
    ("usb_rx", 1,
        Subsignal("p", Pins("AF9")),
        Subsignal("n", Pins("AF10")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_por    = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()

        # # #

        # clk / rst
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # power on reset
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(ClockSignal())
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # pll
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_clk200, 200e6)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | ~pll.locked | ~rst_n)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="usb", lane=0, with_analyzer=False):
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.bridge.wishbone)

        # USB3 Mux ---------------------------------------------------------------------------------
        self.comb += [
            platform.request("usb_pol").eq(platform.request("usb_dir")),
            platform.request("usb_en").eq(1),
            platform.request("usb_amsel").eq(0),
        ]

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = ECP5USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk200"),
            refclk_freq  = 200e6,
            tx_pads      = platform.request(connector + "_tx", lane),
            rx_pads      = platform.request(connector + "_rx", lane),
            channel      = 1)
        self.submodules += usb3_serdes

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules.usb3_pipe = usb3_pipe
        #self.comb += usb3_pipe.reset.eq(~platform.request("rst_n"))

        # USB3 Core --------------------------------------------------------------------------------
        usb3_core = USB3Core(platform)
        self.submodules.usb3_core = usb3_core
        self.comb += [
            usb3_pipe.source.connect(usb3_core.sink),
            usb3_core.source.connect(usb3_pipe.sink),
            usb3_core.reset.eq(~usb3_pipe.ready),
        ]
        self.add_csr("usb3_core")

        # Leds -------------------------------------------------------------------------------------
        rgb_leds = [platform.request("rgb_led") for i in range(4)]
        counter = Signal(32)
        self.sync += counter.eq(counter + 1)
        self.comb += [
            rgb_leds[0].r.eq(usb3_serdes.ready),
            rgb_leds[0].g.eq(~usb3_serdes.ready),
            rgb_leds[0].b.eq(1),
            rgb_leds[1].r.eq(usb3_pipe.ready),
            rgb_leds[1].g.eq(~usb3_pipe.ready),
            rgb_leds[1].b.eq(1),
            rgb_leds[2].r.eq(1),
            rgb_leds[2].g.eq(1),
            rgb_leds[2].b.eq(1),
            rgb_leds[3].r.eq(1),
            rgb_leds[3].g.eq(1),
            rgb_leds[3].b.eq(counter[26]),
        ]

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                # LFPS
                usb3_serdes.tx_idle,
                usb3_serdes.rx_idle,
                usb3_serdes.tx_pattern,
                usb3_serdes.rx_polarity,
                usb3_pipe.lfps.rx_polling,
                usb3_pipe.lfps.tx_polling,

                # Training Sequence
                usb3_pipe.ts.tx_enable,
                usb3_pipe.ts.rx_ts1,
                usb3_pipe.ts.rx_ts2,
                usb3_pipe.ts.tx_enable,
                usb3_pipe.ts.tx_tseq,
                usb3_pipe.ts.tx_ts1,
                usb3_pipe.ts.tx_ts2,
                usb3_pipe.ts.tx_done,

                # LTSSM
                usb3_pipe.ltssm.polling.fsm,
                usb3_pipe.ready,

                # Endpoints
                usb3_serdes.rx_datapath.skip_remover.skip,
                usb3_serdes.source,
                usb3_serdes.sink,
                usb3_pipe.source,
                usb3_pipe.sink,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, csr_csv="tools/analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    with open("README.md") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[1:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="build bitstream")
    parser.add_argument("--load",  action="store_true", help="load bitstream (to SRAM)")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    os.makedirs("build/gateware", exist_ok=True)
    os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
    os.system("cp usb3_core/daisho/usb3/*.init build/gateware/")
    platform = ecpix5.Platform(toolchain="trellis")
    platform.add_extension(_usb3_io)
    soc     = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="tools/csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".svf"))

if __name__ == "__main__":
    main()
