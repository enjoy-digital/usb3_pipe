#!/usr/bin/env python3

# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import sys

from migen import *

from litex.boards.platforms import pcie_screamer

from litex.build.generic_platform import *
from litex.build.xilinx import VivadoProgrammer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from litescope import LiteScopeAnalyzer

from usb3_pipe import A7USB3SerDes, USB3PIPE
from usb3_core.core import USB3Core

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # PCIe
    ("pcie_tx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("A6")),
    ),

    ("pcie_rx", 0,
        Subsignal("p", Pins("B10")),
        Subsignal("n", Pins("A10")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys    = ClockDomain()
        self.clock_domains.cd_oob    = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_oob,    sys_clk_freq/8)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, with_analyzer=False):
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.bridge.wishbone)

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk125"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request("pcie_tx"),
            rx_pads      = platform.request("pcie_rx"))
        self.submodules += usb3_serdes
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules.usb3_pipe = usb3_pipe
        self.comb += usb3_pipe.reset.eq(~platform.request("user_btn", 0))

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
        self.comb += platform.request("user_led", 0).eq(usb3_serdes.ready)
        self.comb += platform.request("user_led", 1).eq(usb3_pipe.ready)

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

# Load ---------------------------------------------------------------------------------------------

def load():
    prog = VivadoProgrammer()
    prog.load_bitstream("build/gateware/top.bit")
    exit()

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true", help="build bitstream")
    parser.add_argument("--load",  action="store_true", help="load bitstream (to SRAM)")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    if args.build:
        os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
        os.system("cp usb3_core/daisho/usb3/*.init build/gateware/")
        platform = pcie_screamer.Platform()
        platform.add_extension(_usb3_io)
        soc     = USB3SoC(platform)
        builder = Builder(soc, output_dir="build", csr_csv="tools/csr.csv")
        builder.build()

    if args.load:
        load()

if __name__ == "__main__":
    main()
