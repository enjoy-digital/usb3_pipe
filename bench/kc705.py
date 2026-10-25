#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_kc705

from litex.build.generic_platform import *

from litex.soc.cores.clock import *

from usb3_pipe import K7USB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device, usb3_analyzer
from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # SFP (GTX Quad 117) / Through XillyUSB's SFP2USB.
    ("sfp_tx", 0,
        Subsignal("p", Pins("H2")),
        Subsignal("n", Pins("H1")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("G4")),
        Subsignal("n", Pins("G3")),
    ),
    # SMA (GTX Quad 117).
    ("sma_tx", 0,
        Subsignal("p", Pins("K2")),
        Subsignal("n", Pins("K1")),
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("K6")),
        Subsignal("n", Pins("K5")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()
        self.cd_oob = ClockDomain()

        # # #

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq,   margin=0)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(USB3BaseSoC):
    def __init__(self, platform, connector="sfp",
        with_analyzer=False, analyzer_group="link", analyzer_csv="analyzer.csv",
        usb3_kwargs={}, **kwargs):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # USB3BaseSoC ------------------------------------------------------------------------------
        USB3BaseSoC.__init__(self, platform, sys_clk_freq, **kwargs)

        # USB3 SerDes ------------------------------------------------------------------------------
        # GTX reference clock: 125MHz SGMII oscillator, dedicated to the GTX Quad 117 (SFP/SMA).
        usb3_serdes = K7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = platform.request("sgmii_clock"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        add_usb3_device(self, usb3_serdes,
            leds = [platform.request("user_led", i) for i in range(3)],
            **usb3_kwargs)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            self.analyzer = usb3_analyzer(self, analyzer_group, analyzer_csv)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the Xilinx KC705 (Kintex7 GTX).")
    parser.add_argument("--connector", default="sfp", help="Transceiver connector (sfp: SFP2USB adapter).", choices=["sfp", "sma"])
    usb3_soc_args(parser, output_dir="build/xilinx_kc705")
    args = parser.parse_args()

    platform = xilinx_kc705.Platform()
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform,
        connector   = args.connector,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc)

if __name__ == "__main__":
    main()
