#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex_boards.platforms import alibaba_xcku3p

from litex.soc.cores.clock import *

from usb3_pipe import USPUSB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device, usb3_analyzer
from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_oob    = ClockDomain()
        self.cd_clk100 = ClockDomain()

        # # #

        # Clk100.
        clk100_pads = platform.request("clk100")
        clk100      = Signal()
        self.specials += [
            Instance("IBUFDS", i_I=clk100_pads.p, i_IB=clk100_pads.n, o_O=clk100),
            Instance("BUFG",   i_I=clk100, o_O=self.cd_clk100.clk),
        ]

        # PLL.
        self.pll = pll = USPMMCM(speedgrade=-2)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq,   margin=0)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(USB3BaseSoC):
    def __init__(self, platform, sfp=0,
        with_analyzer=False, analyzer_group="link", analyzer_csv="analyzer.csv",
        usb3_kwargs={}, **kwargs):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # USB3BaseSoC ------------------------------------------------------------------------------
        USB3BaseSoC.__init__(self, platform, sys_clk_freq, **kwargs)

        # USB3 SerDes ------------------------------------------------------------------------------
        # GTY reference clock: 100MHz oscillator through the fabric (CPLL from GTGREFCLK).
        usb3_serdes = USPUSB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk100"),
            refclk_freq  = 100e6,
            tx_pads      = platform.request("sfp_tx", sfp),
            rx_pads      = platform.request("sfp_rx", sfp))

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        add_usb3_device(self, usb3_serdes,
            leds = [platform.request("user_led", i) for i in range(3)],
            **usb3_kwargs)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            self.analyzer = usb3_analyzer(self, analyzer_group, analyzer_csv)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the Alibaba Cloud XCKU3P board (Kintex Ultrascale+ GTY).")
    parser.add_argument("--sfp", default=0, type=int, help="SFP cage (SFP2USB adapter).", choices=[0, 1])
    usb3_soc_args(parser, output_dir="build/alibaba_xcku3p", with_uartbone=False) # No UART: JTAGBone.
    args = parser.parse_args()

    platform = alibaba_xcku3p.Platform()
    soc = USB3SoC(platform,
        sfp         = args.sfp,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc)

if __name__ == "__main__":
    main()
