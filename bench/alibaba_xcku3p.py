#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.gen import *

from litex_boards.platforms import alibaba_xcku3p

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe          import USPUSB3SerDes
from usb3_pipe.phy      import USB3PHY
from usb3_pipe.debug    import USB3PHYDebug, USB3SerDesDebug
from usb3_core.examples import add_usb3_example, USB3_EXAMPLES

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

class USB3SoC(SoCCore):
    def __init__(self, platform, sfp=0, with_core=True, example="loopback", cpu_type=None,
        with_serdes_debug=False):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCCore ----------------------------------------------------------------------------------
        # No CPU by default (bridges only), CPU with the console examples (BIOS on the USB3 CDC-ACM).
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                = "USB3SoC",
            ident_version        = True,
            cpu_type             = cpu_type,
            integrated_rom_size  = 0x10000 if cpu_type is not None else 0,
            integrated_sram_size = 0x2000  if cpu_type is not None else 0,
            with_uart            = False,
            with_timer           = cpu_type is not None,
        )

        # JTAGBone ---------------------------------------------------------------------------------
        self.add_jtagbone()

        # USB3 SerDes ------------------------------------------------------------------------------
        # GTY reference clock: 100MHz oscillator through the fabric (CPLL from GTGREFCLK).
        self.usb3_serdes = usb3_serdes = USPUSB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk100"),
            refclk_freq  = 100e6,
            tx_pads      = platform.request("sfp_tx", sfp),
            rx_pads      = platform.request("sfp_rx", sfp))

        # USB3 PHY (PIPE + LTSSM) ------------------------------------------------------------------
        self.usb3_phy = usb3_phy = USB3PHY(
            serdes       = usb3_serdes,
            sys_clk_freq = sys_clk_freq,
            role         = "device")

        # USB3 PHY/SerDes Debug --------------------------------------------------------------------
        self.usb3_debug = USB3PHYDebug(usb3_phy, usb3_serdes, sys_clk_freq)
        if with_serdes_debug:
            self.usb3_serdes_debug = USB3SerDesDebug(usb3_serdes, sys_clk_freq,
                ltssm_state = usb3_phy.ltssm_state)

        # USB3 Core (Example design) ---------------------------------------------------------------
        if with_core:
            add_usb3_example(self, usb3_phy, sys_clk_freq, example=example)
        else:
            self.comb += usb3_phy.source.ready.eq(1)

        # Leds -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("user_led", 0).eq(usb3_serdes.ready),
            platform.request("user_led", 1).eq(usb3_phy.u0),
            platform.request("user_led", 2).eq(usb3_phy.recovery),
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the Alibaba Cloud XCKU3P board (Kintex Ultrascale+ GTY).")
    parser.add_argument("--build",             action="store_true", help="Build bitstream.")
    parser.add_argument("--load",              action="store_true", help="Load bitstream.")
    parser.add_argument("--sfp",               default=0, type=int, help="SFP cage (SFP2USB adapter).", choices=[0, 1])
    parser.add_argument("--no-core",           action="store_true", help="Build without the USB3 Core (PHY only).")
    parser.add_argument("--example",           default="loopback",  help="USB3 Device Core example design.", choices=USB3_EXAMPLES)
    parser.add_argument("--cpu-type",          default=None,        help="CPU (console examples, ex: vexriscv).")
    parser.add_argument("--with-serdes-debug", action="store_true", help="Enable SerDes debug (TX/loopback controls, clocks/RX errors measurements).")
    parser.add_argument("--output-dir",        default=None,        help="Build directory (default: build/alibaba_xcku3p).")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    output_dir = args.output_dir or "build/alibaba_xcku3p"
    platform = alibaba_xcku3p.Platform()
    soc     = USB3SoC(platform,
        sfp               = args.sfp,
        with_core         = not args.no_core,
        example           = args.example,
        cpu_type          = args.cpu_type,
        with_serdes_debug = args.with_serdes_debug)
    builder = Builder(soc, output_dir=output_dir, csr_csv=os.path.join(output_dir, "csr.csv"))
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
