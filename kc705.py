#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_kc705

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe          import K7USB3SerDes
from usb3_pipe.phy      import USB3PHY
from usb3_pipe.debug    import USB3PHYDebug, USB3SerDesDebug
from usb3_core.examples import add_usb3_example, USB3_EXAMPLES

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

class USB3SoC(SoCCore):
    def __init__(self, platform, connector="sfp", with_uartbone=True, with_jtagbone=False,
        with_core=True, example="loopback", cpu_type=None, with_serdes_debug=False):
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

        # UARTBone / JTAGBone ----------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(baudrate=115200)
        if with_jtagbone:
            self.add_jtagbone()

        # USB3 SerDes ------------------------------------------------------------------------------
        # GTX reference clock: 125MHz SGMII oscillator, dedicated to the GTX Quad 117 (SFP/SMA).
        self.usb3_serdes = usb3_serdes = K7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = platform.request("sgmii_clock"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

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
    parser = argparse.ArgumentParser(description="USB3-PIPE on the Xilinx KC705 (Kintex7 GTX).")
    parser.add_argument("--build",             action="store_true", help="Build bitstream.")
    parser.add_argument("--load",              action="store_true", help="Load bitstream.")
    parser.add_argument("--connector",         default="sfp",       help="Transceiver connector (sfp: SFP2USB adapter).", choices=["sfp", "sma"])
    parser.add_argument("--with-jtagbone",     action="store_true", help="Enable JTAGBone.")
    parser.add_argument("--no-uartbone",       action="store_true", help="Disable UARTBone.")
    parser.add_argument("--no-core",           action="store_true", help="Build without the USB3 Core (PHY only).")
    parser.add_argument("--example",           default="loopback",  help="USB3 Device Core example design.", choices=USB3_EXAMPLES)
    parser.add_argument("--cpu-type",          default=None,        help="CPU (console examples, ex: vexriscv).")
    parser.add_argument("--with-serdes-debug", action="store_true", help="Enable SerDes debug (TX/loopback controls, clocks/RX errors measurements).")
    parser.add_argument("--output-dir",        default=None,        help="Build directory (default: build/xilinx_kc705).")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    output_dir = args.output_dir or "build/xilinx_kc705"
    platform = xilinx_kc705.Platform()
    platform.add_extension(_usb3_io)
    soc     = USB3SoC(platform,
        connector         = args.connector,
        with_uartbone     = not args.no_uartbone,
        with_jtagbone     = args.with_jtagbone,
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
