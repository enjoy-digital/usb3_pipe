#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab on the SQRL Acorn CLE-215+ (Artix7 GTP), LiteX Acorn Baseboard Mini: development/
validation target (USB3 through the SFP2USB adapter, same SerDes setup as bench/acorn.py).

DUT pads on the 4 baseboard debug pins (LVCMOS33):

| Debug pin | FPGA | Function                                                  |
|-----------|------|-----------------------------------------------------------|
| 0         | H5   | DUT UART0 TX                                              |
| 1         | J5   | DUT UART0 RX (loop to pin 0 for tests)                    |
| 2         | J2   | DUT GPIO0                                                 |
| 3         | K2   | DUT GPIO1 (loop to pin 2: GPIO0 output -> GPIO1 edges)    |

DUT UARTs 1-3 are internally looped back (TX to RX), power switch 0 drives user_led3 (on: led on).
No UARTBone on the baseboard UART: CSRs through the USB3 control CDC-ACM or JTAGBone.
"""

import os
import sys
import argparse

# Repository root on the path.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from migen import *

from litex.gen import *

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *

from litex.soc.cores.clock import *

from usb3_pipe import A7USB3SerDes

from agentlab.soc import AgentLabSoC, agentlab_args, agentlab_build

# IOs ----------------------------------------------------------------------------------------------

_usb3_io = [
    # SFP (Baseboard Mini).
    ("sfp_tx", 0,
        Subsignal("p", Pins("D5")),
        Subsignal("n", Pins("C5")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("D11")),
        Subsignal("n", Pins("C11")),
    ),
]

_dut_io = [
    # DUT pads on the debug pins.
    ("dut_uart", 0,
        Subsignal("tx", Pins("H5")),
        Subsignal("rx", Pins("J5"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33"),
    ),
    ("dut_gpio", 0, Pins("J2 K2"), IOStandard("LVCMOS33"), Misc("PULLDOWN=TRUE")),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_oob    = ClockDomain()
        self.cd_clk200 = ClockDomain()

        # # #

        # Clk200.
        clk200_pads = platform.request("clk200")
        clk200      = Signal()
        self.specials += [
            Instance("IBUFDS", i_I=clk200_pads.p, i_IB=clk200_pads.n, o_O=clk200),
            Instance("BUFG",   i_I=clk200, o_O=self.cd_clk200.clk),
        ]

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq,   margin=0)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

# AgentLab SoC -------------------------------------------------------------------------------------

class AgentLabAcorn(AgentLabSoC):
    def __init__(self, platform, with_jtagbone=True):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # AgentLabSoC ------------------------------------------------------------------------------
        AgentLabSoC.__init__(self, platform, sys_clk_freq, with_jtagbone=with_jtagbone)

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk200"), # Directly from the oscillator (no PLL jitter).
            refclk_freq  = 200e6,
            tx_pads      = platform.request("sfp_tx"),
            rx_pads      = platform.request("sfp_rx"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # AgentLab ---------------------------------------------------------------------------------
        power_led = Signal() # Power switch 0 on user_led3 (active low).
        self.comb += platform.request("user_led", 3).eq(~power_led)
        dut_pads = {
            "uarts" : [platform.request("dut_uart", 0), None, None, None],
            "gpio"  : platform.request("dut_gpio"),
            "power" : power_led,
        }
        self.add_agentlab(usb3_serdes, dut_pads,
            leds          = [platform.request("user_led", i) for i in range(3)],
            leds_inverted = True)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="AgentLab on the SQRL Acorn CLE-215+ (LiteX Acorn Baseboard Mini).")
    agentlab_args(parser, output_dir="build/agentlab_acorn")
    parser.add_argument("--no-jtagbone", action="store_true", help="Disable JTAGBone.")
    parser.add_argument("--vivado-place-directive", default="default", help="Vivado place_design directive (see bench/acorn.py).")
    args = parser.parse_args()

    platform = sqrl_acorn.Platform(with_multiboot=False)
    platform.add_extension(_usb3_io)
    platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
    platform.add_extension(_dut_io)
    soc = AgentLabAcorn(platform, with_jtagbone=not args.no_jtagbone)
    agentlab_build(parser, args, soc, vivado_place_directive=args.vivado_place_directive)

if __name__ == "__main__":
    main()
