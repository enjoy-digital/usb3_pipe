#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab on the Xilinx KC705 (Kintex7 GTX): main target (USB3 through the SFP2USB adapter on the
SFP cage, GTX Quad 117 with the 125MHz SGMII reference clock, as bench/kc705.py).

DUT pads on the FMC LPC LA pairs, used single-ended (VADJ bank, 2.5V by default on the KC705:
LVCMOS25; change the IOStandard if VADJ is reprogrammed to 1.8V/3.3V):

| FMC LPC            | Function                                                        |
|--------------------|-----------------------------------------------------------------|
| LA02_P / LA02_N    | DUT UART0 TX / RX                                               |
| LA03_P / LA03_N    | DUT UART1 TX / RX                                               |
| LA04_P / LA04_N    | DUT UART2 TX / RX                                               |
| LA05_P / LA05_N    | DUT UART3 TX / RX                                               |
| LA06-LA13 P/N      | DUT GPIO0-15 (LA06_P: GPIO0, LA06_N: GPIO1, ... LA13_N: GPIO15) |
| LA14_P/N, LA15_P/N | DUT power switches 0-3 (LA14_P: 0, LA14_N: 1, LA15_P: 2, ...)   |
| LA16-LA31 P/N      | Logic analyzer channels 0-31 (LA16_P: 0, LA16_N: 1, ... LA31_N) |
| LA32-LA33 P/N      | Reserved: JTAG/SWD TCK/SWCLK, TMS/SWDIO, TDI, TDO (phase 3)     |
| LA01_CC_P          | Reserved: JTAG/SWD nRESET (phase 3)                             |
| LA00_CC, LA01_CC_N | Unused (clock capable: external LA clock, later)                |

Logic analyzer ring buffer in the DDR3 SODIMM (MT8JTF12864, 1GB, initialized by the VexRiscv BIOS:
console on a stub UART). --no-dram: phase 1 instrument only (no CPU/DRAM/logic analyzer).

Eyes (--with-eyes, default): UVC view of the bench (interfaces 12-13, 11-12 with --no-dram, EP14 IN).
"""

import os
import sys
import argparse

# Repository root on the path.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_kc705

from litex.build.generic_platform import *

from litex.soc.cores.clock import *

from litedram.modules import MT8JTF12864
from litedram.phy     import s7ddrphy

from usb3_pipe import K7USB3SerDes

from agentlab.soc import AgentLabSoC, agentlab_args, agentlab_build

# IOs ----------------------------------------------------------------------------------------------

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
]

def _lpc_pins(pairs):
    return " ".join(f"LPC:LA{pair}_{pol}" for pair in pairs for pol in ["P", "N"])

_dut_io = [
    # DUT UARTs.
    *[("dut_uart", n,
        Subsignal("tx", Pins(f"LPC:LA{2 + n:02d}_P")),
        Subsignal("rx", Pins(f"LPC:LA{2 + n:02d}_N"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS25"),
    ) for n in range(4)],

    # DUT GPIOs.
    ("dut_gpio",  0, Pins(_lpc_pins(["06", "07", "08", "09", "10", "11", "12", "13"])),
        IOStandard("LVCMOS25")),

    # DUT power switches.
    ("dut_power", 0, Pins(_lpc_pins(["14", "15"])), IOStandard("LVCMOS25")),

    # Logic analyzer channels.
    ("dut_la", 0, Pins(_lpc_pins(["16", "17_CC", "18_CC", "19", "20", "21", "22", "23",
        "24", "25", "26", "27", "28", "29", "30", "31"])), IOStandard("LVCMOS25")),

    # JTAG/SWD (phase 3).
    ("dut_jtag", 0,
        Subsignal("tck",   Pins("LPC:LA32_P")),
        Subsignal("tms",   Pins("LPC:LA32_N")),
        Subsignal("tdi",   Pins("LPC:LA33_P")),
        Subsignal("tdo",   Pins("LPC:LA33_N")),
        Subsignal("srst",  Pins("LPC:LA01_CC_P")),
        IOStandard("LVCMOS25"),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_dram=True):
        self.cd_sys = ClockDomain()
        self.cd_oob = ClockDomain()
        if with_dram:
            self.cd_sys4x  = ClockDomain()
            self.cd_idelay = ClockDomain()

        # # #

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq,   margin=0)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

        # DRAM clocks (K7DDRPHY: sys4x, IDELAYCTRL reference).
        if with_dram:
            pll.create_clkout(self.cd_sys4x,  4*sys_clk_freq, margin=0)
            pll.create_clkout(self.cd_idelay, 200e6,          margin=0)
            self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# AgentLab SoC -------------------------------------------------------------------------------------

class AgentLabKC705(AgentLabSoC):
    def __init__(self, platform, with_uartbone=True, with_jtagbone=False, with_dram=True,
        with_eyes=True):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_dram=with_dram)

        # AgentLabSoC ------------------------------------------------------------------------------
        AgentLabSoC.__init__(self, platform, sys_clk_freq,
            with_uartbone = with_uartbone,
            with_jtagbone = with_jtagbone,
            cpu_type      = "vexriscv" if with_dram else None)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if with_dram:
            self.add_agentlab_dram(
                phy = s7ddrphy.K7DDRPHY(platform.request("ddram"),
                    memtype      = "DDR3",
                    nphases      = 4,
                    sys_clk_freq = sys_clk_freq),
                module = MT8JTF12864(sys_clk_freq, "1:4"))

        # USB3 SerDes ------------------------------------------------------------------------------
        # GTX reference clock: 125MHz SGMII oscillator, dedicated to the GTX Quad 117 (SFP/SMA).
        usb3_serdes = K7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = platform.request("sgmii_clock"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request("sfp_tx"),
            rx_pads      = platform.request("sfp_rx"))
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # AgentLab ---------------------------------------------------------------------------------
        dut_pads = {
            "uarts" : [platform.request("dut_uart", n) for n in range(4)],
            "gpio"  : platform.request("dut_gpio"),
            "power" : platform.request("dut_power"),
            "la"    : platform.request("dut_la") if with_dram else None,
        }
        self.add_agentlab(usb3_serdes, dut_pads,
            leds      = [platform.request("user_led", i) for i in range(3)],
            with_eyes = with_eyes)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="AgentLab on the Xilinx KC705 (FMC LPC DUT pads).")
    agentlab_args(parser, output_dir="build/agentlab_kc705")
    parser.add_argument("--no-uartbone",   action="store_true", help="Disable UARTBone (board UART).")
    parser.add_argument("--with-jtagbone", action="store_true", help="Enable JTAGBone.")
    parser.add_argument("--no-dram",       action="store_true", help="No DRAM/CPU/logic analyzer (phase 1 instrument).")
    parser.add_argument("--with-eyes",     action="store_true", default=True, help="Enable Eyes (UVC view of the bench, default).")
    parser.add_argument("--no-eyes",       action="store_false", dest="with_eyes", help="Disable Eyes.")
    args = parser.parse_args()

    platform = xilinx_kc705.Platform()
    platform.add_extension(_usb3_io)
    platform.add_extension(_dut_io)
    soc = AgentLabKC705(platform,
        with_uartbone = not args.no_uartbone,
        with_jtagbone = args.with_jtagbone,
        with_dram     = not args.no_dram,
        with_eyes     = args.with_eyes)
    agentlab_build(parser, args, soc)

if __name__ == "__main__":
    main()
