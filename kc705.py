#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *

from litex_boards.platforms import kc705

from litex.build.generic_platform import *
from litex.build.xilinx import VivadoProgrammer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy import LiteEthPHY
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litescope import LiteScopeAnalyzer

from usb3_pipe import K7USB3SerDes, USB3PIPE
from usb3_core.core import USB3Core

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # HiTechGlobal USB3.0 FMC P3 connector
    ("usb3_rx", 0,
        Subsignal("p", Pins("HPC:DP0_M2C_P")),
        Subsignal("n", Pins("HPC:DP0_M2C_N")),
    ),
    ("usb3_tx", 0,
        Subsignal("p", Pins("HPC:DP0_C2M_P")),
        Subsignal("n", Pins("HPC:DP0_C2M_N")),
    ),

    # PCIe
    ("pcie_rx", 0,
        Subsignal("p", Pins("M6")),
        Subsignal("n", Pins("M5")),
    ),
    ("pcie_tx", 0,
        Subsignal("p", Pins("L4")),
        Subsignal("n", Pins("L3")),
    ),

    # SMA
    ("sma_tx", 0,
        Subsignal("p", Pins("K2")),
        Subsignal("n", Pins("K1"))
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("K6")),
        Subsignal("n", Pins("K5"))
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
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_oob,    sys_clk_freq/8)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="pcie", with_etherbone=True, with_analyzer=True):
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # Ethernet <--> Wishbone -------------------------------------------------------------------
        if with_etherbone:
            # phy
            self.submodules.eth_phy = LiteEthPHY(
                clock_pads = platform.request("eth_clocks"),
                pads       = platform.request("eth"),
                clk_freq   = sys_clk_freq)
            self.add_csr("eth_phy")
            # core
            self.submodules.eth_core = LiteEthUDPIPCore(
                phy         = self.eth_phy,
                mac_address = 0x10e2d5000000,
                ip_address  = "192.168.1.50",
                clk_freq    = sys_clk_freq)
            # etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.eth_core.udp, 1234)
            self.add_wb_master(self.etherbone.wishbone.bus)

            # timing constraints
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_rx.clk, 1e9/125e6)
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_tx.clk, 1e9/125e6)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.eth_phy.crg.cd_eth_rx.clk,
                self.eth_phy.crg.cd_eth_tx.clk)

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = K7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk125"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"))
        self.submodules += usb3_serdes
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules.usb3_pipe = usb3_pipe
        self.comb += usb3_pipe.reset.eq(platform.request("cpu_reset"))

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

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    with open("README.md") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[1:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    os.makedirs("build/kc705/gateware", exist_ok=True)
    os.system("cd usb3_core/daisho && make && ./usb_descrip_gen")
    os.system("cp usb3_core/daisho/usb3/*.init build/kc705/gateware/")
    platform = kc705.Platform()
    platform.add_extension(_usb3_io)
    soc     = USB3SoC(platform)
    builder = Builder(soc, csr_csv="tools/csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
