# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

#!/usr/bin/env python3

from migen import *

from litex.boards.platforms import kc705

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.common import convert_ip
from liteeth.phy import LiteEthPHY
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litescope import LiteScopeAnalyzer

from usb3_pipe import K7USB3SerDes, USB3PIPE

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
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb3_oob = ClockDomain()

        # # #

        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk156"), 156.5e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_usb3_oob, sys_clk_freq/8)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="pcie",
        with_etherbone=True, mac_address=0x10e2d5000000, ip_address="192.168.1.50",
        with_analyzer=True):

        sys_clk_freq = int(156.5e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

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
                mac_address = mac_address,
                ip_address  = convert_ip(ip_address),
                clk_freq    = sys_clk_freq)
            # etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.eth_core.udp, 1234)
            self.add_wb_master(self.etherbone.wishbone.bus)

            # timing constraints
            self.crg.cd_sys.clk.attr.add("keep")
            self.eth_phy.crg.cd_eth_rx.clk.attr.add("keep")
            self.eth_phy.crg.cd_eth_tx.clk.attr.add("keep")
            self.platform.add_period_constraint(self.crg.cd_sys.clk, 1e9/156.5e6)
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
            refclk_pads  = platform.request("sgmii_clock"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"))
        self.submodules += usb3_serdes

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules += usb3_pipe
        self.comb += usb3_pipe.sink.valid.eq(1)
        self.comb += usb3_pipe.source.ready.eq(1)

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
                usb3_pipe.ts.rx_tseq,
                usb3_pipe.ts.rx_ts1,
                usb3_pipe.ts.rx_ts2,
                usb3_pipe.ts.tx_enable,
                usb3_pipe.ts.tx_tseq,
                usb3_pipe.ts.tx_ts1,
                usb3_pipe.ts.tx_ts2,
                usb3_pipe.ts.tx_done,

                # LTSSM
                usb3_pipe.ltssm.polling_fsm,
                usb3_pipe.ready,

                # Endpoints
                usb3_serdes.source,
                usb3_serdes.sink,
                usb3_pipe.source,
                usb3_pipe.sink,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, csr_csv="tools/analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = kc705.Platform()
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="tools/csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
