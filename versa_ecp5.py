#!/usr/bin/env python3

# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *

from litex.boards.platforms import versa_ecp5

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone


from litex.soc.cores.uart import UARTWishboneBridge

from litescope import LiteScopeAnalyzer

from usb3_pipe import ECP5USB3SerDes, USB3PIPE

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # SMA
    ("sma_tx", 0,
        Subsignal("p", Pins("W8")),
        Subsignal("n", Pins("W9")),
    ),
    ("sma_rx", 0,
        Subsignal("p", Pins("Y7")),
        Subsignal("n", Pins("Y8")),
    ),

    # PCIe
    ("pcie_rx", 0,
        Subsignal("p", Pins("Y5")),
        Subsignal("n", Pins("Y6")),
    ),
    ("pcie_tx", 0,
        Subsignal("p", Pins("W4")),
        Subsignal("n", Pins("W5")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        self.cd_sys.clk.attr.add("keep")

        # clk / rst
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # pll
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)


# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="pcie",
            with_etherbone=True, mac_address=0x10e2d5000000, ip_address="192.168.1.50",
            with_analyzer=True):

        sys_clk_freq = int(133e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.bridge.wishbone)

        # Ethernet <--> Wishbone -------------------------------------------------------------------
        if with_etherbone:
            # phy
            self.submodules.eth_phy = LiteEthPHYRGMII(
                clock_pads = platform.request("eth_clocks"),
                pads       = platform.request("eth"))
            self.add_csr("eth_phy")
            # core
            self.submodules.eth_core = LiteEthUDPIPCore(
                phy         = self.eth_phy,
                mac_address = mac_address,
                ip_address  = ip_address,
                clk_freq    = sys_clk_freq)
            # etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.eth_core.udp, 1234)
            self.add_wb_master(self.etherbone.wishbone.bus)

            # timing constraints
            self.eth_phy.crg.cd_eth_rx.clk.attr.add("keep")
            self.eth_phy.crg.cd_eth_tx.clk.attr.add("keep")
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_rx.clk, 1e9/125e6)
            self.platform.add_period_constraint(self.eth_phy.crg.cd_eth_tx.clk, 1e9/125e6)

        # USB3 SerDes ------------------------------------------------------------------------------
        self.comb += platform.request("refclk_en").eq(1)
        self.comb += platform.request("refclk_rst_n").eq(1)
        usb3_serdes = ECP5USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = platform.request("refclk", 1),
            refclk_freq  = 156.25e6,
            tx_pads      = platform.request(connector + "_tx"),
            rx_pads      = platform.request(connector + "_rx"),
            channel      = 0)
        self.submodules += usb3_serdes

        # USB3 PIPE --------------------------------------------------------------------------------
        usb3_pipe = USB3PIPE(serdes=usb3_serdes, sys_clk_freq=sys_clk_freq, with_scrambling=False)
        self.submodules += usb3_pipe
        self.comb += usb3_pipe.sink.valid.eq(1)
        self.comb += usb3_pipe.source.ready.eq(1)

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
                usb3_serdes.rx_skip,
                usb3_serdes.source,
                usb3_serdes.sink,
                usb3_pipe.source,
                usb3_pipe.sink,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, csr_csv="tools/analyzer.csv")
            self.add_csr("analyzer")


        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(usb3_serdes.ready)
        self.comb += platform.request("user_led", 1).eq(usb3_pipe.ready)

        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 4).eq(sys_counter[26])

        rx_counter = Signal(32)
        self.sync.rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 5).eq(rx_counter[26])

        tx_counter = Signal(32)
        self.sync.tx += tx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 6).eq(tx_counter[26])

# Build --------------------------------------------------------------------------------------------

def main():
    platform = versa_ecp5.Platform(toolchain="trellis")
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="tools/csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
