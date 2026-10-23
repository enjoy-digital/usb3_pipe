#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys

from migen import *

from litex.gen import *

from litex_boards.platforms import xilinx_kc705

from litex.build.generic_platform import *
from litex.build.xilinx import VivadoProgrammer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy import LiteEthPHY

from litescope import LiteScopeAnalyzer

from litex.soc.interconnect import stream

from usb3_pipe             import K7USB3SerDes
from usb3_pipe.phy         import USB3PHY
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # PCIe / Through PCIsh-to-USB3 breakout board.
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

    # SFP / Through XillyUSB's SFP2USB.
    ("sfp_tx", 0,
        Subsignal("p", Pins("H2")),
        Subsignal("n", Pins("H1")),
    ),
    ("sfp_rx", 0,
        Subsignal("p", Pins("G4")),
        Subsignal("n", Pins("G3")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_oob    = ClockDomain()
        self.cd_clk125 = ClockDomain()

        # # #

        self.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_oob,    sys_clk_freq/8)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, connector="sfp", with_etherbone=True, with_analyzer=True):
        sys_clk_freq = int(125e6)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone()

        # Etherbone --------------------------------------------------------------------------------
        if with_etherbone:
            self.eth_phy = LiteEthPHY(
                clock_pads = platform.request("eth_clocks"),
                pads       = platform.request("eth"),
                clk_freq   = sys_clk_freq)
            self.add_etherbone(phy=self.eth_phy, ip_address="192.168.1.50")

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

        # USB3 PHY (PIPE + LTSSM) ------------------------------------------------------------------
        self.usb3_phy = usb3_phy = USB3PHY(
            serdes       = usb3_serdes,
            sys_clk_freq = sys_clk_freq,
            role         = "device")

        # USB3 Device Core (Bulk loopback: EP2 OUT -> FIFO -> EP1 IN) ------------------------------
        self.usb3_core = usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq,
            descriptors = bulk_descriptors(max_burst=8),
            endpoints   = [USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)])
        usb3_core.add_csr()
        self.usb3_loopback = usb3_loopback = stream.SyncFIFO([("data", 32), ("last_be", 4)], 2*256, buffered=True)
        self.comb += [
            usb3_core.endpoints[1].source.connect(usb3_loopback.sink),
            usb3_loopback.source.connect(usb3_core.endpoints[0].sink),
        ]

        # Debug IOs (Through CYUSB3ACC-005) --------------------------------------------------------
        _debug_ios = [
            ("tx_idle", 0, Pins("AK20"), IOStandard("LVCMOS12")), # I2C_SDA
            ("rx_idle", 0, Pins("AK21"), IOStandard("LVCMOS12")), # I2C_SCL
        ]
        self.platform.add_extension(_debug_ios)
        self.comb += [
            platform.request("tx_idle").eq(usb3_serdes.tx_idle),
            platform.request("rx_idle").eq(usb3_serdes.rx_idle),
        ]

        # Leds -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("user_led", 0).eq(usb3_serdes.ready),
            platform.request("user_led", 1).eq(usb3_phy.u0),
        ]

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                # LFPS.
                usb3_serdes.tx_idle,
                usb3_serdes.rx_idle,
                usb3_serdes.tx_pattern,
                usb3_serdes.rx_polarity,
                usb3_phy.pipe.lfps.rx_polling,
                usb3_phy.pipe.lfps.tx_polling,

                # Training Sequence.
                usb3_phy.pipe.ts.tx_enable,
                usb3_phy.pipe.ts.rx_ts1,
                usb3_phy.pipe.ts.rx_ts2,
                usb3_phy.pipe.ts.tx_tseq,
                usb3_phy.pipe.ts.tx_ts1,
                usb3_phy.pipe.ts.tx_ts2,
                usb3_phy.pipe.ts.tx_done,

                # LTSSM.
                usb3_phy.ltssm_state,
                usb3_phy.u0,

                # Endpoints.
                usb3_serdes.rx_datapath.skip_remover.skip,
                usb3_serdes.source,
                usb3_serdes.sink,
                usb3_phy.source,
                usb3_phy.sink,
            ]
            self.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, csr_csv="analyzer.csv")

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    with open("README.md") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[1:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--build",         action="store_true", help="Build bitstream.")
    parser.add_argument("--load",          action="store_true", help="Load bitstream.")
    parser.add_argument("--with-analyzer", action="store_true", help="Enable LiteScope Analyzer.")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    platform = xilinx_kc705.Platform()
    platform.add_extension(_usb3_io)
    soc     = USB3SoC(platform, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
