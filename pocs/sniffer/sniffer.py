#!/usr/bin/env python3

# USB3 Sniffer Proof of Concept:
# PCIe Screamer with FT601 TX lanes duplicated to PCIe RX lanes for sniffing FT601 --> Host comm

from migen import *
from migen.genlib.misc import WaitTimer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect import stream
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from usb3_pipe.gtp_7series import GTPQuadPLL, GTP
from usb3_pipe.lfps import LFPSReceiver, LFPSTransmitter

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("clk100", 0, Pins("R4"), IOStandard("LVCMOS33")),

    ("user_led", 0, Pins("AB1"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("AB8"), IOStandard("LVCMOS33")),

    ("user_btn", 0, Pins("AA1"), IOStandard("LVCMOS33")),
    ("user_btn", 1, Pins("AB6"), IOStandard("LVCMOS33")),

    ("serial", 0,
        Subsignal("tx", Pins("T1")),
        Subsignal("rx", Pins("U1")),
        IOStandard("LVCMOS33"),
    ),

    ("pcie_tx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("A6")),
    ),

    ("pcie_rx", 0,
        Subsignal("p", Pins("B10")),
        Subsignal("n", Pins("A10")),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35t-fgg484-2", _io, toolchain="vivado")

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        clk100 = platform.request("clk100")
        platform.add_period_constraint(clk100, 1e9/100e6)

        self.cd_sys.clk.attr.add("keep")
        self.cd_clk125.clk.attr.add("keep")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3Sniffer --------------------------------------------------------------------------------------

class USB3Sniffer(SoCMini):
    def __init__(self, platform):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3Sniffer", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial bridge ----------------------------------------------------------------------------
        self.submodules.serial_bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.serial_bridge.wishbone)

        # Capture ----------------------------------------------------------------------------------
        # qpll
        qpll = GTPQuadPLL(ClockSignal("clk125"), 125e6, 5e9)
        print(qpll)
        self.submodules += qpll
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # gtp
        tx_pads = platform.request("pcie_tx")
        rx_pads = platform.request("pcie_rx")
        self.submodules.gtp = gtp = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True)
        gtp.add_stream_endpoints()
        gtp.add_controls()
        gtp._tx_enable.storage.reset = 1 # Enabled by default
        gtp._rx_enable.storage.reset = 1 # Enabled by default
        self.add_csr("gtp")
        gtp.cd_tx.clk.attr.add("keep")
        gtp.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
        platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.rx_clk_freq)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtp.cd_tx.clk)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtp.cd_rx.clk)

        # converter
        converter = stream.StrideConverter(
            [("data", 16), ("ctrl", 2)],
            [("data", 64), ("ctrl", 8)])
        converter = ClockDomainsRenamer("rx")(converter)
        self.submodules += converter

        # cdc
        cdc = stream.AsyncFIFO([("data", 64), ("ctrl", 8)], 8)
        cdc = ClockDomainsRenamer({"write": "rx", "read": "sys"})(cdc)
        self.submodules += cdc

        # flow
        self.comb += [
            gtp.source.connect(converter.sink),
            converter.source.connect(cdc.sink),
            cdc.source.ready.eq(1)
        ]

        # Analyzer ---------------------------------------------------------------------------------
        analyzer_signals = [
            gtp.rx_init.done,
            cdc.source,
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 1024, clock_domain="sys",
            csr_csv="analyzer.csv")
        self.add_csr("analyzer")

        # Led (Set Led0 if continuous K28.5 are detected) ------------------------------------------
        def K(x, y):
            return (y << 5) | x
        k28_5_timer = WaitTimer(int(250e6*1e-1))
        k28_5_timer = ClockDomainsRenamer("rx")(k28_5_timer)
        self.submodules += k28_5_timer
        self.comb += [
            k28_5_timer.wait.eq(1),
            If(gtp.source.ctrl[0] & (gtp.source.data[:8] == K(28, 5)),
                k28_5_timer.wait.eq(0)
            ),
            If(gtp.source.ctrl[1] & (gtp.source.data[8:] == K(28, 5)),
                k28_5_timer.wait.eq(0)
            ),
            platform.request("user_led", 0).eq(~k28_5_timer.done)
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    platform = Platform()
    soc = USB3Sniffer(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
