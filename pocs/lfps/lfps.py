#!/usr/bin/env python3

# USB3 TLPS Obserser Proof of Concept
# PCIe Screamer with PCIe Riser connected to a Host

from migen import *

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect import stream
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from gtp_7series import GTPQuadPLL, GTP

from usb3_pipe.lfps import LFPSReceiver, LFPSTransmitter

from litescope import LiteScopeAnalyzer

# PCIe Screamer IOs ----------------------------------------------------------------------------------------------

_io_pcie_screamer = [
    ("clk100", 0, Pins("R4"), IOStandard("LVCMOS33")),

    ("user_led", 0, Pins("AB1"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("AB8"), IOStandard("LVCMOS33")),

    ("user_btn", 0, Pins("AA1"), IOStandard("LVCMOS33")),
    ("user_btn", 1, Pins("AB6"), IOStandard("LVCMOS33")),

    ("user_gpio", 0, Pins("Y6"), IOStandard("LVCMOS33")),
    ("user_gpio", 1, Pins("AA6"), IOStandard("LVCMOS33")),

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

# PCIe Screamer Platform -----------------------------------------------------------------------------------------

class PCIeScreamerPlatform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35t-fgg484-2", _io_pcie_screamer, toolchain="vivado")

# USBSniffer IOs ----------------------------------------------------------------------------------------------

_io_usb_sniffer = [
    ("clk100", 0, Pins("J19"), IOStandard("LVCMOS33")),

    ("user_gpio", 0, Pins("U21"), IOStandard("LVCMOS33")),
    ("user_gpio", 1, Pins("T21"), IOStandard("LVCMOS33")),

    ("serial", 0,
        Subsignal("tx", Pins("J16")),
        Subsignal("rx", Pins("H13")),
        IOStandard("LVCMOS33"),
    ),

    ("switch", 0, Pins("AA3"), IOStandard("LVCMOS33")),

    ("pcie_tx", 0,
        Subsignal("p", Pins("B6")),
        Subsignal("n", Pins("A6")),
    ),

    ("pcie_rx", 0,
        Subsignal("p", Pins("B10")),
        Subsignal("n", Pins("A10")),
    ),
]

# USBSniffer Platform -----------------------------------------------------------------------------------------

class USBSnifferPlatform(XilinxPlatform):
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a50t-fgg484-2", _io_usb_sniffer, toolchain="vivado")

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

# USB3LFPS -----------------------------------------------------------------------------------------

class USB3LFPS(SoCMini):
    def __init__(self, platform):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3LFPS", ident_version=True)

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

        if isinstance(platform, PCIeScreamerPlatform):
            self.comb += platform.request("switch").eq(0)

        # Override GTP parameters/signals for LFPS -------------------------------------------------
        txelecidle = Signal()
        rxelecidle = Signal()
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100, # bit 8 enable OOB
            p_RXOOB_CLK_CFG  = "PMA",
            p_RXOOB_CFG      = 0b0000110,
            i_RXELECIDLEMODE = 0b00,
            o_RXELECIDLE     = rxelecidle,
            i_TXELECIDLE     = txelecidle)

        # Redirect Elec Idle signals to GPIOs --s----------------------------------------------------
        self.comb += platform.request("user_gpio", 0).eq(rxelecidle)
        self.comb += platform.request("user_gpio", 1).eq(txelecidle)

        # LFPS Polling Receive ---------------------------------------------------------------------
        lfps_receiver = LFPSReceiver(sys_clk_freq=sys_clk_freq)
        self.submodules += lfps_receiver
        self.comb += lfps_receiver.idle.eq(rxelecidle)

        # LFPS Polling Transmit --------------------------------------------------------------------
        lfps_transmitter = LFPSTransmitter(sys_clk_freq=sys_clk_freq, lfps_clk_freq=25e6)
        self.submodules += lfps_transmitter
        self.comb += [
            txelecidle.eq(lfps_transmitter.idle),
            gtp.tx_produce_pattern.eq(~lfps_transmitter.idle),
            gtp.tx_pattern.eq(lfps_transmitter.pattern)
        ]

        # Analyzer ---------------------------------------------------------------------------------
        analyzer_signals = [
            rxelecidle,
            txelecidle,

            lfps_receiver.polling,
            lfps_receiver.count,
            lfps_receiver.found,
            lfps_receiver.fsm,

            lfps_transmitter.idle,
            lfps_transmitter.pattern
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="sys",
            csr_csv="analyzer.csv")
        self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = PCIeScreamerPlatform()
    #platform = USBSnifferPlatform()
    soc = USB3LFPS(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
