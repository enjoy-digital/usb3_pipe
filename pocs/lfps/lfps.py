#!/usr/bin/env python3

# USB3 TLPS Obserser Proof of Concept
# PCIe Screamer with PCIe Riser connected to a Host

from migen import *
from migen.genlib.misc import WaitTimer

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.interconnect import stream
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from gtp_7series import GTPQuadPLL, GTP

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------

_io = [
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

        # Redirect RXELECIDLE to GPIO (for scope observation) and Analyzer -------------------------
        rxelecidle = Signal()
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100, # bit 8 enable OOB detection
            p_RXOOB_CLK_CFG  = "PMA",
            p_RXOOB_CFG      = 0b0000110,
            i_RXELECIDLEMODE = 0b00,
            o_RXELECIDLE     = rxelecidle)
        self.comb += platform.request("user_gpio", 0).eq(rxelecidle)

        # LFPS Polling generation ------------------------------------------------------------------
        # 5Gbps linerate / 4ns per words
        # 25MHz burst can be generated with 5 all ones / 5 all zeroes cycles.
        txelecidle = Signal()
        lfps_polling_pattern = Signal(20)
        lfps_polling_count   = Signal(4)
        self.sync.tx += [
            lfps_polling_count.eq(lfps_polling_count + 1),
            If(lfps_polling_count == 4,
                lfps_polling_count.eq(0),
                lfps_polling_pattern.eq(~lfps_polling_pattern),
            )
        ]
        lfps_burst_timer  = WaitTimer(int(1e-6*sys_clk_freq))
        lfps_repeat_timer = WaitTimer(int(10e-6*sys_clk_freq))
        self.submodules += lfps_burst_timer, lfps_repeat_timer
        self.comb += [
            lfps_burst_timer.wait.eq(~lfps_repeat_timer.done),
            lfps_repeat_timer.wait.eq(~lfps_repeat_timer.done),
        ]

        self.comb += gtp.tx_produce_pattern.eq(1)
        self.comb += gtp.tx_pattern.eq(lfps_polling_pattern)
        self.comb += txelecidle.eq(lfps_burst_timer.done)
        gtp.gtp_params.update(i_TXELECIDLE=txelecidle) # FIXME: check TX OOB settings
        self.comb += platform.request("user_gpio", 1).eq(txelecidle)

        # Analyzer ---------------------------------------------------------------------------------
        analyzer_signals = [rxelecidle]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="sys",
            csr_csv="analyzer.csv")
        self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = Platform()
    soc = USB3Sniffer(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
