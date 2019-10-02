#!/usr/bin/env python3

from migen import *

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.common import convert_ip
from liteeth.phy import LiteEthPHY
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litex.boards.platforms import kc705

from litescope import LiteScopeAnalyzer

from usb3_pipe.gtx_7series import GTXChannelPLL, GTX
from usb3_pipe.lfps import LFPSReceiver, LFPSTransmitter

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
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_oob = ClockDomain()

        # # #

        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk156"), 156.5e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform, with_etherbone=False, mac_address=0x10e2d5000000, ip_address="192.168.1.50"):

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

        # Transceiver ------------------------------------------------------------------------------
        # refclk
        refclk = Signal()
        refclk_pads = platform.request("sgmii_clock") # Use SGMII clock (FMC does not provide one)
        self.specials += [
            Instance("IBUFDS_GTE2",
                i_CEB=0,
                i_I=refclk_pads.p,
                i_IB=refclk_pads.n,
                o_O=refclk)
        ]

        # pll
        pll_cls = GTXChannelPLL
        pll = pll_cls(refclk, 125e6, 5e9)
        print(pll)
        self.submodules += pll

        # gtx
        tx_pads = platform.request("usb3_tx")
        rx_pads = platform.request("usb3_rx")
        self.submodules.gtx = gtx = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width=40,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True)
        gtx.add_stream_endpoints()
        gtx.add_controls()
        self.add_csr("gtx")
        gtx._tx_enable.storage.reset = 1 # Enabled by default
        gtx._rx_enable.storage.reset = 1 # Enabled by default
        self.submodules += gtx

        # timing constraints
        gtx.cd_tx.clk.attr.add("keep")
        gtx.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(gtx.cd_tx.clk, 1e9/gtx.tx_clk_freq)
        platform.add_period_constraint(gtx.cd_rx.clk, 1e9/gtx.rx_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            gtx.cd_tx.clk,
            gtx.cd_rx.clk)

        # Override GTX parameters/signals for LFPS -------------------------------------------------
        txelecidle = Signal()
        rxelecidle = Signal()
        gtx.gtx_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100, # bit 8 enable OOB
            p_RXOOB_CFG      = 0b0000110,
            i_RXOOBRESET     = 0,
            i_CLKRSVD        = ClockSignal("oob"),
            i_RXELECIDLEMODE = 0b00,
            o_RXELECIDLE     = rxelecidle,
            i_TXELECIDLE     = txelecidle)

        # LFPS Polling Receive ---------------------------------------------------------------------
        lfps_receiver = LFPSReceiver(sys_clk_freq=sys_clk_freq)
        self.submodules += lfps_receiver
        self.comb += lfps_receiver.idle.eq(rxelecidle)

        # LFPS Polling Transmit --------------------------------------------------------------------
        if True:
            lfps_transmitter = LFPSTransmitter(sys_clk_freq=sys_clk_freq, lfps_clk_freq=25e6)
            self.submodules += lfps_transmitter
            self.comb += [
                txelecidle.eq(lfps_transmitter.idle),
                gtx.tx_produce_pattern.eq(~lfps_transmitter.idle),
                gtx.tx_pattern.eq(lfps_transmitter.pattern)
            ]

        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(gtx.tx_ready)
        self.comb += platform.request("user_led", 1).eq(gtx.rx_ready)
        self.comb += platform.request("user_led", 7).eq(rxelecidle)

        # Analyzer ---------------------------------------------------------------------------------
        analyzer_signals = [
            rxelecidle,
            txelecidle,

            lfps_receiver.polling,
            lfps_receiver.count,
            lfps_receiver.found,
            lfps_receiver.fsm,
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 32768, csr_csv="analyzer.csv")
        self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = kc705.Platform()
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
