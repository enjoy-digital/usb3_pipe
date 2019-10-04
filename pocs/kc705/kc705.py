#!/usr/bin/env python3

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import PulseSynchronizer, MultiReg

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

from usb3_pipe.common import TSEQ, TS1, TS2
from usb3_pipe.gtx_7series import GTXChannelPLL, GTX
from usb3_pipe.lfps import LFPSReceiver, LFPSTransmitter
from usb3_pipe.ordered_set import OrderedSetReceiver, OrderedSetTransmitter

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
    def __init__(self, platform,
        with_etherbone=True, mac_address=0x10e2d5000000, ip_address="192.168.1.50",
        with_lfps_analyzer=False,
        with_rx_analyzer=True,
        with_tx_analyzer=True,
        with_fsm_analyzer=True):

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
        gtx._tx_enable.storage.reset   = 1 # Enabled by default
        gtx._rx_enable.storage.reset   = 1 # Enabled by default
        gtx._tx_polarity.storage.reset = 1
        gtx._rx_polarity.storage.reset = 1
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
        lfps_transmitter = LFPSTransmitter(sys_clk_freq=sys_clk_freq, lfps_clk_freq=25e6)
        self.submodules += lfps_transmitter
        self.comb += [
            If(lfps_transmitter.polling,
                txelecidle.eq(lfps_transmitter.tx_idle),
                gtx.tx_produce_pattern.eq(~lfps_transmitter.tx_idle),
                gtx.tx_pattern.eq(lfps_transmitter.tx_pattern)
            ).Else(
                txelecidle.eq(0),
                gtx.tx_produce_pattern.eq(0)

            )
        ]

        # TSEQ Receiver ----------------------------------------------------------------------------
        tseq_receiver = OrderedSetReceiver(ordered_set=TSEQ, n_ordered_sets=1024, data_width=32)
        tseq_receiver = ClockDomainsRenamer("rx")(tseq_receiver)
        self.submodules += tseq_receiver

        # TS1 Receiver -----------------------------------------------------------------------------
        ts1_receiver = OrderedSetReceiver(ordered_set=TS1, n_ordered_sets=16, data_width=32)
        ts1_receiver = ClockDomainsRenamer("rx")(ts1_receiver)
        self.submodules += ts1_receiver

        # TS2 Receiver -----------------------------------------------------------------------------
        ts2_receiver = OrderedSetReceiver(ordered_set=TS2, n_ordered_sets=1024, data_width=32)
        ts2_receiver = ClockDomainsRenamer("rx")(ts2_receiver)
        self.submodules += ts2_receiver

        # TS2 Transmitter --------------------------------------------------------------------------
        ts2_transmitter = OrderedSetTransmitter(ordered_set=TS2, n_ordered_sets=1024, data_width=32)
        ts2_transmitter = ClockDomainsRenamer("tx")(ts2_transmitter)
        self.submodules += ts2_transmitter

        # Hacky Startup FSM (just to experiment on hardware) ---------------------------------------
        tseq_det_sync = PulseSynchronizer("rx", "sys")
        ts1_det_sync  = PulseSynchronizer("rx", "sys")
        ts2_det_sync  = PulseSynchronizer("rx", "sys")
        ts2_send_sync = PulseSynchronizer("sys", "tx")
        ts2_done      = Signal()
        self.submodules += tseq_det_sync, ts1_det_sync, ts2_det_sync, ts2_send_sync
        self.comb += [
            tseq_det_sync.i.eq(tseq_receiver.detected),
            ts1_det_sync.i.eq(ts1_receiver.detected),
            ts2_det_sync.i.eq(ts2_receiver.detected),
            ts2_transmitter.send.eq(ts2_send_sync.o),
        ]
        self.specials += MultiReg(ts2_transmitter.done, ts2_done)

        fsm = FSM(reset_state="POLLING-LFPS")
        fsm = ResetInserter()(fsm)
        self.submodules += fsm
        self.comb += fsm.reset.eq(lfps_receiver.polling)
        fsm.act("POLLING-LFPS",
            gtx.rx_align.eq(1),
            lfps_transmitter.polling.eq(1),
            NextState("WAIT-TSEQ"),
        )
        fsm.act("WAIT-TSEQ",
            gtx.rx_align.eq(1),
            lfps_transmitter.polling.eq(1),
            gtx.source.connect(tseq_receiver.sink),
            If(tseq_det_sync.o,
                NextState("SEND-POLLING-LFPS-WAIT-TS1")
            )
        )
        fsm.act("SEND-POLLING-LFPS-WAIT-TS1",
            gtx.rx_align.eq(0),
            gtx.source.connect(ts1_receiver.sink),
            If(ts1_det_sync.o,
                ts2_send_sync.i.eq(1),
                NextState("SEND-TS2-WAIT-TS2")
            )
        )
        fsm.act("SEND-TS2-WAIT-TS2",
            gtx.rx_align.eq(0),
            ts2_send_sync.i.eq(ts2_done),
            gtx.source.connect(ts2_receiver.sink),
            ts2_transmitter.source.connect(gtx.sink),
            If(ts2_det_sync.o,
                NextState("READY")
            )
        )
        fsm.act("READY",
            gtx.rx_align.eq(0)
        )

        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(gtx.tx_ready)
        self.comb += platform.request("user_led", 1).eq(gtx.rx_ready)
        self.comb += platform.request("user_led", 7).eq(rxelecidle)
        polling_timer = WaitTimer(int(sys_clk_freq*1e-1))
        self.submodules += polling_timer
        self.comb += [
            polling_timer.wait.eq(~lfps_receiver.polling),
            platform.request("user_led", 2).eq(~polling_timer.done)
        ]

        # LFPS Analyzer ----------------------------------------------------------------------------
        if with_lfps_analyzer:
            analyzer_signals = [
                rxelecidle,
                txelecidle,

                lfps_receiver.polling,
                lfps_receiver.count,
                lfps_receiver.found,
                lfps_receiver.fsm,
            ]
            self.submodules.lfps_analyzer = LiteScopeAnalyzer(analyzer_signals, 32768, clock_domain="sys", csr_csv="lfps_analyzer.csv")
            self.add_csr("lfps_analyzer")

        # RX Analyzer ------------------------------------------------------------------------------
        if with_rx_analyzer:
            analyzer_signals = [
                fsm,
                gtx.source,
                tseq_receiver.detected,
                ts1_receiver.detected,
                ts1_receiver.reset,
                ts1_receiver.loopback,
                ts1_receiver.scrambling,
                ts2_receiver.detected,
                ts2_receiver.reset,
                ts2_receiver.loopback,
                ts2_receiver.scrambling
            ]
            self.submodules.rx_analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="rx", csr_csv="rx_analyzer.csv")
            self.add_csr("rx_analyzer")

        # TX Analyzer ------------------------------------------------------------------------------
        if with_tx_analyzer:
            analyzer_signals = [
                fsm,
                gtx.sink,
                ts2_transmitter.send,
                ts2_transmitter.done,
            ]
            self.submodules.tx_analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="tx", csr_csv="tx_analyzer.csv")
            self.add_csr("tx_analyzer")

        # FSM Analyzer -----------------------------------------------------------------------------
        if with_fsm_analyzer:
            analyzer_signals = [
                fsm,
                tseq_det_sync.o,
                ts1_det_sync.o,
                ts2_det_sync.o,
                ts2_send_sync.i,
                ts2_done
            ]
            self.submodules.fsm_analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="sys", csr_csv="fsm_analyzer.csv")
            self.add_csr("fsm_analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = kc705.Platform()
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
