# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

#!/usr/bin/env python3

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.uart import UARTWishboneBridge

from litex.boards.platforms import nexys_video

from litescope import LiteScopeAnalyzer

from usb3_pipe.common import TSEQ, TS1, TS2
from usb3_pipe.serdes import A7USB3SerDes
from usb3_pipe.scrambler import Scrambler
from usb3_pipe.lfps import LFPSChecker, LFPSGenerator
from usb3_pipe.ordered_set import OrderedSetChecker, OrderedSetGenerator

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
        self.clock_domains.cd_oob = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        clk100 = platform.request("clk100")
        platform.add_period_constraint(clk100, 1e9/100e6)

        self.cd_sys.clk.attr.add("keep")
        self.cd_clk125.clk.attr.add("keep")

        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform,
        with_lfps_analyzer=False,
        with_rx_analyzer=True,
        with_tx_analyzer=True,
        with_fsm_analyzer=True):

        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial bridge ----------------------------------------------------------------------------
        self.submodules.serial_bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq)
        self.add_wb_master(self.serial_bridge.wishbone)

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk125"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request("pcie_tx"),
            rx_pads      = platform.request("pcie_rx"))
        self.submodules += usb3_serdes
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # LFPS Polling Receive ---------------------------------------------------------------------
        lfps_receiver = LFPSChecker(sys_clk_freq=sys_clk_freq)
        self.submodules += lfps_receiver
        self.comb += lfps_receiver.idle.eq(usb3_serdes.rx_idle)

        # LFPS Polling Transmit --------------------------------------------------------------------
        lfps_transmitter = LFPSGenerator(sys_clk_freq=sys_clk_freq, lfps_clk_freq=25e6)
        self.submodules += lfps_transmitter
        self.comb += [
            If(lfps_transmitter.polling,
                usb3_serdes.tx_idle.eq(lfps_transmitter.tx_idle),
                usb3_serdes.tx_pattern.eq(lfps_transmitter.tx_pattern)
            ).Else(
                 usb3_serdes.tx_idle.eq(0)
            )
        ]

        # TSEQ Receiver ----------------------------------------------------------------------------
        tseq_receiver = OrderedSetChecker(ordered_set=TSEQ, n_ordered_sets=1024, data_width=32)
        tseq_receiver = ClockDomainsRenamer("rx")(tseq_receiver)
        self.submodules += tseq_receiver

        # TS1 Receiver -----------------------------------------------------------------------------
        ts1_receiver = OrderedSetChecker(ordered_set=TS1, n_ordered_sets=16, data_width=32)
        ts1_receiver = ClockDomainsRenamer("rx")(ts1_receiver)
        self.submodules += ts1_receiver

        # TS2 Receiver -----------------------------------------------------------------------------
        ts2_receiver = OrderedSetChecker(ordered_set=TS2, n_ordered_sets=1024, data_width=32)
        ts2_receiver = ClockDomainsRenamer("rx")(ts2_receiver)
        self.submodules += ts2_receiver

        # TS2 Transmitter --------------------------------------------------------------------------
        ts2_transmitter = OrderedSetGenerator(ordered_set=TS2, n_ordered_sets=1024, data_width=32)
        ts2_transmitter = ClockDomainsRenamer("tx")(ts2_transmitter)
        self.submodules += ts2_transmitter

        # Scrambler --------------------------------------------------------------------------------
        scrambler = Scrambler()
        scrambler = ClockDomainsRenamer("tx")(scrambler)
        self.submodules += scrambler

        # Hacky Startup FSM (just to experiment on hardware) ---------------------------------------
        tseq_det_sync = PulseSynchronizer("rx", "tx")
        ts1_det_sync  = PulseSynchronizer("rx", "tx")
        ts2_det_sync  = PulseSynchronizer("rx", "tx")
        self.submodules += tseq_det_sync, ts1_det_sync, ts2_det_sync
        self.comb += [
            tseq_det_sync.i.eq(tseq_receiver.detected),
            ts1_det_sync.i.eq(ts1_receiver.detected),
            ts2_det_sync.i.eq(ts2_receiver.detected),
        ]

        fsm = FSM(reset_state="POLLING-LFPS")
        fsm = ClockDomainsRenamer("tx")(fsm)
        fsm = ResetInserter()(fsm)
        self.submodules += fsm
        self.comb += fsm.reset.eq(lfps_receiver.polling)
        fsm.act("POLLING-LFPS",
            scrambler.reset.eq(1),
            usb3_serdes.rx_align.eq(1),
            lfps_transmitter.polling.eq(1),
            NextValue(ts2_transmitter.send, 0),
            NextState("WAIT-TSEQ"),
        )
        fsm.act("WAIT-TSEQ",
            usb3_serdes.rx_align.eq(1),
            lfps_transmitter.polling.eq(1),
            usb3_serdes.source.connect(tseq_receiver.sink),
            If(tseq_det_sync.o,
                NextState("SEND-POLLING-LFPS-WAIT-TS1")
            )
        )
        fsm.act("SEND-POLLING-LFPS-WAIT-TS1",
            usb3_serdes.rx_align.eq(0),
            usb3_serdes.source.connect(ts1_receiver.sink),
            If(ts1_det_sync.o,
                NextValue(ts2_transmitter.send, 1),
                NextState("SEND-TS2-WAIT-TS2")
            )
        )
        ts2_det = Signal()
        fsm.act("SEND-TS2-WAIT-TS2",
            usb3_serdes.rx_align.eq(0),
            usb3_serdes.source.connect(ts2_receiver.sink),
            ts2_transmitter.source.connect(usb3_serdes.sink),
            NextValue(ts2_det, ts2_det | ts2_det_sync.o),
            NextValue(ts2_transmitter.send, 0),
            If(ts2_transmitter.done,
                If(ts2_det,
                    NextState("READY")
                ).Else(
                    NextValue(ts2_transmitter.send, 1)
                )
            )
        )
        fsm.act("READY",
            usb3_serdes.rx_align.eq(0),
            scrambler.sink.valid.eq(1),
            scrambler.source.connect(usb3_serdes.sink),
        )

        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(usb3_serdes.gtp.tx_ready & usb3_serdes.gtp.rx_ready)
        self.comb += platform.request("user_led", 1).eq(usb3_serdes.rx_idle)

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
                usb3_serdes.source,
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
                usb3_serdes.sink,
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
            ]
            self.submodules.fsm_analyzer = LiteScopeAnalyzer(analyzer_signals, 4096, clock_domain="sys", csr_csv="fsm_analyzer.csv")
            self.add_csr("fsm_analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    platform = Platform()
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
