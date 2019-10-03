#!/usr/bin/env python3

from migen import *
from migen.genlib.misc import WaitTimer

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.boards.platforms import nexys_video

from usb3_pipe.gtp_7series import GTPQuadPLL, GTP
from usb3_pipe.lfps import LFPSReceiver, LFPSTransmitter

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io = [
    # HiTechGlobal USB3.0 FMC P3 connector
    ("usb3_rx", 0,
        Subsignal("p", Pins("LPC:DP0_M2C_P")),
        Subsignal("n", Pins("LPC:DP0_M2C_N")),
    ),
    ("usb3_tx", 0,
        Subsignal("p", Pins("LPC:DP0_C2M_P")),
        Subsignal("n", Pins("LPC:DP0_C2M_N")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_clk125 = ClockDomain()

        # # #

        clk100 = platform.request("clk100")

        self.cd_sys.clk.attr.add("keep")
        self.cd_clk125.clk.attr.add("keep")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_clk125, 125e6)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCMini):
    def __init__(self, platform):

        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="USB3SoC", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Transceiver ------------------------------------------------------------------------------
        # qpll
        qpll = GTPQuadPLL(ClockSignal("clk125"), 125e6, 5e9)
        print(qpll)
        self.submodules += qpll
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # gtp
        tx_pads = platform.request("usb3_tx")
        rx_pads = platform.request("usb3_rx")
        self.submodules.gtp = gtp = GTP(qpll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True)
        gtp.add_stream_endpoints()
        gtp.add_controls()
        self.add_csr("gtp")
        gtp._tx_enable.storage.reset = 1 # Enabled by default
        gtp._rx_enable.storage.reset = 1 # Enabled by default
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

        # LFPS Polling Receive ---------------------------------------------------------------------
        lfps_receiver = LFPSReceiver(sys_clk_freq=sys_clk_freq)
        self.submodules += lfps_receiver
        self.comb += lfps_receiver.idle.eq(rxelecidle)

        # LFPS Polling Transmit --------------------------------------------------------------------
        lfps_transmitter = LFPSTransmitter(sys_clk_freq=sys_clk_freq, lfps_clk_freq=25e6)
        self.submodules += lfps_transmitter
        self.comb += [
            lfps_transmitter.polling.eq(1), # Always generate Polling LFPS for now to receive TSEQ/TS1
            txelecidle.eq(lfps_transmitter.tx_idle),
            gtp.tx_produce_pattern.eq(~lfps_transmitter.tx_idle),
            gtp.tx_pattern.eq(lfps_transmitter.tx_pattern),
        ]

        # Leds -------------------------------------------------------------------------------------
        self.comb += platform.request("user_led", 0).eq(gtp.tx_ready)
        self.comb += platform.request("user_led", 1).eq(gtp.rx_ready)
        self.comb += platform.request("user_led", 7).eq(rxelecidle)
        polling_timer = WaitTimer(int(sys_clk_freq*1e-1))
        self.submodules += polling_timer
        self.comb += [
            polling_timer.wait.eq(~lfps_receiver.polling),
            platform.request("user_led", 2).eq(~polling_timer.done)
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    platform = nexys_video.Platform()
    platform.add_extension(_usb3_io)
    soc = USB3SoC(platform)
    builder = Builder(soc, output_dir="build", csr_csv="csr.csv")
    vns = builder.build()

if __name__ == "__main__":
    main()
