#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause


from migen import *

from litex.gen import *

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.cores.xadc import S7SystemMonitor
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litedram.modules import MT41K512M16
from litedram.phy     import s7ddrphy
from litedram.gen     import LiteDRAMCoreControl

from litescope import LiteScopeAnalyzer

from usb3_pipe       import A7USB3SerDes
from usb3_pipe.phy   import USB3PHY
from usb3_pipe.debug import USB3PHYDebug, USB3SerDesDebug
from usb3_core.examples import add_usb3_example, USB3_EXAMPLES

# USB3 IOs -----------------------------------------------------------------------------------------

_usb3_io_standard = [
    # SFP.
    ("sfp_tx", 0,
        Subsignal("p", Pins("D7")),
        Subsignal("n", Pins("C7")),
    ),

    ("sfp_rx", 0,
        Subsignal("p", Pins("D9")),
        Subsignal("n", Pins("C9")),
    ),
]

_usb3_io_mini = [
    # SFP.
    ("sfp_tx", 0,
        Subsignal("p", Pins("D5")),
        Subsignal("n", Pins("C5")),
    ),

    ("sfp_rx", 0,
        Subsignal("p", Pins("D11")),
        Subsignal("n", Pins("C11")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_dram=False):
        self.cd_sys    = ClockDomain()
        self.cd_oob    = ClockDomain()
        self.cd_clk200 = ClockDomain()
        if with_dram:
            self.cd_sys4x     = ClockDomain()
            self.cd_sys4x_dqs = ClockDomain()
            self.cd_idelay    = ClockDomain()

        # # #

        # Clk200.
        clk200_pads = platform.request("clk200")
        clk200      = Signal()
        self.specials += [
            Instance("IBUFDS", i_I=clk200_pads.p, i_IB=clk200_pads.n, o_O=clk200),
            Instance("BUFG",   i_I=clk200, o_O=self.cd_clk200.clk),
        ]

        # PLL.
        self.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq,   margin=0)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

        # DRAM clocks (A7DDRPHY: sys4x/sys4x_dqs, IDELAYCTRL reference).
        if with_dram:
            pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq, margin=0)
            pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, margin=0, phase=90)
            pll.create_clkout(self.cd_idelay,    200e6,          margin=0)
            self.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(SoCCore):
    def __init__(self, platform, with_uartbone=True, with_jtagbone=False, with_core=True,
        example="loopback", cpu_type=None, with_dram=False,
        with_serdes_debug=False, with_analyzer=False, analyzer_group="link", analyzer_csv="analyzer.csv",
        rxcdr_cfg=None, gtp_params={}, lpm_hold="none", with_lpm=False):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_dram=with_dram)

        # SoCCore ----------------------------------------------------------------------------------
        # No CPU by default (bridges only), CPU with the console example (BIOS on the USB3 CDC-ACM).
        # DRAM: CPU required (BIOS DRAM initialization/calibration). The BIOS console goes to a stub
        # UART: a crossover UART would block the BIOS (and the DRAM initialization) until a host
        # reads it. Initialization status: ddrctrl_init_done/init_error CSRs.
        assert not (with_dram and cpu_type is None), "DRAM requires a CPU (BIOS initialization)."
        with_stub_uart = with_dram and example not in ["console", "composite"]
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                = "USB3SoC",
            ident_version        = True,
            cpu_type             = cpu_type,
            integrated_rom_size  = 0x10000 if cpu_type is not None else 0,
            integrated_sram_size = 0x2000  if cpu_type is not None else 0,
            with_uart            = with_stub_uart,
            uart_name            = "stub",
            with_timer           = cpu_type is not None,
        )

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if with_dram:
            self.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype          = "DDR3",
                nphases          = 4,
                sys_clk_freq     = sys_clk_freq,
                iodelay_clk_freq = 200e6)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41K512M16(sys_clk_freq, "1:4"),
                l2_cache_size = 8192)
            # DRAM initialization status (written by the BIOS).
            self.ddrctrl = LiteDRAMCoreControl()

        # UARTBone / JTAGBone ----------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(baudrate=115200)
        if with_jtagbone:
            self.add_jtagbone()

        # USB3 SerDes ------------------------------------------------------------------------------
        self.usb3_serdes = usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk200"), # Directly from the oscillator (no PLL jitter).
            refclk_freq  = 200e6,
            tx_pads      = platform.request("sfp_tx"),
            rx_pads      = platform.request("sfp_rx"),
            rxcdr_cfg    = rxcdr_cfg)
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
        usb3_serdes.gtp.gtp_params.update(gtp_params) # Debug overrides (--gtp-param).

        # USB3 PHY (PIPE + LTSSM) ------------------------------------------------------------------
        self.usb3_phy = usb3_phy = USB3PHY(
            serdes       = usb3_serdes,
            sys_clk_freq = sys_clk_freq,
            role         = "device")

        # USB3 PHY/SerDes Debug --------------------------------------------------------------------
        self.usb3_debug = USB3PHYDebug(usb3_phy, usb3_serdes, sys_clk_freq)
        if with_serdes_debug:
            self.usb3_serdes_debug = USB3SerDesDebug(usb3_serdes, sys_clk_freq,
                ltssm_state = usb3_phy.ltssm_state)
            # FPGA temperature/voltages (RX quality vs temperature, bench/usb3_soak.py).
            self.xadc = S7SystemMonitor()

        # GTP LPM adaptation hold (debug, --lpm-hold): adapt only in Polling.RxEQ, or never.
        if lpm_hold != "none":
            lpm_hold_signal = Signal()
            if lpm_hold == "rxeq":
                self.comb += lpm_hold_signal.eq(usb3_phy.ltssm_state != usb3_phy.ltssm_states["Polling.RxEQ"])
            else:
                self.comb += lpm_hold_signal.eq(1)
            usb3_serdes.gtp.gtp_params.update(
                i_RXLPMHFHOLD = lpm_hold_signal,
                i_RXLPMLFHOLD = lpm_hold_signal,
                i_RXOSHOLD    = lpm_hold_signal,
            )

        # USB3 Core (Example design) -------------------------------------------------------------
        if with_core:
            add_usb3_example(self, usb3_phy, sys_clk_freq, example=example, with_lpm=with_lpm)
        else:
            self.comb += usb3_phy.source.ready.eq(1)

        # Leds -------------------------------------------------------------------------------------
        self.comb += [
            platform.request("user_led", 0).eq(~usb3_serdes.ready),
            platform.request("user_led", 1).eq(~usb3_phy.u0),
            platform.request("user_led", 2).eq(~usb3_phy.recovery),
            platform.request("user_led", 3).eq(1), # Not Used.
        ]

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            pipe  = usb3_phy.pipe
            ltssm = usb3_phy.ltssm
            link_signals = [
                usb3_phy.ltssm_state,
                ltssm.u0,
                ltssm.recovery,
                ltssm.rx_ready,
                ltssm.tx_ready,
                ltssm.idle_handshake,
                ltssm.rx_idle8,
                ltssm.ts2_decode,
            ]
            groups = {
                "link" : link_signals + [
                    pipe.ts_rx_ts1, pipe.ts_rx_ts1_inv, pipe.ts_rx_ts2, pipe.ts_tx_done,
                    pipe.ts_tx_tseq, pipe.ts_tx_ts1, pipe.ts_tx_ts2,
                    pipe.lfps_rx_polling, pipe.lfps_tx_polling, pipe.lfps_tx_count,
                    usb3_serdes.rx_idle, usb3_serdes.rx_errors, usb3_serdes.rx_polarity,
                ],
                "lfps" : link_signals + [
                    usb3_serdes.tx_idle, usb3_serdes.tx_lfps, usb3_serdes.tx_pattern,
                    usb3_serdes.rx_idle, usb3_serdes.rx_errors,
                    pipe.lfps_rx_polling, pipe.lfps_rx_ping, pipe.lfps_rx_exit_u1,
                    pipe.lfps_rx_exit_u2, pipe.lfps_rx_wakeup_u3, pipe.lfps_rx_reset,
                    pipe.lfps_tx_polling, pipe.lfps_tx_count,
                ],
                "ts" : link_signals + [
                    pipe.ts_rx_ts1, pipe.ts_rx_ts1_inv, pipe.ts_rx_ts2, pipe.ts_tx_done,
                    pipe.ts_tx_tseq, pipe.ts_tx_ts1, pipe.ts_tx_ts2,
                    usb3_serdes.source, usb3_serdes.sink,
                ],
                "datapath" : link_signals + [
                    usb3_serdes.rx_errors,
                    usb3_serdes.rx_datapath.skip_remover.skip,
                    usb3_serdes.source, usb3_serdes.sink,
                    usb3_phy.source, usb3_phy.sink,
                ],
            }
            if with_serdes_debug:
                groups["phase"] = link_signals + [
                    self.usb3_serdes_debug.rx_tx_phase,
                    usb3_serdes.rx_idle, usb3_serdes.rx_errors,
                ]
            self.analyzer = LiteScopeAnalyzer(groups[analyzer_group],
                depth        = 4096,
                clock_domain = "sys",
                register     = True,
                csr_csv      = analyzer_csv
            )

# Build --------------------------------------------------------------------------------------------

import argparse

def main():
    with open("README.md") as f:
        description = [str(f.readline()) for i in range(7)]
    parser = argparse.ArgumentParser(description="".join(description[1:]), formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--variant",       default="mini",      help="Select LiteX Acorn Baseboard variant (default: mini).",  choices=["mini", "standard"])
    parser.add_argument("--build",         action="store_true", help="Build bitstream.")
    parser.add_argument("--load",          action="store_true", help="Load bitstream.")
    parser.add_argument("--with-jtagbone",     action="store_true", help="Enable JTAGBone.")
    parser.add_argument("--no-uartbone",       action="store_true", help="Disable UARTBone.")
    parser.add_argument("--no-core",           action="store_true", help="Build without the USB3 Core (PHY only).")
    parser.add_argument("--example",           default="loopback",  help="USB3 Device Core example design.", choices=USB3_EXAMPLES)
    parser.add_argument("--with-lpm",          action="store_true", help="U1/U2 Link Power Management (loopback/bench examples).")
    parser.add_argument("--cpu-type",          default=None,        help="CPU (console example, ex: vexriscv).")
    parser.add_argument("--with-dram",         action="store_true", help="Enable the DDR3 SDRAM (requires a CPU, default with the msc_dram example).")
    parser.add_argument("--with-serdes-debug", action="store_true", help="Enable SerDes debug (TX/loopback controls, clocks/RX errors measurements).")
    parser.add_argument("--with-analyzer",     action="store_true", help="Enable LiteScope Analyzer.")
    parser.add_argument("--analyzer-group",    default="link",      help="LiteScope probes group.", choices=["link", "lfps", "ts", "datapath", "phase"])
    parser.add_argument("--rxcdr-cfg",         default=None,        help="GTP RXCDR_CFG override (hex, debug).")
    parser.add_argument("--gtp-param",         action="append", default=[], help="GTP parameter override NAME=VALUE (debug, ex: p_RXCDR_HOLD_DURING_EIDLE=1).")
    parser.add_argument("--lpm-hold",          default="none",      help="GTP LPM adaptation hold (debug).", choices=["none", "rxeq", "always"])
    parser.add_argument("--vivado-place-directive", default="default", help="Vivado place_design directive (the GTP reference clock is fabric-routed on this board: RX quality depends on the placement, see doc/debugging-guide.md).")
    parser.add_argument("--output-dir",        default=None,        help="Build directory (default: build/sqrl_acorn).")
    args = parser.parse_args()

    if not args.build and not args.load:
        parser.print_help()

    output_dir = args.output_dir or "build/sqrl_acorn"
    # DRAM disk: DDR3 SDRAM, initialized by the BIOS (VexRiscv CPU by default, console on a stub UART).
    with_dram = args.with_dram or (args.example == "msc_dram")
    cpu_type  = args.cpu_type
    if with_dram and cpu_type is None:
        cpu_type = "vexriscv"
    platform = sqrl_acorn.Platform(with_multiboot=False)
    if args.variant == "standard":
        platform.add_extension(_usb3_io_standard)
    if args.variant == "mini":
        platform.add_extension(_usb3_io_mini)
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
    soc     = USB3SoC(platform,
        with_uartbone     = not args.no_uartbone,
        with_jtagbone     = args.with_jtagbone,
        with_core         = not args.no_core,
        example           = args.example,
        cpu_type          = cpu_type,
        with_dram         = with_dram,
        with_serdes_debug = args.with_serdes_debug,
        with_analyzer     = args.with_analyzer,
        analyzer_group    = args.analyzer_group,
        analyzer_csv      = os.path.join(output_dir, "analyzer.csv"),
        rxcdr_cfg         = None if args.rxcdr_cfg is None else int(args.rxcdr_cfg, 0),
        gtp_params        = {k: int(v, 0) for k, v in (p.split("=") for p in args.gtp_param)},
        lpm_hold          = args.lpm_hold,
        with_lpm          = args.with_lpm)
    builder = Builder(soc, output_dir=output_dir, csr_csv=os.path.join(output_dir, "csr.csv"))
    builder.build(run=args.build, vivado_place_directive=args.vivado_place_directive)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
