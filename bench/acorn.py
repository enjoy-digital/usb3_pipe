#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.cores.xadc import S7SystemMonitor

from litedram.modules import MT41K512M16
from litedram.phy     import s7ddrphy
from litedram.gen     import LiteDRAMCoreControl

from usb3_pipe import A7USB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device, usb3_analyzer
from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

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

class USB3SoC(USB3BaseSoC):
    def __init__(self, platform, cpu_type=None, with_dram=False,
        gtp_params={}, rxcdr_cfg=None, lpm_hold="none",
        with_analyzer=False, analyzer_group="link", analyzer_csv="analyzer.csv",
        usb3_kwargs={}, **kwargs):
        sys_clk_freq = int(125e6)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_dram=with_dram)

        # USB3BaseSoC ------------------------------------------------------------------------------
        # DRAM: CPU required (BIOS DRAM initialization/calibration). The BIOS console goes to a stub
        # UART: a crossover UART would block the BIOS (and the DRAM initialization) until a host
        # reads it. Initialization status: ddrctrl_init_done/init_error CSRs.
        assert not (with_dram and cpu_type is None), "DRAM requires a CPU (BIOS initialization)."
        with_stub_uart = with_dram and usb3_kwargs.get("example") not in ["console", "composite"]
        USB3BaseSoC.__init__(self, platform, sys_clk_freq,
            cpu_type  = cpu_type,
            with_uart = with_stub_uart,
            **kwargs)

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

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = ClockSignal("clk200"), # Directly from the oscillator (no PLL jitter).
            refclk_freq  = 200e6,
            tx_pads      = platform.request("sfp_tx"),
            rx_pads      = platform.request("sfp_rx"),
            rxcdr_cfg    = rxcdr_cfg)
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")
        usb3_serdes.gtp.gtp_params.update(gtp_params) # Debug overrides (--gtp-param).

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        add_usb3_device(self, usb3_serdes,
            leds          = [platform.request("user_led", i) for i in range(3)],
            leds_inverted = True,
            **usb3_kwargs)
        self.comb += platform.request("user_led", 3).eq(1) # Not Used.

        # FPGA temperature/voltages (RX quality vs temperature, bench/usb3_soak.py) ----------------
        if usb3_kwargs.get("with_serdes_debug", False):
            self.xadc = S7SystemMonitor()

        # GTP LPM adaptation hold (debug, --lpm-hold): adapt only in Polling.RxEQ, or never.
        if lpm_hold != "none":
            ltssm_state     = self.usb3_phy.ltssm_state
            ltssm_states    = self.usb3_phy.ltssm_states
            lpm_hold_signal = Signal()
            if lpm_hold == "rxeq":
                self.comb += lpm_hold_signal.eq(ltssm_state != ltssm_states["Polling.RxEQ"])
            else:
                self.comb += lpm_hold_signal.eq(1)
            usb3_serdes.gtp.gtp_params.update(
                i_RXLPMHFHOLD = lpm_hold_signal,
                i_RXLPMLFHOLD = lpm_hold_signal,
                i_RXOSHOLD    = lpm_hold_signal,
            )

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            self.analyzer = usb3_analyzer(self, analyzer_group, analyzer_csv)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the SQRL Acorn CLE-215+ (Artix7 GTP), LiteX Acorn Baseboard (Mini).")
    parser.add_argument("--variant",   default="mini",      help="LiteX Acorn Baseboard variant.", choices=["mini", "standard"])
    parser.add_argument("--with-dram", action="store_true", help="Enable the DDR3 SDRAM (requires a CPU, default with the msc_dram example).")
    parser.add_argument("--vivado-place-directive", default="default", help="Vivado place_design directive (the GTP reference clock is fabric-routed on this board: RX quality depends on the placement, see doc/debugging-guide.md).")
    usb3_soc_args(parser, output_dir="build/sqrl_acorn")
    gtp_group = parser.add_argument_group(title="GTP bring-up/debug overrides")
    gtp_group.add_argument("--rxcdr-cfg", default=None,   help="GTP RXCDR_CFG override (hex).")
    gtp_group.add_argument("--gtp-param", default=[],     help="GTP parameter override NAME=VALUE (ex: p_RXCDR_HOLD_DURING_EIDLE=1).", action="append")
    gtp_group.add_argument("--lpm-hold",  default="none", help="GTP LPM adaptation hold (rxeq: adapt only in Polling.RxEQ).", choices=["none", "rxeq", "always"])
    args = parser.parse_args()

    # DRAM disk: DDR3 SDRAM, initialized by the BIOS (VexRiscv CPU by default, console on a stub UART).
    with_dram = args.with_dram or (args.example == "msc_dram")
    if with_dram and args.cpu_type is None:
        args.cpu_type = "vexriscv"

    platform = sqrl_acorn.Platform(with_multiboot=False)
    if args.variant == "standard":
        platform.add_extension(_usb3_io_standard)
    if args.variant == "mini":
        platform.add_extension(_usb3_io_mini)
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
    soc = USB3SoC(platform,
        with_dram   = with_dram,
        rxcdr_cfg   = None if args.rxcdr_cfg is None else int(args.rxcdr_cfg, 0),
        gtp_params  = {k: int(v, 0) for k, v in (p.split("=") for p in args.gtp_param)},
        lpm_hold    = args.lpm_hold,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc, vivado_place_directive=args.vivado_place_directive)

if __name__ == "__main__":
    main()
