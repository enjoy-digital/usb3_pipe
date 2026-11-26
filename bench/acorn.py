#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3-PIPE on the SQRL Acorn CLE-215+ (Artix7 GTP) in a LiteX Acorn Baseboard.

Baseboard Mini by default (--variant standard), SFP2USB adapter in the SFP cage: Gen 1x1 (5Gbps),
main validation target (doc/manual/09_boards.md).

Clocking: GTP reference clock from the 200MHz oscillator (GTGREFCLK0, no PCIe host clock:
doc/manual/06_transceivers.md), sys at 125MHz.

Bridge: UARTBone by default, JTAGBone with --with-jtagbone (baseboard FT4232H). Optional DDR3
(--with-dram, msc_dram example) with a VexRiscv CPU.

Gen 1x2 (--lanes 2, Mini, experimental): Lane 0 on SFP-0, Lane 1 on SFP-1. Two boards, SFP-0 <->
SFP-0 and SFP-1 <-> SFP-1 (SFP DAC cables), one in each role (--role host/device) with the PRBS
Link-Layer stand-ins (bench/usb3_lanes.py), or the Device Core (--lanes-core) for a Gen 1x2 USB host
through a dual SFP to USB-C breakout.

Build and load (JTAG, volatile): ./bench/acorn.py --build --load [--example <name>]. Shared
options: bench/usb3_soc.py.
"""

import argparse

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex_boards.platforms import sqrl_acorn

from litex.build.generic_platform import *

from litex.soc.cores.clock import *
from litex.soc.cores.xadc  import S7SystemMonitor

from litedram.modules import MT41K512M16
from litedram.phy     import s7ddrphy
from litedram.gen     import LiteDRAMCoreControl

from usb3_pipe import A7USB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device, add_usb3_lanes_link, usb3_analyzer
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

_usb3_io_mini_x2 = [
    # SFP-1 (Gen 1x2: Lane 1).
    ("sfp_tx", 1,
        Subsignal("p", Pins("B4")),
        Subsignal("n", Pins("A4")),
    ),

    ("sfp_rx", 1,
        Subsignal("p", Pins("B8")),
        Subsignal("n", Pins("A8")),
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
        gtp_params={}, rxcdr_cfg=None, lpm_hold="none", tx_postcursor=0, tx_diffctrl=None,
        lanes=1, role="device", x2_capable=True, lanes_core=False,
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
        # Gen 1x2 (lanes=2): Lane 0 on SFP-0, Lane 1 on SFP-1 (same GTP quad), Lane 1 on the Lane 0
        # Quad PLL and TX clock (lanes TX in lockstep).
        usb3_serdes = []
        for lane in range(lanes):
            serdes = A7USB3SerDes(platform,
                sys_clk      = self.crg.cd_sys.clk,
                sys_clk_freq = sys_clk_freq,
                refclk_pads  = ClockSignal("clk200"), # Directly from the oscillator (no PLL jitter).
                refclk_freq  = 200e6,
                tx_pads      = platform.request("sfp_tx", lane if lanes == 2 else None),
                rx_pads      = platform.request("sfp_rx", lane if lanes == 2 else None),
                rxcdr_cfg    = rxcdr_cfg,
                pll          = None if lane == 0 else usb3_serdes[0].pll,
                tx_clk       = None if lane == 0 else usb3_serdes[0].gtp.cd_tx.clk,
                # Gen 1x2 (two Acorn Minis): RX electrical idle detection spikes after the partner
                # Polling LFPS bursts on some bitstream loads, rejected below the shortest LFPS burst.
                rx_idle_min_burst = 40e-9 if lanes == 2 else None)
            # TX post-cursor pre-emphasis (UG482 TXPOSTCURSOR) and differential swing (TXDIFFCTRL).
            if tx_postcursor:
                serdes.gtp.gtp_params.update(i_TXPOSTCURSOR=tx_postcursor)
            if tx_diffctrl is not None:
                serdes.gtp.gtp_params.update(i_TXDIFFCTRL=tx_diffctrl)
            serdes.gtp.gtp_params.update(gtp_params) # Debug overrides (--gtp-param).
            usb3_serdes.append(serdes)
        # Fabric reference clock on GTGREFCLK0 (doc/manual: Transceivers): allowed as a warning.
        platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        leds = [platform.request("user_led", i) for i in range(3)]
        if lanes == 1:
            add_usb3_device(self, usb3_serdes[0],
                leds          = leds,
                leds_inverted = True,
                **usb3_kwargs)
        else:
            add_usb3_lanes_link(self, usb3_serdes,
                role          = role,
                x2_capable    = x2_capable,
                leds          = leds,
                leds_inverted = True,
                with_core     = lanes_core,
                example       = usb3_kwargs.get("example", "loopback"),
                with_lpm      = usb3_kwargs.get("with_lpm", False),
                max_burst     = usb3_kwargs.get("max_burst", 8))
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
            for serdes in usb3_serdes:
                serdes.gtp.gtp_params.update(
                    i_RXLPMHFHOLD = lpm_hold_signal,
                    i_RXLPMLFHOLD = lpm_hold_signal,
                    i_RXOSHOLD    = lpm_hold_signal,
                )

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            self.analyzer = usb3_analyzer(self, analyzer_group, analyzer_csv)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="USB3-PIPE on the SQRL Acorn CLE-215+ (Artix7 GTP), LiteX Acorn Baseboard (Mini).")
    parser.add_argument("--variant",   default="mini",      help="LiteX Acorn Baseboard variant.", choices=["mini", "standard"])
    parser.add_argument("--with-dram", action="store_true", help="Enable the DDR3 SDRAM (requires a CPU, default with the msc_dram example).")
    usb3_soc_args(parser, output_dir="build/sqrl_acorn")
    gtp_group = parser.add_argument_group(title="GTP bring-up/debug overrides")
    gtp_group.add_argument("--rxcdr-cfg", default=None,   help="GTP RXCDR_CFG override (hex).")
    gtp_group.add_argument("--gtp-param", default=[],     help="GTP parameter override NAME=VALUE (ex: p_RXCDR_HOLD_DURING_EIDLE=1).", action="append")
    gtp_group.add_argument("--lpm-hold",  default=None,   help="GTP LPM adaptation hold (rxeq: adapt only in Polling.RxEQ; default: rxeq with --lanes 2, else none).", choices=["none", "rxeq", "always"])
    gtp_group.add_argument("--tx-postcursor", default=None, type=int, help="GTP TX post-cursor pre-emphasis (TXPOSTCURSOR 0-31; default: 6 with --lanes 2, else 0).")
    gtp_group.add_argument("--tx-diffctrl",   default=None, type=int, help="GTP TX differential swing (TXDIFFCTRL 0-15; default: 15 with --lanes 2, else the SerDes default).")
    x2_group = parser.add_argument_group(title="Gen 1x2 (Mini, two lanes on SFP-0/SFP-1, experimental)")
    x2_group.add_argument("--lanes",      default=1, type=int, help="Lanes: 1 (Gen 1x1, SFP-0) or 2 (Gen 1x2, SFP-0/SFP-1).", choices=[1, 2])
    x2_group.add_argument("--role",       default="device",    help="--lanes 2: USB3PHY role (two boards: one in each role).", choices=["device", "host"])
    x2_group.add_argument("--no-x2",      action="store_true", help="--lanes 2: not Gen 1x2 capable (Gen 1x1 negotiated on SFP-0).")
    x2_group.add_argument("--lanes-core", action="store_true", help="--lanes 2, Device role: USB3 Device Core (--example) instead of the PRBS stand-ins.")
    args = parser.parse_args()

    if args.lanes == 2:
        assert args.variant == "mini", "--lanes 2: Mini only (SFP-0/SFP-1)."
        assert not args.with_serdes_debug, "--lanes 2: no --with-serdes-debug."
        assert (args.cpu_type is None) and not args.with_dram, "--lanes 2: no CPU/DRAM."
        assert not (args.lanes_core and args.no_core), "--lanes-core: USB3 Device Core required."
        assert not (args.lanes_core and args.role == "host"), "--lanes-core: Device role only."
    else:
        assert args.role == "device", "--role host requires --lanes 2."
        assert not (args.no_x2 or args.lanes_core), "--no-x2/--lanes-core require --lanes 2."

    # DRAM disk: DDR3 SDRAM, initialized by the BIOS (VexRiscv CPU by default, console on a stub
    # UART).
    with_dram = args.with_dram or (args.example == "msc_dram")
    if with_dram and args.cpu_type is None:
        args.cpu_type = "vexriscv"

    platform = sqrl_acorn.Platform(with_multiboot=False)
    if args.variant == "standard":
        platform.add_extension(_usb3_io_standard)
    if args.variant == "mini":
        platform.add_extension(_usb3_io_mini)
        if args.lanes == 2:
            platform.add_extension(_usb3_io_mini_x2)
        platform.add_extension(sqrl_acorn._litex_acorn_baseboard_mini_io, prepend=True)
    soc = USB3SoC(platform,
        with_dram   = with_dram,
        rxcdr_cfg   = None if args.rxcdr_cfg is None else int(args.rxcdr_cfg, 0),
        gtp_params  = {k: int(v, 0) for k, v in (p.split("=") for p in args.gtp_param)},
        lpm_hold    = args.lpm_hold or ("rxeq" if args.lanes == 2 else "none"),
        # Gen 1x2 (two Acorn Minis): TX post-cursor 6 and full swing, the SFP-1 transmitter path the
        # weakest; without the full swing some bitstream loads never trained at Gen 1x2.
        tx_postcursor = args.tx_postcursor if args.tx_postcursor is not None else (6 if args.lanes == 2 else 0),
        tx_diffctrl   = args.tx_diffctrl if args.tx_diffctrl is not None else (15 if args.lanes == 2 else None),
        lanes       = args.lanes,
        role        = args.role,
        x2_capable  = not args.no_x2,
        lanes_core  = args.lanes_core,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc)

if __name__ == "__main__":
    main()
