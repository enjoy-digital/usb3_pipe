#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 SoC shared by the bench targets (acorn.py, kc705.py, alibaba_xcku3p.py).

- USB3BaseSoC     : SoCCore (no CPU by default), UARTBone/JTAGBone.
- add_usb3_device : USB3PHY + debug (USB3PHYDebug, USB3SerDesDebug), USB3 Device Core example
                    design and LEDs.
- usb3_analyzer   : LiteScope analyzer (usb3_pipe.debug.usb3_analyzer_groups).
- usb3_soc_args/usb3_soc_argdict/usb3_soc_build: common target arguments and build/load.

The targets only provide their CRG, IOs and USB3 SerDes (+ board specifics).
"""

import os

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litescope import LiteScopeAnalyzer

from usb3_pipe.phy   import USB3PHY
from usb3_pipe.debug import USB3PHYDebug, USB3SerDesDebug
from usb3_pipe.debug import usb3_analyzer_groups, USB3_ANALYZER_GROUPS

from usb3_core.examples import add_usb3_example, USB3_EXAMPLES

# USB3 SoC -----------------------------------------------------------------------------------------

class USB3BaseSoC(SoCCore):
    """SoCCore with Wishbone bridges (UARTBone/JTAGBone), USB3 device added with add_usb3_device.

    No CPU by default (bridges only), CPU with the console/composite examples (BIOS on the USB3
    CDC-ACM) or for the DRAM initialization. with_uart: stub UART (BIOS console when the console is
    not on the USB3 CDC-ACM).
    """
    def __init__(self, platform, sys_clk_freq, cpu_type=None, with_uart=False,
        with_uartbone=True, with_jtagbone=False):
        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                = "USB3SoC",
            ident_version        = True,
            cpu_type             = cpu_type,
            integrated_rom_size  = 0x10000 if cpu_type is not None else 0,
            integrated_sram_size = 0x2000  if cpu_type is not None else 0,
            with_uart            = with_uart,
            uart_name            = "stub",
            with_timer           = cpu_type is not None,
        )

        # UARTBone / JTAGBone ----------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(baudrate=115200)
        if with_jtagbone:
            self.add_jtagbone()

# USB3 Device --------------------------------------------------------------------------------------

class RXRawSnapshot(LiteXModule):
    """Raw RX words snapshot (bring-up): circular capture of the raw words (rx_nr: reset-less rx clock, kept
    across RX re-initializations) with their flags, stopped `depth//2` words after a trigger
    (flags bit selected by the trigger CSR) once armed; read back with index/data CSRs (oldest word
    at index 0, 40-bit word in data0/data1[:8], flags in data1[8:])."""
    def __init__(self, rx_raw, flags, depth=1024):
        from migen.genlib.cdc import MultiReg, PulseSynchronizer
        from litex.soc.interconnect.csr import CSR, CSRStorage, CSRStatus
        self._arm     = CSR()
        self._trigger = CSRStorage(bits_for(len(flags) - 1))
        self._done    = CSRStatus()
        self._index   = CSRStorage(bits_for(depth - 1))
        self._data0   = CSRStatus(32)
        self._data1   = CSRStatus(32)

        # # #

        mem = Memory(len(rx_raw) + len(flags), depth)
        wr  = mem.get_port(write_capable=True, clock_domain="rx_nr")
        rd  = mem.get_port(clock_domain="sys")
        self.specials += mem, wr, rd
        arm_ps = PulseSynchronizer("sys", "rx_nr")
        self.submodules += arm_ps
        sel  = Signal(len(self._trigger.storage))
        done = Signal()
        self.specials += [
            MultiReg(self._trigger.storage, sel, "rx_nr"),
            MultiReg(done, self._done.status),
        ]
        armed = Signal()
        post  = Signal(max=depth)
        adr   = Signal(max=depth)
        start = Signal(max=depth)
        self.comb += arm_ps.i.eq(self._arm.re)
        self.sync.rx_nr += [
            If(arm_ps.o,
                armed.eq(1), done.eq(0), post.eq(0)
            ).Elif(~done,
                adr.eq(adr + 1),
                If(armed & (flags >> sel)[0],
                    armed.eq(0), post.eq(depth//2)
                ).Elif(post != 0,
                    post.eq(post - 1),
                    If(post == 1, done.eq(1), start.eq(adr + 1))
                )
            )
        ]
        self.comb += [
            wr.adr.eq(adr),
            wr.dat_w.eq(Cat(rx_raw, flags)),
            wr.we.eq(~done),
            rd.adr.eq(start + self._index.storage),
            self._data0.status.eq(rd.dat_r[:32]),
            self._data1.status.eq(rd.dat_r[32:]),
        ]

def add_usb3_gen2_rx(soc, serdes, phy):
    """Gen 2 bring-up (M1): 10Gbps line rate once Gen 2 is negotiated in SuperSpeedPlus Polling
    (rate CSR: 0 auto, 1 force 5Gbps, 2 force 10Gbps) and Gen 2 RX datapath on the raw transceiver
    symbols with its debug CSRs (usb3_gen2_rx_*: alignment status, blocks/Ordered Sets counters)."""
    from litex.soc.interconnect.csr import CSRStorage, CSRStatus
    from usb3_pipe.datapath import Gen2RXDatapath
    from usb3_pipe.debug import Gen2RXDebug
    xcvr = getattr(serdes, serdes.xcvr_name)
    soc.usb3_gen2_rate = CSRStorage(2, description="Line rate: 0 auto (negotiated), 1 5Gbps, 2 10Gbps.")
    rate = soc.usb3_gen2_rate.storage
    soc.comb += serdes.rate_10g.eq(
        (rate == 2) | ((rate == 0) & phy.gen2 & ~phy.ltssm.ssp_polling))
    # Gen 2 RX datapath/counters in a reset-less RX clock domain: kept across the RX
    # re-initializations (rate changes), counters accumulated over the 10Gbps periods.
    soc.cd_rx_nr = ClockDomain(reset_less=True)
    soc.comb += soc.cd_rx_nr.clk.eq(ClockSignal("rx"))
    gen2_rx = ClockDomainsRenamer("rx_nr")(Gen2RXDatapath())
    soc.usb3_gen2_rx_datapath = gen2_rx
    soc.comb += [
        gen2_rx.sink.valid.eq(1),
        gen2_rx.sink.data.eq(xcvr.rx_raw),
        gen2_rx.source.ready.eq(1),
    ]
    soc.usb3_gen2_rx = Gen2RXDebug(gen2_rx, clock_domain="rx_nr")
    aligner = gen2_rx.aligner
    soc.usb3_gen2_snapshot = RXRawSnapshot(xcvr.rx_raw, flags=Cat(
        aligner.source.valid & aligner.source.header_error,                 # 0: Header error.
        aligner.source.valid & aligner.source.header_error & aligner.locked, # 1: Header error (locked).
        aligner.error,                                                      # 2: Alignment error.
        aligner.aligned,                                                    # 3.
        aligner.locked,                                                     # 4.
        aligner.polarity,                                                   # 5.
        aligner.source.valid,                                               # 6.
    ))
    soc.usb3_gen2_status = CSRStatus(2, description="Gen 2 negotiated [0], 10Gbps rate [1].")
    soc.comb += soc.usb3_gen2_status.status.eq(Cat(phy.gen2, serdes.rate_10g))

def add_usb3_device(soc, serdes, leds=[], leds_inverted=False,
    with_serdes_debug = False,
    with_core         = True,
    example           = "loopback",
    with_lpm          = False):
    """Add a USB3 device on serdes to soc: USB3PHY + debug, USB3 Device Core example design (with_core)
    and LEDs (SerDes ready, U0, Recovery).

    Note: a function (as add_usb3_example), not a USB3BaseSoC method, and the debug modules (XADC,
    LiteScope analyzer: usb3_analyzer) assigned from the target __init__: Migen signal names follow
    the instantiation stack (gateware/analyzer.csv names kept)."""
    sys_clk_freq = soc.sys_clk_freq

    # USB3 SerDes ----------------------------------------------------------------------------------
    soc.usb3_serdes = serdes

    # USB3 PHY (PIPE + LTSSM) ----------------------------------------------------------------------
    gen2_capable = hasattr(serdes, "rate_10g") and getattr(serdes, "gen2_capable", False)
    soc.usb3_phy = usb3_phy = USB3PHY(
        serdes       = serdes,
        sys_clk_freq = sys_clk_freq,
        role         = "device",
        gen2_capable = gen2_capable)
    if gen2_capable:
        add_usb3_gen2_rx(soc, serdes, usb3_phy)

    # USB3 PHY/SerDes Debug ------------------------------------------------------------------------
    soc.usb3_debug = USB3PHYDebug(usb3_phy, serdes, sys_clk_freq)
    if with_serdes_debug:
        soc.usb3_serdes_debug = USB3SerDesDebug(serdes, sys_clk_freq,
            ltssm_state = usb3_phy.ltssm_state)

    # USB3 Core (Example design) -------------------------------------------------------------------
    if with_core:
        add_usb3_example(soc, usb3_phy, sys_clk_freq, example=example, with_lpm=with_lpm)
    else:
        soc.comb += usb3_phy.source.ready.eq(1)

    # Leds -----------------------------------------------------------------------------------------
    for led, signal in zip(leds, [serdes.ready, usb3_phy.u0, usb3_phy.recovery]):
        soc.comb += led.eq(~signal if leds_inverted else signal)

# USB3 Analyzer ------------------------------------------------------------------------------------

def usb3_analyzer(soc, group="link", csv="analyzer.csv"):
    """LiteScope analyzer on the soc USB3 PHY/SerDes (usb3_analyzer_groups group), to be assigned by
    the SoC: soc.analyzer = usb3_analyzer(soc, ...)."""
    groups = usb3_analyzer_groups(soc.usb3_phy, soc.usb3_serdes,
        serdes_debug = getattr(soc, "usb3_serdes_debug", None))
    assert group in groups, f"{group} group requires --with-serdes-debug."
    return LiteScopeAnalyzer(groups[group],
        depth        = 4096,
        clock_domain = "sys",
        register     = True,
        csr_csv      = csv
    )

# Arguments / Build --------------------------------------------------------------------------------

def usb3_soc_args(parser, output_dir, with_uartbone=True):
    """Common bench target arguments (with_uartbone: target with a UART, --no-uartbone/--with-jtagbone;
    JTAGBone only otherwise)."""
    parser.add_argument("--build",             action="store_true", help="Build bitstream.")
    parser.add_argument("--load",              action="store_true", help="Load bitstream.")
    parser.add_argument("--output-dir",        default=output_dir,  help=f"Build directory (default: {output_dir}).")
    if with_uartbone:
        parser.add_argument("--no-uartbone",   action="store_true", help="Disable UARTBone.")
        parser.add_argument("--with-jtagbone", action="store_true", help="Enable JTAGBone.")
    parser.add_argument("--cpu-type",          default=None,        help="CPU (console/composite examples, ex: vexriscv).")
    parser.add_argument("--no-core",           action="store_true", help="Build without the USB3 Core (PHY only).")
    parser.add_argument("--example",           default="loopback",  help="USB3 Device Core example design.", choices=USB3_EXAMPLES)
    parser.add_argument("--with-lpm",          action="store_true", help="U1/U2 Link Power Management (loopback/bench examples).")
    parser.add_argument("--with-serdes-debug", action="store_true", help="Enable SerDes debug (TX/loopback controls, clocks/RX errors measurements).")
    parser.add_argument("--with-analyzer",     action="store_true", help="Enable LiteScope Analyzer.")
    parser.add_argument("--analyzer-group",    default="link",      help="LiteScope probes group (phase: requires --with-serdes-debug).", choices=USB3_ANALYZER_GROUPS)

def usb3_soc_argdict(args):
    """Target SoC arguments from the common bench target arguments: USB3BaseSoC (bridges, CPU),
    analyzer (usb3_analyzer) and usb3_kwargs (add_usb3_device)."""
    return {
        "with_uartbone"     : not getattr(args, "no_uartbone", True),
        "with_jtagbone"     : getattr(args, "with_jtagbone", True),
        "cpu_type"          : args.cpu_type,
        "with_analyzer"     : args.with_analyzer,
        "analyzer_group"    : args.analyzer_group,
        "analyzer_csv"      : os.path.join(args.output_dir, "analyzer.csv"),
        "usb3_kwargs"       : {
            "with_serdes_debug" : args.with_serdes_debug,
            "with_core"         : not args.no_core,
            "example"           : args.example,
            "with_lpm"          : args.with_lpm,
        },
    }

def usb3_soc_build(parser, args, soc, **kwargs):
    """Build (--build, kwargs: toolchain arguments) and load (--load) soc."""
    if not args.build and not args.load:
        parser.print_help()
    builder = Builder(soc,
        output_dir = args.output_dir,
        csr_csv    = os.path.join(args.output_dir, "csr.csv"))
    builder.build(run=args.build, **kwargs)
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))
