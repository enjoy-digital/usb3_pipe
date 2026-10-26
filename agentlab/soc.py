#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab SoC (doc/agentlab.md), shared by the targets (agentlab/targets/).

- AgentLabSoC       : SoCCore (no CPU, or a CPU for the DRAM initialization), Wishbone bridges
                      (UARTBone over the USB3 control CDC-ACM, optional board UARTBone/JTAGBone).
- add_agentlab_dram : DDR3 SDRAM (soc.sdram), initialized/calibrated by the BIOS (CPU, console on a
                      stub UART), dram_ready from the LiteDRAM init status (ddrctrl).
- add_agentlab      : USB3 PHY + composite device (AgentLabUSB), AgentLab instrument on the DUT
                      pads, logic analyzer (DRAM ring buffer), optional Eyes (UVC view, text grid
                      Wishbone region agentlab_eyes).
- agentlab_args/agentlab_build: common target arguments and build/load.

The targets only provide their CRG, USB3 SerDes, DRAM PHY/module and DUT pads:

    dut_pads = {
        "uarts" : [uart pads (tx/rx) or None (internal loopback)] (4),
        "gpio"  : GPIOs pads (tristate) or None,
        "power" : power switches pads or None,
        "la"    : logic analyzer channels pads (<= 32), "probes" (internal probes) or None,
    }

DRAM: soc.sdram (LiteDRAM crossbar) is shared: the logic analyzer takes one write port and one read
port on the upper half of the DRAM (ring buffer, agentlab_la_ring_base/size constants), the lower
half is left to the CPU (main_ram, BIOS) and the other users (crossbar.get_port(), soc.dram_ready).
"""

import os

from migen import *

from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.uart import Stream2Wishbone

from litedram.gen import LiteDRAMCoreControl

from usb3_pipe.phy   import USB3PHY
from usb3_pipe.debug import USB3PHYDebug

from agentlab.gateware.usb        import AgentLabUSB, agentlab_usb_connect
from agentlab.gateware.instrument import AgentLab

# AgentLab SoC -------------------------------------------------------------------------------------

class AgentLabSoC(SoCCore):
    """SoCCore, Wishbone bridges (board UARTBone/JTAGBone), AgentLab added with add_agentlab.

    No CPU by default; cpu_type (ex: vexriscv) for the DRAM: the BIOS initializes/calibrates the
    DRAM, its console goes to a stub UART (a crossover UART would block the BIOS until a host
    reads it).
    """
    mem_map = {
        "agentlab_eyes" : 0x9000_0000, # Eyes text grid (in the IO region of the LiteX CPUs).
    }

    def __init__(self, platform, sys_clk_freq, with_uartbone=False, with_jtagbone=False,
        cpu_type=None, **kwargs):
        # SoCCore ----------------------------------------------------------------------------------
        with_cpu = cpu_type is not None
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                = "AgentLab",
            ident_version        = True,
            cpu_type             = cpu_type,
            integrated_rom_size  = 0x10000 if with_cpu else 0,
            integrated_sram_size = 0x2000  if with_cpu else 0,
            with_uart            = with_cpu,
            uart_name            = "stub",
            with_timer           = with_cpu,
            **kwargs
        )

        # UARTBone / JTAGBone ----------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(baudrate=115200)
        if with_jtagbone:
            self.add_jtagbone()

    def add_agentlab_dram(self, phy, module, l2_cache_size=8192):
        """DDR3 SDRAM on phy/module (soc.sdram, CPU main_ram), initialized by the BIOS:
        soc.dram_ready (LiteDRAM init done without error, ddrctrl CSRs written by the BIOS)."""
        assert self.cpu_type is not None, "DRAM requires a CPU (BIOS initialization)."
        self.ddrphy = phy
        self.add_sdram("sdram",
            phy           = phy,
            module        = module,
            l2_cache_size = l2_cache_size)
        self.ddrctrl    = LiteDRAMCoreControl()
        self.dram_ready = Signal()
        self.comb += self.dram_ready.eq(self.ddrctrl.init_done.storage &
            ~self.ddrctrl.init_error.storage)

    def add_agentlab(self, serdes, dut_pads, n_gpios=0, n_power=0, leds=[], leds_inverted=False,
        la_ring_base=None, la_ring_size=None, with_eyes=False):
        """USB3 PHY + AgentLab composite device and instrument on serdes/dut_pads, LEDs (SerDes
        ready, U0, configured). Logic analyzer with dut_pads["la"] (requires the DRAM: ring buffer
        at la_ring_base/la_ring_size bytes, default: upper half of the DRAM). with_eyes: UVC view
        of the bench (gateware text-mode renderer, no CPU/DRAM)."""
        sys_clk_freq = self.sys_clk_freq
        uart_pads    = dut_pads.get("uarts", [None]*4)
        la_inputs    = dut_pads.get("la", None)
        with_la      = la_inputs is not None

        # USB3 SerDes / PHY ------------------------------------------------------------------------
        self.usb3_serdes = serdes
        self.usb3_phy    = usb3_phy = USB3PHY(
            serdes       = serdes,
            sys_clk_freq = sys_clk_freq,
            role         = "device")
        self.usb3_debug  = USB3PHYDebug(usb3_phy, serdes, sys_clk_freq)

        # USB3 Composite Device --------------------------------------------------------------------
        self.usb3 = usb3 = AgentLabUSB(usb3_phy, sys_clk_freq, n_uarts=len(uart_pads),
            with_la   = with_la,
            with_eyes = with_eyes)
        usb3.core.add_csr()

        # Control: UARTBone over the USB3 control CDC-ACM (litex_server --uart --uart-port
        # /dev/ttyACMx).
        self.usb3_uartbone = usb3_uartbone = Stream2Wishbone(usb3.control, clk_freq=sys_clk_freq)
        self.bus.add_master(name="usb3_uartbone", master=usb3_uartbone.wishbone)

        # Logic Analyzer DRAM ports (ring buffer) -------------------------------------------------
        la_kwargs = {}
        if with_la:
            assert hasattr(self, "sdram"), "Logic analyzer requires the DRAM (add_agentlab_dram)."
            write_port = self.sdram.crossbar.get_port(mode="write")
            read_port  = self.sdram.crossbar.get_port(mode="read")
            dram_size  = 2**write_port.address_width*write_port.data_width//8
            la_kwargs  = dict(
                write_port = write_port,
                read_port  = read_port,
                ring_base  = dram_size//2 if la_ring_base is None else la_ring_base,
                ring_size  = dram_size//2 if la_ring_size is None else la_ring_size,
            )

        # AgentLab Instrument ----------------------------------------------------------------------
        self.agentlab = agentlab = AgentLab(sys_clk_freq,
            uart_pads  = uart_pads,
            gpio_pads  = dut_pads.get("gpio",  None),
            n_gpios    = n_gpios,
            power_pads = dut_pads.get("power", None),
            n_power    = n_power,
            la_inputs  = la_inputs,
            la_kwargs  = la_kwargs,
            with_eyes  = with_eyes)

        # Configuration constants (csr.csv, host side).
        self.add_constant("AGENTLAB_UARTS",            agentlab.n_uarts)
        self.add_constant("AGENTLAB_GPIOS",            agentlab.n_gpios)
        self.add_constant("AGENTLAB_POWER",            agentlab.n_power)
        self.add_constant("AGENTLAB_EVENTS_INTERFACE", usb3.map["events"]["interface"])
        self.add_constant("AGENTLAB_EVENTS_EP",        usb3.map["events"]["in_ept"])
        if with_la:
            la = agentlab.la
            self.comb += la.dram_ready.eq(self.dram_ready)
            self.add_constant("AGENTLAB_LA_CHANNELS",      la.n)
            self.add_constant("AGENTLAB_LA_INTERFACE",     usb3.map["la"]["interface"])
            self.add_constant("AGENTLAB_LA_EP",            usb3.map["la"]["in_ept"])
            self.add_constant("AGENTLAB_LA_RING_BASE",     la.ring_base)
            self.add_constant("AGENTLAB_LA_RING_SIZE",     la.ring_size)
            self.add_constant("AGENTLAB_LA_RING_SAMPLES",  la.ring_samples)
            self.add_constant("AGENTLAB_LA_WORD_SAMPLES",  la.ratio)
            self.add_constant("AGENTLAB_LA_BLOCK_SAMPLES", la.block_samples)

        # Eyes: text grid Wishbone region (one cell per 32-bit word), configuration constants.
        if with_eyes:
            eyes = agentlab.eyes
            self.bus.add_slave(name="agentlab_eyes", slave=eyes.bus, region=SoCRegion(
                origin = self.mem_map["agentlab_eyes"],
                size   = 2**log2_int(4*eyes.cols*eyes.rows, need_pow2=False),
                cached = False))
            self.add_constant("AGENTLAB_EYES_INTERFACE",  usb3.map["eyes"]["interface"])
            self.add_constant("AGENTLAB_EYES_EP",         usb3.map["eyes"]["in_ept"])
            self.add_constant("AGENTLAB_EYES_WIDTH",      eyes.width)
            self.add_constant("AGENTLAB_EYES_HEIGHT",     eyes.height)
            self.add_constant("AGENTLAB_EYES_COLUMNS",    eyes.cols)
            self.add_constant("AGENTLAB_EYES_ROWS",       eyes.rows)
            self.add_constant("AGENTLAB_EYES_WAVE_DEPTH", eyes.wave_depth)
            self.add_constant("AGENTLAB_EYES_LANES",      agentlab.eyes_lanes)

        # DUT UARTs <-> CDC-ACM functions, events -> Bulk IN, Eyes -> UVC.
        self.comb += agentlab_usb_connect(usb3, agentlab)

        # Leds -------------------------------------------------------------------------------------
        for led, signal in zip(leds, [serdes.ready, usb3_phy.u0, usb3.core.configured]):
            self.comb += led.eq(~signal if leds_inverted else signal)

# Arguments / Build --------------------------------------------------------------------------------

def agentlab_args(parser, output_dir):
    """Common AgentLab target arguments."""
    parser.add_argument("--build",      action="store_true", help="Build bitstream.")
    parser.add_argument("--load",       action="store_true", help="Load bitstream.")
    parser.add_argument("--output-dir", default=output_dir,  help=f"Build directory (default: {output_dir}).")

def agentlab_build(parser, args, soc, **kwargs):
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
