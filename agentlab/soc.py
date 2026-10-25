#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab SoC (doc/agentlab.md), shared by the targets (agentlab/targets/).

- AgentLabSoC     : SoCCore (no CPU), Wishbone bridges (UARTBone over the USB3 control CDC-ACM,
                    optional board UARTBone/JTAGBone).
- add_agentlab    : USB3 PHY + composite device (AgentLabUSB), AgentLab instrument on the DUT pads.
- agentlab_args/agentlab_build: common target arguments and build/load.

The targets only provide their CRG, USB3 SerDes and DUT pads:

    dut_pads = {
        "uarts" : [uart pads (tx/rx) or None (internal loopback)] (4),
        "gpio"  : GPIOs pads (tristate) or None,
        "power" : power switches pads or None,
    }
"""

import os

from migen import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.uart import Stream2Wishbone

from usb3_pipe.phy   import USB3PHY
from usb3_pipe.debug import USB3PHYDebug

from agentlab.gateware.usb        import AgentLabUSB, agentlab_usb_connect
from agentlab.gateware.instrument import AgentLab

# AgentLab SoC -------------------------------------------------------------------------------------

class AgentLabSoC(SoCCore):
    """SoCCore without CPU, Wishbone bridges (board UARTBone/JTAGBone), AgentLab added with
    add_agentlab."""
    def __init__(self, platform, sys_clk_freq, with_uartbone=False, with_jtagbone=False):
        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                = "AgentLab",
            ident_version        = True,
            cpu_type             = None,
            integrated_rom_size  = 0,
            integrated_sram_size = 0,
            with_uart            = False,
            with_timer           = False,
        )

        # UARTBone / JTAGBone ----------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(baudrate=115200)
        if with_jtagbone:
            self.add_jtagbone()

    def add_agentlab(self, serdes, dut_pads, n_gpios=0, n_power=0, leds=[], leds_inverted=False):
        """USB3 PHY + AgentLab composite device and instrument on serdes/dut_pads, LEDs (SerDes
        ready, U0, configured)."""
        sys_clk_freq = self.sys_clk_freq
        uart_pads    = dut_pads.get("uarts", [None]*4)

        # USB3 SerDes / PHY ------------------------------------------------------------------------
        self.usb3_serdes = serdes
        self.usb3_phy    = usb3_phy = USB3PHY(
            serdes       = serdes,
            sys_clk_freq = sys_clk_freq,
            role         = "device")
        self.usb3_debug  = USB3PHYDebug(usb3_phy, serdes, sys_clk_freq)

        # USB3 Composite Device --------------------------------------------------------------------
        self.usb3 = usb3 = AgentLabUSB(usb3_phy, sys_clk_freq, n_uarts=len(uart_pads))
        usb3.core.add_csr()

        # Control: UARTBone over the USB3 control CDC-ACM (litex_server --uart --uart-port
        # /dev/ttyACMx).
        self.usb3_uartbone = usb3_uartbone = Stream2Wishbone(usb3.control, clk_freq=sys_clk_freq)
        self.bus.add_master(name="usb3_uartbone", master=usb3_uartbone.wishbone)

        # AgentLab Instrument ----------------------------------------------------------------------
        self.agentlab = agentlab = AgentLab(sys_clk_freq,
            uart_pads  = uart_pads,
            gpio_pads  = dut_pads.get("gpio",  None),
            n_gpios    = n_gpios,
            power_pads = dut_pads.get("power", None),
            n_power    = n_power)

        # Configuration constants (csr.csv, host side).
        self.add_constant("AGENTLAB_UARTS",            agentlab.n_uarts)
        self.add_constant("AGENTLAB_GPIOS",            agentlab.n_gpios)
        self.add_constant("AGENTLAB_POWER",            agentlab.n_power)
        self.add_constant("AGENTLAB_EVENTS_INTERFACE", usb3.map["events"]["interface"])
        self.add_constant("AGENTLAB_EVENTS_EP",        usb3.map["events"]["in_ept"])

        # DUT UARTs <-> CDC-ACM functions, events -> Bulk IN.
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
