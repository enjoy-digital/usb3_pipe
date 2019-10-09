#!/usr/bin/env python3

import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *


from usb3_pipe import USB3SerDesModel
from usb3_pipe import USB3PIPE

# IOs ----------------------------------------------------------------------------------------------

class SimPins(Pins):
    def __init__(self, n=1):
        Pins.__init__(self, "s "*n)


_io = [
    ("sys_clk", 0, SimPins(1)),
    ("sys_rst", 0, SimPins(1))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    default_clk_name = "sys_clk"

    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

    def do_finalize(self, fragment):
        pass

# USB3PIPESim --------------------------------------------------------------------------------------

class USB3PIPESim(SoCMini):
    def __init__(self):
        platform = Platform()
        sys_clk_freq = int(133e6)
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # USB3 Host
        host_usb3_serdes = USB3SerDesModel()
        host_usb3_pipe   = USB3PIPE(serdes=host_usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules += host_usb3_serdes, host_usb3_pipe

        # USB3 Device
        dev_usb3_serdes = USB3SerDesModel()
        dev_usb3_pipe   = USB3PIPE(serdes=dev_usb3_serdes, sys_clk_freq=sys_clk_freq)
        self.submodules += dev_usb3_serdes, dev_usb3_pipe

        # Connect Host <--> Device
        self.comb += host_usb3_serdes.connect(dev_usb3_serdes)

        # Simulation End
        self.sync += If(host_usb3_pipe.ready & dev_usb3_pipe.ready, Finish())

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PIPE Simulation")
    parser.add_argument("--trace", action="store_true", help="enable VCD tracing")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    soc = USB3PIPESim()
    builder = Builder(soc, output_dir="build")
    builder.build(sim_config=sim_config, trace=args.trace)


if __name__ == "__main__":
    main()
