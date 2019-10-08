# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.lfps import LFPSUnit
from usb3_pipe.ordered_set import OrderedSetUnit
from usb3_pipe.ltssm import LTSSM

# USB3 PHY -----------------------------------------------------------------------------------------

class USB3PHY(Module):
    def __init__(self, serdes, sys_clk_freq):
        assert sys_clk_freq > 125e6
        self.enable = Signal() # i
        self.ready  = Signal() # o

        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # LFPS Unit --------------------------------------------------------------------------------
        lfps_unit = LFPSUnit(sys_clk_freq=sys_clk_freq, serdes=serdes)
        self.submodules.lfps_unit = lfps_unit

        # OrderedSet Unit --------------------------------------------------------------------------
        ordered_set_unit = OrderedSetUnit(serdes=serdes)
        self.submodules.ordered_set_unit = ordered_set_unit

        # LTSSM ------------------------------------------------------------------------------------
        ltssm = LTSSM(lfps_unit=lfps_unit, ordered_set_unit=ordered_set_unit)
        self.submodules.ltssm = ltssm
        self.comb += self.ready.eq(ltssm.polling_fsm.idle)

