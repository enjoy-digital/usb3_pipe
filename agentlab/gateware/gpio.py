#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab DUT GPIOs (tristate) and power outputs, with their events (changes/levels for
AgentLabVectorEvents)."""

from migen import *
from migen.genlib.cdc import MultiReg
from migen.fhdl.specials import Tristate

from litex.gen import *

from litex.soc.interconnect.csr import *

# AgentLab GPIO ------------------------------------------------------------------------------------

class AgentLabGPIO(LiteXModule):
    """N tristate DUT GPIOs: output enable/output (CSRs), synchronized inputs, edges (masked) as
    events.

    pads: GPIOs pads (tristate), None: internal i/o/oe signals (simulation).
    """
    def __init__(self, pads=None, n=None):
        n = len(pads) if pads is not None else n
        self.n       = n
        self.i       = Signal(n) # i (pads=None).
        self.o       = Signal(n) # o (pads=None).
        self.oe      = Signal(n) # o (pads=None).
        self.changes = Signal(n) # o: Edges (masked).
        self.levels  = Signal(n) # o: Synchronized inputs.

        self._oe        = CSRStorage(n, description="GPIOs Output Enable (bit per GPIO).")
        self._out       = CSRStorage(n, description="GPIOs Output (bit per GPIO).")
        self._in        = CSRStatus(n,  description="GPIOs Input (synchronized, bit per GPIO).")
        self._edge_mask = CSRStorage(n, description="GPIOs edge events enable (bit per GPIO).")

        # # #

        # Pads.
        self.comb += [
            self.o.eq(self._out.storage),
            self.oe.eq(self._oe.storage),
        ]
        if pads is not None:
            self.specials += Tristate(pads, self.o, self.oe, self.i)

        # Inputs (synchronized), edges.
        levels   = self.levels
        levels_d = Signal(n)
        self.specials += MultiReg(self.i, levels)
        self.sync += levels_d.eq(levels)
        self.comb += [
            self._in.status.eq(levels),
            self.changes.eq((levels ^ levels_d) & self._edge_mask.storage),
        ]

# AgentLab Power -----------------------------------------------------------------------------------

class AgentLabPower(LiteXModule):
    """M DUT power switches outputs (CSR), optional sense inputs, changes as events.

    pads: power switches control pads (None: CSR/events only), sense_pads: optional power status
    inputs (status: outputs state without).
    """
    def __init__(self, pads=None, n=None, sense_pads=None):
        n = len(pads) if pads is not None else n
        self.n       = n
        self.changes = Signal(n) # o: Outputs changes.
        self.levels  = Signal(n) # o: Outputs.

        self._out    = CSRStorage(n, description="Power switches outputs (bit per switch, 1: on).")
        self._status = CSRStatus(n,  description="Power status (sense inputs, else outputs).")

        # # #

        out   = self._out.storage
        out_d = Signal(n)
        self.sync += out_d.eq(out)
        self.comb += [
            self.levels.eq(out),
            self.changes.eq(out ^ out_d),
        ]
        if pads is not None:
            self.comb += pads.eq(out)
        if sense_pads is not None:
            self.specials += MultiReg(sense_pads, self._status.status)
        else:
            self.comb += self._status.status.eq(out)
