#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Timebase: 64-bit free-running timestamp (sys_clk cycles), shared by every event."""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# AgentLab Timebase --------------------------------------------------------------------------------

class AgentLabTimebase(LiteXModule):
    """64-bit free-running counter (sys_clk cycles), CSR latch/read."""
    def __init__(self):
        self.timestamp = timestamp = Signal(64) # o

        self._latch = CSRStorage(fields=[
            CSRField("latch", size=1, pulse=True, description="Latch the timestamp (read: value)."),
        ])
        self._value = CSRStatus(64, description="Timestamp latched (sys_clk cycles).")

        # # #

        self.sync += [
            timestamp.eq(timestamp + 1),
            If(self._latch.fields.latch, self._value.status.eq(timestamp)),
        ]
