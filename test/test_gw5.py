#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.gw5    import gw5_usb3_rate_writes
from usb3_pipe.serdes import GW5RateSequencer

class FakeUPAR(Module):
    """GTR12UPAR user port model: a write takes 3 cycles once granted."""
    def __init__(self):
        self.addr   = Signal(24)
        self.wrdata = Signal(32)
        self.we     = Signal()
        self.re     = Signal()
        self.done   = Signal()
        self.writes = []
        self.busy   = Signal(2)
        self.sync += [
            self.done.eq(0),
            If(self.busy != 0,
                self.busy.eq(self.busy - 1),
                If(self.busy == 1, self.done.eq(1)),
            ).Elif(self.we & ~self.done,
                self.busy.eq(3),
            )
        ]

class TestGW5RateSequencer(unittest.TestCase):
    def test_rate_writes(self):
        for lane in [0, 1]:
            writes = gw5_usb3_rate_writes(lane)
            # Rate dependent registers: CDR, QPLL1, TX lanes clock tree x2, TX/RX gear FIFOs.
            self.assertEqual(len(writes), 6)

    def test_sequencer(self):
        writes = gw5_usb3_rate_writes(0)
        upar   = FakeUPAR()
        dut    = GW5RateSequencer(upar, writes, settle_cycles=8)
        dut.submodules += upar
        log    = []
        def generator():
            for rate in [1, 0]:
                yield dut.rate_10g.eq(rate)
                for _ in range(8):
                    yield
                self.assertEqual((yield dut.busy), 1)
                for _ in range(200):
                    if (yield upar.we) and not (yield upar.busy) and not (yield upar.done):
                        log.append((rate, (yield upar.addr), (yield upar.wrdata)))
                    if not (yield dut.busy):
                        break
                    yield
                self.assertEqual((yield dut.busy), 0)
        run_simulation(dut, generator())
        expected = [(1, a, v10) for a, v5, v10 in writes] + [(0, a, v5) for a, v5, v10 in writes]
        self.assertEqual(log, expected)

if __name__ == "__main__":
    unittest.main()
