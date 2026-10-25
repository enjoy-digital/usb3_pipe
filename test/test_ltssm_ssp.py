#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SuperSpeedPlus Polling (usb3_pipe/ltssm.py SSPPolling, 7.5.4.3-7.5.4.6): negotiation between two
ports (Gen 1x2/Gen 1x1 capability match, SuperSpeed partner fallback)."""

import unittest

from migen import *

from litex.gen import *

from usb3_pipe.lfps  import SCDGenerator
from usb3_pipe.ltssm import SSPPolling

SYS_CLK_FREQ = 133e6
TIMER_SCALE  = 10

# Helpers ------------------------------------------------------------------------------------------

class Link(LiteXModule):
    """Two ports, electrical idle of each RX = partner TX not sending LFPS (with a few cycles of
    channel/detection delay)."""
    def __init__(self, a, b, delay=3):
        self.a = a
        self.b = b
        for tx, rx in [(a, b), (b, a)]:
            sr = Signal(delay)
            self.sync += sr.eq(Cat(~tx.tx_lfps, sr))
            self.comb += rx.idle.eq(sr[-1])

class SSPort(LiteXModule):
    """SuperSpeed port: Polling.LFPS with non varying tRepeat."""
    def __init__(self):
        self.gen     = SCDGenerator(SYS_CLK_FREQ, TIMER_SCALE)
        self.tx_lfps = Signal()
        self.idle    = Signal()
        self.comb += [
            self.gen.enable.eq(1),
            self.gen.mode.eq(0),
            self.tx_lfps.eq(self.gen.tx_lfps),
        ]

# Tests --------------------------------------------------------------------------------------------

class TestSSPPolling(unittest.TestCase):
    def polling_run(self, a, b, cycles=40000):
        dut = Link(a, b)
        results = {}
        def generator(dut):
            for port in [a, b]:
                if isinstance(port, SSPPolling):
                    yield port.start.eq(1)
            yield a.ds_port.eq(1)
            for i in range(cycles):
                done = True
                for name, port in [("a", a), ("b", b)]:
                    if isinstance(port, SSPPolling):
                        for status in ["done", "ss_fallback", "fail"]:
                            if (yield getattr(port, status)) and (name, status) not in results:
                                results[(name, status)] = (i, (yield port.x2))
                        done &= any(k[0] == name for k in results)
                if done:
                    break
                yield
        run_simulation(dut, generator(dut))
        return results

    def test_polling_x2(self):
        # Both Gen 1x2: Gen 1x2 negotiated, both ports done.
        r = self.polling_run(SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, x2_capable=True),
                             SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, x2_capable=True))
        self.assertIn(("a", "done"), r)
        self.assertIn(("b", "done"), r)
        self.assertEqual(r[("a", "done")][1], 1)
        self.assertEqual(r[("b", "done")][1], 1)

    def test_polling_x2_x1(self):
        # Gen 1x2 DFP / Gen 1x1 SSP UFP: Gen 1x1 (Table 7-14), the DFP adjusts its capability.
        r = self.polling_run(SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, x2_capable=True),
                             SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, x2_capable=False))
        self.assertEqual(r[("a", "done")][1], 0)
        self.assertEqual(r[("b", "done")][1], 0)

    def test_polling_ss_partner(self):
        # SuperSpeed partner (no SCD): SuperSpeed fallback.
        r = self.polling_run(SSPPolling(SYS_CLK_FREQ, TIMER_SCALE), SSPort())
        self.assertIn(("a", "ss_fallback"), r)

if __name__ == "__main__":
    unittest.main()
