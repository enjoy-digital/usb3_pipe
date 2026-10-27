#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SuperSpeedPlus Polling (usb3_pipe/ltssm.py SSPPolling, 7.5.4.3-7.5.4.6): negotiation between two
ports (Gen 1x2/Gen 2x1/Gen 1x1 capability match, SuperSpeed partner fallback), LBPMs decoded on the
wire."""

import unittest

from migen import *

from litex.gen import *

from usb3_pipe.lfps  import SCDGenerator, LFPSRXFrontEnd, LBPMChecker
from usb3_pipe.lfps  import LBPM_PHY_CAPABILITY, LBPM_PHY_READY, lbpm_phy_capability
from usb3_pipe.ltssm import SSPPolling

SYS_CLK_FREQ = 133e6
TIMER_SCALE  = 10

# Helpers ------------------------------------------------------------------------------------------

class Link(LiteXModule):
    """Two ports, electrical idle of each RX = partner TX not sending LFPS (with a few cycles of
    channel/detection delay), LBPMs sent by each port decoded by a monitor (monitors[name])."""
    def __init__(self, a, b, delay=3):
        self.a = a
        self.b = b
        self.monitors = {}
        for name, tx, rx in [("a", a, b), ("b", b, a)]:
            sr = Signal(delay)
            self.sync += sr.eq(Cat(~tx.tx_lfps, sr))
            self.comb += rx.idle.eq(sr[-1])
            frontend = LFPSRXFrontEnd()
            checker  = LBPMChecker(frontend, SYS_CLK_FREQ, TIMER_SCALE)
            self.submodules += frontend, checker
            self.comb += frontend.idle.eq(sr[-1])
            self.monitors[name] = checker

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
        lbpms   = {"a": [], "b": []}
        @passive
        def monitor(dut):
            while True:
                for name, checker in dut.monitors.items():
                    if (yield checker.received):
                        lbpms[name].append((yield checker.data))
                yield
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
                                results[(name, status)] = (i, (yield port.x2), (yield port.gen2))
                        done &= any(k[0] == name for k in results)
                if done:
                    break
                yield
        run_simulation(dut, [generator(dut), monitor(dut)])
        self.lbpms = lbpms
        return results

    def gen2_test(self, host, device):
        # SSP DFP (host) / UFP (device): capability exchange decoded on the wire and negotiated
        # speed/lanes, returns (x2, gen2).
        r = self.polling_run(SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, **host),
                             SSPPolling(SYS_CLK_FREQ, TIMER_SCALE, **device))
        self.assertIn(("a", "done"), r)
        self.assertIn(("b", "done"), r)
        self.assertEqual(r[("a", "done")][1:], r[("b", "done")][1:])
        # Device LBPMs: its own PHY Capability first, PHY Ready last.
        caps = [d for d in self.lbpms["b"] if (d & 0x3) == LBPM_PHY_CAPABILITY]
        self.assertEqual(caps[0], lbpm_phy_capability(gen2=device["gen2_capable"],
            x2=device["x2_capable"]))
        self.assertEqual(self.lbpms["b"][-1] & 0x3, LBPM_PHY_READY)
        # Common capability: at least 4 matched PHY Capability LBPMs sent by both ports.
        x2, gen2 = r[("b", "done")][1:]
        for name in ["a", "b"]:
            common = [d for d in self.lbpms[name] if d == lbpm_phy_capability(gen2=gen2, x2=x2)]
            self.assertGreaterEqual(len(common), 4)
        return x2, gen2

    def test_polling_gen2x1(self):
        # Gen 2x1 host / Gen 2x1 device: Gen 2x1 (the device advertises Gen 2, single lane).
        gen2x1 = dict(x2_capable=False, gen2_capable=True)
        self.assertEqual(self.gen2_test(gen2x1, gen2x1), (0, 1))

    def test_polling_gen2x2_host_gen2x1(self):
        # Gen 2x2 host / Gen 2x1 device: Gen 2x1 (Table 7-14), the host adjusts its capability.
        self.assertEqual(self.gen2_test(dict(x2_capable=True, gen2_capable=True),
            dict(x2_capable=False, gen2_capable=True)), (0, 1))

    def test_polling_gen1x1_host_gen2x1(self):
        # SuperSpeedPlus Gen 1x1 host / Gen 2x1 device: Gen 1x1, the device adjusts its capability.
        self.assertEqual(self.gen2_test(dict(x2_capable=False, gen2_capable=False),
            dict(x2_capable=False, gen2_capable=True)), (0, 0))

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
