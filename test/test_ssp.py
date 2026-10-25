#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SuperSpeedPlus Polling: SCD1/SCD2 and LBPM generation/decoding (usb3_pipe/lfps.py), SSP Polling
negotiation between two ports (usb3_pipe/ltssm.py: Gen 1x2/Gen 1x1 capability match, SuperSpeed
partner fallback)."""

import random
import unittest

from migen import *

from litex.gen import *

from usb3_pipe.lfps  import *
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

class TestSSP(unittest.TestCase):
    def test_scd_bits(self):
        # SCD1 = 0010b, SCD2 = 1101b, transmitted LSb first (6.9.4.2).
        self.assertEqual((SCD1, SCD2), (0b0010, 0b1101))
        self.assertEqual(scd_bits(SCD1), [0, 1, 0, 0])
        self.assertEqual(scd_bits(SCD2), [1, 0, 1, 1])

    def test_lbpm_values(self):
        # Table 7-13.
        self.assertEqual(lbpm_phy_capability(gen2=False, x2=False), 0b00000000)
        self.assertEqual(lbpm_phy_capability(gen2=False, x2=True),  0b00100000)
        self.assertEqual(lbpm_phy_capability(gen2=True,  x2=True),  0b00100100)
        self.assertEqual(lbpm_phy_ready(dfp=True,  x2=True), 0b01000001)
        self.assertEqual(lbpm_phy_ready(dfp=False, x2=True), 0b00000001)

    def scd_run(self, mode):
        class DUT(LiteXModule):
            def __init__(self):
                self.gen      = SCDGenerator(SYS_CLK_FREQ, TIMER_SCALE)
                self.frontend = LFPSRXFrontEnd()
                self.chk      = SCDChecker(self.frontend, SYS_CLK_FREQ, TIMER_SCALE)
                self.comb += self.frontend.idle.eq(~self.gen.tx_lfps)
        dut = DUT()
        events = {"scd1": 0, "scd2": 0, "polling": 0, "plain": 0}
        def generator(dut):
            yield dut.gen.enable.eq(1)
            yield dut.gen.mode.eq(mode)
            for _ in range(8000):
                for name in events:
                    events[name] += (yield getattr(dut.chk, name))
                yield
            events["count"] = (yield dut.gen.count)
        run_simulation(dut, generator(dut))
        return events

    def test_scd(self):
        scd1 = self.scd_run(1)
        self.assertGreaterEqual(scd1["scd1"], scd1["count"] - 2)
        self.assertEqual(scd1["scd2"], 0)
        self.assertEqual(scd1["plain"], 0)
        scd2 = self.scd_run(2)
        self.assertGreaterEqual(scd2["scd2"], scd2["count"] - 2)
        self.assertEqual(scd2["scd1"], 0)
        plain = self.scd_run(0)
        self.assertEqual(plain["scd1"] + plain["scd2"], 0)
        self.assertGreaterEqual(plain["plain"], plain["count"] - 2)
        self.assertEqual(plain["polling"], plain["plain"])

    def test_lbpm(self):
        rng      = random.Random(0)
        messages = [rng.getrandbits(8) for _ in range(8)]
        class DUT(LiteXModule):
            def __init__(self):
                self.gen      = LBPMGenerator(SYS_CLK_FREQ, TIMER_SCALE)
                self.frontend = LFPSRXFrontEnd()
                self.chk      = LBPMChecker(self.frontend, SYS_CLK_FREQ, TIMER_SCALE)
                self.comb += self.frontend.idle.eq(~self.gen.tx_lfps)
        dut      = DUT()
        received = []
        def generator(dut):
            yield dut.gen.data.eq(messages[0])
            yield
            yield dut.gen.enable.eq(1)
            for m in messages[1:]:
                # Next LBPM data, sampled at the end of the current one.
                yield dut.gen.data.eq(m)
                while not (yield dut.gen.sent):
                    yield
                yield
            for _ in range(1000):
                yield
        def checker(dut):
            for _ in range(5000):
                if (yield dut.chk.received):
                    received.append((yield dut.chk.data))
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(received[:len(messages)], messages)

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
