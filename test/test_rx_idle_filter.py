#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""RX electrical idle filter (usb3_pipe/datapath.py RXIdleFilter): idle gaps shorter than
idle_cycles filtered (idle detection toggling during a burst), optional rejection of non-idle
spikes shorter than burst_cycles, bursts stretched by stretch_cycles."""

import random
import unittest

from migen import *

from usb3_pipe.datapath import RXIdleFilter

# Helpers ------------------------------------------------------------------------------------------

def run_filter(raw, idle_cycles=10, burst_cycles=1):
    rx_elecidle = Signal(reset=1)
    dut = RXIdleFilter(rx_elecidle, idle_cycles=idle_cycles, burst_cycles=burst_cycles)
    out = []
    def generator():
        for v in raw + [1]*32:
            yield rx_elecidle.eq(v)
            yield
            out.append((yield dut.idle))
    run_simulation(dut, generator())
    return dut, out

def reference_filter(raw, idle_cycles=10):
    """Previous RXIdleFilter (no spike rejection): idle after idle_cycles idle cycles."""
    out, count, r = [], idle_cycles, [1, 1] # MultiReg (2 stages).
    for v in raw + [1]*32:
        out.append(int(count == idle_cycles))
        count = min(count + 1, idle_cycles) if r[1] else 0
        r = [v, r[0]]
    return out

def bursts(out):
    """(start, length) of the non-idle runs."""
    r, start = [], None
    for i, v in enumerate(out + [1]):
        if not v and start is None:
            start = i
        if v and start is not None:
            r.append((start, i - start))
            start = None
    return r

# Test RX Idle Filter ------------------------------------------------------------------------------

class TestRXIdleFilter(unittest.TestCase):
    def test_default_unchanged(self):
        # burst_cycles=1 (default): previous behavior, cycle-exact (random idle detection toggling).
        prng = random.Random(42)
        raw  = []
        for _ in range(200):
            raw += [prng.choice([0, 1])]*prng.randint(1, 16)
        dut, out = run_filter(raw)
        self.assertEqual(dut.stretch_cycles, 9)
        self.assertEqual(out, reference_filter(raw)[:len(out)])

    def test_spike_rejected(self):
        # 32ns spike (4 cycles at 125MHz), 4us after a 1us burst (seen on hardware): only the burst.
        raw = [1]*20 + [0]*125 + [1]*384 + [0]*4 + [1]*100
        dut, out = run_filter(raw, burst_cycles=5)
        b = bursts(out)
        self.assertEqual(len(b), 1)
        self.assertEqual(b[0][1], 125 + dut.stretch_cycles)

    def test_shortest_lfps_burst(self):
        # 40ns burst (Ping.LFPS tBurst min, 5 cycles at 125MHz): kept, stretched by stretch_cycles.
        raw = [1]*20 + [0]*5 + [1]*100
        dut, out = run_filter(raw, burst_cycles=5)
        self.assertEqual([l for _, l in bursts(out)], [5 + dut.stretch_cycles])

    def test_idle_toggling_in_burst(self):
        # Idle gaps shorter than idle_cycles inside a burst: one burst.
        for burst_cycles in [1, 5]:
            with self.subTest(burst_cycles=burst_cycles):
                raw = [1]*20 + ([0]*20 + [1]*6)*5 + [0]*20 + [1]*100
                dut, out = run_filter(raw, burst_cycles=burst_cycles)
                self.assertEqual(len(bursts(out)), 1)

if __name__ == "__main__":
    unittest.main()
