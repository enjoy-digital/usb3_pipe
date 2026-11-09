#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""RX electrical idle filter (usb3_pipe/datapath.py RXIdleFilter): idle gaps shorter than
idle_cycles filtered (idle detection toggling during a burst), non-idle spikes shorter than
burst_cycles rejected (shorter than the shortest LFPS burst, 40ns), bursts stretched by
stretch_cycles."""

import unittest

from migen import *

from usb3_pipe.datapath import RXIdleFilter

def run_filter(raw, idle_cycles=10, burst_cycles=5):
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

def bursts(out):
    """(start, length) of the non-idle runs."""
    r, start = [], None
    for i, v in enumerate(out + [1]):
        if not v and start is None:
            start = i
        if v and start is not None:
            r.append((start, i - start)); start = None
    return r

class TestRXIdleFilter(unittest.TestCase):
    def test_spike_rejected(self):
        # 32ns spike (4 cycles at 125MHz), 4us after a 1us burst (seen on hardware): only the burst.
        raw = [1]*20 + [0]*125 + [1]*384 + [0]*4 + [1]*100
        dut, out = run_filter(raw)
        b = bursts(out)
        self.assertEqual(len(b), 1)
        self.assertEqual(b[0][1], 125 + dut.stretch_cycles)

    def test_shortest_lfps_burst(self):
        # 40ns burst (Ping.LFPS tBurst min, 5 cycles): kept, stretched by stretch_cycles.
        raw = [1]*20 + [0]*5 + [1]*100
        dut, out = run_filter(raw)
        self.assertEqual([l for _, l in bursts(out)], [5 + dut.stretch_cycles])

    def test_idle_toggling_in_burst(self):
        # Idle gaps shorter than idle_cycles inside a burst: one burst.
        raw = [1]*20 + ([0]*20 + [1]*6)*5 + [0]*20 + [1]*100
        dut, out = run_filter(raw)
        self.assertEqual(len(bursts(out)), 1)

    def test_no_spike_rejection(self):
        # burst_cycles=1: the previous behavior (a spike seen as a burst, stretch idle_cycles - 1).
        raw = [1]*20 + [0]*4 + [1]*100
        dut, out = run_filter(raw, burst_cycles=1)
        self.assertEqual(dut.stretch_cycles, 9)
        self.assertEqual([l for _, l in bursts(out)], [4 + 9])

if __name__ == "__main__":
    unittest.main()
