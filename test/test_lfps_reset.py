#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.lfps   import LFPSUnit
from usb3_pipe.serdes import RXErrorMonitor

SYS_CLK_FREQ = 125e6
TIMER_SCALE  = 1000 # LFPS timings divided by 1000 (ms -> us).

def ms_to_cycles(t):
    return int(t*1e-3*SYS_CLK_FREQ/TIMER_SCALE)

class SerDesStub(Module):
    def __init__(self, with_rx_errors=True):
        self.rx_idle    = Signal(reset=1)
        self.tx_idle    = Signal()
        self.tx_pattern = Signal(20)
        if with_rx_errors:
            self.rx_errors = Signal()

class DecoderStub:
    def __init__(self):
        self.invalid = Signal()

# Warm Reset detection (USB 3.2 6.9.3, Table 6-30) -------------------------------------------------

class TestWarmResetDetection(unittest.TestCase):
    def run_line(self, segments, with_rx_errors=True):
        """Drive the RX line with (duration_ms, idle, errors) segments, return the rx_reset trace."""
        serdes = SerDesStub(with_rx_errors)
        dut    = LFPSUnit(serdes=serdes, sys_clk_freq=SYS_CLK_FREQ, timer_scale=TIMER_SCALE)
        dut.submodules += serdes
        trace  = []

        def generator(dut):
            for duration, idle, errors in segments:
                yield serdes.rx_idle.eq(idle)
                if with_rx_errors:
                    yield serdes.rx_errors.eq(errors)
                for _ in range(ms_to_cycles(duration)):
                    yield
                    trace.append((yield dut.rx_reset))

        run_simulation(dut, generator(dut))
        return trace

    def test_u0_traffic_is_not_warm_reset(self):
        # SuperSpeed data: non electrical idle line, valid symbols (no decode errors), 200ms.
        trace = self.run_line([(1, 1, 0), (200, 0, 0)])
        self.assertEqual(sum(trace), 0)

    def test_warm_reset_detected_within_treset_delay(self):
        # Warm Reset LFPS (tBurst 80-120ms): declared within tResetDelay (18-50ms).
        trace = self.run_line([(1, 1, 0), (100, 0, 1)])
        detect_ms = (trace.index(1) - ms_to_cycles(1))/ms_to_cycles(1)
        self.assertGreaterEqual(detect_ms, 18)
        self.assertLessEqual(detect_ms, 50)

    def test_u3_wakeup_is_not_warm_reset(self):
        # Longest U3 Wakeup LFPS burst (10ms, Table 6-30) is not a Warm Reset.
        trace = self.run_line([(1, 1, 0), (10, 0, 1), (5, 1, 0)])
        self.assertEqual(sum(trace), 0)

    def test_warm_reset_released_when_lfps_stops(self):
        trace = self.run_line([(1, 1, 0), (100, 0, 1), (1, 1, 0)])
        self.assertEqual(trace[-1], 0)

# RX Error Monitor ---------------------------------------------------------------------------------

class TestRXErrorMonitor(unittest.TestCase):
    def test_rx_error_monitor(self):
        decoders = [DecoderStub(), DecoderStub()]
        dut      = RXErrorMonitor(decoders, window=16)

        def generator(dut):
            # No errors.
            for _ in range(32):
                yield
            self.assertEqual((yield dut.errors), 0)
            # Errors every 8 cycles (< window): errors stays set.
            for i in range(128):
                yield decoders[i % 2].invalid.eq(i % 8 == 0)
                yield
                if i > 8:
                    self.assertEqual((yield dut.errors), 1)
            yield decoders[0].invalid.eq(0)
            yield decoders[1].invalid.eq(0)
            # No more errors: errors cleared after the window.
            for _ in range(32):
                yield
            self.assertEqual((yield dut.errors), 0)

        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
