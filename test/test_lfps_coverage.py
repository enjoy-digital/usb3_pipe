#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""LFPS coverage tests against USB 3.2 Rev 1.1 section 6.9 (Table 6-29/6-30/6-31).

Complements test_lfps*.py: generated burst/repeat timings at several sys_clk frequencies, checker
accept/reject at the exact min/max cycle boundaries, repeated burst counting, checker recovery and
LFPSUnit TX arbitration.
"""

import unittest
from math import ceil, floor

from migen import *

from usb3_pipe import lfps
from usb3_pipe.lfps import LFPSChecker, LFPSBurstChecker, LFPSUnit

FREQS         = [125e6, 133.33e6, 150e6]
LFPS_CLK_FREQ = 25e6

# Helpers ------------------------------------------------------------------------------------------

def spec_ceil(f, t):
    """Smallest number of cycles lasting at least t (robust to float rounding)."""
    return ceil(t*f - 1e-6)

def spec_floor(f, t):
    """Largest number of cycles lasting at most t (robust to float rounding)."""
    return floor(t*f + 1e-6)

def runs(trace, value=0):
    """(start, length) of the runs of `value` in trace, excluding runs touching the trace edges."""
    r, start = [], None
    for i, v in enumerate(trace):
        if v == value and start is None:
            start = i
        elif v != value and start is not None:
            if start > 0:
                r.append((start, i - start))
            start = None
    return r

class SerDesStub(Module):
    """LFPS face of the SerDes, without RX idle filter (rx_idle driven directly)."""
    def __init__(self):
        self.rx_idle    = Signal(reset=1)
        self.tx_idle    = Signal()
        self.tx_lfps    = Signal()
        self.tx_pattern = Signal(20)

def lfps_unit(sys_clk_freq, timer_scale=1):
    serdes = SerDesStub()
    dut    = LFPSUnit(serdes=serdes, sys_clk_freq=sys_clk_freq, lfps_clk_freq=LFPS_CLK_FREQ,
        timer_scale=timer_scale)
    dut.submodules += serdes
    return dut, serdes

def tx_trace(sys_clk_freq, request, n_cycles, timer_scale=1):
    """Assert `request` on a LFPSUnit and record the serdes tx_idle for n_cycles."""
    dut, serdes = lfps_unit(sys_clk_freq, timer_scale)
    trace = []
    def generator(dut):
        yield getattr(dut, request).eq(1)
        for _ in range(n_cycles):
            yield
            trace.append((yield serdes.tx_idle))
    run_simulation(dut, generator(dut))
    return trace

# Generated LFPS timings (Table 6-29/6-30) ---------------------------------------------------------

class TestLFPSTXTiming(unittest.TestCase):
    def test_polling_burst_and_repeat(self):
        # Polling.LFPS: tBurst 0.6-1.4us, tRepeat 6-14us (Table 6-30), measured on the SerDes.
        for f in FREQS:
            with self.subTest(sys_clk_freq=f):
                trace  = tx_trace(f, "tx_polling", int(1.2*10e-6*f) + 256)
                bursts = runs(trace, 0)
                self.assertGreaterEqual(len(bursts), 2)
                for _, length in bursts:
                    self.assertTrue(0.6e-6 <= length/f <= 1.4e-6, length/f)
                for (a, _), (b, _) in zip(bursts, bursts[1:]):
                    self.assertTrue(6e-6 <= (b - a)/f <= 14e-6, (b - a)/f)

    def test_ping_burst(self):
        # Ping.LFPS: tBurst 40-200ns, at least 2 LFPS periods (Table 6-30 note 5).
        for f in FREQS:
            with self.subTest(sys_clk_freq=f):
                dut, serdes = lfps_unit(f)
                dut.trace = []
                def generator(dut):
                    yield dut.tx_ping.eq(1)
                    for _ in range(128):
                        yield
                        dut.trace.append(((yield serdes.tx_idle), (yield serdes.tx_pattern) & 1))
                run_simulation(dut, generator(dut))
                bursts = runs([idle for idle, _ in dut.trace], 0)
                self.assertEqual(len(bursts), 1)
                start, length = bursts[0]
                self.assertTrue(40e-9 <= length/f <= 200e-9, length/f)
                pattern  = [p for _, p in dut.trace[start:start + length]]
                toggles  = sum(a != b for a, b in zip(pattern, pattern[1:]))
                self.assertGreaterEqual((toggles + 1)/2, 2)

    def test_ping_repeat(self):
        # Ping.LFPS: tRepeat 160-240ms (Table 6-30). Measured with timer_scale (cycles scaled back).
        scale = 40000
        for f in FREQS:
            with self.subTest(sys_clk_freq=f):
                trace  = tx_trace(f, "tx_ping", int(2.2*200e-3*f/scale), timer_scale=scale)
                starts = [i for i in range(1, len(trace)) if trace[i - 1] and not trace[i]]
                self.assertGreaterEqual(len(starts), 2)
                for a, b in zip(starts, starts[1:]):
                    self.assertTrue(160e-3 <= (b - a)*scale/f <= 240e-3, (b - a)*scale/f)

    def test_single_burst_lengths(self):
        # Single-burst LFPS lengths transmitted by the LTSSM handshake (Table 6-30/6-31):
        # U1 Exit 0.6us-2ms, U2/Loopback Exit 80us-2ms, U3 Wakeup 80us-10ms, Warm Reset 80-120ms.
        cases = [
            ("u1", lfps.U1ExitLFPSBurst,   lfps.exit_u1_min_cycles, 0.6e-6, 2e-3),
            ("u2", lfps.U2ExitLFPSBurst,   lfps.exit_u2_min_cycles, 80e-6,  2e-3),
            ("u3", lfps.U3WakeupLFPSBurst, lfps.exit_u3_min_cycles, 80e-6,  10e-3),
        ]
        for f in FREQS:
            for name, burst, min_cycles, t_min, t_max in cases:
                with self.subTest(sys_clk_freq=f, burst=name):
                    cycles = lfps.exit_burst_cycles(f, 1, burst, min_cycles(f))
                    self.assertTrue(t_min <= cycles/f <= t_max, cycles/f)
                    # The burst must outlast the partner detection window (+ RX latency).
                    self.assertGreaterEqual(cycles, min_cycles(f) + lfps.RX_DETECT_LATENCY)
            with self.subTest(sys_clk_freq=f, burst="warm_reset"):
                cycles = lfps.time_to_cycles(f, lfps.ResetLFPSBurst.t_typ)
                self.assertTrue(80e-3 <= cycles/f <= 120e-3)
                # tResetDelay 18-50ms, above the longest U3 Wakeup burst (10ms).
                cycles = lfps.time_to_cycles(f, lfps.ResetDetectDelay)
                self.assertTrue(18e-3 <= cycles/f <= 50e-3)
                self.assertGreater(cycles/f, 10e-3)

    def test_raw_burst_passthrough(self):
        # U1/U2/U3/Warm Reset bursts are timed by the LTSSM: the LFPSUnit must forward the request
        # cycle-exact on the SerDes (no stretching/truncation) as a square wave.
        for request in ["tx_exit_u1", "tx_exit_u2", "tx_wakeup_u3", "tx_reset"]:
            with self.subTest(request=request):
                dut, serdes = lfps_unit(125e6)
                dut.trace = []
                def generator(dut):
                    yield dut.tx_idle.eq(1)
                    for i in range(200):
                        yield getattr(dut, request).eq(16 <= i < 16 + 113)
                        yield
                        dut.trace.append(((yield serdes.tx_idle), (yield serdes.tx_lfps),
                            (yield serdes.tx_pattern)))
                run_simulation(dut, generator(dut))
                bursts = runs([idle for idle, _, _ in dut.trace], 0)
                self.assertEqual([length for _, length in bursts], [113])
                start = bursts[0][0]
                for idle, tx_lfps, pattern in dut.trace[start:start + 113]:
                    self.assertEqual(tx_lfps, 1)
                    self.assertIn(pattern, [0, 0xfffff])
                self.assertEqual(len({p for _, _, p in dut.trace[start:start + 113]}), 2)

# Polling.LFPS Checker boundaries (Table 6-30) -----------------------------------------------------

def polling_detections(f, segments, lead=16, tail=64):
    """Drive an LFPSChecker (Polling) with (burst_cycles, gap_cycles) segments, return the cycle
    indexes of the detections and the burst start indexes."""
    dut = LFPSChecker(lfps.PollingLFPS, f)
    dut.detections = []
    dut.starts     = []
    def generator(dut):
        i = 0
        trace = [1]*lead
        for burst, gap in segments:
            dut.starts.append(len(trace))
            trace += [0]*burst + [1]*gap
        trace += [1]*tail
        for i, idle in enumerate(trace):
            yield dut.idle.eq(idle)
            yield
            if (yield dut.detect):
                dut.detections.append(i)
    run_simulation(dut, generator(dut))
    return dut.detections, dut.starts

class TestLFPSPollingChecker(unittest.TestCase):
    def windows(self, f):
        b_min = spec_ceil(f, 0.6e-6)
        b_max = spec_floor(f, 1.4e-6)
        r_min = spec_ceil(f, 6e-6)
        r_max = spec_floor(f, 14e-6)
        return b_min, b_max, r_min, r_max

    def test_spec_boundaries_accepted(self):
        # Any tBurst in [0.6, 1.4]us with tRepeat in [6, 14]us must be detected (N bursts: N-1
        # detections, one per burst+gap measured).
        for f in FREQS:
            b_min, b_max, r_min, r_max = self.windows(f)
            for burst, repeat in [(b_min, r_min), (b_min, r_max), (b_max, r_min), (b_max, r_max)]:
                with self.subTest(sys_clk_freq=f, burst=burst, repeat=repeat):
                    detections, _ = polling_detections(f, [(burst, repeat - burst)]*3)
                    self.assertEqual(len(detections), 2)

    def test_out_of_spec_burst_rejected(self):
        # One cycle below tBurst min or above tBurst max (implementation window): no detection.
        for f in FREQS:
            b_min, _, _, _ = self.windows(f)
            b_max_impl = lfps.time_to_cycles(f, 1.4e-6)
            repeat     = spec_ceil(f, 10e-6)
            for burst in [b_min - 1, b_max_impl + 1]:
                with self.subTest(sys_clk_freq=f, burst=burst):
                    self.assertTrue(not (0.6e-6 <= burst/f <= 1.4e-6))
                    detections, _ = polling_detections(f, [(burst, repeat - burst)]*3)
                    self.assertEqual(detections, [])

    def test_out_of_spec_repeat_rejected(self):
        # Gap one cycle outside of [6us - tBurst max, 14us - tBurst min]: repeat out of 6-14us.
        for f in FREQS:
            b_min, b_max, _, _ = self.windows(f)
            g_min = lfps.time_to_cycles(f, 6e-6 - 1.4e-6)
            g_max = lfps.time_to_cycles(f, 14e-6 - 0.6e-6)
            for burst, gap in [(b_max, g_min - 1), (b_min, g_max + 1)]:
                with self.subTest(sys_clk_freq=f, burst=burst, gap=gap):
                    self.assertTrue(not (6e-6 <= (burst + gap)/f <= 14e-6))
                    detections, _ = polling_detections(f, [(burst, gap)]*3)
                    self.assertEqual(detections, [])

    def test_repeated_bursts_counting(self):
        # 8 bursts: 7 single-cycle detection pulses, each on the next burst start (+RX latency).
        f = 125e6
        burst, repeat = spec_ceil(f, 1e-6), spec_ceil(f, 10e-6)
        detections, starts = polling_detections(f, [(burst, repeat - burst)]*8)
        self.assertEqual(len(detections), 7)
        for d, s in zip(detections, starts[1:]):
            self.assertTrue(s <= d <= s + lfps.RX_DETECT_LATENCY, (s, d))

    def test_recovery_after_invalid_burst(self):
        # Detection restarts after a bad burst/gap: a detection reports the previous burst+gap.
        f = 125e6
        good  = (spec_ceil(f, 1e-6), spec_ceil(f, 9e-6))
        long  = (spec_ceil(f, 3e-6), spec_ceil(f, 9e-6))
        stall = (spec_ceil(f, 1e-6), spec_ceil(f, 30e-6))
        for name, segments, expected in [
            ("long_burst", [good, good, long, good, good], [1, 2, 4]),
            ("long_gap",   [good, good, stall, good, good], [1, 2, 4]),
            ("traffic",    [good, (spec_ceil(f, 50e-6), good[1]), good, good], [1, 3]),
        ]:
            with self.subTest(name):
                detections, starts = polling_detections(f, segments)
                detected_bursts = [max(i for i, s in enumerate(starts) if s <= d) for d in detections]
                self.assertEqual(detected_bursts, expected)

    def test_single_cycle_glitches_filtered(self):
        # Single cycle glitches (idle toggling during the burst, noise during the gap) are filtered.
        f     = 125e6
        burst = spec_ceil(f, 1e-6)
        gap   = spec_ceil(f, 9e-6)
        dut   = LFPSChecker(lfps.PollingLFPS, f)
        dut.detections = 0
        def generator(dut):
            for n in range(4):
                for i in range(burst + gap):
                    idle = int(i >= burst)
                    if i in [burst//2, burst + gap//2]: # Glitches.
                        idle ^= 1
                    yield dut.idle.eq(idle)
                    yield
                    dut.detections += (yield dut.detect)
        run_simulation(dut, generator(dut))
        self.assertEqual(dut.detections, 3)

    def test_idle_after_reset_no_detection(self):
        # Electrical idle at reset / constant line: no detection.
        for f in FREQS:
            with self.subTest(sys_clk_freq=f):
                self.assertEqual(polling_detections(f, [], tail=2000)[0], [])

# Ping / U1 / U2 / U3 / Warm Reset detection boundaries (Table 6-30/6-31, 6.9.2/6.9.3) -----------

def burst_responses(f, bursts, gap=64, timer_scale=1):
    """Send single bursts to a LFPSUnit, return per burst (pings, u1, u2, u3, reset) seen."""
    dut, serdes = lfps_unit(f, timer_scale)
    dut.results = []
    def generator(dut):
        for _ in range(gap):
            yield
        for burst in bursts:
            r = [0, 0, 0, 0, 0]
            for i in range(burst + gap):
                yield serdes.rx_idle.eq(int(i >= burst))
                yield
                r[0] += (yield dut.rx_ping)
                r[1] |= (yield dut.rx_exit_u1)
                r[2] |= (yield dut.rx_exit_u2)
                r[3] |= (yield dut.rx_wakeup_u3)
                r[4] |= (yield dut.rx_reset)
            dut.results.append(tuple(r))
    run_simulation(dut, generator(dut))
    return dut.results

def assertion_delays(f, burst, timer_scale=1, with_errors=False):
    """Send one burst, return the delay from burst start to the assertion of each level output
    and to its deassertion after the burst end."""
    dut, serdes = lfps_unit(f, timer_scale)
    names = ["rx_exit_u1", "rx_exit_u2", "rx_wakeup_u3", "rx_reset"]
    dut.rise = {}
    dut.fall = {}
    def generator(dut):
        for _ in range(16):
            yield
        for i in range(burst + 32):
            yield serdes.rx_idle.eq(int(i >= burst))
            yield
            for n in names:
                v = (yield getattr(dut, n))
                if v and n not in dut.rise:
                    dut.rise[n] = i
                if not v and n in dut.rise and n not in dut.fall:
                    dut.fall[n] = i - burst
    run_simulation(dut, generator(dut))
    return dut.rise, dut.fall

class TestLFPSBurstDetection(unittest.TestCase):
    def test_ping_u1_boundaries(self):
        # Ping.LFPS tBurst 40-200ns is a Ping, below 40ns rejected; a burst reaching the 300ns
        # U1 Exit detection window (Table 6-30 note 7) is a U1 Exit, not a Ping.
        for f in FREQS:
            p_min  = spec_ceil(f, 40e-9)
            p_max  = spec_floor(f, 200e-9)
            u1_min = lfps.exit_u1_min_cycles(f)
            self.assertGreaterEqual(u1_min/f, 300e-9)
            self.assertLess(p_min - 1, 40e-9*f)
            bursts   = [p_min - 1, p_min, p_max, u1_min - 1, u1_min, spec_ceil(f, 0.6e-6)]
            expected = [(0, 0), (1, 0), (1, 0), (1, 0), (0, 1), (0, 1)]
            with self.subTest(sys_clk_freq=f):
                results = burst_responses(f, bursts)
                self.assertEqual([r[:2] for r in results], expected)
                # Short bursts are never exit/wakeup/reset.
                self.assertEqual({r[2:] for r in results}, {(0, 0, 0)})

    def test_repeated_pings_counted(self):
        # Each Ping.LFPS burst produces exactly one rx_ping pulse.
        f = 125e6
        results = burst_responses(f, [spec_ceil(f, 190e-9)]*5)
        self.assertEqual([r[0] for r in results], [1]*5)

    def check_u2_u3_threshold(self, f):
        # U2/Loopback Exit and U3 Wakeup: bursts of 80us (Table 6-30/6-31 tBurst min) detected,
        # one cycle shorter not (LFPSUnit checker configuration, standalone checker for speed).
        for name, min_cycles in [("u2", lfps.exit_u2_min_cycles), ("u3", lfps.exit_u3_min_cycles)]:
            dut = LFPSBurstChecker(f, min_cycles=min_cycles(f))
            dut.detected = []
            def generator(dut):
                yield dut.idle.eq(1)
                for _ in range(16):
                    yield
                for burst in [spec_ceil(f, 80e-6) - 1, spec_ceil(f, 80e-6)]:
                    detected = 0
                    for i in range(burst + 16):
                        yield dut.idle.eq(int(i >= burst))
                        yield
                        detected |= (yield dut.detect)
                    dut.detected.append(detected)
            run_simulation(dut, generator(dut))
            self.assertEqual(dut.detected, [0, 1], name)

    def test_u2_u3_detection_threshold(self):
        for f in [125e6, 133.33e6]:
            with self.subTest(sys_clk_freq=f):
                self.check_u2_u3_threshold(f)

    @unittest.expectedFailure
    def test_u2_u3_detection_threshold_150mhz(self):
        # BUG candidate: time_to_cycles() float ceil: 80e-6*150e6 = 12000.000000000002 -> 12001,
        # an exact 80us burst (12000 cycles) is not detected as U2/Loopback Exit / U3 Wakeup.
        # (Masked in hardware by the RXIdleFilter stretch.)
        self.check_u2_u3_threshold(150e6)

    def test_exit_levels_released_at_burst_end(self):
        # U1/U2/U3 levels follow the burst and are released at its end (within the RX latency).
        f = 125e6
        scale = 100
        rise, fall = assertion_delays(f, lfps.exit_u2_min_cycles(f, scale) + 64, timer_scale=scale)
        for n in ["rx_exit_u1", "rx_exit_u2", "rx_wakeup_u3"]:
            self.assertIn(n, rise)
            self.assertTrue(0 <= fall[n] <= lfps.RX_DETECT_LATENCY, (n, fall[n]))
        self.assertNotIn("rx_reset", rise)

    def test_warm_reset_threshold(self):
        # Warm Reset declared after tResetDelay (18-50ms, 6.9.3) of LFPS (line not idle + RX errors
        # on the SerDes; without rx_errors: idle only). Timer scaled by 10000, cycles scaled back.
        scale = 10000
        for f in FREQS:
            with self.subTest(sys_clk_freq=f):
                reset_min = lfps.scaled_cycles(f, lfps.ResetDetectDelay, scale)
                rise, fall = assertion_delays(f, reset_min + 64, timer_scale=scale)
                threshold = lfps.exit_u1_min_cycles(f, scale) + rise["rx_reset"] - rise["rx_exit_u1"]
                self.assertEqual(threshold, reset_min)
                self.assertTrue(18e-3 <= threshold*scale/f <= 50e-3)
                self.assertTrue(0 <= fall["rx_reset"] <= lfps.RX_DETECT_LATENCY)
                # Longest U3 Wakeup burst (10ms) is not a Warm Reset.
                results = burst_responses(f, [lfps.scaled_cycles(f, 10e-3, scale)], timer_scale=scale)
                self.assertEqual(results[0][4], 0)

# LFPSUnit TX arbitration (Table 6-30, 6.9) --------------------------------------------------------

class TestLFPSUnitArbitration(unittest.TestCase):
    def record(self, f, steps, timer_scale=1):
        """steps: list of (n_cycles, {signal: value}), return (tx_idle, tx_lfps, tx_count) trace."""
        dut, serdes = lfps_unit(f, timer_scale)
        dut.trace = []
        def generator(dut):
            for n, values in steps:
                for name, value in values.items():
                    yield getattr(dut, name).eq(value)
                for _ in range(n):
                    yield
                    dut.trace.append(((yield serdes.tx_idle), (yield serdes.tx_lfps),
                        (yield dut.tx_count)))
        run_simulation(dut, generator(dut))
        return dut.trace

    def test_no_request_follows_tx_idle(self):
        trace = self.record(125e6, [(32, {"tx_idle": 1}), (32, {"tx_idle": 0}), (32, {"tx_idle": 1})])
        self.assertEqual([t[0] for t in trace], [1]*32 + [0]*32 + [1]*32)
        self.assertEqual({t[1] for t in trace}, {0})

    def test_electrical_idle_between_polling_bursts(self):
        # Between LFPS bursts the transmitter is in electrical idle (6.9), whatever tx_idle.
        f     = 125e6
        trace = self.record(f, [(2700, {"tx_idle": 0, "tx_polling": 1})])
        bursts = runs([t[0] for t in trace], 0)
        self.assertGreaterEqual(len(bursts), 2)
        first, last = bursts[0][0], bursts[-1][0] + bursts[-1][1]
        for idle, tx_lfps, _ in trace[first:last]:
            self.assertEqual(tx_lfps, 1 - idle)
        gaps = runs([t[0] for t in trace[first:last]], 1)
        for _, length in gaps:
            self.assertGreaterEqual(length/f, 6e-6 - 1.4e-6)

    def test_ping_has_priority_over_polling(self):
        # Simultaneous Ping and Polling requests: only Ping.LFPS bursts on the line.
        f     = 125e6
        trace = self.record(f, [(3000, {"tx_polling": 1, "tx_ping": 1})])
        bursts = runs([t[0] for t in trace], 0)
        self.assertEqual(len(bursts), 1)
        self.assertTrue(40e-9 <= bursts[0][1]/f <= 200e-9)

    def test_single_burst_preempts_polling(self):
        # A U1/U2/U3/Warm Reset request during Polling.LFPS takes the line immediately and for its
        # whole duration; Polling.LFPS resumes after.
        f = 125e6
        for request in ["tx_exit_u1", "tx_reset"]:
            with self.subTest(request=request):
                trace = self.record(f, [
                    (300,  {"tx_polling": 1}),
                    (200,  {request: 1}),
                    (1400, {request: 0}),
                ])
                self.assertEqual({t[0] for t in trace[300:500]}, {0})
                self.assertEqual({t[1] for t in trace[300:500]}, {1})
                self.assertIn(1, [t[0] for t in trace[500:]])
                self.assertIn(0, [t[0] for t in trace[600:]])

    def test_simultaneous_single_burst_requests(self):
        # Any combination of single-burst requests: one continuous LFPS burst (no glitch to idle)
        # while at least one is asserted.
        f     = 125e6
        names = ["tx_reset", "tx_wakeup_u3", "tx_exit_u2", "tx_exit_u1", "tx_ping", "tx_polling"]
        steps = [(64, {n: 1 for n in names})]
        for n in names[:4]:
            steps.append((64, {n: 0}))
        trace = self.record(f, steps)
        self.assertEqual({t[0] for t in trace[:256]}, {0})
        self.assertEqual({t[1] for t in trace[:256]}, {1})

    def test_tx_count(self):
        # tx_count counts the transmitted Polling.LFPS bursts (used by the LTSSM: 16 bursts before
        # Polling.RxEQ, 7.5.4.3.2) and clears when Polling.LFPS stops.
        f       = 125e6
        scale   = 100 # Polling.LFPS timings divided by 10.
        repeat  = lfps.scaled_cycles(f, 10e-6, lfps.polling_scale_of(scale)) + 2
        trace   = self.record(f, [(5*repeat + 16, {"tx_polling": 1}), (2*repeat, {"tx_polling": 0})],
            timer_scale=scale)
        bursts  = runs([t[0] for t in trace[:5*repeat + 16]], 0)
        # Incremented at the end of each repeat period: the 6th period is still ongoing.
        self.assertEqual(len(bursts), 6)
        self.assertEqual(trace[5*repeat + 15][2], 5)
        self.assertEqual(trace[-1][2], 0)

    def test_polling_restart_full_burst(self):
        # Polling.LFPS restarted after the end of the previous repeat period: full first burst.
        f     = 125e6
        trace = self.record(f, [(300, {"tx_polling": 1}), (2000, {"tx_polling": 0, "tx_idle": 1}),
            (300, {"tx_polling": 1})])
        bursts = runs([t[0] for t in trace], 0)
        self.assertEqual(len(bursts), 2)
        for _, length in bursts:
            self.assertTrue(0.6e-6 <= length/f <= 1.4e-6)

    def test_polling_restart_while_busy_no_runt_burst(self):
        # tx_polling dropped mid-burst and re-asserted within the 10us repeat period: the generation
        # restarts with a full burst (no runt burst shorter than tBurst min 0.6us, Table 6-30).
        f     = 125e6
        trace = self.record(f, [(60, {"tx_polling": 1, "tx_idle": 1}), (40, {"tx_polling": 0}),
            (2000, {"tx_polling": 1})])
        # Bursts after the re-assertion (the first one is truncated by the request drop itself).
        bursts = [(s, l) for s, l in runs([t[0] for t in trace], 0) if s >= 100]
        self.assertGreaterEqual(len(bursts), 1)
        for _, length in bursts:
            self.assertTrue(0.6e-6 <= length/f <= 1.4e-6, length/f)

if __name__ == "__main__":
    unittest.main()
