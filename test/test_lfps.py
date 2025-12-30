#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe import lfps

class TestLFPS(unittest.TestCase):
    def test_lfps_burst_generator(self):
        def burst_generator(dut, nbursts, burst_length):
            for i in range(nbursts):
                yield dut.length.eq(burst_length)
                yield dut.start.eq(1)
                yield
                yield dut.start.eq(0)
                while not (yield dut.done):
                    yield
                for i in range(256):
                    yield
            dut.run = False

        def burst_clk_checker(dut, sys_clk_freq, lfps_clk_freq):
            transitions  = 0
            ones_ticks   = 0
            zeroes_ticks = 0
            tx_pattern   = 0
            while dut.run:
                if not (yield dut.tx_idle):
                    if (yield dut.tx_pattern != tx_pattern):
                        transitions += 1
                    if (yield dut.tx_pattern != 0):
                        ones_ticks   += 1
                    else:
                        zeroes_ticks += 1
                    tx_pattern = (yield dut.tx_pattern)
                yield
            total_ticks = ones_ticks + zeroes_ticks
            # Check burst clk duty cycle (less than 10% variation)
            self.assertEqual(abs(ones_ticks  /(total_ticks) - 50e-2) < 10e-2, True)
            self.assertEqual(abs(zeroes_ticks/(total_ticks) - 50e-2) < 10e-2, True)
            # Check burst clk cycles (less than 10% error)
            expected_cycles = sys_clk_freq/lfps_clk_freq
            computed_cycles = 2*total_ticks/transitions
            self.assertEqual(abs(expected_cycles/computed_cycles - 1.0) < 10e-2, True)

        def burst_length_checker(dut, burst_length):
            transitions  = 0
            ticks        = 0
            tx_idle      = 0
            while dut.run:
                if (yield dut.tx_idle) != tx_idle:
                    transitions += 1
                if not (yield dut.tx_idle):
                    ticks += 1
                tx_idle = (yield dut.tx_idle)
                yield
            # Check burst length (less than 20% error)
            self.assertEqual(abs((2*ticks/transitions)/burst_length - 1.0) < 20e-2, True)

        sys_clk_freq  = 100e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSBurstGenerator(sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            burst_generator(dut, nbursts=8, burst_length=256),
            burst_clk_checker(dut, sys_clk_freq=sys_clk_freq, lfps_clk_freq=lfps_clk_freq),
            burst_length_checker(dut, burst_length=256)
        ]
        run_simulation(dut, generators)


    def test_lfps_burst_generator_short_long(self):
        def burst_generator(dut):
            lengths = [8, 16, 32, 64, 128, 256, 512]
            for i in range(len(lengths)):
                yield dut.length.eq(lengths[i])
                yield dut.start.eq(1)
                yield
                yield dut.start.eq(0)
                while not (yield dut.done):
                    yield
                for j in range(64):
                    yield
            dut.run = False

        def burst_length_checker(dut):
            in_burst     = 0
            ticks        = 0
            expected     = 0
            errors       = 0
            while dut.run:
                if not (yield dut.tx_idle):
                    if in_burst == 0:
                        in_burst = 1
                        ticks = 0
                        expected = (yield dut.length)
                    ticks += 1
                else:
                    if in_burst == 1:
                        in_burst = 0
                        # Check burst length (less than 20% error)
                        if expected != 0:
                            if not (abs(ticks/expected - 1.0) < 20e-2):
                                errors += 1
                yield
            self.assertEqual(errors, 0)

        sys_clk_freq  = 100e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSBurstGenerator(sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            burst_generator(dut),
            burst_length_checker(dut),
        ]
        run_simulation(dut, generators)


    def test_lfps_burst_generator_ignored_while_busy(self):
        def burst_generator(dut, burst_length):
            yield dut.length.eq(burst_length)
            yield dut.start.eq(1)
            yield
            yield dut.start.eq(0)
            for i in range(16):
                yield dut.start.eq(1) # Try to retrigger while busy.
                yield
            yield dut.start.eq(0)
            while not (yield dut.done):
                yield
            for i in range(256):
                yield
            dut.run = False

        def burst_count_checker(dut):
            bursts      = 0
            tx_idle     = 1
            while dut.run:
                if not (yield dut.tx_idle) and tx_idle:
                    bursts += 1
                tx_idle = (yield dut.tx_idle)
                yield
            # Ensure only one burst happened.
            self.assertEqual(bursts, 1)

        sys_clk_freq  = 100e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSBurstGenerator(sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            burst_generator(dut, burst_length=256),
            burst_count_checker(dut),
        ]
        run_simulation(dut, generators)


    def test_lfps_generator(self):
        def lfps_generator(dut):
            yield dut.generate.eq(1)
            for i in range(int(1e4)):
                yield
            dut.run = False

        def lfps_checker(dut, burst_length, burst_repeat):
            bursts      = 0
            burst_ticks = 0
            total_ticks = 0
            tx_idle     = 0
            while dut.run:
                if not (yield dut.tx_idle) and tx_idle:
                    bursts += 1
                if not (yield dut.tx_idle):
                    burst_ticks += 1
                total_ticks += 1
                tx_idle = (yield dut.tx_idle)
                yield
            # Check burst length (less than 10% error)
            self.assertEqual(abs((burst_ticks/bursts)/burst_length - 1.0) < 10e-2, True)
            # Check burst repeat (less than 10% error)
            self.assertEqual(abs((total_ticks/bursts)/burst_repeat - 1.0) < 10e-2, True)

        sys_clk_freq  = 100e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSGenerator(lfps.PollingLFPS, sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            lfps_generator(dut),
            lfps_checker(dut,
                burst_length=lfps.time_to_cycles(sys_clk_freq, lfps.PollingLFPSBurst.t_typ),
                burst_repeat=lfps.time_to_cycles(sys_clk_freq, lfps.PollingLFPSRepeat.t_typ)),
        ]
        run_simulation(dut, generators)


    def test_lfps_generator_disable_mid_run(self):
        def lfps_generator(dut):
            yield dut.generate.eq(1)
            for i in range(int(2000)):
                yield
            yield dut.generate.eq(0)
            for i in range(int(2000)):
                yield
            dut.run = False

        def lfps_burst_counter(dut):
            bursts_a    = 0
            bursts_b    = 0
            tx_idle     = 0
            ticks       = 0
            while dut.run:
                if not (yield dut.tx_idle) and tx_idle:
                    if ticks < 2000:
                        bursts_a += 1
                    else:
                        bursts_b += 1
                tx_idle = (yield dut.tx_idle)
                ticks += 1
                yield
            self.assertEqual(bursts_a > 0, True)
            self.assertEqual(bursts_b <= 1, True)

        sys_clk_freq  = 100e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSGenerator(lfps.PollingLFPS, sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            lfps_generator(dut),
            lfps_burst_counter(dut),
        ]
        run_simulation(dut, generators)


    def test_lfps_generator_different_sys_clk(self):
        def lfps_generator(dut):
            yield dut.generate.eq(1)
            for i in range(int(2e4)):
                yield
            dut.run = False

        def lfps_checker(dut, burst_length, burst_repeat):
            bursts      = 0
            burst_ticks = 0
            total_ticks = 0
            tx_idle     = 0
            while dut.run:
                if not (yield dut.tx_idle) and tx_idle:
                    bursts += 1
                if not (yield dut.tx_idle):
                    burst_ticks += 1
                total_ticks += 1
                tx_idle = (yield dut.tx_idle)
                yield
            self.assertEqual(bursts > 0, True)
            # Check burst length (less than 10% error)
            self.assertEqual(abs((burst_ticks/bursts)/burst_length - 1.0) < 10e-2, True)
            # Check burst repeat (less than 10% error)
            self.assertEqual(abs((total_ticks/bursts)/burst_repeat - 1.0) < 10e-2, True)

        sys_clk_freq  = 125e6
        lfps_clk_freq = 25e6

        dut = lfps.LFPSGenerator(lfps.PollingLFPS, int(sys_clk_freq), lfps_clk_freq)
        dut.run = True
        generators = [
            lfps_generator(dut),
            lfps_checker(dut,
                burst_length=lfps.time_to_cycles(sys_clk_freq, lfps.PollingLFPSBurst.t_typ),
                burst_repeat=lfps.time_to_cycles(sys_clk_freq, lfps.PollingLFPSRepeat.t_typ)),
        ]
        run_simulation(dut, generators)
