# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe import lfps

class TestLFPS(unittest.TestCase):
    def test_lfps_transmitter(self):
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

        dut = lfps.LFPSBurstTransmitter(sys_clk_freq, lfps_clk_freq)
        dut.run = True
        generators = [
            burst_generator(dut, nbursts=8, burst_length=256),
            burst_clk_checker(dut, sys_clk_freq=sys_clk_freq, lfps_clk_freq=lfps_clk_freq),
            burst_length_checker(dut, burst_length=256)
        ]
        run_simulation(dut, generators, vcd_name="lfps.vcd")

