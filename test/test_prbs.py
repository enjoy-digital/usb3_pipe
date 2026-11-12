#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from types import SimpleNamespace

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.prbs import PRBSLinkTester

# Fake PHY (loopback) ------------------------------------------------------------------------------

class LoopbackPHY(Module):
    """PHY stand-in: link words sent on sink are received on source (2 register stages), errors
    injected with corrupt (data words XORed)."""
    def __init__(self, data_width):
        layout = [("data", data_width), ("ctrl", data_width//8)]
        self.u0      = Signal()
        self.corrupt = Signal()
        pipes = [stream.PipeValid(layout) for _ in range(2)]
        self.submodules += pipes
        self.sink   = pipes[0].sink
        self.source = stream.Endpoint(layout)
        self.comb += [
            pipes[0].source.connect(pipes[1].sink),
            pipes[1].source.connect(self.source),
            If(self.corrupt & (pipes[1].source.ctrl == 0),
                self.source.data.eq(pipes[1].source.data ^ 1)
            )
        ]

class LanesLoopbackPHY(Module):
    """Gen 1x2 PHY stand-in: 64-bit link words (x2) and the 32-bit Lane 0 interface (x1)."""
    def __init__(self):
        self.u0 = Signal()
        self.x2 = Signal()
        x2 = LoopbackPHY(64)
        x1 = LoopbackPHY(32)
        self.submodules += x2, x1
        self.sink,    self.source    = x2.sink, x2.source
        self.sink_x1, self.source_x1 = x1.sink, x1.source

class DUT(Module):
    def __init__(self, data_width):
        self.submodules.phy    = phy    = LoopbackPHY(data_width)
        self.submodules.tester = tester = PRBSLinkTester(phy, data_width=data_width, gap_bits=2)

# Test ---------------------------------------------------------------------------------------------

class TestPRBSLinkTester(unittest.TestCase):
    def run_tester(self, data_width):
        dut = DUT(data_width)
        t   = dut.tester
        def generator():
            # Not in U0: no packets.
            for _ in range(64):
                yield
            self.assertEqual((yield t._tx_words.status), 0)

            # U0: packets sent/checked without errors.
            yield dut.phy.u0.eq(1)
            for _ in range(2000):
                yield
            tx_words = (yield t._tx_words.status)
            rx_words = (yield t._rx_words.status)
            self.assertGreater(rx_words, 1000)
            self.assertLessEqual(tx_words - rx_words, 32) # In flight.
            self.assertGreater((yield t._rx_packets.status), 10)
            self.assertEqual((yield t._rx_errors.status), 0)

            # Corrupted data word: errors (bounded, self-synchronizing checker).
            yield dut.phy.corrupt.eq(1)
            yield
            yield dut.phy.corrupt.eq(0)
            for _ in range(200):
                yield
            errors = (yield t._rx_errors.status)
            self.assertGreaterEqual(errors, 1)
            self.assertLessEqual(errors, 2)

            # Disabled: no new packets (CSR fields: the CSRs are not simulated).
            yield t._control.fields.enable.eq(0)
            for _ in range(64):
                yield
            tx_words = (yield t._tx_words.status)
            for _ in range(200):
                yield
            self.assertEqual((yield t._tx_words.status), tx_words)

        run_simulation(dut, generator())

    def test_lanes_x2_x1(self):
        # Gen 1x2 PHY: counters of the negotiated mode (64-bit link words with x2, Lane 0 32-bit
        # words with Gen 1x1), only the active interface used.
        phy    = LanesLoopbackPHY()
        tester = PRBSLinkTester(phy, data_width=64, gap_bits=2)
        dut    = Module()
        dut.submodules += phy, tester
        def generator():
            for x2 in [1, 0, 1]:
                # Mode negotiated outside U0 (packets in progress completed, loopback drained).
                yield phy.u0.eq(0)
                for _ in range(128):
                    yield
                yield phy.x2.eq(x2)
                yield phy.u0.eq(1)
                yield
                start = (yield tester._rx_words.status)
                for _ in range(1000):
                    self.assertFalse((yield phy.source_x1.valid) if x2 else (yield phy.source.valid))
                    yield
                self.assertGreater((yield tester._rx_words.status) - start, 400)
                self.assertEqual((yield tester._rx_errors.status), 0)
        run_simulation(dut, generator())

    def test_32bit(self):
        self.run_tester(32)

    def test_64bit(self):
        self.run_tester(64)

if __name__ == "__main__":
    unittest.main()
