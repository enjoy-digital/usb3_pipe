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

    def test_32bit(self):
        self.run_tester(32)

    def test_64bit(self):
        self.run_tester(64)

if __name__ == "__main__":
    unittest.main()
