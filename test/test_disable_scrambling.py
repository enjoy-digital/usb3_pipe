#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.common import TS2, LinkConfig
from usb3_pipe.core import USB3PIPE
from usb3_pipe.phy import USB3PHY

from test.common   import ordered_set_words
from test.model.gt import GTModel, wire_monitor

# Helpers ------------------------------------------------------------------------------------------

DISABLE_SCRAMBLING = LinkConfig(reset=0, loopback=0, scrambling=0)
NORMAL             = LinkConfig()

# Disable Scrambling (USB 3.2 6.4.1.1.2 Table 6-6, 7.5.4.10.1, 7.5.10.5.1) -------------------------

class TestDisableScrambling(unittest.TestCase):
    def run_pipe(self, ts2_link_config, disable_scrambling=0):
        """Receive TS2s, then (Polling.Idle) Idle, then U0 data, all unscrambled. Return the received
        data and the transmitted wire words."""
        rng  = random.Random(0)
        data = [(rng.randrange(1, 2**32), 0) for _ in range(32)]
        rx   = ordered_set_words(TS2, link_config=ts2_link_config)*8 + [(0, 0)]*8 + data

        gt  = GTModel()
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt
        n_ts2 = len(ordered_set_words(TS2))*8

        def generator(dut):
            dut.received = []
            yield dut.disable_scrambling.eq(disable_scrambling)
            yield dut.source.ready.eq(1)
            for i, (d, c) in enumerate(rx):
                # Polling.Idle after the TS2s, then U0.
                yield dut.ts2_decode.eq(n_ts2 <= i < n_ts2 + 8)
                yield dut.idle_handshake.eq(n_ts2 <= i < n_ts2 + 8)
                yield dut.tx_ready.eq(i >= n_ts2 + 8)
                yield dut.rx_ready.eq(i >= n_ts2 + 8)
                yield gt.source.valid.eq(1)
                yield gt.source.data.eq(d)
                yield gt.source.ctrl.eq(c)
                yield
                if (yield dut.source.valid):
                    dut.received.append(((yield dut.source.data), (yield dut.source.ctrl)))
            yield gt.source.valid.eq(0)

        def checker(dut):
            for _ in range(n_ts2):
                yield
            dut.wire = yield from wire_monitor(gt, 16)

        run_simulation(dut, [generator(dut), checker(dut)])
        return data, dut.received, dut.wire

    def test_disable_scrambling_bit_received(self):
        # The port shall disable the scrambling if the Disabling Scrambling bit is asserted in the
        # TS2 received in Polling.Configuration (7.5.4.10.1): the partner then sends unscrambled
        # Idle Symbols and data, and so do we.
        data, received, wire = self.run_pipe(DISABLE_SCRAMBLING)
        received = [w for w in received if w != (0, 0)] # Remove Idle Symbols.
        self.assertEqual(received[:len(data) - 2], data[:len(data) - 2])
        self.assertEqual(set(wire), {(0, 0)}) # Unscrambled Idle Symbols.

    def test_scrambling_enabled_by_default(self):
        data, received, wire = self.run_pipe(NORMAL)
        self.assertNotEqual(received[:len(data) - 2], data[:len(data) - 2])
        self.assertNotEqual(set(wire), {(0, 0)})

    def test_directed_disable_scrambling(self):
        # When directed, the port shall set the Disabling Scrambling bit in the TS2 (7.5.4.9.1) and
        # disable the scrambling.
        gt  = GTModel()
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt
        def generator(dut):
            dut.link_config = []
            yield dut.disable_scrambling.eq(1)
            yield dut.ts_tx_enable.eq(1)
            yield dut.ts_tx_ts2.eq(1)
            for _ in range(4*8):
                yield
                if (yield gt.sink.valid) and (yield gt.sink.ctrl) == 0b1111:
                    yield
                    dut.link_config.append(((yield gt.sink.data) >> 8) & 0xff)
        run_simulation(dut, generator(dut))
        self.assertEqual(set(dut.link_config), {DISABLE_SCRAMBLING})
        data, received, wire = self.run_pipe(NORMAL, disable_scrambling=1)
        self.assertEqual(set(wire), {(0, 0)})

    def test_phy_connects_ts2_decode(self):
        dut  = USB3PHY(serdes=GTModel(), sys_clk_freq=int(125e6), with_endianness_swap=False)
        comb = dut.get_fragment().comb
        self.assertTrue(any(getattr(s, "l", None) is dut.pipe.ts2_decode and s.r is dut.ltssm.ts2_decode
            for s in comb))

if __name__ == "__main__":
    unittest.main()
