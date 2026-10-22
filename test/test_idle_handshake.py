#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.common import TS2
from usb3_pipe.core import USB3PIPE
from usb3_pipe.ltssm import USB3LTSSM
from usb3_pipe.phy import USB3PHY

from test.common import ScramblerModel, ordered_set_words, words_to_symbols, symbols_to_words
from test import test_ltssm
from test.test_pipe_tx import GTTXModel, wire_monitor

# Helpers ------------------------------------------------------------------------------------------

class GTModel(GTTXModel):
    def __init__(self):
        GTTXModel.__init__(self)
        self.ready = Signal(reset=1)

def partner_wire(n_idle_words, data_words=[]):
    """Partner stream: TS2s, n_idle_words Idle words, then data, scrambled (6.3.1.3, 7.2.3)."""
    partner = ScramblerModel()
    ts2     = partner.process(words_to_symbols(ordered_set_words(TS2)*8), enable=False)
    idle    = partner.process(words_to_symbols([(0, 0)]*n_idle_words))
    data    = partner.process(words_to_symbols(data_words))
    return symbols_to_words(ts2 + idle + data)

# Idle Symbol handshake (USB 3.2 7.5.4.10.2, 7.5.10.4.2, 7.5.12.4.2) -------------------------------

class TestIdleHandshake(unittest.TestCase):
    def pipe_handshake(self, wire, n_cycles=None):
        gt  = GTModel()
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt
        n_cycles = n_cycles or len(wire) + 32

        def generator(dut):
            dut.done_cycle = None
            dut.first_idle = None
            yield dut.idle_handshake.eq(1)
            for i in range(n_cycles):
                d, c = wire[i] if i < len(wire) else (0, 0)
                yield gt.source.valid.eq(i < len(wire))
                yield gt.source.data.eq(d)
                yield gt.source.ctrl.eq(c)
                yield
                if dut.done_cycle is None and (yield dut.idle_done):
                    dut.done_cycle = i

        def checker(dut):
            dut.wire = yield from wire_monitor(gt, n_cycles)

        run_simulation(dut, [generator(dut), checker(dut)])
        return dut

    def test_pipe_transmits_idle_symbols(self):
        # The port in Gen 1 operation shall transmit Idle Symbols if the next state is U0
        # (7.5.4.10.1): the wire must carry scrambled 00h, continuing from the post-TS2 seed.
        dut = self.pipe_handshake(partner_wire(0), n_cycles=64)
        symbols = ScramblerModel(seed=0x7dbd).process(words_to_symbols(dut.wire))
        self.assertTrue(len(dut.wire) >= 32)
        self.assertEqual(set(symbols_to_words(symbols)), {(0, 0)})

    def test_pipe_handshake_done(self):
        # Exit to U0 when 8 consecutive Idle Symbols are received and 16 Idle Symbols are sent
        # after receiving one Idle Symbol (7.5.4.10.2).
        n_ts2 = len(ordered_set_words(TS2))*8
        dut = self.pipe_handshake(partner_wire(16))
        self.assertIsNotNone(dut.done_cycle)
        # At least 2 words (8 symbols) received and 4 words (16 symbols) sent after the first one.
        self.assertGreaterEqual(dut.done_cycle, n_ts2 + 4 - 1)

    def test_pipe_handshake_needs_consecutive_idle(self):
        # Idle Symbols interleaved with data are not 8 consecutive Idle Symbols.
        rng   = random.Random(0)
        data  = [(rng.randrange(1, 2**32), 0) for _ in range(64)]
        wire  = partner_wire(0, sum([[(0, 0), w] for w in data], []))
        dut   = self.pipe_handshake(wire)
        self.assertIsNone(dut.done_cycle)

    def test_pipe_handshake_needs_partner(self):
        # No Idle Symbol received (partner still sending TS2): no exit.
        wire = symbols_to_words(words_to_symbols(ordered_set_words(TS2)*64))
        dut  = self.pipe_handshake(wire)
        self.assertIsNone(dut.done_cycle)

    def test_ltssm_polling_idle_waits_for_handshake(self):
        # Polling.Idle transmits Idle Symbols and only goes to U0 on the Idle handshake.
        t = test_ltssm.TestLTSSM()
        dut = USB3LTSSM(sys_clk_freq=int(125e6), with_timers=False)

        def generator(dut):
            yield dut.rx_idle8.eq(0)
            # Same path as drive_to_u0, stopping in Polling.Idle.
            yield dut.lfps_tx_count.eq(20)
            yield dut.lfps_rx_polling.eq(1)
            yield
            yield dut.lfps_tx_count.eq(24)
            yield
            self.assertTrue((yield from t.wait_for(dut, dut.ts_tx_tseq)))
            for pulse in [dut.ts_tx_done, dut.ts_rx_ts1, dut.ts_tx_done, dut.ts_rx_ts2, dut.ts_tx_done,
                dut.ts_tx_done]:
                yield pulse.eq(1)
                yield
                yield pulse.eq(0)
                yield
            yield dut.lfps_rx_polling.eq(0)
            self.assertTrue((yield from t.wait_for(dut, dut.polling_idle)))
            for _ in range(64):
                self.assertEqual((yield dut.idle_handshake), 1)
                self.assertEqual((yield dut.u0), 0)
                yield
            yield dut.rx_idle8.eq(1)
            self.assertTrue((yield from t.wait_for(dut, dut.u0, timeout=4)))
            self.assertEqual((yield dut.idle_handshake), 0)

        run_simulation(dut, generator(dut))

    def test_phy_connects_idle_handshake(self):
        # USB3PHY must connect the PIPE Idle handshake to the LTSSM (no rx_idle8 proxy).
        dut  = USB3PHY(serdes=GTModel(), sys_clk_freq=int(125e6), with_endianness_swap=False)
        comb = dut.get_fragment().comb
        def driven_by(l, r):
            return any(getattr(s, "l", None) is l and s.r is r for s in comb)
        self.assertTrue(driven_by(dut.ltssm.rx_idle8,       dut.pipe.idle_done))
        self.assertTrue(driven_by(dut.pipe.idle_handshake, dut.ltssm.idle_handshake))

if __name__ == "__main__":
    unittest.main()
