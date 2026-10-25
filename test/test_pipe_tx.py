#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.core import USB3PIPE

from test.common   import ScramblerModel, words_to_symbols, symbols_to_words, stream_driver
from test.model.gt import GTModel, wire_monitor

# USB3PIPE TX --------------------------------------------------------------------------------------

class TestUSB3PIPETX(unittest.TestCase):
    def test_tx_logical_idle_when_no_link_layer_data(self):
        # In U0, Idle Symbols (00h, scrambled) are transmitted when no packet/link command is sent
        # (USB 3.2 7.2.3). The wire must stay a valid scrambled stream when the Link Layer has no
        # data: descrambled, it must be the Link Layer data with only Idle Symbols inserted.
        rng   = random.Random(0)
        words = [(rng.randrange(1, 2**32), 0) for _ in range(256)] # Non-zero data words.

        gt  = GTModel()
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt

        def generator(dut):
            yield dut.tx_ready.eq(1)
            yield
            yield from stream_driver(dut.sink, words, rng=rng, valid_rate=0.7)

        def checker(dut):
            dut.wire = yield from wire_monitor(gt, 4*len(words))

        run_simulation(dut, [generator(dut), checker(dut)])

        # Descramble from the scrambler seed (state after the last TS2, see Scrambler).
        symbols = ScramblerModel(seed=0x7dbd).process(words_to_symbols(dut.wire))
        received = [w for w in symbols_to_words(symbols) if w != (0, 0)]
        self.assertEqual(received[:len(words)], words)

if __name__ == "__main__":
    unittest.main()
