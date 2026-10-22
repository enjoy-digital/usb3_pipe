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

from test.common import ScramblerModel, words_to_symbols, symbols_to_words, stream_driver

# Transceiver TX model -----------------------------------------------------------------------------

class GTTXModel(Module):
    """Transceiver TX model: always ready, and like the GT (which ignores valid), transmits the last
    presented word again when the stream is not valid. Also provides the RX/LFPS interface."""
    def __init__(self):
        self.sink        = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source      = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.rx_align    = Signal()
        self.rx_polarity = Signal()
        self.rx_idle     = Signal()
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)
        self.comb += self.sink.ready.eq(1)

def wire_monitor(gt, n):
    """Record n words transmitted on the wire (last word repeated when not valid), starting from the
    first valid word."""
    words = []
    last  = None
    for _ in range(n):
        yield
        if (yield gt.sink.valid):
            last = ((yield gt.sink.data), (yield gt.sink.ctrl))
        if last is not None:
            words.append(last)
    return words

# USB3PIPE TX --------------------------------------------------------------------------------------

class TestUSB3PIPETX(unittest.TestCase):
    def test_tx_logical_idle_when_no_link_layer_data(self):
        # In U0, Idle Symbols (00h, scrambled) are transmitted when no packet/link command is sent
        # (USB 3.2 7.2.3). The wire must stay a valid scrambled stream when the Link Layer has no
        # data: descrambled, it must be the Link Layer data with only Idle Symbols inserted.
        rng   = random.Random(0)
        words = [(rng.randrange(1, 2**32), 0) for _ in range(256)] # Non-zero data words.

        gt  = GTTXModel()
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
