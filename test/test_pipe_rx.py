#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import TS2
from usb3_pipe.core import USB3PIPE

from test.common import ScramblerModel, ordered_set_words, words_to_symbols, symbols_to_words

# Transceiver RX model -----------------------------------------------------------------------------

class GTRXModel(Module):
    """Transceiver RX model: presents one received word per cycle (the GT ignores back-pressure)."""
    def __init__(self):
        self.sink        = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source      = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.rx_align    = Signal()
        self.rx_polarity = Signal()
        self.rx_idle     = Signal()
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)

# USB3PIPE RX --------------------------------------------------------------------------------------

class TestUSB3PIPERX(unittest.TestCase):
    def test_rx_descrambler_syncs_on_ts2_before_rx_ready(self):
        # The descrambler LFSR is initialized on every received COM (USB 3.2 6.3.1.3 rule 5) and
        # advances on every non-SKP symbol, including Training Sequences (not scrambled). The
        # partner's last TS2 are received in Polling.Configuration/Idle, before U0 (rx_ready), and
        # its Idle Symbols/data continue from the post-TS2 LFSR state: the first data received in
        # U0 must be correctly descrambled.
        rng   = random.Random(0)
        words = [(rng.randrange(1, 2**32), 0) for _ in range(64)] # Non-zero data words.

        partner = ScramblerModel()
        ts2     = partner.process(words_to_symbols(ordered_set_words(TS2)*16), enable=False)
        idle    = partner.process([(0x00, 0)]*4*32)
        data    = partner.process(words_to_symbols(words))
        wire    = symbols_to_words(ts2 + idle + data)
        rx_ready_delay = len(ordered_set_words(TS2))*16 + 16 # rx_ready set while receiving Idle.

        gt  = GTRXModel()
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt

        def generator(dut):
            for i, (d, c) in enumerate(wire):
                yield dut.rx_ready.eq(i >= rx_ready_delay)
                yield gt.source.valid.eq(1)
                yield gt.source.data.eq(d)
                yield gt.source.ctrl.eq(c)
                yield
            yield gt.source.valid.eq(0)
            for _ in range(8):
                yield

        def checker(dut):
            dut.received = []
            yield dut.source.ready.eq(1)
            for _ in range(len(wire) + 8):
                yield
                if (yield dut.source.valid):
                    dut.received.append(((yield dut.source.data), (yield dut.source.ctrl)))

        run_simulation(dut, [generator(dut), checker(dut)])
        received = [w for w in dut.received if w != (0, 0)] # Remove Idle Symbols.
        self.assertEqual(received, words)

if __name__ == "__main__":
    unittest.main()
