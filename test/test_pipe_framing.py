#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import SKP, SDP, SHP, SLC, EPF, TS2
from usb3_pipe.core import USB3PIPE

from test.model.gt import GTModel
from test.common import (ScramblerModel, words_to_symbols, symbols_to_words, ordered_set_words, strip_skp,
    swap_word)

# Helpers ------------------------------------------------------------------------------------------

class SerDesRXModel(GTModel):
    """SerDes model: RX words as delivered by the SerDes RX datapath (COM aligned, SKP removed)."""
    def __init__(self):
        GTModel.__init__(self, tx_ready=True)

def framing(start):
    return [(start, 1)]*3 + [(EPF.value, 1)]

# Test PIPE Framing Alignment ----------------------------------------------------------------------

class TestPIPEFramingAlignment(unittest.TestCase):
    def framing_test(self, shift):
        # Partner transmitter: 2 TS2 then U0 with Logical Idle and Link Layer packets (Link Command,
        # Header Packet + Data Packet), each aligned on its own symbol count. SKP Ordered Sets (2
        # symbols, 6.4.3.1) are inserted between packets, the SKP removal then shifts the framing by
        # 2 symbols in the received words (observed on hardware with an xHCI host in U0).
        rng  = random.Random(shift)
        data = lambda n: [(rng.randrange(1, 256), 0) for _ in range(n)]
        idle = lambda n: [(0x00, 0)]*n
        skp  = [(SKP.value, 1)]*2
        packets = [
            framing(SLC.value) + data(4),                                 # Link Command.
            framing(SHP.value) + data(16) + framing(SDP.value) + data(8), # Header + Data Packet.
            framing(SLC.value) + data(4),                                 # Link Command.
        ]
        u0 = idle(32)
        for packet in packets*2:
            # Partner alignment: SKP Ordered Set + 2 Idle Symbols.
            u0 += skp*shift + idle(2*shift) + packet + idle(8)
        u0 += idle(32)
        ts = []
        for _ in range(2):
            ts += words_to_symbols(ordered_set_words(TS2))
        scrambler = ScramblerModel()
        wire = scrambler.process(ts, enable=False) + scrambler.process(u0)
        wire = strip_skp(wire)
        wire += idle((-len(wire)) % 4)
        rx_words = symbols_to_words(wire)

        serdes = SerDesRXModel()
        dut    = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6))
        dut.submodules += serdes

        def generator():
            for i, (d, c) in enumerate(rx_words):
                yield serdes.source.valid.eq(1)
                yield serdes.source.data.eq(d)
                yield serdes.source.ctrl.eq(c)
                if i == 2*len(ordered_set_words(TS2)) + 4: # rx_ready set while receiving Idle.
                    yield dut.rx_ready.eq(1)
                yield
                while not (yield serdes.source.ready):
                    yield
            yield serdes.source.valid.eq(0)
            for _ in range(8):
                yield

        def checker():
            dut.words = []
            for _ in range(len(rx_words) + 16):
                yield dut.source.ready.eq(1)
                yield
                if (yield dut.source.valid):
                    dut.words.append(((yield dut.source.data), (yield dut.source.ctrl)))

        run_simulation(dut, [generator(), checker()])

        # Link Layer view (first symbol on data[31:24]): packets aligned, Logical Idle removed.
        expected = [swap_word(w) for packet in packets*2 for w in symbols_to_words(packet)]
        received = [w for w in dut.words if w != (0, 0)]
        self.assertEqual(received, expected)

    def test_framing_aligned(self):
        self.framing_test(shift=0)

    def test_framing_shifted_by_skp(self):
        self.framing_test(shift=1)

if __name__ == "__main__":
    unittest.main()
