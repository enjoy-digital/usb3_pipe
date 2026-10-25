#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

from usb3_pipe.common import K
from usb3_pipe.datapath import RXErrorSubstitution

K28_4 = K(28, 4)

# Helpers ------------------------------------------------------------------------------------------

K_SYMBOLS = [x | (y << 5) for x, y in [(28, 0), (28, 1), (28, 2), (28, 3), (28, 4), (28, 5), (28, 6),
    (28, 7), (23, 7), (27, 7), (29, 7), (30, 7)]]

def random_symbols(n, rng):
    return [(rng.choice(K_SYMBOLS), 1) if rng.random() < 0.1 else (rng.randrange(256), 0)
        for _ in range(n)]

def encode(symbols):
    """Encode symbols (2 per cycle) with LiteX's Encoder, return the 10-bit codes."""
    dut = Encoder(2, True)
    def generator(dut):
        dut.codes = []
        for i in range(0, len(symbols), 2):
            for j in range(2):
                yield dut.d[j].eq(symbols[i + j][0])
                yield dut.k[j].eq(symbols[i + j][1])
            yield
        for _ in range(4):
            yield
    def monitor(dut):
        for _ in range(2):
            yield
        for _ in range(len(symbols)//2):
            yield
            dut.codes += [(yield dut.output[0]), (yield dut.output[1])]
    run_simulation(dut, [generator(dut), monitor(dut)])
    return dut.codes

class SerDesRX(LiteXModule):
    """Decodes 10-bit codes (2 per cycle) to a data/ctrl stream, like the transceivers."""
    def __init__(self):
        self.input    = [Signal(10) for _ in range(2)]
        self.decoders = [Decoder(True) for _ in range(2)]
        self.submodules += self.decoders
        self.source   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        for i in range(2):
            self.comb += [
                self.decoders[i].input.eq(self.input[i]),
                self.source.data[8*i:8*(i+1)].eq(self.decoders[i].d),
                self.source.ctrl[i].eq(self.decoders[i].k),
            ]
        self.comb += self.source.valid.eq(1)

def substitute(codes):
    """Return the received (data, ctrl) symbols after RXErrorSubstitution."""
    class DUT(LiteXModule):
        def __init__(self):
            self.serdes       = serdes       = SerDesRX()
            self.substitution = substitution = RXErrorSubstitution(serdes, "sys")
            self.comb += serdes.source.connect(substitution.sink)
            self.comb += substitution.source.ready.eq(1)
    dut = DUT()
    def generator(dut):
        dut.received = []
        for i in range(0, len(codes), 2):
            yield dut.serdes.input[0].eq(codes[i])
            yield dut.serdes.input[1].eq(codes[i + 1])
            yield
            if i:
                data = (yield dut.substitution.source.data)
                ctrl = (yield dut.substitution.source.ctrl)
                dut.received += [(data & 0xff, ctrl & 1), (data >> 8, ctrl >> 1)]
        yield
        data = (yield dut.substitution.source.data)
        ctrl = (yield dut.substitution.source.ctrl)
        dut.received += [(data & 0xff, ctrl & 1), (data >> 8, ctrl >> 1)]
    run_simulation(dut, generator(dut))
    return dut.received

# 8b/10b decode/disparity errors (USB 3.2 6.3.1.2, 6.3.1.4) ----------------------------------------

class Test8b10bErrors(unittest.TestCase):
    def test_encoder_model_matches_litex_encoder(self):
        from usb3_pipe.coding import encode_8b10b
        rng     = random.Random(0)
        symbols = random_symbols(2000, rng)
        codes   = encode(symbols)
        rd = 0
        for (d, k), code in zip(symbols, codes):
            expected, rd = encode_8b10b(d, k, rd)
            self.assertEqual(code, int("{:010b}".format(expected)[::-1], 2))

    def test_valid_stream_not_substituted(self):
        rng     = random.Random(1)
        symbols = random_symbols(2000, rng)
        self.assertEqual(substitute(encode(symbols)), symbols)

    def test_invalid_code_substituted(self):
        # A received symbol that does not match any valid 8b/10b symbol shall be forwarded to the
        # link layer as K28.4 (6.3.1.4), including codes with a valid number of ones.
        rng     = random.Random(2)
        symbols = random_symbols(64, rng)
        codes   = encode(symbols)
        codes[33] = 0b1111100000 # 5 ones, not a valid 8b/10b code.
        received = substitute(codes)
        self.assertEqual(received[33], (K28_4, 1))
        self.assertEqual(received[:33], symbols[:33])

    def test_disparity_error_substituted(self):
        # All received symbols shall be in the column of the current running disparity, disparity
        # errors shall be reported (6.3.1.2): a valid code from the wrong column is substituted.
        symbols = [(0x03, 0)]*32 # D3.0: unbalanced code, alternates the running disparity.
        codes   = encode(symbols)
        codes[20] = codes[21] # Same column twice: second one has the wrong disparity.
        received = substitute(codes)
        self.assertEqual(received[:20], symbols[:20])
        self.assertIn((K28_4, 1), received[20:23])

if __name__ == "__main__":
    unittest.main()
