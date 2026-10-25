#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Seeded-random, model-checked coverage tests of the PIPE datapath (scrambling, SKP insertion/
removal, word alignment, USB3PIPE TX->RX loop)."""

import os
import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM, SKP, SDP, SHP, END, EPF, SLC, TS1, TS2
from usb3_pipe.core import USB3PIPE
from usb3_pipe.scrambling import Scrambler, Descrambler
from usb3_pipe.serdes import RXWordAligner, RXSKPRemover, TXSKPInserter, TXDatapath, RXDatapath

from test.common import (ScramblerModel, words_to_symbols, symbols_to_words, ordered_set_words,
    strip_skp, stream_driver, swap_word)

SKP_WORD    = (SKP.value*0x01010101, 0b1111)
# Link Layer framing K-codes (USB 3.2 6.3.1, Table 6-3): never COM/SKP.
FRAMING_K   = [SDP.value, SHP.value, END.value, EPF.value, SLC.value]
TX_SEED     = 0x7dbd # Scrambler default seed: LFSR state after a TS2 (see test_scrambling).
# Length scale of the random runs (ex: USB3_PIPE_TEST_SCALE=10 for longer runs).
SCALE       = float(os.environ.get("USB3_PIPE_TEST_SCALE", "1"))

# Helpers ------------------------------------------------------------------------------------------

def random_word(rng, k_rate=0.05, k_values=FRAMING_K):
    """Random (data, ctrl) word, each symbol being a K-code with probability k_rate."""
    data = ctrl = 0
    for i in range(4):
        if rng.random() < k_rate:
            data |= rng.choice(k_values) << 8*i
            ctrl |= 1 << i
        else:
            data |= rng.randrange(256) << 8*i
    return data, ctrl

def handshake_monitor(endpoint, rng, ready_rate, done, fields=("data", "ctrl"), idle=32):
    """Collect words on handshakes with random ready, until done() and `idle` cycles without valid.

    Also checks the stream stability rule: a valid word not accepted stays unchanged."""
    words   = []
    pending = None
    quiet   = 0
    while True:
        ready = 1 if ready_rate >= 1.0 else int(rng.random() < ready_rate)
        yield endpoint.ready.eq(ready)
        yield
        valid = (yield endpoint.valid)
        word  = []
        for f in fields:
            word.append((yield getattr(endpoint, f)))
        word  = tuple(word)
        if pending is not None:
            assert valid and word == pending, "unstable stream: {} -> {}".format(pending, word)
        pending = word if (valid and not ready) else None
        if valid and ready:
            words.append(word)
        quiet = 0 if valid else quiet + 1
        if done() and quiet > idle:
            break
    yield endpoint.ready.eq(0)
    return words

# Scrambling (USB 3.2 6.3.1.3, Appendix B) ---------------------------------------------------------

class TestScramblingRandom(unittest.TestCase):
    def test_scrambler_vs_model_backpressure(self):
        # 10k symbols of random data/framing K-codes, random scrambling enable per word, random
        # sink valid/source ready: Scrambler output == ScramblerModel (K-codes not scrambled, LFSR
        # advancing on every symbol, enable only bypassing the XOR).
        rng    = random.Random(0x5c4a)
        n      = int(2500*SCALE)
        words  = [random_word(rng, k_rate=0.1) for _ in range(n)]
        enable = [int(rng.random() < 0.9) for _ in range(n)]
        model  = ScramblerModel(seed=TX_SEED)
        expected = []
        for w, en in zip(words, enable):
            expected += symbols_to_words(model.process(words_to_symbols([w]), enable=en))

        dut = Scrambler()
        dut.done = False

        def generator(dut):
            for (data, ctrl), en in zip(words, enable):
                while rng.random() > 0.8:
                    yield dut.sink.valid.eq(0)
                    yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield dut.enable.eq(en)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(1), 0.7, lambda: dut.done)

        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(len(dut.words), n)
        self.assertEqual(dut.words, expected)

    def test_descrambler_vs_model_com_resync(self):
        # 6k symbols: random data/framing K-codes with word-aligned COM words (as after the
        # RXWordAligner: TS1/TS2 start with 4 COM), scrambled by the model (LFSR initialized after
        # COM), descrambled by the Descrambler with random valid/ready: plaintext recovered.
        rng   = random.Random(0xdec0)
        plain = []
        while len(plain) < 1500*SCALE:
            r = rng.random()
            if r < 0.01:
                plain += ordered_set_words(rng.choice([TS1, TS2])) # COM word + 3 D words.
            elif r < 0.02:
                plain.append((COM.value*0x01010101, 0b1111))
            else:
                plain.append(random_word(rng, k_rate=0.05))
        wire = symbols_to_words(ScramblerModel().process(words_to_symbols(plain)))

        dut = Descrambler()
        dut.done = False

        def generator(dut):
            yield from stream_driver(dut.sink, wire, rng=random.Random(2), valid_rate=0.8)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(3), 0.7, lambda: dut.done)

        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.words, plain)

    def test_scrambler_descrambler_roundtrip_random(self):
        # 1.6k symbols of random data/framing K-codes through Scrambler -> Descrambler (same seed)
        # with random valid/ready: output == input.
        rng   = random.Random(0x7007)
        words = [random_word(rng, k_rate=0.1) for _ in range(int(400*SCALE))]

        class DUT(Module):
            def __init__(self):
                self.submodules.scr = Scrambler(reset=0xffff)
                self.submodules.des = Descrambler(reset=0xffff)
                self.comb += self.scr.source.connect(self.des.sink)
                self.sink   = self.scr.sink
                self.source = self.des.source
                self.done   = False

        def generator(dut):
            yield from stream_driver(dut.sink, words, rng=random.Random(4), valid_rate=0.8)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(5), 0.7, lambda: dut.done)

        dut = DUT()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.words, words)

    # Symbol-level rules (6.3.1.3) ----------------------------------------------------------------
    # Minimal repros. In USB3PIPE, SKP are removed before the Descrambler (RXDatapath) and inserted
    # after the Scrambler (TXDatapath), COM are word-aligned by the RXWordAligner, and Training
    # Sequences bypass the Scrambler: the (de)scrambler still follows the rules at the symbol level.

    def run_word_level(self, dut, words):
        def generator(dut):
            yield from stream_driver(dut.sink, words)
        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(0), 1.0, lambda: True)
        run_simulation(dut, [generator(dut), checker(dut)])
        return dut.words

    def test_descrambler_skp_does_not_advance_lfsr(self):
        # 6.3.1.3: the LFSR is advanced on every symbol except SKP.
        plain = [(SKP.value*0x0101, 0b0011)] + [(0, 0)]*4 # SKP SKP D D, then D.
        wire  = symbols_to_words(ScramblerModel().process(words_to_symbols(plain)))
        self.assertEqual(self.run_word_level(Descrambler(), wire), plain)

    def test_descrambler_com_reset_mid_word(self):
        # 6.3.1.3: the LFSR is initialized after COM, including for the D symbols following a COM in
        # the same 32-bit word (COM not on the last lane).
        plain = [(0, 0)]*2 + [(COM.value, 0b0001)] + [(0, 0)]*4 # COM D D D, then D.
        wire  = symbols_to_words(ScramblerModel().process(words_to_symbols(plain)))
        self.assertEqual(self.run_word_level(Descrambler(), wire), plain)

    def test_scrambler_com_reset(self):
        # 6.3.1.3: the transmitter LFSR is also initialized after COM.
        plain = [(0, 0)]*2 + [(COM.value*0x01010101, 0b1111)] + [(0, 0)]*4
        wire  = symbols_to_words(ScramblerModel(seed=TX_SEED).process(words_to_symbols(plain)))
        self.assertEqual(self.run_word_level(Scrambler(), plain), wire)

# TX SKP Inserter (USB 3.2 6.4.3) ------------------------------------------------------------------

class TestTXSKPInserterRandom(unittest.TestCase):
    def test_tx_skp_inserter_random_packets(self):
        # 100k symbols of random packets (1..264 words: up to a max DPP) with random gaps, random
        # source ready. Checks: payload/first/last integrity, SKP only outside first..last, average
        # rate of 1 SKP Ordered Set per 354 symbols, at most 4 SKP Ordered Sets in a row.
        rng     = random.Random(0x354)
        packets = []
        nwords  = 0
        while nwords < 25000*SCALE:
            n = rng.choice([rng.randint(1, 8), rng.randint(1, 64), rng.randint(1, 264)])
            packets.append([random_word(rng, k_rate=0.02) for _ in range(n)])
            nwords += n
        expected = []
        for p in packets:
            for i, (d, c) in enumerate(p):
                expected.append((d, c, int(i == 0), int(i == len(p) - 1)))
        self.assertNotIn(SKP_WORD, [(d, c) for d, c, _, _ in expected])

        dut = TXSKPInserter()
        dut.done = False

        def generator(dut):
            for d, c, first, last in expected:
                if first:
                    for _ in range(rng.choice([0, 0, 0, 1, 2, 5])):
                        yield dut.sink.valid.eq(0)
                        yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(d)
                yield dut.sink.ctrl.eq(c)
                yield dut.sink.first.eq(first)
                yield dut.sink.last.eq(last)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(6), 0.8,
                lambda: dut.done, fields=("data", "ctrl", "first", "last"))

        run_simulation(dut, [generator(dut), checker(dut)])

        out = dut.words
        # Payload integrity.
        payload = [w for w in out if (w[0], w[1]) != SKP_WORD]
        self.assertEqual(payload, expected)
        # No SKP inside a packet; at most 2 SKP words (4 SKP Ordered Sets) in a row.
        in_packet = False
        run       = 0
        for d, c, first, last in out:
            if (d, c) == SKP_WORD:
                self.assertFalse(in_packet)
                run += 1
                self.assertLessEqual(run, 2)
                continue
            run = 0
            if first:
                in_packet = True
            if last:
                in_packet = False
        # Average rate (6.4.3): 1 SKP Ordered Set (2 SKP symbols) every 354 symbols, up to 4 SKP
        # Ordered Sets possibly still buffered at the end. The inserter schedules 2 SKP Ordered Sets
        # every 176 words: 1 every 352 symbols without SKP symbols, 356 with them (average 354), so
        # accept both ways of counting the transmitted symbols.
        skp_os  = 2*sum(1 for w in out if (w[0], w[1]) == SKP_WORD)
        symbols = 4*len(expected)
        self.assertGreaterEqual(skp_os, (symbols + 2*skp_os)//356 - 4)
        self.assertLessEqual(skp_os, symbols//352 + 4)
        self.assertAlmostEqual(symbols/skp_os, 354, delta=354*0.01)

# RX SKP Remover (USB 3.2 6.4.3) -------------------------------------------------------------------

class TestRXSKPRemoverRandom(unittest.TestCase):
    def test_rx_skp_remover_random_skp_os(self):
        # 60k symbols: random data/K-codes with 0/1/2 SKP Ordered Sets (SKP pairs) inserted at any
        # symbol position (any lane alignment), random valid/ready: output == strip_skp(input),
        # no loss/duplication.
        rng     = random.Random(0x3c)
        symbols = []
        while len(symbols) < 60000*SCALE:
            for _ in range(rng.randint(1, 400)):
                if rng.random() < 0.05:
                    symbols.append((rng.choice(FRAMING_K + [COM.value]), 1))
                else:
                    symbols.append((rng.randrange(256), 0))
            symbols += [(SKP.value, 1)]*2*rng.choice([0, 1, 1, 2])
        symbols += [(0, 0)]*((-len(symbols)) % 4)
        stripped = strip_skp(symbols)
        expected = symbols_to_words(stripped[:4*(len(stripped)//4)])

        dut = RXSKPRemover()
        dut.done = False

        def generator(dut):
            yield from stream_driver(dut.sink, symbols_to_words(symbols),
                rng=random.Random(7), valid_rate=0.85)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(8), 0.75, lambda: dut.done)

        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.words, expected)

# RX Word Aligner ----------------------------------------------------------------------------------

class TestRXWordAlignerRandom(unittest.TestCase):
    def test_rx_word_aligner_random_shift(self):
        # 24k symbols: segments of TS1/TS2 (COM-aligned) followed by data with Link Layer framing
        # K-codes, each segment shifted by a random 0-3 symbols on the wire (alignment changes
        # between segments).
        # The aligner must deliver each segment word-aligned (COM in lane 0) from its first COM
        # word, keeping the alignment on non-COM K-codes. Random sink valid/source ready.
        rng      = random.Random(0xa119)
        wire     = []
        segments = []
        while len(wire) < 24000*SCALE:
            shift   = rng.randrange(4)
            content = []
            for _ in range(rng.randint(1, 4)):
                content += words_to_symbols(ordered_set_words(rng.choice([TS1, TS2])))
            for _ in range(rng.randint(16, 600)):
                content += words_to_symbols([random_word(rng, k_rate=0.03)])
            # Junk (D symbols only, no marker) so that the segment starts at `shift` in its word.
            junk = 4*rng.randint(1, 4) + (shift - len(wire)) % 4
            wire += [(rng.randrange(256), 0) for _ in range(junk)]
            self.assertEqual(len(wire) % 4, shift)
            wire += content
            segments.append(symbols_to_words(content))
        wire += [(0, 0)]*(8 + (-len(wire)) % 4)

        dut = RXWordAligner()
        dut.done = False

        def generator(dut):
            yield from stream_driver(dut.sink, symbols_to_words(wire),
                rng=random.Random(9), valid_rate=0.85)
            dut.done = True

        def checker(dut):
            dut.words = yield from handshake_monitor(dut.source, random.Random(10), 0.75, lambda: dut.done)

        run_simulation(dut, [generator(dut), checker(dut)])

        out = dut.words
        pos = 0
        for i, seg in enumerate(segments):
            start = pos
            while start < len(out) and out[start:start + len(seg)] != seg:
                start += 1
            self.assertLess(start, len(out), "segment {} not found aligned".format(i))
            # The segment is found right after the previous one (only junk words in between).
            self.assertLessEqual(start - pos, 6, "segment {}".format(i))
            pos = start + len(seg)

# USB3PIPE End-to-End ------------------------------------------------------------------------------

class SerDesLoopModel(Module):
    """SerDes loop model: TXDatapath (SKP insertion, CDC, 32->16-bit) looped to RXDatapath (16->32-bit,
    SKP removal, CDC, word alignment)."""
    def __init__(self):
        self.sink          = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source        = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.rx_align      = Signal()
        self.rx_polarity   = Signal()
        self.rx_idle       = Signal()
        self.tx_idle       = Signal()
        self.tx_pattern    = Signal(20)
        self.tx_skp_enable = Signal(reset=1)

        # # #

        self.submodules.tx = tx = TXDatapath("serdes")
        self.submodules.rx = rx = RXDatapath("serdes")
        self.comb += [
            tx.skp_enable.eq(self.tx_skp_enable),
            self.sink.connect(tx.sink),
            tx.source.connect(rx.sink),
            rx.source.connect(self.source),
        ]

class TestUSB3PIPELoopRandom(unittest.TestCase):
    def test_usb3pipe_loop_random_packets(self):
        # 2 TS2 (descrambler LFSR sync on COM) then U0: ~640 symbols of random Link Layer packets
        # (first/last, random gaps -> Logical Idle) on the TX sink with tx_ready, looped through the
        # SerDes model (SKP insertion/removal, CDCs, word alignment), received on the RX source with
        # rx_ready (random ready).
        # Checks: RX data == TX data (Idle Symbols removed), and the wire endianness: descrambled
        # wire == EndiannessSwap(TX data) (first transmitted symbol = data[31:24]/ctrl[3]).
        # Packets start with a framing Ordered Set (SHP/SDP/SLC x3 + EPF, 7.2.1/7.2.2), the RX words
        # are re-aligned on them in U0: the start K-codes only appear there.
        rng     = random.Random(0xe2e)
        packets = []
        nwords  = 0
        while nwords < 160*SCALE:
            n = rng.randint(1, 32)
            start = rng.choice([SHP.value, SDP.value, SLC.value])
            packets.append([((start << 24) | (start << 16) | (start << 8) | EPF.value, 0b1111)] +
                [random_word(rng, k_rate=0.05, k_values=[END.value, EPF.value]) for _ in range(n)])
            nwords += n + 1
        words = []
        for p in packets:
            for i, (d, c) in enumerate(p):
                if (d, c) == (0, 0): # Idle Symbols word: not distinguishable from Logical Idle.
                    d = 1
                words.append((d, c, int(i == 0), int(i == len(p) - 1)))

        class Top(Module):
            def __init__(self):
                self.submodules.serdes = SerDesLoopModel()
                self.submodules.pipe   = USB3PIPE(serdes=self.serdes, sys_clk_freq=int(125e6))
                self.done = False

        top    = Top()
        serdes = top.serdes
        dut    = top.pipe

        def generator(dut):
            # 2 TS2 (not scrambled, LFSR state 7DBDh after each TS2 on both sides).
            yield dut.ts_tx_enable.eq(1)
            yield dut.ts_tx_ts2.eq(1)
            yield
            n = 0
            while True:
                n += (yield serdes.sink.valid) & (yield serdes.sink.ready)
                if n == 2*len(ordered_set_words(TS2)):
                    break
                yield
            yield dut.ts_tx_ts2.eq(0)
            yield dut.ts_tx_enable.eq(0)
            yield dut.tx_ready.eq(1) # Logical Idle.
            for _ in range(32): # rx_ready set while receiving Idle (U0 entry after 8 Idle Symbols).
                yield
            yield dut.rx_ready.eq(1)
            for _ in range(8):
                yield
            for d, c, first, last in words:
                while rng.random() > 0.8:
                    yield dut.sink.valid.eq(0)
                    yield
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(d)
                yield dut.sink.ctrl.eq(c)
                yield dut.sink.first.eq(first)
                yield dut.sink.last.eq(last)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(40):
                yield
            top.done = True

        def rx_checker(dut):
            dut.rx_words = []
            while not top.done:
                ready = int(rng.random() < 0.9)
                yield dut.source.ready.eq(ready)
                yield
                if ready and (yield dut.source.valid):
                    dut.rx_words.append(((yield dut.source.data), (yield dut.source.ctrl)))

        def wire_monitor(dut):
            dut.wire = []
            while not top.done:
                yield
                if (yield dut.tx_ready) and (yield serdes.sink.valid) and (yield serdes.sink.ready):
                    dut.wire.append(((yield serdes.sink.data), (yield serdes.sink.ctrl)))

        run_simulation(top, [generator(dut), rx_checker(dut), wire_monitor(dut)],
            clocks={"sys": 8, "serdes": 8})

        tx_words = [(d, c) for d, c, _, _ in words]
        # Wire: descrambled from the post-TS2 seed, Idle Symbols removed, symbol order swapped.
        wire = symbols_to_words(ScramblerModel(seed=TX_SEED).process(words_to_symbols(dut.wire)))
        wire = [w for w in wire if w != (0, 0)]
        self.assertEqual(wire, [swap_word(w) for w in tx_words])
        # RX: Idle Symbols removed.
        rx_words = [w for w in dut.rx_words if w != (0, 0)]
        self.assertEqual(rx_words, tx_words)

if __name__ == "__main__":
    unittest.main()
