#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Shared test helpers: USB3 Gen1 reference models and stream drivers/monitors.

Symbols are (byte, k) tuples. Words are 32-bit data/4-bit ctrl with the first symbol in lane 0
(data[7:0]/ctrl[0]), as on the PIPE/SerDes streams (before USB3PIPE's EndiannessSwap).
"""

import random
import shutil

from pathlib import Path

from usb3_pipe.common import COM, SKP

# Test Output Directories --------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]

def get_output_dir(test, *names):
    """Return a fresh build/test/<test id>[/<names>] directory (one per test/subtest: safe with
    pytest-xdist, kept after the run for debug)."""
    output_dir = ROOT.joinpath("build", "test", test.id().removeprefix("test."), *names)
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True)
    return output_dir

# Symbols <-> Words --------------------------------------------------------------------------------

def words_to_symbols(words):
    """Convert (data, ctrl) words to a list of (byte, k) symbols (lane 0 first)."""
    symbols = []
    for data, ctrl in words:
        for i in range(4):
            symbols.append(((data >> 8*i) & 0xff, (ctrl >> i) & 0x1))
    return symbols

def symbols_to_words(symbols):
    """Convert (byte, k) symbols to (data, ctrl) words (lane 0 first), len must be a multiple of 4."""
    assert len(symbols) % 4 == 0
    words = []
    for i in range(0, len(symbols), 4):
        data = ctrl = 0
        for j, (byte, k) in enumerate(symbols[i:i+4]):
            data |= byte << 8*j
            ctrl |= k << j
        words.append((data, ctrl))
    return words

def ordered_set_words(ordered_set, link_config=None):
    """Return the (data, ctrl) words of an Ordered Set (COM symbols as K)."""
    data_bytes = ordered_set.to_bytes()
    if link_config is not None:
        data_bytes = bytearray(data_bytes)
        data_bytes[5] = link_config
    symbols = [(b, int(b == COM.value and i < 4)) for i, b in enumerate(data_bytes)]
    return symbols_to_words(symbols)

def is_skp(symbol):
    byte, k = symbol
    return k == 1 and byte == SKP.value

def strip_skp(symbols):
    """Remove SKP symbols (what the receiver elasticity/SKP removal is expected to deliver)."""
    return [s for s in symbols if not is_skp(s)]

# Gen1 Scrambler Reference Model (USB 3.2 6.3.1.3, Appendix B) -------------------------------------

LFSR_SEED = 0xffff

def lfsr_advance(lfsr):
    """Advance the G(X)=X^16+X^5+X^4+X^3+1 LFSR by one serial shift."""
    msb  = (lfsr >> 15) & 0x1
    lfsr = (lfsr << 1) & 0xffff
    if msb:
        lfsr ^= 0x0039 # X^5 + X^4 + X^3 + 1.
    return lfsr

def lfsr_scramble_byte(lfsr, byte):
    """XOR a byte with the LFSR output (D15 with D0 first), advancing 8 serial shifts."""
    out = 0
    for i in range(8):
        out  |= (((byte >> i) & 0x1) ^ ((lfsr >> 15) & 0x1)) << i
        lfsr  = lfsr_advance(lfsr)
    return out, lfsr

class ScramblerModel:
    """Symbol-level Gen1 (de)scrambler (scrambling and descrambling are the same operation).

    - D symbols are scrambled, K symbols are not.
    - The LFSR advances 8 shifts on every symbol except SKP.
    - The LFSR is initialized (FFFFh) after a COM.
    - Training Sequences are not scrambled (not modelled: don't feed them, or use enable=False).
    """
    def __init__(self, seed=LFSR_SEED):
        self.lfsr = seed

    def process(self, symbols, enable=True):
        out = []
        for byte, k in symbols:
            if k and byte == SKP.value:
                out.append((byte, k))
                continue
            if k:
                _, self.lfsr = lfsr_scramble_byte(self.lfsr, byte)
                out.append((byte, k))
            else:
                scrambled, self.lfsr = lfsr_scramble_byte(self.lfsr, byte)
                out.append((scrambled if enable else byte, k))
            if k and byte == COM.value:
                self.lfsr = LFSR_SEED
        return out

# Stream Drivers/Monitors --------------------------------------------------------------------------

def stream_driver(endpoint, words, rng=None, valid_rate=1.0, last=None):
    """Drive (data, ctrl) words on a sink endpoint, with optional random valid gaps.

    `last` is an optional list of booleans (same length as words) driving endpoint.last.
    """
    rng = rng or random.Random(0)
    for i, (data, ctrl) in enumerate(words):
        while valid_rate < 1.0 and rng.random() > valid_rate:
            yield endpoint.valid.eq(0)
            yield
        yield endpoint.valid.eq(1)
        yield endpoint.data.eq(data)
        yield endpoint.ctrl.eq(ctrl)
        if last is not None:
            yield endpoint.last.eq(last[i])
        yield
        while not (yield endpoint.ready):
            yield
    yield endpoint.valid.eq(0)

def stream_monitor(endpoint, n, rng=None, ready_rate=1.0, timeout=None):
    """Collect n (data, ctrl) words from a source endpoint, with optional random ready gaps.

    Returns the collected words (use `words = yield from stream_monitor(...)`).
    """
    rng   = rng or random.Random(1)
    words = []
    cycles = 0
    while len(words) < n:
        ready = 1 if ready_rate >= 1.0 else int(rng.random() < ready_rate)
        yield endpoint.ready.eq(ready)
        yield
        if ready and (yield endpoint.valid):
            words.append(((yield endpoint.data), (yield endpoint.ctrl)))
        cycles += 1
        if timeout is not None and cycles > timeout:
            break
    yield endpoint.ready.eq(0)
    return words

def random_symbols(n, rng, k_rate=0.0, com_rate=0.0, skp_rate=0.0, k_values=(0x5c, 0xfb, 0xfd)):
    """Generate n random (byte, k) symbols with optional K/COM/SKP insertion rates."""
    symbols = []
    for _ in range(n):
        r = rng.random()
        if r < com_rate:
            symbols.append((COM.value, 1))
        elif r < com_rate + skp_rate:
            symbols.append((SKP.value, 1))
        elif r < com_rate + skp_rate + k_rate:
            symbols.append((rng.choice(k_values), 1))
        else:
            symbols.append((rng.randrange(256), 0))
    return symbols
