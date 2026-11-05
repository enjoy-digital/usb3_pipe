#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 2 (128b/132b) bit-serial reference models of the USB 3.2 rules (6.3.2 encoding/scrambling,
6.4.1.2 Ordered Sets/DC balance, 6.4.3.3 SKP Ordered Sets) and a channel model (bit offset,
polarity, bit slips/errors, SKP Ordered Sets edits)."""

import random

from usb3_pipe.common     import *
from usb3_pipe.coding     import GEN2_DC_REDUCE_ONES, GEN2_DC_REDUCE_ZEROS
from usb3_pipe.scrambling import GEN2_LFSR_SEED

# Reference Models ---------------------------------------------------------------------------------

def figure_6_12_step(lfsr):
    """One shift of the Figure 6-12 LFSR, written register by register from the figure."""
    d   = [(lfsr >> i) & 0x1 for i in range(23)]
    out = d[22]
    n   = [0]*23
    n[0] = d[22]
    for i in range(1, 23):
        n[i] = d[i-1]
    for i in [2, 5, 8, 16, 21]:
        n[i] ^= d[22]
    return out, sum(b << i for i, b in enumerate(n))

class Gen2ScramblerModel:
    """Bit-serial Gen 2 LFSR (Figure 6-12, LSB first)."""
    def __init__(self, seed=GEN2_LFSR_SEED):
        self.seed = seed
        self.lfsr = seed

    def init(self):
        self.lfsr = self.seed

    def byte(self, value, advance=True, scramble=True):
        if not advance:
            return value
        out = 0
        for i in range(8):
            bit, self.lfsr = figure_6_12_step(self.lfsr)
            out |= (((value >> i) & 0x1) ^ (bit if scramble else 0)) << i
        return out

def int_to_bits(value, n):
    return [(value >> i) & 0x1 for i in range(n)]

def bits_to_int(bits):
    return sum(b << i for i, b in enumerate(bits))

def bits_from_words(words, width=32):
    return [b for w in words for b in int_to_bits(w, width)]

def words_from_bits(bits, width=32):
    bits = bits + [0]*((-len(bits)) % width)
    return [bits_to_int(bits[width*i:width*(i+1)]) for i in range(len(bits)//width)]

def popcount(bits):
    return sum(bits)

class Gen2ReferenceDecoder:
    """Decodes an aligned Gen 2 bit stream with the 6.3.2.3 scrambling rules and checks the Ordered
    Sets: TS identifiers, DC Balance Symbols (6.4.1.2.2 algorithm), SKP LFSR Symbols (Table 6-13)."""
    def __init__(self, test):
        self.test   = test
        self.model  = Gen2ScramblerModel()
        self.dc     = 0
        self.blocks = [] # (kind, position, length, payload/info)

    def decode(self, bits):
        pos = 0
        while pos + 36 <= len(bits):
            header = bits_to_int(bits[pos:pos+4])
            syms   = [bits_to_int(bits[pos+4+8*i:pos+12+8*i]) for i in range((len(bits) - pos - 4)//8)][:40]
            if header == BLOCK_HEADER_CONTROL and syms[0] in [GEN2_SKP, GEN2_SKPEND]:
                n = 0
                while n < len(syms) and syms[n] == GEN2_SKP:
                    n += 1
                if n + 4 > len(syms):
                    break # SKP Ordered Set truncated (end of the bits).
                self.test.assertEqual(syms[n], GEN2_SKPEND)
                self.test.assertEqual(syms[n+1:n+4], gen2_skp_lfsr_symbols(self.model.lfsr))
                length = 4 + 8*(n + 4)
                self.blocks.append(("SKP", pos, length, n))
                pos += length
                continue
            if pos + 132 > len(bits):
                break
            syms = syms[:16]
            if header == BLOCK_HEADER_CONTROL and syms == gen2_sync_payload():
                self.blocks.append(("SYNC", pos, 132, None))
                self.model.init()
            elif header == BLOCK_HEADER_CONTROL and syms[0] == GEN2_SDS:
                self.test.assertEqual(syms, gen2_sds_payload())
                for s in syms:
                    self.model.byte(s, scramble=False)
                self.dc = 0
                self.blocks.append(("SDS", pos, 132, None))
            elif header == BLOCK_HEADER_CONTROL:
                identifier = syms[0]
                self.test.assertIn(identifier, [GEN2_TSEQ, GEN2_TS1, GEN2_TS2])
                # DC Balance at the end of Symbol 11.
                block_bits = bits[pos:pos+132]
                balance    = self.dc + 2*popcount(block_bits[:100]) - 100
                dc14 = dc15 = None
                if abs(balance) > 31:
                    dc14, dc15 = GEN2_DC_REDUCE_ONES if balance > 0 else GEN2_DC_REDUCE_ZEROS
                elif abs(balance) > 15:
                    dc15 = (GEN2_DC_REDUCE_ONES if balance > 0 else GEN2_DC_REDUCE_ZEROS)[1]
                payload = [self.model.byte(syms[0], scramble=False)]
                payload += [self.model.byte(s) for s in syms[1:14]]
                for raw, dc in [(syms[14], dc14), (syms[15], dc15)]:
                    descrambled = self.model.byte(raw)
                    if dc is not None:
                        self.test.assertEqual(raw, dc)
                    else:
                        self.test.assertEqual(descrambled, identifier)
                    payload.append(identifier)
                self.test.assertEqual(payload[:4] + payload[6:], [identifier]*14)
                self.test.assertEqual(payload[4], 0)
                self.dc = max(-511, min(511, self.dc + 2*popcount(block_bits) - 132))
                self.blocks.append(({GEN2_TSEQ: "TSEQ", GEN2_TS1: "TS1", GEN2_TS2: "TS2"}[identifier],
                    pos, 132, payload[5]))
            else:
                self.test.assertEqual(header, BLOCK_HEADER_DATA)
                data = [self.model.byte(s) for s in syms]
                self.blocks.append(("DATA", pos, 132, symbols_to_int(data)))
            pos += 132
        return self.blocks

# Channel Model ------------------------------------------------------------------------------------

def gen2_channel(bits, blocks, skp_edits={}, slips=[], flips=[], offset=0, invert=False, seed=0):
    """Channel on a Gen 2 bit stream decoded by Gen2ReferenceDecoder (blocks):
    - skp_edits: {SKP Ordered Set index: SKP Symbols added/removed} (re-timers clock compensation).
    - slips:     [(position, +1: bit inserted/-1: bit removed)] (CDR bit slips).
    - flips:     [position] (bit errors).
    - offset:    random bits first, invert: polarity inversion.
    Positions are in the original stream. Returns the bits and the SKP Symbols count of each SKP
    Ordered Set after the edits."""
    rng   = random.Random(seed)
    bits  = list(bits)
    skps  = [b for b in blocks if b[0] == "SKP"]
    edits = [(skps[i][1] + 4, "skp", delta) for i, delta in skp_edits.items()]
    edits += [(pos, "slip", delta) for pos, delta in slips]
    edits += [(pos, "flip", None) for pos in flips]
    for pos, kind, delta in sorted(edits, key=lambda e: e[0], reverse=True):
        if kind == "skp":
            if delta > 0:
                bits[pos:pos] = int_to_bits(GEN2_SKP, 8)*delta
            else:
                del bits[pos:pos+8*(-delta)]
        elif kind == "slip":
            if delta > 0:
                bits[pos:pos] = [rng.getrandbits(1)]
            else:
                del bits[pos]
        else:
            bits[pos] ^= 1
    bits = [rng.getrandbits(1) for _ in range(offset)] + [b ^ int(invert) for b in bits]
    return bits, [b[3] + skp_edits.get(i, 0) for i, b in enumerate(skps)]
