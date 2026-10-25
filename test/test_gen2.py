#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 2 (128b/132b) PHY building blocks against bit-serial reference models of the USB 3.2 rules
(6.3.2 encoding/scrambling, 6.4.1.2 Ordered Sets/DC balance, 6.4.3.3 SKP Ordered Sets) and a Gen 2
TX -> channel (bit offset, polarity inversion, bit slip, SKP add/remove) -> RX loopback."""

import random
import unittest

import pytest
import functools

from migen import *

from litex.gen import *

from usb3_pipe.gen2 import *

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

def bits_from_words(words):
    return [b for w in words for b in int_to_bits(w, 32)]

def words_from_bits(bits):
    bits = bits + [0]*((-len(bits)) % 32)
    return [bits_to_int(bits[32*i:32*(i+1)]) for i in range(len(bits)//32)]

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
                while syms[n] == GEN2_SKP:
                    n += 1
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

# Gen 2 TX Simulation ------------------------------------------------------------------------------

class TXDUT(LiteXModule):
    def __init__(self, **kwargs):
        self.tx      = Gen2TX(**kwargs)
        self.gearbox = Gen2BlockGearbox()
        self.comb += self.tx.source.connect(self.gearbox.sink)

@functools.lru_cache(maxsize=None)
def run_tx(n_tseq=20, n_ts1=80, n_ts2=24, n_packets=40, link_config=0x00, seed=0, sync_interval_tseq=8):
    """Gen2TX + gearbox: TSEQ, TS1, TS2 then Data (random packets of 1-4 blocks, gaps of Logical
    Idle). Returns the transmitted words and the packets data blocks."""
    rng     = random.Random(seed)
    dut     = TXDUT(sync_interval_tseq=sync_interval_tseq)
    words   = []
    packets = [[rng.getrandbits(128) for _ in range(rng.randint(1, 4))] for _ in range(n_packets)]
    state   = {"done": False}
    def control(dut):
        yield dut.tx.enable.eq(1)
        yield dut.tx.link_config.eq(link_config)
        for name, n in [("tx_tseq", n_tseq), ("tx_ts1", n_ts1), ("tx_ts2", n_ts2)]:
            yield getattr(dut.tx, name).eq(1)
            count = 0
            while count < n:
                count += (yield dut.tx.tx_os)
                yield
            yield getattr(dut.tx, name).eq(0)
        yield dut.tx.tx_data.eq(1)
        for packet in packets:
            for _ in range(rng.randint(0, 3)):
                yield
            for i, data in enumerate(packet):
                yield dut.tx.sink.valid.eq(1)
                yield dut.tx.sink.data.eq(data)
                yield dut.tx.sink.last.eq(i == len(packet) - 1)
                yield
                while not (yield dut.tx.sink.ready):
                    yield
            yield dut.tx.sink.valid.eq(0)
        for _ in range(64):
            yield
        state["done"] = True
    def collect(dut):
        yield dut.gearbox.source.ready.eq(1)
        while not state["done"]:
            if (yield dut.gearbox.source.valid):
                words.append((yield dut.gearbox.source.data))
            yield
    run_simulation(dut, [control(dut), collect(dut)])
    return words, packets

# Short Gen2TX runs (default suite), full runs: pytest -m slow.
RUN_TX_SHORT = dict(n_ts1=48, n_ts2=16, n_packets=8)

# Tests --------------------------------------------------------------------------------------------

class TestGen2(unittest.TestCase):
    def test_lfsr_figure_6_12(self):
        # The polynomial/taps used by the hardware and gen2_lfsr_advance are the Figure 6-12 LFSR.
        rng = random.Random(0)
        for _ in range(100):
            lfsr  = rng.getrandbits(23)
            masks = gen2_lfsr_advance([1 << n for n in range(23)])
            hw    = sum((bin(lfsr & m).count("1") & 0x1) << i for i, m in enumerate(masks))
            out, ref = figure_6_12_step(lfsr)
            self.assertEqual(hw,  ref)
            self.assertEqual(out, (lfsr >> 22) & 0x1)
        # Maximal length: 2^23 - 1 period (primitive polynomial), sampled: no return before.
        lfsr = GEN2_LFSR_SEED
        for _ in range(4096):
            _, lfsr = figure_6_12_step(lfsr)
            self.assertNotEqual(lfsr, GEN2_LFSR_SEED)

    def test_skp_lfsr_symbols(self):
        # Table 6-13: 4N+1 = {~LFSR[22], LFSR[22:16]} (40h-FFh), LFSR[15:8], LFSR[7:0].
        self.assertEqual(gen2_skp_lfsr_symbols(0x400000), [0x40, 0x00, 0x00])
        self.assertEqual(gen2_skp_lfsr_symbols(0x000000), [0x80, 0x00, 0x00])
        self.assertEqual(gen2_skp_lfsr_symbols(GEN2_LFSR_SEED), [0x80 | 0x1d, 0xbf, 0xbc])
        for lfsr in range(0, 2**23, 4099):
            self.assertTrue(0x40 <= gen2_skp_lfsr_symbols(lfsr)[0] <= 0xff)

    def test_scrambler_vs_model(self):
        rng   = random.Random(0)
        dut   = Gen2Scrambler(nbytes=4)
        model = Gen2ScramblerModel()
        def generator(dut):
            for _ in range(200):
                data     = [rng.randrange(256) for _ in range(4)]
                advance  = [rng.random() < 0.9 for _ in range(4)]
                scramble = [rng.random() < 0.8 for _ in range(4)]
                yield dut.ce.eq(1)
                yield dut.i.eq(int.from_bytes(bytes(data), "little"))
                yield dut.advance.eq(sum(a << i for i, a in enumerate(advance)))
                yield dut.scramble.eq(sum(s << i for i, s in enumerate(scramble)))
                yield
                out = (yield dut.o)
                ref = [model.byte(d, a, s) for d, a, s in zip(data, advance, scramble)]
                self.assertEqual(out, int.from_bytes(bytes(ref), "little"))
        run_simulation(dut, generator(dut))

    def test_scrambler_init(self):
        # init: LFSR re-initialized to the seed (SYNC Ordered Set).
        dut = Gen2Scrambler(nbytes=4)
        def generator(dut):
            yield dut.ce.eq(1)
            yield dut.advance.eq(0b1111)
            yield dut.scramble.eq(0b1111)
            yield
            first = (yield dut.o)
            for _ in range(10):
                yield
            yield dut.init.eq(1)
            yield
            yield dut.init.eq(0)
            yield
            self.assertEqual((yield dut.o), first)
        run_simulation(dut, generator(dut))

    def tx_vs_reference_test(self, **kwargs):
        # Gen2TX output decoded with the bit-serial reference model: Ordered Sets content, DC Balance
        # Symbols, SKP LFSR Symbols, SYNC/SKP scheduling and Data Blocks.
        words, packets = run_tx(link_config=0x08, **kwargs)
        blocks = Gen2ReferenceDecoder(self).decode(bits_from_words(words))
        kinds  = [b[0] for b in blocks]
        # SYNC first, then every 8 TSEQ (sync_interval_tseq) / 32 TS1/TS2.
        self.assertEqual(kinds[0], "SYNC")
        os_since_sync = 0
        for kind in kinds:
            if kind == "SYNC":
                self.assertLessEqual(os_since_sync, 32)
                os_since_sync = 0
            elif kind in ["TSEQ", "TS1", "TS2"]:
                os_since_sync += 1
        tseq = [i for i, k in enumerate(kinds) if k == "TSEQ"]
        self.assertEqual(sum(1 for k in kinds[tseq[0]:tseq[-1]] if k == "SYNC"), 2)
        # No SKP after TSEQ (Y reset), SKP after SYNC/TS1/TS2/SDS/Data (6.4.3.3).
        for i, kind in enumerate(kinds):
            if kind == "SKP":
                self.assertNotEqual(kinds[i-1], "TSEQ")
        # SKP rate: ~1 every 40 non-SKP blocks once out of TSEQ.
        start = kinds.index("TS1")
        n_skp = kinds[start:].count("SKP")
        n_blk = len(kinds[start:]) - n_skp
        self.assertAlmostEqual(n_skp, n_blk/40, delta=2)
        # Link Functionality.
        self.assertTrue(all(b[3] == 0x08 for b in blocks if b[0] in ["TS1", "TS2"]))
        # Data: SDS then the packets data blocks with Logical Idle (00h) blocks between packets.
        self.assertEqual(kinds.count("SDS"), 1)
        data = [b[3] for b in blocks[kinds.index("SDS"):] if b[0] == "DATA"]
        self.assertEqual([d for d in data if d != 0], [d for p in packets for d in p if d != 0])
        # No SKP inside a packet (after a Logical Idle block or the last block of a packet only).
        lasts = set()
        n = 0
        for p in packets:
            n += len(p)
            lasts.add(n - 1)
        idx       = 0
        allowed   = True
        for kind, pos, length, info in blocks[kinds.index("SDS") + 1:]:
            if kind == "DATA":
                if info == 0:
                    allowed = True
                else:
                    allowed = idx in lasts
                    idx += 1
            elif kind == "SKP":
                self.assertTrue(allowed)

    def test_tx_vs_reference(self):
        # Short: less TS1/TS2/packets.
        self.tx_vs_reference_test(**RUN_TX_SHORT)

    @pytest.mark.slow
    def test_tx_vs_reference_long(self):
        self.tx_vs_reference_test()

    def test_dc_balance(self):
        # Long TS1 run: the running DC Balance stays bounded (DC Balance Symbols, 6.4.1.2.2).
        words, _ = run_tx(n_tseq=1, n_ts1=120, n_ts2=1, n_packets=1)
        bits    = bits_from_words(words)
        decoder = Gen2ReferenceDecoder(self)
        decoder.decode(bits)
        dc, worst = 0, 0
        for kind, pos, length, info in decoder.blocks:
            if kind in ["TS1"]:
                dc += 2*popcount(bits[pos:pos+length]) - length
                worst = max(worst, abs(dc))
        self.assertLess(worst, 64)

    def test_gearbox_aligner_roundtrip(self):
        # Blocks (SYNC first, data/control, SKP Ordered Sets of 4-36 SKP Symbols) -> gearbox -> bit
        # offset -> aligner: blocks recovered after the SYNC, SKP removed, Block Header single bit
        # error corrected.
        rng    = random.Random(1)
        blocks = [(BLOCK_HEADER_CONTROL, symbols_to_int(gen2_sync_payload()), 0)]
        skps   = []
        for i in range(64):
            if i % 10 == 5:
                n = rng.choice([0, 4, 8, 20, 36])
                skps.append(n)
                payload = gen2_skp_payload(0x123456 + i, n)
                blocks.append(("SKP", payload))
            else:
                header = rng.choice([BLOCK_HEADER_DATA, BLOCK_HEADER_CONTROL])
                data   = rng.getrandbits(128)
                if header == BLOCK_HEADER_CONTROL and (data & 0xff) in [GEN2_SKP, GEN2_SKPEND]:
                    data ^= 0x1
                blocks.append((header, data, 0))
        bits = []
        for block in blocks:
            if block[0] == "SKP":
                bits += int_to_bits(BLOCK_HEADER_CONTROL, 4)
                bits += [b for s in block[1] for b in int_to_bits(s, 8)]
            else:
                bits += int_to_bits(block[0], 4) + int_to_bits(block[1], 128)
        expected = [(h, d) for h, d, *_ in [b for b in blocks if b[0] != "SKP"]]

        # TX gearbox (132-bit blocks only, SKP payloads are checked through the TX tests).
        tx = Gen2BlockGearbox()
        words = []
        normal = [b for b in blocks if b[0] != "SKP"]
        def tx_generator(dut):
            for header, payload, _ in normal:
                yield dut.sink.valid.eq(1)
                yield dut.sink.header.eq(header)
                yield dut.sink.data.eq(payload)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(16):
                yield
        def tx_checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(600):
                if (yield dut.source.valid):
                    words.append((yield dut.source.data))
                yield
        run_simulation(tx, [tx_generator(tx), tx_checker(tx)])
        ref = [b for h, p, _ in normal for b in int_to_bits(h, 4) + int_to_bits(p, 128)]
        self.assertGreaterEqual(32*len(words), len(ref) - 31)
        self.assertEqual(bits_from_words(words)[:len(ref)], ref[:32*len(words)])

        # Channel: random bit offset, a Block Header single bit error.
        offset = 37
        bits   = [rng.getrandbits(1) for _ in range(offset)] + bits + [0]*64
        bits[offset + 132*3 + 1] ^= 1

        # RX aligner.
        rx  = Gen2BlockAligner()
        out = []
        skp = []
        def rx_generator(dut):
            for w in words_from_bits(bits):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(w)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
        def rx_checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(len(bits)//32 + 64):
                if (yield dut.source.valid):
                    out.append(((yield dut.source.header), (yield dut.source.data),
                        (yield dut.source.header_error)))
                if (yield dut.skp):
                    skp.append(((yield dut.skp_count), (yield dut.skp_lfsr)))
                yield
        run_simulation(rx, [rx_generator(rx), rx_checker(rx)])
        received = [(h, p) for h, p, e in out]
        self.assertEqual(received, expected[:len(received)])
        self.assertGreaterEqual(len(received), len(expected) - 1)
        self.assertEqual([i for i, (h, p, e) in enumerate(out) if e], [3])
        self.assertEqual([n for n, l in skp], skps)
        self.assertEqual([l for n, l in skp], [0x123456 + i for i in range(64) if i % 10 == 5])

    def loopback_test(self, edits={0: -4, 1: +4, 2: +16, 3: -20}, **kwargs):
        # Gen2TX -> gearbox -> channel -> Gen2BlockAligner -> Gen2RX:
        # - channel: bit offset, polarity inversion, a bit slip during TS1 (realignment on the next
        #   SYNC), SKP Ordered Sets shortened/lengthened (re-timer clock compensation).
        # - RX: polarity detected on SYNC, TS1/TS2 detected, SDS, Data Blocks recovered, received
        #   SKP LFSR state equal to the RX descrambler state.
        rng = random.Random(2)
        words, packets = run_tx(link_config=0x08, **kwargs)
        bits   = bits_from_words(words)
        blocks = Gen2ReferenceDecoder(self).decode(bits)
        sent_data = [b[3] for b in blocks[[b[0] for b in blocks].index("SDS"):] if b[0] == "DATA"]

        # SKP Ordered Sets edits (applied from the end to keep the positions valid).
        skp_blocks = [b for b in blocks if b[0] == "SKP"]
        self.assertGreaterEqual(len(skp_blocks), len(edits))
        for index in sorted(edits, reverse=True):
            kind, pos, length, n = skp_blocks[index]
            delta = edits[index]
            if delta > 0:
                bits[pos+4:pos+4] = int_to_bits(GEN2_SKP, 8)*delta
            else:
                del bits[pos+4:pos+4+8*(-delta)]
        expected_skp = [b[3] + edits.get(i, 0) for i, b in enumerate(skp_blocks)]
        # Bit slip in the TS1 phase (removed bit), before the edits positions.
        ts1 = [b for b in blocks if b[0] == "TS1"]
        slip_pos = ts1[3][1] + 50
        self.assertLess(slip_pos, skp_blocks[0][1])
        del bits[slip_pos]
        # Offset and polarity inversion.
        bits = [rng.getrandbits(1) for _ in range(19)] + [b ^ 1 for b in bits]

        class RXDUT(LiteXModule):
            def __init__(self):
                self.aligner = Gen2BlockAligner()
                self.rx      = Gen2RX()
                self.comb += self.aligner.source.connect(self.rx.sink)
        dut = RXDUT()
        events = {"ts1": 0, "ts2": 0, "sds": 0, "error": 0, "skp": [], "data": []}
        def rx_generator(dut):
            for w in words_from_bits(bits):
                yield dut.aligner.sink.valid.eq(1)
                yield dut.aligner.sink.data.eq(w)
                yield
            yield dut.aligner.sink.valid.eq(0)
        def rx_checker(dut):
            yield dut.rx.source.ready.eq(1)
            for _ in range(len(bits)//32 + 32):
                events["ts1"]   += (yield dut.rx.rx_ts1)
                events["ts2"]   += (yield dut.rx.rx_ts2)
                events["sds"]   += (yield dut.rx.rx_sds)
                events["error"] += (yield dut.aligner.error)
                if (yield dut.aligner.skp):
                    events["skp"].append(((yield dut.aligner.skp_count),
                        (yield dut.aligner.skp_lfsr), (yield dut.rx.lfsr)))
                if (yield dut.rx.source.valid):
                    events["data"].append((yield dut.rx.source.data))
                yield
            events["polarity"]    = (yield dut.aligner.polarity)
            events["locked"]      = (yield dut.aligner.locked)
            events["link_config"] = (yield dut.rx.link_config)
        run_simulation(dut, [rx_generator(dut), rx_checker(dut)])

        self.assertEqual(events["polarity"], 1)
        self.assertEqual(events["locked"], 1)
        self.assertGreaterEqual(events["error"], 1) # Bit slip detected.
        self.assertGreaterEqual(events["ts1"], 2)
        self.assertGreaterEqual(events["ts2"], 2)
        self.assertEqual(events["sds"], 1)
        self.assertEqual(events["link_config"], 0x08)
        # SKP received after the realignment (SKP Ordered Sets before it may be lost).
        received_skp = [n for n, l, r in events["skp"]]
        self.assertEqual(received_skp, expected_skp[len(expected_skp) - len(received_skp):])
        self.assertGreaterEqual(len(received_skp), len(edits))
        for n, skp_lfsr, rx_lfsr in events["skp"]:
            self.assertEqual(skp_lfsr, rx_lfsr)
        self.assertEqual(events["data"], sent_data[:len(events["data"])])
        self.assertGreaterEqual(len(events["data"]), len(sent_data) - 2)
        self.assertEqual([d for d in events["data"] if d != 0][:10], [d for p in packets for d in p][:10])

    def test_loopback(self):
        # Short: less TS1/TS2/packets, one SKP Ordered Set edit.
        self.loopback_test(edits={0: -4}, **RUN_TX_SHORT)

    @pytest.mark.slow
    def test_loopback_long(self):
        self.loopback_test()

if __name__ == "__main__":
    unittest.main()
