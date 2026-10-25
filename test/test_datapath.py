#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 TX/RX datapaths: Gen 1 word alignment, SKP Ordered Sets insertion/removal and datapaths,
Gen 2 (128b/132b) TX/RX against bit-serial reference models of the USB 3.2 rules (6.3.2
encoding/scrambling, 6.4.1.2 Ordered Sets/DC balance, 6.4.3.3 SKP Ordered Sets) and a Gen 2 TX ->
channel (bit offset, polarity inversion, bit slip, SKP add/remove) -> RX loopback."""

import random
import unittest
import functools

import pytest

from migen import *

from litex.gen import *

from usb3_pipe.common   import *
from usb3_pipe.coding   import Gen2BlockGearbox, Gen2BlockAligner
from usb3_pipe.datapath import RXWordAligner, RXSKPRemover, TXSKPInserter, RXDatapath, TXDatapath
from usb3_pipe.datapath import Gen2TX, Gen2RX

from test.common import ordered_set_words, words_to_symbols, symbols_to_words, stream_monitor
from test.model.gen2 import Gen2ReferenceDecoder, int_to_bits, bits_from_words, words_from_bits
from test.model.gen2 import popcount


class TestDatapath(unittest.TestCase):
    def test_rx_word_aligner(self):
        # On a re-alignment, the word before the COM run only contains the symbols preceding it.
        datas_input = [
            0x030201bc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0x0201bc07, 0x0605040b, 0x0a09080f, 0x0e0d0c13,
            0x9cbc1f01, 0x0300202f, 0x8d03a500, 0x00005352,
            0xbc878685, 0x848b8a89, 0x888f8e8d, 0x8c929190,
            0xbcbcbcbc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0xbcbcbc00, 0x070605bc, 0x0b0a0908, 0x0f0e0d0c,
            0xbcbc0000, 0x0706bcbc, 0x0b0a0908, 0x0f0e0d0c,
            0xbc000000, 0x07bcbcbc, 0x0b0a0908, 0x0f0e0d0c,
        ]
        ctrls_input = [
            0x1, 0x0, 0x0, 0x0,
            0x2, 0x0, 0x0, 0x0,
            0x4, 0x0, 0x0, 0x0,
            0x8, 0x0, 0x0, 0x0,
            0xf, 0x0, 0x0, 0x0,
            0xe, 0x1, 0x0, 0x0,
            0xc, 0x3, 0x0, 0x0,
            0x8, 0x7, 0x0, 0x0,
        ]
        datas_reference = [
            0x030201bc, 0x07060504, 0x0b0a0908, 0x070f0e0d,
            0x0b0201bc, 0x0f060504, 0x130a0908, 0x1f010e0d,
            0x202f9cbc, 0xa5000300, 0x53528d03, 0x87868500,
            0x8b8a89bc, 0x8f8e8d84, 0x92919088, 0x8c929190,
            0xbcbcbcbc, 0x07060504, 0x0b0a0908, 0x000f0e0d,
            0xbcbcbcbc, 0x08070605, 0x0c0b0a09, 0x00000f0e,
            0xbcbcbcbc, 0x09080706, 0x0d0c0b0a, 0x0000000f,
            0xbcbcbcbc, 0x0a090807, 0x0e0d0c0b, 0x0e0d0c0f,
        ]
        ctrls_reference = [
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0xf, 0x0, 0x0, 0x0,
            0xf, 0x0, 0x0, 0x0,
            0xf, 0x0, 0x0, 0x0,
            0xf, 0x0, 0x0, 0x0,
        ]

        def generator(dut):
            yield dut.sink.valid.eq(1)
            for data, ctrl in zip(datas_input, ctrls_input):
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield

        def checker(dut):
            dut.datas_errors = 0
            dut.ctrls_errors = 0
            yield dut.source.ready.eq(1)
            while not (yield dut.source.valid):
                yield
            for data, ctrl in zip(datas_reference, ctrls_reference):
                #print("{:08x} vs {:08x}".format((yield dut.source.data), data))
                if (yield dut.source.data) != data:
                    dut.datas_errors += 1
                if (yield dut.source.ctrl) != ctrl:
                    dut.ctrls_errors += 1
                yield

        dut = RXWordAligner()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.datas_errors, 0)
        self.assertEqual(dut.ctrls_errors, 0)

    def test_rx_word_aligner_keeps_alignment_on_k_after_data(self):
        # Symbol alignment is acquired on COM (TS1/TS2, USB 3.2 6.4.1.1) and must be kept in U0,
        # where Link Layer framing K-symbols (SHP/END/EPF...) follow data words but no COM is sent.
        rng  = random.Random(0)
        data = lambda n: [(rng.randrange(256), 0) for _ in range(n)]
        SHP, END, EPF = (K(27, 7), 1), (K(29, 7), 1), (K(23, 7), 1)
        aligned  = words_to_symbols(ordered_set_words(TS1))*4
        aligned += data(16) + [SHP, SHP, SHP, EPF] + data(16) + [END, END, END, EPF]
        aligned += data(16) + [SHP, SHP, SHP, EPF] + data(16)
        for offset in range(4):
            with self.subTest(offset=offset):
                wire = symbols_to_words(data(offset) + aligned + data(4 - offset))
                expected = symbols_to_words(aligned)

                def generator(dut):
                    for d, c in wire:
                        yield dut.sink.valid.eq(1)
                        yield dut.sink.data.eq(d)
                        yield dut.sink.ctrl.eq(c)
                        yield
                    yield dut.sink.valid.eq(0)

                def checker(dut):
                    dut.words = yield from stream_monitor(dut.source, len(wire) - 1, timeout=4*len(wire))

                dut = RXWordAligner()
                run_simulation(dut, [generator(dut), checker(dut)])
                start = dut.words.index(expected[0])
                self.assertEqual(dut.words[start:start + len(expected)], expected)

    def test_rx_skip_remover(self):
        datas_input = [
            0x0302013c, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0x02013c07, 0x0605040b, 0x0a09080f, 0x0e0d0c13,
            0x9c3c1f01, 0x0300202f, 0x8d03a500, 0x00005352,
            0x3c878685, 0x848b8a89, 0x888f8e8d, 0x8c929190,
            0xbc3c3c4a, 0x00bcbcbc, 0x4a4a4a00, 0x4a4a4a4a,
        ]
        ctrls_input = [
            0x1, 0x0, 0x0, 0x0,
            0x2, 0x0, 0x0, 0x0,
            0x4, 0x0, 0x0, 0x0,
            0x8, 0x0, 0x0, 0x0,
            0xe, 0x7, 0x0, 0x0,
        ]
        datas_reference = [
            0x04030201, 0x08070605, 0x0c0b0a09, 0x070f0e0d,
            0x040b0201, 0x080f0605, 0x0c130a09, 0x1f010e0d,
            0x00202f9c, 0x03a50003, 0x0053528d, 0x87868500,
            0x848b8a89, 0x888f8e8d, 0x8c929190,
            0xbcbcbc4a, 0x4a0000bc, 0x4a4a4a4a,
        ]
        ctrls_reference = [
            0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0, 0x0,
            0x0, 0x0, 0x0,
            0xe, 0x1, 0x0,
        ]

        def generator(dut):
            for data, ctrl in zip(datas_input, ctrls_input):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
                yield dut.sink.valid.eq(0)
                yield
            for i in range(128):
                yield

        def checker(dut):
            dut.datas_errors = 0
            dut.ctrls_errors = 0
            for data, ctrl in zip(datas_reference, ctrls_reference):
                while not (yield dut.source.valid):
                    yield
                yield dut.source.ready.eq(1)
                #print("{:08x} vs {:08x}".format((yield dut.source.data), data))
                if (yield dut.source.data) != data:
                    dut.datas_errors += 1
                if (yield dut.source.ctrl) != ctrl:
                    dut.ctrls_errors += 1
                yield
                yield dut.source.ready.eq(0)
                yield

        dut = RXSKPRemover()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.datas_errors, 0)
        self.assertEqual(dut.ctrls_errors, 0)

    def test_rx_skip_remover_d3c_not_removed(self):
        datas_input = [0x3c112233, 0x44556677]
        ctrls_input = [0x0,        0x0]
        datas_reference = datas_input
        ctrls_reference = ctrls_input

        def generator(dut):
            for data, ctrl in zip(datas_input, ctrls_input):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
            yield dut.sink.valid.eq(0)
            for i in range(32):
                yield

        def checker(dut):
            dut.datas_errors = 0
            dut.ctrls_errors = 0
            yield dut.source.ready.eq(1)
            yield
            k = 0
            while k < len(datas_reference):
                if (yield dut.source.valid):
                    if (yield dut.source.data) != datas_reference[k]:
                        dut.datas_errors += 1
                    if (yield dut.source.ctrl) != ctrls_reference[k]:
                        dut.ctrls_errors += 1
                    k += 1
                yield

        dut = RXSKPRemover()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.datas_errors, 0)
        self.assertEqual(dut.ctrls_errors, 0)

    def test_rx_skip_remover_multi_skp(self):
        datas_input = [
            0x3c3c3c3c,  # 4x SKP K.
            0x11223344,
            0x3c00113c,  # 2x SKP K mixed with data bytes.
            0x55667788,
        ]
        ctrls_input = [
            0xf,
            0x0,
            0x9,         # lane0 + lane3 are K.
            0x0,
        ]

        datas_reference = [
            0x11223344,
            0x77880011,
        ]
        ctrls_reference = [
            0x0,
            0x0,
        ]

        def generator(dut):
            yield dut.sink.valid.eq(1)
            k = 0
            while k < len(datas_input):
                yield dut.sink.data.eq(datas_input[k])
                yield dut.sink.ctrl.eq(ctrls_input[k])
                yield
                if (yield dut.sink.ready):
                    k += 1
            yield dut.sink.valid.eq(0)
            for i in range(128):
                yield
            dut.run = False

        def checker(dut):
            dut.datas_errors = 0
            dut.ctrls_errors = 0
            k = 0
            cycles = 0
            while dut.run and (cycles < 4096):
                yield dut.source.ready.eq((cycles & 0x1) == 0)  # Backpressure.
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    if (yield dut.source.data) != datas_reference[k]:
                        dut.datas_errors += 1
                    if (yield dut.source.ctrl) != ctrls_reference[k]:
                        dut.ctrls_errors += 1
                    k += 1
                    if k == len(datas_reference):
                        break
                cycles += 1
            self.assertEqual(k, len(datas_reference))

        dut = RXSKPRemover()
        dut.run = True
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.datas_errors, 0)
        self.assertEqual(dut.ctrls_errors, 0)

    def test_tx_skip_inserter(self):
        def generator(dut):
            for i in range(256):
                yield dut.sink.valid.eq(1)
                yield

        @passive
        def checker(dut):
            yield dut.source.ready.eq(1)
            dut.ctrl_errors = 0
            dut.data_errors = 0
            while not (yield dut.source.valid):
                yield
            yield
            yield
            while True:
                for i in range(175):
                    yield
                if (yield dut.source.ctrl) != 0b1111:
                    dut.ctrl_errors += 1
                if (yield dut.source.data) != 0x3c3c3c3c:
                    dut.data_errors += 1
                yield

        dut = TXSKPInserter()
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(dut.data_errors, 0)
        self.assertEqual(dut.ctrl_errors, 0)

    def test_datapath_loopback(self, nwords=512):
        prng  = random.Random(42)
        datas = [prng.randrange(2**32) for _ in range(nwords)]
        ctrls = [prng.randrange(2**4)  for _ in range(nwords)]

        def remove_skp(datas, ctrls):
            _ctrls = []
            for data, ctrl in zip(datas, ctrls):
                for i in range(4):
                    if ((((data >> (8*i)) & 0xff) == 0x3c) and
                        (((ctrl >> i) & 0x1) == 1)):
                        ctrl &= ~(1<<i)
                _ctrls.append(ctrl)
            return datas, _ctrls

        datas, ctrls = remove_skp(datas, ctrls)

        class DUT(Module):
            def __init__(self):
                self.submodules.tx = TXDatapath("serdes")
                self.submodules.rx = RXDatapath("serdes")
                self.comb += self.rx.word_aligner.enable.eq(0)
                self.comb += self.tx.source.connect(self.rx.sink)

        def generator(dut):
            for data, ctrl in zip(datas, ctrls):
                yield dut.tx.sink.valid.eq(1)
                yield dut.tx.sink.data.eq(data)
                yield dut.tx.sink.ctrl.eq(ctrl)
                yield
                while not (yield dut.tx.sink.ready):
                    yield
                yield dut.tx.sink.valid.eq(0)

        @passive
        def checker(dut):
            dut.data_errors = 0
            dut.ctrl_errors = 0
            yield dut.rx.source.ready.eq(1)
            for data, ctrl in zip(datas, ctrls):
                while not (yield dut.rx.source.valid):
                    yield
                if (yield dut.rx.source.data) != data:
                    dut.data_errors += 1
                if (yield dut.rx.source.ctrl) != ctrl:
                    dut.ctrl_errors += 1
                yield

        dut = DUT()
        run_simulation(dut,
            generators = [generator(dut), checker(dut)],
            clocks     = {"sys": 1e9/133e6, "serdes": 1e9/125e6},
        )
        self.assertEqual(dut.data_errors, 0)
        self.assertEqual(dut.ctrl_errors, 0)

    def test_tx_skip_inserter_payload_integrity(self):
        nwords = 512

        def generator(dut):
            for i in range(nwords):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(0x11223300 + i)
                yield dut.sink.ctrl.eq(0)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for i in range(256):
                yield

        def checker(dut):
            yield dut.source.ready.eq(1)
            yield
            k = 0
            while k < nwords:
                if (yield dut.source.valid):
                    data = (yield dut.source.data)
                    ctrl = (yield dut.source.ctrl)
                    if ctrl == 0b1111 and data == 0x3c3c3c3c:
                        yield
                        continue
                    self.assertEqual(data, 0x11223300 + k)
                    self.assertEqual(ctrl, 0)
                    k += 1
                yield

        dut = TXSKPInserter()
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_tx_skip_inserter_no_skp_in_packet(self):
        def generator(dut):
            yield dut.source.ready.eq(1)
            yield

            # One "packet" of 512 words.
            for i in range(512):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(0x11223300 + i)
                yield dut.sink.ctrl.eq(0)
                yield dut.sink.first.eq(1 if i == 0 else 0)
                yield dut.sink.last.eq(1 if i == 511 else 0)
                yield
                while not (yield dut.sink.ready):
                    yield

            # Then idle a bit to allow SKPs to drain.
            yield dut.sink.valid.eq(0)
            for _ in range(512):
                yield
            dut.run = False

        def checker(dut):
            skp_seen_in_packet = 0
            in_packet = 0
            yield
            while dut.run:
                if (yield dut.source.valid) and (yield dut.source.ready):
                    data = (yield dut.source.data)
                    ctrl = (yield dut.source.ctrl)
                    if (yield dut.source.first):
                        in_packet = 1
                    if in_packet:
                        if ctrl == 0b1111 and data == 0x3c3c3c3c:
                            skp_seen_in_packet += 1
                    if (yield dut.source.last):
                        in_packet = 0
                yield
            self.assertEqual(skp_seen_in_packet, 0)

        dut = TXSKPInserter()
        dut.run = True
        run_simulation(dut, [generator(dut), checker(dut)])

    def test_datapath_loopback_with_skps(self, nwords=128):
        prng  = random.Random(42)
        datas = [prng.randrange(2**32) for _ in range(nwords)]
        ctrls = [0 for _ in range(nwords)]  # Payload only.

        class DUT(Module):
            def __init__(self):
                self.submodules.tx = TXDatapath("serdes")
                self.submodules.rx = RXDatapath("serdes")
                self.comb += self.rx.word_aligner.enable.eq(0)
                self.comb += self.tx.source.connect(self.rx.sink)

        def generator(dut):
            for data, ctrl in zip(datas, ctrls):
                yield dut.tx.sink.valid.eq(1)
                yield dut.tx.sink.data.eq(data)
                yield dut.tx.sink.ctrl.eq(ctrl)
                yield
                while not (yield dut.tx.sink.ready):
                    yield
                yield dut.tx.sink.valid.eq(0)

            # Flush word (helps drain aligner/windowing paths).
            yield dut.tx.sink.valid.eq(1)
            yield dut.tx.sink.data.eq(0)
            yield dut.tx.sink.ctrl.eq(0)
            yield
            while not (yield dut.tx.sink.ready):
                yield
            yield dut.tx.sink.valid.eq(0)

            for i in range(256):
                yield

        @passive
        def checker(dut):
            dut.data_errors = 0
            dut.ctrl_errors = 0
            yield dut.rx.source.ready.eq(1)
            for data, ctrl in zip(datas, ctrls):
                while not (yield dut.rx.source.valid):
                    yield
                if (yield dut.rx.source.data) != data:
                    dut.data_errors += 1
                if (yield dut.rx.source.ctrl) != ctrl:
                    dut.ctrl_errors += 1
                yield

        dut = DUT()
        run_simulation(dut,
            generators = [generator(dut), checker(dut)],
            clocks     = {"sys": 1e9/133e6, "serdes": 1e9/125e6},
        )
        self.assertEqual(dut.data_errors, 0)
        self.assertEqual(dut.ctrl_errors, 0)

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

class TestGen2Datapath(unittest.TestCase):
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
