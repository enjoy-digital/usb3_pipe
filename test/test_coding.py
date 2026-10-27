#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""128b/132b (Gen 2) block gearbox/aligner (6.3.2, 6.4.1.2.4) against bit-serial reference models."""

import os
import re
import random
import shutil
import tempfile
import subprocess
import unittest

from migen import *

from usb3_pipe.common import *
from usb3_pipe.coding import Gen2BlockGearbox, Gen2BlockAligner, popcount

from test.model.gen2 import int_to_bits, bits_from_words, words_from_bits

# Gen 2 --------------------------------------------------------------------------------------------

class TestGen2Coding(unittest.TestCase):
    def gearbox_aligner_roundtrip_test(self, data_width=32):
        # Blocks (SYNC first, data/control, SKP Ordered Sets of 0-36 SKP Symbols) -> gearbox -> bit
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
        tx = Gen2BlockGearbox(data_width)
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
        self.assertGreaterEqual(data_width*len(words), len(ref) - (data_width - 1))
        self.assertEqual(bits_from_words(words, data_width)[:len(ref)], ref[:data_width*len(words)])

        # Channel: random bit offset, a Block Header single bit error.
        offset = 37
        bits   = [rng.getrandbits(1) for _ in range(offset)] + bits + [0]*64
        bits[offset + 132*3 + 1] ^= 1

        # RX aligner.
        rx  = Gen2BlockAligner(data_width)
        out = []
        skp = []
        def rx_generator(dut):
            for w in words_from_bits(bits, data_width):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(w)
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
        def rx_checker(dut):
            yield dut.source.ready.eq(1)
            for _ in range(len(bits)//data_width + 64):
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

    def test_gearbox_aligner_roundtrip(self):
        self.gearbox_aligner_roundtrip_test(data_width=32)

    def test_gearbox_aligner_roundtrip_40(self):
        # Hardware width (40-bit words at 250MHz).
        self.gearbox_aligner_roundtrip_test(data_width=40)

    def test_gearbox_aligner_roundtrip_64(self):
        self.gearbox_aligner_roundtrip_test(data_width=64)

    @unittest.skipIf(shutil.which("yosys") is None, "yosys not found")
    def test_popcount_verilog(self):
        # popcount compared to constants in the generated Verilog (Verilog expression widths are not
        # reproduced by the Migen simulator): evaluated with yosys for all 4-bit values.
        from migen.fhdl import verilog
        i   = Signal(4)
        le1 = Signal()
        gt1 = Signal()
        cnt = Signal(3)
        m   = Module()
        m.comb += [
            le1.eq(popcount(i) <= 1),
            gt1.eq(popcount(i) > 1),
            cnt.eq(popcount(i)),
        ]
        with tempfile.TemporaryDirectory() as d:
            f = os.path.join(d, "popcount.v")
            verilog.convert(m, ios={i, le1, gt1, cnt}, name="popcount").write(f)
            script = f"read_verilog {f}; prep -top popcount; " + "; ".join(
                f"eval -set {i.backtrace[-1][0]} {v} -show {le1.backtrace[-1][0]},{gt1.backtrace[-1][0]},{cnt.backtrace[-1][0]}"
                for v in range(16))
            log = subprocess.run(["yosys", "-p", script], capture_output=True, text=True, check=True).stdout
        # "Eval result: { \\le1 \\gt1 \\cnt } = 5'xyzzz."
        results = [int(x, 2) for x in re.findall(r"Eval result: .* = 5'([01]+)", log)]
        values  = sum([[r >> 4, (r >> 3) & 1, r & 0b111] for r in results], [])
        expected = []
        for v in range(16):
            n = bin(v).count("1")
            expected += [int(n <= 1), int(n > 1), n]
        self.assertEqual(values, expected)

