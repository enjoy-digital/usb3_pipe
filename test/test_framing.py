#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common  import GEN2_IDLE
from usb3_pipe.framing import Gen2TXFramer, Gen2TXPacker, Gen2RXFraming, gen2_tx_unit_layout
from usb3_pipe.framing import Gen2TXUnitsWidener, WordsNarrower
from usb3_pipe.framing import Gen2TXBlocksPacker, Gen2TXBlocksReader, gen2_tx_blocks_layout
from usb3_pipe.framing import SHP, SDP, EDB, END, SLC, EPF

# Helpers ------------------------------------------------------------------------------------------

def k_word(k):
    """Framing Ordered Set word (Symbol 0 on data[0:8]): 3 K Symbols then EPF."""
    return (k | (k << 8) | (k << 16) | (EPF << 24), 0b1111)

def words_from_bytes(symbols):
    """[(byte, k)] to words (Symbol 0 on data[0:8]), last word padded with 00h data."""
    symbols = symbols + [(0, 0)]*((-len(symbols)) % 4)
    words = []
    for i in range(0, len(symbols), 4):
        data = sum(b << (8*j) for j, (b, k) in enumerate(symbols[i:i+4]))
        ctrl = sum(k << j for j, (b, k) in enumerate(symbols[i:i+4]))
        words.append((data, ctrl))
    return words

def header_packet(rng, dw0_type=0x00, length=None):
    dws = [rng.getrandbits(32) for _ in range(3)] + [rng.getrandbits(32)]
    dws[0] = (dws[0] & ~0x1f) | dw0_type
    if length is not None:
        dws[1] = (dws[1] & 0xffff) | (length << 16)
    return [k_word(SHP)] + [(dw, 0) for dw in dws]

def link_command(rng):
    lcw = rng.getrandbits(16)
    # LGOOD_n/LCRD_x (Gen2RXFraming source_lc) 1/3 of the time.
    lcw = rng.choice([lcw, lcw, (lcw & ~0x7f0), (lcw & ~0x7f8) | 0x080])
    return [k_word(SLC), (lcw | (lcw << 16), 0)]

def is_fast_lc(unit):
    """LGOOD_n/LCRD_x Link Command (Gen2RXFraming source_lc)."""
    if unit[0] != k_word(SLC):
        return False
    lcw = unit[1][0] & 0xffff
    return ((lcw & 0x7f0) == 0x000) or ((lcw & 0x7f8) == 0x080)

def split_units(units, lc_enable=True):
    """Expected Gen2RXFraming outputs: words (source), Link Command words (source_lc)."""
    fast  = lambda u: lc_enable and is_fast_lc(u)
    words = [w for u in units if not fast(u) for w in u]
    lcs   = [u[1][0] for u in units if fast(u)]
    return words, lcs

def dpp(rng, length, nullified=False, aborted=False):
    if nullified:
        return [k_word(EDB)]
    data = [(rng.getrandbits(8), 0) for _ in range(length + 4)] # Data + CRC-32.
    end  = [(EDB if aborted else END, 1)]*3 + [(EPF, 1)] # DPPABORT/DPPEND.
    return [k_word(SDP)] + words_from_bytes(data + end)

def random_units(rng, n, short=False):
    units = []
    for _ in range(n):
        kind = rng.choice(["hp", "lc", "dp", "dp", "nullified", "aborted"])
        if kind == "hp":
            units.append(header_packet(rng))
        elif kind == "lc":
            units.append(link_command(rng))
        else:
            length = rng.choice([0, 1, 2, 3, 4, 5, 13] + ([] if short else [64, 255, 1024]))
            units.append(header_packet(rng, dw0_type=0x08, length=length) +
                dpp(rng, length, nullified=(kind == "nullified"), aborted=(kind == "aborted")))
    return units

# Loopback -----------------------------------------------------------------------------------------

class Gen2FramingLoopback(Module):
    """Gen2TXFramer -> units FIFO (store and forward) -> Gen2TXPacker -> Data Blocks (Gen2TX model:
    a block every 3 cycles at most, Idle Data Blocks when no block) -> Gen2RXFraming.

    slots: block requests from the slot Signal (driven by the test) instead of every 3 cycles.
    nbytes=32: framer entries grouped by 32 bytes (Gen2TXUnitsWidener), Gen2TXBlocksPacker, blocks
    buffer and Gen2TXBlocksReader, a block every 1.65 cycles (80-bit Gen2TX: 20 blocks per 33
    cycles), blocks paired for the RX framing (32 bytes per cycle)."""
    def __init__(self, slots=False, nbytes=8):
        self.framer  = framer  = Gen2TXFramer()
        self.fifo    = fifo    = stream.SyncFIFO(gen2_tx_unit_layout(nbytes), 512, buffered=True)
        self.packer  = packer  = {8: Gen2TXPacker(8), 32: Gen2TXBlocksPacker()}[nbytes]
        self.rx      = rx      = Gen2RXFraming(nbytes)
        self.submodules += framer, fifo, packer, rx
        if nbytes >= 16:
            widener = Gen2TXUnitsWidener(nbytes)
            self.submodules += widener
            self.comb += [
                framer.source.connect(widener.sink),
                widener.source.connect(fifo.sink),
            ]
        else:
            self.comb += framer.source.connect(fifo.sink)
        self.comb += fifo.source.connect(packer.sink)
        units_written = Signal(16)
        units_started = Signal(16)
        self.sync += [
            If(fifo.sink.valid & fifo.sink.ready & fifo.sink.last, units_written.eq(units_written + 1)),
            If(packer.unit_end, units_started.eq(units_started + 1)),
        ]
        self.comb += packer.units_available.eq(units_written != units_started)
        # SKP Ordered Sets pending periodically (units ended in their block).
        skp_timer = Signal(6)
        self.sync += skp_timer.eq(skp_timer + 1)
        self.comb += packer.skp_pending.eq(skp_timer[5])

        # 32 bytes: blocks entries buffer, chains read once complete (written count delayed: buffer
        # latency).
        blocks = packer.source
        if nbytes == 32:
            blocks_fifo = stream.SyncFIFO(gen2_tx_blocks_layout(2), 128, buffered=True)
            reader      = Gen2TXBlocksReader()
            self.submodules += blocks_fifo, reader
            chains_written = Signal(16)
            chains_read    = Signal(16)
            chains_d       = [Signal(16) for _ in range(4)]
            self.sync += [
                If(blocks_fifo.sink.valid & blocks_fifo.sink.ready & blocks_fifo.sink.last,
                    chains_written.eq(chains_written + 1)),
                If(reader.chain_end, chains_read.eq(chains_read + 1)),
                chains_d[0].eq(chains_written),
                [chains_d[i].eq(chains_d[i - 1]) for i in range(1, 4)],
            ]
            self.comb += [
                packer.source.connect(blocks_fifo.sink),
                blocks_fifo.source.connect(reader.sink),
                reader.chains_available.eq(chains_d[-1] != chains_read),
            ]
            blocks = reader.source

        # Gen2TX model: block every 3 cycles (1.65 with 16/32 bytes), idle block when none.
        self.block       = block = Signal(128)
        self.block_valid = block_valid = Signal()
        self.slot = slot = Signal()
        if nbytes >= 16:
            # 132-bit blocks from 80-bit words: a block when the bits count reaches 132.
            acc = Signal(8)
            self.sync += If(acc + 80 >= 132, acc.eq(acc + 80 - 132)).Else(acc.eq(acc + 80))
            if not slots:
                self.comb += slot.eq(acc + 80 >= 132)
        else:
            timer = Signal(2)
            self.sync += If(timer == 2, timer.eq(0)).Else(timer.eq(timer + 1))
            if not slots:
                self.comb += slot.eq(timer == 2)
        idle = int.from_bytes(bytes([GEN2_IDLE])*16, "little")
        self.comb += [
            blocks.ready.eq(slot),
        ]
        self.sync += [
            block_valid.eq(slot),
            If(slot,
                block.eq(Mux(blocks.valid, blocks.data[:128], idle))
            )
        ]
        # RX FIFO (Gen2LinkRX blocks buffer: the RX framing stalls 1 cycle on each packet start).
        rx_fifo = stream.SyncFIFO([("data", 8*nbytes)], 32 if nbytes >= 16 else 8)
        self.submodules += rx_fifo
        if nbytes == 32:
            # Blocks pairs.
            second = Signal()
            first  = Signal(128)
            self.sync += If(block_valid, second.eq(~second), first.eq(block))
            self.comb += [
                rx_fifo.sink.valid.eq(block_valid & second),
                rx_fifo.sink.data.eq(Cat(first, block)),
            ]
        else:
            # Blocks -> 8 bytes per cycle (2 cycles per block, then 1 idle cycle).
            half = Signal()
            hold = Signal(128)
            self.sync += [
                If(block_valid, hold.eq(block), half.eq(1)).Elif(half, half.eq(0)),
            ]
            self.comb += [
                rx_fifo.sink.valid.eq(block_valid | half),
                rx_fifo.sink.data.eq(Mux(block_valid, block[0:64], hold[64:128])),
            ]
        self.comb += [
            rx_fifo.source.connect(rx.sink),
            rx.source.ready.eq(1),
            rx.source_lc.ready.eq(1),
            rx.lc_enable.eq(1),
        ]
        self.rx_overflow = Signal()
        self.sync += If(rx_fifo.sink.valid & ~rx_fifo.sink.ready, self.rx_overflow.eq(1))

class TestGen2Framing(unittest.TestCase):
    def loopback_test(self, seed, n_units=24, slots=False, short=False, pairs=False, nbytes=8,
        gaps=0.2):
        rng   = random.Random(seed)
        units = random_units(rng, n_units, short=short)
        words = [w for u in units for w in u]
        out   = []
        lcs   = []
        overflow = []
        dut   = Gen2FramingLoopback(slots=slots, nbytes=nbytes)
        # Words pairs: 1 or 2 words per entry (pairs=True), else 1.
        entries = []
        i = 0
        while i < len(words):
            n = 2 if (pairs and (i + 1) < len(words) and rng.random() < 0.7) else 1
            entries.append(words[i:i+n])
            i += n
        def generator(dut):
            for entry in entries:
                yield dut.framer.sink.valid.eq(1)
                yield dut.framer.sink.data.eq(sum(d << (32*j) for j, (d, c) in enumerate(entry)))
                yield dut.framer.sink.ctrl.eq(sum(c << (4*j) for j, (d, c) in enumerate(entry)))
                yield dut.framer.sink.words.eq(len(entry))
                yield
                while not (yield dut.framer.sink.ready):
                    yield
                # Random gaps between words (core TX).
                if rng.random() < gaps:
                    yield dut.framer.sink.valid.eq(0)
                    yield
            yield dut.framer.sink.valid.eq(0)
        def checker(dut):
            for i in range(len(words)*8 + 2000):
                if (yield dut.rx.source.valid):
                    data  = (yield dut.rx.source.data)
                    ctrl  = (yield dut.rx.source.ctrl)
                    nwords = (yield dut.rx.source.words)
                    for w in range(nwords):
                        out.append(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf))
                if (yield dut.rx.source_lc.valid):
                    lcs.append((yield dut.rx.source_lc.data))
                if (yield dut.rx_overflow) and not overflow:
                    overflow.append(i)
                yield
        @passive
        def slots_generator(dut):
            # Block requests every 3 to 5 cycles (random phase relative to the units).
            while True:
                yield dut.slot.eq(1)
                yield
                yield dut.slot.eq(0)
                for _ in range(rng.choice([2, 2, 3, 4])):
                    yield
        generators = [generator(dut), checker(dut)] + ([slots_generator(dut)] if slots else [])
        run_simulation(dut, generators)
        self.assertEqual((out, lcs), split_units(units))
        self.assertEqual(overflow, [])


    def test_loopback(self):
        for seed in range(3):
            self.loopback_test(seed)

    def test_loopback_block_slots(self):
        # Data Blocks requested at random intervals: a unit must never be split by a Logical Idle
        # Data Block (block requested while the packer is starting the next unit).
        # Short units: many unit boundaries.
        for seed in range(2):
            self.loopback_test(seed, n_units=200, slots=True, short=True)

    def test_loopback_pairs(self):
        # Words pairs input (64-bit Link Layer words): DPP data 8 bytes per cycle.
        for seed in range(3):
            self.loopback_test(seed, pairs=True)

    def test_loopback_32_bytes(self):
        # 32 bytes per cycle (packer/RX framing at the Link Layer frequency: 2 Data Blocks): chains
        # read once complete from the blocks buffer, no Logical Idle Data Block inside a unit.
        for seed in range(4):
            self.loopback_test(seed, pairs=True, nbytes=32, gaps=1)

    def test_loopback_32_bytes_short(self):
        self.loopback_test(0, n_units=120, pairs=True, short=True, nbytes=32, gaps=0)

if __name__ == "__main__":
    unittest.main()

# Gen2LinkTX (sys -> tx clock domains) ---------------------------------------------------------------

UNITS_DELAY = 8

class Gen2LinkTXLoopback(Module):
    """Gen2LinkTX (sys at half the tx clock rate) -> Data Blocks (Gen2TX model in the tx clock domain,
    a block every 3 or 4 cycles, Idle Data Blocks when no block) -> Gen2RXFraming (tx clock domain)."""
    def __init__(self):
        from usb3_pipe.framing import Gen2LinkTX
        self.link_tx = link_tx = Gen2LinkTX(units_delay=UNITS_DELAY)
        rx = ClockDomainsRenamer("tx")(Gen2RXFraming())
        self.rx = rx
        self.submodules += link_tx, rx
        timer = Signal(4)
        slot  = Signal()
        self.sync.tx += If(timer == 9, timer.eq(0)).Else(timer.eq(timer + 1))
        self.comb += slot.eq((timer == 2) | (timer == 5) | (timer == 9)) # 3/3/4 cycles: 40-bit gearbox.
        idle  = int.from_bytes(bytes([GEN2_IDLE])*16, "little")
        block = Signal(128)
        block_valid = Signal()
        self.comb += link_tx.source.ready.eq(slot)
        self.sync.tx += [
            block_valid.eq(slot),
            If(slot, block.eq(Mux(link_tx.source.valid, link_tx.source.data, idle))),
        ]
        half = Signal()
        hold = Signal(128)
        self.sync.tx += If(block_valid, hold.eq(block), half.eq(1)).Elif(half, half.eq(0))
        rx_fifo = ClockDomainsRenamer("tx")(stream.SyncFIFO([("data", 64)], 8))
        self.submodules += rx_fifo
        self.comb += [
            rx_fifo.sink.valid.eq(block_valid | half),
            rx_fifo.sink.data.eq(Mux(block_valid, block[0:64], hold[64:128])),
            rx_fifo.source.connect(rx.sink),
            rx.source.ready.eq(1),
            rx.source_lc.ready.eq(1),
            rx.lc_enable.eq(1),
        ]

class TestGen2LinkTX(unittest.TestCase):
    def test_link_tx_cdc(self):
        rng   = random.Random(7)
        units = random_units(rng, 24)
        words = [w for u in units for w in u]
        out   = []
        lcs   = []
        dut   = Gen2LinkTXLoopback()
        def generator(dut):
            for data, ctrl in words:
                yield dut.link_tx.sink.valid.eq(1)
                yield dut.link_tx.sink.data.eq(data)
                yield dut.link_tx.sink.ctrl.eq(ctrl)
                yield
                while not (yield dut.link_tx.sink.ready):
                    yield
                if rng.random() < 0.3:
                    yield dut.link_tx.sink.valid.eq(0)
                    for _ in range(rng.randrange(1, 4)):
                        yield
            yield dut.link_tx.sink.valid.eq(0)
        @passive
        def checker(dut):
            while True:
                if (yield dut.rx.source.valid):
                    data   = (yield dut.rx.source.data)
                    ctrl   = (yield dut.rx.source.ctrl)
                    nwords = (yield dut.rx.source.words)
                    for w in range(nwords):
                        out.append(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf))
                if (yield dut.rx.source_lc.valid):
                    lcs.append((yield dut.rx.source_lc.data))
                yield
        def timeout(dut):
            for _ in range(len(words)*4 + 2000):
                yield
        run_simulation(dut, {"sys": [generator(dut), timeout(dut)], "tx": [checker(dut)]},
            clocks={"sys": 8, "tx": 4})
        self.assertEqual((out, lcs), split_units(units))

class Gen2LinkLoopback32(Module):
    """Gen2LinkTX (32 bytes, sys at half the tx clock rate) -> Data Blocks (Gen2TX model in the tx
    clock domain: a block every 1.65 cycles, Idle Data Blocks when no block) -> Gen2LinkRX (32 bytes,
    rx = tx clock domain, words pairs in sys)."""
    def __init__(self):
        from usb3_pipe.framing import Gen2LinkTX, Gen2LinkRX
        self.link_tx = link_tx = Gen2LinkTX(units_delay=UNITS_DELAY, link_words=2, nbytes=32)
        self.link_rx = link_rx = ClockDomainsRenamer({"rx": "tx"})(Gen2LinkRX(link_words=2,
            nbytes=32))
        self.submodules += link_tx, link_rx
        acc  = Signal(8)
        slot = Signal()
        self.sync.tx += If(acc + 80 >= 132, acc.eq(acc + 80 - 132)).Else(acc.eq(acc + 80))
        self.comb += slot.eq(acc + 80 >= 132)
        idle = int.from_bytes(bytes([GEN2_IDLE])*16, "little")
        self.comb += link_tx.source.ready.eq(slot)
        self.sync.tx += [
            link_rx.sink.valid.eq(slot),
            link_rx.sink.data.eq(Mux(link_tx.source.valid, link_tx.source.data, idle)),
        ]
        self.comb += [
            link_rx.data_stream.eq(1),
            link_rx.rx_ready.eq(1),
            link_rx.lc_enable.eq(1),
            link_rx.source.ready.eq(1),
            link_rx.source_lc.ready.eq(1),
        ]

class TestGen2Link32(unittest.TestCase):
    def test_link_loopback_32_bytes(self):
        rng   = random.Random(3)
        units = random_units(rng, 24)
        words = [w for u in units for w in u]
        out   = []
        lcs   = []
        dut   = Gen2LinkLoopback32()
        def generator(dut):
            i = 0
            while i < len(words):
                n = 2 if (i + 1) < len(words) and rng.random() < 0.7 else 1
                entry = words[i:i+n]
                yield dut.link_tx.sink.valid.eq(1)
                yield dut.link_tx.sink.data.eq(sum(d << (32*j) for j, (d, c) in enumerate(entry)))
                yield dut.link_tx.sink.ctrl.eq(sum(c << (4*j) for j, (d, c) in enumerate(entry)))
                yield dut.link_tx.sink.words.eq(n)
                yield
                while not (yield dut.link_tx.sink.ready):
                    yield
                i += n
            yield dut.link_tx.sink.valid.eq(0)
        @passive
        def checker(dut):
            while True:
                if (yield dut.link_rx.source.valid):
                    data   = (yield dut.link_rx.source.data)
                    ctrl   = (yield dut.link_rx.source.ctrl)
                    nwords = (yield dut.link_rx.source.words)
                    for w in range(nwords):
                        out.append(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf))
                if (yield dut.link_rx.source_lc.valid):
                    lcs.append((yield dut.link_rx.source_lc.data))
                yield
        overflow = []
        def timeout(dut):
            # Long enough to fill the RX buffers without a Link Layer rate RX path (blocks at the
            # line rate, Idle Data Blocks included).
            for _ in range(len(words)*4 + 6000):
                yield
            overflow.append((yield dut.link_rx.overflow))
        run_simulation(dut, {"sys": [generator(dut), timeout(dut), checker(dut)]},
            clocks={"sys": 8, "tx": 4})
        self.assertEqual((out, lcs), split_units(units))
        self.assertEqual(overflow, [0])

# Gen2TXBlocksPacker/Gen2TXBlocksReader: random block requests, no Idle Symbols inside a unit ---

class Gen2TXBlocksBench(Module):
    def __init__(self):
        self.submodules.packer = packer = Gen2TXBlocksPacker()
        self.submodules.fifo   = fifo   = stream.SyncFIFO(gen2_tx_blocks_layout(2), 128, buffered=True)
        self.submodules.reader = reader = Gen2TXBlocksReader()
        self.slot  = Signal() # Block request (Gen2TX).
        self.taken = Signal() # Block request of the previous cycle.
        self.valid = Signal()
        self.data  = Signal(128)
        self.hold  = Signal() # Buffer writes held (slower write side).
        # Complete chains in the buffer (written count delayed: buffer latency).
        written = Signal(16)
        read    = Signal(16)
        delayed = [Signal(16) for _ in range(4)]
        self.sync += [
            If(fifo.sink.valid & fifo.sink.ready & fifo.sink.last, written.eq(written + 1)),
            If(reader.chain_end, read.eq(read + 1)),
            delayed[0].eq(written),
            [delayed[i].eq(delayed[i - 1]) for i in range(1, 4)],
        ]
        self.comb += [
            packer.units_available.eq(1),
            packer.source.connect(fifo.sink, omit={"valid", "ready"}),
            fifo.sink.valid.eq(packer.source.valid & ~self.hold),
            packer.source.ready.eq(fifo.sink.ready & ~self.hold),
            fifo.source.connect(reader.sink),
            reader.chains_available.eq(delayed[-1] != read),
            reader.source.ready.eq(self.slot),
        ]
        self.sync += [
            self.taken.eq(self.slot),
            self.valid.eq(reader.source.valid),
            self.data.eq(reader.source.data),
        ]

class TestGen2TXBlocks(unittest.TestCase):
    def test_random_requests(self):
        rng   = random.Random(0)
        # Units bytes (no Idle Symbol value), 32 bytes entries (entries before the last one full).
        units = [[rng.randrange(GEN2_IDLE) for _ in range(rng.choice([1, 4, 8, 12, 16, 20, 24, 40, 100]))]
            for _ in range(80)]
        entries = [(u[i:i+32], (i + 32) >= len(u)) for u in units for i in range(0, len(u), 32)]
        dut    = Gen2TXBlocksBench()
        blocks = []
        def generator(dut):
            for data, last in entries:
                yield dut.packer.sink.valid.eq(1)
                yield dut.packer.sink.data.eq(int.from_bytes(bytes(data + [GEN2_IDLE]*(32 - len(data))), "little"))
                yield dut.packer.sink.bytes.eq(len(data))
                yield dut.packer.sink.last.eq(last)
                yield
                while not (yield dut.packer.sink.ready):
                    yield
            yield dut.packer.sink.valid.eq(0)
        def requests(dut):
            for _ in range(12000):
                yield dut.slot.eq(rng.random() < 0.6)
                yield dut.hold.eq(rng.random() < 0.75)
                yield dut.packer.skp_pending.eq(rng.random() < 0.3)
                yield
                if (yield dut.taken):
                    # Logical Idle Data Block when no block.
                    data = (yield dut.data) if (yield dut.valid) else int.from_bytes(
                        bytes([GEN2_IDLE])*16, "little")
                    blocks.append(data.to_bytes(16, "little"))
        run_simulation(dut, [generator(dut), requests(dut)])
        stream = b"".join(blocks)
        self.assertEqual([b for b in stream if b != GEN2_IDLE], [b for u in units for b in u])
        # Idle Symbols only between units.
        owner = [n for n, u in enumerate(units) for _ in u]
        k, idle = 0, False
        for b in stream:
            if b == GEN2_IDLE:
                idle = True
                continue
            if idle and k > 0:
                self.assertNotEqual(owner[k], owner[k - 1])
            idle = False
            k += 1

# Gen2RXFraming: packets at any byte position, random Idle Symbols between them ----------------------

def gen2_unit_bytes(unit):
    """Gen 2 bytes of a unit of Link Layer words (as transmitted by a host: framing bytes, DPHSTART +
    Length Field Replicas for the DPHs, bytes after DPPEND/DPPABORT removed)."""
    from usb3_pipe.framing import GEN2_FRAMING_BYTES, GEN2_DPHP
    symbols = []
    for data, ctrl in unit:
        for j in range(4):
            symbols.append(((data >> (8*j)) & 0xff, (ctrl >> j) & 1))
    out    = []
    is_dph = (symbols[0] == (SHP, 1)) and ((symbols[4][0] & 0x1f) == 0x08)
    i      = 0
    while i < len(symbols):
        b, k = symbols[i]
        if k:
            b = GEN2_FRAMING_BYTES[b]
            if is_dph and i < 3:
                b = GEN2_DPHP
        out.append(b)
        if is_dph and i == 19:
            length = symbols[10][0] | (symbols[11][0] << 8)
            out += [length & 0xff, length >> 8]*2
        # DPP end: EPF after END/EDB.
        if k and symbols[i][0] == EPF and i > 3 and (i >= 24 or not is_dph):
            if i > 23 or not is_dph:
                break
        i += 1
    return out

class TestGen2RXFraming(unittest.TestCase):
    def test_rx_framing_idle_gaps(self):
        for lc_enable in [0, 1]:
            self.rx_framing_idle_gaps(lc_enable)

    def test_rx_framing_idle_gaps_16_bytes(self):
        for lc_enable in [0, 1]:
            self.rx_framing_idle_gaps(lc_enable, nbytes=16)

    def test_rx_framing_idle_gaps_32_bytes(self):
        # 32 bytes per cycle (2 Data Blocks: RX framing at the Link Layer frequency).
        for lc_enable in [0, 1]:
            self.rx_framing_idle_gaps(lc_enable, nbytes=32)

    def rx_framing_idle_gaps(self, lc_enable, nbytes=8):
        rng   = random.Random(3)
        units = random_units(rng, 40)
        stream_bytes = []
        expected     = []
        for u in units:
            stream_bytes += [GEN2_IDLE]*rng.choice([0, 0, 1, 2, 3, 5, 7, 8, 13, 16, 17])
            stream_bytes += gen2_unit_bytes(u)
            # Expected words: unit words with the bytes after the DPP end zeroed (DPHs: 20 bytes).
            expected += u
        stream_bytes += [GEN2_IDLE]*64
        stream_bytes += [GEN2_IDLE]*((-len(stream_bytes)) % nbytes)
        words8 = [int.from_bytes(bytes(stream_bytes[i:i+nbytes]), "little")
            for i in range(0, len(stream_bytes), nbytes)]
        dut = Gen2RXFraming(nbytes)
        out = []
        lcs = []
        def generator(dut):
            for w in words8:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(w)
                yield
                while not (yield dut.sink.ready):
                    yield
                if rng.random() < 0.3:
                    yield dut.sink.valid.eq(0)
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(32):
                yield
        @passive
        def checker(dut):
            yield dut.source.ready.eq(1)
            yield dut.source_lc.ready.eq(1)
            yield dut.lc_enable.eq(lc_enable)
            while True:
                if (yield dut.source.valid):
                    data, ctrl, n = (yield dut.source.data), (yield dut.source.ctrl), (yield dut.source.words)
                    for w in range(n):
                        out.append(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf))
                if (yield dut.source_lc.valid):
                    lcs.append((yield dut.source_lc.data))
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual((out, lcs), split_units(units, lc_enable))

# WordsNarrower ------------------------------------------------------------------------------------

class TestWordsNarrower(unittest.TestCase):
    def narrower_test(self, n_in, n_out, words):
        rng = random.Random(n_in*10 + n_out)
        entries = [[(rng.getrandbits(32), rng.getrandbits(4)) for _ in range(rng.randint(1, n_in))]
            for _ in range(64)]
        dut = WordsNarrower(n_in, n_out, words=words)
        out = []
        def generator(dut):
            for entry in entries:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(sum(d << (32*j) for j, (d, c) in enumerate(entry)))
                yield dut.sink.ctrl.eq(sum(c << (4*j) for j, (d, c) in enumerate(entry)))
                yield dut.sink.words.eq(len(entry))
                yield
                while not (yield dut.sink.ready):
                    yield
            yield dut.sink.valid.eq(0)
            for _ in range(8):
                yield
        @passive
        def checker(dut):
            while True:
                yield dut.source.ready.eq(rng.random() < 0.7)
                yield
                if (yield dut.source.valid) and (yield dut.source.ready):
                    n = (yield dut.source.words) if words else 1
                    self.assertTrue(1 <= n <= n_out)
                    data, ctrl = (yield dut.source.data), (yield dut.source.ctrl)
                    out.extend(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf) for w in range(n))
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(out, [w for e in entries for w in e])

    def test_narrower(self):
        self.narrower_test(4, 2, words=True)
        self.narrower_test(4, 1, words=False)
        self.narrower_test(2, 1, words=False)

# Generated Verilog --------------------------------------------------------------------------------

class TestGen2FramingVerilog(unittest.TestCase):
    @unittest.skipIf(__import__("shutil").which("yosys") is None, "yosys not found")
    def test_byte_offset_verilog(self):
        # Byte shifts by a variable amount in the generated Verilog (a shift amount is self-determined
        # in Verilog: 8*n was truncated, not reproduced by the Migen simulator), with yosys.
        import os, re, subprocess, tempfile
        from migen.fhdl import verilog
        from usb3_pipe.framing import byte_offset
        w = Signal(128)
        n = Signal(3)
        o = Signal(8)
        m = Module()
        m.comb += o.eq((w >> byte_offset(n))[0:8])
        with tempfile.TemporaryDirectory() as d:
            f = os.path.join(d, "shift.v")
            verilog.convert(m, ios={w, n, o}, name="shift").write(f)
            script = f"read_verilog {f}; prep -top shift; " + "; ".join(
                f"eval -set {w.backtrace[-1][0]} 128'h0f0e0d0c0b0a09080706050403020100 -set {n.backtrace[-1][0]} {v} "
                f"-show {o.backtrace[-1][0]}" for v in range(8))
            log = subprocess.run(["yosys", "-p", script], capture_output=True, text=True, check=True).stdout
        self.assertEqual([int(x, 2) for x in re.findall(r"Eval result: .* = 8'([01]+)", log)], list(range(8)))

