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
    return [k_word(SLC), (lcw | (lcw << 16), 0)]

def dpp(rng, length, nullified=False):
    if nullified:
        return [k_word(EDB)]
    data = [(rng.getrandbits(8), 0) for _ in range(length + 4)] # Data + CRC-32.
    end  = [(END, 1)]*3 + [(EPF, 1)]
    return [k_word(SDP)] + words_from_bytes(data + end)

def random_units(rng, n):
    units = []
    for _ in range(n):
        kind = rng.choice(["hp", "lc", "dp", "dp", "nullified"])
        if kind == "hp":
            units.append(header_packet(rng))
        elif kind == "lc":
            units.append(link_command(rng))
        else:
            length = rng.choice([0, 1, 2, 3, 4, 5, 13, 64, 255, 1024])
            units.append(header_packet(rng, dw0_type=0x08, length=length) +
                dpp(rng, length, nullified=(kind == "nullified")))
    return units

# Loopback -----------------------------------------------------------------------------------------

class Gen2FramingLoopback(Module):
    """Gen2TXFramer -> units FIFO (store and forward) -> Gen2TXPacker -> Data Blocks (Gen2TX model:
    a block every 3 cycles at most, Idle Data Blocks when no block) -> Gen2RXFraming."""
    def __init__(self):
        self.framer  = framer  = Gen2TXFramer()
        self.fifo    = fifo    = stream.SyncFIFO(gen2_tx_unit_layout(), 512, buffered=True)
        self.packer  = packer  = Gen2TXPacker()
        self.rx      = rx      = Gen2RXFraming()
        self.submodules += framer, fifo, packer, rx
        self.comb += [
            framer.source.connect(fifo.sink),
            fifo.source.connect(packer.sink),
        ]
        units_written = Signal(16)
        units_started = Signal(16)
        self.sync += [
            If(fifo.sink.valid & fifo.sink.ready & fifo.sink.last, units_written.eq(units_written + 1)),
            If(packer.unit_end, units_started.eq(units_started + 1)),
        ]
        self.comb += packer.units_available.eq(units_written != units_started)

        # Gen2TX model: block every 3 cycles, idle block when none.
        self.block       = block = Signal(128)
        self.block_valid = block_valid = Signal()
        timer = Signal(2)
        self.sync += If(timer == 2, timer.eq(0)).Else(timer.eq(timer + 1))
        idle = int.from_bytes(bytes([GEN2_IDLE])*16, "little")
        self.comb += [
            packer.source.ready.eq(timer == 2),
        ]
        self.sync += [
            block_valid.eq(timer == 2),
            If(timer == 2,
                block.eq(Mux(packer.source.valid, packer.source.data, idle))
            )
        ]
        # Blocks -> 8 bytes per cycle (2 cycles per block, then 1 idle cycle).
        half = Signal()
        hold = Signal(128)
        self.sync += [
            If(block_valid, hold.eq(block), half.eq(1)).Elif(half, half.eq(0)),
        ]
        # RX FIFO (the RX framing stalls 1 cycle on each packet start).
        rx_fifo = stream.SyncFIFO([("data", 64)], 8)
        self.submodules += rx_fifo
        self.comb += [
            rx_fifo.sink.valid.eq(block_valid | half),
            rx_fifo.sink.data.eq(Mux(block_valid, block[0:64], hold[64:128])),
            rx_fifo.source.connect(rx.sink),
            rx.source.ready.eq(1),
        ]
        self.rx_overflow = Signal()
        self.sync += If(rx_fifo.sink.valid & ~rx_fifo.sink.ready, self.rx_overflow.eq(1))

class TestGen2Framing(unittest.TestCase):
    def loopback_test(self, seed, n_units=24):
        rng   = random.Random(seed)
        units = random_units(rng, n_units)
        words = [w for u in units for w in u]
        out   = []
        dut   = Gen2FramingLoopback()
        def generator(dut):
            for data, ctrl in words:
                yield dut.framer.sink.valid.eq(1)
                yield dut.framer.sink.data.eq(data)
                yield dut.framer.sink.ctrl.eq(ctrl)
                yield
                while not (yield dut.framer.sink.ready):
                    yield
                # Random gaps between words (core TX).
                if rng.random() < 0.2:
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
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(out, words)


    def test_loopback(self):
        for seed in range(3):
            self.loopback_test(seed)

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
        ]

class TestGen2LinkTX(unittest.TestCase):
    def test_link_tx_cdc(self):
        rng   = random.Random(7)
        units = random_units(rng, 24)
        words = [w for u in units for w in u]
        out   = []
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
                yield
        def timeout(dut):
            for _ in range(len(words)*4 + 2000):
                yield
        run_simulation(dut, {"sys": [generator(dut), timeout(dut)], "tx": [checker(dut)]},
            clocks={"sys": 8, "tx": 4})
        self.assertEqual(out, words)

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
        stream_bytes += [GEN2_IDLE]*((-len(stream_bytes)) % 8)
        words8 = [int.from_bytes(bytes(stream_bytes[i:i+8]), "little") for i in range(0, len(stream_bytes), 8)]
        dut = Gen2RXFraming()
        out = []
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
            while True:
                if (yield dut.source.valid):
                    data, ctrl, n = (yield dut.source.data), (yield dut.source.ctrl), (yield dut.source.words)
                    for w in range(n):
                        out.append(((data >> (32*w)) & 0xffffffff, (ctrl >> (4*w)) & 0xf))
                yield
        run_simulation(dut, [generator(dut), checker(dut)])
        self.assertEqual(out, expected)

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

