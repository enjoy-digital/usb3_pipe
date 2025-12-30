#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
import random

from migen import *

from usb3_pipe.serdes import RXWordAligner
from usb3_pipe.serdes import RXSKPRemover, TXSKPInserter
from usb3_pipe.serdes import RXDatapath, TXDatapath


class TestSerDes(unittest.TestCase):
    def test_rx_word_aligner(self):
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
            0x030201bc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0x0b0201bc, 0x0f060504, 0x130a0908, 0x010e0d0c,
            0x202f9cbc, 0xa5000300, 0x53528d03, 0x86850000,
            0x8b8a89bc, 0x8f8e8d84, 0x92919088, 0xbcbcbc8c,
            0xbcbcbcbc, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
            0xbcbcbcbc, 0x08070605, 0x0c0b0a09, 0x000f0e0d,
            0xbcbcbcbc, 0x09080706, 0x0d0c0b0a, 0x00000f0e,
            0xbcbcbcbc, 0x0a090807, 0x0e0d0c0b, 0x0e0d0c0f,
        ]
        ctrls_reference = [
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0x0,
            0x1, 0x0, 0x0, 0xe,
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
