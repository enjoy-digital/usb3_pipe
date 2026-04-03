#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM, TS1
from usb3_pipe.core import USB3PIPE


class DummySerDes(Module):
    def __init__(self):
        self.sink = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.rx_align = Signal()
        self.rx_polarity = Signal()
        self.rx_idle = Signal()

        self.tx_idle = Signal()
        self.tx_pattern = Signal(20)


class TestUSB3PIPE(unittest.TestCase):
    def test_tx_waits_for_tx_ready(self):
        serdes = DummySerDes()
        dut = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6), with_endianness_swap=False)

        def generator(dut):
            yield serdes.sink.ready.eq(1)
            yield dut.sink.valid.eq(1)
            yield dut.sink.data.eq(0)
            yield dut.sink.ctrl.eq(0)

            for _ in range(4):
                self.assertEqual((yield dut.sink.ready), 0)
                self.assertEqual((yield serdes.sink.valid), 0)
                yield

            yield dut.tx_ready.eq(1)
            yield
            self.assertEqual((yield dut.sink.ready), 1)
            self.assertEqual((yield serdes.sink.valid), 1)
            self.assertEqual((yield serdes.sink.ctrl), 0)

        run_simulation(dut, generator(dut))

    def test_rx_waits_for_rx_ready(self):
        serdes = DummySerDes()
        dut = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6), with_endianness_swap=False)

        def generator(dut):
            yield dut.source.ready.eq(1)
            yield serdes.source.valid.eq(1)
            yield serdes.source.ctrl.eq(0b1111)
            yield serdes.source.data.eq(COM.value*0x01010101)

            for _ in range(4):
                self.assertEqual((yield dut.source.valid), 0)
                yield

            yield dut.rx_ready.eq(1)
            yield
            self.assertEqual((yield dut.source.valid), 1)
            self.assertEqual((yield dut.source.ctrl), 0b1111)
            self.assertEqual((yield dut.source.data), COM.value*0x01010101)

        run_simulation(dut, generator(dut))

    def test_short_rx_idle_glitch_does_not_report_lfps_polling(self):
        serdes = DummySerDes()
        dut = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6), with_endianness_swap=False)

        def generator(dut):
            for _ in range(32):
                yield serdes.rx_idle.eq(1)
                yield

            yield serdes.rx_idle.eq(0)
            yield

            for _ in range(128):
                yield serdes.rx_idle.eq(1)
                self.assertEqual((yield dut.lfps_rx_polling), 0)
                yield

        run_simulation(dut, generator(dut))

    def test_malformed_ts1_is_not_reported(self):
        serdes = DummySerDes()
        dut = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6), with_endianness_swap=False)
        ts1_length = len(TS1.to_bytes())//4
        ts1_words  = [int.from_bytes(TS1.to_bytes()[4*i:4*(i+1)], "little") for i in range(ts1_length)]

        def generator(dut):
            yield dut.ts_rx_enable.eq(1)
            yield serdes.source.valid.eq(1)
            for _ in range(8):
                for j, word in enumerate(ts1_words):
                    yield serdes.source.ctrl.eq(0b1111 if j == 0 else 0b0000)
                    yield serdes.source.data.eq(word ^ (0x1 if j == 2 else 0x0))
                    yield
            yield serdes.source.valid.eq(0)
            for _ in range(32):
                self.assertEqual((yield dut.ts_rx_ts1), 0)
                self.assertEqual((yield dut.ts_rx_ts1_inv), 0)
                self.assertEqual((yield dut.ts_rx_ts2), 0)
                yield

        run_simulation(dut, generator(dut))

    def test_serdes_control_passthrough(self):
        serdes = DummySerDes()
        dut = USB3PIPE(serdes=serdes, sys_clk_freq=int(125e6), with_endianness_swap=False)

        def generator(dut):
            yield dut.serdes_rx_align.eq(1)
            yield dut.serdes_rx_polarity.eq(1)
            yield
            self.assertEqual((yield serdes.rx_align), 1)
            self.assertEqual((yield serdes.rx_polarity), 1)

        run_simulation(dut, generator(dut))
