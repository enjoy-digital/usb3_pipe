#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.sim import passive

from usb3_pipe.common import SKP
from usb3_pipe.core import USB3PIPE
from usb3_pipe.datapath import TXSKPInserter

from test.model.gt import GTModel

SKP_WORD = (int.from_bytes(bytes([SKP.value]*4), "little"), 0b1111)

# Helpers ------------------------------------------------------------------------------------------

def run_inserter(packets, enable=None):
    """Send packets (word counts, first/last delimited) through the TXSKPInserter, return the output
    words as a string: "D" for data, "S" for a SKP word (2 SKP Ordered Sets)."""
    dut    = TXSKPInserter()
    enable = enable or [1]*len(packets)

    def generator(dut):
        for n, en in zip(packets, enable):
            if hasattr(dut, "enable"):
                yield dut.enable.eq(en)
            for i in range(n):
                yield dut.sink.valid.eq(1)
                yield dut.sink.first.eq(i == 0)
                yield dut.sink.last.eq(i == n - 1)
                yield dut.sink.data.eq(0x11223344)
                yield
                while not (yield dut.sink.ready):
                    yield
        yield dut.sink.valid.eq(0)
        for _ in range(16):
            yield

    @passive
    def monitor(dut):
        dut.out = []
        yield dut.source.ready.eq(1)
        while True:
            yield
            if (yield dut.source.valid):
                word = ((yield dut.source.data), (yield dut.source.ctrl))
                dut.out.append("S" if word == SKP_WORD else "D")

    run_simulation(dut, [generator(dut), monitor(dut)])
    return "".join(dut.out)

# TX SKP (USB 3.2 6.4.3, 6.4.3.1) ------------------------------------------------------------------

class TestTXSKP(unittest.TestCase):
    def test_skp_average_rate(self):
        # SKP Ordered Sets at an average of every 354 symbols (6.4.3), between packets.
        out = run_inserter([16]*400)
        data, skp = out.count("D"), 2*out.count("S")
        self.assertEqual(data, 16*400)
        self.assertAlmostEqual(4*data/skp, 354, delta=354*0.02)

    def test_skp_max_four_buffered(self):
        # The transmitter is allowed to buffer up to a maximum of four SKP Ordered Sets (6.4.3):
        # after a long transmission without SKP opportunity, at most 4 SKP Ordered Sets.
        out   = run_inserter([20000, 16])
        burst = len(out[20000:]) - len(out[20000:].lstrip("S"))
        self.assertEqual(out[:20000], "D"*20000)
        self.assertLessEqual(2*burst, 4)

    def test_skp_disabled(self):
        # No SKP scheduled while disabled (TSEQ, 6.4.3.1: SKP only after TS1/TS2, packets and
        # Logical Idle) and Y reset when enabled (on Polling.Active entry, 6.4.3.1).
        out = run_inserter([20000, 16, 16], enable=[0, 1, 1])
        self.assertEqual(out[:20000 + 16 + 16], "D"*(20000 + 16 + 16))

    def test_pipe_disables_skp_during_tseq(self):
        # The PIPE disables SKP scheduling while transmitting TSEQ.
        gt = GTModel()
        gt.tx_skp_enable = Signal(reset=1)
        dut = USB3PIPE(serdes=gt, sys_clk_freq=int(125e6), with_endianness_swap=False)
        dut.submodules += gt

        def generator(dut):
            yield dut.ts_tx_enable.eq(1)
            for tseq, expected in [(1, 0), (0, 1)]:
                yield dut.ts_tx_tseq.eq(tseq)
                yield dut.ts_tx_ts1.eq(not tseq)
                yield
                yield
                self.assertEqual((yield gt.tx_skp_enable), expected)

        run_simulation(dut, generator(dut))

if __name__ == "__main__":
    unittest.main()
