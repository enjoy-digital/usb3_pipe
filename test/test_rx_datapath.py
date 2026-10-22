#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from usb3_pipe.common import SKP
from usb3_pipe.serdes import RXDatapath

from test.common import strip_skp

# Partner TX stream model --------------------------------------------------------------------------

def partner_symbols(n, rng):
    """Partner TX symbols: random data with a SKP Ordered Set (2 x SKP) every 354 symbols on average
    (USB 3.2 6.4.3), inserted on symbol-pair boundaries."""
    symbols = []
    y = 0
    while len(symbols) < n:
        symbols += [(rng.randrange(256), 0), (rng.randrange(256), 0)]
        y += 2
        if y >= 354:
            symbols += [(SKP.value, 1), (SKP.value, 1)]
            y -= 354
    return symbols

# RX Datapath --------------------------------------------------------------------------------------

class TestRXDatapath(unittest.TestCase):
    def run_rx_datapath(self, ppm, n_symbols=8000, seed=0):
        """Partner clock offset by `ppm` vs local sys clock; the transceiver ignores ready (always
        valid at the line rate), as the GT does."""
        rng     = random.Random(seed)
        symbols = partner_symbols(n_symbols, rng)
        words16 = [(symbols[i][0] | (symbols[i+1][0] << 8), symbols[i][1] | (symbols[i+1][1] << 1))
            for i in range(0, len(symbols) - 1, 2)]
        expected = strip_skp(symbols)

        def rx_generator(dut):
            for data, ctrl in words16:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
            yield dut.sink.valid.eq(0)

        def sys_checker(dut):
            dut.received = []
            yield dut.source.ready.eq(1)
            for _ in range(len(words16)//2 + 64):
                yield
                if (yield dut.source.valid):
                    data = (yield dut.source.data)
                    ctrl = (yield dut.source.ctrl)
                    dut.received += [((data >> 8*i) & 0xff, (ctrl >> i) & 0x1) for i in range(4)]

        sys_period = 20000
        rx_period  = round(sys_period/2/(1 + ppm*1e-6)) # 16-bit words at 2x the 32-bit word rate.
        dut = RXDatapath(clock_domain="rx", phy_dw=16)
        run_simulation(dut, {"rx": rx_generator(dut), "sys": sys_checker(dut)},
            clocks={"sys": sys_period, "rx": rx_period})
        return dut.received, expected

    def test_rx_datapath_partner_slower(self):
        received, expected = self.run_rx_datapath(ppm=-3000)
        self.assertEqual(received, expected[:len(received)])
        self.assertGreater(len(received), 0.99*len(expected))

    def test_rx_datapath_partner_faster(self):
        # Partner faster (within the SKP compensation capability): the receiver must compensate
        # with SKP removal (elasticity buffer, USB 3.2 6.4.3) without losing data symbols.
        received, expected = self.run_rx_datapath(ppm=+3000)
        self.assertEqual(received, expected[:len(received)])
        self.assertGreater(len(received), 0.99*len(expected))

if __name__ == "__main__":
    unittest.main()
