#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3PHY with pipe_cd (PIPE at twice the LTSSM/Link clock, Link words pairs): PHY reset reaching
the pipe_cd side (PIPE, Link words CDC FIFOs, RX throttle counter), two clock domains simulation."""

import unittest

from migen import *

from litex.gen import *

from usb3_pipe.common import SHP, EPF
from usb3_pipe.phy    import USB3PHY

from test.common   import ScramblerModel, words_to_symbols, symbols_to_words
from test.model.gt import GTModel

# Helpers ------------------------------------------------------------------------------------------

HP_WORDS = 5 # Header Packet: framing word + 4 words.

def header_packets(n, first=0):
    """n Header Packets (SHP framing word + 4 data words, PIPE words order), payload words numbered
    from first."""
    words = []
    for i in range(n):
        words += [symbols_to_words([(SHP.value, 1)]*3 + [(EPF.value, 1)])[0]]
        words += [(0x01000000*(i + 1) + 0x100*(first + i) + k, 0) for k in range(4)]
    return words

def scrambled(words):
    """Words scrambled from the LFSR seed (as the device descrambler after its reset)."""
    return symbols_to_words(ScramblerModel().process(words_to_symbols(words)))

class DUT(LiteXModule):
    """USB3PHY (device) with the PIPE in the pipe clock domain at twice the sys frequency, Link words
    pairs, a GT model in pipe. Real timers: the GT model RX (no idle, errors) would otherwise be
    taken for a Warm Reset LFPS within the test."""
    def __init__(self):
        self.gt  = gt  = ClockDomainsRenamer("pipe")(GTModel())
        self.phy = phy = USB3PHY(serdes=gt, sys_clk_freq=int(62.5e6), with_endianness_swap=False,
            link_words=2, pipe_cd="pipe", pipe_clk_freq=int(125e6))
        self.phase = 0 # Test phase (sys generator -> pipe generators).

# Test ---------------------------------------------------------------------------------------------

class TestPHYPipeCD(unittest.TestCase):
    def run_phy(self, sys_generator, pipe_generators):
        dut = DUT()
        run_simulation(dut, {"sys": [sys_generator(dut)], "pipe": pipe_generators(dut)},
            clocks={"sys": 16, "pipe": 8})
        return dut

    def force_u0(self, dut):
        """LTSSM forced to U0 (rx_ready/tx_ready to the PIPE through the resynchronization)."""
        yield dut.phy.ltssm.fsm.state.eq(dut.phy.ltssm_states["U0"])
        for _ in range(8):
            yield
        self.assertEqual((yield dut.phy.u0), 1)

    def test_reset_clears_pipe_side(self):
        # RX words received in U0 and not read by the Link Layer fill the RX CDC FIFO (throttle
        # asserted), TX words pass the TX CDC FIFO; after a PHY reset nothing stale remains: no RX
        # word, throttle released, TX FIFO empty, and the words received after a new U0 entry are
        # the new ones.
        n_hp  = 64 # 320 words: 160 pairs, more than half of the 256 entries RX FIFO.
        rx    = header_packets(n_hp)
        rx2   = header_packets(4, first=100)
        # Logical Idle after the packets (flushes the RX framing aligner, dropped by the Logical
        # Idle filter); device descrambler reset by the PHY reset: the second burst scrambled from
        # the seed.
        idle  = [(0, 0)]*16
        wire1 = scrambled(rx + idle)
        wire2 = scrambled(rx2 + idle)

        def sys_generator(dut):
            dut.phase = 0
            yield dut.phy.source.ready.eq(0)
            yield from self.force_u0(dut)
            # TX words through the TX CDC (drained by the PIPE in U0).
            for i in range(16):
                yield dut.phy.sink.valid.eq(1)
                yield dut.phy.sink.data.eq(0x11110000 + i)
                yield dut.phy.sink.words.eq(2)
                yield dut.phy.sink.first.eq(i == 0)
                yield dut.phy.sink.last.eq(i == 15)
                yield
                while not (yield dut.phy.sink.ready):
                    yield
            yield dut.phy.sink.valid.eq(0)
            # RX words received (pipe generator), not read: throttle asserted.
            dut.phase = 1
            for _ in range(2000):
                if (yield dut.phy.rx_throttle):
                    break
                yield
            self.assertEqual((yield dut.phy.rx_throttle), 1)
            self.assertEqual((yield dut.phy.source.valid), 1)
            for _ in range(32):
                yield
            # PHY reset.
            yield dut.phy.reset.eq(1)
            for _ in range(8):
                yield
            yield dut.phy.reset.eq(0)
            for _ in range(32):
                yield
            self.assertEqual((yield dut.phy.u0), 0)
            self.assertEqual((yield dut.phy.rx_throttle), 0,      "RX throttle level not reset")
            self.assertEqual((yield dut.phy.source.valid), 0,     "stale RX words after the reset")
            self.assertEqual((yield dut.phy.link_cdc.tx_cdc.source.valid), 0,
                "stale TX words after the reset")
            # New U0 entry, new RX words: the first received words are the new ones.
            yield from self.force_u0(dut)
            dut.phase = 2
            received = []
            yield dut.phy.source.ready.eq(1)
            for _ in range(2000):
                if (yield dut.phy.source.valid):
                    n = (yield dut.phy.source.words)
                    d = (yield dut.phy.source.data)
                    c = (yield dut.phy.source.ctrl)
                    for w in range(n):
                        received.append(((d >> (32*w)) & 0xffffffff, (c >> (4*w)) & 0xf))
                if len(received) >= len(rx2):
                    break
                yield
            self.assertEqual(received[:len(rx2)], rx2)
            dut.phase = 3

        def pipe_generators(dut):
            def rx_driver():
                # Burst 1 in phase 1, burst 2 in phase 2 (after the PHY reset).
                for phase, burst in [(1, wire1), (2, wire2)]:
                    while dut.phase != phase:
                        yield
                    for _ in range(8):
                        yield
                    for data, ctrl in burst:
                        yield dut.gt.source.valid.eq(1)
                        yield dut.gt.source.data.eq(data)
                        yield dut.gt.source.ctrl.eq(ctrl)
                        yield
                    yield dut.gt.source.valid.eq(0)
                while dut.phase != 3:
                    yield
            return [rx_driver()]

        self.run_phy(sys_generator, pipe_generators)

if __name__ == "__main__":
    unittest.main()
