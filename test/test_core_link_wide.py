#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Link Layer on PHY words pairs (64-bit, USB3PHY link_words=2) vs words (32-bit): Data
Packet Payloads received/transmitted 8 bytes per cycle, same Link Layer words on the PHY."""

import random
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common     import *
from usb3_core.host_model import *
from usb3_core.link       import USB3LinkLayer

LENGTHS = [0, 1, 3, 4, 5, 7, 8, 9, 12, 13, 64, 1023, 1024]

# DUT ----------------------------------------------------------------------------------------------

class LinkDUT(LiteXModule):
    def __init__(self, link_words):
        self.phy = phy = PHYStandIn()
        phy.gen2_active = Signal(reset=1)
        if link_words == 2:
            phy.sink   = stream.Endpoint(link_pairs_layout())
            phy.source = stream.Endpoint(link_pairs_layout())
        self.link = USB3LinkLayer(phy, sys_clk_freq=125e6, ssp_tt={2: TT_BULK})

def advertisement():
    words = link_command(LGOOD(15))
    for x in range(4):
        words += link_command(LCRD(x)) + link_command(LCRD2(x))
    return words

def start(phy):
    yield phy.polling_idle.eq(1)
    yield
    yield phy.polling_idle.eq(0)
    yield phy.u0.eq(1)
    yield phy.sink.ready.eq(1)

def send_words(phy, words, link_words, rng):
    """PHY source: words, or words pairs (1 or 2 words per entry)."""
    i = 0
    while i < len(words):
        n = 2 if (link_words == 2 and (i + 1) < len(words) and rng.random() < 0.8) else 1
        yield phy.source.valid.eq(1)
        yield phy.source.data.eq(sum(d << (32*j) for j, (d, c) in enumerate(words[i:i+n])))
        yield phy.source.ctrl.eq(sum(c << (4*j) for j, (d, c) in enumerate(words[i:i+n])))
        if link_words == 2:
            yield phy.source.words.eq(n)
        yield
        while not (yield phy.source.ready):
            yield
        i += n
    yield phy.source.valid.eq(0)

# Tests --------------------------------------------------------------------------------------------

class TestCoreLinkWide(unittest.TestCase):
    def rx_test(self, link_words):
        # Host DPHs/DPPs (Bulk OUT, EP2) -> rx_dpp payloads, CRC-32 checked.
        rng      = random.Random(link_words)
        dut      = LinkDUT(link_words)
        link     = dut.link
        phy      = dut.phy
        payloads = [[rng.randrange(256) for _ in range(n)] for n in LENGTHS]
        received = []
        status   = []
        partial  = []
        words    = advertisement()
        for seq, payload in enumerate(payloads):
            words += dph(seq % 16, len(payload), ept=2, tt=TT_BULK)
            words += data_packet_payload(payload)

        def host():
            yield from start(phy)
            yield from send_words(phy, words, link_words, rng)
            for _ in range(512):
                yield

        @passive
        def monitor():
            yield link.rx_hp.ready.eq(1)
            current = []
            while True:
                if (yield link.rx_dpp.valid):
                    data   = (yield link.rx_dpp.data)
                    nbytes = (yield link.rx_dpp.bytes)
                    current += list(data.to_bytes(link_words*4, "little"))[:nbytes]
                    # Full words except the last one (endpoints memories written per word).
                    if not (yield link.rx_dpp.last):
                        partial.append(nbytes != 4*link_words)
                if (yield link.rx_dpp_done):
                    received.append(current)
                    status.append((yield link.rx_dpp_ok))
                    current = []
                yield

        run_simulation(dut, [host(), monitor()])
        self.assertEqual(received, payloads)
        self.assertEqual(status, [1]*len(payloads))
        self.assertFalse(any(partial))

    def tx_test(self, link_words):
        # DPHs + DPPs (tx_hp/tx_dpp, 64-bit DPP words with link_words=2) -> PHY words: DPPs decoded,
        # CRC-32 checked.
        rng      = random.Random(10 + link_words)
        dut      = LinkDUT(link_words)
        link     = dut.link
        phy      = dut.phy
        dw       = 32*link_words
        payloads = [[rng.randrange(256) for _ in range(n)] for n in LENGTHS]
        sent     = []

        def host():
            yield from start(phy)
            yield from send_words(phy, advertisement(), link_words, rng)
            # LGOOD_n/LCRD2_x for the DPHs as they are sent (credits).
            n = 0
            while n < len(payloads):
                # DPHs sent: SHP framing words (PHY words order).
                dphs = [w for w in sent if w == (0xfbfbfbf7, 0b1111)]
                if len(dphs) > n:
                    yield from send_words(phy, link_command(LGOOD(n % 16)) +
                        link_command(LCRD2(n % 4)), link_words, rng)
                    n += 1
                yield
            for _ in range(2048):
                yield

        def hps():
            for _ in range(64):
                yield
            for seq, payload in enumerate(payloads):
                yield link.tx_hp.valid.eq(1)
                yield link.tx_hp.dw0.eq(TYPE_DP)
                yield link.tx_hp.dw1.eq(dph_dw1(seq=seq, ept=1, direction=1, length=len(payload)))
                yield link.tx_hp.dpp.eq(1)
                yield
                while not (yield link.tx_hp.ready):
                    yield
                yield link.tx_hp.valid.eq(0)
                yield

        @passive
        def dpps():
            # DPP words in the DPHs order, random gaps before each DPP (prefetched by the endpoints).
            for payload in payloads:
                words = [payload[i:i+dw//8] for i in range(0, len(payload), dw//8)] or [[]]
                for i, w in enumerate(words):
                    yield link.tx_dpp.valid.eq(1)
                    yield link.tx_dpp.data.eq(int.from_bytes(bytes(w + [0]*(dw//8 - len(w))), "little"))
                    yield link.tx_dpp.bytes.eq(len(w))
                    yield link.tx_dpp.first.eq(i == 0)
                    yield link.tx_dpp.last.eq(i == len(words) - 1)
                    yield
                    while not (yield link.tx_dpp.ready):
                        yield
                yield link.tx_dpp.valid.eq(0)
                yield

        @passive
        def monitor():
            while True:
                if (yield phy.sink.valid) & (yield phy.sink.ready):
                    data = (yield phy.sink.data)
                    ctrl = (yield phy.sink.ctrl)
                    n    = (yield phy.sink.words) if link_words == 2 else 1
                    for j in range(n):
                        sent.append(((data >> (32*j)) & 0xffffffff, (ctrl >> (4*j)) & 0xf))
                yield

        run_simulation(dut, [host(), hps(), dpps(), monitor()])
        dpps_rx = [p for p in decode_packets(sent) if p[0] == "DPP"]
        self.assertEqual([list(p[1]) for p in dpps_rx], [p for p in payloads])
        self.assertEqual([p[2] for p in dpps_rx], [True]*len(payloads))

    def test_rx_throttle(self):
        # PHY rx_throttle: LGOOD_n still sent for the received DPHs, their LCRD_x held while throttled
        # and sent once released (the host stops on its credits: RX flow control).
        rng  = random.Random(0)
        dut  = LinkDUT(link_words=2)
        dut.phy.rx_throttle = rx_throttle = Signal()
        dut.link = USB3LinkLayer(dut.phy, sys_clk_freq=125e6, ssp_tt={2: TT_BULK})
        link = dut.link
        phy  = dut.phy
        sent   = []
        counts = {}
        words  = advertisement()
        for seq in range(2):
            words += dph(seq, 8, ept=2, tt=TT_BULK) + data_packet_payload(list(range(8)))

        def host():
            yield rx_throttle.eq(1)
            yield from start(phy)
            yield from send_words(phy, words, 2, rng)
            for _ in range(512):
                yield
            counts["throttled"] = list(decode_packets(sent))
            yield rx_throttle.eq(0)
            for _ in range(512):
                yield
            counts["released"] = list(decode_packets(sent))

        @passive
        def monitor():
            yield link.rx_hp.ready.eq(1)
            while True:
                if (yield phy.sink.valid) & (yield phy.sink.ready):
                    data = (yield phy.sink.data)
                    ctrl = (yield phy.sink.ctrl)
                    for j in range((yield phy.sink.words)):
                        sent.append(((data >> (32*j)) & 0xffffffff, (ctrl >> (4*j)) & 0xf))
                yield

        run_simulation(dut, [host(), monitor()])
        def lcs(packets, cmd):
            return [p for p in packets if p[0] == "LC" and (p[1] & 0x780) == cmd]
        throttled, released = counts["throttled"], counts["released"]
        self.assertEqual(len(lcs(throttled, 0x000)), 1 + 2) # LGOOD_n: advertisement + 2 DPHs.
        self.assertEqual(len(lcs(throttled, 0x080)), 0)     # No LCRD_x while throttled.
        self.assertGreaterEqual(len(lcs(released, 0x080)), 2)

    def test_rx_words(self):
        self.rx_test(link_words=1)

    def test_rx_pairs(self):
        self.rx_test(link_words=2)

    def test_tx_words(self):
        self.tx_test(link_words=1)

    def test_tx_pairs(self):
        self.tx_test(link_words=2)

if __name__ == "__main__":
    unittest.main()
