# This file is part of USB3-PIPE.
# SPDX-License-Identifier: BSD-2-Clause

"""Dedicated Link Commands path (PHY sink_lc): LGOOD_n/LCRD_x of the Link Layer are not behind the
DPH + DPP being transmitted.

A host Header Packet (ITP, acknowledgement) received while a DPP is being sent must be acknowledged
within PENDING_HP_TIMER (3us): with the Link Commands on the same stream as the DPP they are written
after it (and leave after it), a DPP time (~2.1us at 5Gbps) later."""

import random
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common     import *
from usb3_core.host_model import *
from usb3_core.link       import USB3LinkLayer

PAYLOAD = 1024

# DUT ----------------------------------------------------------------------------------------------

class LinkDUT(LiteXModule):
    def __init__(self, with_lc_path):
        self.phy = phy = PHYStandIn()
        phy.ssp_capable = True
        phy.gen2_active = Signal(reset=1)
        phy.sink   = stream.Endpoint(link_pairs_layout(2))
        phy.source = stream.Endpoint(link_pairs_layout(2))
        if with_lc_path:
            phy.sink_lc = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.link = USB3LinkLayer(phy, sys_clk_freq=125e6, ssp_tt={2: TT_BULK})

def advertisement():
    words = link_command(LGOOD(15))
    for x in range(4):
        words += link_command(LCRD(x)) + link_command(LCRD2(x))
    return words

def send_words(phy, words):
    """PHY source: one word per entry."""
    for data, ctrl in words:
        yield phy.source.valid.eq(1)
        yield phy.source.data.eq(data)
        yield phy.source.ctrl.eq(ctrl)
        yield phy.source.words.eq(1)
        yield
        while not (yield phy.source.ready):
            yield
    yield phy.source.valid.eq(0)

def run_link(with_lc_path, itps, packets=1, seed=0):
    """Transmit `packets` DPH + DPP and receive host ITPs: after `itps` (cycles after the DPH start of
    each packet, or None: random). Returns the words/cycles seen on sink and sink_lc."""
    rng   = random.Random(seed)
    dut   = LinkDUT(with_lc_path)
    link  = dut.link
    phy   = dut.phy
    cycle = [0]
    sink  = [] # (cycle, word, ctrl) on the main stream.
    lc    = [] # (cycle, word, ctrl) on sink_lc.
    payloads = [[rng.randrange(256) for _ in range(PAYLOAD)] for _ in range(packets)]
    dph_cycles = []

    def host():
        yield phy.polling_idle.eq(1)
        yield
        yield phy.polling_idle.eq(0)
        yield phy.u0.eq(1)
        yield phy.sink.ready.eq(1)
        yield link.rx_hp.ready.eq(1)
        if with_lc_path:
            yield phy.sink_lc.ready.eq(1)
        yield from send_words(phy, advertisement())
        # One ITP per packet, `itps` cycles after its DPH start (the ITPs and DPHs are numbered in
        # their own sequences: the host Header Sequence Number counts its HPs).
        for n in range(packets):
            while len(dph_cycles) <= n:
                yield
            delay = itps[n] if itps is not None else rng.randrange(0, 200)
            while cycle[0] < dph_cycles[n] + delay:
                yield
            yield from send_words(phy, itp(n, timestamp=n))
            # The DPH is acknowledged (LGOOD_n) and returns a credit, as a host does.
            yield from send_words(phy, link_command(LGOOD(n % 16)) + link_command(LCRD2(n % 4)))
        for _ in range(1024):
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
            # Next DPH once the DPP words are all taken.
            while (yield link.tx_dpp.valid):
                yield
            for _ in range(rng.randrange(0, 40)):
                yield

    @passive
    def dpps():
        dw = 64
        for payload in payloads:
            words = [payload[i:i+dw//8] for i in range(0, len(payload), dw//8)]
            for i, w in enumerate(words):
                yield link.tx_dpp.valid.eq(1)
                yield link.tx_dpp.data.eq(int.from_bytes(bytes(w), "little"))
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
                n    = (yield phy.sink.words)
                for j in range(n):
                    word = ((data >> (32*j)) & 0xffffffff, (ctrl >> (4*j)) & 0xf)
                    sink.append((cycle[0], *word))
                    if word == (0xfbfbfbf7, 0b1111) and len(dph_cycles) < packets:
                        dph_cycles.append(cycle[0])
            if with_lc_path and (yield phy.sink_lc.valid) & (yield phy.sink_lc.ready):
                lc.append((cycle[0], (yield phy.sink_lc.data), (yield phy.sink_lc.ctrl)))
            yield
            cycle[0] += 1

    run_simulation(dut, [host(), hps(), dpps(), monitor()])
    return sink, lc, payloads

def lc_infos(words):
    """Link Commands (info, cycle of the framing word) in words (cycle, data, ctrl)."""
    positions = []
    decoded  = decode_packets([(d, c) for _, d, c in words], positions)
    return [(p[1], words[pos][0]) for p, pos in zip(decoded, positions) if p[0] == "LC"]

# Tests --------------------------------------------------------------------------------------------

class TestLinkCommandsPath(unittest.TestCase):
    def test_lgood_of_an_itp_during_a_dpp_does_not_wait_for_it(self):
        sink, lc, payloads = run_link(with_lc_path=True, itps=[30])
        # DPP framing: SDP ... END on the main stream.
        end = [cycle for cycle, data, ctrl in sink if (data, ctrl) == (0xfdfdfdf7, 0b1111)]
        self.assertEqual(len(end), 1)
        lgood = [cycle for info, cycle in lc_infos(lc) if info == LGOOD(0)]
        self.assertEqual(len(lgood), 1)
        # The LGOOD_n of the ITP (received ~30 cycles after the DPH start) is sent in the next few
        # cycles on sink_lc, while the DPP (128 cycles) is still being sent on the main stream.
        self.assertLess(lgood[0], end[0] - 40)
        # Nothing but the DPH/DPP on the main stream: no LGOOD_n behind the DPP.
        self.assertEqual([info for info, _ in lc_infos(sink) if info == LGOOD(0)], [])

    def test_without_the_dedicated_path_the_lgood_waits_behind_the_dpp(self):
        # The same scenario on a PHY without sink_lc: the LGOOD_n is behind the DPP (the legacy
        # behavior, kept).
        sink, lc, payloads = run_link(with_lc_path=False, itps=[30])
        end   = [cycle for cycle, data, ctrl in sink if (data, ctrl) == (0xfdfdfdf7, 0b1111)]
        lgood = [cycle for info, cycle in lc_infos(sink) if info == LGOOD(0)]
        self.assertEqual((len(end), len(lgood)), (1, 1))
        self.assertGreater(lgood[0], end[0])

    def test_every_header_packet_is_acknowledged_in_order_and_the_dpps_are_intact(self):
        packets = 12
        for seed in range(3):
            sink, lc, payloads = run_link(with_lc_path=True, itps=None, packets=packets, seed=seed)
            # LGOOD_n: the Advertisement (LGOOD_15 on the U0 entry), then those of the ITPs in order,
            # each once.
            lgood = [info & 0xf for info, _ in lc_infos(lc) if (info & 0x7f0) == 0x000]
            self.assertEqual(lgood, [15] + list(range(packets)), f"seed {seed}")
            # DPPs: contents and CRC-32.
            dpps = [p for p in decode_packets([(d, c) for _, d, c in sink]) if p[0] == "DPP"]
            self.assertEqual([list(p[1]) for p in dpps], payloads, f"seed {seed}")
            self.assertEqual([p[2] for p in dpps], [True]*packets)
            # The dedicated path only carries LGOOD/LCRD (no LBAD/LRTY).
            infos = [info for info, _ in lc_infos(lc)]
            self.assertTrue(all((i & 0x7f0) == 0x000 or (i & 0x7f8) == 0x080 or (i & 0x7fc) == 0x084
                for i in infos), [hex(i) for i in infos])

if __name__ == "__main__":
    unittest.main()
