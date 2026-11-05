#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Link Layer in SuperSpeedPlus operation (PHY stand-in at Gen 2): Type 1/Type 2 Tx Header
Buffers (7.2.4.1: 4 unacknowledged Type 1 Header Packets and 4 unacknowledged Type 2 DPHs)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.common     import *
from usb3_core.host_model import *
from usb3_core.link       import USB3LinkLayer

# DUT ----------------------------------------------------------------------------------------------

class LinkDUT(LiteXModule):
    """link_words=2: Link Layer on PHY words pairs (64-bit words, ex: Gen 2 with pipe_cd), PHY
    stand-in words on the test side."""
    def __init__(self, link_words=1):
        from test.model.device import PairsPHY
        self.phy = phy = PHYStandIn()
        phy.gen2_active = Signal(reset=1)
        link_phy = phy if link_words == 1 else PairsPHY(phy, self)
        self.link = USB3LinkLayer(link_phy, sys_clk_freq=125e6, ssp_tt={1: TT_BULK})

# Test ---------------------------------------------------------------------------------------------

class TestCoreLinkSSP(unittest.TestCase):
    link_words = 1

    def test_tx_header_buffers_types(self):
        # The host advertises its credits but never acknowledges (no LGOOD_n): the Port Capability
        # LMP and 3 ACK TPs fill the Type 1 buffers, 4 DPHs (TT Bulk) are still accepted in the Type 2
        # buffers, the 5th Type 1/Type 2 Header Packets are not.
        dut  = LinkDUT(self.link_words)
        link = dut.link
        phy  = dut.phy
        accepted = []

        def host():
            words = link_command(LGOOD(15))
            for x in range(4):
                words += link_command(LCRD(x)) + link_command(LCRD2(x))
            yield phy.polling_idle.eq(1)
            yield
            yield phy.polling_idle.eq(0)
            yield phy.u0.eq(1)
            yield phy.sink.ready.eq(1)
            for data, ctrl in words:
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)

        def push(dw0, dw1, dpp=0, timeout=256):
            yield link.tx_hp.valid.eq(1)
            yield link.tx_hp.dw0.eq(dw0)
            yield link.tx_hp.dw1.eq(dw1)
            yield link.tx_hp.dpp.eq(dpp)
            yield
            for _ in range(timeout):
                if (yield link.tx_hp.ready):
                    yield link.tx_hp.valid.eq(0)
                    yield
                    return True
                yield
            yield link.tx_hp.valid.eq(0)
            yield
            return False

        def dpps():
            # Zero-length DPPs for the DPHs.
            yield "passive"
            yield link.tx_dpp.valid.eq(1)
            yield link.tx_dpp.bytes.eq(0)
            yield link.tx_dpp.last.eq(1)
            while True:
                yield

        def scenario():
            for _ in range(128):
                yield
            ack = tp_dw1(TP_SUB_ACK, ept=2, nump=1)
            dph = dph_dw1(seq=0, ept=1, direction=1, length=0)
            for _ in range(3):
                accepted.append(("ack", (yield from push(TYPE_TP, ack))))
            for _ in range(4):
                accepted.append(("dph", (yield from push(TYPE_DP, dph, dpp=1))))
            accepted.append(("ack", (yield from push(TYPE_TP, ack))))
            accepted.append(("dph", (yield from push(TYPE_DP, dph, dpp=1))))

        run_simulation(dut, [host(), dpps(), scenario()])
        self.assertEqual(accepted, [("ack", True)]*3 + [("dph", True)]*4 + [("ack", False), ("dph", False)])

    def test_tx_header_buffers_burst(self):
        # DPHs (Type 2) offered on every cycle, never acknowledged: exactly 4 accepted (Type 2 buffers
        # full flags including the previous pushes, ex: registered with 64-bit words).
        dut  = LinkDUT(self.link_words)
        link = dut.link
        phy  = dut.phy
        accepted = []

        def host():
            words = link_command(LGOOD(15))
            for x in range(4):
                words += link_command(LCRD(x)) + link_command(LCRD2(x))
            yield phy.polling_idle.eq(1)
            yield
            yield phy.polling_idle.eq(0)
            yield phy.u0.eq(1)
            yield phy.sink.ready.eq(1)
            for data, ctrl in words:
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)

        def dpps():
            yield "passive"
            yield link.tx_dpp.valid.eq(1)
            yield link.tx_dpp.bytes.eq(0)
            yield link.tx_dpp.last.eq(1)
            while True:
                yield

        def scenario():
            for _ in range(128):
                yield
            yield link.tx_hp.valid.eq(1)
            yield link.tx_hp.dw0.eq(TYPE_DP)
            yield link.tx_hp.dw1.eq(dph_dw1(seq=0, ept=1, direction=1, length=0))
            yield link.tx_hp.dpp.eq(1)
            for _ in range(256):
                yield
                if (yield link.tx_hp.ready):
                    accepted.append(1)

        run_simulation(dut, [host(), dpps(), scenario()])
        self.assertEqual(len(accepted), 4)

    def test_dph_retransmission_nullified_dpp(self):
        # LBAD received after a DPH: LRTY, then the DPH retransmitted (DL bit set) followed by a
        # nullified DPP (DPPABORT), the Gen 2 framing has a DPP after each DPH (7.2.4.1.13).
        dut  = LinkDUT(self.link_words)
        link = dut.link
        phy  = dut.phy
        sent = []

        def host():
            words = link_command(LGOOD(15))
            for x in range(4):
                words += link_command(LCRD(x)) + link_command(LCRD2(x))
            yield phy.polling_idle.eq(1)
            yield
            yield phy.polling_idle.eq(0)
            yield phy.u0.eq(1)
            yield phy.sink.ready.eq(1)
            for data, ctrl in words:
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)
            # LBAD once the DPH and its DPP are sent.
            while not any(w == (swap(FRAMING_END), 0b1111) for w in sent):
                yield
            for data, ctrl in link_command(0x180): # LBAD.
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)
            for _ in range(256):
                yield

        @passive
        def monitor():
            while True:
                if (yield phy.sink.valid) & (yield phy.sink.ready):
                    sent.append(((yield phy.sink.data), (yield phy.sink.ctrl)))
                yield

        def push():
            for _ in range(128):
                yield
            yield link.tx_hp.valid.eq(1)
            yield link.tx_hp.dw0.eq(TYPE_DP)
            yield link.tx_hp.dw1.eq(dph_dw1(seq=0, ept=1, direction=1, length=4))
            yield link.tx_hp.dpp.eq(1)
            yield
            while not (yield link.tx_hp.ready):
                yield
            yield link.tx_hp.valid.eq(0)

        @passive
        def dpp():
            yield link.tx_dpp.valid.eq(1)
            yield link.tx_dpp.data.eq(0x12345678)
            yield link.tx_dpp.bytes.eq(4)
            yield link.tx_dpp.last.eq(1)
            while True:
                yield

        run_simulation(dut, [host(), monitor(), push(), dpp()])
        # Framing words sent (K Symbols, PHY order): SHP for the Header Packets.
        shp = [i for i, w in enumerate(sent) if w == (swap(FRAMING_SHP), 0b1111)]
        dphs = [i for i in shp if (swap(sent[i + 1][0]) & 0x1f) == TYPE_DP]
        self.assertEqual(len(dphs), 2)
        retry = dphs[1]
        lcw   = swap(sent[retry + 4][0]) >> 16
        self.assertEqual((lcw >> 9) & 1, 1) # DL bit set.
        self.assertEqual(sent[retry + 5], (swap(FRAMING_EDB), 0b1111))

    def test_advertisement_after_link_reset(self):
        # U0 period with Header Packets received (rx_seq advanced, an LGOOD_n not sent yet when U0
        # is exited), then a link reset (Polling.Idle) and a new U0 entry: the first Link Command
        # sent is the Header Sequence Number Advertisement LGOOD_15 (no stale LGOOD_n).
        dut  = LinkDUT(self.link_words)
        link = dut.link
        phy  = dut.phy
        sent = []

        def send(words):
            for data, ctrl in words:
                yield phy.source.valid.eq(1)
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            yield phy.source.valid.eq(0)

        def host():
            yield link.rx_hp.ready.eq(1)
            yield phy.sink.ready.eq(1)
            for period in range(2):
                yield phy.polling_idle.eq(1)
                yield
                yield phy.polling_idle.eq(0)
                yield phy.u0.eq(1)
                if period == 1:
                    sent.clear()
                words = link_command(LGOOD(15))
                for x in range(4):
                    words += link_command(LCRD(x)) + link_command(LCRD2(x))
                yield from send(words)
                if period == 0:
                    # Header Packets received, U0 exited right after (LGOOD_n pending).
                    for seq in range(3):
                        yield from send(tp(TP_SUB_ACK, seq, ept=2))
                    yield phy.sink.ready.eq(0)
                    for _ in range(8):
                        yield
                    yield phy.u0.eq(0)
                    yield phy.sink.ready.eq(1)
                for _ in range(64):
                    yield

        @passive
        def monitor():
            while True:
                if (yield phy.sink.valid) & (yield phy.sink.ready):
                    sent.append(((yield phy.sink.data), (yield phy.sink.ctrl)))
                yield

        run_simulation(dut, [host(), monitor()])
        lcs = [p[1] for p in decode_packets(sent) if p[0] == "LC"]
        self.assertEqual(lcs[0], LGOOD(15))

def swap(w):
    return int.from_bytes(w.to_bytes(4, "little"), "big")

class TestCoreLinkSSPPairs(TestCoreLinkSSP):
    # 64-bit Link Layer words (registered Tx Header Buffers counts, spaced pushes).
    link_words = 2

if __name__ == "__main__":
    unittest.main()
