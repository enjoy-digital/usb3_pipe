#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.common import TS1, TS1_INV, TS2, LinkConfig
from usb3_pipe.training import TSChecker

from test.common import ordered_set_words

# TS Checker (USB 3.2 6.4.1.1, Table 6-4/6-5/6-6) --------------------------------------------------

class TestTSChecker(unittest.TestCase):
    def check(self, ordered_set, link_config, n_ordered_sets=8, n=16):
        """Feed n Ordered Sets with the given Link Functionality byte, return the detections and the
        decoded (reset, loopback, scrambling) at the first detection."""
        words = ordered_set_words(ordered_set, link_config=link_config)*n

        def generator(dut):
            # Words, then idle cycles (detection latency).
            for word in words + [None]*4:
                yield dut.sink.valid.eq(word is not None)
                if word is not None:
                    data, ctrl = word
                    yield dut.sink.data.eq(data)
                    yield dut.sink.ctrl.eq(ctrl)
                yield
                if (yield dut.detected):
                    if dut.config is None:
                        dut.config = ((yield dut.reset), (yield dut.loopback), (yield dut.scrambling))
                    dut.detections += 1
            yield dut.sink.valid.eq(0)

        dut = TSChecker(ordered_set=ordered_set, n_ordered_sets=n_ordered_sets)
        dut.detections = 0
        dut.config     = None
        run_simulation(dut, generator(dut))
        return dut.detections, dut.config

    def test_link_functionality(self):
        # TS1/TS2 with any Link Functionality must be detected and decoded (Table 6-6).
        cases = [
            # (link_config,                                  expected (reset, loopback, scrambling))
            (LinkConfig(reset=0, loopback=0, scrambling=1), (0, 0, 1)),
            (LinkConfig(reset=1, loopback=0, scrambling=1), (1, 0, 1)),
            (LinkConfig(reset=0, loopback=1, scrambling=1), (0, 1, 1)),
            (LinkConfig(reset=0, loopback=0, scrambling=0), (0, 0, 0)),
            (LinkConfig(reset=1, loopback=0, scrambling=0), (1, 0, 0)),
        ]
        for ordered_set in [TS1, TS1_INV, TS2]:
            for link_config, expected in cases:
                with self.subTest(ordered_set=ordered_set.name, link_config=link_config):
                    detections, config = self.check(ordered_set, link_config)
                    self.assertEqual(detections, 2)
                    self.assertEqual(config, expected)

    def test_link_functionality_reset_and_loopback_ignored(self):
        # Bit 0 and bit 2 set simultaneously: all training control bits shall be ignored (6.4.1.1.2).
        link_config = LinkConfig(reset=1, loopback=1, scrambling=0)
        detections, config = self.check(TS2, link_config)
        self.assertEqual(detections, 2)
        self.assertEqual(config, (0, 0, 1))

    def test_link_functionality_only_from_complete_ordered_sets(self):
        # TS1 and TS2 share the same 4 COM symbols: a TS1 (or a corrupted TS2) must not change the
        # Link Functionality reported by the TS2 checker.
        words  = ordered_set_words(TS2, link_config=LinkConfig())*8
        words += ordered_set_words(TS1, link_config=LinkConfig(reset=1, loopback=0, scrambling=0))*8

        def generator(dut):
            for data, ctrl in words:
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(data)
                yield dut.sink.ctrl.eq(ctrl)
                yield
            yield dut.sink.valid.eq(0)
            yield
            dut.config = ((yield dut.reset), (yield dut.loopback), (yield dut.scrambling))

        dut = TSChecker(ordered_set=TS2, n_ordered_sets=8)
        run_simulation(dut, generator(dut))
        self.assertEqual(dut.config, (0, 0, 1))

    def test_error_on_last_word_not_detected(self):
        # 8 consecutive Ordered Sets must be received without error (6.4.1.1.2): a corrupted last
        # word of the 8th Ordered Set must not report a detection.
        for ordered_set in [TS1, TS2]:
            with self.subTest(ordered_set=ordered_set.name):
                words = ordered_set_words(ordered_set)*8
                words[-1] = (words[-1][0] ^ 0x1, words[-1][1])

                def generator(dut):
                    dut.detections = 0
                    for data, ctrl in words:
                        yield dut.sink.valid.eq(1)
                        yield dut.sink.data.eq(data)
                        yield dut.sink.ctrl.eq(ctrl)
                        yield
                        dut.detections += (yield dut.detected)
                    yield dut.sink.valid.eq(0)
                    for i in range(4): # Detection latency.
                        yield
                        dut.detections += (yield dut.detected)

                dut = TSChecker(ordered_set=ordered_set, n_ordered_sets=8)
                run_simulation(dut, generator(dut))
                self.assertEqual(dut.detections, 0)

if __name__ == "__main__":
    unittest.main()
