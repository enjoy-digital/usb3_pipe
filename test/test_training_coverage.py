#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""TSUnit coverage tests against USB 3.2 Rev 1.1 section 6.4.1 (Table 6-4/6-5/6-6) and the LTSSM
Training Sequence requirements (7.5.4: 65536 TSEQ, 16 TS1/TS2, 8 consecutive TS1/TS2 received).

Complements test_training.py / test_ts_checker.py / test_ts2_link_config.py at the TSUnit level.
"""

import random
import unittest

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import COM, SKP, TSEQ, TS1, TS1_INV, TS2, LinkConfig
from usb3_pipe.datapath import RXSKPRemover
from usb3_pipe.training import TSUnit

from test.common import ordered_set_words, stream_driver, symbols_to_words, words_to_symbols

# Spec reference Ordered Sets (Table 6-4 TSEQ, Table 6-5 TS1/TS2), written independently ----------

def D(x, y):
    return (y << 5) | x

K28_5 = (5 << 5) | 28 # COM

def spec_symbols(name, link_functionality=0):
    """(byte, k) symbols of a Training Sequence Ordered Set."""
    if name == "TSEQ":
        data = [D(31, 7), D(23, 0), D(0, 6), D(20, 0), D(18, 5), D(7, 7), D(2, 0), D(2, 4),
                D(18, 3), D(14, 3), D(8, 1), D(6, 5), D(30, 5), D(13, 3), D(31, 5)] + [D(10, 2)]*16
        return [(K28_5, 1)] + [(d, 0) for d in data]
    fill = {"TS1": D(10, 2), "TS1_INV": D(21, 5), "TS2": D(5, 2)}[name]
    return [(K28_5, 1)]*4 + [(D(0, 0), 0), (link_functionality, 0)] + [(fill, 0)]*10

def spec_words(name, link_functionality=0):
    return symbols_to_words(spec_symbols(name, link_functionality))

# Helpers ------------------------------------------------------------------------------------------

class SerDesStub(Module):
    def __init__(self, loopback=False):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        if loopback:
            self.comb += [
                self.sink.ready.eq(1),
                self.source.valid.eq(self.sink.valid),
                self.source.data.eq(self.sink.data),
                self.source.ctrl.eq(self.sink.ctrl),
            ]
        else:
            self.comb += self.source.ready.eq(1) # RX always accepted by the TSUnit.

def ts_unit(loopback=False):
    serdes = SerDesStub(loopback)
    dut    = TSUnit(serdes)
    dut.submodules += serdes
    return dut, serdes

def tx_record(dut, serdes, steps, ready_rate=1.0, seed=0):
    """steps: list of (n_cycles, {signal: value}). Return per cycle (valid, ready, data, ctrl,
    first, last, tx_done)."""
    rng   = random.Random(seed)
    trace = []
    for n, values in steps:
        for name, value in values.items():
            yield getattr(dut, name).eq(value)
        for _ in range(n):
            yield serdes.sink.ready.eq(int(rng.random() < ready_rate))
            yield
            trace.append(((yield serdes.sink.valid), (yield serdes.sink.ready),
                (yield serdes.sink.data), (yield serdes.sink.ctrl), (yield serdes.sink.first),
                (yield serdes.sink.last), (yield dut.tx_done)))
    return trace

def accepted(trace):
    return [t for t in trace if t[0] and t[1]]

# TX -----------------------------------------------------------------------------------------------

class TestTSUnitTX(unittest.TestCase):
    def run_tx(self, steps, ready_rate=1.0):
        dut, serdes = ts_unit()
        dut.trace = None
        def generator(dut):
            dut.trace = yield from tx_record(dut, serdes, steps, ready_rate)
        run_simulation(dut, generator(dut))
        return dut.trace

    def test_ordered_set_structure(self):
        # TSEQ/TS1/TS2 symbols and K flags (Table 6-4/6-5): only COM symbols are K.
        for request, name in [("tx_tseq", "TSEQ"), ("tx_ts1", "TS1"), ("tx_ts2", "TS2")]:
            with self.subTest(name):
                trace = self.run_tx([(48, {"tx_enable": 1, request: 1})])
                words = [(t[2], t[3]) for t in accepted(trace)]
                n     = len(spec_words(name))
                self.assertEqual(words[:2*n], spec_words(name)*2)

    def test_ts1_ts2_burst_size(self):
        # TS1/TS2 bursts of 16 Ordered Sets (64 words), tx_done on the last accepted word, bursts
        # back to back while requested, also with SerDes back-pressure.
        for request in ["tx_ts1", "tx_ts2"]:
            for ready_rate in [1.0, 0.6]:
                with self.subTest(request=request, ready_rate=ready_rate):
                    trace = self.run_tx([(400, {"tx_enable": 1, request: 1})], ready_rate)
                    words = accepted(trace)
                    dones = [i for i, t in enumerate(words) if t[6]]
                    self.assertGreaterEqual(len(dones), 2)
                    self.assertEqual(dones, [64*(k + 1) - 1 for k in range(len(dones))])
                    for i, t in enumerate(words):
                        self.assertEqual(t[3], 0b1111 if i % 4 == 0 else 0)  # COM header each OS.
                        self.assertEqual(t[4], int(i % 64 == 0))             # first
                        self.assertEqual(t[5], int(i % 64 == 63))            # last
                    # tx_done is a pulse only on an accepted last word (never on a stalled one).
                    self.assertEqual(sum(t[6] for t in trace), len(dones))
                    if ready_rate == 1.0:
                        self.assertEqual({t[0] for t in trace}, {1}) # Continuous stream.

    def test_tseq_count(self):
        # Polling.RxEQ: 65536 TSEQ Ordered Sets (7.5.4.4, 6.4.1.1.1). A full run is 524288 cycles:
        # the first Ordered Sets are checked, then the generator word counter is advanced (at an
        # Ordered Set boundary) and the remaining words are counted up to tx_done.
        dut, serdes = ts_unit()
        fragment = dut.get_fragment()
        first    = dut.tseq_generator.source.first
        count    = [s.r.operands[0] for s in fragment.comb if getattr(s, "l", None) is first][0]
        n_words  = 65536*len(spec_words("TSEQ"))
        dut.result = {}
        def generator(dut):
            yield serdes.sink.ready.eq(1)
            yield dut.tx_enable.eq(1)
            yield dut.tx_tseq.eq(1)
            words = []
            while len(words) < 32:
                yield
                if (yield serdes.sink.valid):
                    words.append(((yield serdes.sink.data), (yield serdes.sink.ctrl)))
                    self.assertEqual((yield count), len(words) - 1)
                    self.assertEqual((yield dut.tx_done), 0)
            dut.result["words"] = words
            # Skip to 4 Ordered Sets before the end (counter forced at an Ordered Set boundary).
            skipped = (n_words - 32) - len(words)
            yield count.eq(n_words - 32)
            remaining = 0
            while True:
                yield
                if (yield serdes.sink.valid) & (yield serdes.sink.ready):
                    remaining += 1
                    if remaining == 1:
                        dut.result["resume"] = ((yield serdes.sink.data), (yield serdes.sink.ctrl))
                if (yield dut.tx_done):
                    break
                self.assertLess(remaining, 64)
            dut.result["remaining"] = remaining
            dut.result["sent"]      = len(words) + skipped + remaining
            dut.result["first"] = (yield serdes.sink.first)
            dut.result["last"]  = (yield serdes.sink.last)
            yield
            # Request held: a new TSEQ sequence starts with a COM.
            dut.result["next"] = ((yield serdes.sink.data), (yield serdes.sink.ctrl),
                (yield serdes.sink.first))
        run_simulation(fragment, generator(dut))
        self.assertEqual(dut.result["words"], spec_words("TSEQ")*4)
        self.assertEqual(dut.result["resume"], spec_words("TSEQ")[0])
        self.assertEqual(dut.result["remaining"], 32)
        self.assertEqual(dut.result["sent"], n_words)
        self.assertEqual(dut.result["sent"]//8, 65536)
        self.assertEqual(dut.result["last"], 1)
        self.assertEqual(dut.result["next"], spec_words("TSEQ")[0] + (1,))

    def test_tx_enable_gating(self):
        # Requests without tx_enable: nothing transmitted, no tx_done. Enabling starts a burst on
        # an Ordered Set boundary.
        trace = self.run_tx([
            (100, {"tx_enable": 0, "tx_ts1": 1}),
            (80,  {"tx_enable": 1}),
        ])
        self.assertEqual({t[0] for t in trace[:100]}, {0})
        self.assertEqual(sum(t[6] for t in trace[:100]), 0)
        words = accepted(trace[100:])
        self.assertEqual([(t[2], t[3]) for t in words[:4]], spec_words("TS1"))
        self.assertEqual([i for i, t in enumerate(words) if t[6]], [63])

    def test_tx_enable_deassert_mid_burst(self):
        # tx_enable deasserted mid-burst: transmission stops at the end of the Ordered Set in
        # progress (words 8-11), no tx_done.
        trace = self.run_tx([
            (10, {"tx_enable": 1, "tx_ts2": 1}),
            (100, {"tx_enable": 0}),
        ])
        self.assertEqual(len(accepted(trace)), 12)
        self.assertEqual({t[0] for t in trace[12:]}, {0})
        self.assertEqual(sum(t[6] for t in trace), 0)

    def check_restart_on_ordered_set_boundary(self, steps, request_steps):
        trace = self.run_tx(steps + [(80, request_steps)])
        words = accepted(trace[sum(n for n, _ in steps):])
        # Restarted burst: starts with a full Ordered Set and lasts 16 Ordered Sets.
        self.assertEqual([(t[2], t[3]) for t in words[:4]], spec_words("TS1"))
        self.assertEqual(words[0][4], 1)
        self.assertEqual([i for i, t in enumerate(words) if t[6]], [63])

    def test_tx_enable_reassert_restarts_ordered_set(self):
        # When tx_enable drops mid-burst, the generation is aborted: when re-enabled, the line
        # restarts on an Ordered Set boundary (COM header, 6.4.1.1 / Table 6-5) with a full burst.
        self.check_restart_on_ordered_set_boundary(
            [(6, {"tx_enable": 1, "tx_ts1": 1}), (20, {"tx_enable": 0})],
            {"tx_enable": 1})

    def test_ts1_rerequest_restarts_ordered_set(self):
        # LTSSM path: Polling.Active TS1 interrupted -> Polling.Configuration TS2 -> ... ->
        # Recovery.Active TS1: the TS1 generator restarts a full burst on an Ordered Set boundary.
        self.check_restart_on_ordered_set_boundary(
            [(6, {"tx_enable": 1, "tx_ts1": 1}), (100, {"tx_ts1": 0, "tx_ts2": 1}),
             (4, {"tx_ts2": 0})],
            {"tx_ts1": 1})

    def test_link_functionality_tx(self):
        # Link Functionality symbol (Table 6-6): bit 0 Reset, bit 2 Loopback, bit 3 Disable
        # Scrambling, other bits reserved (0). TS1 always transmitted with 0 by the TSUnit.
        for reset in [0, 1]:
            for loopback in [0, 1]:
                for scrambling in [0, 1]:
                    with self.subTest(reset=reset, loopback=loopback, scrambling=scrambling):
                        values = {"tx_enable": 1, "tx_ts2": 1, "tx_ts2_reset": reset,
                            "tx_ts2_loopback": loopback, "tx_ts2_scrambling": scrambling}
                        trace = self.run_tx([(64, values)])
                        lf    = reset | (loopback << 2) | ((1 - scrambling) << 3)
                        words = [(t[2], t[3]) for t in accepted(trace)]
                        self.assertEqual(words, spec_words("TS2", lf)*16)
        trace = self.run_tx([(64, {"tx_enable": 1, "tx_ts1": 1, "tx_ts2_reset": 1,
            "tx_ts2_loopback": 1, "tx_ts2_scrambling": 0})])
        self.assertEqual([(t[2], t[3]) for t in accepted(trace)], spec_words("TS1", 0)*16)

# RX -----------------------------------------------------------------------------------------------

def rx_detections(words, valid_rate=1.0, with_skp_remover=False, seed=0):
    """Feed words to a TSUnit RX (optionally through the RX SKP remover), return the cycle indexes
    of the rx_ts1/rx_ts1_inv/rx_ts2 pulses and the final (reset, loopback, scrambling)."""
    serdes = SerDesStub()
    dut    = TSUnit(serdes)
    dut.submodules += serdes
    sink   = serdes.source
    if with_skp_remover:
        dut.submodules.skp_remover = skp_remover = RXSKPRemover()
        dut.comb += skp_remover.source.connect(serdes.source)
        sink = skp_remover.sink
    dut.result = {"ts1": [], "ts1_inv": [], "ts2": []}
    dut.done   = False
    def driver(dut):
        yield from stream_driver(sink, words, rng=random.Random(seed), valid_rate=valid_rate)
        for _ in range(16):
            yield
        dut.done = True
    def monitor(dut):
        i = 0
        while not dut.done:
            for n in ["ts1", "ts1_inv", "ts2"]:
                if (yield getattr(dut, "rx_" + n)):
                    dut.result[n].append(i)
            i += 1
            yield
        dut.result["config"] = ((yield dut.rx_ts2_reset), (yield dut.rx_ts2_loopback),
            (yield dut.rx_ts2_scrambling))
    run_simulation(dut, [driver(dut), monitor(dut)])
    return dut.result

def counts(result):
    return (len(result["ts1"]), len(result["ts1_inv"]), len(result["ts2"]))

class TestTSUnitRX(unittest.TestCase):
    def test_ts1_ts1_inv_ts2_discrimination(self):
        # 16 Ordered Sets of each type: 2 detections (8 consecutive, 7.5.4.7/7.5.4.8) of this type
        # only; TSEQ never detected as TS1/TS2.
        for name, expected in [("TS1", (2, 0, 0)), ("TS1_INV", (0, 2, 0)), ("TS2", (0, 0, 2)),
            ("TSEQ", (0, 0, 0))]:
            with self.subTest(name):
                self.assertEqual(counts(rx_detections(spec_words(name)*16)), expected)

    def test_detection_on_last_word_of_8th_ordered_set(self):
        result = rx_detections(spec_words("TS1")*8)
        self.assertEqual(len(result["ts1"]), 1)
        # Pulse on the 32th word (driver/monitor offset of 1 cycle) + TSChecker latency (2 cycles).
        self.assertEqual(result["ts1"][0], 8*4 + 2)

    def test_8_consecutive_required(self):
        # Sequences broken before 8 consecutive Ordered Sets are not detected.
        cases = [
            ("7_ts1",               spec_words("TS1")*7,                                (0, 0, 0)),
            ("7_ts1_ts2_8_ts1",     spec_words("TS1")*7 + spec_words("TS2") + spec_words("TS1")*8, (1, 0, 0)),
            ("ts1_ts2_alternating", (spec_words("TS1") + spec_words("TS2"))*8,          (0, 0, 0)),
            ("4_ts1_4_ts1_inv",     spec_words("TS1")*4 + spec_words("TS1_INV")*4,      (0, 0, 0)),
            ("4_ts2_4_ts2_lf",      spec_words("TS2")*4 + spec_words("TS2", 0x01)*4,    (0, 0, 1)),
        ]
        for name, words, expected in cases:
            with self.subTest(name):
                self.assertEqual(counts(rx_detections(words)), expected)

    def test_valid_gaps(self):
        # Stream gaps (SKP removal, clock compensation) do not break the consecutive count.
        for name in ["TS1", "TS1_INV", "TS2"]:
            with self.subTest(name):
                result = rx_detections(spec_words(name)*16, valid_rate=0.5, seed=1)
                expected = {"TS1": (2, 0, 0), "TS1_INV": (0, 2, 0), "TS2": (0, 0, 2)}[name]
                self.assertEqual(counts(result), expected)

    def test_interleaved_skp(self):
        # SKP Ordered Sets (2 SKP symbols, 6.4.3) between the TS Ordered Sets, removed by the RX
        # SKP remover: 8 consecutive Ordered Sets still detected.
        for name in ["TS1", "TS2"]:
            with self.subTest(name):
                symbols = []
                for i in range(16):
                    symbols += spec_symbols(name)
                    if i in [2, 5, 9, 10]:
                        symbols += [(SKP.value, 1)]*2
                result = rx_detections(symbols_to_words(symbols), with_skp_remover=True)
                expected = {"TS1": (2, 0, 0), "TS2": (0, 0, 2)}[name]
                self.assertEqual(counts(result), expected)

    def test_link_functionality_loopback(self):
        # TS2 Link Functionality through TSUnit TX -> RX: decoded as transmitted (Table 6-6), Reset
        # and Loopback both set: ignored (6.4.1.1.2).
        for reset in [0, 1]:
            for loopback in [0, 1]:
                for scrambling in [0, 1]:
                    with self.subTest(reset=reset, loopback=loopback, scrambling=scrambling):
                        dut, serdes = ts_unit(loopback=True)
                        dut.result = None
                        def generator(dut):
                            yield dut.tx_ts2_reset.eq(reset)
                            yield dut.tx_ts2_loopback.eq(loopback)
                            yield dut.tx_ts2_scrambling.eq(scrambling)
                            yield dut.tx_enable.eq(1)
                            yield dut.tx_ts2.eq(1)
                            detections = 0
                            for _ in range(70):
                                yield
                                detections += (yield dut.rx_ts2)
                            dut.result = (detections, ((yield dut.rx_ts2_reset),
                                (yield dut.rx_ts2_loopback), (yield dut.rx_ts2_scrambling)))
                        run_simulation(dut, generator(dut))
                        if reset and loopback:
                            expected = (0, 0, 1)
                        else:
                            expected = (reset, loopback, scrambling)
                        self.assertEqual(dut.result, (2, expected))

if __name__ == "__main__":
    unittest.main()
