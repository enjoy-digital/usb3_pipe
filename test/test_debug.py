#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from types import SimpleNamespace

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import K
from usb3_pipe.debug  import USB3PHYDebug, USB3PHY_DEBUG_HISTORY_DEPTH

# Fake PHY/SerDes ----------------------------------------------------------------------------------

PIPE_EVENTS = ["ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2", "lfps_rx_polling", "lfps_rx_ping",
    "lfps_rx_exit_u1", "lfps_rx_exit_u2", "lfps_rx_wakeup_u3", "lfps_rx_reset"]

U3 = 30 # Fake LTSSM U3 state encoding.

def fake_phy(pipe_cd=None):
    """PHY/SerDes stand-ins for USB3PHYDebug (pipe_cd: PIPE/SerDes in another clock domain, the
    PIPE events then taken from the LTSSM inputs)."""
    phy = SimpleNamespace(
        reset           = Signal(),
        serdes_rx_reset = Signal(),
        ltssm_state     = Signal(5),
        ltssm_states    = {"U3": U3},
        ltssm           = SimpleNamespace(u0=Signal(), recovery=Signal(), u1=Signal(), u2=Signal(),
            u3=Signal(), **{name: Signal(name=name) for name in PIPE_EVENTS}),
        pipe            = SimpleNamespace(**{name: Signal(name=name) for name in PIPE_EVENTS}),
        pipe_cd         = pipe_cd,
    )
    serdes = SimpleNamespace(
        ready       = Signal(),
        rx_idle     = Signal(),
        rx_polarity = Signal(),
        rx_errors   = Signal(),
        source      = stream.Endpoint([("data", 32), ("ctrl", 4)]),
    )
    return phy, serdes

class DUT(LiteXModule):
    def __init__(self, pipe_cd=None):
        self.phy, self.serdes = fake_phy(pipe_cd)
        self.submodules.debug = USB3PHYDebug(self.phy, self.serdes, sys_clk_freq=4e6)

# Test ---------------------------------------------------------------------------------------------

class TestUSB3PHYDebug(unittest.TestCase):
    def test_counters_history_reset(self):
        dut = DUT()
        dbg = dut.debug
        def generator():
            # Events: edges counted once, levels counted per cycle.
            yield dut.phy.pipe.ts_rx_ts1.eq(1)
            for _ in range(3):
                yield
            yield dut.phy.pipe.ts_rx_ts1.eq(0)
            yield dut.serdes.rx_errors.eq(1)
            for _ in range(5):
                yield
            yield dut.serdes.rx_errors.eq(0)
            yield dut.serdes.source.valid.eq(1)
            yield dut.serdes.source.ready.eq(1)
            yield dut.serdes.source.ctrl.eq(0b0100)
            yield dut.serdes.source.data.eq(K(28, 4) << 16)
            yield
            yield dut.serdes.source.valid.eq(0)
            yield
            yield
            self.assertEqual((yield dbg._ts1_count.status), 1)
            self.assertEqual((yield dbg._rx_errors_cycles_count.status), 5)
            self.assertEqual((yield dbg._k28_4_count.status), 1)

            # History: more transitions than the depth, read back oldest first.
            n = USB3PHY_DEBUG_HISTORY_DEPTH + 3
            for i in range(n):
                yield dut.phy.ltssm_state.eq(1 + (i % 20))
                yield
                yield
            self.assertEqual((yield dbg._history_count.status), n)
            states = []
            for i in range(USB3PHY_DEBUG_HISTORY_DEPTH):
                yield dbg._history_index.storage.eq(i)
                yield
                states.append((yield dbg._history_state.status))
            expected = [1 + (i % 20) for i in range(n)][-USB3PHY_DEBUG_HISTORY_DEPTH:]
            self.assertEqual(states, expected)

            # Status / PHY reset / clear.
            yield dbg._phy_reset.storage.eq(1)
            yield
            self.assertEqual((yield dut.phy.reset), 1)
            self.assertEqual((yield dbg._status.fields.ltssm_state), 1 + ((n - 1) % 20))
            yield dbg._clear.re.eq(1)
            yield
            yield dbg._clear.re.eq(0)
            yield
            self.assertEqual((yield dbg._ts1_count.status), 0)
            self.assertEqual((yield dbg._history_count.status), 0)

        run_simulation(dut, generator())

    def test_pipe_cd_resynchronization(self):
        # PIPE/SerDes in another clock domain (pipe, twice the sys frequency): PIPE events from the
        # LTSSM inputs, SerDes levels and K28.4 pulses resynchronized, each event counted once.
        dut = DUT(pipe_cd="pipe")
        dbg = dut.debug

        def sys_generator():
            yield dut.phy.ltssm.ts_rx_ts1.eq(1)
            yield
            yield dut.phy.ltssm.ts_rx_ts1.eq(0)
            for _ in range(64):
                yield
            self.assertEqual((yield dbg._ts1_count.status), 1)
            self.assertEqual((yield dbg._rx_idle_count.status), 1)
            self.assertEqual((yield dbg._k28_4_count.status), 2)
            self.assertEqual((yield dbg._status.fields.rx_idle), 1)
            self.assertEqual((yield dbg._status.fields.serdes_ready), 1)

        def pipe_generator():
            yield dut.serdes.source.ready.eq(1)
            yield dut.serdes.ready.eq(1)
            for _ in range(4):
                yield
            yield dut.serdes.rx_idle.eq(1)
            # Two K28.4 words (one symbol each), 8 cycles apart (pulse resynchronization).
            for _ in range(2):
                yield dut.serdes.source.valid.eq(1)
                yield dut.serdes.source.data.eq(K(28, 4))
                yield dut.serdes.source.ctrl.eq(0b0001)
                yield
                yield dut.serdes.source.valid.eq(0)
                for _ in range(8):
                    yield
            for _ in range(64):
                yield

        run_simulation(dut, {"sys": [sys_generator()], "pipe": [pipe_generator()]},
            clocks={"sys": 16, "pipe": 8})

    def test_history_u3_exit(self):
        # Frozen full history, armed: the U3 exit restarts it (first entry: the state after U3).
        dut = DUT()
        dbg = dut.debug
        def read_history():
            count  = (yield dbg._history_count.status)
            states = []
            for i in range(min(count, USB3PHY_DEBUG_HISTORY_DEPTH)):
                yield dbg._history_index.storage.eq(i)
                yield
                states.append((yield dbg._history_state.status))
            return states
        def generator():
            yield dbg._history_freeze.storage.eq(1)
            for i in range(USB3PHY_DEBUG_HISTORY_DEPTH + 2):
                yield dut.phy.ltssm_state.eq(1 + (i % 20))
                yield
            self.assertEqual((yield dbg._history_count.status), USB3PHY_DEBUG_HISTORY_DEPTH)
            # Arm, enter U3 (no restart), stay, exit.
            yield dbg._history_u3_exit.storage.eq(1)
            yield dbg._history_u3_exit.re.eq(1)
            yield
            yield dbg._history_u3_exit.re.eq(0)
            for state in [U3, U3, U3, 7, 8, 9, 8]:
                yield dut.phy.ltssm_state.eq(state)
                yield
            yield
            self.assertEqual((yield from read_history()), [7, 8, 9, 8])
            # One shot: a second U3 exit does not restart the history.
            for state in [U3, 5]:
                yield dut.phy.ltssm_state.eq(state)
                yield
            yield
            self.assertEqual((yield from read_history()), [7, 8, 9, 8, U3, 5])

        run_simulation(dut, generator())

if __name__ == "__main__":
    unittest.main()
