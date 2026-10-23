#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import unittest
import subprocess

from types   import SimpleNamespace
from pathlib import Path

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import K
from usb3_pipe.debug  import USB3PHYDebug, USB3PHY_DEBUG_HISTORY_DEPTH

# Fake PHY/SerDes ----------------------------------------------------------------------------------

PIPE_EVENTS = ["ts_rx_ts1", "ts_rx_ts1_inv", "ts_rx_ts2", "lfps_rx_polling", "lfps_rx_ping",
    "lfps_rx_exit_u1", "lfps_rx_exit_u2", "lfps_rx_wakeup_u3", "lfps_rx_reset"]

def fake_phy():
    phy = SimpleNamespace(
        reset       = Signal(),
        ltssm_state = Signal(5),
        ltssm       = SimpleNamespace(u0=Signal(), recovery=Signal()),
        pipe        = SimpleNamespace(**{name: Signal(name=name) for name in PIPE_EVENTS}),
    )
    serdes = SimpleNamespace(
        ready       = Signal(),
        rx_idle     = Signal(),
        rx_polarity = Signal(),
        rx_errors   = Signal(),
        source      = stream.Endpoint([("data", 32), ("ctrl", 4)]),
    )
    return phy, serdes

class DUT(Module):
    def __init__(self):
        self.phy, self.serdes = fake_phy()
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
            self.assertEqual((yield dbg._status.status) & 0xff, 1 + ((n - 1) % 20))
            yield dbg._clear.re.eq(1)
            yield
            yield dbg._clear.re.eq(0)
            yield
            self.assertEqual((yield dbg._ts1_count.status), 0)
            self.assertEqual((yield dbg._history_count.status), 0)

        run_simulation(dut, generator())

    def test_acorn_elaboration(self):
        # Acorn target elaborates with the USB3PHY + debug + each analyzer group (no Vivado).
        root = Path(__file__).resolve().parents[1]
        for args in [["--no-core", "--with-jtagbone", "--with-serdes-debug"],
                     ["--no-core", "--with-analyzer", "--analyzer-group", "lfps"],
                     ["--no-core", "--with-analyzer", "--analyzer-group", "datapath"],
                     ["--no-core", "--with-serdes-debug", "--with-analyzer", "--analyzer-group", "phase",
                      "--lpm-hold", "rxeq", "--gtp-param", "p_RXCDR_HOLD_DURING_EIDLE=1"]]:
            with self.subTest(args=args):
                p = subprocess.run([sys.executable, "acorn.py", "--output-dir", "build/test_acorn"] + args,
                    cwd=str(root), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                self.assertEqual(p.returncode, 0, p.stdout[-3000:])
                csr = (root / "build" / "test_acorn" / "csr.csv").read_text()
                self.assertIn("usb3_debug_history_count", csr)
                self.assertIn("usb3_debug_serdes_rx_reset", csr)
                if "--with-serdes-debug" in args:
                    for name in ["usb3_serdes_debug_drp_sel", "usb3_serdes_debug_drp_addr",
                                 "usb3_serdes_debug_tx_inhibit_state"]:
                        self.assertIn(name, csr)
