# SPDX-License-Identifier: BSD-2-Clause

"""Debug snapshots must not expose live arbitration to the CSR read mux."""

import unittest

from migen import Cat, Signal
from migen.sim import run_simulation

from test.model.device import BulkLoopbackDUT


class TestCoreStatus(unittest.TestCase):
    def test_link_state_snapshot(self):
        for words in (1, 2, 4):
            with self.subTest(link_words=words):
                dut = BulkLoopbackDUT(link_words=words)
                state = Signal(len(dut.core.link.dbg_state))
                dut.core.link.dbg_state = state
                dut.core.add_csr()
                fields = dut.core._link_state.fields
                snapshot = Cat(*[getattr(fields, f.name) for f in fields.fields])

                def scenario():
                    previous = 0
                    values = [(1 << len(state)) - 1, 0, 0x25c35a69] + [1 << i for i in range(31)]
                    for value in values:
                        yield state.eq(value)
                        yield
                        self.assertEqual((yield snapshot), previous)
                        yield
                        yield
                        self.assertEqual((yield snapshot), value)
                        previous = value

                run_simulation(dut, scenario())
