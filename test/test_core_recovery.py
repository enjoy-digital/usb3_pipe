#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core Link Layer Recovery requests (Migen simulation, reactive Python host): a request
is a level until the LTSSM leaves U0, counted once (recovery_req, dbg_recovery pulse, Recovery causes
counters)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model import *

from test.model.host   import *
from test.model.device import BulkLoopbackDUT as DUT

class TestCoreRecovery(unittest.TestCase):
    def test_pending_hp_recovery_counted_once(self):
        # Host not acknowledging the device Header Packets: PENDING_HP_TIMER expires (a level until the
        # LTSSM leaves U0, emulated two cycles after directed_recovery as the LTSSM does): one Recovery
        # request counted, one dbg_recovery pulse, one pending_hp cause.
        dut  = DUT()
        dut.core.add_csr()
        host = PyHost(dut.phy, ack=False)
        link = dut.core.link

        def scenario():
            pulses = 0
            host.send(link_command(LGOOD(7)))
            for x in range(4):
                host.send(link_command(LCRD(x)))
            host.send_hp(lmp_port_capability)
            host.send_hp(lmp_port_configuration)
            # Device Header Packets (not acknowledged), then the Recovery request.
            for _ in range(4000):
                pulses += (yield link.dbg_recovery)
                if (yield dut.phy.directed_recovery):
                    break
                yield
            else:
                self.fail("No Recovery request")
            self.assertGreaterEqual(len(host.hps), 1)
            # LTSSM: U0 left on the registered request, Recovery, back to U0.
            for _ in range(2):
                pulses += (yield link.dbg_recovery)
                yield
            yield dut.phy.u0.eq(0)
            yield dut.phy.recovery.eq(1)
            for _ in range(64):
                pulses += (yield link.dbg_recovery)
                yield
            yield dut.phy.recovery.eq(0)
            yield dut.phy.u0.eq(1)
            for _ in range(16):
                pulses += (yield link.dbg_recovery)
                yield
            self.assertEqual(pulses, 1)
            self.assertEqual((yield link.recovery_req), 1)
            self.assertEqual((yield dut.core._recovery_cause.fields.pending_hp), 1)
            self.assertEqual((yield dut.core._recovery_cause.fields.last), 0b01000)

        run_pyhost(dut, host, scenario())

if __name__ == "__main__":
    unittest.main()
