#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab host tests on a real device (pytest -m hw, AGENTLAB_CSR_CSV=<gateware csr.csv>)."""

import os
import unittest

import pytest

from agentlab.host import constants as c
from agentlab.host.discovery  import find_devices
from agentlab.host.instrument import Instrument

pytestmark = pytest.mark.hw

def open_instrument(**kwargs):
    if not find_devices():
        raise unittest.SkipTest("No AgentLab device connected.")
    return Instrument.open(csr_csv=os.environ.get("AGENTLAB_CSR_CSV"), **kwargs)

class TestHardware(unittest.TestCase):
    def test_discovery(self):
        with open_instrument() as i:
            info = i.info()
            self.assertIsNotNone(info["control"])
            self.assertTrue(all(info["uarts"]), info)

    def test_timestamp_monotonic(self):
        with open_instrument() as i:
            if i.csrs is None:
                self.skipTest("AGENTLAB_CSR_CSV not set.")
            t0 = i.timestamp()
            t1 = i.timestamp()
            self.assertGreater(t1, t0)

    def test_marker_event(self):
        with open_instrument(start_events=True) as i:
            if i.csrs is None:
                self.skipTest("AGENTLAB_CSR_CSV not set.")
            t = i.marker(0x5a)
            e = i.events.wait_for(lambda e: e.type == c.EVENT_MARKER and e.value == 0x5a,
                timeout=2.0)
            self.assertIsNotNone(e, i.events.error)
            self.assertLessEqual(e.timestamp, t)
