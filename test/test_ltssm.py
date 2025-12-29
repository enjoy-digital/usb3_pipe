#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe import ltssm


class TestLTSSM(unittest.TestCase):
    def test_ltssm_fsm_syntax(self):
        fsm = ltssm.LTSSMFSM()

    def test_ss_inactive_fsm_syntax(self):
        fsm = ltssm.SSInactiveFSM()

    def test_rx_detect_fsm_syntax(self):
        fsm = ltssm.RXDetectFSM()

    #def test_polling_fsm_syntax(self):
    #    fsm = ltssm.PollingFSM()
