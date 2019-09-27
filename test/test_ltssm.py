# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe import ltssm


class TestLTSSM(unittest.TestCase):
    def test_ss_inactive_fsm_syntax(self):
        fsm = ltssm.SSInactiveFSM()

    def test_rx_detect_fsm_syntax(self):
        fsm = ltssm.RXDetectFSM()

    def test_polling_fsm_syntax(self):
        fsm = ltssm.PollingFSM()
