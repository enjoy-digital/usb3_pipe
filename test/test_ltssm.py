# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import unittest

from migen import *

from usb3_pipe.ltssm import RXDetectFSM, PollingFSM


class TestLTSSM(unittest.TestCase):
    def test_rx_detect_fsm_syntax(self):
        fsm = RXDetectFSM()

    def test_polling_fsm_syntax(self):
        fsm = PollingFSM()
