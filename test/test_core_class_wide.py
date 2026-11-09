#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage (test_core_msc.py) and CDC-ACM (test_core_cdc_acm.py) on PHY words pairs:
64-bit Link Layer and Bulk endpoints, 32-bit functions through the endpoint converters (ex: ECP5
at 75MHz, USB3PHY pipe_cd)."""

import unittest

from test import test_core_msc, test_core_cdc_acm

class TestCoreMSCWide(test_core_msc.TestCoreMSC):
    link_words = 2

class TestCoreCDCACMWide(test_core_cdc_acm.TestCoreCDCACM):
    link_words = 2

if __name__ == "__main__":
    unittest.main()
