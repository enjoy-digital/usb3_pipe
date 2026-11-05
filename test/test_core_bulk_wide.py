#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core Bulk tests (test_core_bulk.py) on PHY words pairs/quads: 64/128-bit Link Layer
and Bulk endpoints, 32-bit Control Endpoint through the Protocol Layer adapters, also with the
cascaded CRC-32s (crc32_cascade)."""

import unittest

from test import test_core_bulk

class TestCoreBulkWide(test_core_bulk.TestCoreBulk):
    link_words = 2

class TestCoreBulkWideCascade(test_core_bulk.TestCoreBulk):
    link_words    = 2
    crc32_cascade = True

class TestCoreBulkQuads(test_core_bulk.TestCoreBulk):
    link_words = 4

class TestCoreBulkCascade(test_core_bulk.TestCoreBulk):
    link_words    = 1
    crc32_cascade = True

if __name__ == "__main__":
    unittest.main()
