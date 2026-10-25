#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Pure Python USB3 Host models/helpers for Migen simulations."""

import struct

from usb3_core.msc import CBW_SIGNATURE

# Mass Storage (Bulk-Only Transport) ---------------------------------------------------------------

def cbw(tag, length, din, cb):
    """Command Block Wrapper (31 bytes)."""
    cb = bytes(cb).ljust(16, b"\x00")
    return list(struct.pack("<IIIBBB16s", CBW_SIGNATURE, tag, length, 0x80 if din else 0, 0, len(cb), cb))
