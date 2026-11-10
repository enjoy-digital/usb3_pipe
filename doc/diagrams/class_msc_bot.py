#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Mass Storage Bulk-Only Transport: CBW, data phase and CSW of a READ(10) and a WRITE(10).

from svgdiag import *
from core_helpers import Lanes


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    l = Lanes(d, "Host", "USB3MSC", rows=7, row=33)
    l.msg("CBW: READ(10), 31 bytes", to_right=True, kind="control", note="Bulk OUT")
    l.msg("Data: blocks", to_right=False, kind="accent", note="Bulk IN, from storage")
    l.msg("CSW: status, 13 bytes", to_right=False, kind="control", note="Bulk IN")
    l.gap(0.5)
    l.msg("CBW: WRITE(10)", to_right=True, kind="control", note="Bulk OUT")
    l.msg("Data: blocks", to_right=True, kind="accent", note="Bulk OUT, to storage")
    l.msg("CSW: status", to_right=False, kind="control", note="once storage idle")
    return d
