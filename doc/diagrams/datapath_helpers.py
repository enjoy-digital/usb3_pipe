#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Shared helpers of the datapath figures (no build(): not a figure).

from svgdiag import *

HEADER = "datapath_helpers"


def strip(d, x, y, cells, h=34, size=13):
    """Row of adjoining square cells: cells = [(width, label, fill)]; returns the boxes."""
    boxes = []
    for w, label, fill in cells:
        boxes.append(d.add(Box(x, y, w, h, label=label, radius=0, size=size, fill=fill)))
        x += w
    return boxes
