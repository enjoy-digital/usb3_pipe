#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# From U0 to a configured device: link capabilities, then control transfers on endpoint 0.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 400)
    e = Exchange(d, "Device", "Host", fill_left=COLOR_ACCENT, fill_right=COLOR_HARDWARE)
    e.note("link layer", side="left", step=10)
    e.msg("Port Capability LMP", to="right", step=32)
    e.msg("Port Capability LMP", to="left", step=32)
    e.msg("Port Configuration LMP", to="left", step=32)
    e.msg("Port Configuration Response", to="right", step=40)
    e.note("endpoint 0", side="left", step=10)
    e.msg("SET_ADDRESS", to="left", kind="accent", step=32)
    e.msg("GET_DESCRIPTOR (Device)", to="left", kind="accent", step=32)
    e.msg("Device descriptor", to="right", step=32)
    e.msg("GET_DESCRIPTOR (BOS, Configuration)", to="left", kind="accent", step=32)
    e.msg("SET_CONFIGURATION", to="left", kind="accent", step=32)
    return d
