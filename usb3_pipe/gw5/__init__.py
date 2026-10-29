#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST-138B/GW5AT-15 GTR12 USB3 SerDes configurations (register writes generated with Gowin's
serdes_toml_to_csr from the TOML files, see gen_toml.py/gen_toml_q0.py)."""

import os

def gw5_usb3_csr_config(lane, rate="5G", refclk="100M", position="Q1"):
    """GTR12 register configuration of Q1 lane (0/1) at rate (5G) from the 100MHz REFCLK1 (100M) or
    the 125MHz REFCLK0 (125M) of the quad (GW5AST-138B), or of Q0 lane 0 from the 125MHz REFCLK0
    (GW5AT-15)."""
    assert refclk in ["100M", "125M"]
    assert position in ["Q0", "Q1"]
    suffix = "" if refclk == "100M" else "_ref125"
    quad   = "" if position == "Q1" else "_q0"
    if position == "Q0":
        assert (lane, refclk) == (0, "125M")
    return open(os.path.join(os.path.dirname(__file__), f"usb3_{rate.lower()}{quad}_ln{lane}{suffix}.csr")).read()
