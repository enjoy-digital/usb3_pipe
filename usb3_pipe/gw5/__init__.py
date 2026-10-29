#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST-138B GTR12 USB3 SerDes configurations (register writes generated with Gowin's
serdes_toml_to_csr from the TOML files, see gen_toml.py)."""

import os

def gw5_usb3_csr_config(lane, rate="5G", refclk="100M"):
    """GTR12 register configuration of Q1 lane (0/1) at rate (5G) from the 100MHz REFCLK1 (100M) or
    the 125MHz REFCLK0 (125M) of the quad."""
    assert refclk in ["100M", "125M"]
    suffix = "" if refclk == "100M" else "_ref125"
    return open(os.path.join(os.path.dirname(__file__), f"usb3_{rate.lower()}_ln{lane}{suffix}.csr")).read()
