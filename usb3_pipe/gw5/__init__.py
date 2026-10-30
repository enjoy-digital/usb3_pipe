#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST-138B/GW5AT-15 GTR12 USB3 SerDes configurations (register writes generated with Gowin's
serdes_toml_to_csr from the TOML files, see gen_toml.py/gen_toml_q0.py)."""

import os
import re

# GW5AT-15 Q0 lane 0 with QPLL0: registers set as in Gowin's USB 3.0 PHY IP configuration where
# serdes_toml_to_csr differs ({address: value, None: write removed}). Hardware: with the
# serdes_toml_to_csr CDR settings (0x80025c, 0x800259/0x80025d) the RX 8b/10b errors prevent the link
# training (Polling.Idle timeouts); QPLL0/TX lane clock tree (0x80810c, 0x808120, 0x8082a0) as the IP.
GW5AT15_Q0_QPLL0_OVERRIDES = {
    0x80025c : 0x00000071,
    0x800259 : None,
    0x80025d : None,
    0x80810c : 0x10280108,
    0x808120 : 0x0000001a,
    0x8082a0 : 0x00003150,
}

def gw5_csr_overrides(csr_config, overrides):
    """csr_config with the register writes of overrides replaced ({address: value}) or removed
    ({address: None}), each address written exactly once in csr_config."""
    for addr, value in overrides.items():
        pattern = rf"upar_write_driver\({addr:#08x},0x[0-9A-Fa-f]+\)[^\n]*\n"
        assert len(re.findall(pattern, csr_config)) == 1, f"{addr:#08x}"
        new = "" if value is None else f"upar_write_driver({addr:#08x},{value:#010x}) # Gowin USB 3.0 PHY IP.\n"
        csr_config = re.sub(pattern, new, csr_config)
    return csr_config

def gw5_usb3_csr_config(lane, rate="5G", refclk="100M", position="Q1", pll=None):
    """GTR12 register configuration of Q1 lane (0/1) at rate (5G) from the 100MHz REFCLK1 (100M) or
    the 125MHz REFCLK0 (125M) of the quad (GW5AST-138B, QPLL1), or of Q0 lane 0 from the 125MHz
    REFCLK0 (GW5AT-15: QPLL0, as Gowin's USB 3.0 PHY IP, GW5AT15_Q0_QPLL0_OVERRIDES applied; QPLL1: no
    link on hardware). pll: QPLL0 on Q0, QPLL1 on Q1 by default."""
    assert refclk in ["100M", "125M"]
    assert position in ["Q0", "Q1"]
    if pll is None:
        pll = "QPLL0" if position == "Q0" else "QPLL1"
    assert pll in ["QPLL0", "QPLL1"]
    suffix = "" if refclk == "100M" else "_ref125"
    quad   = "" if position == "Q1" else "_q0"
    if position == "Q0":
        assert (lane, refclk) == (0, "125M")
        suffix += "" if pll == "QPLL1" else "_qpll0"
    else:
        assert pll == "QPLL1"
    csr_config = open(os.path.join(os.path.dirname(__file__), f"usb3_{rate.lower()}{quad}_ln{lane}{suffix}.csr")).read()
    if (position, pll) == ("Q0", "QPLL0"):
        csr_config = gw5_csr_overrides(csr_config, GW5AT15_Q0_QPLL0_OVERRIDES)
    return csr_config
