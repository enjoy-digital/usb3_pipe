#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST-138B GTR12 USB3 SerDes configurations (register writes generated with Gowin's
serdes_toml_to_csr from the TOML files, see gen_toml.py)."""

import os
import re

def gw5_usb3_csr_config(lane, rate="5G", refclk="100M"):
    """GTR12 register configuration of Q1 lane (0/1) at rate (5G/10G) from the 100MHz REFCLK1 (100M)
    or the 125MHz REFCLK0 (125M, 5G only) of the quad."""
    assert refclk in ["100M", "125M"]
    assert (rate == "5G") or (refclk == "100M")
    suffix = "" if refclk == "100M" else "_ref125"
    return open(os.path.join(os.path.dirname(__file__), f"usb3_{rate.lower()}_ln{lane}{suffix}.csr")).read()

def gw5_usb3_rate_writes(lane, refclk="100M"):
    """Register writes of the 5G <-> 10G rate change of a lane: [(addr, value_5g, value_10g)], the
    writes of the 5G and 10G configurations that differ (same writes sequence)."""
    def writes(rate):
        csr = gw5_usb3_csr_config(lane, rate, refclk)
        return [(int(a, 16), int(v, 16)) for a, v in
            re.findall(r"upar_write_driver\((0x[0-9a-fA-F]+),(0x[0-9a-fA-F]+)\)", csr)]
    w5, w10 = writes("5G"), writes("10G")
    assert [a for a, v in w5] == [a for a, v in w10]
    return [(a, v5, v10) for (a, v5), (_, v10) in zip(w5, w10) if v5 != v10]

# TX Electrical Idle: lane TX driver control register (Q1 lanes), as written by the Gowin USB 3.2 PHY
# IP (Gowin IDE SerDes IP library) on its TxElecIdle changes (measured on hardware): 0x1 driver in
# electrical idle, 0x7 driver active. The Gowin IP generator also writes 0x1 at initialization.
GW5_TX_IDLE_VALUE   = 0x1
GW5_TX_ACTIVE_VALUE = 0x7

def gw5_tx_idle_reg(lane):
    """Address of the TX driver control register of a Q1 lane."""
    return 0x9003a4 + 0x200*lane
