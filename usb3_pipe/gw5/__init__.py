#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gowin GW5AST-138B/GW5AT-15 GTR12 USB3 SerDes configurations (register writes generated with
Gowin's serdes_toml_to_csr from the TOML files, see gen_toml.py/gen_toml_q0.py)."""

import os
import re

# GW5AT-15 Q0 lane 0 with QPLL0: registers set as in Gowin's USB 3.0 PHY IP configuration where
# serdes_toml_to_csr differs ({address: value, None: write removed}). Hardware: with the
# serdes_toml_to_csr CDR settings (0x80025c, 0x800259/0x80025d) the RX 8b/10b errors prevent the
# link training (Polling.Idle timeouts); QPLL0/TX lane clock tree (0x80810c, 0x808120, 0x8082a0) as
# the IP.
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
        new = ("" if value is None else
            f"upar_write_driver({addr:#08x},{value:#010x}) # Gowin USB 3.0 PHY IP.\n")
        csr_config = re.sub(pattern, new, csr_config)
    return csr_config

def gw5_usb3_csr_config(lane, rate="5G", refclk="100M", position="Q1", pll=None):
    """GTR12 register configuration of Q1 lane (0/1) at rate (5G/10G) from the 100MHz REFCLK1 (100M)
    or the 125MHz REFCLK0 (125M, 5G only) of the quad (GW5AST-138B, QPLL1), or of Q0 lane 0 at 5G
    from the 125MHz REFCLK0 (GW5AT-15: QPLL0, as Gowin's USB 3.0 PHY IP, GW5AT15_Q0_QPLL0_OVERRIDES
    applied; QPLL1: no link on hardware). pll: QPLL0 on Q0, QPLL1 on Q1 by default.

    CPLL (Q1 lane 0, 125MHz REFCLK0, 5G/10G): configurations generated from the LiteEth GW5
    10GBASE-R lane configuration (validated 10G CPLL clocking) with the USB3 rates and widths, CDR
    VCO bias 0x7D (as LiteEth: the generated value does not lock), automatic RX equalizer at 5G and
    manual at 10G (boost 12, attenuation 7: the automatic one does not receive a host at 10G through
    the SFP2USB adapter, the manual one prevents the 5G link, measured on hardware). The 10G rate on
    the GW5AST-138B Q1 lanes requires this configuration (the QPLL1 10G configuration gives bit
    errors)."""
    assert refclk in ["100M", "125M"]
    assert position in ["Q0", "Q1"]
    if pll is None:
        pll = "QPLL0" if position == "Q0" else "QPLL1"
    assert pll in ["QPLL0", "QPLL1", "CPLL"]
    if pll == "CPLL":
        assert (position, lane, refclk) == ("Q1", 0, "125M")
        return open(os.path.join(os.path.dirname(__file__),
            f"usb3_{rate.lower()}_ln0_cpll.csr")).read()
    assert (rate == "5G") or ((position, refclk) == ("Q1", "100M"))
    suffix = "" if refclk == "100M" else "_ref125"
    quad   = "" if position == "Q1" else "_q0"
    if position == "Q0":
        assert (lane, refclk) == (0, "125M")
        suffix += "" if pll == "QPLL1" else "_qpll0"
    else:
        assert pll == "QPLL1"
    csr_config = open(os.path.join(os.path.dirname(__file__),
        f"usb3_{rate.lower()}{quad}_ln{lane}{suffix}.csr")).read()
    if (position, pll) == ("Q0", "QPLL0"):
        csr_config = gw5_csr_overrides(csr_config, GW5AT15_Q0_QPLL0_OVERRIDES)
    return csr_config

def gw5_usb3_rate_writes(lane, refclk="100M", pll="QPLL1", clk_freq=62.5e6):
    """Register writes of the 5G <-> 10G rate change of a Q1 lane: [(addr, value_5g, value_10g)],
    the writes of the 5G and 10G configurations that differ (same writes sequence). CPLL: the writes
    done with the CPLL in reset and the CDR signal detect toggled, the CPLL released after 50us,
    then 200us for its lock: [(addr, value_5g, value_10g, wait_cycles)], clk_freq: rate sequencer
    clock frequency. The rate change has to be short: the link partner trains its receiver equalizer
    on the TSEQ Ordered Sets right after the SuperSpeedPlus Polling (Polling.RxEQ, ~7ms at 10Gbps).
    Measured on hardware: with 1ms waits (~4ms rate change) the host does not receive the Gen 2
    Training Sequences, with these waits it does."""
    def writes(rate):
        csr = gw5_usb3_csr_config(lane, rate, refclk, pll=pll)
        return [(int(a, 16), int(v, 16)) for a, v in
            re.findall(r"upar_write_driver\((0x[0-9a-fA-F]+),(0x[0-9a-fA-F]+)\)", csr)]
    w5, w10 = writes("5G"), writes("10G")
    # CPLL: RX equalizer automatic at 5G, manual at 10G (writes set separately below).
    eq = set()
    if pll == "CPLL":
        eq = {0x9003a7, 0x9003a2, GW5_RX_CTLE_BOOST_REG, GW5_RX_CTLE_ATT_REG}
    w5  = [(a, v) for a, v in w5  if a not in eq]
    w10 = [(a, v) for a, v in w10 if a not in eq]
    assert [a for a, v in w5] == [a for a, v in w10]
    diff = [(a, v5, v10) for (a, v5), (_, v10) in zip(w5, w10) if v5 != v10]
    if pll != "CPLL":
        return diff
    cpll_reset  = 0x90a004 + 0x200*lane
    toggle_rxsd = 0x90033e + 0x200*lane
    reset_wait  = int(50e-6*clk_freq)
    lock_wait   = int(200e-6*clk_freq)
    return ([(cpll_reset, 0x22322, 0x22322)] + diff + [
        # RX equalizer: manual at 10G (boost 12, attenuation 8), automatic at 5G (registers at 0, as
        # after the 5G configuration, read on hardware). Measured on hardware (SFP2USB, xHCI Gen 2
        # host, idle link bytes with a bit error per 4s): attenuation 8: ~700, 9: ~900, 7: ~1650,
        # the other boosts/attenuations 5x-50x worse or no link.
        (0x9003a7, 0x60000000, 0x60000000),
        (GW5_RX_CTLE_ATT_REG, 0x00000000, 0x00088881),
        (GW5_RX_CTLE_BOOST_REG, 0x00000000, 0x000cccc1),
        (0x9003a2, 0x00000000, 0x00010000),
        (0x9003a7, 0x00000000, 0x60000000),
        (toggle_rxsd, 0x10000, 0x10000, reset_wait),
        (cpll_reset,  0x22233, 0x22233, lock_wait),
        (toggle_rxsd, 0x00000, 0x00000),
    ])

# Q1 lane 0 registers of the 10Gbps TX FFE (runtime values: USB3 SerDes) and RX CTLE (configuration
# values in gw5_usb3_rate_writes), with the default values of the TX FFE ones.
GW5_TX_LEVEL_REG           = 0x908234
GW5_TX_LEVEL_DEFAULT       = 0xe000
GW5_TX_CURSORS_REG         = 0x908238
GW5_TX_CURSORS_DEFAULT     = 0xb00
GW5_TX_FFE_MANUAL_REG      = 0x9082d8
GW5_TX_FFE_MANUAL_VALUE    = 0x110
GW5_RX_CTLE_BOOST_REG      = 0x9082d0
GW5_RX_CTLE_ATT_REG        = 0x9082d4

# TX Electrical Idle: lane TX driver control register, as written by the Gowin USB 3.2 PHY IP (Gowin
# IDE SerDes IP library) on its TxElecIdle changes (measured on hardware): 0x1 driver in electrical
# idle, 0x7 driver active. The Gowin IP generator also writes 0x1 at initialization.
GW5_TX_IDLE_VALUE   = 0x1
GW5_TX_ACTIVE_VALUE = 0x7

def gw5_tx_idle_reg(lane, position="Q1"):
    """Address of the TX driver control register of a lane (Q0: 0x8003a4, Q1: 0x9003a4, 0x200 per
    lane, as listed by the Gowin IP generator)."""
    assert position in ["Q0", "Q1"]
    return {"Q0": 0x8003a4, "Q1": 0x9003a4}[position] + 0x200*lane
