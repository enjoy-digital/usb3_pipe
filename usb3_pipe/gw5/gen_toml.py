#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""GW5AST-138B Q1 USB3 SerDes configuration TOML.

Usage: python3 gen_toml.py <base.toml> <lane> <rate> > usb3_<rate>_ln<lane>.toml, then
       serdes_toml_to_csr.bin usb3_<rate>_ln<lane>.toml -o usb3_<rate>_ln<lane>.csr --comment
       (Gowin IDE bin/serdes_toml_to_csr.dist, tested with Gowin 1.9.10).

base.toml: a complete Gowin Customized PHY TOML for the GW5AST-138 (ex: Sipeed TangMega-138KPro
example, sfp+/customized_phy/10.3125G/src/serdes/serdes.toml, Apache-2.0), the USB3 lane settings
are applied on it: 100 MHz REFCLK1 (Tang Mega 138K Pro dock), raw 20-bit PMA (8b/10b in the fabric),
TX/RX buffers gear for a 125 MHz fabric clock (40-bit at 5Gbps, 80-bit at 10Gbps), K28.5 comma
alignment.
"""

import re, sys
base, lane, rate = sys.argv[1], int(sys.argv[2]), sys.argv[3]
t = open(base).read()
def section(t, name, f):
    pat = re.compile(r"(?ms)^\[" + re.escape(name) + r"\]\s*$(.*?)(?=^\[|\Z)")
    m = pat.search(t)
    return t[:m.start(1)] + f(m.group(1)) + t[m.end(1):]
def setkv(body, k, v):
    if re.search(rf"(?m)^{k}\s*=", body):
        return re.sub(rf"(?m)^{k}\s*=.*$", f"{k} = {v}", body)
    return body.rstrip("\n") + f"\n{k} = {v}\n\n"
def q1(b):
    for k, v in [("ref_pad1_freq", '"100M"'), ("refimux0_sel", 3), ("rx_quad_clk_internal_sel", f'"LN{lane}_PMA_RX_CLK"')]:
        b = setkv(b, k, v)
    return b
gear = {"5G": '"1:2"', "10G": '"1:4"'}[rate]
def ln_on(b):
    for k, v in [("enable", "true"), ("tx_data_rate", f'"{rate}"'), ("rx_data_rate", f'"{rate}"'), ("width_mode", 20),
                 ("tx_gear_rate", gear), ("rx_gear_rate", gear), ("encode_enable", "false"), ("decode_enable", "false"),
                 ("word_align_enable", "true"), ("comma", '"K28.5"'), ("comma_mask", '"1111111111"'), ("loopBack", '"OFF"')]:
        b = setkv(b, k, v)
    return b
def ln_off(b):
    return setkv(b, "enable", "false")
t = section(t, "q1", q1)
t = section(t, f"q1.ln{lane}", ln_on)
t = section(t, f"q1.ln{1-lane}", ln_off)
print(f"# GW5AST-138B Q1 lane {lane}: 100 MHz REFCLK1, {rate}b/s, raw 20-bit PMA, {'40' if rate == '5G' else '80'}-bit fabric, K28.5 alignment.")
print(t, end="")
