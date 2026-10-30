#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""GW5AT-15 Q0 USB3 SerDes configuration TOML (single quad devices, ex: Sipeed SLogic16U3).

Usage: python3 gen_toml_q0.py usb3_5g_ln0_ref125.toml [--pll qpll1] > usb3_5g_q0_ln0_ref125.toml, then
       serdes_toml_to_csr_15k.bin $PWD/usb3_5g_q0_ln0_ref125.toml -o $PWD/usb3_5g_q0_ln0_ref125.csr --comment
       (Gowin IDE bin/serdes_toml_to_csr.dist, tested with Gowin 1.9.12.04: absolute paths required).

The GW5AST-138B Q1 lane 0 configuration (validated on hardware, see gen_toml.py) is moved to the Q0
quad: quad settings and lane 0 settings copied from Q1 (reference clock not propagated: single quad,
ref_prop_dir = 0 as in Gowin's USB 3.0 PHY IP configuration for the GW5AT-15), Q1 disabled.

The TOML is then converted to the Gowin 1.9.12 schema (the GW5AT-15 tool rejects the 1.9.10 one):
device/regulator headers, new quad/lane settings at their disabled/default values (resets, POR and
power-down not controlled by the fabric, digital signal detect off, shared RX width, no MIPI clock,
bias defaults), encode/decode_enable replaced by encode/decode_mode. The top/bias/TX driver
registers generated match Gowin's USB 3.0 PHY IP configuration for the GW5AT-15 (Gowin IDE 1.9.12).

--pll selects the lane PLL: qpll1 (default, as on the GW5AST-138B) or qpll0 (as Gowin's USB 3.0 PHY
IP on the GW5AT-15).
"""

import re
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("base")
parser.add_argument("--pll", default="qpll1", choices=["qpll0", "qpll1"])
args = parser.parse_args()

t = open(args.base).read()

def get(t, name):
    pat = re.compile(r"(?ms)^\[" + re.escape(name) + r"\]\s*$(.*?)(?=^\[|\Z)")
    return pat.search(t)

def section(t, name, f):
    m = get(t, name)
    return t[:m.start(1)] + f(m.group(1)) + t[m.end(1):]

def setkv(body, k, v):
    if re.search(rf"(?m)^{k}\s*=", body):
        return re.sub(rf"(?m)^{k}\s*=.*$", f"{k} = {v}", body)
    return body.rstrip("\n") + f"\n{k} = {v}\n\n"

def setkvs(body, kvs):
    for k, v in kvs.items():
        body = setkv(body, k, v)
    return body

# Q1 lane 0 configuration moved to Q0 lane 0.
q1     = get(t, "q1").group(1)
q1_ln0 = get(t, "q1.ln0").group(1).replace('"q1.ln0"', '"q0.ln0"')

t = section(t, "q0",     lambda b: setkv(q1, "ref_prop_dir", 0))
t = section(t, "q0.ln0", lambda b: setkv(q1_ln0, "pcs_tx_clk_src", {"qpll0": 1, "qpll1": 2}[args.pll]))
for name in ["q1", "q1.ln0", "q0.ln1", "q0.ln2", "q0.ln3"]:
    t = section(t, name, lambda b: setkv(b, "enable", "false"))

# Gowin 1.9.12 schema.
quad_kvs = {
    "mclk_freq"            : '"0M"',
    "gpio_freq"            : '"0M"',
    "por_toggle_by_fabric" : "false",
    "pd_toggle_by_fabric"  : "false",
    "cmu0_reset_by_fabric" : "false",
    "cmu1_reset_by_fabric" : "false",
    "bias_tc"              : '"0"',
    "bias_res"             : '"0"',
}
lane_kvs = {
    "cpll_to_mipi_enable"     : "false",
    "cpll_to_mipi_freq"       : '"0M"',
    "rx_seperated_width_mode" : '"share"',
    "cpll_reset_by_fabric"    : "false",
    "rxsd_use_dlogic"         : "false",
    "locked_from_fabric"      : "false",
}
def lane(b):
    for name in ["encode", "decode"]:
        m = re.search(rf"(?m)^{name}_enable\s*=\s*(\w+)\s*\n", b)
        b = b[:m.start()] + b[m.end():]
        b = setkv(b, f"{name}_mode", '"8b10b"' if m.group(1) == "true" else '"OFF"')
    return setkvs(b, lane_kvs)
for q in ["q0", "q1"]:
    t = section(t, q, lambda b: setkvs(b, quad_kvs))
    for l in range(4):
        t = section(t, f"{q}.ln{l}", lane)

t = re.sub(r"(?m)^# .*\n", "", t, count=1)
print(f"# GW5AT-15 Q0 lane 0: 125 MHz REFCLK0, {args.pll.upper()}, 5Gb/s, raw 20-bit PMA, 40-bit fabric, K28.5 alignment.")
print('device = "GW5AT-15"\n\n[regulator]\nregulator_enable = false\n')
print(t, end="")
