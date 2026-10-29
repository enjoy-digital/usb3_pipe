#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""GW5AT-15 Q0 USB3 SerDes configuration TOML (single quad devices, ex: Sipeed SLogic16U3).

Usage: python3 gen_toml_q0.py usb3_5g_ln0_ref125.toml > usb3_5g_q0_ln0_ref125.toml, then
       serdes_toml_to_csr.bin usb3_5g_q0_ln0_ref125.toml -o usb3_5g_q0_ln0_ref125.csr --comment
       (Gowin IDE bin/serdes_toml_to_csr.dist, tested with Gowin 1.9.10).

The GW5AST-138B Q1 lane 0 configuration (validated on hardware, see gen_toml.py) is moved to the Q0
quad: quad settings and lane 0 settings copied from Q1 (reference clock not propagated: single quad,
ref_prop_dir = 0 as in Gowin's USB 3.0 PHY IP configuration for the GW5AT-15), Q1 disabled.
"""

import re, sys

base = sys.argv[1]
t    = open(base).read()

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

q1     = get(t, "q1").group(1)
q1_ln0 = get(t, "q1.ln0").group(1).replace('"q1.ln0"', '"q0.ln0"')

t = section(t, "q0",     lambda b: setkv(q1, "ref_prop_dir", 0))
t = section(t, "q0.ln0", lambda b: q1_ln0)
for name in ["q1", "q1.ln0", "q0.ln1", "q0.ln2", "q0.ln3"]:
    t = section(t, name, lambda b: setkv(b, "enable", "false"))
t = re.sub(r"(?m)^# .*\n", "", t, count=1)
print("# GW5AT-15 Q0 lane 0: 125 MHz REFCLK0, 5Gb/s, raw 20-bit PMA, 40-bit fabric, K28.5 alignment.")
print(t, end="")
