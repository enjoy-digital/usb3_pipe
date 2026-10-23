#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY/Core resources and timings per block (Vivado out-of-context synthesis + placement).

LiteX designs are flattened (single Verilog module): this script synthesizes each block alone (its
interface signals as ports, sys clock constrained, ports considered registered at the boundary) to
report its LUTs/FFs/BRAMs and its timing (WNS at the target frequency).

Example:
  ./scripts/usb3_resources.py --sys-clk-freq 125e6
  ./scripts/usb3_resources.py --blocks link endpoint_in
"""

import os
import re
import sys
import argparse
import subprocess

from pathlib import Path

from migen import *

from litex.gen import *
from litex.gen.fhdl.verilog import convert

from litex.soc.interconnect import stream

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from usb3_core.host_model  import PHYStandIn
from usb3_core.crc         import CRC16, CRC32
from usb3_core.link        import USB3LinkLayer
from usb3_core.protocol    import USB3ProtocolLayer
from usb3_core.control     import USB3EndpointControl
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore

# Blocks -------------------------------------------------------------------------------------------

def device_core(sys_clk_freq):
    phy = PHYStandIn()
    return USB3DeviceCore(phy, sys_clk_freq, bulk_descriptors(max_burst=8),
        endpoints=[USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)]), phy

BLOCKS = {
    "crc16"        : lambda f: (CRC16(), None),
    "crc32"        : lambda f: (CRC32(), None),
    "link"         : lambda f: (lambda phy: (USB3LinkLayer(phy, sys_clk_freq=f), phy))(PHYStandIn()),
    "ep0"          : lambda f: (USB3EndpointControl(bulk_descriptors()), None),
    "endpoint_in"  : lambda f: (USB3BulkIN(ept=1, max_burst=8), None),
    "endpoint_out" : lambda f: (USB3BulkOUT(ept=2, max_burst=8), None),
    "device"       : device_core,
}

# Ports --------------------------------------------------------------------------------------------

def interface_signals(obj, depth=0):
    """Signals of the block interface (Signals/Records/stream Endpoints attributes, PHY stand-in)."""
    signals = set()
    for name, value in vars(obj).items():
        if name.startswith("_"):
            continue
        if isinstance(value, Signal):
            signals.add(value)
        elif isinstance(value, Record):
            signals.update(value.flatten())
        elif isinstance(value, list) and depth == 0:
            for v in value:
                if isinstance(v, (Signal, Record)):
                    signals.update(interface_signals(type("o", (), {"x": v})(), depth + 1))
    return signals

class Top(LiteXModule):
    def __init__(self, block, phy):
        self.cd_sys = ClockDomain()
        self.block  = block
        self.ios    = {self.cd_sys.clk, self.cd_sys.rst} | interface_signals(block)
        if phy is not None:
            self.ios |= interface_signals(phy)
        for ep in getattr(block, "endpoints", []):
            self.ios |= interface_signals(ep)

# Vivado -------------------------------------------------------------------------------------------

def run_vivado(name, build_dir, part, period):
    tcl = f"""
read_verilog {name}.v
synth_design -top {name} -part {part} -mode out_of_context -flatten_hierarchy rebuilt
create_clock -name sys_clk -period {period} [get_ports sys_clk]
set_input_delay  0 -clock sys_clk [get_ports -filter {{DIRECTION == IN && NAME != sys_clk}}]
set_output_delay 0 -clock sys_clk [all_outputs]
opt_design
place_design
report_utilization -file {name}_utilization.rpt
report_timing_summary -file {name}_timing.rpt
"""
    with open(os.path.join(build_dir, f"{name}.tcl"), "w") as f:
        f.write(tcl)
    subprocess.run(["vivado", "-mode", "batch", "-nojournal", "-nolog", "-source", f"{name}.tcl"],
        cwd=build_dir, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def parse_reports(name, build_dir):
    util   = open(os.path.join(build_dir, f"{name}_utilization.rpt")).read()
    timing = open(os.path.join(build_dir, f"{name}_timing.rpt")).read()
    def util_value(label):
        m = re.search(rf"\|\s*{label}\s*\|\s*([\d.]+)", util)
        return float(m.group(1)) if m else 0
    m = re.search(r"WNS\(ns\).*?\n.*?\n\s*(-?[\d.]+)", timing, re.S)
    return {
        "luts" : int(util_value("Slice LUTs")),
        "ffs"  : int(util_value("Slice Registers")),
        "bram" : util_value("Block RAM Tile"),
        "wns"  : float(m.group(1)) if m else None,
    }

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--blocks",       nargs="+", default=list(BLOCKS), choices=list(BLOCKS))
    parser.add_argument("--sys-clk-freq", default=125e6, type=float, help="Target frequency (Hz).")
    parser.add_argument("--part",         default="xc7a200tfbg484-3",  help="FPGA part.")
    parser.add_argument("--build-dir",    default="build/resources",   help="Build directory.")
    args = parser.parse_args()

    os.makedirs(args.build_dir, exist_ok=True)
    period = 1e9/args.sys_clk_freq
    print(f"| Block | LUTs | FFs | BRAM36 | WNS @ {args.sys_clk_freq/1e6:g}MHz (ns) |")
    print("|---|---|---|---|---|")
    for name in args.blocks:
        block, phy = BLOCKS[name](args.sys_clk_freq)
        top = Top(block, phy)
        convert(top, ios=top.ios, name=name).write(os.path.join(args.build_dir, f"{name}.v"))
        run_vivado(name, args.build_dir, args.part, period)
        r = parse_reports(name, args.build_dir)
        wns = "NA" if r["wns"] is None else f"{r['wns']:+.3f}"
        print(f"| {name} | {r['luts']} | {r['ffs']} | {r['bram']:g} | {wns} |", flush=True)

if __name__ == "__main__":
    main()
