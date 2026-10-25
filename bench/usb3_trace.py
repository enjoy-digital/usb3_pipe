#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core Header Packets trace (USB3DeviceCore.add_csr(with_trace=True)).

Examples:
  ./bench/usb3_trace.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv clear
  ./bench/usb3_trace.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv dump
"""

import argparse

from common import add_remote_args, open_remote

TYPES       = {0x00: "LMP", 0x04: "TP", 0x08: "DPH", 0x0c: "ITP"}
TP_SUBTYPES = {1: "ACK", 2: "NRDY", 3: "ERDY", 4: "STATUS", 5: "STALL", 6: "DEV_NOTIF", 7: "PING",
    8: "PING_RESP"}

def decode(entry):
    dw1       = entry & 0xffffffff
    typ       = (entry >> 32) & 0x1f
    direction = "TX" if (entry >> 37) & 1 else "RX"
    t         = (entry >> 38)*16
    kind      = TYPES.get(typ, f"0x{typ:02x}")
    if typ == 0x04:
        sub  = TP_SUBTYPES.get(dw1 & 0xf, str(dw1 & 0xf))
        info = (f"{sub:9s} ept {(dw1 >> 8) & 0xf} dir {(dw1 >> 7) & 1} nump {(dw1 >> 16) & 0x1f:2d} "
            f"seq {(dw1 >> 21) & 0x1f:2d}" + (" rty" if (dw1 >> 6) & 1 else ""))
    elif typ == 0x08:
        info = (f"ept {(dw1 >> 8) & 0xf} dir {(dw1 >> 7) & 1} seq {dw1 & 0x1f:2d} len {dw1 >> 16:5d}" +
            (" setup" if (dw1 >> 15) & 1 else "") + (" eob" if (dw1 >> 6) & 1 else ""))
    else:
        info = f"dw1 0x{dw1:08x}"
    return f"{t/125:12.3f}us {direction} {kind:4s} {info}"

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_remote_args(parser)
    parser.add_argument("--prefix",  default=None,                       help="Trace CSRs prefix (default: autodetect).")
    parser.add_argument("command",   choices=["clear", "dump"])
    args = parser.parse_args()

    bus = open_remote(args)
    prefix = args.prefix or [n[:-len("_count")] for n in bus.regs.__dict__ if n.endswith("trace_count")][0]
    reg    = lambda name: getattr(bus.regs, f"{prefix}_{name}")
    if args.command == "clear":
        reg("control").write(0b11)
    else:
        count = reg("count").read()
        print(f"{count} entries")
        for i in range(count):
            reg("index").write(i)
            print(decode(reg("data").read()))
    bus.close()

if __name__ == "__main__":
    main()
