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
  ./bench/usb3_trace.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv link
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

# Link Layer trace (USB3LinkTrace).
LC_NAMES = {0x0: "LGOOD", 0x1: "LCRD", 0x2: "LRTY", 0x3: "LBAD"}

def decode_lc(info):
    cls, typ, sub = info >> 9, (info >> 7) & 0x3, info & 0x7f
    if cls == 0:
        if typ == 0:
            return f"LGOOD_{info & 0xf}"
        if typ == 1:
            return f"LCRD{'2' if info & 0x4 else ''}_{'ABCD'[info & 0x3]}"
        return {2: "LRTY", 3: "LBAD"}[typ]
    if cls == 1:
        return {0: f"LGO_U{info & 0x3}", 1: "LAU", 2: "LXU", 3: "LPMA"}[typ]
    if cls == 2:
        return {0: "LUP", 3: "LDN"}.get(typ, f"0x{info:03x}")
    return f"0x{info:03x}"

def decode_link(entry):
    t       = entry & 0xffffffff
    events  = (entry >> 32) & 0x7f
    rx_lc   = (entry >> 39) & 0x7ff
    tx_lc   = (entry >> 50) & 0x7ff
    rx_hp   = (entry >> 61) & 0xffff
    tx_hp   = (entry >> 77) & 0x3ff
    causes  = (entry >> 87) & 0x1f
    r = []
    if events & 0x10: r.append("U0 entry")
    if events & 0x20: r.append("U0 exit")
    if events & 0x01: r.append(f"RX {decode_lc(rx_lc)}")
    if events & 0x02: r.append(f"TX {decode_lc(tx_lc)}")
    if events & 0x04:
        typ, sub, seq = rx_hp & 0x1f, (rx_hp >> 5) & 0xf, (rx_hp >> 9) & 0xf
        ok, acc, seqer = (rx_hp >> 13) & 1, (rx_hp >> 14) & 1, (rx_hp >> 15) & 1
        r.append(f"RX HP {TYPES.get(typ, hex(typ))}" + (f"/{sub}" if typ == 0 else "") +
            f" seq {seq}" + ("" if ok else " CRC-ERR") + (" accepted" if acc else "") + (" SEQ-ERR" if seqer else ""))
    if events & 0x08:
        seq, typ, rty = tx_hp & 0xf, (tx_hp >> 4) & 0x1f, (tx_hp >> 9) & 1
        r.append(f"TX HP {TYPES.get(typ, hex(typ))} seq {seq}" + (" retry" if rty else ""))
    if events & 0x40:
        names = ["seq-err", "lgood", "lcrd", "pending-hp", "credit-hp"]
        r.append("RECOVERY (" + ",".join(n for i, n in enumerate(names) if (causes >> i) & 1) + ")")
    return f"{t/125:12.3f}us " + " | ".join(r)

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_remote_args(parser)
    parser.add_argument("--prefix",  default=None,                       help="Trace CSRs prefix (default: autodetect).")
    parser.add_argument("command",   choices=["clear", "dump", "link", "link-clear"])
    parser.add_argument("--circular", action="store_true", help="Link trace: circular, stops on a U0 exit.")
    parser.add_argument("--filter",   action="store_true", help="Link trace: drop keep-alives (LUP/LDN/ITPs).")
    args = parser.parse_args()

    bus = open_remote(args)
    names  = [n for n in bus.regs.__dict__ if n.endswith("trace_count")]
    prefix = args.prefix or [n[:-len("_count")] for n in names if
        (n.endswith("link_trace_count") == args.command.startswith("link"))][0]
    reg    = lambda name: getattr(bus.regs, f"{prefix}_{name}")
    if args.command == "link-clear":
        # enable, clear, circular (stops on a U0 exit), filter (keep-alives).
        reg("control").write(0b1 | 0b10 | (int(args.circular) << 2) | (int(args.filter) << 3))
    elif args.command == "link":
        count   = reg("count").read()
        pointer = reg("pointer").read()
        depth   = 1024
        print(f"{count} entries")
        start = pointer if count == depth else 0
        for i in range(count):
            reg("index").write((start + i) % depth)
            print(decode_link(reg("data").read()))
    elif args.command == "clear":
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
