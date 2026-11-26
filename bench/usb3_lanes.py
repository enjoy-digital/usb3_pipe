#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Gen 1x2 link test client (bench/acorn.py --lanes 2: USB3PHYDebug, USB3LanesDebug and
PRBSLinkTester CSRs, through litex_server), one board per litex_server.

Two Acorn Baseboard Minis, SFP-0 <-> SFP-0 and SFP-1 <-> SFP-1 (SFP DAC cables), one board in each
role:
  ./bench/acorn.py --lanes 2 --role host   --with-jtagbone --output-dir build/acorn_x2_host   --build --load
  ./bench/acorn.py --lanes 2 --role device --with-jtagbone --output-dir build/acorn_x2_device --build --load

Load the Device board last: a Device started alone times out to Compliance (no partner LFPS) and
only leaves it on a Warm Reset (retrain the Host board then).

Examples (one litex_server --jtag per board, ports 1234/1235):
  ./bench/usb3_lanes.py --port 1234 --csr-csv build/acorn_x2_host/csr.csv status
  ./bench/usb3_lanes.py --port 1235 --csr-csv build/acorn_x2_device/csr.csv prbs --duration 60
  ./bench/usb3_lanes.py --port 1234 --csr-csv build/acorn_x2_host/csr.csv x2-disable
  ./bench/usb3_lanes.py --port 1234 --csr-csv build/acorn_x2_host/csr.csv watch
"""

import json
import time
import argparse

from common import add_remote_args, open_remote # First: repository root on the path.

from usb3_debug import USB3DebugClient

PRBS_COUNTERS = ["tx_words", "rx_words", "rx_packets", "rx_errors"]

# USB3 Lanes Client --------------------------------------------------------------------------------

class USB3LanesClient:
    def __init__(self, bus):
        self.bus = bus
        self.dbg = USB3DebugClient(bus)

    def reg(self, name):
        return getattr(self.bus.regs, name)

    def has(self, name):
        return hasattr(self.bus.regs, name)

    def status(self):
        dbg    = self.dbg.status()
        status = self.reg("usb3_lanes_debug_status").read()
        r = {
            "time_us"        : dbg["time_us"],
            "ltssm_state"    : dbg["ltssm_state"],
            "x2"             : (status >> 0) & 0x1,
            "deskew_aligned" : (status >> 2) & 0x1,
            "lane_swap"      : (status >> 3) & 0x1,
            "ready"          : [(status >> (4 + lane)) & 0x1 for lane in range(2)],
            "rx_idle"        : [(status >> (6 + lane)) & 0x1 for lane in range(2)],
            "rx_polarity"    : [(status >> (8 + lane)) & 0x1 for lane in range(2)],
            "u0_count"       : dbg.get("u0"),
            "recovery_count" : dbg.get("recovery"),
        }
        # Lanes event/cycle counters (USB3LanesDebug: deskew, Recovery triggers, per lane TS/errors).
        prefix = "usb3_lanes_debug_"
        for name in sorted(self.bus.regs.__dict__):
            if name.startswith(prefix) and name.endswith("_count"):
                r[name[len(prefix):-len("_count")]] = self.reg(name).read()
        return r

    def prbs(self):
        return {name: self.reg(f"usb3_prbs_{name}").read() for name in PRBS_COUNTERS}

    def clear(self):
        self.dbg.clear()
        self.reg("usb3_lanes_debug_clear").write(1)

    def x2_enable(self, enable):
        self.reg("usb3_lanes_debug_control").write(int(enable))

# Run ----------------------------------------------------------------------------------------------

def counters_delta(new, old):
    return {k: v - old.get(k, 0) for k, v in new.items()}

def main():
    parser = argparse.ArgumentParser(description="USB3 Gen 1x2 link test client.")
    add_remote_args(parser)
    parser.add_argument("--wait",     default=1.0,  type=float, help="Wait time (s) after a retrain.")
    parser.add_argument("--period",   default=1.0,  type=float, help="watch/prbs polling period (s).")
    parser.add_argument("--duration", default=10.0, type=float, help="prbs: test duration (s).")
    parser.add_argument("command", choices=["status", "clear", "retrain", "watch", "prbs",
        "x2-enable", "x2-disable"])
    args = parser.parse_args()

    bus   = open_remote(args)
    lanes = USB3LanesClient(bus)

    if args.command == "status":
        print(json.dumps(lanes.status()))
    elif args.command == "clear":
        lanes.clear()
    elif args.command in ["retrain", "x2-enable", "x2-disable"]:
        # PHY reset (new training: SuperSpeedPlus Polling, x2 negotiated again), status after --wait.
        # x2-enable/x2-disable: Gen 1x2 capability advertised (x2-disable: Gen 1x1 negotiated).
        if args.command != "retrain":
            lanes.x2_enable(args.command == "x2-enable")
        lanes.dbg.retrain()
        lanes.clear()
        time.sleep(args.wait)
        print(json.dumps(lanes.status()))
    elif args.command == "watch":
        while True:
            s = lanes.status()
            print(f"{s['time_us']/1e6:10.3f}s {s['ltssm_state']:24s} x2: {s['x2']} "
                  f"deskew: {s['deskew_aligned']} swap: {s['lane_swap']} "
                  f"U0: {s['u0_count']} Recovery: {s['recovery_count']}")
            time.sleep(args.period)
    elif args.command == "prbs":
        # PRBS Link-Layer stand-ins: words/errors in U0 over --duration (both boards send and check).
        assert lanes.has("usb3_prbs_rx_errors"), "PRBS stand-ins not in this build (--lanes-core)."
        start = lanes.prbs()
        t0    = time.time()
        while time.time() - t0 < args.duration:
            time.sleep(args.period)
        delta = counters_delta(lanes.prbs(), start)
        x2    = lanes.status()["x2"]
        delta["rx_MBps"] = round(delta["rx_words"]*(8 if x2 else 4)/(time.time() - t0)/1e6, 1)
        delta["x2"]      = x2
        print(json.dumps(delta))
    bus.close()

if __name__ == "__main__":
    main()
