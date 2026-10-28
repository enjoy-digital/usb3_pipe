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

Examples (litex_server --jtag on ports 1234/1235, one per board):
  ./bench/usb3_lanes.py --port 1234 --csr-csv build/acorn_x2_host/csr.csv status
  ./bench/usb3_lanes.py --port 1235 --csr-csv build/acorn_x2_device/csr.csv retrain --wait 2
  ./bench/usb3_lanes.py --port 1235 --csr-csv build/acorn_x2_device/csr.csv prbs --duration 60
  ./bench/usb3_lanes.py --port 1235 --csr-csv build/acorn_x2_device/csr.csv watch
"""

import json
import time
import argparse

from common import add_remote_args, open_remote # First: repository root on the path.

from usb3_debug import USB3DebugClient

LANES_COUNTERS = ["deskew"] + [f"lane{lane}_{name}" for lane in range(2)
    for name in ["ts1", "ts1_inv", "ts2", "rx_errors_cycles"]]

PRBS_COUNTERS = ["tx_words", "rx_words", "rx_packets", "rx_errors"]

PRBS_WORD_BYTES = 8 # 64-bit link words.

# USB3 Lanes Client --------------------------------------------------------------------------------

class USB3LanesClient:
    def __init__(self, bus):
        self.bus = bus
        self.dbg = USB3DebugClient(bus)

    def reg(self, name):
        return getattr(self.bus.regs, name)

    def prbs(self):
        return {name: self.reg(f"usb3_prbs_{name}").read() for name in PRBS_COUNTERS}

    def status(self):
        dbg    = self.dbg.status()
        status = self.reg("usb3_lanes_debug_status").read()
        r = {
            "time_us"        : dbg["time_us"],
            "ltssm_state"    : dbg["ltssm_state"],
            "x2"             : (status >> 0) & 0x1,
            "gen2"           : (status >> 1) & 0x1,
            "deskew_aligned" : (status >> 2) & 0x1,
            "ready"          : [(status >> (4 + lane)) & 0x1 for lane in range(2)],
            "rx_idle"        : [(status >> (6 + lane)) & 0x1 for lane in range(2)],
            "rx_polarity"    : [(status >> (8 + lane)) & 0x1 for lane in range(2)],
            "u0_count"       : dbg["u0"],
            "recovery_count" : dbg["recovery"],
        }
        r.update({name: self.reg(f"usb3_lanes_debug_{name}_count").read() for name in LANES_COUNTERS})
        r.update({f"prbs_{k}": v for k, v in self.prbs().items()})
        return r

    def clear(self):
        # PHY/lanes counters (the PRBS counters are free running, see PRBSLinkTester).
        self.dbg.clear()
        self.reg("usb3_lanes_debug_clear").write(1)

    def prbs_enable(self, enable=True):
        self.reg("usb3_prbs_control").write(int(enable))

def counters_delta(new, old):
    """32-bit wrapping counters differences."""
    return {k: (new[k] - old[k]) % 2**32 for k in new}

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 Gen 1x2 link test client.")
    add_remote_args(parser)
    parser.add_argument("--wait",     default=1.0,  type=float, help="Wait time (s) after retrain.")
    parser.add_argument("--period",   default=1.0,  type=float, help="watch/prbs polling period (s).")
    parser.add_argument("--duration", default=10.0, type=float, help="prbs: test duration (s).")
    parser.add_argument("command",    choices=["status", "clear", "retrain", "watch", "prbs",
        "prbs-enable", "prbs-disable"])
    args = parser.parse_args()

    bus   = open_remote(args)
    lanes = USB3LanesClient(bus)

    if args.command == "status":
        print(json.dumps(lanes.status()))
    elif args.command == "clear":
        lanes.clear()
    elif args.command == "retrain":
        lanes.dbg.retrain()
        lanes.reg("usb3_lanes_debug_clear").write(1)
        time.sleep(args.wait)
        print(json.dumps(lanes.status()))
        count, entries = lanes.dbg.history()
        print(json.dumps({"transitions": count, "history": entries}))
    elif args.command == "watch":
        last = None
        try:
            while True:
                s   = lanes.status()
                key = {k: v for k, v in s.items() if (k != "time_us") and not k.startswith("prbs_")}
                if key != last:
                    print(json.dumps(s), flush=True)
                    last = key
                time.sleep(args.period)
        except KeyboardInterrupt:
            pass
    elif args.command == "prbs":
        # PRBS packets rate/errors from the counters differences (polled faster than the 32-bit
        # counters wrap: ~34s for the words at 125M words/s).
        assert args.period <= 10.0
        start    = time.time()
        last     = lanes.prbs()
        total    = {k: 0 for k in last}
        recovery = lanes.dbg.status()["recovery"]
        while time.time() - start < args.duration:
            time.sleep(args.period)
            t     = time.time()
            prbs  = lanes.prbs()
            delta = counters_delta(prbs, last)
            total = {k: total[k] + delta[k] for k in total}
            last  = prbs
            print(json.dumps({
                "time_s"       : round(t - start, 1),
                "tx_MBps"      : round(delta["tx_words"]*PRBS_WORD_BYTES/args.period/1e6, 1),
                "rx_MBps"      : round(delta["rx_words"]*PRBS_WORD_BYTES/args.period/1e6, 1),
                "rx_errors"    : delta["rx_errors"],
                "ltssm_state"  : lanes.dbg.status()["ltssm_state"],
            }), flush=True)
        elapsed = time.time() - start
        status  = lanes.status()
        print(json.dumps({
            "duration_s"   : round(elapsed, 1),
            "tx_bytes"     : total["tx_words"]*PRBS_WORD_BYTES,
            "rx_bytes"     : total["rx_words"]*PRBS_WORD_BYTES,
            "rx_packets"   : total["rx_packets"],
            "rx_errors"    : total["rx_errors"],
            "rx_MBps"      : round(total["rx_words"]*PRBS_WORD_BYTES/elapsed/1e6, 1),
            "recoveries"   : status["recovery_count"] - recovery,
            "x2"           : status["x2"],
        }))
    elif args.command == "prbs-enable":
        lanes.prbs_enable(True)
    elif args.command == "prbs-disable":
        lanes.prbs_enable(False)

    bus.close()

if __name__ == "__main__":
    main()
