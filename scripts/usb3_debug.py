#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY Debug client (USB3PHYDebug CSRs, through litex_server).

Examples:
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv status
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv history
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv retrain --wait 2
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv watch
"""

import sys
import json
import time
import argparse

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from litex import RemoteClient

from usb3_pipe.debug import USB3PHYDebug, USB3PHY_DEBUG_HISTORY_DEPTH
from usb3_pipe.gen   import get_ltssm_states

COUNTERS = [
    "ts1", "ts1_inv", "ts2",
    "lfps_polling", "lfps_ping", "lfps_exit_u1", "lfps_exit_u2", "lfps_wakeup_u3", "lfps_reset",
    "rx_idle", "rx_errors_cycles", "k28_4", "u0", "recovery",
]

RX_COUNTERS = ["symbols", "decoder_invalid", "code_invalid", "disparity", "bursts"]

# USB3 Debug Client --------------------------------------------------------------------------------

class USB3DebugClient:
    def __init__(self, bus, name="usb3_debug"):
        self.bus    = bus
        self.name   = name
        self.states = {v: k for k, v in get_ltssm_states().items()}

    def reg(self, name):
        return getattr(self.bus.regs, f"{self.name}_{name}")

    def state_name(self, value):
        return self.states.get(value, f"?{value}")

    def status(self):
        counters = {name: self.reg(f"{name}_count").read() for name in COUNTERS}
        status   = self.reg("status").read()
        r = {
            "time_us"      : self.reg("timestamp").read(),
            "ltssm_state"  : self.state_name(status & 0xff),
            "serdes_ready" : (status >> 8)  & 0x1,
            "rx_idle"      : (status >> 9)  & 0x1,
            "rx_polarity"  : (status >> 10) & 0x1,
            "transitions"  : self.reg("history_count").read(),
        }
        r.update(counters)
        if hasattr(self.bus.regs, "usb3_serdes_debug_rx_errors_symbols_count"):
            for name in RX_COUNTERS:
                r[f"rx_{name}"] = getattr(self.bus.regs, f"usb3_serdes_debug_rx_errors_{name}_count").read()
            r["rx_max_good_run"] = self.bus.regs.usb3_serdes_debug_rx_errors_max_good_run.read()
            tx_freq = self.bus.regs.usb3_serdes_debug_tx_freq_value.read()
            rx_freq = self.bus.regs.usb3_serdes_debug_rx_freq_value.read()
            r["rx_tx_ppm"] = round(1e6*(rx_freq/max(tx_freq, 1) - 1))
        return r

    def history(self):
        count   = self.reg("history_count").read()
        entries = []
        for i in range(min(count, USB3PHY_DEBUG_HISTORY_DEPTH)):
            self.reg("history_index").write(i)
            entries.append((self.reg("history_time").read(), self.state_name(self.reg("history_state").read())))
        return count, entries

    def clear(self):
        self.reg("clear").write(1)
        if hasattr(self.bus.regs, "usb3_serdes_debug_rx_errors_clear"):
            self.bus.regs.usb3_serdes_debug_rx_errors_clear.write(1)

    def retrain(self, reset_time=0.01):
        # Hold the PHY in reset, clear counters/history and release.
        self.reg("phy_reset").write(1)
        self.clear()
        time.sleep(reset_time)
        self.reg("phy_reset").write(0)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PHY Debug client.")
    parser.add_argument("--csr-csv", default="build/sqrl_acorn/csr.csv", help="CSR configuration file.")
    parser.add_argument("--host",    default="localhost",                help="litex_server host.")
    parser.add_argument("--port",    default=1234, type=int,             help="litex_server port.")
    parser.add_argument("--wait",    default=1.0,  type=float,           help="Wait time (s) after retrain.")
    parser.add_argument("--period",  default=0.2,  type=float,           help="Watch polling period (s).")
    parser.add_argument("command",   choices=["status", "history", "clear", "retrain", "watch"])
    args = parser.parse_args()

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    dbg = USB3DebugClient(bus)

    def print_history():
        count, entries = dbg.history()
        print(json.dumps({"transitions": count, "history": entries}))

    if args.command == "status":
        print(json.dumps(dbg.status()))
    elif args.command == "history":
        print_history()
    elif args.command == "clear":
        dbg.clear()
    elif args.command == "retrain":
        dbg.retrain()
        time.sleep(args.wait)
        print(json.dumps(dbg.status()))
        print_history()
    elif args.command == "watch":
        last = None
        try:
            while True:
                s = dbg.status()
                key = {k: v for k, v in s.items() if k != "time_us"}
                if key != last:
                    print(json.dumps(s), flush=True)
                    last = key
                time.sleep(args.period)
        except KeyboardInterrupt:
            pass

    bus.close()

if __name__ == "__main__":
    main()
