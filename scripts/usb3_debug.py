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

SerDes (--with-serdes-debug):
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv rx-reset
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv rx-check --count 12
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv drp-read 0xa8
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv drp-write 0xa9 0x2448
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv rxcdr-cfg
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv rxcdr-cfg 0x87fe206024481010
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv drp-dump drp.json
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv tx-inhibit-state Polling.Active
  ./scripts/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv reconnect --wait 12

Host (Linux, kern.log readable: adm group):
  ./scripts/usb3_debug.py host
"""

import os
import re
import sys
import glob
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

# GTPE2_CHANNEL DRP addresses (UG482 Table D-2).
GTP_DRP_RXCDR_CFG = [0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad] # [15:0], ..., [82:80].

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

    def rx_reset(self, reset_time=0.01):
        # SerDes RX re-initialization (PMA/CDR/offset calibration), ex after DRP changes.
        self.reg("serdes_rx_reset").write(1)
        time.sleep(reset_time)
        self.reg("serdes_rx_reset").write(0)
        time.sleep(0.05)

    def rx_check(self, timeout=3.0, good_run=1000000):
        # RX quality on a link training: max error-free RX run (symbols, millions on a healthy
        # receiver) while the partner sends Training Sequences. "good"/"bad", or "no_partner" when
        # no TS is received (ex: host in Rx.Detect/eSS.Inactive, between its retries): the RX
        # quality can't be measured then.
        self.retrain()
        t0 = time.time()
        while (time.time() - t0) < timeout:
            s = self.status()
            if s.get("rx_max_good_run", 0) > good_run:
                break
            time.sleep(0.2)
        run = s.get("rx_max_good_run", 0)
        if run > good_run:
            return "good", run
        return ("bad" if (s["ts1"] + s["ts1_inv"] + s["ts2"]) else "no_partner"), run

# Transceiver DRP Client ---------------------------------------------------------------------------

class GTDRPClient:
    """Transceiver DRP access (USB3SerDesDebug DRPControl)."""
    def __init__(self, bus, name="usb3_serdes_debug"):
        self.bus  = bus
        self.name = name

    def reg(self, name):
        return getattr(self.bus.regs, f"{self.name}_{name}")

    def access(self, addr, we, di=0):
        self.reg("drp_sel").write(1)
        self.reg("drp_addr").write(addr)
        self.reg("drp_di").write(di)
        self.reg("drp_we").write(we)
        self.reg("drp_start").write(1)
        for i in range(100):
            if self.reg("drp_done").read():
                break
        else:
            raise RuntimeError("DRP access timeout.")
        do = self.reg("drp_do").read()
        self.reg("drp_sel").write(0)
        return do

    def read(self, addr):
        return self.access(addr, we=0)

    def write(self, addr, value):
        self.access(addr, we=1, di=value)

    def field(self, addr, msb, lsb, value=None):
        word = self.read(addr)
        mask = ((1 << (msb - lsb + 1)) - 1) << lsb
        if value is None:
            return (word & mask) >> lsb
        self.write(addr, (word & ~mask) | ((value << lsb) & mask))

    def rxcdr_cfg(self, value=None):
        if value is None:
            return sum(self.read(a) << (16*i) for i, a in enumerate(GTP_DRP_RXCDR_CFG))
        for i, a in enumerate(GTP_DRP_RXCDR_CFG):
            self.write(a, (value >> (16*i)) & (0x7 if i == 5 else 0xffff))

# Host (Linux) -------------------------------------------------------------------------------------

def host_status(lines=20):
    # SuperSpeed ports/devices (sysfs) and the last USB kernel messages.
    r = {"devices": [], "kern_log": []}
    for d in glob.glob("/sys/bus/usb/devices/*"):
        def attr(name):
            try:
                return open(os.path.join(d, name)).read().strip()
            except OSError:
                return None
        if attr("speed") in ["5000", "10000", "20000"] and attr("idVendor") and attr("bDeviceClass") != "09":
            r["devices"].append({k: attr(k) for k in ["idVendor", "idProduct", "manufacturer", "product",
                "serial", "speed", "bConfigurationValue"]} | {"path": os.path.basename(d)})
    try:
        with open("/var/log/kern.log", errors="replace") as f:
            r["kern_log"] = [l.split("] ", 1)[-1] for l in f.read().splitlines() if re.search(r"\busb\d* ", l)][-lines:]
    except OSError as e:
        r["kern_log"] = [str(e)]
    return r

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PHY Debug client.")
    parser.add_argument("--csr-csv", default="build/sqrl_acorn/csr.csv", help="CSR configuration file.")
    parser.add_argument("--host",    default="localhost",                help="litex_server host.")
    parser.add_argument("--port",    default=1234, type=int,             help="litex_server port.")
    parser.add_argument("--wait",    default=1.0,  type=float,           help="Wait time (s) after retrain.")
    parser.add_argument("--period",  default=0.2,  type=float,           help="Watch polling period (s).")
    parser.add_argument("--count",   default=12,   type=int,             help="rx-check: number of RX re-initializations.")
    parser.add_argument("command",   choices=["status", "history", "clear", "retrain", "watch",
        "rx-reset", "rx-check", "drp-read", "drp-write", "drp-dump", "rxcdr-cfg", "tx-inhibit-state", "reconnect",
        "host"])
    parser.add_argument("args",      nargs="*", help="Command arguments.")
    args = parser.parse_args()

    if args.command == "host":
        print(json.dumps(host_status(), indent=1))
        return

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
    elif args.command == "rx-reset":
        dbg.rx_reset()
    elif args.command == "rx-check":
        # SerDes RX re-initialization outcome over several resets (RX init does not always end in a
        # good state, the PHY re-initializes it on training failures).
        results = []
        for i in range(args.count):
            dbg.rx_reset()
            result, run = dbg.rx_check()
            results.append(result)
            print(json.dumps({"rx_reset": i, "result": result, "rx_max_good_run": run}), flush=True)
        print(json.dumps({r: results.count(r) for r in ["good", "bad", "no_partner"]}))
    elif args.command in ["drp-read", "drp-write", "drp-dump", "rxcdr-cfg"]:
        drp = GTDRPClient(bus)
        if args.command == "drp-read":
            addr = int(args.args[0], 0)
            print(f"0x{addr:03x}: 0x{drp.read(addr):04x}")
        elif args.command == "drp-write":
            drp.write(int(args.args[0], 0), int(args.args[1], 0))
        elif args.command == "drp-dump":
            dump = {f"0x{a:03x}": drp.read(a) for a in range(0x000, 0x0b0)}
            json.dump(dump, open(args.args[0], "w"), indent=1)
        elif args.command == "rxcdr-cfg":
            if len(args.args):
                drp.rxcdr_cfg(int(args.args[0], 0))
                dbg.rx_reset() # CDR settings are applied on RX re-initialization (UG482).
            print(f"RXCDR_CFG: 0x{drp.rxcdr_cfg():021x}")
    elif args.command == "reconnect":
        # Host disconnect/re-enumeration: break the link (RX unable to lock with a wrong RXCDR_CFG)
        # long enough for the host to disconnect the device (a retrain alone only triggers a Warm
        # Reset), then restore it.
        drp = GTDRPClient(bus)
        cfg = drp.rxcdr_cfg()
        drp.rxcdr_cfg(cfg ^ (0xc << 16))
        dbg.rx_reset()
        dbg.retrain()
        time.sleep(args.wait)
        drp.rxcdr_cfg(cfg)
        dbg.rx_reset()
        dbg.retrain()
    elif args.command == "tx-inhibit-state":
        # Inhibit TX while in an LTSSM state (ex: Polling.Active, to observe the partner's signal
        # without our TX), no argument: disable.
        states = get_ltssm_states()
        value  = (0x100 | states[args.args[0]]) if len(args.args) else 0
        bus.regs.usb3_serdes_debug_tx_inhibit_state.write(value)

    bus.close()

if __name__ == "__main__":
    main()
