#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 link stability (soak) test: PHY state/counters, host enumeration state and USB kernel
messages logged periodically (JSON lines), optional Bulk loopback batches (--example loopback),
bandwidth benchmark runs (--example bench) and reconnects. Ctrl-C ends the test with the summary.

Examples:
  # 5 minutes, Bulk loopback (64 x 1024 bytes) each period, one reconnect.
  ./scripts/usb3_soak.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv --duration 300 \\
      --bulk 64 --reconnect-every 150 --log soak.jsonl
  # 5 minutes, full duplex benchmark (data checked on both sides) 20s per period.
  ./scripts/usb3_soak.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv --duration 300 \\
      --period 30 --bench 20
"""

import os
import re
import sys
import json
import time
import argparse
import subprocess

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from litex import RemoteClient

from usb3_debug import USB3DebugClient, GTDRPClient, host_status

# Kernel Log ---------------------------------------------------------------------------------------

class KernelLog:
    """New USB kernel messages since the last read (/var/log/kern.log, adm group)."""
    def __init__(self, filename="/var/log/kern.log"):
        self.filename = filename
        self.pos      = os.path.getsize(filename)

    def read(self):
        with open(self.filename, errors="replace") as f:
            f.seek(self.pos)
            lines    = f.read().splitlines()
            self.pos = f.tell()
        return [l.split("] ", 1)[-1] for l in lines if re.search(r"\busb\d* ", l)]

KERNEL_EVENTS = {
    "new_device"    : "new SuperSpeed USB device",
    "disconnect"    : "USB disconnect",
    "reset"         : "reset SuperSpeed USB device",
    "cannot_enable" : "Cannot enable",
    "errors"        : "error",
}

# Bulk ---------------------------------------------------------------------------------------------

def bulk_batch(count):
    # Bulk loopback batch (usb3_bulk.py), returns (packets, errors) or None if not available.
    try:
        import usb3_bulk
        dev = usb3_bulk.open_device()
    except (SystemExit, Exception) as e:
        return {"error": str(e)[:80]}
    try:
        errors = usb3_bulk.loopback(dev, 1024, count)
        return {"packets": count, "errors": errors}
    except Exception as e:
        return {"error": str(e)[:80]}
    finally:
        import usb.util
        usb.util.dispose_resources(dev)

def bench_run(duration, csr_csv, port):
    # Full duplex benchmark (usb3_bench.py both), returns MB/s and errors or the error.
    cmd = [sys.executable, str(Path(__file__).resolve().parent / "usb3_bench.py"), "both",
        "--duration", str(duration), "--csr-csv", csr_csv, "--port", str(port)]
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=duration + 60).stdout
    except subprocess.TimeoutExpired:
        return {"error": "timeout"}
    r = {}
    for direction in ["IN", "OUT"]:
        m = re.search(rf"^{direction}\s*:.*:\s+([\d.]+) MB/s", out, re.M)
        if m:
            r[direction.lower() + "_mbps"] = float(m.group(1))
    m = re.search(r"IN data errors: (\d+), transfer failures: (\d+)", out)
    m2 = re.search(r"Device OUT checker: .* (\d+) errors", out)
    if not m:
        return {"error": out.strip().splitlines()[-1][:80] if out.strip() else "no output"}
    r["errors"] = int(m.group(1)) + int(m.group(2)) + (int(m2.group(1)) if m2 else 0)
    return r

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csr-csv",         default="build/sqrl_acorn/csr.csv", help="CSR configuration file.")
    parser.add_argument("--host",            default="localhost",                help="litex_server host.")
    parser.add_argument("--port",            default=1234, type=int,             help="litex_server port.")
    parser.add_argument("--duration",        default=3600, type=float,           help="Test duration (s).")
    parser.add_argument("--period",          default=10,   type=float,           help="Logging period (s).")
    parser.add_argument("--bulk",            default=0,    type=int,             help="Bulk loopback packets per period (0: disabled, --example loopback).")
    parser.add_argument("--bench",           default=0,    type=float,           help="Full duplex benchmark duration per period (s, 0: disabled, --example bench).")
    parser.add_argument("--reconnect-every", default=0,    type=float,           help="Reconnect period (s, 0: never).")
    parser.add_argument("--log",             default="soak.jsonl",               help="JSON lines log file.")
    args = parser.parse_args()

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    dbg    = USB3DebugClient(bus)
    klog   = KernelLog()
    totals = {k: 0 for k in KERNEL_EVENTS}
    totals.update({"periods": 0, "enumerated": 0, "bulk_packets": 0, "bulk_errors": 0, "bench_runs": 0,
        "bench_errors": 0, "reconnects": 0})
    dbg.clear()
    t0 = last_reconnect = time.time()
    try:
        soak(args, bus, dbg, klog, totals, t0, last_reconnect)
    except KeyboardInterrupt:
        print("Interrupted.")
    totals["phy"] = {k: v for k, v in dbg.status().items() if k in ["u0", "recovery", "u1", "u2", "u3", "lfps_reset"]}
    totals["core"] = core_counters(bus)
    print(json.dumps({"summary": totals}))
    bus.close()

def core_counters(bus):
    # USB3 Device Core statistics (when present).
    return {k: getattr(bus.regs, "usb3_core_" + k).read() for k in ["rx_lbad", "tx_retry", "recovery_req"]
        if hasattr(bus.regs, "usb3_core_" + k)}

def soak(args, bus, dbg, klog, totals, t0, last_reconnect):
    with open(args.log, "a") as log:
        while (time.time() - t0) < args.duration:
            time.sleep(args.period)
            entry = {"t": round(time.time() - t0, 1)}
            if args.reconnect_every and (time.time() - last_reconnect) > args.reconnect_every:
                # Host disconnect/re-enumeration (see usb3_debug.py reconnect).
                drp = GTDRPClient(bus)
                cfg = drp.rxcdr_cfg()
                drp.rxcdr_cfg(cfg ^ (0xc << 16))
                dbg.rx_reset()
                dbg.retrain()
                time.sleep(12)
                drp.rxcdr_cfg(cfg)
                dbg.rx_reset()
                dbg.retrain()
                time.sleep(5)
                last_reconnect = time.time()
                totals["reconnects"] += 1
                entry["reconnect"] = True
            status  = dbg.status()
            host    = host_status(lines=0)
            devices = [d for d in host["devices"] if (d["idVendor"], d["idProduct"]) == ("1d50", "605a")]
            entry["phy"]  = {k: status[k] for k in ["ltssm_state", "u0", "recovery", "u1", "u2", "u3",
                "lfps_reset", "rx_max_good_run", "rx_tx_ppm"] if k in status}
            if hasattr(bus.regs, "xadc_temperature"):
                entry["temperature"] = round(bus.regs.xadc_temperature.read()*503.975/4096 - 273.15, 1)
            entry["host"] = devices[0] if devices else None
            kernel = klog.read()
            entry["kernel"] = {k: sum(v in l for l in kernel) for k, v in KERNEL_EVENTS.items()}
            entry["kernel_lines"] = kernel[-4:]
            if args.bulk and devices:
                entry["bulk"] = bulk_batch(args.bulk)
                totals["bulk_packets"] += entry["bulk"].get("packets", 0)
                totals["bulk_errors"]  += entry["bulk"].get("errors", 0)
            if args.bench and devices:
                entry["bench"] = bench_run(args.bench, args.csr_csv, args.port)
                totals["bench_runs"]   += 1
                totals["bench_errors"] += entry["bench"].get("errors", 1)
            entry["core"] = core_counters(bus)
            totals["periods"]    += 1
            totals["enumerated"] += bool(devices and devices[0].get("bConfigurationValue") == "1")
            for k in KERNEL_EVENTS:
                totals[k] += entry["kernel"][k]
            log.write(json.dumps(entry) + "\n")
            log.flush()
            print(json.dumps({k: entry[k] for k in ["t", "phy", "core", "bench", "temperature"] if k in entry} |
                {"enumerated": bool(devices)}), flush=True)

if __name__ == "__main__":
    main()
