#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Acorn USB3 hardware test run.

Loads a bitstream (JTAG, SRAM only) on the Acorn (selected by its baseboard FTDI USB product
string), starts litex_server (JTAGBone, on a dedicated port), retrains the link N times through the USB3PHYDebug CSRs and
records for each attempt the PHY status/history and the host side view (new kern.log lines, USB
port states). Results are written to build/hw_runs/<timestamp>/.

Example:
  ./bench/acorn_run.py --build-dir build/sqrl_acorn --attempts 20
"""

import os
import sys
import json
import time
import glob
import signal
import argparse
import subprocess

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from litex import RemoteClient

from usb3_debug import USB3DebugClient

ACORN_FTDI_PRODUCT = "Quad RS232-HS" # LiteX Acorn Baseboard (Mini) FT4232H.

# Helpers ------------------------------------------------------------------------------------------

def find_ftdi(product):
    """Return (busnum, devnum, location) of the USB device with the given product string."""
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            if Path(dev, "product").read_text().strip() == product:
                busnum   = int(Path(dev, "busnum").read_text())
                devnum   = int(Path(dev, "devnum").read_text())
                location = Path(dev).name.replace("-", ":", 1) # OpenOCD ftdi_location (bus:ports).
                return busnum, devnum, location
        except OSError:
            pass
    raise RuntimeError(f"FTDI '{product}' not found.")

def openocd_config(location, filename):
    """OpenOCD config for the Acorn baseboard FT4232H (channel A), selected by USB location."""
    with open(filename, "w") as f:
        f.write(f"""interface ftdi
adapter speed 25000
transport select jtag
ftdi_vid_pid 0x0403 0x6011
ftdi_channel 0
ftdi_location {location}
ftdi_layout_init 0x00e8 0x60eb
reset_config none
source [find cpld/xilinx-xc7.cfg]
""")

def usb_ports_state():
    states = {}
    for p in sorted(glob.glob("/sys/bus/usb/devices/*/usb*-port*")):
        try:
            states[Path(p).name] = Path(p, "state").read_text().strip()
        except OSError:
            pass
    return states

def usb_superspeed_devices():
    devs = {}
    for d in glob.glob("/sys/bus/usb/devices/[0-9]*-[0-9]*"):
        if ":" in Path(d).name:
            continue
        try:
            speed = Path(d, "speed").read_text().strip()
            if speed in ["5000", "10000", "20000"]:
                prod = Path(d, "product").read_text().strip() if Path(d, "product").exists() else ""
                devs[Path(d).name] = {"speed": speed, "product": prod}
        except OSError:
            pass
    return devs

class KernLog:
    def __init__(self, path="/var/log/kern.log"):
        self.path = path
        self.pos  = os.path.getsize(path)

    def new_lines(self):
        with open(self.path, errors="replace") as f:
            f.seek(self.pos)
            lines = f.read()
            self.pos = f.tell()
        return [l for l in lines.splitlines() if "usb" in l.lower() or "xhci" in l.lower()]

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Acorn USB3 hardware test run.")
    parser.add_argument("--build-dir", default="build/sqrl_acorn",  help="Acorn build directory.")
    parser.add_argument("--no-load",   action="store_true",          help="Do not load the bitstream.")
    parser.add_argument("--port",      default=1240, type=int,   help="litex_server port (dedicated).")
    parser.add_argument("--attempts",  default=20, type=int,         help="Link training attempts.")
    parser.add_argument("--wait",      default=2.0, type=float,      help="Time (s) after each retrain.")
    parser.add_argument("--soak",      default=0.0, type=float,      help="Final soak time (s) in the reached state.")
    parser.add_argument("--out",       default=None,                 help="Output directory.")
    args = parser.parse_args()

    build_dir = Path(args.build_dir)
    csr_csv   = build_dir / "csr.csv"
    bitstream = build_dir / "gateware" / "sqrl_acorn.bit"
    out_dir   = Path(args.out or ROOT / "build" / "hw_runs" / time.strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    log = open(out_dir / "run.jsonl", "w")
    def record(kind, **kwargs):
        entry = {"t": time.time(), "kind": kind, **kwargs}
        log.write(json.dumps(entry) + "\n")
        log.flush()
        print(json.dumps(entry), flush=True)

    kern = KernLog()
    busnum, devnum, location = find_ftdi(ACORN_FTDI_PRODUCT)
    record("setup", bitstream=str(bitstream), ftdi=f"{busnum}:{devnum}", location=location,
        ports=usb_ports_state(), ss_devices=usb_superspeed_devices())

    # Load (JTAG, SRAM).
    if not args.no_load:
        r = subprocess.run(["openFPGALoader", "--busdev-num", f"{busnum}:{devnum}", "-c", "ft4232",
            "--write-sram", str(bitstream)], capture_output=True, text=True)
        record("load", returncode=r.returncode, output=(r.stdout + r.stderr)[-2000:])
        if r.returncode != 0:
            sys.exit(1)
        time.sleep(1.0)

    # litex_server (JTAGBone).
    openocd_config(location, out_dir / "openocd.cfg")
    server = subprocess.Popen(["litex_server", "--jtag", "--jtag-config", "openocd.cfg",
        "--jtag-port", str(args.port + 18800), "--bind-port", str(args.port)],
        cwd=out_dir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(5.0)
    bus = RemoteClient(port=args.port, csr_csv=str(csr_csv))
    bus.open()
    ident = "".join(chr(bus.read(bus.bases.identifier_mem + 4*i) & 0xff)
        for i in range(64)).split("\0")[0] if hasattr(bus.bases, "identifier_mem") else ""
    record("server", port=args.port, ident=ident)
    if "USB3SoC" not in ident:
        record("error", msg="Unexpected SoC identifier (wrong board or port?).")
        os.killpg(server.pid, signal.SIGTERM)
        sys.exit(1)

    dbg = USB3DebugClient(bus)
    try:
        record("initial", status=dbg.status(), history=dbg.history()[1], kern=kern.new_lines())
        for attempt in range(args.attempts):
            # A link suspended by the host (U3, ex: autosuspend of the UVC/CDC-ACM examples) is not
            # retrained: the host still sees the port in U3 and would not train it (the device
            # would end in Polling.LFPS timeout -> Compliance).
            if dbg.status()["ltssm_state"] == "U3":
                record("suspended", n=attempt, kern=kern.new_lines())
                time.sleep(args.wait)
                continue
            dbg.retrain()
            time.sleep(args.wait)
            count, history = dbg.history()
            record("attempt", n=attempt, status=dbg.status(), transitions=count, history=history,
                kern=kern.new_lines(), ports=usb_ports_state(), ss_devices=usb_superspeed_devices())
        if args.soak:
            dbg.clear()
            t0 = time.time()
            while time.time() - t0 < args.soak:
                time.sleep(min(10.0, args.soak))
                record("soak", status=dbg.status(), kern=kern.new_lines(), ss_devices=usb_superspeed_devices())
    finally:
        bus.close()
        os.killpg(server.pid, signal.SIGTERM)
        server.wait()
    print(f"Results: {out_dir}")

if __name__ == "__main__":
    main()
