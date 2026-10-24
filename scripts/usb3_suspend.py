#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 suspend/resume (U3) test.

Waits for the host to suspend the device (U3, Linux autosuspend of devices with a driver: UVC,
CDC-ACM, Mass Storage), resumes it by opening its device node, then checks the resume:
- clean: U3 exit (LFPS wakeup handshake, Recovery, U0), no reset from the host.
- reset: the U3 exit failed and the host reset the device (kernel "reset SuperSpeed USB device").
The LTSSM history is captured from U3 (history_freeze) to see where the U3 exit failed.

Requires a litex_server connected to the SoC (USB3PHYDebug) and read access to /var/log/kern.log.

Example (uvc/composite examples):
  ./scripts/usb3_suspend.py --dev /dev/video0 --cycles 8 --csr-csv build/sqrl_acorn/csr.csv
"""

import os
import sys
import time
import argparse
import subprocess

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from litex import RemoteClient

from usb3_debug import USB3DebugClient

# Kernel Log ---------------------------------------------------------------------------------------

class KernLog:
    def __init__(self, path="/var/log/kern.log"):
        self.f = open(path, errors="replace")
        self.f.seek(0, os.SEEK_END)

    def new_lines(self):
        return [l.rstrip() for l in self.f.readlines()]

# Resume -------------------------------------------------------------------------------------------

def resume(dev, timeout=20):
    """Resume the device by using it: stream frames (video) or open the port (tty)."""
    if "video" in dev:
        cmd = ["v4l2-ctl", "-d", dev, "--stream-mmap", "--stream-count=30"]
    else:
        cmd = ["python3", "-c", f"import serial; serial.Serial('{dev}').write(b'\\n')"]
    t0 = time.time()
    r  = subprocess.run(["timeout", str(timeout)] + cmd, capture_output=True, text=True)
    return r.returncode == 0, time.time() - t0

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 suspend/resume (U3) test.")
    parser.add_argument("--dev",     required=True,           help="Device node used to resume the device (/dev/videoX, /dev/ttyACMx).")
    parser.add_argument("--cycles",  default=8,  type=int,    help="Suspend/resume cycles.")
    parser.add_argument("--timeout", default=30, type=float,  help="Suspend wait timeout (s).")
    parser.add_argument("--csr-csv", default="build/sqrl_acorn/csr.csv", help="CSR configuration file.")
    parser.add_argument("--host",    default="localhost",     help="litex_server host.")
    parser.add_argument("--port",    default=1234, type=int,  help="litex_server port.")
    args = parser.parse_args()

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    dbg  = USB3DebugClient(bus)
    kern = KernLog()

    results = {"clean": 0, "reset": 0, "failed": 0, "no_suspend": 0}
    try:
        for cycle in range(args.cycles):
            # Wait for the host suspend (U3).
            t0 = time.time()
            while dbg.status()["ltssm_state"] != "U3" and (time.time() - t0) < args.timeout:
                time.sleep(0.2)
            if dbg.status()["ltssm_state"] != "U3":
                print(f"cycle {cycle}: no suspend (autosuspend disabled?)")
                results["no_suspend"] += 1
                continue

            # Capture the LTSSM history from U3, resume.
            dbg.history_freeze(True)
            dbg.clear()
            kern.new_lines()
            ok, duration = resume(args.dev)
            time.sleep(1.0)
            _, history = dbg.history()
            dbg.history_freeze(False)
            reset = any("reset SuperSpeed" in l for l in kern.new_lines())
            result = "failed" if not ok else ("reset" if reset else "clean")
            results[result] += 1
            print(f"cycle {cycle}: {result} ({duration:.1f}s), LTSSM: " +
                " ".join(state for _, state in history[:8]) + (" ..." if len(history) > 8 else ""))
    finally:
        bus.close()
    print("Results: " + ", ".join(f"{k}: {v}" for k, v in results.items()))

if __name__ == "__main__":
    main()
