#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Bench common, imported first by the bench targets/scripts:
- Repository root on the path (usb3_pipe/usb3_core of this checkout, not an installed copy).
- litex_server (RemoteClient) arguments.
- Kernel log reader (/var/log/kern.log, journalctl -k fallback).
- USB sysfs helpers (ports states, SuperSpeed devices, device lookup).
"""

import os
import re
import sys
import glob
import subprocess

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# litex_server -------------------------------------------------------------------------------------

DEFAULT_CSR_CSV = "build/sqrl_acorn/csr.csv"
DEFAULT_PORT    = 1234 # LiteX default litex_server port.

def add_remote_args(parser, csr_csv=DEFAULT_CSR_CSV, csr_csv_help="CSR configuration file."):
    """litex_server arguments: --csr-csv, --host, --port."""
    parser.add_argument("--csr-csv", default=csr_csv,      help=csr_csv_help)
    parser.add_argument("--host",    default="localhost",  help="litex_server host.")
    parser.add_argument("--port",    default=DEFAULT_PORT, help="litex_server port.", type=int)

def open_remote(args):
    """Open a RemoteClient on the litex_server/CSR file from the add_remote_args arguments."""
    from litex import RemoteClient
    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    return bus

# Kernel Log ---------------------------------------------------------------------------------------

DEFAULT_KERN_LOG = "/var/log/kern.log" # adm group.

def add_kern_log_args(parser):
    """Kernel log argument: --kern-log."""
    parser.add_argument("--kern-log", default=DEFAULT_KERN_LOG,
        help=f"Kernel log file (default: {DEFAULT_KERN_LOG}, journalctl -k when missing).")

class KernLog:
    """Kernel log new lines (since the creation/last read): from the kernel log file, or from
    journalctl -k (-f) when the file does not exist (ex: systems without rsyslog)."""
    def __init__(self, path=DEFAULT_KERN_LOG):
        self.path = path
        self.file = None
        self.proc = None
        self.buf  = ""
        if os.path.exists(path):
            self.file = open(path, errors="replace")
            self.file.seek(0, os.SEEK_END)
        else:
            self.proc = subprocess.Popen(["journalctl", "-k", "-f", "-n", "0", "-o", "short"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            os.set_blocking(self.proc.stdout.fileno(), False)

    def new_lines(self):
        if self.file is not None:
            return [l.rstrip("\n") for l in self.file.readlines()]
        data = b""
        while True:
            try:
                chunk = os.read(self.proc.stdout.fileno(), 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            data += chunk
        lines    = (self.buf + data.decode(errors="replace")).split("\n")
        self.buf = lines.pop()
        return lines

    def close(self):
        if self.file is not None:
            self.file.close()
        if self.proc is not None:
            self.proc.terminate()
            self.proc.wait()

def usb_kernel_messages(lines):
    """USB kernel messages of lines (without the timestamp/host prefix)."""
    return [l.split("] ", 1)[-1] for l in lines if re.search(r"\busb\d* ", l)]

def kern_log_lines(path=DEFAULT_KERN_LOG):
    """Kernel log lines (current boot with journalctl -k when the file does not exist)."""
    if os.path.exists(path):
        with open(path, errors="replace") as f:
            return f.read().splitlines()
    return subprocess.run(["journalctl", "-k", "-b", "-o", "short"], stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, text=True, errors="replace").stdout.splitlines()

# USB sysfs ----------------------------------------------------------------------------------------

USB_SYSFS = "/sys/bus/usb/devices"

SUPERSPEED_SPEEDS = ["5000", "10000", "20000"]

def usb_attr(dev, name):
    """USB device sysfs attribute (None when not available)."""
    try:
        return Path(dev, name).read_text().strip()
    except OSError:
        return None

def usb_find_device(product, location=None):
    """Return (busnum, devnum, location) of the USB device with the given product string.

    Several boards can use the same adapter (ex: FT4232H "Quad RS232-HS"): the device is then
    selected by its sysfs location (location or $USB3_PIPE_FTDI, ex: 5-4), never guessed.
    """
    location = location or os.environ.get("USB3_PIPE_FTDI")
    devs = [dev for dev in glob.glob(f"{USB_SYSFS}/*") if usb_attr(dev, "product") == product]
    if location is not None:
        devs = [dev for dev in devs if Path(dev).name == location]
    if len(devs) != 1:
        found = [Path(dev).name for dev in devs]
        raise RuntimeError(f"USB device '{product}': {len(devs)} match(es) {found}, select one with "
            "$USB3_PIPE_FTDI (sysfs location, ex: 5-4).")
    dev      = devs[0]
    busnum   = int(usb_attr(dev, "busnum"))
    devnum   = int(usb_attr(dev, "devnum"))
    location = Path(dev).name.replace("-", ":", 1) # OpenOCD ftdi_location (bus:ports).
    return busnum, devnum, location

def usb_ports_state():
    """USB hub ports link states ({port: state})."""
    states = {}
    for p in sorted(glob.glob(f"{USB_SYSFS}/*/usb*-port*")):
        state = usb_attr(p, "state")
        if state is not None:
            states[Path(p).name] = state
    return states

def usb_superspeed_devices(attrs=["speed", "product"], with_hubs=True):
    """SuperSpeed USB devices ({path: {attr: value}})."""
    devs = {}
    for d in sorted(glob.glob(f"{USB_SYSFS}/[0-9]*-[0-9]*")):
        if ":" in Path(d).name: # Interfaces.
            continue
        if usb_attr(d, "speed") not in SUPERSPEED_SPEEDS:
            continue
        if not with_hubs and usb_attr(d, "bDeviceClass") == "09":
            continue
        devs[Path(d).name] = {k: usb_attr(d, k) for k in attrs}
    return devs
