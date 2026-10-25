#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Bulk loopback test (acorn.py: EP2 OUT -> FIFO -> EP1 IN), through libusb (pyusb).

The device must be accessible to the user (udev rule: bench/99-usb3-pipe.rules):
  sudo cp bench/99-usb3-pipe.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules
  (then re-enumerate the device, ex: ./bench/usb3_debug.py reconnect)

Examples:
  ./bench/usb3_bulk.py info
  ./bench/usb3_bulk.py loopback --size 1024 --count 1000
  ./bench/usb3_bulk.py loopback --size 64 --count 1000 --random-sizes
"""

import sys
import time
import random
import argparse

import usb.core
import usb.util

USB3_VID = 0x1d50
USB3_PID = 0x605a
EP_OUT   = 0x02
EP_IN    = 0x81
MAX_SIZE = 1024 # SuperSpeed Bulk max packet size (one packet per transfer: loopback FIFO depth).

# Device -------------------------------------------------------------------------------------------

def open_device(vid=USB3_VID, pid=USB3_PID):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        raise SystemExit(f"Device {vid:04x}:{pid:04x} not found.")
    try:
        if dev.get_active_configuration() is None:
            dev.set_configuration()
    except usb.core.USBError as e:
        if e.errno == 13:
            raise SystemExit("Access denied: install bench/99-usb3-pipe.rules (see --help).")
        dev.set_configuration()
    usb.util.claim_interface(dev, 0)
    return dev

def info(dev):
    print(f"Device {dev.idVendor:04x}:{dev.idProduct:04x}, bus {dev.bus} address {dev.address}, "
          f"speed {dev.speed} (4: SuperSpeed), bcdUSB {dev.bcdUSB:04x}")
    for cfg in dev:
        for intf in cfg:
            for ep in intf:
                print(f"  EP 0x{ep.bEndpointAddress:02x}: {usb.util.endpoint_type(ep.bmAttributes)} "
                      f"wMaxPacketSize {ep.wMaxPacketSize}")

# Loopback -----------------------------------------------------------------------------------------

def loopback(dev, size, count, random_sizes=False, timeout=1000, seed=0):
    rng    = random.Random(seed)
    errors = 0
    nbytes = 0
    t0     = time.time()
    for i in range(count):
        n    = rng.randint(1, size) if random_sizes else size
        data = bytes(rng.randrange(256) for _ in range(n))
        dev.write(EP_OUT, data, timeout=timeout)
        back = bytes(dev.read(EP_IN, MAX_SIZE, timeout=timeout))
        if back != data:
            errors += 1
            if errors <= 4:
                first = next((j for j, (a, b) in enumerate(zip(back, data)) if a != b), min(len(back), len(data)))
                print(f"  packet {i}: {len(data)} bytes sent, {len(back)} received, first error at byte {first}")
        nbytes += n
    dt = time.time() - t0
    print(f"{count} packets, {nbytes} bytes, {errors} errors, {2*nbytes*8/dt/1e6:.1f} Mbps (OUT+IN, "
          f"one packet per transfer)")
    return errors

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--size",         default=1024, type=int, help="Packets size (bytes, max 1024).")
    parser.add_argument("--count",        default=100,  type=int, help="Number of packets.")
    parser.add_argument("--random-sizes", action="store_true",    help="Random packets sizes (up to --size).")
    parser.add_argument("--timeout",      default=1000, type=int, help="Transfers timeout (ms).")
    parser.add_argument("--seed",         default=0,    type=int, help="Random seed.")
    parser.add_argument("command",        choices=["info", "loopback"])
    args = parser.parse_args()
    assert 1 <= args.size <= MAX_SIZE

    dev = open_device()
    info(dev)
    if args.command == "loopback":
        sys.exit(1 if loopback(dev, args.size, args.count, args.random_sizes, args.timeout, args.seed) else 0)

if __name__ == "__main__":
    main()
