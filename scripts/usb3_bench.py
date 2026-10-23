#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Bulk bandwidth benchmark (acorn.py --example bench: EP1 IN counter source, EP2 OUT counter
checker), with asynchronous libusb transfers (python-libusb1) queued to keep the link busy.

Examples:
  ./scripts/usb3_bench.py in                     # Device -> Host, data checked.
  ./scripts/usb3_bench.py out                    # Host -> Device (errors: device CSRs, --csr-csv).
  ./scripts/usb3_bench.py both --duration 10
  ./scripts/usb3_bench.py in --transfer-size 4194304 --queue 16 --no-check
"""

import sys
import time
import argparse

import numpy as np
import usb1

USB3_VID = 0x1d50
USB3_PID = 0x605a
EP_IN    = 0x81
EP_OUT   = 0x02

# Benchmark ----------------------------------------------------------------------------------------

class Bench:
    def __init__(self, handle, direction, transfer_size, queue, check=True):
        self.handle        = handle
        self.direction     = direction
        self.transfer_size = transfer_size
        self.queue         = queue
        self.check         = check
        self.bytes         = {"in": 0, "out": 0}
        self.errors        = 0
        self.failures      = 0
        self.next_word     = None
        self.out_word      = 0
        self.transfers     = []
        self.running       = True

    def out_data(self):
        words = np.arange(self.out_word, self.out_word + self.transfer_size//4, dtype=np.uint32)
        self.out_word += self.transfer_size//4
        return words.tobytes()

    def check_in(self, data):
        words = np.frombuffer(data[:len(data) & ~3], dtype=np.uint32)
        if len(words) == 0:
            return
        if self.next_word is not None and words[0] != self.next_word:
            self.errors += 1
        diffs = np.diff(words.astype(np.int64)) & 0xffffffff
        self.errors   += int(np.count_nonzero(diffs != 1))
        self.next_word = (int(words[-1]) + 1) & 0xffffffff

    def callback(self, transfer):
        status = transfer.getStatus()
        if status == usb1.TRANSFER_COMPLETED:
            n = transfer.getActualLength()
            if transfer.getEndpoint() & 0x80:
                self.bytes["in"] += n
                if self.check:
                    self.check_in(transfer.getBuffer()[:n])
            else:
                self.bytes["out"] += n
        elif status != usb1.TRANSFER_CANCELLED:
            self.failures += 1
        if self.running and status != usb1.TRANSFER_CANCELLED:
            if not (transfer.getEndpoint() & 0x80):
                transfer.setBuffer(self.out_data())
            transfer.submit()

    def start(self):
        directions = ["in", "out"] if self.direction == "both" else [self.direction]
        for d in directions:
            for _ in range(self.queue):
                t = self.handle.getTransfer()
                if d == "in":
                    t.setBulk(EP_IN, self.transfer_size, callback=self.callback, timeout=5000)
                else:
                    t.setBulk(EP_OUT, self.out_data(), callback=self.callback, timeout=5000)
                t.submit()
                self.transfers.append(t)

    def stop(self, context):
        self.running = False
        for t in self.transfers:
            try:
                t.cancel()
            except usb1.USBError:
                pass
        while any(t.isSubmitted() for t in self.transfers):
            context.handleEventsTimeout(0.1)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("direction",       choices=["in", "out", "both"])
    parser.add_argument("--duration",      default=5.0,     type=float, help="Duration (s).")
    parser.add_argument("--transfer-size", default=1 << 20, type=int,   help="Transfer size (bytes, multiple of 1024).")
    parser.add_argument("--queue",         default=8,       type=int,   help="Queued transfers per direction.")
    parser.add_argument("--no-check",      action="store_true",         help="Disable the IN data check.")
    parser.add_argument("--csr-csv",       default=None,                help="CSR file: read the device OUT checker (litex_server).")
    parser.add_argument("--port",          default=1234,    type=int,   help="litex_server port (with --csr-csv).")
    args = parser.parse_args()

    with usb1.USBContext() as context:
        handle = context.openByVendorIDAndProductID(USB3_VID, USB3_PID)
        if handle is None:
            raise SystemExit(f"Device {USB3_VID:04x}:{USB3_PID:04x} not found.")
        handle.claimInterface(0)
        speed = handle.getDevice().getDeviceSpeed()
        print(f"Device {USB3_VID:04x}:{USB3_PID:04x}, speed {speed} (4: SuperSpeed).")

        bench = Bench(handle, args.direction, args.transfer_size, args.queue, check=not args.no_check)
        bench.start()
        t0   = time.time()
        last = {"in": 0, "out": 0}
        tl   = t0
        while time.time() - t0 < args.duration:
            context.handleEventsTimeout(0.1)
            if time.time() - tl >= 1.0:
                now = time.time()
                rates = {d: (bench.bytes[d] - last[d])/(now - tl)/1e6 for d in ["in", "out"]}
                print(f"  IN {rates['in']:7.1f} MB/s  OUT {rates['out']:7.1f} MB/s  errors {bench.errors}  failures {bench.failures}")
                last = dict(bench.bytes)
                tl   = now
        bench.stop(context)
        dt = time.time() - t0
        for d in ["in", "out"]:
            if bench.bytes[d]:
                print(f"{d.upper():3s}: {bench.bytes[d]/1e6:9.1f} MB in {dt:.1f}s: {bench.bytes[d]/dt/1e6:7.1f} MB/s")
        print(f"IN data errors: {bench.errors}, transfer failures: {bench.failures}")
        handle.releaseInterface(0)

    if args.csr_csv:
        from litex import RemoteClient
        bus = RemoteClient(csr_csv=args.csr_csv, port=args.port)
        bus.open()
        print(f"Device OUT checker: {bus.regs.usb3_bench_sink_words.read()*4/1e6:.1f} MB received, "
              f"{bus.regs.usb3_bench_sink_errors.read()} errors.")
        bus.close()

    sys.exit(1 if (bench.errors or bench.failures) else 0)

if __name__ == "__main__":
    main()
