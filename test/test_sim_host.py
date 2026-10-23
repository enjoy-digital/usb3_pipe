#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Host model <-> Device (USB3PHY + USB3 Device Core) Verilator simulation (sim.py): link training,
link initialization, enumeration control transfers and Bulk transfers, as with an xHCI host."""

import sys
import unittest
import subprocess

from pathlib import Path

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors

from test.test_native_core import check_native_enumeration

ROOT = Path(__file__).resolve().parents[1]

def dpps(packets):
    return [p[0] for kind, *p in packets if kind == "DPP"]

class TestSimHost(unittest.TestCase):
    def run_sim(self, *args):
        cmd = [sys.executable, str(ROOT / "sim.py"), "--output-dir", "build/sim_host"] + list(args)
        p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=600)
        out = p.stdout or ""
        self.assertEqual(p.returncode, 0, out[-4000:])
        self.assertIn("HOST script done", out, out[-4000:])
        return parse_sim_output(out, enumeration_script(
            skp        = "--no-host-skp" not in args,
            status_gap = int(args[args.index("--host-status-gap") + 1]) if "--host-status-gap" in args else 64,
            pm         = "--host-pm" in args))

    def check_enumeration(self, sections):
        check_native_enumeration(self, sections)

    def test_enumeration(self):
        # SKP Ordered Sets between packets (2 symbols: the framing is shifted in the received words).
        self.check_enumeration(self.run_sim())

    def test_enumeration_no_skp(self):
        self.check_enumeration(self.run_sim("--no-host-skp"))

    def test_enumeration_fast_host(self):
        # Data stage requested right after the SETUP ACK (no host latency).
        self.check_enumeration(self.run_sim("--host-latency", "0"))

    def test_enumeration_back_to_back_header_packets(self):
        # Data stage ACK TP immediately followed by the STATUS TP (no Logical Idle/SKP between the
        # Header Packets).
        self.check_enumeration(self.run_sim("--host-status-gap", "0", "--no-host-skp"))

    def test_lgo_u1_u2_rejected(self):
        # LGO_U1/LGO_U2 rejected with LXU (U1/U2 not enabled), the link stays in U0.
        sections = self.run_sim("--host-pm")
        self.check_enumeration(sections)
        for name in ["lgo_u1", "lgo_u2"]:
            lcs = [p[1] for p in sections[name] if p[0] == "LC"]
            self.assertIn(LXU(), lcs, name)
            self.assertNotIn(LAU(), lcs, name)
        packets = sections["get_device_descriptor_after_pm"]
        self.assertEqual(dpps(packets), [bulk_descriptors().device()])

if __name__ == "__main__":
    unittest.main()
