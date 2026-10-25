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

import pytest

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors

from test.common       import get_output_dir
from test.model.device import check_core_enumeration
from test.model.host   import dpps

pytestmark = pytest.mark.sim

ROOT = Path(__file__).resolve().parents[1]

class TestSimHost(unittest.TestCase):
    def run_sim(self, *args):
        cmd = [sys.executable, str(ROOT / "sim.py"), "--output-dir", str(get_output_dir(self))] + list(args)
        p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            timeout=600)
        out = p.stdout or ""
        self.assertEqual(p.returncode, 0, out[-4000:])
        self.assertIn("HOST script done", out, out[-4000:])
        self.output = out
        return parse_sim_output(out, enumeration_script(
            skp        = "--no-host-skp" not in args,
            status_gap = int(args[args.index("--host-status-gap") + 1]) if "--host-status-gap" in args else 64,
            pm         = "--host-pm" in args,
            lpm        = "--host-lpm" in args))

    def check_enumeration(self, sections):
        check_core_enumeration(self, sections)

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

    def test_lpm_u1_u2(self):
        # Device Core with LPM: SET_FEATURE(U1_ENABLE/U2_ENABLE) reported by GET_STATUS, LGO_U1/LGO_U2
        # accepted (LAU), U1/U2 entered by both LTSSMs, Host initiated exit (LFPS handshake, Recovery),
        # Advertisements on U0 re-entry, then control transfers.
        sections = self.run_sim("--host-lpm")
        self.check_enumeration(sections)
        self.assertEqual(dpps(sections["get_status"]), [[0x0c, 0x00]])
        for n in [1, 2]:
            lcs = [p[1] for p in sections[f"u{n}"] if p[0] == "LC"]
            self.assertIn(LAU(), lcs)
            self.assertNotIn(LXU(), lcs)
            for name in ["HOST", "DEV "]:
                self.assertIn(f"{name} entering U{n} state", self.output)
            lcs = [p[1] for p in sections[f"u{n}_u0"] if p[0] == "LC"]
            self.assertEqual(lcs[1:5], [LCRD(x) for x in range(4)], lcs)
            self.assertEqual(lcs[0] & ~0x7, LGOOD(0))
            packets = sections[f"get_device_descriptor_after_u{n}"]
            self.assertEqual(dpps(packets), [bulk_descriptors().device()])

if __name__ == "__main__":
    unittest.main()
