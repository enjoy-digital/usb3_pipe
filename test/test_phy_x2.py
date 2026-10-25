#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 1x2 Host <-> Device Verilator simulations (sim_x2.py): SuperSpeedPlus Polling (SCD1/SCD2, LBPM
PHY Capability/Ready: Gen 1x2 negotiated), link training to U0 on both lanes, 64-bit PRBS packets
checked on both sides with lane-to-lane skew, per lane polarity inversion and per lane SKP Ordered
Sets differences."""

import re
import sys
import unittest
import subprocess

from pathlib import Path

import pytest

from test.common import get_output_dir

pytestmark = pytest.mark.sim

ROOT = Path(__file__).resolve().parents[1]

CONFIGS_SLOW = {
    "lane0_late"     : ["--lane0-delay", "1", "--lane1-delay", "0", "--lane1-word-shift", "0"],
    "lane1_late"     : ["--lane1-delay", "1", "--lane1-word-shift", "0"],
    "no_skew"        : ["--lane1-delay", "0", "--lane1-word-shift", "0"],
    "ppm_m300"       : ["--ppm", "-300"],
    "ppm_p300"       : ["--ppm", "300"],
}

class TestPHYX2(unittest.TestCase):
    def run_sim(self, name, args):
        cmd = [sys.executable, str(ROOT / "sim_x2.py"), "--output-dir", str(get_output_dir(self, name))] + args
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=1200)
        except subprocess.TimeoutExpired:
            self.fail(f"{name}: Simulation timeout.")
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, f"{name}: Simulation failed." + tail)
        self.assertNotIn("Simulation timeout", out, f"{name}: No PRBS completion." + tail)
        for side in ["HOST", r"DEV\s"]:
            # SuperSpeedPlus Polling done with Gen 1x2 negotiated.
            self.assertRegex(out, side + r"\s*SSP entering DONE state \(x2: 1\)", tail)
            # Single link training to U0, lanes deskewed.
            u0 = re.findall(side + r"\s*entering U0 state", out)
            self.assertEqual(len(u0), 1, f"{name}: {side} entered U0 {len(u0)} times." + tail)
            self.assertNotRegex(out, side + r"\s*entering (Recovery|eSS|Polling.ExitTo|Compliance)", tail)
            self.assertRegex(out, side + r"\s*lanes deskewed", tail)
        for side in ["HOST", "DEV"]:
            m = re.search(side + r" PRBS:\s+(\d+) words,\s+(\d+) packets,\s+(\d+) errors, x2: (\d)", out)
            self.assertIsNotNone(m, f"{name}: No {side} PRBS report." + tail)
            self.assertGreater(int(m.group(2)), 0)
            self.assertEqual(int(m.group(3)), 0, f"{name}: {side} PRBS errors." + tail)
            self.assertEqual(int(m.group(4)), 1)

    def test_phy_x2(self):
        # Device RX Lane 1 late by 2 Symbols (1 word delay, 2 Symbols word shift), Host Lane 1 TX
        # inverted, extra SKP Ordered Sets alternately on Lane 1/Lane 0.
        self.run_sim("nominal", [])

    @pytest.mark.slow
    def test_phy_x2_configs(self):
        for name, args in CONFIGS_SLOW.items():
            with self.subTest(config=name):
                self.run_sim(name, args)

if __name__ == "__main__":
    unittest.main()
