#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Long Host <-> Device Verilator simulations (sim.py --no-core), run with: pytest -m slow.

Each configuration trains the link to U0 (spec timers, timer_scale=1) and then checks PRBS packets
on both sides with the PIPEs' datapaths (scrambling, SKP insertion/removal, CDC, word alignment) in
the loop.
"""

import re
import sys
import unittest
import subprocess

from pathlib import Path

import pytest

from test.common import get_output_dir

pytestmark = pytest.mark.sim

# Configurations -----------------------------------------------------------------------------------

PRBS_WORDS = 2**18 # 1M symbols per direction.

CONFIGS = {
    "nominal"      : [],
    "ppm_p300"     : ["--ppm",  "300"],
    "ppm_m300"     : ["--ppm", "-300"],
    "ppm_p600"     : ["--ppm",  "600"],
    "ppm_m600"     : ["--ppm", "-600"],
    "ssc"          : ["--ssc"],
    "ssc_ppm_p600" : ["--ssc", "--ppm", "600"],
    "word_shift_1" : ["--rx-word-shift", "1"],
    "word_shift_2" : ["--rx-word-shift", "2"],
    "word_shift_1_ppm_m300" : ["--rx-word-shift", "1", "--ppm", "-300"],
}

# Test ---------------------------------------------------------------------------------------------

@pytest.mark.slow
class TestPHYLong(unittest.TestCase):
    def run_sim(self, name, args):
        root = Path(__file__).resolve().parents[1]
        cmd  = [sys.executable, str(root / "sim.py"), "--no-core",
            "--prbs-words", str(PRBS_WORDS),
            "--output-dir", str(get_output_dir(self, name))] + args
        try:
            p = subprocess.run(cmd,
                cwd     = str(root),
                stdout  = subprocess.PIPE,
                stderr  = subprocess.STDOUT,
                text    = True,
                timeout = 1800,
                check   = False,
            )
        except subprocess.TimeoutExpired:
            self.fail(f"{name}: Simulation timeout.")
        return p.returncode, p.stdout or ""

    def test_phy_long(self):
        for name, args in CONFIGS.items():
            with self.subTest(config=name):
                returncode, out = self.run_sim(name, args)
                tail = "\n---- tail ----\n" + out[-4000:]
                self.assertEqual(returncode, 0, f"{name}: Simulation failed." + tail)
                self.assertNotIn("Simulation timeout", out, f"{name}: No PRBS completion." + tail)

                # Single link training to U0 (no Recovery/Rx.Detect loops).
                for side in ["HOST", r"DEV\s"]:
                    u0 = re.findall(side + r"\s*entering U0 state", out)
                    self.assertEqual(len(u0), 1, f"{name}: {side} entered U0 {len(u0)} times." + tail)
                    self.assertNotRegex(out, side + r"\s*entering (Recovery|eSS|Polling.ExitTo)",
                        f"{name}: Unexpected {side} state." + tail)

                # PRBS payload checked on both sides without errors.
                for side in ["HOST", "DEV"]:
                    m = re.search(side + r" PRBS:\s+(\d+) words,\s+(\d+) packets,\s+(\d+) errors", out)
                    self.assertIsNotNone(m, f"{name}: No {side} PRBS report." + tail)
                    words, packets, errors = map(int, m.groups())
                    self.assertGreaterEqual(words, PRBS_WORDS)
                    self.assertEqual(errors, 0, f"{name}: {side} PRBS errors." + tail)
