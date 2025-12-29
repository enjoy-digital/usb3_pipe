#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#

import re
import sys
import unittest
import subprocess

from pathlib import Path

class TestPHY(unittest.TestCase):
    def test_phy_reaches_idle(self):
        # Repo root = ../
        root = Path(__file__).resolve().parents[1]

        # Sim scrpt.
        sim_script = root / "sim.py"
        self.assertTrue(sim_script.exists(), f"Missing simulation script: {sim_script}")

        cmd = [sys.executable, str(sim_script)]
        try:
            p = subprocess.run(
                cmd,
                cwd     = str(root),
                stdout  = subprocess.PIPE,
                stderr  = subprocess.STDOUT,
                text    = True,
                timeout = 100,   # seconds
                check   = False,
            )
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or "") if isinstance(e.stdout, str) else ""
            self.fail("Simulation timeout (100s) before reaching Idle.\n\n" + out[-4000:])

        out = p.stdout or ""

        # If the sim failed, show the tail to help debug quickly.
        if p.returncode != 0:
            self.fail(f"Simulation exited with code {p.returncode}.\n\n" + out[-4000:])

        # Look for both host/dev reaching Polling.Idle.
        host_idle = re.search(r"HOST entering Polling\.Idle state",  out)
        dev_idle  = re.search(r"DEV\s+entering Polling\.Idle state", out)

        if not (host_idle and dev_idle):
            self.fail(
                "Did not observe both Host and Dev entering Polling.Idle.\n\n"
                "---- tail ----\n" + out[-4000:]
            )
