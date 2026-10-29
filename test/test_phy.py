#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Host <-> Device Verilator simulations (sim.py): Gen 1x1 link training to U0 and Gen 1x2 (sim.py
--lanes 2): SuperSpeedPlus Polling (SCD1/SCD2, LBPM PHY Capability/Ready: Gen 1x2 negotiated), link
training to U0 on both lanes, 64-bit PRBS packets checked on both sides with lane-to-lane skew, per
lane polarity inversion and per lane SKP Ordered Sets differences."""

import re
import sys
import unittest
import subprocess

from pathlib import Path

import pytest

from test.common import get_output_dir

pytestmark = pytest.mark.sim

ROOT = Path(__file__).resolve().parents[1]

class TestPHY(unittest.TestCase):
    def test_phy_reaches_idle(self):
        # Repo root = ../
        root = Path(__file__).resolve().parents[1]

        # Sim scrpt.
        sim_script = root / "sim.py"
        self.assertTrue(sim_script.exists(), f"Missing simulation script: {sim_script}")

        cmd = [sys.executable, str(sim_script), "--output-dir", str(get_output_dir(self))]
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
        host_idle = re.search(r"HOST entering U0 state",  out)
        dev_idle  = re.search(r"DEV\s+entering U0 state", out)

        if not (host_idle and dev_idle):
            self.fail(
                "Did not observe both Host and Dev entering U0.\n\n"
                "---- tail ----\n" + out[-4000:]
            )

    def test_phy_gen2_capable(self):
        # Gen 2x1 capable Host/Device (sim.py --gen2-capable): SuperSpeedPlus Polling with Gen 2
        # negotiated on both sides (training then continues at Gen 1 speed: no Gen 2 speed switch).
        cmd = [sys.executable, str(ROOT / "sim.py"), "--gen2-capable",
            "--output-dir", str(get_output_dir(self))]
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=600)
        except subprocess.TimeoutExpired:
            self.fail("Simulation timeout.")
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        for side in ["HOST", r"DEV\s"]:
            self.assertRegex(out, side + r"\s*SSP entering DONE state \(x2: 0, gen2: 1\)", tail)
            self.assertRegex(out, side + r"\s*entering U0 state", tail)
        self.assertIn("HOST script done", out, tail)

    def test_phy_gen2(self):
        # Gen 2x1 Host/Device (sim.py --gen2): Gen 2 negotiated in the SuperSpeedPlus Polling, then
        # Gen 2 training (TSEQ/TS1/TS2 128b/132b blocks, SDS/Logical Idle) to U0 on raw words, then
        # the enumeration with the Gen 2 packet framing in the Data Blocks.
        cmd = [sys.executable, str(ROOT / "sim.py"), "--gen2",
            "--output-dir", str(get_output_dir(self))]
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=900)
        except subprocess.TimeoutExpired:
            self.fail("Simulation timeout.")
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        for side in ["HOST", r"DEV\s"]:
            self.assertRegex(out, side + r"\s*SSP entering DONE state \(x2: 0, gen2: 1\)", tail)
            for state in ["Polling.RxEQ", "Polling.Active", "Polling.Configuration", "Polling.Idle", "U0"]:
                self.assertRegex(out, side + r"\s*entering " + state.replace(".", r"\.") + " state", tail)
        self.assertIn("HOST script done", out, tail)

    def run_link_words2(self, args):
        # Device core on PHY words pairs (64-bit Link Layer/Bulk endpoints, sim.py --link-words 2):
        # enumeration and Bulk loopback with the host script, no Recovery.
        cmd = [sys.executable, str(ROOT / "sim.py"), "--link-words", "2", *args,
            "--output-dir", str(get_output_dir(self))]
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=900)
        except subprocess.TimeoutExpired:
            self.fail("Simulation timeout.")
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        self.assertIn("HOST script done", out, tail)
        self.assertNotRegex(out, r"entering Recovery\.Active state", tail)

    def test_phy_link_words2(self):
        # Gen 1: words pairs <-> Gen 1 PIPE words adapters.
        self.run_link_words2([])

    def test_phy_gen2_link_words2(self):
        # Gen 2: words pairs to/from the Gen 2 framing (DPP data 8 bytes per cycle).
        self.run_link_words2(["--gen2"])

    def test_phy_dev_pipe_cd(self):
        # Gen 1: Device LTSSM/Link Layer/Core at half the PIPE frequency (USB3PHY pipe_cd, words
        # pairs: Logical Idle filtering, RX words packing, TX packets store-and-forward).
        self.run_link_words2(["--dev-pipe-cd"])

    def test_phy_dev_pipe_cd_no_u1u2(self):
        # Device without U1/U2 (USB3PHY with_u1u2=False) and LTSSM/Link Layer/Core at half the PIPE
        # frequency (FPGAs with slower fabrics): Host LGO_U1/LGO_U2 rejected (LXU), script done.
        self.run_link_words2(["--dev-pipe-cd", "--dev-no-u1u2", "--host-pm"])

    def test_phy_gen2_lpm(self):
        # Gen 2 U1/U2 (sim.py --gen2 --host-lpm): Host model LGO_U1/LGO_U2 after the enumeration,
        # U1/U2 exits (LFPS) and Recovery at Gen 2 back to U0, script done.
        cmd = [sys.executable, str(ROOT / "sim.py"), "--gen2", "--host-lpm", "--link-words", "2",
            "--output-dir", str(get_output_dir(self))]
        try:
            p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=900)
        except subprocess.TimeoutExpired:
            self.fail("Simulation timeout.")
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        for state in ["U1", "U2", r"Recovery\.Active"]:
            self.assertRegex(out, r"DEV\s*entering " + state + " state", tail)
        self.assertIn("HOST script done", out, tail)

# Gen 1x2 (sim.py --lanes 2) -----------------------------------------------------------------------

CONFIGS_SLOW = {
    "lane0_late"     : ["--lane0-delay", "1", "--lane1-delay", "0", "--lane1-word-shift", "0"],
    "lane1_late"     : ["--lane1-delay", "1", "--lane1-word-shift", "0"],
    "no_skew"        : ["--lane1-delay", "0", "--lane1-word-shift", "0"],
    "ppm_m300"       : ["--ppm", "-300"],
    "ppm_p300"       : ["--ppm", "300"],
}

class TestPHYX2(unittest.TestCase):
    def run_sim(self, name, args):
        cmd = [sys.executable, str(ROOT / "sim.py"), "--lanes", "2",
            "--output-dir", str(get_output_dir(self, name))] + args
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
            self.assertRegex(out, side + r"\s*SSP entering DONE state \(x2: 1, gen2: 0\)", tail)
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
