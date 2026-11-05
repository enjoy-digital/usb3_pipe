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

# Simulation output checks -------------------------------------------------------------------------

def _host_rx_words(out):
    """HOST_RX words of the simulation output, (data, ctrl, data in transmission byte order)."""
    words = []
    for m in re.finditer(r"HOST_RX ([0-9a-f]{8}) ([0-9a-f])", out):
        v = int(m.group(1), 16)
        words.append((v, int(m.group(2), 16), int.from_bytes(v.to_bytes(4, "big"), "little")))
    return words

def device_packets_crc_errors(out):
    """Device Header Packets (CRC-16/CRC-5) and Link Commands (CRC-5) received by the host model with
    errors: (packets checked, errors)."""
    from usb3_core.host_model import crc16, crc5
    words, n, errors, i = _host_rx_words(out), 0, 0, 0
    while i < len(words):
        data, ctrl, _ = words[i]
        if (data, ctrl) == (0xfbfbfbf7, 0xf) and i + 4 < len(words):
            hdr = b"".join(w[2].to_bytes(4, "little") for w in words[i+1:i+4])
            crc, lcw = words[i+4][2] & 0xffff, words[i+4][2] >> 16
            n += 1
            errors += not ((crc16(list(hdr)) == crc) and (crc5(lcw & 0x7ff) == (lcw >> 11)))
            i += 5
        elif (data, ctrl) == (0xfefefef7, 0xf) and i + 1 < len(words):
            v = words[i+1][2]
            n += 1
            errors += not (((v & 0xffff) == (v >> 16)) and (crc5(v & 0x7ff) == ((v >> 11) & 0x1f)))
            i += 2
        else:
            i += 1
    return n, errors

def host_lgood_times(out):
    """Times (cycles) of the LGOOD_n received by the host model (HOST_LCRX logs)."""
    times, pending = [], None
    for line in out.splitlines():
        m = re.match(r"HOST_LCRX\s+(\d+)", line)
        if m:
            pending = int(m.group(1))
        m = re.match(r"HOST_RX ([0-9a-f]{8}) 0$", line)
        if m and pending is not None:
            v = int.from_bytes(int(m.group(1), 16).to_bytes(4, "big"), "little") & 0x7ff
            if (v & 0x7f0) == 0:
                times.append(pending)
            pending = None
    return times

def host_lgood_latencies(out):
    """Host Header Packet -> device LGOOD_n latencies (cycles) from the host model timing logs (after
    the last host U0 entry, the first LGOOD_n being the Advertisement)."""
    hp, lgood, pending, adv = [], [], None, True
    for line in out.splitlines():
        if "HOST entering U0" in line:
            hp, lgood, pending, adv = [], [], None, True
        m = re.match(r"HOST_HPTX\s+(\d+)", line)
        if m:
            hp.append(int(m.group(1)))
        m = re.match(r"HOST_LCRX\s+(\d+)", line)
        if m:
            pending = int(m.group(1))
        m = re.match(r"HOST_RX ([0-9a-f]{8}) 0$", line)
        if m and pending is not None:
            v = int.from_bytes(int(m.group(1), 16).to_bytes(4, "big"), "little") & 0x7ff
            if (v & 0x7f0) == 0:
                if adv:
                    adv = False
                else:
                    lgood.append(pending)
            pending = None
    return [l - h for h, l in zip(hp, lgood)]


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

    def run_link_words2(self, args, recovery=False):
        # Device core on PHY words pairs (64-bit Link Layer/Bulk endpoints, sim.py --link-words 2):
        # enumeration and Bulk loopback with the host script, no Recovery (unless recovery).
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
        if not recovery:
            self.assertNotRegex(out, r"entering Recovery\.Active state", tail)
        return out, tail

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

    def test_phy_gen2_dev_pipe_cd(self):
        # Gen 2 with the Device LTSSM/Link Layer/Core at half the PIPE frequency (USB3PHY pipe_cd):
        # SuperSpeedPlus Polling/Gen 2 training through the LTSSM <-> PIPEs resynchronization, Gen 2
        # words pairs and LGOOD_n/LCRD_x fast path through the AsyncFIFOs, RX credits throttling.
        self.run_link_words2(["--gen2", "--dev-pipe-cd"])

    def test_phy_gen2_80_dev_pipe_cd(self):
        # Gen 2 on 80-bit raw words (ex: GW5A GTR12 1:4 gear at 125MHz) with the Device Link Layer at
        # half the PIPE frequency: framing/packing at 32 bytes per cycle at the Link Layer frequency
        # (blocks pairs/blocks buffers between the PIPE and the Link Layer clock domains).
        self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80"])

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_ssc_itp(self):
        # Same with the Host SSC and ITPs (SKP Ordered Sets between the packets, long SKP OS).
        self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80", "--ssc",
            "--host-itp"])

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_recovery(self):
        # Same with the Host directed to Recovery from U0 after the script: Recovery at Gen 2 on
        # 80-bit raw words (Data Blocks -> TS1/TS2 -> SDS/Logical Idle), back to U0 on both sides.
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--host-recovery"], recovery=True)
        for side in ["HOST", r"DEV\s"]:
            self.assertRegex(out, side + r"\s*entering Recovery\.Idle state", tail)
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_burst_out(self, extra=[]):
        # Gen 2 80-bit with Bulk OUT bursts of max size DPs back to back (as a host with queued
        # transfers) and a reactive host model (LGOOD_n/LCRD_x for each device Header Packet as soon
        # as received): no Recovery, device Header Packets/Link Commands CRCs, LGOOD_n latency of the
        # Link Layer slower than the line rate bounded (DPPs buffered before the next DPH).
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--host-burst-out", "4", "--host-burst-len", "1024", *extra])
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)
        n, errors = device_packets_crc_errors(out)
        self.assertGreater(n, 50)
        self.assertEqual(errors, 0)
        latencies = host_lgood_latencies(out)
        self.assertGreaterEqual(len(latencies), 16)
        self.assertLess(max(latencies), 1333) # 10us at the 133MHz host model clock.

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_lbad(self, extra=[]):
        # Gen 2 80-bit Bulk OUT burst with a DPH CRC error: device LBAD, Header Packets (and their DPPs)
        # ignored until the host LRTY, retransmission accepted: no Recovery, CRCs, and the
        # retransmitted DPHs acknowledged without waiting for the ignored DPPs (skipped at the DPP
        # data rate).
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--host-lbad-test", "--host-burst-len", "1024", "--dev-loopback-depth", "4096", *extra])
        self.assertIn("HOST_LBAD_RX", out, tail)
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)
        n, errors = device_packets_crc_errors(out)
        self.assertEqual(errors, 0)
        # LGOOD_n latency of the retransmitted DPH (first LGOOD_n after the last host LRTY ones).
        section = out[out.index("HOST_LBAD_RX"):]
        hptx  = [int(t) for t in re.findall(r"HOST_HPTX\s+(\d+)", section)]
        self.assertGreaterEqual(len(hptx), 3)
        retx   = hptx[-3] # First retransmitted DPH.
        lgoods = [t for t in host_lgood_times(section) if t > retx]
        self.assertTrue(len(lgoods) > 0)
        self.assertLess(lgoods[0] - retx, 800) # ~450 cycles for a normal DPH (6us at 133MHz).

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_quads(self):
        # Gen 2 80-bit with 128-bit Link Layer words (USB3PHY link_words=4: RX words quads from the
        # 32 bytes framing, DPP data received 16 bytes per cycle, 128-bit Bulk endpoints).
        self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--link-words", "4"])

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_quads_burst_out(self):
        # Bulk OUT bursts (see test_phy_gen2_80_dev_pipe_cd_burst_out) on 128-bit Link Layer words.
        self.test_phy_gen2_80_dev_pipe_cd_burst_out(["--link-words", "4"])

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_quads_lbad(self):
        # DPH CRC error/LBAD/LRTY (see test_phy_gen2_80_dev_pipe_cd_lbad) on 128-bit Link Layer words.
        self.test_phy_gen2_80_dev_pipe_cd_lbad(["--link-words", "4"])

    @pytest.mark.slow
    def test_phy_gen1_duplex(self):
        # Gen 1 (32-bit Link Layer words) full duplex: Bulk OUT DPs streamed (Rx Header Buffer
        # credits) while the device sends Bulk IN DPs: no Recovery, CRCs.
        cmd = [sys.executable, str(ROOT / "sim.py"), "--host-burst-duplex", "4", "--host-burst-len",
            "1024", "--dev-loopback-depth", "4096", "--output-dir", str(get_output_dir(self))]
        p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=900)
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        self.assertIn("HOST script done", out, tail)
        self.assertNotRegex(out, r"entering Recovery\.Active state", tail)
        section = out[out.index("HOST_MARK          9"):]
        n, errors = device_packets_crc_errors(section)
        self.assertGreater(n, 80)
        self.assertEqual(errors, 0)

    @pytest.mark.slow
    def test_phy_gen1_duplex_skp(self):
        # Gen 1 full duplex at full rate: ACK TP behind each OUT DP, SKP Ordered Sets at the packets
        # boundaries: the framing shifted by 2 symbols right behind a packet or a Link Command (the
        # framing aligner lost the end of the word before the shift: Recovery storm on hardware).
        cmd = [sys.executable, str(ROOT / "sim.py"), "--host-burst-duplex", "64", "--host-burst-duplex-skp",
            "--host-burst-len", "1024", "--dev-loopback-depth", "4096", "--output-dir", str(get_output_dir(self))]
        p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=900)
        out  = p.stdout or ""
        tail = "\n---- tail ----\n" + out[-4000:]
        self.assertEqual(p.returncode, 0, "Simulation failed." + tail)
        self.assertIn("HOST script done", out, tail)
        self.assertNotRegex(out, r"entering Recovery\.Active state", tail)

    def test_phy_gen2_80_dev_pipe_cd_quads_duplex(self):
        # Full duplex on 128-bit Link Layer words: Bulk OUT DPs streamed while the device sends Bulk
        # IN DPs (looped back data): no Recovery, CRCs, host Header Packets acknowledged (LGOOD_n)
        # within the host PENDING_HP_TIMER while the device transmits DPPs.
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--link-words", "4", "--host-burst-duplex", "4", "--host-burst-len", "1024",
            "--dev-loopback-depth", "4096"])
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)
        section = out[out.index("HOST_MARK          9"):]
        n, errors = device_packets_crc_errors(section)
        self.assertGreater(n, 80)
        self.assertEqual(errors, 0)
        latencies = host_lgood_latencies(section)
        self.assertGreaterEqual(len(latencies), 20)
        self.assertLess(max(latencies), 1333) # 10us at the 133MHz host model clock.

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_quads_recovery_mid(self):
        # Host directed Recovery between Bulk OUT DPs on 128-bit Link Layer words, Advertisements on
        # the U0 re-entry (host model: as soon as its LTSSM is in U0), then Bulk OUT/IN: no device
        # Recovery request (ex: PENDING_HP_TIMER without the host Advertisement), CRCs.
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--link-words", "4", "--host-recovery-mid", "--host-burst-len", "1024",
            "--dev-loopback-depth", "4096"], recovery=True)
        self.assertIn("HOST_ADVERTISEMENT", out, tail)
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)

    @pytest.mark.slow
    def test_phy_gen2_80_dev_pipe_cd_quads_recovery_cut(self):
        # Host directed Recovery cutting a Bulk OUT DP (DPH acknowledged, DPP incomplete): the DPP
        # completed as missing at the U0 exit (its DPH processed, Rx Header Buffer freed), the DP
        # requested again (ACK TP retry) and sent again by the host, then Bulk OUT/IN: no device
        # Recovery request, CRCs.
        out, tail = self.run_link_words2(["--gen2", "--dev-pipe-cd", "--gen2-data-width", "80",
            "--link-words", "4", "--host-recovery-mid", "--host-recovery-cut", "--host-burst-len", "1024",
            "--dev-loopback-depth", "4096"], recovery=True)
        self.assertIn("HOST_ADVERTISEMENT", out, tail)
        self.assertNotRegex(out, r"Link Layer Recovery request", tail)
        n, errors = device_packets_crc_errors(out)
        self.assertEqual(errors, 0)

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
