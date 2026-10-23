#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Host model <-> Device (USB3PHY + Daisho USB3 Core) Verilator simulation (sim.py): link training,
link initialization, enumeration control transfers and Bulk OUT/IN transfers (looped back by the
device), as with an xHCI host."""

import sys
import unittest
import subprocess

from pathlib import Path

from usb3_core.host_model import *

ROOT = Path(__file__).resolve().parents[1]

def device_descriptors():
    rom = (ROOT / "usb3_core" / "daisho_mod" / "usb3" / "usb3_descrip.bin").read_bytes()
    return list(rom[:18])

def tps(packets):
    """Transaction Packets: (subtype, nump, seq)."""
    return [(dw1 & 0xf, (dw1 >> 16) & 0x1f, (dw1 >> 21) & 0x1f)
        for kind, *p in packets if kind == "HP" for dw0, dw1, dw2, seq, crc in [p] if (dw0 & 0x1f) == 0x04]

def lmps(packets):
    """LMPs subtypes."""
    return [(dw0 >> 5) & 0xf for kind, *p in packets if kind == "HP" for dw0, dw1, dw2, seq, crc in [p]
        if (dw0 & 0x1f) == 0x00]

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
        return parse_sim_output(out, enumeration_script(skp="--no-host-skp" not in args, pm="--host-pm" in args))

    def check_enumeration(self, sections):
        # Packets integrity (CRCs).
        for name, packets in sections.items():
            for packet in packets:
                self.assertTrue(packet[-1], f"{name}: bad CRC {packet}")

        # Link initialization (7.2.4.1.1, 8.4.5): Advertisements, Port Capability LMP, Port
        # Configuration Response LMP.
        link_init = sections["link_init"]
        lcs = [p[1] for p in link_init if p[0] == "LC"]
        self.assertEqual(lcs[:5], [LGOOD(7)] + [LCRD(x) for x in range(4)])
        self.assertEqual(lmps(link_init), [LMP_SUB_PORT_CAP, LMP_SUB_PORT_CFG_RESP])

        # GET_DESCRIPTOR (Device): Setup ACK, data, Status ACK (8.12.2).
        packets = sections["get_device_descriptor"]
        self.assertEqual(tps(packets), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])
        self.assertEqual(dpps(packets), [device_descriptors()])

        # GET_DESCRIPTOR (Configuration, 9 bytes).
        packets = sections["get_configuration_descriptor"]
        self.assertEqual(dpps(packets)[0][:2], [9, 2])

        # GET_DESCRIPTOR (String 1): UTF-16LE.
        string = bytes(dpps(sections["get_string_descriptor"])[0])
        self.assertEqual(string[1], 3)
        self.assertEqual(string[2:string[0]].decode("utf-16-le"), "Great Scott Gadgets")

        # SET_CONFIGURATION: Setup ACK, Status ACK with the next expected sequence number.
        self.assertEqual(tps(sections["set_configuration"]), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])

        # Bulk OUT (EP2): ACK TPs (NumP=1, next sequence number), then Bulk IN (EP1): the packets are
        # looped back by the device (short and max size packets).
        for i, payload in enumerate(bulk_payloads()):
            self.assertEqual(tps(sections[f"bulk_out_{i}"]), [(TP_SUB_ACK, 1, i + 1)])
            self.assertEqual(dpps(sections[f"bulk_in_{i}"]), [payload])

    def test_enumeration(self):
        # SKP Ordered Sets between packets (2 symbols: the framing is shifted in the received words).
        self.check_enumeration(self.run_sim())

    def test_enumeration_no_skp(self):
        self.check_enumeration(self.run_sim("--no-host-skp"))

    def test_enumeration_fast_host(self):
        # Data stage requested right after the SETUP ACK (no host latency).
        self.check_enumeration(self.run_sim("--host-latency", "0"))

    @unittest.expectedFailure
    def test_enumeration_back_to_back_header_packets(self):
        # Data stage ACK TP immediately followed by the STATUS TP (no Logical Idle/SKP between the
        # Header Packets): the Daisho core does not answer the STATUS TP (the xHCI host observed on
        # hardware leaves ~100 words between them, 4 Idle words are enough).
        self.check_enumeration(self.run_sim("--host-status-gap", "0", "--no-host-skp"))

    def test_lgo_u1_u2_rejected(self):
        # LGO_U1/LGO_U2 rejected with LXU (7.2.4.2.3), the link stays in U0 and is still functional.
        sections = self.run_sim("--host-pm")
        self.check_enumeration(sections)
        for name in ["lgo_u1", "lgo_u2"]:
            lcs = [p[1] for p in sections[name] if p[0] == "LC"]
            self.assertIn(LXU(), lcs, name)
            self.assertNotIn(LAU(), lcs, name)
        packets = sections["get_device_descriptor_after_pm"]
        self.assertEqual(dpps(packets), [device_descriptors()])

if __name__ == "__main__":
    unittest.main()
