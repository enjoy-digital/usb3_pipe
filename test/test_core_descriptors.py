#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import re
import shutil
import tempfile
import unittest
import subprocess

CORE_PATH = os.path.join(os.path.dirname(__file__), "..", "usb3_core")

def read_offsets(filename):
    offsets = {}
    with open(filename) as f:
        for line in f:
            m = re.match(r"parameter\s+\[\d+:0\]\s+(\w+)\s*=\s*'d(\d+);", line)
            if m:
                offsets[m.group(1)] = int(m.group(2))
    return offsets

# Daisho Descriptors -------------------------------------------------------------------------------

@unittest.skipIf(shutil.which("gcc") is None, "gcc not available")
class TestDaishoDescriptors(unittest.TestCase):
    def generate(self, core):
        # Run the descriptors generator on a copy of the core and return the generated offsets.
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(CORE_PATH, core, "usb_descrip_gen.c")
            shutil.copy(src, tmp)
            os.makedirs(os.path.join(tmp, "usb2"))
            os.makedirs(os.path.join(tmp, "usb3"))
            subprocess.run(["gcc", "usb_descrip_gen.c", "-lm", "-o", "usb_descrip_gen"], cwd=tmp, check=True)
            subprocess.run(["./usb_descrip_gen"], cwd=tmp, check=True, stdout=subprocess.DEVNULL)
            with open(os.path.join(tmp, "usb3", "usb3_descrip.bin"), "rb") as f:
                self.rom = f.read()
            return read_offsets(os.path.join(tmp, "usb_descrip.vh"))

    def check_offsets(self, offsets):
        # Descriptors are stored one after the other in the ROM (USB3: 32-bit words): Configuration
        # after Device, BOS after Configuration, Strings after BOS (a Descriptor offset of 0 makes
        # the core return the Device Descriptor for every GET_DESCRIPTOR request).
        self.assertEqual(offsets["DESCR_USB3_DEVICE"], 0)
        self.assertEqual(offsets["DESCR_USB3_CONFIG"], 18//4 + 1)
        self.assertEqual(offsets["DESCR_USB3_BOS"], offsets["DESCR_USB3_CONFIG"] + (offsets["DESCR_USB3_CONFIG_LEN"] + 3)//4)
        self.assertGreater(offsets["DESCR_USB3_STRING0"], offsets["DESCR_USB3_BOS"])

    def check_strings(self, offsets):
        # String Descriptors are UTF-16LE (USB 3.2 9.6.9).
        rom    = self.rom
        offset = 4*offsets["DESCR_USB3_STRING1"]
        length = rom[offset]
        self.assertEqual(rom[offset + 1], 0x03)
        string = rom[offset + 2:offset + length].decode("utf-16-le")
        self.assertEqual(string, "Great Scott Gadgets")

    def test_daisho_mod_generator_offsets(self):
        offsets = self.generate("daisho_mod")
        self.check_offsets(offsets)
        self.check_strings(offsets)
        # Generated ROM matches the one used for synthesis.
        with open(os.path.join(CORE_PATH, "daisho_mod", "usb3", "usb3_descrip.bin"), "rb") as f:
            self.assertEqual(f.read(), self.rom)
        # Generated offsets match the ones used for synthesis.
        for vh in ["usb_descrip.vh", "usb3/usb_descrip.vh"]:
            self.assertEqual(read_offsets(os.path.join(CORE_PATH, "daisho_mod", vh)), offsets)

    def test_daisho_generator_offsets(self):
        offsets = self.generate("daisho")
        self.check_offsets(offsets)
        self.check_strings(offsets)
        self.assertEqual(read_offsets(os.path.join(CORE_PATH, "daisho", "usb3", "usb_descrip.vh")), offsets)

if __name__ == "__main__":
    unittest.main()
