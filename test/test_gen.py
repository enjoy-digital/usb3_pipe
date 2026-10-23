#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import re
import sys
import glob
import tempfile
import unittest
import subprocess

import yaml

from usb3_pipe.gen import ConfigError, get_config, get_ports, get_ltssm_states, get_ltssm_state_localparam

root_dir     = os.path.join(os.path.dirname(__file__), "..")
examples_dir = os.path.join(root_dir, "examples")

def get_verilog_ports(filename, module):
    """Return {name: (direction, width)} of a Verilog module's ports."""
    with open(filename) as f:
        verilog = f.read()
    header = re.search(r"^module {}\s*\((.*?)\);".format(module), verilog, re.S | re.M).group(1)
    ports  = {}
    for m in re.finditer(r"(input|output)\s+wire\s+(?:\[(\d+):0\]\s+)?(\w+)", header):
        direction = {"input": "i", "output": "o"}[m.group(1)]
        width     = int(m.group(2)) + 1 if m.group(2) is not None else 1
        ports[m.group(3)] = (direction, width)
    return ports

class TestGen(unittest.TestCase):
    def generate(self, config, name="usb3_pipe_core"):
        output_dir = tempfile.mkdtemp()
        config_file = os.path.join(output_dir, "config.yml")
        with open(config_file, "w") as f:
            yaml.dump(config, f)
        subprocess.run([sys.executable, "-m", "usb3_pipe.gen", config_file,
            "--output-dir", output_dir,
            "--name",       name],
            cwd=root_dir, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return output_dir

    def check_core(self, config, name="usb3_pipe_core"):
        output_dir = self.generate(config, name)
        gateware   = os.path.join(output_dir, "gateware")

        # Generated files.
        for filename in [
            os.path.join(gateware, f"{name}.v"),
            os.path.join(gateware, f"{name}.xdc"),
            os.path.join(output_dir, f"{name}.md"),
            os.path.join(output_dir, f"{name}_inst.v"),
            os.path.join(output_dir, f"{name}_ltssm.vh")]:
            self.assertTrue(os.path.isfile(filename), filename)

        # Verilog ports match the ports table.
        expected = {p.name: (p.direction, p.width) for p in get_ports(get_config(config))}
        ports    = get_verilog_ports(os.path.join(gateware, f"{name}.v"), name)
        self.assertEqual(ports, expected)

        # Documentation/Instantiation template list all ports.
        with open(os.path.join(output_dir, f"{name}.md")) as f:
            doc = f.read()
        with open(os.path.join(output_dir, f"{name}_inst.v")) as f:
            inst = f.read()
        for port in expected:
            self.assertIn(f"| `{port}` |", doc)
            self.assertIn(f".{port} ", inst)

        # LTSSM states encoding: documented and in the Verilog header, fits ltssm_state.
        with open(os.path.join(output_dir, f"{name}_ltssm.vh")) as f:
            header = f.read()
        states = get_ltssm_states()
        self.assertIn("U0", states)
        for state, value in states.items():
            self.assertIn(f"| {value} | {state} |", doc)
            self.assertRegex(header, r"localparam {}\s+= {}'d{};".format(
                get_ltssm_state_localparam(name, state), expected["ltssm_state"][1], value))
            self.assertLess(value, 2**expected["ltssm_state"][1])

        # Memories initialization files referenced by the Verilog are generated.
        with open(os.path.join(gateware, f"{name}.v")) as f:
            for init in re.findall(r'\$readmemh\("([^"]+)"', f.read()):
                self.assertTrue(os.path.isfile(os.path.join(gateware, init)), init)

    def test_examples(self):
        examples = sorted(glob.glob(os.path.join(examples_dir, "*.yml")))
        self.assertTrue(examples)
        for example in examples:
            with self.subTest(example=os.path.basename(example)):
                with open(example) as f:
                    self.check_core(yaml.safe_load(f))

    def test_a7_link_ctrl_host(self):
        self.check_core({
            "phy"            : "A7USB3SerDes",
            "refclk"         : "pads",
            "role"           : "host",
            "with_link_ctrl" : True,
        }, name="usb3_host")

    def test_a7_usb_core(self):
        # PHY + USB3 Device Core with Bulk IN/OUT AXI-Stream endpoints.
        self.check_core({
            "phy"           : "A7USB3SerDes",
            "refclk"        : "pads",
            "usb_core"      : True,
            "usb_pid"       : 0x1234,
            "usb_max_burst" : 4,
        }, name="usb3_device")

    def test_invalid_configs(self):
        for config in [
            {},                                                      # Missing phy.
            {"phy": "ECP5USB3SerDes"},                               # Unsupported phy.
            {"phy": "A7USB3SerDes", "sys_clk_freq": 100e6},          # sys_clk_freq < 125MHz.
            {"phy": "A7USB3SerDes", "refclk": "sma"},                # Unsupported refclk.
            {"phy": "A7USB3SerDes", "refclk": "sys", "refclk_freq": 100e6}, # refclk != sys_clk.
            {"phy": "A7USB3SerDes", "role": "hub"},                  # Unsupported role.
            {"phy": "A7USB3SerDes", "with_lmp": True},               # Unknown key.
            {"phy": "A7USB3SerDes", "usb_core": True, "role": "host"},           # usb_core: device only.
            {"phy": "A7USB3SerDes", "usb_core": True, "with_link_ctrl": True},   # usb_core drives it.
            {"phy": "A7USB3SerDes", "usb_core": True, "usb_max_burst": 3},       # Unsupported MaxBurst.
            ]:
            with self.subTest(config=config):
                with self.assertRaises(ConfigError):
                    get_config(config)

if __name__ == "__main__":
    unittest.main()
