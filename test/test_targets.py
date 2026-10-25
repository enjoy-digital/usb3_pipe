#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Targets elaborations (no bitstream build): each target with each USB3 Device Core example it
supports, and the Acorn debug/analyzer configurations.

The software (BIOS) is not compiled (CPU examples: no RISC-V toolchain needed): the targets are run
with LiteX's Builder compile_software defaulting to False.
"""

import sys
import subprocess

from pathlib import Path

import pytest

from usb3_core.examples import USB3_EXAMPLES

ROOT = Path(__file__).resolve().parents[1]

# Helpers ------------------------------------------------------------------------------------------

RUN_TARGET = """
import sys, runpy
import litex.soc.integration.builder as builder
class Builder(builder.Builder):
    def __init__(self, *args, **kwargs):
        kwargs.setdefault("compile_software", False)
        super().__init__(*args, **kwargs)
builder.Builder = Builder
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
"""

def elaborate(target, output_dir, args=[]):
    """Elaborate target (gateware/software generation, no build) into output_dir, return csr.csv."""
    cmd = [sys.executable, "-c", RUN_TARGET, f"{target}.py", "--output-dir", str(output_dir)] + args
    p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    assert p.returncode == 0, p.stdout[-4000:]
    assert list((output_dir / "gateware").glob("*.v")), p.stdout[-4000:]
    return (output_dir / "csr.csv").read_text()

# Targets x Examples -------------------------------------------------------------------------------

TARGETS = ["acorn", "kc705", "alibaba_xcku3p"]

CPU_EXAMPLES  = ["console", "composite"] # Require a CPU.
DRAM_EXAMPLES = ["msc_dram"]             # Require a DRAM controller (Acorn only).

def target_examples():
    for target in TARGETS:
        for example in USB3_EXAMPLES:
            if example in DRAM_EXAMPLES and target != "acorn":
                continue
            yield target, example

@pytest.mark.parametrize("target, example", list(target_examples()))
def test_target_example(target, example, tmp_path):
    args = ["--example", example]
    if example in CPU_EXAMPLES:
        args += ["--cpu-type", "vexriscv"]
    csr = elaborate(target, tmp_path, args)
    assert "usb3" in csr

# Acorn Debug/Analyzer -----------------------------------------------------------------------------

ACORN_DEBUG_ARGS = {
    "jtagbone_serdes_debug" : ["--no-core", "--with-jtagbone", "--with-serdes-debug"],
    "analyzer_lfps"         : ["--no-core", "--with-analyzer", "--analyzer-group", "lfps"],
    "analyzer_datapath"     : ["--no-core", "--with-analyzer", "--analyzer-group", "datapath"],
    "analyzer_phase"        : ["--no-core", "--with-serdes-debug", "--with-analyzer", "--analyzer-group",
                               "phase", "--lpm-hold", "rxeq", "--gtp-param", "p_RXCDR_HOLD_DURING_EIDLE=1"],
}

@pytest.mark.parametrize("name", list(ACORN_DEBUG_ARGS))
def test_acorn_debug(name, tmp_path):
    # Acorn target elaborates with the USB3PHY + debug + each analyzer group (no Vivado).
    args = ACORN_DEBUG_ARGS[name]
    csr  = elaborate("acorn", tmp_path, args)
    assert "usb3_debug_history_count" in csr
    assert "usb3_debug_serdes_rx_reset" in csr
    if "--with-serdes-debug" in args:
        for name in ["usb3_serdes_debug_drp_sel", "usb3_serdes_debug_drp_addr",
                     "usb3_serdes_debug_tx_inhibit_state"]:
            assert name in csr
