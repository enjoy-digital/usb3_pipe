#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab targets (agentlab/targets/) elaborations (no bitstream build): CSRs (host API contract,
doc/agentlab.md) and configuration constants."""

import sys
import subprocess

import pytest

from test.test_targets import ROOT, RUN_TARGET

# Helpers ------------------------------------------------------------------------------------------

def elaborate(target, output_dir, args=[]):
    """Elaborate the AgentLab target into output_dir, return csr.csv."""
    cmd = [sys.executable, "-c", RUN_TARGET, f"agentlab/targets/{target}.py",
        "--output-dir", str(output_dir)] + args
    p = subprocess.run(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True)
    assert p.returncode == 0, p.stdout[-4000:]
    assert list((output_dir / "gateware").glob("*.v")), p.stdout[-4000:]
    return (output_dir / "csr.csv").read_text()

# Targets ------------------------------------------------------------------------------------------

AGENTLAB_CSRS = [
    "agentlab_marker",
    "agentlab_timestamp_latch",
    "agentlab_timestamp_value",
    *[f"agentlab_uart{n}_{name}" for n in range(4)
        for name in ["baudrate", "tuning_word", "status"]],
    "agentlab_gpio_oe",
    "agentlab_gpio_out",
    "agentlab_gpio_in",
    "agentlab_gpio_edge_mask",
    "agentlab_power_out",
    "agentlab_power_status",
    "agentlab_events_enable",
    "agentlab_events_clear",
    "agentlab_events_lost",
    "agentlab_events_level",
    "usb3_core_status",
]

AGENTLAB_LA_CSRS = [
    "agentlab_la_mode",
    "agentlab_la_divider",
    "agentlab_la_trigger_source",
    "agentlab_la_trigger_mask",
    "agentlab_la_trigger_value",
    "agentlab_la_trigger_edge",
    "agentlab_la_trigger_ext",
    "agentlab_la_pre_trigger",
    "agentlab_la_length",
    "agentlab_la_control",
    "agentlab_la_status",
    "agentlab_la_level",
    "agentlab_la_count",
    "agentlab_la_trigger_offset",
    "agentlab_la_start_timestamp",
    "agentlab_la_trigger_timestamp",
    "ddrctrl_init_done",
    "ddrctrl_init_error",
]

TARGETS = {
    #  target  : (gpios, power, la word samples)
    "acorn"    : (2,  1, 4),  # DDR3 x16: 128-bit native port.
    "kc705"    : (16, 4, 16), # DDR3 x64: 512-bit native port.
}

@pytest.mark.parametrize("target", list(TARGETS))
@pytest.mark.parametrize("dram", [True, False])
def test_agentlab_target(target, dram, tmp_path):
    csr = elaborate(target, tmp_path, [] if dram else ["--no-dram"])
    registers = [line.split(",")[1] for line in csr.splitlines() if line.startswith("csr_register")]
    for name in AGENTLAB_CSRS:
        assert name in registers, name
    for name in AGENTLAB_LA_CSRS:
        assert (name in registers) == dram, name
    gpios, power, word_samples = TARGETS[target]
    assert "constant,agentlab_uarts,4" in csr
    assert f"constant,agentlab_gpios,{gpios}" in csr
    assert f"constant,agentlab_power,{power}" in csr
    assert "constant,agentlab_events_ep,11" in csr
    if dram:
        # Logic analyzer: 32 channels, EP12, ring buffer in the upper half of the 1GB DDR3.
        assert "constant,agentlab_la_channels,32" in csr
        assert "constant,agentlab_la_interface,11" in csr
        assert "constant,agentlab_la_ep,12" in csr
        assert f"constant,agentlab_la_ring_base,{512*2**20}" in csr
        assert f"constant,agentlab_la_ring_size,{512*2**20}" in csr
        assert f"constant,agentlab_la_ring_samples,{128*2**20}" in csr
        assert f"constant,agentlab_la_word_samples,{word_samples}" in csr
        assert "memory_region,main_ram,0x40000000,1073741824" in csr
    else:
        assert "agentlab_la" not in csr
