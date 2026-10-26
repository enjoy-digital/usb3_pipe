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

AGENTLAB_EYES_CSRS = [f"agentlab_eyes_{name}" for name in ["wave_row", "wave_lanes",
    "wave_column", "wave_first", "wave_color", "wave_divider", "wave_control", "wave_status",
    "frames"]]

TARGETS = {
    #  target  : (gpios, power)
    "acorn"    : (2,  1),
    "kc705"    : (16, 4),
}

def eyes_lanes(gpios, power):
    return gpios + power + 2*4 # GPIOs, power switches, 4 DUT UARTs TX/RX lines.

@pytest.mark.parametrize("target", list(TARGETS))
def test_agentlab_target(target, tmp_path):
    csr = elaborate(target, tmp_path)
    registers = [line.split(",")[1] for line in csr.splitlines() if line.startswith("csr_register")]
    for name in AGENTLAB_CSRS:
        assert name in registers, name
    gpios, power = TARGETS[target]
    assert "constant,agentlab_uarts,4" in csr
    assert f"constant,agentlab_gpios,{gpios}" in csr
    assert f"constant,agentlab_power,{power}" in csr
    assert "constant,agentlab_events_ep,11" in csr
    # Eyes (default): CSRs, constants, text grid region.
    for name in AGENTLAB_EYES_CSRS:
        assert name in registers, name
    assert "constant,agentlab_eyes_interface,11" in csr
    assert "constant,agentlab_eyes_ep,14" in csr
    assert "constant,agentlab_eyes_columns,160" in csr
    assert f"constant,agentlab_eyes_wave_lanes,{eyes_lanes(gpios, power)}" in csr
    assert "memory_region,agentlab_eyes,0x90000000,32768,io" in csr

@pytest.mark.parametrize("target", list(TARGETS))
def test_agentlab_target_no_eyes(target, tmp_path):
    csr = elaborate(target, tmp_path, ["--no-eyes"])
    assert "agentlab_eyes" not in csr
