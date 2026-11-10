#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register appendix of the manual: which designs tools/csr_doc.py elaborates and which register banks
# it documents (the engine is generic, part of the manual kit).

import importlib
import json
import os
import sys
import tempfile

# Configurations: bench target module and command line. The Acorn configuration covers the Gen 1
# banks (PHY debug, SerDes debug, device core, throughput profile); the XCKU3P configuration adds the
# Gen 2 bring-up banks.
CONFIGS = {
    "acorn_bench"      : ("acorn",          ["--with-serdes-debug", "--example", "bench"]),
    "xcku3p_gen2_bench": ("alibaba_xcku3p", ["--with-serdes-debug", "--example", "bench", "--gen2-capable"]),
}

# Banks documented in the appendix (prefix match on the bank name, or on the register name for
# the registers the bench SoC adds at its top level, in the `main` bank), in order, with their title
# and the Python function or class that defines them.
BANKS = [
    ("usb3_debug",        "PHY debug",                     "USB3PHYDebug", "usb3_pipe/debug.py"),
    ("usb3_serdes_debug", "Transceiver debug",             "USB3SerDesDebug", "usb3_pipe/debug.py"),
    ("usb3_serdes",       "Transceiver electrical controls", "the LiteICLink transceiver wrapper", "liteiclink"),
    ("usb3_core",         "Device core status and traces", "USB3DeviceCore.add_csr", "usb3_core/device.py"),
    ("usb3_bench",        "Bench example",                 "USB3Bench", "usb3_core/bench.py"),
    ("usb3_profile",      "Throughput profile",            "add_usb3_profile", "bench/usb3_soc.py"),
    ("usb3_gen2",         "Gen 2 bring-up",                "add_usb3_gen2_rx", "bench/usb3_soc.py"),
]

# Notes printed after the title of a bank family: what the register descriptions do not say
# (shared interface, where the contents are described).
NOTES = {
    "usb3_gen2": ["The snapshot banks share one interface: `arm` starts a capture, `trigger` selects",
                  "its stop condition, `done` tells when the buffer is full, then `index` selects an",
                  "entry and `data0` to `dataN` read it. [Debugging](#debugging) describes what each",
                  "snapshot holds."],
}

# Opening paragraph of the appendix and the sentence that ends the access paragraph.
INTRO = ["This appendix lists the registers of the USB 3 cores that a bench target exposes on its",
         "CSR bus. It is generated from the register definitions of the gateware, elaborated on two",
         "bench configurations without running an FPGA toolchain. Offsets are in bytes from the base",
         "of the bank; the base addresses depend on the SoC and are in the `csr.csv` of a build."]
USAGE = ["The tools of `bench/` use these", "names through the `csr.csv` of the build, so they rarely need the offsets."]
MAIN_BANK = "Registers of the top-level `main` bank of the bench SoC."


def elaborate(name, repo_root):
    """Elaborate a bench target (Builder without running the toolchain); return (soc, csr_json)."""
    module_name, argv = CONFIGS[name]
    sys.path.insert(0, os.path.join(repo_root, "bench"))
    sys.path.insert(0, repo_root)
    target   = importlib.import_module(module_name)
    captured = {}

    def capture(parser, args, soc, **kwargs):
        from litex.soc.integration.builder import Builder
        builder = Builder(soc,
            output_dir      = args.output_dir,
            csr_json        = os.path.join(args.output_dir, "csr.json"),
            compile_software = False)
        builder.build(run=False)
        captured["soc"]  = soc
        captured["json"] = json.load(open(os.path.join(args.output_dir, "csr.json")))

    with tempfile.TemporaryDirectory() as tmp:
        target.usb3_soc_build = capture
        old_argv = sys.argv
        sys.argv = [module_name] + argv + ["--output-dir", tmp]
        try:
            target.main()
        finally:
            sys.argv = old_argv
    return captured["soc"], captured["json"], module_name, argv
