#!/usr/bin/env python3
#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register documentation tool of the manual:
#
#   csr_doc.py dump [--check] [CONFIG ...]   elaborate the bench targets (no FPGA toolchain) and
#                                            write the CSR snapshots doc/manual/csr/<config>.json
#                                            (--check: fail if a committed snapshot differs)
#   csr_doc.py generate [--check]            write doc/manual/A_registers.md from the snapshots
#                                            (--check: fail if the appendix is not up to date)
#
# The snapshots carry, per register bank of the USB3 cores: the register name, its offset from the
# bank base, its width, access, reset value, description and fields, as the LiteX CSR definitions
# document them. Descriptions therefore live in the gateware sources (CSRStorage/CSRStatus/CSRField
# descriptions): edit them there, refresh the snapshots with `dump`, then regenerate the appendix.

import argparse
import importlib
import json
import os
import sys
import tempfile

HERE         = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT    = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SNAPSHOT_DIR = os.path.join(REPO_ROOT, "doc", "manual", "csr")
APPENDIX     = os.path.join(REPO_ROOT, "doc", "manual", "A_registers.md")

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
PREFIXES = [b[0] for b in BANKS]

# Notes printed after the title of a bank family, for registers whose definitions carry no
# description.
NOTES = {
    "usb3_gen2": ["The snapshot banks share one interface: `arm` starts a capture, `trigger` selects",
                  "its stop condition, `done` tells when the buffer is full, then `index` selects an",
                  "entry and `data0` to `dataN` read it. Their registers carry no description in the",
                  "source; [Debugging](#debugging) describes what each snapshot holds."],
}

# Dump ---------------------------------------------------------------------------------------------

def _const(v):
    """Reset value as an int, or None when it is not a constant."""
    from migen.fhdl.structure import Constant
    if isinstance(v, Constant):
        return v.value
    if isinstance(v, int):
        return v
    return None


def _csr_entry(csr, csr_json, bank, base):
    from litex.soc.interconnect.csr import CSRStorage, CSRStatus
    full = f"{bank}_{csr.name}"
    reg  = csr_json["csr_registers"].get(full)
    if reg is None:
        return None
    if isinstance(csr, CSRStorage):
        access, reset = "rw", _const(csr.storage.reset)
    elif isinstance(csr, CSRStatus):
        access, reset = "ro", None
    else:
        access, reset = "wo", None
    fields = []
    for f in getattr(csr, "fields", None) and csr.fields.fields or []:
        fields.append({
            "name"        : f.name,
            "offset"      : f.offset,
            "size"        : f.size,
            "reset"       : f.reset_value if isinstance(f.reset_value, int) else None,
            "description" : (f.description or "").strip(),
            "values"      : [[str(v[0]), str(v[1])] for v in (f.values or [])],
        })
    return {
        "name"        : csr.name,
        "offset"      : reg["addr"] - base,
        "size"        : csr.size,
        "access"      : access,
        "reset"       : reset,
        "description" : " ".join((getattr(csr, "description", None) or "").split()),
        "fields"      : fields,
    }


def dump_config(name):
    """Elaborate a bench target (Builder without running the toolchain) and return its snapshot."""
    module_name, argv = CONFIGS[name]
    sys.path.insert(0, os.path.join(REPO_ROOT, "bench"))
    sys.path.insert(0, REPO_ROOT)
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
    soc, csr_json = captured["soc"], captured["json"]
    banks = {}
    for bank, region in sorted(soc.csr.regions.items()):
        objs = region.obj
        if bank == "main":
            objs = [c for c in objs if c.name.startswith(tuple(PREFIXES))]
        elif not bank.startswith(tuple(PREFIXES)):
            continue
        base = csr_json["csr_bases"][bank]
        regs = [_csr_entry(c, csr_json, bank, base) for c in objs]
        banks[bank] = sorted([r for r in regs if r is not None], key=lambda r: r["offset"])
    return {"config": name, "target": module_name, "argv": argv, "banks": banks}


def snapshot_path(name):
    return os.path.join(SNAPSHOT_DIR, f"{name}.json")


def cmd_dump(names, check):
    rc = 0
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    for name in names or list(CONFIGS):
        text = json.dumps(dump_config(name), indent=1, sort_keys=True) + "\n"
        path = snapshot_path(name)
        if check:
            if not os.path.exists(path) or open(path).read() != text:
                print(f"STALE: {os.path.relpath(path, REPO_ROOT)} (run csr_doc.py dump)")
                rc = 1
            continue
        open(path, "w").write(text)
        print(f"wrote {os.path.relpath(path, REPO_ROOT)}")
    return rc

# Generate -----------------------------------------------------------------------------------------

def _esc(s):
    return s.replace("|", "\\|").replace("<", "\\<").replace(">", "\\>")


def _wrap_name(name, limit=15):
    """Long register names: allow a break at an underscore (code spans cannot wrap)."""
    if len(name) <= limit:
        return f"`{name}`"
    parts, line, out = name.split("_"), "", []
    for p in parts:
        cand = f"{line}_{p}" if line else p
        if len(cand) + 1 > limit and line:
            out.append(line + "_")
            line = p
        else:
            line = cand
    out.append(line)
    return " ".join(f"`{o}`" for o in out)


def _reset(v):
    return "-" if v is None else f"`{v:#x}`"


def _bank_groups(snaps):
    """Banks of every configuration, each documented once from the configuration where it has the
    most registers (the offsets of a bank are those of one elaboration)."""
    merged = {}
    for snap in snaps:
        for bank, regs in snap["banks"].items():
            if bank not in merged or len(regs) > len(merged[bank][1]):
                merged[bank] = (snap["config"], regs)
    return merged


def generate():
    snaps = [json.load(open(snapshot_path(n))) for n in CONFIGS if os.path.exists(snapshot_path(n))]
    merged = _bank_groups(snaps)
    out = []
    out += ["<!-- Generated by doc/manual/tools/csr_doc.py generate: do not edit, edit the CSR",
            "     descriptions in the gateware sources and regenerate. -->", ""]
    out += ["# Register reference {#registers}", ""]
    out += ["This appendix lists the registers of the USB 3 cores that a bench target exposes on its",
            "CSR bus. It is generated from the register definitions of the gateware, elaborated on two",
            "bench configurations without running an FPGA toolchain. Offsets are in bytes from the base",
            "of the bank; the base addresses depend on the SoC and are in the `csr.csv` of a build.", ""]
    out += ["| Configuration | Target and options |",
            "|------------------------|--------------------------------------------------------|"]
    for s in snaps:
        argv = " ".join(f"`{a}`" for a in s["argv"])
        out.append(f"| `{s['config']}` | `{s['target']}` {argv} |")
    out += ["",
            "Access is `rw` for a register written by software (`CSRStorage`), `ro` for a status",
            "(`CSRStatus`) and `wo` for a write-only strobe (`CSR`). The reset value of a `rw`",
            "register ends its description. A register wider than 32 bits takes several",
            "32-bit words of the bus, most significant word first. The tools of `bench/` use these",
            "names through the `csr.csv` of the build, so they rarely need the offsets.", ""]
    def owner(name):
        """Longest documented prefix of a bank or register name."""
        hits = [p for p in PREFIXES if name.startswith(p)]
        return max(hits, key=len) if hits else None

    for prefix, title, origin, path in BANKS:
        groups = []
        for bank in sorted(merged):
            config, regs = merged[bank]
            if bank == "main":
                regs = [r for r in regs if owner(r["name"]) == prefix]
                if regs:
                    groups.append(("main", regs))
            elif owner(bank) == prefix:
                groups.append((bank, regs))
        if not groups:
            continue
        anchor = "reg-" + prefix.replace("_", "-")
        out += [f"## {title} {{#{anchor}}}", ""]
        where = f"`{path}`" if "/" in path else path
        what  = f"`{origin}`" if origin[0] != "t" else origin
        out += [f"Defined by {what} ({where}).", ""]
        if prefix in NOTES:
            out += NOTES[prefix] + [""]
        for bank, regs in groups:
            if bank == "main":
                out += ["Registers of the top-level `main` bank of the bench SoC.", ""]
            elif len(groups) > 1:
                out += [f"### `{bank}`", ""]
            else:
                out += [f"Bank `{bank}`.", ""]
            out += ["| Register | Offset | Bits | Access | Description |",
                    "|----------------------------|-------------|--------|---------|"
                    "----------------------------------------|"]
            notes = []
            for r in regs:
                desc = _esc(r["description"])
                if not desc and len(r["fields"]) > 1:
                    desc = "See the fields below."
                elif not desc and len(r["fields"]) == 1:
                    desc = _esc(" ".join(r["fields"][0]["description"].split()))
                elif not desc and r["access"] == "wo":
                    desc = "Write strobe."
                if r["reset"] is not None:
                    desc = f"{desc} Reset {_reset(r['reset'])}.".strip()
                desc = desc or "No description."
                out.append(f"| {_wrap_name(r['name'])} | `{r['offset']:#06x}` | {r['size']} | {r['access']} "
                           f"| {desc} |")
                if r["fields"] and len(r["fields"]) > 1:
                    notes.append(r)
            out.append("")
            for r in notes:
                out += [f"Fields of `{r['name']}`:", ""]
                out += ["| Field | Bits | Description |",
                        "|------------------------|---------|------------------------------------------------------|"]
                for f in r["fields"]:
                    hi = f["offset"] + f["size"] - 1
                    bits = f"{hi}:{f['offset']}" if f["size"] > 1 else f"{f['offset']}"
                    desc = _esc(" ".join(f["description"].split()))
                    out.append(f"| {_wrap_name(f['name'])} | {bits} | {desc} |")
                out.append("")
    return "\n".join(out).rstrip() + "\n"


def cmd_generate(check):
    text = generate()
    if check:
        if not os.path.exists(APPENDIX) or open(APPENDIX).read() != text:
            print("STALE: doc/manual/A_registers.md (run csr_doc.py generate)")
            return 1
        return 0
    open(APPENDIX, "w").write(text)
    print(f"wrote {os.path.relpath(APPENDIX, REPO_ROOT)}")
    return 0

# Main ---------------------------------------------------------------------------------------------

def main(argv):
    parser = argparse.ArgumentParser(description="Register documentation tool of the manual.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("dump", help="Elaborate the bench targets and write the CSR snapshots.")
    p.add_argument("--check", action="store_true", help="Fail if a committed snapshot differs.")
    p.add_argument("configs", nargs="*", help=f"Configurations (default: all of {', '.join(CONFIGS)}).")
    p = sub.add_parser("generate", help="Write A_registers.md from the snapshots.")
    p.add_argument("--check", action="store_true", help="Fail if the appendix is not up to date.")
    args = parser.parse_args(argv)
    if args.cmd == "dump":
        for name in args.configs:
            if name not in CONFIGS:
                parser.error(f"unknown configuration {name!r}")
        return cmd_dump(args.configs, args.check)
    return cmd_generate(args.check)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
