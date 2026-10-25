#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 LiteScope capture/analysis (acorn.py --with-analyzer, through litex_server).

Captures (the PHY is retrained until the trigger fires):
  # U0 entry (datapath group: link commands, LMPs, packets).
  ./bench/usb3_analyzer.py --csr-csv build/sqrl_acorn/csr.csv capture --state U0 --offset 256
  # Polling.RxEQ -> Polling.Active (the partner's TSEQ: RX quality on a known pattern).
  ./bench/usb3_analyzer.py --csr-csv build/sqrl_acorn/csr.csv capture --state Polling.RxEQ:Polling.Active
  # A data word on the Link Layer RX (ex: SET_CONFIGURATION SETUP data, the PHY is not retrained).
  ./bench/usb3_analyzer.py --csr-csv build/sqrl_acorn/csr.csv capture --data 0x00090100 --no-retrain
  # RX vs TX clocks phase over ~2ms (phase group).
  ./bench/usb3_analyzer.py --csr-csv build/sqrl_acorn/csr.csv capture --state Polling.LFPS:Polling.RxEQ --subsampling 64

Analysis of a capture (analyzer.vcd):
  ./bench/usb3_analyzer.py analyze analyzer.vcd
"""

import sys
import time
import argparse
import collections

from common import add_remote_args, open_remote # First: repository root on the path.

from usb3_pipe.gen import get_ltssm_states

# Gen1 TSEQ Ordered Set (USB 3.2 Table 6-3).
TSEQ = [0xbc, 0xff, 0x17, 0xc0, 0x14, 0xb2, 0xe7, 0x02, 0x82, 0x72, 0x6e, 0x28, 0xa6, 0xbe, 0x6d, 0xbf] + [0x4a]*16

# Link Layer framing (Link Layer view, first symbol on data[31:24]).
FRAMING = {0xfefefef7: "SLC", 0xfbfbfbf7: "SHP", 0x5c5c5cf7: "SDP", 0xfdfdfdf7: "END", 0x7c7c7cf7: "EDB"}

# Link Commands (USB 3.2 Table 7-?), 11-bit Link Command Information (class/type/subtype).
def link_command_name(word):
    lc   = (word >> 8) & 0xff | (word & 0xff) << 8 # 16-bit Link Command Word (byte-swapped).
    info = lc & 0x7ff
    cls, typ, sub = (info >> 9) & 0x3, (info >> 7) & 0x3, info & 0xf
    names = {
        (0, 0): lambda: f"LGOOD_{sub & 0x7}",
        (0, 1): lambda: f"LCRD_{'ABCD'[sub & 0x3]}",
        (0, 2): lambda: "LRTY",
        (0, 3): lambda: "LBAD",
        (1, 0): lambda: f"LGO_U{sub & 0x3}",
        (1, 1): lambda: "LAU",
        (1, 2): lambda: "LXU",
        (1, 3): lambda: "LPMA",
        (2, 0): lambda: "LUP",
        (2, 3): lambda: "LDN",
    }
    return names.get((cls, typ), lambda: f"LC_0x{info:03x}")()

# VCD ----------------------------------------------------------------------------------------------

def load_vcd(filename):
    """Minimal VCD reader: returns (names, samples) with one dict per LiteScope sample."""
    ids, names, cur, rows, t = {}, [], {}, [], None
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if line.startswith("$var"):
                p = line.split()
                ids[p[3]] = p[4]
                names.append(p[4])
                cur[p[4]] = 0
            elif line.startswith("#"):
                if t is not None:
                    rows.append((t, dict(cur)))
                t = int(line[1:])
            elif line[:1] == "b":
                v, i = line[1:].split()
                cur[ids[i]] = int(v.replace("x", "0").replace("z", "0"), 2)
            elif line[:1] in "01xz" and len(line) > 1:
                cur[ids[line[1:]]] = int(line[0]) if line[0] in "01" else 0
    if t is not None:
        rows.append((t, dict(cur)))
    # Resample: one sample per scope clock period (2 VCD steps).
    ts      = [t for t, _ in rows]
    step    = min(b - a for a, b in zip(ts, ts[1:]) if b > a)
    samples = []
    j       = 0
    for t in range(ts[0], ts[-1] + step, step):
        while j + 1 < len(rows) and rows[j + 1][0] <= t:
            j += 1
        samples.append(rows[j][1])
    return names, samples[::2]

def find(names, suffix):
    for name in names:
        if name.endswith(suffix):
            return name
    return None

# Analysis -----------------------------------------------------------------------------------------

def symbols(samples, data, ctrl, valid=None, states=None, state=None):
    r = []
    for s in samples:
        if valid is not None and not s[valid]:
            continue
        if states is not None and s[state] not in states:
            continue
        d, c = s[data], s[ctrl]
        r += [((d >> 8*k) & 0xff, (c >> k) & 0x1) for k in range(4)]
    return r

def analyze(filename, ns_per_sample=8, ppm_window=32):
    names, samples = load_vcd(filename)
    state_names = {v: k for k, v in get_ltssm_states().items()}
    state  = find(names, "ltssm_state")
    rx_d   = find(names, "usb3_serdes_source_source_payload_data")
    rx_c   = find(names, "usb3_serdes_source_source_payload_ctrl")
    rx_v   = find(names, "usb3_serdes_source_source_valid")
    phase  = find(names, "rx_tx_phase")
    print(f"{filename}: {len(samples)} samples, signals: {len(names)}")

    # LTSSM states.
    if state is not None:
        last = None
        for i, s in enumerate(samples):
            if s[state] != last:
                print(f"  {i:5d}: {state_names.get(s[state], s[state])}")
                last = s[state]

    # RX symbols errors (K28.4 substitutions) per LTSSM state and longest error-free runs.
    if rx_d is not None:
        per_state = collections.defaultdict(lambda: [0, 0])
        for s in samples:
            if not s[rx_v]:
                continue
            for k in range(4):
                per_state[s[state]][1] += 1
                per_state[s[state]][0] += ((s[rx_c] >> k) & 1) and ((s[rx_d] >> 8*k) & 0xff) == 0x9c
        print("RX errors (K28.4 substitutions) per state:")
        for st, (e, n) in sorted(per_state.items()):
            print(f"  {state_names.get(st, st):24s}: {e}/{n} ({100*e/n:.2f}%)")
        syms = symbols(samples, rx_d, rx_c, rx_v)
        runs, cur = [], 0
        for b, k in syms:
            if k and b == 0x9c:
                runs.append(cur)
                cur = 0
            else:
                cur += 1
        runs.append(cur)
        print(f"Longest error-free runs (symbols): {sorted(runs, reverse=True)[:5]}")

        # Partner's TSEQ quality (aligned on received COMs).
        data   = [b for b, k in syms]
        starts = [i for i in range(len(data) - 32) if data[i:i+3] == TSEQ[:3]]
        if starts:
            good  = sum(data[i:i+32] == TSEQ for i in starts)
            match = sum(sum(a == b for a, b in zip(data[i:i+32], TSEQ)) for i in starts)
            pos   = collections.Counter(j for i in starts for j in range(32) if data[i + j] != TSEQ[j])
            print(f"TSEQ: {good}/{len(starts)} fully correct, {100*match/(32*len(starts)):.1f}% symbols, "
                  f"errors per symbol position: {dict(sorted(pos.items()))}")

    # Link Layer packets (RX: PHY source, TX: PHY sink).
    for direction, prefix in [("RX", "endpoint0_source_"), ("TX", "endpoint0_sink_")]:
        d, v = find(names, prefix + "payload_data"), find(names, prefix + "valid")
        if d is None:
            continue
        print(f"Link Layer {direction}:")
        words = [(i, s[d]) for i, s in enumerate(samples) if s[v] and s[d] != 0]
        k = 0
        while k < len(words):
            i, w = words[k]
            if w in FRAMING:
                name = FRAMING[w]
                if name == "SLC" and k + 1 < len(words):
                    print(f"  {i:5d}: {link_command_name(words[k + 1][1])}")
                    k += 2
                    continue
                payload = []
                for _, x in words[k + 1:k + 5]:
                    if x in FRAMING:
                        break
                    payload.append(f"{x:08x}")
                payload = " ".join(payload)
                print(f"  {i:5d}: {name} {payload}")
            k += 1

    # RX vs TX clocks frequency offset (phase group).
    if phase is not None:
        acc, prev, p = 0, None, []
        for s in samples:
            if prev is not None:
                acc += (s[phase] - prev + 32768) % 65536 - 32768
            prev = s[phase]
            p.append(acc)
        print(f"RX vs TX clock offset (ppm, {ppm_window}-sample windows):")
        dt = ns_per_sample*1e-9
        for i in range(0, len(p) - ppm_window, ppm_window):
            ppm = (p[i + ppm_window] - p[i])/(ppm_window*dt*250e6)*1e6
            print(f"  {i*dt*1e6:9.1f}us {state_names.get(samples[i][state], '?'):24s} {ppm:7.0f}")

# Capture ------------------------------------------------------------------------------------------

def capture(args):
    from litescope import LiteScopeAnalyzerDriver
    from usb3_debug import USB3DebugClient

    states = get_ltssm_states()
    bus    = open_remote(args)
    dbg      = USB3DebugClient(bus)
    analyzer = LiteScopeAnalyzerDriver(bus.regs, "analyzer", config_csv=args.analyzer_csv)
    analyzer.configure_group(0)
    analyzer.configure_subsampler(args.subsampling)
    signals = [l.split(",")[2] for l in open(args.analyzer_csv) if l.startswith("signal,")]
    signal  = lambda suffix: [n for n in signals if n.endswith(suffix)][0]
    if args.state is not None:
        # State entry (STATE) or transition (FROM:TO): sequential trigger terms.
        for name in args.state.split(":"):
            analyzer.add_trigger(cond={signal("ltssm_state"): str(states[name])})
    if args.data is not None:
        analyzer.add_trigger(cond={
            signal("endpoint0_source_payload_data") : args.data,
            signal("endpoint0_source_valid")        : "1",
        })
    for i in range(args.retries):
        # Arm with the PHY held in reset (if retraining), so the trigger sees a real state entry.
        if not args.no_retrain:
            dbg.reg("phy_reset").write(1)
            dbg.clear()
        analyzer.run(offset=args.offset, length=args.length)
        if not args.no_retrain:
            dbg.reg("phy_reset").write(0)
        t0 = time.time()
        while (time.time() - t0) < args.timeout and not analyzer.done():
            time.sleep(0.05)
        if analyzer.done():
            break
    if not analyzer.done():
        print("No trigger.")
        bus.close()
        sys.exit(1)
    analyzer.upload()
    analyzer.save(args.vcd)
    bus.close()
    analyze(args.vcd, ns_per_sample=8*args.subsampling, ppm_window=args.ppm_window)

# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 LiteScope capture/analysis.")
    add_remote_args(parser)
    parser.add_argument("--analyzer-csv", default="build/sqrl_acorn/analyzer.csv", help="Analyzer configuration file.")
    parser.add_argument("--state",        default=None,                            help="Trigger on LTSSM state entry (STATE) or transition (FROM:TO).")
    parser.add_argument("--data",         default=None,                            help="Trigger on a Link Layer RX data word (hex, x: don't care nibble).")
    parser.add_argument("--offset",       default=16,   type=int,                  help="Trigger offset (samples).")
    parser.add_argument("--length",       default=4096, type=int,                  help="Capture length (samples).")
    parser.add_argument("--subsampling",  default=1,    type=int,                  help="Subsampling.")
    parser.add_argument("--retries",      default=20,   type=int,                  help="Max retrains.")
    parser.add_argument("--timeout",      default=2.0,  type=float,                help="Wait time per retrain (s).")
    parser.add_argument("--no-retrain",   action="store_true",                     help="Don't retrain the PHY (wait for the trigger).")
    parser.add_argument("--vcd",          default="analyzer.vcd",                  help="Capture file.")
    parser.add_argument("--ppm-window",   default=32,   type=int,                  help="Clock offset measurement window (samples).")
    parser.add_argument("--ns-per-sample", default=8.0, type=float,                help="analyze: sample period (ns, x subsampling).")
    parser.add_argument("command",        choices=["capture", "analyze"])
    parser.add_argument("file",           nargs="?",                               help="analyze: capture file.")
    args = parser.parse_args()

    if args.command == "capture":
        capture(args)
    else:
        analyze(args.file or args.vcd, ns_per_sample=args.ns_per_sample, ppm_window=args.ppm_window)

if __name__ == "__main__":
    main()
