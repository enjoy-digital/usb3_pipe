# USB3-PIPE Hardware Debugging Guide

Practical starting point to debug the USB3 PHY (PIPE + LTSSM) on hardware, without a USB3 protocol
analyzer. Keep it current as new debug flows are added.

The approach: observe the LTSSM and the SerDes through small CSR counters first, capture with
LiteScope only once the question is narrow, check the host side through the Linux kernel log, and
turn every finding into a simulation test.

## Setup

- LiteX Acorn Baseboard (Mini) with the SFP2USB adapter in the SFP cage, USB3 cable to the host.
- JTAG through the baseboard FT4232H (channel A). If other FTDI boards are connected, always select
  the Acorn explicitly (USB bus/device for openFPGALoader, USB location for OpenOCD): the platform
  programmer picks the first FTDI it finds. `bench/acorn_run.py` refuses to guess when several
  adapters match: set `USB3_PIPE_FTDI` to the Acorn sysfs USB location (ex: `5-4`).
- `litex_server` port: the host scripts (`bench/usb3_debug.py`, `usb3_bench.py`, `usb3_soak.py`,
  `usb3_trace.py`, `usb3_analyzer.py`, `usb3_suspend.py`, common `--csr-csv`/`--host`/`--port`
  arguments from `bench/common.py`) default to the LiteX default port 1234 (`--port`),
  `bench/acorn_run.py` starts its own `litex_server` (JTAGBone) on a dedicated port (`--port`,
  default 1240). Use a dedicated port (ex: `litex_server --jtag --bind-port 1240` and
  `--port 1240` on the scripts, as in the examples below) when another `litex_server`/Etherbone
  bridge may already listen on 1234: a client would silently talk to another board. Always check the
  identifier first (`litex_cli --port 1240 --ident` must return `USB3SoC ...`).

## Build And Load

```bash
# Debug build: JTAGBone, SerDes debug (controls/measurements), LiteScope "link" probes.
./bench/acorn.py --build --with-jtagbone --with-serdes-debug --with-analyzer --analyzer-group link \
    --output-dir build/acorn_dbg

# Load (SRAM) + litex_server + N link training attempts, results in build/hw_runs/<date>/.
./bench/acorn_run.py --build-dir build/acorn_dbg --attempts 20
```

`bench/acorn_run.py` records, for each attempt: the LTSSM state/history/counters, the SerDes
measurements and the host view (new kernel log USB lines, USB ports states).

## Host Side

Without root access, the host view comes from:
- The kernel log: `/var/log/kern.log` (readable by the `adm` group, `--kern-log` on
  `acorn_run.py`/`usb3_debug.py`/`usb3_soak.py`/`usb3_suspend.py`, `journalctl -k` when the file is
  missing): connect/enumeration/errors, ex:
  `usb usb6-port1: Cannot enable. Maybe the USB cable is bad?` (link not reaching U0 from the host
  point of view, the host then retries with a Warm Reset every ~4s).
- `lsusb -t` and `/sys/bus/usb/devices/*/usb*-port*/state`.

`./bench/usb3_debug.py host` lists the SuperSpeed devices (VID/PID, strings, speed, configuration)
and the last USB kernel messages. Enumeration messages arrive seconds after a link training, wait
before reading them. Useful messages:
- `can't read configurations, error -61`: wrong descriptor type returned (descriptors ROM/offsets).
- `can't set config #1, error -71`: transaction error on the control transfer (check the Status stage
  ACK TP SeqNum with a `datapath` capture).

## PHY Debug CSRs (always built, `usb3_pipe/debug.py: USB3PHYDebug`)

```bash
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv status   # State/counters.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv history  # Last 32 transitions.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv retrain  # PHY reset + clear.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv watch    # Print changes.
```

- `ltssm_state`, SerDes `ready`, `rx_idle`, `rx_polarity`, us timestamp.
- LTSSM history: last 32 transitions with timestamps (answers "how far did training go, and how long
  did each state last").
- Counters: TS1/TS1_INV/TS2 detections, LFPS detections (Polling/Ping/U1/U2/U3/Warm Reset), RX idle
  edges, RX error cycles, K28.4 substituted words, U0/Recovery entries.
- `phy_reset` holds the PHY in reset (TX electrical idle), useful to observe what the partner sends
  alone.
- `serdes_rx_reset` re-initializes the transceiver RX (PMA/CDR/offset calibration). The PHY also does
  it on each training failure (exit to Rx.Detect): the RX initialization does not always end in a
  good state (~1/12 on Artix7 GTP with an SSC host).

## SerDes Debug CSRs (`--with-serdes-debug`, `USB3SerDesDebug`)

- `tx_polarity`, `tx_inhibit`, `loopback` (0b010: Near-End PMA loopback, set it before a retrain).
- `tx_freq`/`rx_freq` (100ms): the RX clock is the recovered clock. Its offset vs the TX clock shows
  the CDR frequency tracking: a host with SSC averages ~-2500ppm, exactly 0ppm means the RX is not
  tracking the host (ex: decoding our own TX crosstalk).
- RX 8b/10b statistics in the RX clock domain: symbols, decoder/code invalid, disparity errors, error
  bursts and longest error-free run (random BER vs periodic slips). The longest error-free run over a
  training is a good RX quality figure: millions of symbols on a healthy link, ~1000 with CDR slips.
- `tx_inhibit_state`: TX inhibited only in an LTSSM state (ex: Polling.Active, to receive the
  partner's TSEQ/TS1 without our TX, crosstalk tests).
- `rx_tx_phase` (LiteScope `phase` group): RX (recovered) vs TX clocks phase, its slope is the
  instantaneous offset of the recovered clock: an SSC host shows a 33kHz triangle within 0/-5000ppm, a
  flat offset means the CDR frequency tracking is saturated (bit slips).
- DRP access to the transceiver attributes, to try settings without rebuilding (apply with an RX
  re-initialization):

```bash
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv rxcdr-cfg            # Read.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv rxcdr-cfg 0x87fe206024481010
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv drp-read 0x2a         # Any attribute.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv drp-dump drp.json     # Compare builds.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv rx-check --count 12   # RX init quality.
./bench/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv tx-inhibit-state Polling.Active
```

DRP addresses: UG482 Table D-2 (ex: RXCDR_CFG 0xa8-0xad, RXLPM_HF/LF_CFG 0x2a/0x2b, RX_CM_TRIM 0x7e).
Build-time overrides (GTP bring-up/debug): `bench/acorn.py --rxcdr-cfg`, `--gtp-param NAME=VALUE`,
`--lpm-hold {rxeq,always}`.

## LiteScope

Keep captures narrow (well below 128 bits), in the sys clock domain (captures in the 250MHz RX clock
domain do not meet timing). Groups: `link` (LTSSM/TS/LFPS/RX status), `lfps`, `ts`, `datapath`
(SerDes RX/TX words + Link Layer RX/TX words), `phase` (RX vs TX clocks phase, `--with-serdes-debug`),
from `usb3_pipe/debug.py` (`usb3_analyzer_groups`): `--with-analyzer --analyzer-group <group>` on all
the `bench/` targets.

`bench/usb3_analyzer.py` captures (retraining the PHY until the trigger fires) and analyzes:
LTSSM states, RX errors per state and longest error-free runs, the partner's TSEQ reception (known
pattern: errors per symbol position), Link Layer packets/link commands (LGOOD/LCRD/LUP/LDN, SHP/SDP
headers), RX vs TX clock offset profile (`phase`).

```bash
# Link initialization after U0 entry (datapath group).
./bench/usb3_analyzer.py --port 1240 --csr-csv build/acorn_dbg/csr.csv \
    --analyzer-csv build/acorn_dbg/analyzer.csv capture --state U0 --offset 256
# The partner's TSEQ (it is still in Polling.RxEQ while we are in Polling.Active).
./bench/usb3_analyzer.py ... capture --state Polling.RxEQ:Polling.Active
# A control request (ex: SET_CONFIGURATION SETUP data word), without retraining.
./bench/usb3_analyzer.py ... capture --data 0x00090100 --no-retrain --offset 1024
# Recovered clock offset profile over ~2ms (phase group).
./bench/usb3_analyzer.py ... capture --state Polling.LFPS:Polling.RxEQ --subsampling 64
# Re-analyze a capture.
./bench/usb3_analyzer.py analyze analyzer.vcd
```

```bash
litescope_cli --port 1240 --csv build/acorn_dbg/analyzer.csv --csr-csv build/acorn_dbg/csr.csv \
    -v usb3_phy_ltssm_state 4 --subsampling 512 --offset 32 --dump capture.vcd
```

Use `--subsampling` to cover ms-scale sequences (4096 x 512 x 8ns = 16.8ms) on level signals.
LTSSM state encodings: `usb3_pipe.gen.get_ltssm_states()`.

## Bulk Data And Stability

The `loopback` example (`bench/acorn.py` default) loops EP2 OUT back to EP1 IN (USB3DeviceCore Bulk
endpoints). With the device accessible to the user (`bench/99-usb3-pipe.rules`):

```bash
./bench/usb3_bulk.py info                                   # Endpoints, speed.
./bench/usb3_bulk.py loopback --size 1024 --count 1000      # Data check.
./bench/usb3_bulk.py loopback --count 1000 --random-sizes
# Soak: PHY counters (U0/Recovery/U1/U2/U3 entries), enumeration and USB kernel messages every period,
# Bulk loopback batches, periodic host reconnects (JSON lines log + summary).
./bench/usb3_soak.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv --duration 300 --period 10 \
    --bulk 64 --reconnect-every 150 --log soak.jsonl
```

Full duplex bandwidth/soak with data checked on both sides (`--example bench`):

```sh
./bench/usb3_bench.py both --csr-csv build/sqrl_acorn/csr.csv --port 1240
./bench/usb3_soak.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv --duration 300 --period 30 --bench 20
```

## USB3 Device Core: Header Packets trace

`USB3DeviceCore.add_csr(with_trace=True)` (enabled in the device class examples: `cdc_uartbone`,
`console`, `uvc`, `uvc_iso`, `msc`, `msc_dram`, `composite`) records the received and
transmitted Header Packets (TPs/DPHs/LMPs, ITPs skipped) with timestamps: the Protocol Layer exchanges
with the host (control transfers, bursts, NRDY/ERDY flow control, retries) are visible without USB
analyzer.

```sh
./bench/usb3_trace.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv clear
# ... host activity (ex: litex_term, ffplay) ...
./bench/usb3_trace.py --port 1240 --csr-csv build/sqrl_acorn/csr.csv dump
 523164.672us TX TP   ERDY      ept 1 dir 1 nump  2 seq  0
 523165.952us RX TP   ACK       ept 1 dir 1 nump  2 seq  0
 523165.952us TX DPH  ept 1 dir 1 seq  0 len     1
 523166.720us RX TP   ACK       ept 1 dir 1 nump  0 seq  1
```

The core status CSRs (`usb3_core_status`: ready/configured/address, `rx_lbad`, `tx_retry`,
`recovery_req`) count the Link Layer errors/Recovery requests (`usb3_soak.py` logs them). The prefix
is `usb3_core` in the vendor Bulk/`composite` examples, `usb3_<class>_core` in the single class
examples (ex: `usb3_uvc_core_status`); `usb3_trace.py` autodetects it (`--prefix`).

## Suspend/Resume (U3)

Linux autosuspends the devices with a driver (UVC, CDC-ACM, Mass Storage examples): the link goes to
U3 after a few seconds without activity and is resumed when the device is used again. A link in U3
must not be retrained from the debug CSRs (the host still sees its port in U3: the device ends in
Polling.LFPS timeout -> Compliance); `bench/acorn_run.py` reports these attempts as `suspended`.

`bench/usb3_suspend.py` waits for the suspend, resumes the device through its device node and
reports each resume as `clean` (U3 exit) or `reset` (U3 exit failed, the host reset the device),
with the LTSSM history captured from U3 (`history_freeze`):

```sh
$ ./bench/usb3_suspend.py --port 1240 --dev /dev/video0 --cycles 8 --csr-csv build/sqrl_acorn/csr.csv
cycle 2: clean (0.6s), LTSSM: Recovery.Active@0.0ms Recovery.Configuration@0.0ms Recovery.Idle@0.0ms U0@0.0ms
cycle 0: reset (1.7s), LTSSM: Recovery.Active@0.0ms eSS.Inactive@12.0ms Polling.ExitToRxDetect@24.0ms ...
```

The history restarts on the U3 exit (`history_u3_exit` trigger, armed by `usb3_suspend.py`): a
failed exit retraining for longer than the history depth, or a host wakeup starting before the
history is armed, is still captured from its first transition. Times are relative to the U3 exit.
LiteScope captures of a failed exit: trigger on `eSS.Inactive` (only reached when Recovery.Active
times out), with a large offset to see the end of Recovery.Active (`--no-retrain`: never retrain a
link in U3). Linux desktop camera services (PipeWire) also open/resume UVC devices on their own.

On the Acorn (uvc/composite examples, AMD xHCI host), all the resumes are clean (72/72, ~0.6s).

Before the comma alignment fix below, ~50% of the resumes were clean, the others ended with a host
reset (functional, ~1.7s resume). The failures were all captured the same way:
- U3 wakeup handshake completed on our side: the host wakeup LFPS is detected (80us), our 1ms
  response LFPS is sent, then Recovery.Active.
- The host keeps a non idle line for ~8ms after our response (`lfps` group: `rx_idle` low), then
  the line goes idle. On a clean exit, Recovery.Configuration follows within microseconds.
- During Recovery.Active, no TS1/TS2 is detected and 30-65% of the received symbols are 8b/10b
  errors (`ts` group, substituted K28.4); at the end of Recovery.Active the RX is idle and the RX vs
  TX clocks offset is exactly 0ppm (`phase` group, not tracking the host SSC as in U0).
- Recovery.Active times out (12ms) -> eSS.Inactive -> Rx.Detect/Polling; the host Warm Resets the
  port ~600ms after the wakeup.

Not improved by (same composite design, 24 cycles each unless noted, before the fix):
- RXBUF_RESET_ON_EIDLE, a GTP RX re-initialization on U3 exit (earlier builds).
- A GTP RX re-initialization while in U3, before the resume (7/12 clean vs 5/12).
- `RXCDR_HOLD_DURING_EIDLE=0` (13/24), `RXCDR_FR/PH_RESET_ON_EIDLE=1` (16/24).
- The time spent in U3 before the resume (0/3/10s: no correlation).
- Extending our response LFPS while the host LFPS is received (with RX word alignment during the
  handshake: 13/24; without: 4/24).

**Root cause (fixed, `ALIGN_COMMA_WORD=1`):** the end of the failed Recovery.Active was misleading.
Captured from the end of our response LFPS (`u3` analyzer group, trigger `U3 & tx_lfps` then
`~tx_lfps`), the failed exits show the host TS1/TS2 decoded (CDR locked, SSC tracked) but each
Ordered Set received as `COM x5 00 00 D10.2 x9`: the GTP comma aligner slid on every Ordered Set
(`rx_realign`, ~1100 pulses in 32us). With 2 symbols per word, aligning commas on even bytes only
(`ALIGN_COMMA_WORD=2`) conflicts with the consecutive COMs of the USB3 Ordered Sets when the first
one is received on an odd byte (random after the U3 Electrical Idle). Commas are now aligned on any
byte (the RXWordAligner handles the bytes rotation): 72/72 clean resumes.

LiteScope note: CSR writes through litex_server/JTAGBone are buffered, read a CSR after arming
(`analyzer.run()`) before triggering the event to capture.

## Transceiver Reference Clock (Acorn)

The Acorn has no transceiver reference clock without a PCIe host (MGTREFCLK0 is the PCIe edge
connector clock, measured at 0Hz on the LiteX Acorn Baseboard Mini): the GTP reference clock is the
200MHz oscillator from the fabric (BUFG). `A7USB3SerDes` then drives it to GTGREFCLK0
(PLL0REFCLKSEL=0b111, LiteICLink `refclk_from_fabric`): the clock reaches the GTPE2_COMMON through
the clock network. UG482 reserves GTGREFCLK for internal testing (the REQP-49 DRC is lowered to a
warning), but it is the only fabric clock input of the GTP PLL that does not go through general
interconnect.

**RX quality depending on the build (fabric clock on GTREFCLK0):** previous versions routed the
oscillator to GTREFCLK0 (REQP-49 DRC lowered to a warning). The clock then reaches the GTPE2_COMMON
through general interconnect, and the RX quality depended on the place and route: some builds kept a
clean link, others never reached a stable U0 (continuous Recovery, Warm Resets, Polling.LFPS), always
the same for a given bitstream. Timing slack, the FPGA load (32k toggling flip-flops) and the logic
placed near the GTP had no effect. On the same designs, GTGREFCLK0 gave stable links on every build
(13/13 loads, against 0/9 for the unstable GTREFCLK0 builds). If the link is unstable on a design
with a fabric reference clock, check that the PLL uses GTGREFCLK (`PLL0REFCLKSEL` = 3'd7 in the
generated Verilog).

**RX stuck in 16-bit mode (liteiclink < enjoy-digital/liteiclink#54):** the GTP RX init clears
RX_DATA_WIDTH bit 11 (DRP 0x011) during the RX PMA reset, then restores it. Before the fix, an RX
re-init (on a link training failure, or with `rx-reset`) that restarted the init between these two
steps "restored" the modified value. The RX then stays in 16-bit mode:
- RXOUTCLK is at 312.5MHz instead of 250MHz;
- DRP 0x011 reads `0x1130` instead of `0x1930`;
- link training fails.

These symptoms looked like a marginal receiver that depended on the build and on time. To check for it:
- `usb3_debug.py drp-dump`: 0x011 is `0x1930`;
- the `rx`/`tx` frequency meters agree.

Qualify each bitstream with `usb3_debug.py rx-check`. A "no_partner" result is normal while the host
is backing off between enumeration retries.

## Useful Experiments

- **Near-End PMA loopback**: the LTSSM trains against itself up to U0, this validates the GT at 5Gbps,
  8b/10b, word alignment, TS detection and the LTSSM independently of the link/partner.
- **PHY held in reset**: RX activity (rx_idle edges) then only comes from the partner.
- **TX polarity inversion**: if what is received is our own TX (crosstalk), TS1 become TS1_INV.
- **TX inhibit in U0**: if the RX goes electrically idle / the recovered clock drifts, the partner was
  not transmitting.
- **Scrambled Logical Idle looks like our TX**: both scramblers restart from the same seed after the
  TS2, the partner's Idle Symbols match ours delayed (not an echo).
- **Host disconnect/re-enumeration**: break the link for >10s (ex: a bad RXCDR_CFG over DRP), restore
  and re-initialize the RX: the host enumerates again (a retrain alone only triggers a Warm Reset).

## Investigation Loop

1. Reproduce with `bench/acorn_run.py` and note the build command/bitstream/timing.
2. Answer the next question with the CSRs (history, counters, measurements) before LiteScope.
3. Capture a narrow LiteScope trace when the question is temporal.
4. Reproduce the finding in simulation (migen unit test or `sim.py`), fix, run `pytest` and
   `pytest -m slow`, rebuild, re-verify on hardware.
5. Keep reusable probes/scripts in the tree, exploratory ones out of commits.
