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
  programmer picks the first FTDI it finds.
- Use a dedicated `litex_server` port (ex: 1240): another `litex_server`/Etherbone bridge may already
  listen on the default 1234 and a client would silently talk to another board. Always check the
  identifier first (`litex_cli --port 1240 --ident` must return `USB3SoC ...`).

## Build And Load

```bash
# Debug build: JTAGBone, SerDes debug (controls/measurements), LiteScope "link" probes.
./acorn.py --build --with-jtagbone --with-serdes-debug --with-analyzer --analyzer-group link \
    --output-dir build/acorn_dbg

# Load (SRAM) + litex_server + N link training attempts, results in build/hw_runs/<date>/.
./scripts/acorn_run.py --build-dir build/acorn_dbg --attempts 20
```

`scripts/acorn_run.py` records, for each attempt: the LTSSM state/history/counters, the SerDes
measurements and the host view (new `/var/log/kern.log` USB lines, USB ports states).

## Host Side

Without root access, the host view comes from:
- `/var/log/kern.log` (readable by the `adm` group): connect/enumeration/errors, ex:
  `usb usb6-port1: Cannot enable. Maybe the USB cable is bad?` (link not reaching U0 from the host
  point of view, the host then retries with a Warm Reset every ~4s).
- `lsusb -t` and `/sys/bus/usb/devices/*/usb*-port*/state`.

## PHY Debug CSRs (always built, `usb3_pipe/debug.py: USB3PHYDebug`)

```bash
./scripts/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv status   # State/counters.
./scripts/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv history  # Last 32 transitions.
./scripts/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv retrain  # PHY reset + clear.
./scripts/usb3_debug.py --port 1240 --csr-csv build/acorn_dbg/csr.csv watch    # Print changes.
```

- `ltssm_state`, SerDes `ready`, `rx_idle`, `rx_polarity`, us timestamp.
- LTSSM history: last 32 transitions with timestamps (answers "how far did training go, and how long
  did each state last").
- Counters: TS1/TS1_INV/TS2 detections, LFPS detections (Polling/Ping/U1/U2/U3/Warm Reset), RX idle
  edges, RX error cycles, K28.4 substituted words, U0/Recovery entries.
- `phy_reset` holds the PHY in reset (TX electrical idle), useful to observe what the partner sends
  alone.

## SerDes Debug CSRs (`--with-serdes-debug`, `USB3SerDesDebug`)

- `tx_polarity`, `tx_inhibit`, `loopback` (0b010: Near-End PMA loopback, set it before a retrain).
- `tx_freq`/`rx_freq` (100ms): the RX clock is the recovered clock. Its offset vs the TX clock shows
  the CDR frequency tracking: a host with SSC averages ~-2500ppm, exactly 0ppm means the RX is not
  tracking the host (ex: decoding our own TX crosstalk).
- RX 8b/10b statistics in the RX clock domain: symbols, decoder/code invalid, disparity errors, error
  bursts and longest error-free run (random BER vs periodic slips).

## LiteScope

Keep captures narrow (well below 128 bits), in the sys clock domain (captures in the 250MHz RX clock
domain do not meet timing). Groups: `link` (LTSSM/TS/LFPS/RX status), `lfps`, `ts`, `datapath`.

```bash
litescope_cli --port 1240 --csv build/acorn_dbg/analyzer.csv --csr-csv build/acorn_dbg/csr.csv \
    -v usb3_phy_ltssm_state 4 --subsampling 512 --offset 32 --dump capture.vcd
```

Use `--subsampling` to cover ms-scale sequences (4096 x 512 x 8ns = 16.8ms) on level signals.
LTSSM state encodings: `usb3_pipe.gen.get_ltssm_states()`.

## Useful Experiments

- **Near-End PMA loopback**: the LTSSM trains against itself up to U0, this validates the GT at 5Gbps,
  8b/10b, word alignment, TS detection and the LTSSM independently of the link/partner.
- **PHY held in reset**: RX activity (rx_idle edges) then only comes from the partner.
- **TX polarity inversion**: if what is received is our own TX (crosstalk), TS1 become TS1_INV.
- **TX inhibit in U0**: if the RX goes electrically idle / the recovered clock drifts, the partner was
  not transmitting.

## Investigation Loop

1. Reproduce with `scripts/acorn_run.py` and note the build command/bitstream/timing.
2. Answer the next question with the CSRs (history, counters, measurements) before LiteScope.
3. Capture a narrow LiteScope trace when the question is temporal.
4. Reproduce the finding in simulation (migen unit test or `sim.py`), fix, run `pytest` and
   `pytest -m slow`, rebuild, re-verify on hardware.
5. Keep reusable probes/scripts in the tree, exploratory ones out of commits.
