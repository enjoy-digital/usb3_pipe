# Targets (`bench/`)

Each target is a LiteX SoC built on `bench/usb3_soc.py`: `USB3BaseSoC` (SoCCore without CPU by
default, UARTBone/JTAGBone bridge for the CSRs) and `add_usb3_device` (`USB3PHY` + debug CSRs + one
USB3 Device Core example design). The target file only provides the board CRG, the transceiver
pads and the SerDes. The USB3 device enumerates as `1d50:605a` (user access to the vendor class
examples: `sudo cp bench/99-usb3-pipe.rules /etc/udev/rules.d/ && sudo udevadm control --reload`).

| Target | Board | Transceiver | Gen | SFP2USB adapter | Bridge | Status |
|---|---|---|---|---|---|---|
| `acorn.py` | [LiteX Acorn Baseboard](https://github.com/enjoy-digital/litex-acorn-baseboard) (Mini, `--variant standard`) + SQRL Acorn CLE-215+ | Artix7 GTP | 1x1 | SFP cage | UARTBone (default), `--with-jtagbone` | Validated, main Gen 1 target |
| `kc705.py` | Xilinx KC705 | Kintex7 GTX (Quad 117) | 1x1 | SFP cage (`--connector sfp`) or SMA (`--connector sma`) | UARTBone (default), `--with-jtagbone` | Build-tested |
| `alibaba_xcku3p.py` | Alibaba Cloud XCKU3P | Kintex UltraScale+ GTY | 1x1, 2x1 (`--gen2-capable`) | NIC1 (`--sfp 0`, default) or NIC2 (`--sfp 1`) | JTAGBone only | Validated at Gen 1 and Gen 2, main Gen 2 target |
| `sipeed_tang_mega_138k_pro.py` | [Sipeed Tang Mega 138K Pro](https://wiki.sipeed.com/hardware/en/tang/tang-mega-138k/mega-138k-pro.html) | Gowin GW5AST-138B GTR12 (Q1) | 1x1, 2x1 in bring-up | SFP0 (`--sfp 0`, default) or SFP1 | UARTBone (default), `--with-jtagbone` | Validated at Gen 1 |

The Gowin GW5AT-15 of the README (`GW5USB3SerDes` with the GW5AT-15 Q0 configuration of
`usb3_pipe/gw5`, validated at Gen 1 with a direct USB-C connection) has no target in this
repository.

## Common options

Shared by the four targets (`usb3_soc_args`, `--help`):

| Option | Description |
|---|---|
| `--build`, `--load`, `--output-dir <dir>` | Build the bitstream, load it (volatile, Xilinx targets; Gowin: openFPGALoader, below), build directory (default `build/<board>`, `csr.csv` inside). |
| `--example <name>` | USB3 Device Core example design, default `loopback` ([examples.md](examples.md)); `--cpu-type vexriscv` for `console`/`composite`; `--no-core`: PHY only. |
| `--with-lpm` | U1/U2 Link Power Management on the vendor Bulk examples (`USB3DeviceCore(with_lpm=True)`). |
| `--max-burst 2\|4\|8\|16` | Bulk endpoints Max Burst of the vendor Bulk examples (default 8). |
| `--link-words 1\|2\|4` | Link Layer words: 32-bit (default), 64-bit (Gen 2 bandwidth, `pipe_cd` targets) or 128-bit (Gen 2 with wide raw transceiver words), vendor Bulk examples ([usb3-pipe.md](usb3-pipe.md) "Link Layer words"). |
| `--lean` | No PHY debug CSRs (`usb3_debug_*`), throughput profile and core trace: timings on small/slow FPGAs. |
| `--no-uartbone`, `--with-jtagbone` | Bridges of the targets with a UART (the XCKU3P is JTAGBone only). |
| `--with-serdes-debug` | SerDes debug CSRs (`usb3_serdes_debug_*`: TX/loopback controls, clocks and RX errors measurements). |
| `--with-analyzer --analyzer-group link\|lfps\|ts\|datapath\|phase\|u3` | LiteScope analyzer on a probes group ([debugging-guide.md](debugging-guide.md)). |

The host scripts reading CSRs (`--csr-csv build/<board>/csr.csv`, `bench/usb3_debug.py`,
`usb3_bench.py out|both`, `usb3_trace.py`, `usb3_soak.py`, `usb3_analyzer.py`, `usb3_suspend.py`)
connect to a `litex_server` on the board bridge (default port 1234, `--port`); `usb3_bulk.py` only
needs libusb. Host packages: `pip3 install pyusb libusb1`.

## Acorn CLE-215+ (LiteX Acorn Baseboard)

```sh
$ ./bench/acorn.py --build --load --with-jtagbone --example loopback
$ litex_server --jtag                                     # JTAGBone through the baseboard FT4232H.
$ ./bench/usb3_debug.py --csr-csv build/sqrl_acorn/csr.csv status
$ ./bench/usb3_bulk.py loopback --size 1024 --count 1000
```

- GTP reference clock: the board has no transceiver reference clock without a PCIe host, the 200MHz
  oscillator is routed from the fabric to GTGREFCLK0 ([debugging-guide.md](debugging-guide.md),
  "Transceiver Reference Clock").
- `--variant mini|standard`: baseboard variant (SFP pins).
- `--with-dram`: DDR3 (LiteDRAM, initialized by the BIOS: a VexRiscv CPU is added when
  `--cpu-type` is not given, BIOS console on a stub UART); implied by the `msc_dram` example.
- GTP bring-up overrides: `--rxcdr-cfg`, `--gtp-param NAME=VALUE`, `--lpm-hold rxeq|always`.
- With several FTDI adapters connected, select the Acorn explicitly (the programmer picks the first
  FTDI it finds); `bench/acorn_run.py` (load + `litex_server` + N link trainings with the results
  recorded) uses the `USB3_PIPE_FTDI` sysfs location.

Results (README status table): Bulk IN/OUT 446/449 MB/s, full duplex 376 + 400 MB/s, 139 GB soak
without error, 72/72 clean Linux autosuspend resumes.

## KC705

```sh
$ ./bench/kc705.py --build --load --with-jtagbone --connector sfp
```

Kintex7 GTX Quad 117 with the 125MHz SGMII oscillator as reference clock, `--connector sfp` (SFP2USB
adapter) or `--connector sma`; same bridges and options as the Acorn. Build-tested (elaboration in
the test suite), hardware validation in progress.

## Alibaba Cloud XCKU3P (Gen 1 and Gen 2)

```sh
$ ./bench/alibaba_xcku3p.py --build --load --example bench --gen2-capable --link-words 2
$ lsusb -t                                      # 10000M on a USB 3.2 Gen 2 root hub port.
$ litex_server --jtag --jtag-config <openocd config>   # JTAGBone, see below.
$ ./bench/usb3_bench.py both --transfer-size 65536 --csr-csv build/alibaba_xcku3p/csr.csv
```

- JTAG: Digilent JTAG-HS2 (`openFPGALoader -c digilent_hs2`, OpenOCD `cpld/xilinx-xcu.cfg` with
  `CHIP XCKU3P` for JTAGBone). No UART on the board: JTAGBone only, no `msc_dram` (no DRAM
  controller).
- `--refclk sfp|clk100`: GTY reference clock from the SFP MGT clock (156.25MHz, default) or the
  100MHz oscillator through the fabric. The RX polarity is inverted on this board (detected by the
  LTSSM).
- `--gen2-capable`: `USPUSB3SerDes(gen2_capable=True)` (QPLL at 10Gbps, 40-bit raw words, Gen 1 at
  the /2 rate) and `USB3PHY(gen2_capable=True)`: Gen 2x1 advertised in the SuperSpeedPlus Polling.
  **Gen 2 is only negotiated on a USB 3.2 Gen 2 (10Gbps) root hub port** (`lsusb -t`: 10000M); on
  a Gen 1 port the same design trains at 5Gbps. `--link-words 2` (64-bit Link Layer) is needed for
  the Gen 2 bandwidth: 744-785/~880 MB/s Bulk IN/OUT against 426/466 MB/s with 32-bit words (README
  status table, [examples.md](examples.md) "Gen 2").
- Gen 2 CSRs (`add_usb3_gen2_rx`, [debugging-guide.md](debugging-guide.md) "Gen 2"):
  `usb3_gen2_status` (Gen 2 negotiated, 10Gbps rate), `usb3_gen2_capable` (clear it and retrain
  with `usb3_debug.py retrain` for Gen 1 operation), RX/TX snapshots and counters. SoC-level CSRs
  appear with the `main_` prefix in `csr.csv` (`main_usb3_gen2_status`).

## Sipeed Tang Mega 138K Pro (Gowin GW5AST-138B GTR12)

```sh
$ ./bench/sipeed_tang_mega_138k_pro.py --build --example bench    # Gen 1, 64-bit Link Layer at 62.5MHz.
$ openFPGALoader -c ft2232 build/sipeed_tang_mega_138k_pro/gateware/impl/pnr/project.fs
$ litex_server --uart --uart-port /dev/ttyUSBx                     # UARTBone (default bridge).
$ ./bench/usb3_bench.py both --transfer-size 65536 --csr-csv build/sipeed_tang_mega_138k_pro/csr.csv
```

Toolchain: Gowin IDE (its `serdes_toml_to_csr` tool, 1.9.12, generated the GTR12 configurations of
`usb3_pipe/gw5`) and LiteICLink with the GTR12 support. `--load` is not usable on this target:
`usb3_soc_build` loads a `.bit` file and the Gowin flow produces `gateware/impl/pnr/project.fs`,
load it with openFPGALoader as above (volatile).

Clocking: the GW5A fabric does not run the LTSSM/Link Layer/Core at 125MHz. By default `sys` is
62.5MHz (`--sys-clk-freq`), the PIPE and the SerDes datapath run on the GTR12 TX clock at 125MHz
(`USB3PHY(pipe_cd="tx")`, [usb3-pipe.md](usb3-pipe.md) "Slower fabrics") and the Link Layer uses
64-bit words (`--link-words 2` forced, `--link-words 4`: 128-bit). With `--sys-clk-freq 125e6` the
whole design is in `sys` (32-bit words possible). With `pipe_cd`: U1/U2 are only built with
`--with-lpm` (`USB3PHY(with_u1u2=False)` otherwise), the throughput profile of the `bench` example
and the core trace are not built (timings, `--with-trace` adds the trace); the PHY debug CSRs
(`usb3_debug_*`) are, as on the other targets, unless `--lean` (then a minimal `usb3_phy_status`:
LTSSM state, U0, SerDes ready).

Options (`--help`):

| Option | Description |
|---|---|
| `--sys-clk-freq` | System clock frequency (default 62.5MHz; 125MHz: PHY/Core/SoC in `sys`, no `pipe_cd`). |
| `--sfp 0\|1` | SFP cage: GTR12 Q1 lane 0 or 1. |
| `--refclk 100M\|125M` | GTR12 reference clock: 100MHz REFCLK1 (default) or 125MHz REFCLK0, from the dock PLLs. |
| `--pll QPLL1\|CPLL` | GTR12 lane PLL (default QPLL1; CPLL: 125MHz REFCLK0, lane 0, required for 10Gbps). |
| `--serdes-csr <file>` | GTR12 register configuration override (Gowin `serdes_toml_to_csr` output, sources in `usb3_pipe/gw5`). |
| `--rx-idle-source` | RX Electrical Idle indicator (`~astat5` default, `elecidle`, `[~]astat<n>`). |
| `--gen2-capable` | Gen 2 (10Gbps) capable SerDes/PHY: GTR12 rate change through UPAR, 80-bit raw words. |
| `--with-trace` | USB3 Device Core Header Packets trace (`usb3_core_trace_*`, `bench/usb3_trace.py`), off by default with `pipe_cd`. |
| `--with-gen2-debug` | Gen 2 RX monitor on the raw RX words (`usb3_gen2_rx_*`: alignment status, blocks/Ordered Sets counters). |
| `--with-tx-check`, `--with-tx-raw-check` | Gen 2 TX self-checks: transmitted Header Packets/Link Commands CRCs before (`usb3_tx_check_*`) and after (`usb3_tx_raw_check_*`, `tx_raw_counts_*`) the scrambling. |
| `--with-ltssm-history` | Gen 2 bring-up probes: TX FFE/RX CTLE/TX re-initialization CSRs (`usb3_tx_ffe`, `usb3_rx_eq`, `usb3_tx_reinit`, with `--gen2-capable`) and, without `--with-gtr12-debug`, the LGOOD latency histogram, Gen 2 RX backlog/counters/framing probes and GTR12 RX valid probe. |
| `--with-gtr12-debug` | GTR12/PIPE bring-up: lane status transitions (`gtr12_transitions_*`), raw/words RX snapshots and TX snapshot/ring (`rx_snapshot_*`, `tx_snapshot_*`, `tx_ring_*`), SuperSpeedPlus Polling status (`ssp_status_*`), UPAR CSRs. |
| `--with-gtr12-analyzer` | LiteScope analyzer on the GTR12 raw RX words (rx clock domain). |
| `--place-option 0\|1\|2`, `--route-option 0\|1\|2` | Gowin place/route algorithms (defaults 2/1, timing driven). |

The wide bring-up status registers are read 32-bit word by word (`<name>_sel`/`<name>_data`, the
layouts are in the `add_*` docstrings of the target); the GTR12 lane status and UPAR register
access are `usb3_serdes_gtr12_status`/`usb3_serdes_gtr12_upar_*`.

### Gen 2 (10Gbps) bring-up

```sh
$ ./bench/sipeed_tang_mega_138k_pro.py --build --example bench --gen2-capable --pll CPLL --refclk 125M \
    --link-words 4 --with-gen2-debug --with-tx-check
```

`--gen2-capable` with `--pll CPLL --refclk 125M` (SFP0: the 10Gbps rate on the GW5AST-138B Q1 lanes
requires the CPLL configuration, the QPLL1 one gives bit errors) advertises Gen 2x1; once negotiated
on a USB 3.2 Gen 2 host port, the lane rate is switched through UPAR (`GW5RateSequencer`, 80-bit raw
words at 125MHz) and the Gen 2 framing runs at 16 bytes per cycle. `--link-words 4` (128-bit Link
Layer words) is needed for the full rate (with 64-bit words the Gen 2 rate is credit-throttled at
~500MB/s). Runtime CSRs: `usb3_gen2_capable`/`usb3_gen2_status`, `usb3_rx_flow` (Type 2 credits,
RX throttle level), `usb3_tx_ffe`/`usb3_rx_eq`/`usb3_tx_reinit` (`--with-ltssm-history`). Status and
remaining work (timings of the 80-bit datapaths at 125MHz): [gen2-x2-roadmap.md](gen2-x2-roadmap.md)
"Next steps".

Gen 1 results (README status table): Bulk IN/OUT 372/433 MB/s, full duplex 262 + 383 MB/s, 20s
runs without error.

## Other boards

Any 7-Series/UltraScale+/GW5AST board with a transceiver routed to a SFP or PCIe connector can be
used with a target file of about 100 lines: copy the closest target (CRG, `platform.request` of the
transceiver pads, SerDes class), the SoC, the options and the example designs are shared
([integration.md](integration.md) for a design that is not a bench SoC).
