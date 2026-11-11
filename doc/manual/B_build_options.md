# Build options {#build-options}

This appendix lists the command line options of the bench targets, of the full-system simulation
and of the standalone core generator. The options come from the argument parsers of the scripts;
`--help` prints the same list with the defaults of the installed version.

## Bench targets {#opt-targets}

Every bench target in `bench/` builds a SoC with the PHY, the device core and one example. Run
without `--build` or `--load`, a target elaborates the design and writes the sources and the
`csr.csv` without calling the FPGA toolchain.

| Target | Board | Transceiver |
|----------------------------------------------|----------------------------------------|----------------|
| `acorn` | Acorn CLE-215+ on the Acorn Baseboard | GTP |
| `kc705` | KC705 | GTX |
| `alibaba_xcku3p` | Alibaba XCKU3P | GTY |
| `sipeed_tang_mega_138k_pro` | Tang Mega 138K Pro | GTR12 |
| `lambdaconcept_ecpix5` | ECPIX-5 | DCU |

The default build directory is `build/<platform>`, after the LiteX-Boards platform of the board:
`build/sqrl_acorn`, `build/xilinx_kc705`, `build/alibaba_xcku3p`,
`build/sipeed_tang_mega_138k_pro` and `build/lambdaconcept_ecpix5`.

### Common options {#opt-common}

The options below are shared by all the targets (`usb3_soc_args` in `bench/usb3_soc.py`).

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--build` | off | Build the bitstream with the FPGA toolchain |
| `--load` | off | Load the `.bit` bitstream into the FPGA (volatile); not for the Gowin target |
| `--output-dir` | per target | Build directory |
| `--no-uartbone` | off | No UART bridge (targets with a UART only) |
| `--with-jtagbone` | off | JTAG bridge (always on for the XCKU3P target, which has no UART) |
| `--cpu-type` | none | CPU, for the `console` and `composite` examples (`vexriscv`) |
| `--no-core` | off | PHY only, without the device core |
| `--example` | `loopback` | Device design: `loopback`, `wishbone`, `bench`, `cdc_uartbone`, `console`, `uvc`, `uvc_iso`, `msc`, `msc_dram`, `composite` |
| `--with-lpm` | off | U1 and U2 link power management (vendor Bulk examples) |
| `--lean` | off | No PHY debug, throughput profile or traces (timing margin) |
| `--max-burst` | 8 | Burst size of the Bulk endpoints: 2, 4, 8 or 16 (vendor Bulk examples) |
| `--link-words` | 1 | Link layer words: 1 (32-bit), 2 (64-bit) or 4 (128-bit, Gen 2 with wide transceiver words) |
| `--with-serdes-debug` | off | Transceiver debug core: controls, clock and receive measurements, DRP |
| `--with-analyzer` | off | LiteScope analyzer |
| `--analyzer-group` | `link` | Probe group: `link`, `lfps`, `ts`, `datapath`, `phase`, `u3` |

The vendor Bulk examples are `loopback`, `wishbone` and `bench`. Link words above 1 apply to them
and to `cdc_uartbone`, `console`, `msc` and `msc_dram`; the `uvc`, `uvc_iso` and `composite`
examples need 32-bit link words. The `phase` and `u3` groups need `--with-serdes-debug`.

The Tang Mega 138K Pro and ECPIX-5 targets change some of these defaults when their PHY datapath
has its own clock domain, as [Target scripts](#boards-targets) and the board sections explain. The
[Examples](#examples) chapter describes each example and [Debugging](#debugging) the debug
options.

### Acorn options {#opt-acorn}

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--variant` | `mini` | Baseboard variant: `mini` or `standard` |
| `--with-dram` | off | DDR3 memory, initialized by the BIOS; needs a CPU, implied by `msc_dram` |
| `--rxcdr-cfg` | none | GTP clock recovery setting override (hexadecimal) |
| `--gtp-param` | none | GTP parameter override `NAME=VALUE`, repeatable |
| `--lpm-hold` | `none` | Hold the GTP equalizer adaptation: `none`, `rxeq` (adapt only in Polling.RxEQ), `always` |

The `msc_dram` example is only available on the Acorn, the only target with a DRAM controller.

### KC705 options {#opt-kc705}

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--connector` | `sfp` | Transceiver connector: `sfp` (SFP2USB adapter) or `sma` |

### XCKU3P options {#opt-xcku3p}

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--sfp` | 0 | SFP cage of the SFP2USB adapter (0 is NIC1) |
| `--refclk` | `sfp` | GTY reference clock: the SFP clock (156.25 MHz) or the 100 MHz oscillator (`clk100`) |
| `--gen2-capable` | off | Gen 2 capable clocking: 10 Gbps PLL, 40-bit words, Gen 1 at half rate |

### Tang Mega 138K Pro options {#opt-tang}

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--sys-clk-freq` | 62.5 MHz | System clock; 125 MHz puts the PHY, the core and the SoC in one domain |
| `--sfp` | 0 | SFP cage of the SFP2USB adapter |
| `--refclk` | `100M` | GTR12 reference clock: 100 MHz (REFCLK1) or 125 MHz (REFCLK0) |
| `--pll` | QPLL1 | GTR12 lane PLL: `QPLL1` or `CPLL` (125 MHz REFCLK0, needed for 10 Gbps) |
| `--serdes-csr` | none | GTR12 register configuration file override (bring-up) |
| `--rx-idle-source` | `~astat5` | Receive electrical idle indicator of the GTR12 |
| `--gen2-capable` | off | Gen 2 capable transceiver and PHY (rate change, 80-bit words) |
| `--with-ltssm-history` | off | Gen 2 bring-up probes: equalizer and transmitter controls, LGOOD latency |
| `--with-tx-check` | off | Gen 2 transmit self-check of the header packet and link command CRCs |
| `--with-tx-raw-check` | off | Same check on the scrambled transmit blocks |
| `--with-trace` | off | Packet trace of the device core, with the split clocking |
| `--with-gen2-debug` | off | Gen 2 receive monitor on the raw words |
| `--with-gtr12-debug` | off | GTR12 and PIPE bring-up debug |
| `--with-gtr12-analyzer` | off | LiteScope on the raw GTR12 receive words |
| `--place-option` | 2 | Gowin place algorithm: 0, 1 or 2 |
| `--route-option` | 1 | Gowin route algorithm: 0, 1 or 2 |

With the default 62.5 MHz system clock, the link words are 2 at least and U1 and U2 are only built
with `--with-lpm`; the throughput profile is left out.

### ECPIX-5 options {#opt-ecpix5}

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--sys-clk-freq` | 83.33 MHz | System clock; 125 MHz puts the PHY, the core and the SoC in one domain |
| `--channel` | 0 | DCU0 channel of the USB-C lanes: 0 or 1 |
| `--usbc-pol` | `cc` | USB-C mux polarity: `cc` (cable direction read from the TUSB322), `0` or `1` |
| `--serdes-param` | none | DCU parameter override `NAME=VALUE`, repeatable |
| `--seed` | 1 | nextpnr seed |

The ECPIX-5 target always builds without the PHY debug core, the throughput profile and the
traces, as with `--lean`. With the default 83.33 MHz system clock, it forces 64-bit link words, and
`--max-burst` defaults to 16 on this target.

## Full-system simulation {#opt-sim}

`sim.py` builds and runs the Verilator simulation of [Simulation and verification](#simulation).
Numbers are in cycles of the 133 MHz simulation clock unless stated.

| Option | Default | Effect |
|------------------------------------------------|--------------------|----------------------------------------------------|
| `--trace` | off | Write waveforms |
| `--trace-start` | 0 | First traced cycle |
| `--trace-end` | -1 | Last traced cycle (-1: end of the simulation) |
| `--timer-scale` | 1 | Divide the LTSSM and LFPS timers (1: as the specification) |
| `--no-core` | off | PRBS stand-ins replace the device core, data checked on both sides |
| `--prbs-words` | 2^16 | Words checked with `--no-core` (2^14 with two lanes) |
| `--rx-word-shift` | 0 | Receive word shift of the transceiver models (0 to 3 symbols) |
| `--ppm` | 0 | Device clock offset from the host clock (ppm) |
| `--ssc` | off | Spread spectrum clocking on the device clock (0 to -5000 ppm) |
| `--max-cycles` | 2^24 | Simulation timeout |
| `--no-host-skp` | off | No SKP Ordered Sets from the host model |
| `--host-latency` | 256 | Host delay before the Data stage requests |
| `--host-status-gap` | 64 | Host gap between the Data stage ACK and the Status stage |
| `--host-pm` | off | Host U1 and U2 requests after the enumeration |
| `--host-lpm` | off | Device core with LPM, host U1 and U2 entries and exits |
| `--host-itp` | off | Host isochronous timestamp packets between the transfers |
| `--host-recovery` | off | Host-directed Recovery from U0 after the script |
| `--host-hot-reset` | 0 | Host-directed Hot Resets from U0 after the script, each at the next U0 entry |
| `--host-recovery-mid` | off | Recovery between Bulk OUT data packets, then Bulk OUT and IN |
| `--host-recovery-cut` | off | With the previous option, the Recovery cuts a data packet |
| `--host-burst-out` | 0 | Rounds of four back-to-back Bulk OUT data packets |
| `--host-burst-len` | 64 | Payload of the burst data packets (bytes) |
| `--host-burst-stream` | 0 | Bulk OUT data packets sent as the device credits allow |
| `--host-burst-duplex` | 0 | Rounds of Bulk OUT streamed during Bulk IN (full duplex) |
| `--host-burst-duplex-skp` | off | Full duplex with ACKs behind each OUT packet and SKPs between packets |
| `--host-lbad-test` | off | Bulk OUT burst with a header CRC error: LBAD, LRTY, retransmission |
| `--lanes` | 1 | 1 (Gen 1x1) or 2 (Gen 1x2, PRBS stand-ins) |
| `--lane0-delay` | 0 | Host to device delay of lane 0 (words, two lanes) |
| `--lane1-delay` | 1 | Host to device delay of lane 1 (words, two lanes) |
| `--lane1-word-shift` | 2 | Receive word shift of device lane 1 (symbols, two lanes) |
| `--lane-skp` | 4096 | Period of the extra SKPs on one lane (0: none, two lanes) |
| `--gen2-capable` | off | Gen 2 advertised and negotiated in the SuperSpeedPlus polling |
| `--gen2` | off | Training to U0 at Gen 2 on raw words (implies the previous option) |
| `--gen2-data-width` | 40 | Gen 2 raw word width of the transceiver models (80 as the GTR12) |
| `--link-words` | 1 | Device link layer words: 1, 2 or 4 (4 needs `--dev-pipe-cd`, `--gen2` and `--gen2-data-width 80`) |
| `--dev-pipe-cd` | off | Device LTSSM, link layer and core at half the PIPE frequency |
| `--dev-link-ppm` | none | Device link clock from its own generator with this offset |
| `--dev-loopback-depth` | 512 | Depth of the device Bulk loopback buffer (words) |
| `--dev-no-u1u2` | off | Device PHY without U1 and U2 |
| `--dev-ss` | off | With `--gen2-capable`: SuperSpeed device, SuperSpeed fallback of the host |
| `--output-dir` | `build/sim` | Build directory |

## Standalone core generator {#opt-gen}

`usb3_pipe_gen` (installed with the project, `usb3_pipe/gen.py`) generates the PHY, and optionally
the device core, as a Verilog core for a non-LiteX design. Its configuration is a YAML file,
described in [Integration](#integration).

| Option | Default | Effect |
|------------------------------------|------------------------------|--------------------------------------------|
| `config` | required | YAML configuration file |
| `--name` | `usb3_pipe_core` | Name of the core and of its Verilog module |
| `--output-dir` | `build` | Output directory |

The other options are the standard LiteX builder options, such as `--no-compile` and `--csr-csv`;
only `--output-dir` and `--name` matter for the generator.
