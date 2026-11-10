```{=latex}
\part{Use it}
```

```{=html}
<h1 class="part">Part IV. Use it</h1>
```

# Supported FPGAs and boards {#boards}

This chapter lists the FPGA families and the boards the project runs on, with the measured results
of each one. It also explains how a board connects to a USB 3 host and what a custom board needs.
Read it before choosing hardware or porting the project to a new board.

## At a glance {#boards-glance}

The project needs one FPGA transceiver lane per direction pair, a reference clock for it and a
USB 3 connector wired to the lane. Each board has a target script in `bench/` that builds every
example design of [Examples](#examples).

| Board | FPGA | Transceiver | Speed | Status |
|----------------------|----------------------------|---------------|---------------|--------------|
| Acorn CLE-215+ | Artix-7 XC7A200T | GTP | Gen 1 | Validated |
| KC705 | Kintex-7 XC7K325T | GTX | Gen 1 | Build only |
| Alibaba XCKU3P | Kintex UltraScale+ XCKU3P | GTY | Gen 1, Gen 2 | Validated |
| Tang Mega 138K Pro | Gowin GW5AST-138B | GTR12 | Gen 1 | Validated |
| ECPIX-5 | Lattice LFE5UM5G-85F | DCU | Gen 1 | Validated |
| GW5AT-15 device | Gowin GW5AT-15 | GTR12 | Gen 1 | Validated |

The second table gives the connection, the toolchain and the target script of each board.

| Board | Connection | Toolchain | Target |
|--------------------|---------------|----------------|-------------------------------------------|
| Acorn CLE-215+ | SFP2USB | Vivado | `acorn` |
| KC705 | SFP2USB, SMA | Vivado | `kc705` |
| Alibaba XCKU3P | SFP2USB | Vivado | `alibaba_xcku3p` |
| Tang Mega 138K Pro | SFP2USB | Gowin IDE | `sipeed_tang_mega_138k_pro` |
| ECPIX-5 | USB-C | Yosys, nextpnr | `lambdaconcept_ecpix5` |
| GW5AT-15 device | USB-C | Gowin IDE | none |

"Validated" means the device enumerates on a Linux xHCI host and moves data checked on both sides
without errors; the numbers are in each board section and in [Performance](#performance). Gen 2
(10 Gbps) is validated on the Kintex UltraScale+ GTY only.

## Target scripts {#boards-targets}

A target is a LiteX SoC without CPU built on `bench/usb3_soc.py`. The shared part provides a
bridge from the host to the register bus and `add_usb3_device`, which adds the PHY, the debug
registers and one example design. The target file itself only gives the clock generator of the
board, the transceiver pads and the transceiver wrapper.

| Board | Bridge | Loading |
|------------------------|-----------------------------------|------------------------------------|
| Acorn CLE-215+ | UARTBone, or JTAGBone (FT4232H) | `--load`, JTAG |
| KC705 | UARTBone, or JTAGBone | `--load`, JTAG |
| Alibaba XCKU3P | JTAGBone only | `--load`, Digilent JTAG-HS2 |
| Tang Mega 138K Pro | UARTBone, or JTAGBone | openFPGALoader, `.fs` bitstream |
| ECPIX-5 | UARTBone, or JTAGBone | `--load` |

`--with-jtagbone` adds the JTAG bridge and `--no-uartbone` removes the UART one. The build
directory, `build/<platform>` as listed in [Bench targets](#opt-targets), holds `csr.csv`:
the register map that the host scripts take with `--csr-csv`. Loading
is volatile: a power cycle brings the board back to the bitstream of its flash.

Every device enumerates as `1d50:605a`. The host scripts that read registers connect to a
`litex_server` on the bridge of the board, on the LiteX port 1234 unless `--port` says otherwise;
`usb3_bulk.py` only needs libusb. [Build options](#build-options) lists every option of the
targets, [Getting started](#gs-other-boards) the commands of each board.

## How a board connects to the host {#boards-connection}

The figure below shows the chain every board shares. The host port drives one differential pair
towards the device and receives another one. A connector, or an adapter plugged in a cage, routes
both pairs to one transceiver lane of the FPGA. An oscillator feeds the reference clock of the
transceiver; the PHY and the device core run in the fabric.

![The connection of a board to a USB 3 host. The cyan path is the cable: both differential
pairs end on one transceiver lane, clocked by an oscillator of the board.](board_connection.svg)

No USB chip sits on the path. The transceiver serializes and deserializes the bits, and everything
else, from the low frequency signalling to the device classes, is FPGA logic. Only the SuperSpeed
pairs are used: the USB 2 pair of the connector stays unconnected, so the device only appears on a
SuperSpeed host port.

## Adapters {#boards-adapters}

Most FPGA boards have no USB 3 connector on a transceiver lane, but many have an SFP cage. The
SFP2USB adapter of the XillyUSB project plugs into an SFP cage and exposes a USB 3 Micro-B
connector; the photo below shows it next to the SFP cage of the LiteX Acorn Baseboard. It is the
adapter used on the Acorn, KC705, XCKU3P and Tang Mega 138K Pro boards.

![The SFP2USB adapter and the SFP cage of the LiteX Acorn Baseboard.](acorn_baseboard_sfp2usb_2.jpg){width=60%}

The PCIsh-to-USB3.0 breakout board was used for the first developments of the project. It carries
a USB 3 connector on a PCIe-like edge, for boards with transceivers routed to a PCIe slot; its
schematic is `doc/breakout_board.pdf` and its gerber files `doc/breakout_board_gerbers.zip`.

![The PCIsh-to-USB3.0 breakout board.](breakout_board.jpg){width=50%}

## Acorn CLE-215+ {#boards-acorn}

The Acorn CLE-215+ is an Artix-7 board on an M.2 form factor. Mounted on the LiteX Acorn
Baseboard, it is the main development and validation board of the project. The SFP2USB adapter
sits in the SFP cage of the baseboard.

![The LiteX Acorn Baseboard with the Acorn CLE-215+ and the SFP2USB adapter.](acorn_baseboard_sfp2usb_0.jpg){width=70%}

| Item | Value |
|---------------------|------------------------------------------------------------|
| FPGA | XC7A200T, GTP transceiver (`A7USB3SerDes`) |
| Reference clock | 200 MHz oscillator through the fabric (GTGREFCLK0) |
| System clock | 125 MHz |
| Connection | SFP2USB in the baseboard SFP cage |
| Target | `bench/acorn.py`, build directory `build/sqrl_acorn` |
| Options | `--variant mini` or `standard`, `--with-dram` |
| JTAG | FT4232H of the baseboard |

The Acorn has no transceiver reference clock without a PCIe host: its dedicated clock input comes
from the M.2 edge connector. The target therefore feeds the 200 MHz oscillator to the GTP through
the fabric clock network. [Transceivers](#transceivers) explains why the GTGREFCLK0 input gives a
clean link and the general routing does not.

Measured results on a Linux xHCI host:

| Test | Result |
|-------------------|-----------------------------------------------------------------|
| Enumeration | SuperSpeed (5000M) |
| Bulk IN, OUT | 446 MB/s, 449 MB/s (about 90 % of the 8b/10b payload rate) |
| Full duplex | 376 MB/s IN plus 400 MB/s OUT |
| Stability | 139 GB full duplex, data checked, 0 errors, no retry, no Recovery |
| Suspend, resume | 72 of 72 clean resumes from U3, about 0.6 s each |

The first three user LEDs show the transceiver ready, U0 and Recovery states. The `--variant`
option selects the pinout of the SFP lane for the Mini or the standard baseboard. The DDR3
memory is only enabled for the `msc_dram` example or with `--with-dram`; the BIOS initializes it,
so a VexRiscv CPU is added when `--cpu-type` is not given.

The GTP bring-up overrides (`--rxcdr-cfg`, `--gtp-param`, `--lpm-hold`) are listed in
[Acorn options](#opt-acorn). With several FTDI adapters connected, select the Acorn explicitly, as
[Setup](#debug-setup) shows: the programmer picks the first FTDI adapter it finds.

## KC705 {#boards-kc705}

The KC705 is the Kintex-7 evaluation board of AMD. Its target uses the GTX transceiver of quad 117
with the 125 MHz SGMII oscillator as reference clock, dedicated to that quad.

| Item | Value |
|---------------------|------------------------------------------------------------|
| FPGA | XC7K325T, GTX transceiver (`K7USB3SerDes`) |
| Reference clock | 125 MHz SGMII oscillator (dedicated GTX clock input) |
| Connection | SFP2USB in the SFP cage, or the SMA connectors |
| Target | `bench/kc705.py`, `--connector sfp` or `sma` |
| Build directory | `build/xilinx_kc705` |

The target builds and meets timing, but it is not validated on hardware; the test suite
elaborates it with every example. It has the same bridges as the Acorn and no DRAM controller, so
the `msc_dram` example is not available.

## Alibaba XCKU3P {#boards-xcku3p}

The Alibaba Cloud XCKU3P board is a low cost Kintex UltraScale+ network card with two SFP cages.
Its GTY transceivers reach 10 Gbps: it is the Gen 2 development board of the project.

| Item | Value |
|---------------------|------------------------------------------------------------|
| FPGA | XCKU3P, GTY transceiver (`USPUSB3SerDes`) |
| Reference clock | SFP MGT clock, 156.25 MHz (default), or the 100 MHz oscillator |
| Connection | SFP2USB in the NIC1 cage (`--sfp 0`, default) or `--sfp 1` |
| Target | `bench/alibaba_xcku3p.py`, build directory `build/alibaba_xcku3p` |
| Gen 2 | `--gen2-capable`: QPLL at 10 Gbps, Gen 1 at half rate |
| JTAG | Digilent JTAG-HS2; the board has no UART: JTAGBone only |

The board has no UART, so the bridge is JTAGBone. The Digilent JTAG-HS2 loads the bitstream
(`openFPGALoader -c digilent_hs2`), and `litex_server` reaches JTAGBone through OpenOCD with
`cpld/xilinx-xcu.cfg` and `CHIP XCKU3P`. The board has no DRAM controller: no `msc_dram` example.

The receive pair of this board has an inverted polarity, which the PHY detects and corrects during
link training. With `--gen2-capable`, the device advertises Gen 2 in the SuperSpeedPlus polling:
on a USB 3.2 Gen 2 host port the link trains at 10 Gbps, on a Gen 1 port at 5 Gbps. `lsusb -t`
shows `10000M` for a Gen 2 link.

Gen 2 builds also carry bring-up registers, described in [Gen 2](#debug-gen2). Their names start
with `main_` in `csr.csv`, since they belong to the SoC and not to a submodule. Clearing
`usb3_gen2_capable` and retraining gives Gen 1 operation with the same bitstream.

| Test | Gen 1 | Gen 2, 32-bit link | Gen 2, 64-bit link |
|-------------|----------------------|------------------------|----------------------------|
| Enumeration | SuperSpeed (5000M) | SuperSpeedPlus (10000M) | SuperSpeedPlus (10000M) |
| Bulk IN | 446 MB/s | 426 MB/s | 744 MB/s, 785 MB/s at burst 16 |
| Bulk OUT | 450 MB/s | 466 MB/s | 881 MB/s |
| Full duplex | checked, 0 errors | 325 plus 430 MB/s | 619 plus 736 MB/s |

The 64-bit link is the `--link-words 2` option (see [Integration](#integration)). A 156 GB full
duplex run on the 64-bit link ended with 0 errors, no LBAD and no Recovery. The Gen 2 numbers are
measured with 64 KiB transfers: with the 1 MiB default of `usb3_bench.py`, the host itself limits
Bulk IN to about 330 MB/s.

## Tang Mega 138K Pro {#boards-tang}

The Sipeed Tang Mega 138K Pro carries a Gowin GW5AST-138B on a dock with two SFP cages. The GW5A
fabric is about half as fast per logic level as the 7-Series fabric, so this board runs the PHY
datapath and the rest of the design at two different clock rates.

| Item | Value |
|---------------------|------------------------------------------------------------|
| FPGA | GW5AST-138B, GTR12 transceiver (`GW5USB3SerDes`), quad Q1 |
| Reference clock | 100 MHz REFCLK1 from the dock (default) or 125 MHz REFCLK0 |
| Clocks | PHY datapath at 125 MHz, link layer and device core at 62.5 MHz |
| Connection | SFP2USB in SFP0 (lane 0, default) or SFP1 (`--sfp 1`) |
| Target | `bench/sipeed_tang_mega_138k_pro.py` |
| Toolchain | Gowin IDE; LiteICLink with GTR12 support |

The PHY datapath runs on the GTR12 transmit clock; the link layer and device core use 64-bit
words. [Choosing the datapath](#integ-choices) describes this split, the `pipe_cd` option of the PHY.
The common `--load` option expects a `.bit` file and does not apply here: program the `.fs`
bitstream with openFPGALoader, as [Other boards](#gs-other-boards) shows.

The target needs the Gowin IDE and LiteICLink with GTR12 support. The GTR12 register
configurations of `usb3_pipe/gw5` were generated by the `serdes_toml_to_csr` tool of the Gowin
IDE 1.9.12. The clocking changes a few defaults of the shared options:

| Item | Default, `sys` at 62.5 MHz | `sys` at 125 MHz |
|-----------------------|--------------------------------------|--------------------------------|
| Clock domains | PHY datapath at 125 MHz, the rest at 62.5 MHz | All in `sys` |
| Link words | 2 at least; 4 with `--link-words 4` | 1 or 2 |
| U1 and U2 | Only with `--with-lpm` | Always in the LTSSM |
| Packet trace | Only with `--with-trace` | Unless `--lean` |
| Throughput profile | Not built | Unless `--lean` |
| PHY debug | Unless `--lean` | Unless `--lean` |

With `--lean` and the split clocking, a minimal `usb3_phy_status` register keeps the LTSSM state,
U0 and the transceiver ready. The UVC, Isochronous and composite examples need 32-bit link words,
so the code only accepts them with `--sys-clk-freq 125e6`. The bring-up options of
[Tang Mega 138K Pro options](#opt-tang) add the registers of [Gowin GTR12 bring-up](#debug-gtr12).

| Test | Result |
|-------------------|-----------------------------------------------------------------|
| Enumeration | SuperSpeed (5000M) |
| Bulk IN, OUT | 372 MB/s, 433 MB/s |
| Full duplex | 262 MB/s IN plus 383 MB/s OUT |
| Stability | 20 s runs, data checked, 0 errors, no LBAD, no Recovery |

Bulk IN reaches about 400 MB/s with a burst of 16 packets.

### Gen 2 bring-up {#boards-tang-gen2}

Gen 2 on this board is in bring-up. The 10 Gbps rate on the Q1 lanes needs the CPLL with the
125 MHz REFCLK0; the QPLL1 configuration gives bit errors at that rate.

```sh
./bench/sipeed_tang_mega_138k_pro.py --build --example bench \
    --gen2-capable --pll CPLL --refclk 125M --link-words 4 \
    --with-gen2-debug --with-tx-check
```

Once Gen 2 is negotiated on a USB 3.2 Gen 2 host port, the wrapper switches the lane rate through
the runtime register access (`GW5RateSequencer`). The transceiver then delivers 80-bit raw words at
125 MHz, and the Gen 2 framing processes 16 bytes per cycle.

The 128-bit link words of `--link-words 4` are needed for the full rate: with 64-bit words at
62.5 MHz, the credits limit Gen 2 to about 500 MB/s. The 80-bit datapaths do not meet the 125 MHz
timing on the GW5A fabric yet, so the hardware throughput remains to be measured.

## ECPIX-5 {#boards-ecpix5}

The LambdaConcept ECPIX-5 carries a Lattice ECP5-5G with a USB-C connector wired to the transceiver
(DCU) through a SuperSpeed multiplexer. It is the board of the project built with a fully open
source toolchain.

| Item | Value |
|---------------------|------------------------------------------------------------|
| FPGA | LFE5UM5G-85F, DCU0 (`ECP5USB3SerDes`) |
| Reference clock | 250 MHz from a fabric PLL (the 100 MHz oscillator times 2.5) |
| Clocks | PHY datapath at 125 MHz, link layer and device core at 83.33 MHz |
| Connection | On-board USB-C, SuperSpeed mux, TUSB322 CC controller |
| Target | `bench/lambdaconcept_ecpix5.py`, `--seed` for nextpnr |
| Toolchain | Yosys, nextpnr-ecp5, Trellis; LiteICLink with 40-bit `SerDesECP5` |

A 5 Gbps line rate needs a 250 MHz reference clock, as the DCU multiplies it by 20. The 100 MHz
input of the board only reaches 2.5 Gbps, so a fabric PLL generates the 250 MHz clock. The DCU runs
a raw 10-bit channel: 8b/10b coding, comma alignment and polarity are fabric logic.

The PHY datapath runs on the DCU transmit clock at 125 MHz. The LTSSM, the link layer, the device
core and the SoC run at 83.33 MHz on 64-bit link words, the `pipe_cd` split of
[Choosing the datapath](#integ-choices). The target forces the 64-bit words and sets the burst of
the Bulk endpoints to 16 by default, for the Bulk IN rate.

The USB-C connector needs its multiplexer set to the cable orientation. The TUSB322 CC controller
reports the cable direction over I2C; the target polls it (`TUSB322Reader`) and drives the
multiplexer polarity. The `--usbc-pol 0` or `1` option forces a fixed polarity instead, and the
VBUS output of the connector stays off since the board is a device.

| Test | Result |
|-------------------|-----------------------------------------------------------------|
| Enumeration | SuperSpeed (5000M), direct USB-C cable |
| Bulk IN, OUT | 434 MB/s, 449 MB/s |
| Full duplex | 342 MB/s IN plus 349 MB/s OUT |
| Stability | 3 min full duplex (about 110 GB), 0 errors, no Recovery |

On nextpnr seeds 1 to 3, the system clock reaches 119 MHz to 128 MHz for 83.33 MHz, the PHY
datapath 156 MHz to 166 MHz and the receive clock 187 MHz to 196 MHz for 125 MHz. The DCU gearboxes
exceed 430 MHz for 250 MHz. The table below gives the other clockings measured on this board.

| Link layer and core clock | Bulk IN, OUT | Full duplex | Timing |
|-------------------------------------|----------------|----------------|-------------------------------|
| 125 MHz, 32-bit words, no `pipe_cd` | 440, 445 MB/s | 336 + 406 MB/s | Met on some seeds only |
| 100 MHz, burst 16 | 421, 448 MB/s | 372 + 306 MB/s | Fmax 24 % above the clock |
| 62.5 MHz, burst 16 | 403, 436 MB/s | 272 + 385 MB/s | Fmax about twice the clock |

`--sys-clk-freq 125e6` gives the first row: the whole design in `sys` on 32-bit words. The UVC,
Isochronous and composite examples need 32-bit link words, so the code only accepts them in that
mode.

The target always builds without the PHY debug core, the throughput profile and the traces. A small
link status block (`USB3LinkStatus`) replaces them: LTSSM state, U0 and Recovery entries, and
receive errors and idles in U0, as [PHY debug registers](#debug-phy) describes.

The script `bench/lambdaconcept_ecpix5_bringup.py` builds small designs that check and tune the
DCU in seconds of build time. They run with `sys` at 125 MHz.

| Stage | Design | Result |
|----------|--------------------------------------------------------|------------------------------------|
| `serdes` | DCU with PRBS, internal loopback, clock meters | 0 errors over 1.5e11 bits at 5 Gbps |
| `dcu` | 40-bit path in loopback, decode and sequence checks | 0 errors over 1.3e10 symbols |
| `lfps` | LFPS unit only, no LTSSM: host LFPS and TSEQ monitor | Host bursts of 1.11 µs every 9.43 µs |

The `lfps` stage never trains the link, so it is safe for the host. Its receive monitor gave the
DCU receive settings of the wrapper.

## GW5AT-15 {#boards-gw5at}

The GW5AT-15 is a small Gowin device with a single GTR12 quad. The PHY supports it with lane 0 of
quad Q0, a 125 MHz reference clock on REFCLK0 and hardware-validated QPLL0 settings;
`GW5USB3SerDes` selects them from the device name. The register configuration is in
`usb3_pipe/gw5`, next to the GW5AST-138B ones.

It was validated at Gen 1 on a GW5AT-15 device with a direct USB-C connection. Bulk IN reaches
about 400 MB/s with a burst of 16 packets, with the link layer and device core at 62.5 MHz on
64-bit words. The repository has no target script for it: start from the Tang Mega 138K Pro
target.

## Requirements for a custom board {#boards-custom}

Any board with a transceiver lane routed to a USB 3 connector, an SFP cage or a PCIe-like edge
can host the project. The checklist below lists what the hardware must provide; [Integration](#integration)
covers the gateware side and [Transceivers](#transceivers) the transceiver settings.

- **Transceiver**: a lane rated for 5 Gbps (Gen 1) or 10 Gbps (Gen 2), of a supported family:
  Artix-7 GTP, Kintex-7 GTX, UltraScale+ GTY, Gowin GTR12 or ECP5-5G DCU.
- **Reference clock**: a clean oscillator on a dedicated transceiver clock input. A fabric clock
  works on some families (the Acorn and the ECPIX-5 use one) but costs receive margin.
- **Clock accuracy**: the host and the device clocks must each stay within the USB 3 tolerance of
  300 ppm; the PHY absorbs the difference and the host spread spectrum clocking.
- **Wiring**: the device transmit pair goes to the connector receive pins, through the AC coupling
  capacitors the USB 3 specification places on the transmitter side (75 nF to 265 nF); the receive
  pair comes from the connector.
- **Polarity**: an inverted pair is allowed on receive, the PHY detects it; check the transmit
  pair polarity against the schematic.
- **USB-C**: a SuperSpeed multiplexer driven from the cable orientation, read from the CC lines
  (a CC controller such as the TUSB322 of the ECPIX-5), and VBUS left off on a device.
- **Debug access**: a JTAG or UART link to the SoC bus helps a lot during bring-up, see
  [Debugging](#debugging).

Where to go next: [Getting started](#getting-started) to build a first bitstream, and
[Integration](#integration) to add the project to your own design.
