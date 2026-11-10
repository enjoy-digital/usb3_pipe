# Transceivers {#transceivers}

The PHY has no analog part of its own: it drives the high-speed serial transceivers of the FPGA.
This chapter describes the transceiver wrapper that adapts each transceiver to USB 3, then each
supported FPGA family, the clocking and the board connection. It ends with a checklist for porting
the PHY to a new transceiver.

## At a glance {#gt-glance}

| Family | Block | Class | Gen 1 | Gen 2 | Status |
|----------------|-------|--------------------------|-----|-----------|-------------------|
| Artix-7 | GTP | `A7USB3SerDes` | Yes | No | Hardware |
| Kintex-7 | GTX | `K7USB3SerDes` | Yes | Not yet | Build tested |
| Kintex UltraScale+ | GTY | `USPUSB3SerDes` | Yes | Yes | Hardware, Gen 1 and Gen 2 |
| Gowin GW5AST | GTR12 | `GW5USB3SerDes` | Yes | In progress | Hardware, Gen 1 |
| Gowin GW5AT | GTR12 | `GW5USB3SerDes` | Yes | No | Hardware, Gen 1 |
| Lattice ECP5-5G | DCU | `ECP5USB3SerDes` | Yes | No | Hardware, Gen 1 |

The Artix-7 GTP tops out at 6.6 Gbps, which rules out Gen 2 on that family. The boards behind the
"Hardware" entries and their measured throughputs are in [Supported boards](#boards).

## The transceiver wrapper {#gt-wrapper}

Every family has a wrapper class derived from `USB3SerDes` (`usb3_pipe/serdes.py`). The wrapper
gives the PIPE one interface whatever the FPGA, and adds to the transceiver what USB 3 needs and
the vendor primitive lacks. The figure below shows the split.

![The transceiver wrapper: the PIPE sees 32-bit words and a few LFPS signals; the wrapper adds the
datapaths, the electrical idle and LFPS handling, and the USB 3 settings of the
transceiver.](gt_interface.svg)

The wrapper interface, used by the PIPE, the LTSSM and the simulation models alike:

| Signal | Direction | Role |
|-------------------------|-----------|---------------------------------------------------|
| `sink`, `source` | Data | 32-bit words and 4-bit K flags, `sys` clock domain |
| `enable`, `ready` | Control | Transceiver enable, TX and RX ready |
| `tx_polarity`, `rx_polarity` | Control | Pair polarity inversion |
| `tx_idle` | To wrapper | TX electrical idle |
| `tx_pattern`, `tx_lfps` | To wrapper | LFPS: send a raw pattern instead of data |
| `rx_idle` | From wrapper | RX electrical idle, filtered |
| `rx_errors` | From wrapper | Decode errors received (Warm Reset detection) |
| `tx_skp_enable`, `rx_align` | To wrapper | SKP insertion and comma alignment enables |
| `rx_reset` | To wrapper | RX re-initialization (training failure, Warm Reset) |
| `rate_10g`, `gen2_*` | Gen 2 only | Line rate select and raw transceiver words |

The common part (`add_transceiver`, `add_xcvr_datapath`) instantiates the transceiver through
LiteICLink with its TX and RX elastic buffers enabled and its clock aligner off. It adds the TX and
RX datapaths of [Datapath](#datapath), and the period and false path timing constraints between
`sys` and the transceiver clocks.

The family classes then add three things: the PLL and reference clock, the electrical idle and
LFPS signals of the primitive, and the parameter overrides that USB 3 needs.

### USB 3 settings {#gt-settings}

A transceiver configured with the vendor defaults does not hold a USB 3 link. Two properties of
USB 3 explain the overrides:

- **Spread spectrum clocking.** The host sweeps its transmit clock between 0 and -5000 ppm at
  about 33 kHz ([The wire](#primer-wire)). The default clock recovery (CDR) settings of most
  transceivers are tuned for a much smaller offset and lose track; each wrapper loads SSC-tolerant
  CDR settings, taken from the vendor's settings for protocols with SSC (SATA, PCIe).
- **Electrical idle and LFPS.** USB 3 turns the line off between LFPS bursts and in U1 to U3. The
  CDR must keep its state while the line is idle (CDR hold during electrical idle), and the
  out-of-band (OOB) detector of the transceiver must be powered and clocked.

The transmit swing is set for USB 3 as well (`TXDIFFCTRL` on the Xilinx parts). The source
comments of each class cite the vendor document and table each value comes from.

### Electrical idle and LFPS {#gt-lfps}

A transceiver has no LFPS generator or detector; the wrappers build them from what the primitive
offers. The signaling itself is described in [LFPS](#lt-lfps).

**Transmit.** During a burst the wrapper sends constant words through the transceiver, all ones
then all zeros: the toggling pattern is the LFPS square wave. Between bursts it sets the
transceiver's TX electrical idle input. The Gowin GTR12 differs: see its section below.

**Receive.** The OOB detector of the transceiver reports when the line is idle. Its output toggles
during a burst, so the wrapper reports idle only after ten consecutive idle cycles
(`RXIdleFilter`). A Warm Reset is a long LFPS burst, which the receiver must tell from data.
During a burst the 8b/10b decoder reports errors on every word, which a trained link never does,
so the wrapper reports them on `rx_errors`.

Some idle detectors also produce short non-idle spikes. On some bitstream loads of an Artix-7 GTP
board, a 32 ns spike followed each Polling.LFPS burst of the host. The LFPS checkers took it for a
Ping.LFPS and the link never trained.

The Artix-7 wrapper can reject such spikes: with `rx_idle_min_burst`, the end of electrical idle
is reported only after the line has been active for that time. Keep the value at or below 40 ns,
the shortest LFPS burst. The option is off by default.

## Clocking {#gt-clocking}

The figure below shows the clocks around a transceiver. A reference clock feeds the transceiver
PLL, which sets the line rate. The transmit side runs from the PLL (the `tx` clock domain); the
receive side runs from the clock that the CDR recovers from the partner's data (the `rx` clock
domain).

![Transceiver clocking: the reference clock and PLL set the line rate; the tx and rx clock domains
meet the sys clock domain through the CDC FIFOs of the datapaths.](gt_clocking.svg)

The PIPE, the LTSSM and the link layer run in `sys`, at 125 MHz or more for 32-bit words. Each
datapath crosses between its transceiver clock and `sys` with an asynchronous FIFO, so `sys` does
not have to be derived from the transceiver.

| Family | Transceiver words | Transceiver clock | Reference clock used on the benches |
|-------------------|--------------------------|------------------|----------------------------------------|
| Artix-7 GTP | 20-bit | 250 MHz | 200 MHz oscillator, through the fabric |
| Kintex-7 GTX | 20-bit | 250 MHz | 125 MHz SGMII clock |
| Kintex UltraScale+ GTY | 20-bit | 250 MHz | 156.25 MHz SFP clock, or 100 MHz through the fabric |
| GTY, Gen 2 capable | 40-bit, or 40 raw bits | 125 MHz, 250 MHz at 10 Gbps | 156.25 MHz SFP clock |
| Gowin GTR12 | 40-bit, or 80 raw bits | 125 MHz | 100 MHz REFCLK1 or 125 MHz REFCLK0 |
| Lattice ECP5-5G DCU | 40-bit | 125 MHz | 250 MHz from a fabric PLL |

The Xilinx GTP and GTX wrappers also need an `oob` clock domain for the OOB detector. The bench
targets create it at `sys_clk_freq/8`. The GTY clocks its OOB detector from the transceiver and
does not need it.

### Reference clock from the fabric {#gt-fabric-refclk}

A board does not always route a clock to the transceiver's dedicated reference inputs. The Acorn
CLE-215+ is the example: its only transceiver reference input is the PCIe edge clock, which is
absent without a PCIe host. The wrappers accept a fabric clock instead (a `ClockSignal` as
`refclk_pads`).

How the fabric clock reaches the PLL matters. On the Artix-7, the wrapper drives it to the
`GTGREFCLK` input, which takes it through the clock network. The Xilinx documentation reserves
that input for testing, so the build lowers the corresponding design rule check to a warning. The
alternative, `GTREFCLK0` from general routing, gave a receive quality that changed from one place
and route to the next: some bitstreams never reached a stable U0.

If a design with a fabric reference clock has an unstable link, check in the generated Verilog
that the PLL uses `GTGREFCLK` (`PLL0REFCLKSEL` set to `3'd7`). [Debugging](#debugging) has the
procedure. The GTY wrapper also takes a fabric clock on `GTGREFCLK` for its channel PLL, at
Gen 1 only.

## Xilinx 7-Series and UltraScale+ {#gt-xilinx}

The three Xilinx wrappers use the LiteICLink transceivers with 8b/10b in the transceiver.

**Artix-7 GTP** (`A7USB3SerDes`). The quad PLL (`GTPQuadPLL`) runs at 5 Gbps. It can be shared
with another protocol of the same quad: pass an external PLL configured for 5 Gbps
(`pll=`), and the wrapper then leaves its reset to its owner. The CDR settings come from the SSC
table of the GTP user guide; the no-SSC settings saturate the frequency tracking against a host
with SSC. The class also exposes the comma realignment and elastic buffer error flags of the GTP
for debug.

**Kintex-7 GTX** (`K7USB3SerDes`). The channel PLL (`GTXChannelPLL`) runs at 5 Gbps. The CDR and
equalizer settings are those of the Xilinx SATA (with SSC) and PCIe Gen2 wrappers, in LPM mode.
The OOB detector takes its clock from the `oob` clock domain. The KC705 target builds; it has not
been validated on hardware.

**Kintex UltraScale+ GTY** (`USPUSB3SerDes`). Without Gen 2, the channel PLL runs at 5 Gbps with
20-bit words. With `gen2_capable=True`, the quad PLL runs at 10 Gbps with 40-bit words, and the
`rate_10g` input selects the rate:

- Gen 1 runs at half rate: 5 Gbps, 40-bit words with 8b/10b at 125 MHz.
- Gen 2 runs at full rate: 10 Gbps, 40 raw bits at 250 MHz, 8b/10b bypassed.
- A rate change re-initializes the receiver for about 1 µs, so the CDR restarts at the new rate.

The same CDR overrides track the host SSC at both rates. The reference clock may come from
dedicated pads or from the fabric. This is the transceiver used for Gen 2 on hardware.

## Gowin GW5AST and GW5AT {#gt-gowin}

The Gowin wrapper (`GW5USB3SerDes`) drives a GTR12 lane through the LiteICLink GTR12 support. It
covers two devices:

| Device | Quad and lane | Reference clock | Lane PLL |
|----------------|--------------------|--------------------------------|----------------------|
| GW5AST-138B | Q1, lane 0 or 1 | 100 MHz REFCLK1 or 125 MHz REFCLK0 | QPLL1, or CPLL |
| GW5AT-15 | Q0, lane 0 | 125 MHz REFCLK0 | QPLL0 |

The fabric sees 40-bit words at 125 MHz, four 10-bit symbols per cycle. The 8b/10b coding and the
polarity inversion are done in the fabric (LiteICLink); the comma alignment is part of the GTR12
register configuration.

### Register configuration {#gt-gowin-config}

The GTR12 is configured by register writes generated from a TOML description with Gowin's
`serdes_toml_to_csr` tool. The generated files are committed in `usb3_pipe/gw5/`, one per device
position, lane, rate and reference clock; `gen_toml.py` and `gen_toml_q0.py` document how to
regenerate them. The wrapper selects the right file (`gw5_usb3_csr_config`).

On the GW5AT-15, a few CDR and clock tree registers are replaced by hardware-validated values
(`GW5AT15_Q0_QPLL0_OVERRIDES`): with the generated values, decode errors stop the training. The
10 Gbps configuration of the GW5AST-138B uses the CPLL, with a manual receive equalizer at 10 Gbps.

### Runtime register access {#gt-gowin-upar}

Two USB 3 functions need register writes while the link runs, through the GTR12 runtime register
port (UPAR):

- **TX electrical idle.** The GTR12 has no TX electrical idle input usable from the fabric. The
  wrapper writes the lane's TX driver control register on each idle change
  (`GW5TXElectricalIdle`).
- **Rate change** (Gen 2). The registers that differ between 5 Gbps and 10 Gbps are written with
  the lane held in reset (`GW5RateSequencer`). The change must be short: the host starts its
  receiver equalization right after the speed negotiation.
- **Equalization at 10 Gbps** (Gen 2, bring-up). The TX FFE and RX CTLE values can be set at
  runtime and are applied at the next rate change ([Debugging](#debugging)).

### LFPS on the GTR12 {#gt-gowin-lfps}

Constant fabric words did not produce LFPS that the host received. The wrapper therefore sends a
bit-level square wave with a 24 ns period (41.7 MHz) through the raw TX words whenever the
transmitter is idle, and makes the bursts by switching the TX driver through UPAR. The RX idle
indicator is bit 5 of the lane's analog status, inverted (`rx_idle_source="~astat5"`).

### Clocking on the GW5A fabric {#gt-gowin-clocking}

The GW5A fabric does not run the link layer at 125 MHz. On the Tang Mega 138K Pro target, the PIPE
runs at 125 MHz on the GTR12 TX clock and the LTSSM, link layer and device core at 62.5 MHz on
64-bit words (`pipe_cd`, see [PHY architecture](#phy-architecture)). Running the PIPE on the TX
clock removes the TX clock domain crossing. The GTR12 control logic stays on a free-running clock
(`xcvr_cd`).

Gen 2 on the GW5AST-138B (80 raw bits at 125 MHz, 128-bit link words) works in simulation and is
in bring-up on the Tang Mega 138K Pro; timing closure of the 80-bit datapaths on the fabric is in
progress.

## Lattice ECP5-5G {#gt-ecp5}

The ECP5 wrapper (`ECP5USB3SerDes`) drives a DCU channel through the LiteICLink ECP5 support
(`SerDesECP5`), which needs its 40-bit words option. The DCU runs as a raw 10-bit interface;
8b/10b and comma alignment are in the fabric, and the polarity is set through the DCU control
interface (SCI).

The DCU clocks run at 250 MHz with 20-bit words, too fast for the 8b/10b and alignment logic on the
ECP5 fabric. Only the TX and RX gearboxes run at 250 MHz; everything else runs at 125 MHz on 40-bit
words, from the DCU clocks divided by two (`CLKDIVF`).

- **Reference clock.** The target uses 250 MHz from a fabric PLL, multiplied by 20. The 100 MHz
  `EXTREF0` input of the ECPIX-5 reaches only 2.5 Gbps, since the DCU multiplier stops at 25
  (`SerDesECP5PLL`).
- **Receive settings.** RX termination to ground, CDR and DCO calibration and lock window, and
  equalizer, from the LUNA project's ECP5 PHY (`rx_settings`). The CDR settings are the fix that
  let the receiver track the host SSC. The CDR phase detector stays enabled even on loss of signal.
- **Initialization.** TX and RX initialize separately, so the transmitter runs before any signal is
  received (LFPS needs it).
- **LFPS.** TX electrical idle through the DCU `FFC_EI_EN` input; RX idle from the DCU loss of
  signal detector.

As on the Gowin targets, the PIPE can run on the DCU TX clock with the LTSSM and the link layer
in a slower `sys` (`pipe_cd`). The wrapper then keeps the DCU control logic on a free-running
clock (`xcvr_cd`) and drops its TX clock domain crossing (`tx_cdc=False`).

The ECPIX-5 target uses this split by default: the PIPE at 125 MHz, the LTSSM, link layer and
device core at 83.33 MHz on 64-bit words. With `--sys-clk-freq 125e6`, everything runs at 125 MHz
on 32-bit words; nextpnr then meets the timing with some seeds only (`--seed`). The target has no
PHY debug registers and uses a small link status block instead (`USB3LinkStatus`).

### USB-C on the ECPIX-5 {#gt-ecpix5-usbc}

The ECPIX-5 connects the DCU to a USB-C connector through a SuperSpeed multiplexer. A USB-C plug
fits both ways up, and the multiplexer must route the active pairs according to the cable
orientation. The figure below shows how the target sets it.

![ECPIX-5 USB-C: the DCU lanes go through the SuperSpeed mux; the mux polarity follows the cable
direction read from the TUSB322.](gt_ecpix5_usbc.svg)

The TUSB322 CC controller detects the orientation. A small I2C reader in the target polls its
attached state and cable direction register every 10 ms and drives the multiplexer polarity
(`TUSB322Reader`); `--usbc-pol 0` or `1` forces it. The multiplexer enable is active high, and the
VBUS output stays off for a device.

## Board connection {#gt-board}

A USB 3 connector carries two differential pairs (TX and RX) for SuperSpeed, beside the USB 2
pair. The FPGA transceiver lanes connect to the SuperSpeed pairs; the PHY does not use USB 2.

| Connection | Used on | Notes |
|------------------------|----------------------------|----------------------------------------------|
| SFP2USB adapter | Acorn, KC705, XCKU3P, Tang Mega 138K Pro | SFP cage to a USB 3 Type-A cable, from the XillyUSB project |
| PCIsh-to-USB3.0 board | Early development | Breakout board |
| Direct USB-C | ECPIX-5, GW5AT-15 | Mux and CC controller for the orientation |

The specification requires AC coupling capacitors on each transmit pair. When designing a board,
place them on the TX pairs; with an adapter, check where the adapter or the board provides them.
A swapped receive pair is corrected by the PHY during training ([Polarity](#dp-polarity)); a
swapped transmit pair must be set at build time with `tx_polarity`.

## Porting to a new transceiver {#gt-porting}

A new transceiver family needs a `USB3SerDes` subclass. The checklist below follows the existing
classes; `A7USB3SerDes` is the shortest example and the simulation models in `test/model/gt.py`
show the minimal interface.

1. **Line rate and words.** 5 Gbps with 8b/10b, 20-bit (2 symbols) or 40-bit (4 symbols) words.
   Encoder and decoders reachable as `encoder` and `decoders` (LiteICLink interface).
2. **Datapaths.** Call `add_transceiver` or `add_xcvr_datapath`: SKP handling, clock domain
   crossings and alignment come with them. Allow comma alignment on any byte.
3. **Elastic buffers.** TX and RX buffers on; the SKP removal is done by the datapath, not by the
   transceiver.
4. **CDR.** Load the vendor's settings for a protocol with SSC, and hold the CDR during electrical
   idle.
5. **TX electrical idle.** Map `tx_idle` to the transceiver's idle control.
6. **LFPS transmit.** Map `tx_pattern` and `tx_lfps` to a raw pattern output, then check on a
   scope or with a host that the bursts are received (10 to 50 MHz, USB 3.2 table 6-29).
7. **RX idle.** Find a signal-detect or OOB output, pass it to `add_lfps`, and check that it
   follows the bursts of a host.
8. **Polarity and reset.** Connect `tx_polarity` and `rx_polarity`; make `rx_reset` restart the
   receiver (PMA, CDR and PCS).
9. **Constraints.** Declare the tx and rx clock periods and the false paths to `sys`
   (`add_timing_constraints`).
10. **Bring-up.** Validate the transceiver alone first (PRBS, near-end loopback), then LFPS with
    the link training disabled, then the full PHY. The ECP5 bring-up designs in
    `bench/lambdaconcept_ecpix5_bringup.py` follow this order.

A Gen 2 capable wrapper adds `gen2_capable`, the `rate_10g` rate change, and the raw words
`gen2_tx_data` and `gen2_rx_data` (40 or 80 bits, bit 0 first), as `USPUSB3SerDes` does.

## In the code {#gt-code}

| File | Content |
|-----------------------------------|-------------------------------------------------------|
| `usb3_pipe/serdes.py` | `USB3SerDes` and the Xilinx, Gowin and ECP5 wrappers |
| LiteICLink `serdes_ecp5.py` | ECP5 DCU channel, gearboxes, comma aligner, DCU PLL |
| `usb3_pipe/gw5/` | GTR12 register configurations and their generators |
| `test/model/gt.py` | Transceiver model used by the simulations |
| `test/test_serdes.py`, `test_ecp5.py`, `test_gw5.py` | Wrapper unit tests |

Where to go next: [Datapath](#datapath) for the paths inside the wrapper,
[Supported boards](#boards) for the targets that use each class, [Integration](#integration) to
instantiate a wrapper in a design.
