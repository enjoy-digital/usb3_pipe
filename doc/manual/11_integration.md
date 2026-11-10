# Integration {#integration}

This chapter explains how to add the project to your own design. It starts with a LiteX SoC, the
native way, then covers the choices that shape the datapath, the clocks and constraints, the
endpoint streams, and the standalone Verilog core for designs built without LiteX. It ends with a
checklist to port the project to a new board.

## At a glance {#integ-glance}

| Item | Value |
|-----------------------------|-------------------------------------------------------------|
| Language | Python (Migen, LiteX); Verilog through `usb3_pipe_gen` |
| PHY class | `USB3PHY` (`usb3_pipe/phy.py`) |
| Transceiver classes | `A7USB3SerDes`, `K7USB3SerDes`, `USPUSB3SerDes`, `GW5USB3SerDes`, `ECP5USB3SerDes` |
| Device core class | `USB3DeviceCore` (`usb3_core/device.py`) |
| System clock | 125 MHz, or 62.5 MHz to 83.33 MHz with a separate PHY clock domain |
| User interface | Stream endpoints, 32, 64 or 128-bit words, first byte on bits 7 to 0 |
| Reference design | `bench/usb3_soc.py` with the board targets of `bench/` |

## Integration in a LiteX SoC {#integ-litex}

A design is a chain of four parts, shown in the figure below. The transceiver wrapper drives one
lane of the FPGA transceiver; the PHY trains the link and turns the line into a stream of symbols;
the device core runs the link and protocol layers and the endpoints; user logic reads and writes
the endpoint streams. The clock generator provides the system clock and, on AMD devices, a slow
out-of-band clock for the transceiver signal detectors.

![A USB 3 device in a LiteX SoC. The cyan arrow is the link interface between the PHY and the
device core; user logic only sees the endpoint streams.](integ_soc.svg)

The example below is a complete Bulk device on an Artix-7 GTP. It assumes a board with a 125 MHz
reference clock on dedicated transceiver clock pins; the pad names (`clk200`, `gtp_refclk`,
`usb3_tx`, `usb3_rx`) stand for those of your platform. The `oob` clock domain at one eighth of
the system clock is required by the 7-Series wrappers.

```python
from migen import *

from litex.gen import *

from litex.soc.cores.clock          import S7PLL
from litex.soc.integration.soc_core import SoCMini

from usb3_pipe             import A7USB3SerDes
from usb3_pipe.phy         import USB3PHY
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()
        self.cd_oob = ClockDomain()
        self.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk200"), 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8)

class USB3Device(SoCMini):
    def __init__(self, platform, sys_clk_freq=125e6):
        self.crg = _CRG(platform, sys_clk_freq)
        SoCMini.__init__(self, platform, sys_clk_freq)

        # Transceiver wrapper (one GTP lane).
        self.usb3_serdes = A7USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = platform.request("gtp_refclk"),
            refclk_freq  = 125e6,
            tx_pads      = platform.request("usb3_tx"),
            rx_pads      = platform.request("usb3_rx"))

        # PHY: LTSSM, LFPS, coding, scrambling, framing.
        self.usb3_phy = USB3PHY(self.usb3_serdes, sys_clk_freq,
            role = "device")

        # Device core: vendor class, Bulk IN on EP1, Bulk OUT on EP2.
        self.usb3_core = USB3DeviceCore(self.usb3_phy, sys_clk_freq,
            descriptors = bulk_descriptors(vid=0x1d50, pid=0x605a, max_burst=8),
            endpoints   = [
                USB3BulkIN(ept=1,  max_burst=8),
                USB3BulkOUT(ept=2, max_burst=8),
            ])

        # User logic: Bulk OUT packets sent back on Bulk IN.
        ep_in, ep_out = self.usb3_core.endpoints
        self.comb += ep_out.source.connect(ep_in.sink)
```

The transceiver wrapper takes either differential pads, buffered by a dedicated clock input, or a
fabric clock signal as reference clock. The Acorn target uses the second form, with its 200 MHz
oscillator (see [Acorn CLE-215+](#boards-acorn)). The descriptors helper builds a vendor class
device; the other descriptor builders and the device classes are in [Device classes](#classes).

The bench targets go further with `add_usb3_device` of `bench/usb3_soc.py`. It adds the PHY debug
registers, the device core status registers (`USB3DeviceCore.add_csr`), the example designs of
`usb3_core/examples.py` and three status LEDs (transceiver ready, U0, Recovery). Copy that file as
the starting point of a new SoC; the closest target of [Target scripts](#boards-targets) is the
reference for the clocks and the pads.

### Transceiver wrappers {#integ-serdes}

Each transceiver family has its own wrapper class, a subclass of `USB3SerDes` in
`usb3_pipe/serdes.py`. The table below gives what each one needs from the design; the settings
inside the wrappers are in [Transceivers](#transceivers).

| Class | Transceiver | Line rate | Reference clock | Needs from the design |
|---------------------------|----------------|-----------|---------------------|--------------------------|
| `A7USB3SerDes` | Artix-7 GTP | 5 Gbps | Pads or fabric clock | An `oob` clock domain |
| `K7USB3SerDes` | Kintex-7 GTX | 5 Gbps | Pads, 125 MHz | An `oob` clock domain |
| `USPUSB3SerDes` | GTY | 5 or 10 Gbps | Pads (156.25 MHz) or fabric | `gen2_capable` for 10 Gbps |
| `GW5USB3SerDes` | Gowin GTR12 | 5 or 10 Gbps | REFCLK1 100 MHz, REFCLK0 125 MHz | `refclk`, `pll`, `upar_clk` |
| `ECP5USB3SerDes` | ECP5-5G DCU | 5 Gbps | Fabric clock, 250 MHz | A 250 MHz PLL output |

A fabric reference clock is passed as a clock signal (`refclk_pads=ClockSignal("refclk")`) and
reaches the GTP through its GTGREFCLK0 input. The GTY wrapper with `gen2_capable=True` runs its
QPLL at 10 Gbps on 40-bit raw words and Gen 1 at half rate. The Gowin and Lattice wrappers take
`xcvr_cd` and `tx_cdc` for the split clocking of [Choosing the datapath](#integ-choices).

The wrapper adds the period constraints of its transmit and receive clocks and their false paths
with `sys`. It is not a submodule of the PHY: add both to the SoC, as in the example above.

The Gen 2x1 version of the same SoC, as the XCKU3P target builds it with `--gen2-capable`, changes
the wrapper and two PHY parameters:

```python
from usb3_pipe     import USPUSB3SerDes
from usb3_pipe.phy import USB3PHY

self.usb3_serdes = USPUSB3SerDes(platform,
    sys_clk      = self.crg.cd_sys.clk,
    sys_clk_freq = sys_clk_freq,
    refclk_pads  = platform.request("sfp_mgt_clk"),
    refclk_freq  = 156.25e6,
    tx_pads      = platform.request("sfp_tx"),
    rx_pads      = platform.request("sfp_rx"),
    gen2_capable = True)
self.usb3_phy = USB3PHY(self.usb3_serdes, sys_clk_freq,
    role         = "device",
    gen2_capable = True, # Gen 2x1 advertised in the SuperSpeedPlus polling.
    link_words   = 2)    # 64-bit link interface, for the Gen 2 bandwidth.
```

The device advertises Gen 2 and trains at 10 Gbps on a USB 3.2 Gen 2 host port, at 5 Gbps on a
Gen 1 port. The other parameters of the PHY are listed in [Parameters](#phy-parameters).

### Device core parameters {#integ-core}

`USB3DeviceCore` takes the PHY, the frequency of its clock and the parameters below.

| Parameter | Default | Use |
|-------------------------|-----------|-------------------------------------------------------|
| `descriptors` | required | Device, configuration, BOS and string descriptors |
| `endpoints` | none | Bulk and Isochronous endpoints; EP0 is built in |
| `with_lpm` | `False` | Accept the host U1 and U2 requests when idle |
| `with_ep0_handler` | `False` | Pass the class and vendor requests to user logic |
| `crc32_cascade` | `False` | Smaller CRC-32 logic with deeper paths, for small FPGAs |
| `timer_scale` | 1 | Link layer timer scaling, for simulation only |

The descriptors come from a helper (`bulk_descriptors`, `iso_in_descriptors`) or from a list of
interfaces, which is how the device classes and composite devices describe themselves. The BOS
advertises SuperSpeedPlus when the PHY can run Gen 2. [Descriptors](#core-descriptors) and
[Request handler](#core-handler) give the details.

The core reports `ready` (U0, link initialized), `configured` (configuration selected by the host)
and the U1 and U2 enables set by the host. A complete class device (`USB3CDCACM`, `USB3UVC`,
`USB3MSC`) builds its own core; the functions of [Composite devices](#classes-composite) attach to
a shared one.

### Debug registers {#integ-debug}

The debug cores of [Debugging](#debugging) are added next to the PHY and the core. The host tools
expect the register prefixes of the bench SoC: `usb3_debug` for the PHY debug core, and `usb3_core`
(or `usb3_<class>_core` in a single-class device) for the core status and traces.

```python
from usb3_pipe.debug import USB3PHYDebug, USB3SerDesDebug

self.usb3_debug = USB3PHYDebug(self.usb3_phy, self.usb3_serdes, sys_clk_freq)
# Optional: transceiver controls, clock and receive measurements.
self.usb3_serdes_debug = USB3SerDesDebug(self.usb3_serdes, sys_clk_freq,
    ltssm_state = self.usb3_phy.ltssm_state)
# Status, statistics and header packet trace of the device core.
self.usb3_core.add_csr(with_trace=True)
```

A LiteScope analyzer takes its probes from `usb3_analyzer_groups`, one group per question, as
[LiteScope captures](#debug-litescope) explains. The `--lean` option of the bench targets leaves
out the PHY debug core, the throughput profile and the trace, to gain timing margin.

## Choosing the datapath {#integ-choices}

The parameters below set the width, the clocking and the features of the datapath. The defaults
suit Gen 1 on 7-Series and UltraScale+ fabrics; the table says when to change them.

| Parameter | Class | Values | Use |
|--------------------|------------------------|------------|-------------------------------------|
| `link_words` | `USB3PHY` | 1, 2, 4 | 2: 64-bit link interface and endpoints, for Gen 2 rates or a slow fabric |
| `max_burst` | `USB3BulkIN`, `USB3BulkOUT` | 2, 4, 8, 16 | Packets per burst; 16 for the highest Bulk IN rate |
| `pipe_cd` | `USB3PHY` | clock domain | PHY datapath in its own 125 MHz domain, the rest at half rate |
| `gen2_capable` | `USB3PHY` | `True` | Gen 2 advertised; needs a 10 Gbps capable transceiver wrapper |
| `with_u1u2` | `USB3PHY` | `False` | No U1 and U2 in the LTSSM, less logic (U3 kept) |
| `with_lpm` | `USB3DeviceCore` | `True` | Accept the host U1 and U2 requests when idle |

**Link words.** With `link_words=1`, the link interface carries one 32-bit word per cycle, which is
about 500 MB/s at 125 MHz: enough for Gen 1. Gen 2 needs `link_words=2` to go beyond that rate,
and so does a slow fabric at Gen 1. A width of 4 (128 bits) exists for Gen 2 with wide transceiver
words on slow fabrics; it requires `pipe_cd` and `gen2_capable`.

On a wide link, the Bulk endpoints use the link width (`data_width=64` or `128`). A 32-bit Bulk
OUT endpoint is refused there; only the control endpoint is adapted by the protocol layer. The
Mass Storage and CDC-ACM classes keep 32-bit functions behind endpoint converters
(`USB3EndpointUpConverter`, `USB3EndpointDownConverter`). The UVC class, the Isochronous endpoint
and the composite example need 32-bit link words.

**Max burst.** A Bulk endpoint sends or receives up to `max_burst` packets of 1024 bytes before
the host acknowledges them. Each packet slot costs block RAM, and a larger burst hides the host
latency. The vendor examples use 8; 16 gives the best Bulk IN rate on hosts that open windows of
`max_burst` packets.

**PHY clock domain.** On a fabric that does not reach 125 MHz for the link layer and the device
core, `pipe_cd` keeps the PHY datapath and the transceiver wrapper in a 125 MHz domain and runs the
LTSSM, the link layer and the device core in `sys` on 64-bit words. The Tang Mega 138K Pro target
runs `sys` at 62.5 MHz on the Gowin GW5A, the ECPIX-5 target at 83.33 MHz on the ECP5.

The PHY datapath runs on the transmit clock of the transceiver, which is frequency-locked to the
transmitted words: no transmit crossing is needed in the wrapper. The control logic of the wrapper
stays on a free-running clock (`xcvr_cd`), and the caller renames the wrapper into the transmit
domain, as the Tang Mega 138K Pro target does:

```python
# sys at 62.5 MHz; cd_gtr12 is a copy of sys created by the SoC.
usb3_serdes = GW5USB3SerDes(platform,
    sys_clk       = self.crg.cd_sys.clk,
    sys_clk_freq  = 125e6,  # PIPE and transceiver datapath frequency.
    upar_clk      = self.crg.cd_sys.clk,
    xcvr_cd       = "gtr12",
    xcvr_clk_freq = 62.5e6,
    tx_cdc        = False)
usb3_serdes = ClockDomainsRenamer({"sys": "tx"})(usb3_serdes)
usb3_phy    = USB3PHY(usb3_serdes, sys_clk_freq=62.5e6,
    role          = "device",
    pipe_cd       = "tx",
    pipe_clk_freq = 125e6,
    link_words    = 2,      # 4 with a Gen 2 capable wrapper.
    with_u1u2     = False)  # Less logic; U3 is kept.
```

The device core then uses 64-bit Bulk endpoints. When the receive buffer of the PHY fills, the PHY
asks the link layer to hold its credits (`rx_throttle`), without change in the core. The PHY debug
core works with `pipe_cd`; the transceiver debug core and the throughput profile are not used with
it by the targets. [Slower fabrics](#phy-pipe-cd) explains what crosses between the two domains.

## Clocks and timing constraints {#integ-clocks}

The design has three kinds of clocks. The table lists them and who declares their constraints.

| Clock | Frequency | Constraint |
|---------------------|-------------------------|---------------------------------------------------|
| `sys` | 125 MHz, lower with `pipe_cd` | Your clock generator, as for any LiteX clock |
| `oob` | `sys` divided by 8 | Your clock generator (AMD 7-Series and UltraScale+) |
| `tx`, `rx` | Transceiver word clocks | The transceiver wrapper, automatically |
| Reference clock | 100 to 250 MHz | Period constraint on the input pins |

The transceiver wrappers add the period constraints of their transmit and receive clocks and the
false paths between them and `sys`; every crossing between these domains goes through
synchronizers or asynchronous FIFOs. Your design only constrains its own clocks. The system clock
must be at least 125 MHz without `pipe_cd`, since the PHY moves one 32-bit word per cycle at
5 Gbps.

On slow fabrics, keep new logic registered and check timing on the target fabric itself: a design
that meets 125 MHz on a Kintex-7 can miss it by a wide margin on a GW5A or an ECP5.

## Endpoint streams {#integ-streams}

User logic exchanges packets with the endpoints through LiteX streams. Each endpoint carries
`data` (32, 64 or 128 bits, first byte on bits 7 to 0), `be` (one valid flag per byte) and `last`
(end of packet), with the usual `valid` and `ready` handshake.

| Endpoint | Stream | Direction | Rules |
|-------------------|----------|----------------|---------------------------------------------------|
| `USB3BulkIN` | `sink` | Device to host | `be` full except on the last word; packets above 1024 bytes are split |
| `USB3BulkOUT` | `source` | Host to device | `last` on the last word of each packet; `be` on that word |
| `USB3IsoIN` | `sink` | Device to host | One payload per service interval, see [Device classes](#classes) |

A Bulk IN packet shorter than 1024 bytes ends a transfer on the host side, as a short packet.
Back-pressure is allowed on every stream: the IN endpoint stops accepting words when its packet
slots are full, and the OUT endpoint keeps the packets in its slots until user logic reads them.
`USB3DeviceCore.ready` is high in U0 and `configured` after the host selected the configuration.

## Standalone Verilog core {#integ-standalone}

Designs built without LiteX use `usb3_pipe_gen`, installed with the package. It reads a YAML
configuration and generates a self-contained Verilog module with its constraints and its
documentation. The figure below shows the ports of the generated module: the PHY alone exposes a
symbol stream, and with `usb_core` the device core adds a Bulk IN and a Bulk OUT stream.

![The standalone core: clock, reset, reference clock and serial pins on the left; symbol streams
(PHY only) or Bulk streams (with the device core) and status on the right. All streams are
AXI-Stream.](integ_standalone.svg)

```sh
usb3_pipe_gen examples/a7_device.yml     --output-dir build/usb3_pipe_core
usb3_pipe_gen examples/a7_usb_device.yml --output-dir build/usb3_device \
    --name usb3_device
usb3_pipe_gen examples/k7_device.yml     --output-dir build/usb3_k7
usb3_pipe_gen examples/usp_device.yml    --output-dir build/usb3_usp
```

The generator supports the AMD transceivers: Artix-7 GTP, Kintex-7 GTX and UltraScale+ GTY. The
`--name` option sets the module name, `usb3_pipe_core` by default. `--help` also lists the standard
LiteX builder options; only `--output-dir` and `--name` matter for the generator. The
configuration keys are:

| Key | Default | Description |
|----------------------------------|----------------------|----------------------------------------|
| `phy` | required | `A7USB3SerDes`, `K7USB3SerDes` or `USPUSB3SerDes` |
| `device` | PHY family | FPGA device, `xc7a`, `xc7k` or `xcku3p` |
| `speedgrade` | `-1` | Speed grade, for the out-of-band clock PLL |
| `sys_clk_freq` | `125e6` | Frequency of `clk`, at least 125 MHz |
| `refclk` | `pads` | `pads`: `refclk_p/n` ports; `sys`: the `clk` input |
| `refclk_freq` | `125e6` | Reference clock frequency |
| `role` | `device` | `device` (upstream port) or `host` (downstream port) |
| `with_endianness_swap` | `True` | First symbol on bits 31 to 24 of `tdata` |
| `with_link_ctrl` | `False` | Expose the LTSSM link layer control ports |
| `timer_scale` | `1` | Timer scaling, for simulation only |
| `usb_core` | `False` | Include the device core with Bulk endpoints |
| `usb_vid`, `usb_pid` | `0x1d50`, `0x605a` | Device IDs (`usb_core`) |
| `usb_manufacturer` | `Enjoy-Digital` | Strings: also `usb_product`, `usb_serial` |
| `usb_max_burst` | `8` | Bulk max burst: 2, 4, 8 or 16 (`usb_core`) |

An example configuration with the device core:

```yaml
# Artix-7 GTP, USB 3 device with Bulk IN/OUT AXI-Stream endpoints.
phy           : A7USB3SerDes
sys_clk_freq  : 125e6
refclk        : pads
refclk_freq   : 125e6
usb_core      : True
usb_vid       : 0x1d50
usb_pid       : 0x605a
usb_product   : USB3-PIPE
usb_max_burst : 8
```

The ports of the module depend on the configuration:

- **Always**: `clk` and `rst` (active high, asynchronous), `refclk_p/n` with `refclk: pads`, the
  serial pins `tx_p/n` and `rx_p/n`, `ready` (transceiver ready), the LTSSM status outputs
  (`u0`, `recovery`, `rx_ready`, `tx_ready` and the other states) and `ltssm_state`.
- **PHY only**: `sink_*` and `source_*`, 32-bit symbol streams with one K-symbol flag per byte in
  `tuser`. `sink_tlast` marks the end of each packet and is required: the PHY inserts SKP ordered
  sets only between packets. `source_tready` must stay high.
- **With the device core**: `bulk_in_*` (EP1 IN) and `bulk_out_*` (EP2 OUT), 32-bit streams with
  the first byte on `tdata[7:0]` and `tkeep` on the last word, plus `usb_ready` (U0) and
  `usb_configured`.

The generator writes the following files:

| File | Content |
|---------------------------|---------------------------------------------------------|
| `gateware/<name>.v` | Verilog of the core, top module `<name>` |
| `gateware/<name>.xdc` | Constraints of the internal clocks and crossings |
| `gateware/*.init` | Memory initialization files |
| `<name>_inst.v` | Instantiation template |
| `<name>_ltssm.vh` | Encoding of `ltssm_state` as Verilog `localparam` |
| `<name>.md` | Documentation of the generated configuration |

A Verilog instantiation of the `usb3_device` core generated above looks like the following; the
generated `<name>_inst.v` gives the exact port list of your configuration.

```verilog
usb3_device usb3 (
    .clk             (clk125),
    .rst             (rst),
    .refclk_p        (gtp_refclk_p),
    .refclk_n        (gtp_refclk_n),
    .tx_p            (usb3_tx_p),
    .tx_n            (usb3_tx_n),
    .rx_p            (usb3_rx_p),
    .rx_n            (usb3_rx_n),
    .bulk_in_tvalid  (in_tvalid),
    .bulk_in_tready  (in_tready),
    .bulk_in_tlast   (in_tlast),
    .bulk_in_tdata   (in_tdata),
    .bulk_in_tkeep   (in_tkeep),
    .bulk_out_tvalid (out_tvalid),
    .bulk_out_tready (out_tready),
    .bulk_out_tlast  (out_tlast),
    .bulk_out_tdata  (out_tdata),
    .bulk_out_tkeep  (out_tkeep),
    .usb_ready       (usb_ready),
    .usb_configured  (usb_configured),
    .ready           (serdes_ready)
);
```

Your project constrains `clk` and the reference clock pins and places the transceiver and its
pins. Scope the generated `.xdc` to the core instance, since it names the internal clocks of the
core. The generated `<name>.md` is the port reference of your configuration: clocking, data format,
link bring-up and integration notes.

The standalone core has the following limits; the LiteX flow lifts them all:

- Gen 1x1 only: no `gen2_capable`, `link_words` or `pipe_cd` key, 32-bit streams at `sys_clk_freq`
  (125 MHz or more).
- AMD transceivers only: no Gowin or Lattice core.
- With `usb_core`: Bulk IN on EP1 and Bulk OUT on EP2 only, no other device class.
  `with_endianness_swap` must stay set, and `with_link_ctrl` is not available since the device
  core drives the link control.

## Integration without LiteX {#integ-other}

The standalone core is the path for Vivado projects and other HDLs on AMD devices. For another
FPGA family, generate Verilog from a small LiteX design instead: the bench targets of the Gowin and
Lattice boards build the transceiver wrapper, the PHY and the device core with the vendor or open
source toolchains, and the LiteX build produces the Verilog and the constraints before calling them.

The device core only depends on the link interface of the PHY, documented in [PHY
architecture](#phy-architecture). Another PHY that provides the same signals can drive it, and
the PHY can serve another link layer through the symbol streams of the standalone core.

## Porting to a new board {#integ-porting}

A new board needs a target script and, for a new transceiver family, a new wrapper. Start from the
target of the closest board in `bench/` and go through the steps below.

1. Check the hardware against [Requirements for a custom board](#boards-custom): lane, reference
   clock, AC coupling, polarity and, for USB-C, the multiplexer.
2. Write the clock generator: `sys` at 125 MHz and, on AMD devices, `oob` at `sys` divided by 8.
   A target file is about 100 lines: the clock generator, the pads and the wrapper; the SoC, the
   options and the examples are shared.
3. Declare the serial pins of the lane and the reference clock pins as a platform extension.
4. Instantiate the wrapper of the transceiver family with the reference clock of the board;
   [Transceivers](#transceivers) lists the settings of each family.
5. Call `add_usb3_device` (or build the chain of [Integration in a LiteX SoC](#integ-litex)) with
   the `loopback` example and a JTAG or UART bridge for the debug registers.
6. Build, load, and check the LTSSM state through the debug registers before looking at the host:
   [Debugging](#debugging) gives the order of the checks.
7. Qualify the link with `usb3_bulk.py loopback`, then `usb3_bench.py both` and a soak run of a few
   minutes with `usb3_soak.py`.

Where to go next: [Examples](#examples) for complete device designs, [Debugging](#debugging) when
the link does not come up.
