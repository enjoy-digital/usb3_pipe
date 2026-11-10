```{=latex}
\part{The PHY}
```

```{=html}
<h1 class="part">Part II. The PHY</h1>
```

# PHY architecture {#phy-architecture}

This chapter describes the PHY, `usb3_pipe`: the blocks between the FPGA transceiver and the link
layer, the interface it gives to the link layer, its clock domains and its parameters. Read it
before integrating the PHY or writing your own link layer. The link training is in
[Link training](#link-training), the symbol datapath in [Datapath](#datapath) and the
transceiver wrappers in [Transceivers](#transceivers).

## At a glance {#phy-glance}

| Item                 | Value                                                                 |
|----------------------|-----------------------------------------------------------------------|
| Top-level class      | `USB3PHY` (`usb3_pipe/phy.py`)                                        |
| Operating modes      | Gen 1x1 (5 Gbps), Gen 2x1 (10 Gbps), Gen 1x2 (simulation only)        |
| Link interface       | 32-bit words (4 symbols), 64-bit pairs or 128-bit quads (`link_words`) |
| Link clock           | `sys`, 125 MHz or more; lower with `pipe_cd` and 64-bit words         |
| Transceivers         | Artix-7 GTP, Kintex-7 GTX, Kintex UltraScale+ GTY, Gowin GTR12, ECP5 DCU |
| Roles                | `device` (upstream port) or `host` (downstream port, simulation)     |
| External USB chip    | None: the transceiver drives the USB 3 pairs directly                 |

## Overview {#phy-overview}

The PHY turns a raw FPGA transceiver into a trained USB 3 link. It sends and detects the low
frequency signaling of the link start, exchanges the training sequences, scrambles the data and
keeps the two clocks of the link in step. When the link is up, the link layer sees a plain stream
of symbols.

The figure below shows the blocks. The transceiver and its wrapper, `USB3SerDes`, are added by
the caller and passed to `USB3PHY`. Inside the PHY, the PIPE units do the work on the symbols and
the LTSSM decides what they do.

![PHY blocks: the transceiver wrapper feeds the PIPE units (LFPS, training sequences, scrambling,
Idle handshake) and the LTSSM drives them. Follow the cyan path: the link layer exchanges plain,
descrambled symbols with the PHY on `sink` and `source`.](phy_block.svg)

The name PIPE comes from the PHY Interface for PCI Express and USB 3 used by external PHY chips.
The project keeps the split: `USB3PIPE` plays the role of the PHY chip, the LTSSM and the link
layer sit above it. Unlike with a PIPE chip, the LTSSM is part of the PHY here. The link layer
therefore only handles packets and power management link commands, and can be replaced without
touching the training.

The PHY is organized by function. The generation (Gen 1 or Gen 2) and the lane count are
parameters. Each file has its Gen 1 part first, then its Gen 2 or SuperSpeedPlus part.

## Modules {#phy-modules}

The table below lists every file of `usb3_pipe/` with its main classes. The datapath and
transceiver files are described in [Datapath](#datapath) and [Transceivers](#transceivers).

| File             | Main classes                                       | Role                                                        |
|--------------------------|--------------------------------------------|--------------------------------------|
| `phy.py`         | `USB3PHY`, `LinkWordsCDC`                          | Top level: PIPE, LTSSM, link interface, clock domains       |
| `core.py`        | `USB3PIPE`, `Gen2PIPE`, `IdleHandshake`            | Gen 1 and Gen 2 PIPE: LFPS and TS units, scrambling, Idle   |
| `ltssm.py`       | `USB3LTSSM`, `SSPPolling`                          | Link Training and Status State Machine                      |
| `lfps.py`        | `LFPSUnit`, `SCDGenerator`, `LBPMGenerator`        | LFPS patterns, SuperSpeedPlus SCD and LBPM signaling        |
| `training.py`    | `TSUnit`, `TSGenerator`, `TSChecker`               | TSEQ, TS1 and TS2 generation and detection                  |
| `scrambling.py`  | `Scrambler`, `Descrambler`, `Gen2Scrambler`        | Gen 1 and Gen 2 scramblers                                  |
| `coding.py`      | `RX8b10bChecker`, `Gen2BlockGearbox`               | 8b/10b tables and checker, 128b/132b blocks                 |
| `datapath.py`    | `TXDatapath`, `RXDatapath`, `Gen2TX`, `Gen2RX`     | SKP, word alignment, clock crossing, Gen 2 blocks           |
| `framing.py`     | `Gen2LinkTX`, `Gen2LinkRX`, `WordsToPairs`         | Gen 2 packet framing, 64-bit word pairs helpers             |
| `common.py`      | `Symbol`, `OrderedSet`, `AsyncFIFO`                | Symbols, Ordered Sets, LTSSM to PIPE signal list, FIFOs     |
| `crc.py`         | `CRC5`, `CRC16`, `CRC32`                           | CRCs of the link layer packets                              |
| `lanes.py`       | `LaneStriper`, `LaneDestriper`, `LaneDeskew`       | Gen 1x2 lane striping and deskew                            |
| `serdes.py`      | `USB3SerDes`, `GW5USB3SerDes`, `ECP5USB3SerDes`    | Transceiver wrappers: AMD, Gowin and Lattice                |
| `gw5/`           | TOML files, `gen_toml.py`                          | Gowin GTR12 register configurations                         |
| `debug.py`       | `USB3PHYDebug`, `USB3SerDesDebug`                  | Debug registers and LiteScope groups                        |
| `gen.py`         | `USB3PIPECore`                                     | Standalone Verilog core generator `usb3_pipe_gen`           |

The standalone generator wraps the PHY in a Verilog module for designs outside LiteX; it is
described in [Integration](#integration). The debug registers are listed in
[Registers](#registers) and used in [Debugging](#debugging).

## Link interface {#phy-link-interface}

The link interface is everything a link layer needs from the PHY. The device core uses only this
interface, so any other link layer can replace it. The streams and the signals below are
attributes of `USB3PHY`.

### Data streams {#phy-streams}

The PHY carries symbols in both directions as LiteX streams. Each symbol is a byte plus a flag
telling whether it is a control symbol (a K symbol such as SHP or END) or data. The table below
gives the layouts.

| Stream      | Direction       | Layout (`link_words=1`) | Layout (`link_words=2`)        |
|----------------|--------------|------------------------|------------------------------|
| `sink`      | Link to PHY     | `data` 32, `ctrl` 4     | `data` 64, `ctrl` 8, `words` 2 |
| `source`    | PHY to link     | `data` 32, `ctrl` 4     | `data` 64, `ctrl` 8, `words` 2 |
| `source_lc` | PHY to link     | Gen 2 only              | Gen 2 only                     |

On 32-bit words, the first symbol is on `data[31:24]` and `ctrl[3]`; the PHY swaps the
endianness for the link layer (`with_endianness_swap`). On 64-bit word pairs, word 0 comes first
and `words` gives the number of valid words, 1 or 2. With `link_words=4`, the streams carry
128-bit word quads: `data` 128, `ctrl` 16 and a 3-bit `words`, from 1 to 4.

On transmit, the PHY sends Logical Idle when `sink.valid` is low. It lowers `sink.ready` while it
inserts SKP Ordered Sets for clock compensation. On receive, `source` has no back-pressure: words
arrive at the line rate, already aligned on the packet framing.

At Gen 2, the PHY can deliver the link commands that return credits (LGOOD and LCRD) ahead of
the buffered packets. The link layer enables this path with `source_lc_enable`; otherwise these
commands stay in `source`.

### Status and control {#phy-status}

The LTSSM reports the link state on single-bit outputs. The link layer reads them to start and
stop its traffic. The table below gives the outputs of `USB3PHY_STATUS` and a few others.

| Signal          | Meaning                                                              |
|-----------------|----------------------------------------------------------------------|
| `ready`         | Transceiver ready: PLL locked, transceiver reset done                |
| `u0`            | Link in U0: packets can flow                                         |
| `rx_ready`      | Received words valid for the link layer (U0)                         |
| `tx_ready`      | Words to send accepted from the link layer (U0)                      |
| `recovery`      | Link in Recovery (retraining)                                        |
| `u1`, `u2`, `u3`| Link in a low power state                                            |
| `hot_reset`     | Hot Reset in progress                                                |
| `polling_idle`  | Polling.Idle: the link layer prepares its first messages             |
| `rx_keep`       | Received words kept until U0 (Recovery.Idle)                         |
| `ess_inactive`  | eSS.Inactive: link failed, waiting before a new training             |
| `compliance`    | Compliance mode (electrical test patterns)                           |
| `loopback`      | Loopback mode                                                        |
| `gen2_active`   | Gen 2 operation at 10 Gbps                                           |
| `ltssm_state`   | Encoded LTSSM state (debug), names in `ltssm_states`                 |

The link layer drives the LTSSM through the signals of `USB3PHY_LINK_CTRL`. Power management
uses a handshake of link commands: the link layer decodes the link commands from the partner and
the LTSSM decides the answer. The table below groups them.

| Signals                                            | Direction   | Purpose                                       |
|----------------------------------------------------|-------------|-----------------------------------------------|
| `lgo_u1`, `lgo_u2`, `lgo_u3`, `lau`, `lxu`, `lpma` | Link to PHY | Power management link commands received      |
| `tx_lgo_u1`, `tx_lgo_u2`, `tx_lgo_u3`              | PHY to link | Request to send LGO_Ux                       |
| `tx_lau`, `tx_lxu`, `tx_lpma`                      | PHY to link | Request to send LAU, LXU or LPMA             |
| `req_u1`, `req_u2`, `req_u3`                       | Link to PHY | Upper layer asks for a low power state       |
| `u2_timeout`                                       | Link to PHY | U2 inactivity timeout, 256 µs units          |
| `link_activity`                                    | Link to PHY | Traffic seen: holds off the U0 timeout       |
| `directed_recovery`                                | Link to PHY | Retrain the link (link layer error recovery) |
| `hot_reset_req`, `warm_reset_req`                  | Link to PHY | Resets, downstream port only                 |
| `exit_u1_req`, `exit_u2_req`, `wakeup_u3_req`      | Link to PHY | Leave a low power state                      |

A `u2_timeout` of `0x00` or `0xff` disables the U2 inactivity timer. The handshake itself is
described in [Power management](#lt-pm). The signals are the LTSSM ones, so an input that the link
layer does not drive keeps its reset value.

### Capabilities {#phy-capabilities}

A few attributes describe what the PHY supports. They are always defined, as constants when a
feature is absent, so that a link layer can adapt at elaboration time (`USB3PHY_LINK_INTERFACE`).

| Attribute       | Meaning                                                                    |
|----------------------|------------------------------------------------------------|
| `link_words`    | Words per cycle: 1 (32-bit), 2 (64-bit pairs) or 4 (128-bit quads)         |
| `with_u1u2`     | U1 and U2 supported: LGO_U1 and LGO_U2 may be accepted                     |
| `ssp_capable`   | Gen 2 possible: the link layer applies the SuperSpeedPlus rules            |
| `rx_throttle`   | With `pipe_cd`: received words buffered, receive credits held              |
| `rx_t2_credits` | Type 2 credits the partner may hold: 4 (no limit) or a runtime signal      |

## Clock domains {#phy-clocks}

The transceiver runs on its own transmit and receive clocks, `tx` and `rx`: two symbols per cycle
at 250 MHz on most AMD transceivers, four at 125 MHz on the others. The PHY logic runs in `sys`,
four symbols per cycle. One FIFO crosses each direction, as the figure below shows.

![Gen 1 clock domains: the width conversion and the SKP remover run on the transceiver clocks; the
SKP inserter, the word aligner, the PIPE and the LTSSM run in `sys`. The cyan FIFOs are the only
crossings.](phy_clocks.svg)

The SKP remover works on the receive clock, before the FIFO, because the partner's clock may be
faster than the local one: dropping its SKP symbols is what keeps the FIFO from overflowing. The
receive path has no back-pressure, so `sys` must read four symbols per cycle at the line rate:
this is why `sys` runs at 125 MHz or more. The SKP inserter stays in `sys`, where it pauses the
link layer.

At Gen 2, the 128b/132b block datapaths also run on the transceiver clocks, and the packet
framing crosses to `sys` with its own buffers. The stages are described in [Datapath](#datapath),
the clocks of each family in [Clocking](#gt-clocking).

### Resets {#phy-resets}

`USB3PHY` has a reset in `sys` that restarts the link training from Polling. With Gen 2 or
`pipe_cd`, the reset is resynchronized into the other domains, so both sides of each crossing
restart together.

The transceiver receiver does not always lock cleanly after its initialization. On an Artix-7 GTP
with a host using SSC, about one receiver reset in twelve ended with a receiver unable to train
or with a high error rate. The PHY therefore re-runs the receiver reset sequence each time the
LTSSM goes back to Rx.Detect: after each training failure and after a Warm Reset. The
`serdes_rx_reset` input requests it by hand.

### Slower fabrics: `pipe_cd` {#phy-pipe-cd}

On some FPGAs the LTSSM and the link layer cannot run at 125 MHz: Gowin GW5A logic is about
twice as slow per logic level as Artix-7. The `pipe_cd` option keeps the PIPE and the transceiver
wrapper in their own clock domain at 125 MHz. The LTSSM and the link interface run in `sys` at
half the frequency, on 64-bit word pairs, which keeps the same bandwidth. The figure below shows
the split.

![PHY with `pipe_cd`: the PIPE stays at 125 MHz; the LTSSM and the link layer run at a lower
frequency, here the 62.5 MHz of the Gowin target, on 64-bit words. The buffers on both directions
form the link words crossing.](phy_pipe_cd.svg)

The crossing needs more than two FIFOs. The PIPE side runs at the line rate, but the link layer
between packets does not, so each direction adapts the flow. One module holds the buffers of both
directions (`LinkWordsCDC`); the list below gives how each part works.

- **LTSSM and PIPE**: control and status levels are resynchronized; status pulses go through
  pulse synchronizers. Training sequence requests take effect on Ordered Set boundaries, so a
  late request never truncates one.
- **Receive**: Logical Idle words between packets are dropped and the words packed in full pairs
  (`LogicalIdleFilter`, `WordsPacker`). A deep FIFO in block RAM absorbs the line rate. When it
  is half full, `rx_throttle` makes the link layer hold its receive credits, so the partner stops
  after its outstanding packets.
- **Transmit**: packets are stored and forwarded (`PacketsBuffer`), so no Logical Idle enters a
  packet. SKP Ordered Sets go only between packets; a host rejected header packets with SKP inside.
  Words that reach the PIPE outside U0 are dropped, so no stale word is sent at the next U0 entry.

`pipe_rx_depth` overrides the receive FIFO depth. The default is 256 word pairs with
`link_words=2`, and 1024 entries with a Gen 2 capable PHY, where the throttling starts at a
quarter of the FIFO instead of half.

With a Gen 2 capable transceiver, the Gen 2 words bypass the Gen 1 crossing while `gen2_active`
is set. The Gen 2 transmit framing runs in `sys`. With 40-bit raw words, the Gen 2 receive
framing runs in `pipe_cd` and feeds the same receive FIFO. With 80-bit raw words, it runs in
`sys` at 32 bytes per cycle, and the throttling follows its own buffer level.

On the Tang Mega 138K Pro, the PIPE runs on the GTR12 transmit clock, so the transmit path needs
no crossing at all. The ECPIX-5 target does the same on the DCU transmit clock, with `sys` at
83.33 MHz.

## Parameters {#phy-parameters}

The table below lists the parameters of `USB3PHY`.

| Parameter              | Default    | Effect                                                                  |
|----------------------------------|----------|---------------------------------------------------|
| `serdes`               | required   | Transceiver wrapper, or a list of two for `lanes=2`                     |
| `sys_clk_freq`         | required   | `sys` frequency, sets every timer                                        |
| `role`                 | `device`   | `device` (upstream port) or `host` (downstream port)                    |
| `with_endianness_swap` | `True`     | First symbol on `data[31:24]` for the link layer                        |
| `timer_scale`          | 1          | Divides every timer (simulation speed-up); 1 gives the specification values |
| `lanes`                | 1          | 1 (x1) or 2 (Gen 1x2)                                                   |
| `x2_capable`           | `True`     | Advertise two lanes (with `lanes=2`)                                    |
| `gen2_capable`         | `False`    | Advertise Gen 2 in SuperSpeedPlus Polling                               |
| `deskew_depth`         | 8          | Lane deskew FIFO depth (`lanes=2`)                                      |
| `link_words`           | 1          | 1 (32-bit), 2 (64-bit pairs), 4 (128-bit quads, `pipe_cd` and Gen 2)    |
| `pipe_cd`              | `None`     | Clock domain of the PIPE and the SerDes, see [`pipe_cd`](#phy-pipe-cd)  |
| `pipe_clk_freq`        | `None`     | Frequency of `pipe_cd`, 125 MHz or more                                 |
| `pipe_rx_depth`        | `None`     | Receive FIFO depth with `pipe_cd`                                       |
| `with_u1u2`            | `True`     | U1 and U2 support; `False` saves logic, U3 is kept                      |

With `with_u1u2=False`, the LTSSM never enters U1 or U2 and the device core rejects them with
LXU. U3 stays, because an upstream port may not reject it.

## Usage {#phy-usage}

The caller creates the transceiver wrapper, then the PHY. The example below is the Gen 1 setup of
the Acorn target, on an Artix-7 GTP.

```python
from usb3_pipe     import A7USB3SerDes
from usb3_pipe.phy import USB3PHY

self.usb3_serdes = A7USB3SerDes(platform,
    sys_clk      = self.crg.cd_sys.clk,
    sys_clk_freq = sys_clk_freq,
    refclk_pads  = platform.request("refclk"), # Or a fabric clock (ClockSignal).
    refclk_freq  = 125e6,
    tx_pads      = platform.request("sfp_tx"),
    rx_pads      = platform.request("sfp_rx"))
self.usb3_phy = USB3PHY(serdes=self.usb3_serdes, sys_clk_freq=sys_clk_freq, role="device")
```

Gen 2 needs a transceiver able to run at 10 Gbps. On the Kintex UltraScale+ GTY of the XCKU3P
board, the wrapper and the PHY both take `gen2_capable`.

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
    gen2_capable = True)  # 10 Gbps PLL, 40-bit raw words, Gen 1 at half the rate.
self.usb3_phy = USB3PHY(serdes=self.usb3_serdes, sys_clk_freq=sys_clk_freq, role="device",
    gen2_capable = True,  # Gen 2x1 advertised in SuperSpeedPlus Polling.
    link_words   = 2)     # 64-bit link interface for the Gen 2 bandwidth.
```

With 64-bit word pairs, the Gen 2 framing moves the data packets 8 bytes per cycle: about
1 GB/s at 125 MHz, against about 500 MB/s with 32-bit words. At Gen 1, the PHY adapts the pairs
to its 32-bit words. The full SoC integration is in [Integration](#integration).

## Gen 2 operation {#phy-gen2}

With `gen2_capable`, the PHY starts every training at 5 Gbps and runs SuperSpeedPlus Polling. If
both ports support Gen 2, the transceiver switches to 10 Gbps and the training continues on the
`Gen2PIPE`, with 128b/132b blocks. The LFPS stays on the Gen 1 PIPE. If the partner only supports
Gen 1, the link trains at 5 Gbps as usual.

`gen2_active` tells the link layer which rules apply. The next SuperSpeedPlus Polling starts again
at 5 Gbps. The Gen 2 datapath and framing are described in [Datapath](#datapath).

## Gen 1x2 lanes {#phy-lanes}

With `lanes=2`, the PHY drives two transceivers as one Gen 1x2 link at 2 x 5 Gbps. Each lane has
its own PIPE under a single LTSSM. Lane 0 is the configuration lane: it alone carries the LFPS
and the SuperSpeedPlus Polling, while lane 1 stays in electrical idle.

The link words are 64-bit and split symbol by symbol: even symbols on lane 0, odd symbols on
lane 1 (`LaneStriper`). On receive, `LaneDeskew` aligns the lanes on the Ordered Sets, which both
lanes send at the same time, and `LaneDestriper` rebuilds the words. Each lane has its own
scrambler seed and its own polarity.

Gen 1x2 is validated in simulation only. The PHY does not fall back to Gen 1x1 on lane 0, and
the device core has no SuperSpeedPlus link layer for two lanes.

## Limitations {#phy-limitations}

- Hardware: Gen 1x1 validated on the Artix-7 GTP, Kintex UltraScale+ GTY, Gowin GTR12 and ECP5
  DCU (the Kintex-7 GTX is build-tested). Gen 2x1 is validated on the Kintex UltraScale+ GTY and
  in bring-up on the Gowin GW5AST GTR12.
  Gen 1x2 runs in simulation only; Gen 2x2 is not started.
- With `pipe_cd`, Gen 1 is validated on hardware; Gen 2 runs in simulation and is in bring-up.
  Its throughput, a little below the 125 MHz targets, is measured in [Performance](#performance).
- Gen 2 has no elastic buffer: the Gen 2 receive path runs in the recovered clock domain. Gen 2
  does not support disabling the scrambler.
- The device core rejects U1 and U2 by default; `with_lpm` accepts them, validated in simulation
  only.

Where to go next: [Link training](#link-training) for the LTSSM, [Datapath](#datapath) for the
symbol path, [Transceivers](#transceivers) for the wrappers.
