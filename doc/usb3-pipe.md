# USB3 PHY (`usb3_pipe`)

USB 3.2 Gen 1x1 (SuperSpeed, 5Gbps) and Gen 2x1 (SuperSpeedPlus, 10Gbps) PHY on FPGA transceivers: a
PIPE-like interface (LFPS, Training Sequences, 8b/10b or 128b/132b, scrambling, clock compensation)
and the LTSSM, in Migen/LiteX. It provides a link-trained 32-bit symbol stream (with K-symbol flags,
or 64-bit word pairs with `link_words=2`) in the `sys` clock domain (>= 125MHz) to a USB3 Link Layer:
the USB3 Device Core ([usb3-core.md](usb3-core.md)) or any other Link Layer through the standalone
core ([standalone-core.md](standalone-core.md)).

```
 TX/RX pads <-> SerDes (8b/10b, LFPS, SKP, alignment) <-> PIPE (TS, scrambling, Idle) <-> LTSSM
                usb3_pipe/serdes.py (datapath.py)       usb3_pipe/core.py               ltssm.py
                                         usb3_pipe/phy.py: USB3PHY (PIPE(s) + LTSSM)
```

The package is organized by function, the generation (Gen 1/Gen 2) and the lane count being
parameters: each file has its Gen 1 part first, then its Gen 2/SuperSpeedPlus part.

## Modules

| Module | File | Role |
|---|---|---|
| `USB3SerDes`: `A7USB3SerDes`/`K7USB3SerDes`/`USPUSB3SerDes`/`GW5USB3SerDes` | `serdes.py` | Common SerDes on LiteICLink transceivers (Artix7 GTP/Kintex7 GTX/UltraScale+ GTY/Gowin GW5AST GTR12) at 5Gbps (and 10Gbps on the GTY with `gen2_capable`: QPLL at 10Gbps, 40-bit raw words) with their USB3 settings (SSC tolerant CDR, LFPS/OOB): 8b/10b, polarity, electrical idle/LFPS patterns and the TX/RX datapaths below. `GW5USB3SerDes`: GTR12 Q1 lane 0/1, 40-bit words at 125MHz, register configuration generated from `usb3_pipe/gw5` TOML files with Gowin's `serdes_toml_to_csr` (100MHz REFCLK1 or 125MHz REFCLK0). |
| `TXDatapath`/`RXDatapath` (`TXSKPInserter`, `RXSKPRemover`, `RXWordAligner`, `RXErrorSubstitution`, `RXErrorMonitor`, `RXIdleFilter`), `Gen2TX`/`Gen2RX`, `Gen2TXDatapath`/`Gen2RXDatapath` | `datapath.py` | Gen 1: SKP insertion (TX, 6.4.3) and removal (RX), word alignment on COM, 8b/10b errors substitution (6.3.1.4)/monitoring, electrical idle filtering, CDC/data-width. Gen 2: block transmitter/receiver (TSEQ/TS1/TS2/SYNC/SDS/SKP Ordered Sets, Data Blocks, 6.3.2.3 scrambling rules), 40-bit transceiver words <-> blocks datapaths (250MHz) with RX counters. |
| `RX8b10bChecker`, `Gen2BlockGearbox`/`Gen2BlockAligner` | `coding.py` | 8b/10b Python model/code tables and RX checker (6.3.1), 128b/132b blocks gearbox/aligner and DC balance helpers (6.3.2, 6.4.1.2). |
| Symbols/Ordered Sets | `common.py` | Gen 1 Symbols and TS1/TS2/TSEQ, Link Configuration, Gen 2 Block Headers, Symbols/Ordered Sets identifiers and payloads. |
| `LFPSUnit` | `lfps.py` | LFPS generation/detection (Polling, Ping, U1/U2 exit, U3 wakeup, Warm Reset), timings of 6.9. SuperSpeedPlus LFPS signaling: SCD1/SCD2 and LBPM generators/checkers (6.9.4/6.9.5). |
| `TSUnit` | `training.py` | TSEQ/TS1/TS2 generation and detection (Link Configuration: reset, loopback, scrambling). |
| `Scrambler`/`Descrambler`, `Gen2Scrambler` | `scrambling.py` | Gen 1 scrambler (6.3.1.3, symbol level: SKP does not advance, COM initializes), Gen 2 scrambler (6.3.2.3). |
| `USB3PIPE` | `core.py` | PIPE: LFPS/TS units, scrambling, Logical Idle, Idle handshake, framing alignment (SHP/SDP/SLC). |
| `USB3LTSSM`, `SSPPolling` | `ltssm.py` | LTSSM (7.5): Polling, U0, Recovery, U1/U2/U3, Hot/Warm Reset, Compliance, Loopback, eSS.Inactive, PM handshake, timers. SuperSpeedPlus Polling substates (SCD/LBPM, PortMatch/PortConfig, 7.5.4.3-7.5.4.6) used with `USB3LTSSM(ssp=True)`. |
| `Gen2PIPE` | `core.py` | Gen 2 PIPE: Gen 2 TSEQ/TS1/TS2/SYNC, SDS then Data Blocks, Idle Symbol handshake (5Ah Idle Symbols) on the Gen 2 TX/RX datapaths, Link Layer words through the Gen 2 packet framing. |
| `Gen2LinkTX`/`Gen2LinkRX` (`Gen2TXFramer`, `Gen2TXPacker`, `Gen2RXFraming`) | `framing.py` | Gen 2 packet framing (7.2.1): Link Layer words <-> Data Blocks, framing Symbols as scrambled bytes, DPHSTART/Length Field Replicas, TX units (Header Packet, Link Command, DPH + DPP) stored and forwarded (sys -> tx), RX packets re-aligned on words and buffered (rx -> sys), LGOOD_n/LCRD_x on a separate output (`source_lc`). `WordsToPairs`/`PairsToWords`/`WordsWindow`: words pairs helpers (`link_words=2`). |
| `USB3PHY` | `phy.py` | PIPE + LTSSM with the Link interface (below). `gen2_capable=True` with a Gen 2 capable SerDes (`USPUSB3SerDes(gen2_capable=True)`): SuperSpeedPlus Polling advertising Gen 2x1, 10Gbps operation through the `Gen2PIPE` once Gen 2 is negotiated, Gen 1 otherwise (`add_gen2`). `link_words=2`: 64-bit Link interface (words pairs). `lanes=2`: Gen 1x2 (one PIPE per lane, SuperSpeedPlus Polling, lanes deskew/striping, 64-bit link words, [gen2-x2-roadmap.md](gen2-x2-roadmap.md)). |
| `LaneStriper`/`LaneDestriper`/`LaneDeskew` | `lanes.py` | Gen 1x2 lanes striping/deskew (6.13). |
| `USB3PHYDebug`/`USB3SerDesDebug`, `Gen2RXDebug` | `debug.py` | State history, counters, measurements CSRs ([debugging-guide.md](debugging-guide.md)), Gen 2 RX status/counters. |
| `usb3_pipe_gen` | `gen.py` | Standalone Verilog core generator. |

## Link interface (`USB3PHY`)

What a USB3 Link Layer needs from the PHY (the USB3 Device Core only uses this interface):

| Signals | Direction | Description |
|---|---|---|
| `sink` (data 32, ctrl 4) | Link -> PHY | TX symbols (first symbol on `data[31:24]`), Logical Idle sent when `sink.valid` is low, `sink.ready` low during SKP insertions. |
| `source` (data 32, ctrl 4) | PHY -> Link | RX symbols, framing words aligned, no back-pressure. |
| `source_lc` (data 32, ctrl 4), `source_lc_enable` | PHY -> Link | Gen 2 only (`gen2_capable`): LGOOD_n/LCRD_x Link Commands received ahead of the buffered RX words, when the Link Layer sets `source_lc_enable` (else they stay in `source`). |
| `gen2_active` | PHY -> Link | Gen 2 operation (10Gbps): the Link Layer switches to SuperSpeedPlus operation. |
| `u0`, `recovery`, `rx_ready`, `tx_ready`, `u1/u2/u3`, `polling_idle`, `hot_reset`, ... | PHY -> Link | LTSSM status (`USB3PHY_STATUS`). |
| `lgo_u1/2/3`, `lau`, `lxu`, `lpma`, `req_u*`, `tx_lgo_u*`, `tx_lau`, `tx_lxu`, `tx_lpma` | both | PM handshake (LGO_Ux/LAU/LXU/LPMA decoded by the Link Layer, decision by the LTSSM). |
| `u2_timeout` (8) | Link -> PHY | U2 Inactivity Timeout (LMP value, 256us units, 00h/FFh: disabled): U1 -> U2 on timeout. |
| `directed_recovery`, `hot_reset_req`, `warm_reset_req`, `exit_u1/u2_req`, `wakeup_u3_req`, `link_activity` | Link -> PHY | Directed transitions, U0 Recovery timeout activity (`USB3PHY_LINK_CTRL`). |

The LTSSM <-> PIPE signals (LFPS, Training Sequences, Idle Symbol handshake, SerDes/datapath
controls) are internal to `USB3PHY`: listed once in `LTSSM_PIPE_INTERFACE` (`common.py`) with the
same names on both sides, wired by `connect_ltssm_pipe()`.

## Usage

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

Gen 2x1 (UltraScale+ GTY, `bench/alibaba_xcku3p.py --gen2-capable`):

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
    gen2_capable = True)  # QPLL at 10Gbps, 40-bit raw words, Gen 1 at the /2 rate.
self.usb3_phy = USB3PHY(serdes=self.usb3_serdes, sys_clk_freq=sys_clk_freq, role="device",
    gen2_capable = True,  # Gen 2x1 advertised in the SuperSpeedPlus Polling.
    link_words   = 2)     # Optional: 64-bit Link interface (Gen 2 bandwidth, 64-bit endpoints).
```

With `link_words=2`, `sink`/`source` carry words pairs (`data` 64, `ctrl` 8, `words`: 1 or 2 valid
words, word 0 first), adapted to the Gen 1 PIPE words at Gen 1; the Gen 2 framing then moves the DPP
data 8 bytes per cycle (~1GB/s at 125MHz vs ~500MB/s with 32-bit words).

## Slower fabrics: PIPE clock domain (`pipe_cd`)

On fabrics where the LTSSM/Link Layer/Core can't run at 125MHz (Gowin GW5A, ~2x slower per logic
level than 7-Series), `USB3PHY(pipe_cd=..., pipe_clk_freq=...)` keeps the PIPE (and the SerDes,
renamed by the caller) in its own clock domain at 125MHz and the LTSSM and the Link interface in
`sys` (ex: 62.5MHz, with `link_words=2`):

- LTSSM <-> PIPE: controls/status levels resynchronized, status pulses through PulseSynchronizers
  (Training Sequences detections/bursts ends, LFPS detections). The TS1/TS2/TSEQ requests are taken
  into account on the Ordered Sets boundaries (`TSUnit`): no truncated Ordered Set when a request
  changes after a burst end seen with the CDC latency.
- RX: Logical Idle words between the packets dropped (`LogicalIdleFilter`), words packed in pairs
  (`WordsPacker`), no back-pressure (line rate), deep RX FIFO (block RAM). The Link Layer reads the
  Header Packets/DPP tails one word per cycle (below the line rate): the PHY `rx_throttle` output
  (RX FIFO half full) holds the Link Layer Rx Header Buffer credits (LCRD_x), the partner stops
  after its outstanding Header Packets (Bulk OUT bursts at the line rate).
- TX: packets stored and forwarded (`PacketsBuffer`, the Link Layer marks the packets ends): no
  Logical Idle inside the packets, `first`/`last` propagated to the SKP inserter (SKP Ordered Sets
  only between packets: inside packets, a host on the Gowin target rejected Header Packets).
- `USB3PHY(with_u1u2=False)`: no U1/U2 in the LTSSM (LGO_U1/LGO_U2 rejected by the Link Layer), less
  logic when power saving is not needed (U3 kept).

On the Tang Mega 138K Pro target, the PIPE runs on the GTR12 TX fabric clock (frequency-locked to
the transmitted words: no TX clock domain crossing, `TXDatapath(with_cdc=False)`) and the GTR12
control logic stays on a free-running clock (`GW5USB3SerDes(xcvr_cd=...)`).

## Simulation

`sim.py` connects two PHYs (Host/Device) through SerDes models (configurable word shift, ppm offset,
SSC, SKP insertion): the Host runs a scripted Link/Protocol Layer model (`usb3_core/host_model.py`,
packets checked against captures of an xHCI host), the Device the USB3 Device Core:

```sh
$ ./sim.py                       # Link training, enumeration, Bulk transfers.
$ ./sim.py --no-core --ppm 300   # PRBS Link-Layer stand-ins, clock offset.
$ ./sim.py --host-pm             # + Host U1/U2 requests (rejected: Device Core without LPM).
$ ./sim.py --host-lpm            # + U1/U2 entries/exits (Device Core with LPM).
$ ./sim.py --lanes 2             # Gen 1x2: SSP Polling, two lanes, 64-bit PRBS stand-ins.
$ ./sim.py --gen2-capable        # SSP Polling with Gen 2x1 capability (Gen 2 negotiated, U0 at Gen 1).
$ ./sim.py --gen2                # Gen 2x1: training to U0 at 10Gbps, enumeration, Bulk transfers.
$ ./sim.py --gen2 --link-words 2 # Same with the 64-bit Link interface on the Device.
$ ./sim.py --gen2 --host-lpm     # Gen 2 U1/U2 entries/exits (Recovery at Gen 2).
$ ./sim.py --dev-pipe-cd         # Device LTSSM/Link Layer/Core at half the PIPE frequency (pipe_cd).
$ ./sim.py --dev-pipe-cd --dev-no-u1u2 --host-pm --host-itp  # + no U1/U2 (LXU), Host ITPs.
$ ./sim.py --trace               # VCD (build/sim/gateware/sim.vcd).
$ pytest test                    # Unit/system tests (-n auto: parallel with pytest-xdist).
$ pytest test -m slow            # Long simulations (Verilator), not run by default.
```

## Limitations

- Gen 1x1 and Gen 2x1 on hardware (Gen 2x1 on UltraScale+ GTY only, Gen 1x1 on Gowin GW5AST), Gen
  1x2: simulation only, Gen 2x2: not started ([gen2-x2-roadmap.md](gen2-x2-roadmap.md)).
- `pipe_cd` (Gowin GW5AST): Gen 1 only; 64-bit Link Layer at 62.5MHz (~430MB/s Bulk OUT, ~370MB/s
  IN); the GTR12 runtime register access (UPAR) does not complete yet; Gowin: no `--load` in the
  target (openFPGALoader).
- Gen 2: no elastic buffer (the Gen 2 RX path runs in the recovered clock domain), no Disable
  Scrambling.
- U1/U2 are rejected by the USB3 Device Core by default (LXU), accepted with `with_lpm` (simulation
  validated only, Gen 1 and Gen 2).
- The Acorn has no transceiver reference clock without a PCIe host: the 200MHz oscillator is routed
  through the fabric ([debugging-guide.md](debugging-guide.md)).
