# USB3 PHY (`usb3_pipe`)

USB 3.2 Gen 1x1 (SuperSpeed, 5Gbps) PHY on FPGA transceivers: a PIPE-like interface (LFPS, Training
Sequences, 8b/10b, scrambling, clock compensation) and the LTSSM, in Migen/LiteX. It provides a
link-trained 32-bit symbol stream (with K-symbol flags) in the `sys` clock domain (>= 125MHz) to a
USB3 Link Layer: the USB3 Device Core ([usb3-core.md](usb3-core.md)) or any other Link Layer through
the standalone core ([standalone-core.md](standalone-core.md)).

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
| `USB3SerDes`: `A7USB3SerDes`/`K7USB3SerDes`/`USPUSB3SerDes` | `serdes.py` | Common SerDes on LiteICLink transceivers (Artix7 GTP/Kintex7 GTX/Ultrascale+ GTY) at 5Gbps with their USB3 settings (SSC tolerant CDR, LFPS/OOB): 8b/10b, polarity, electrical idle/LFPS patterns and the TX/RX datapaths below. |
| `TXDatapath`/`RXDatapath` (`TXSKPInserter`, `RXSKPRemover`, `RXWordAligner`, `RXErrorSubstitution`, `RXErrorMonitor`, `RXIdleFilter`), `Gen2TX`/`Gen2RX` | `datapath.py` | Gen 1: SKP insertion (TX, 6.4.3) and removal (RX), word alignment on COM, 8b/10b errors substitution (6.3.1.4)/monitoring, electrical idle filtering, CDC/data-width. Gen 2: block transmitter/receiver (TSEQ/TS1/TS2/SYNC/SDS/SKP Ordered Sets, Data Blocks, 6.3.2.3 scrambling rules). |
| `RX8b10bChecker`, `Gen2BlockGearbox`/`Gen2BlockAligner` | `coding.py` | 8b/10b Python model/code tables and RX checker (6.3.1), 128b/132b blocks gearbox/aligner and DC balance helpers (6.3.2, 6.4.1.2). |
| Symbols/Ordered Sets | `common.py` | Gen 1 Symbols and TS1/TS2/TSEQ, Link Configuration, Gen 2 Block Headers, Symbols/Ordered Sets identifiers and payloads. |
| `LFPSUnit` | `lfps.py` | LFPS generation/detection (Polling, Ping, U1/U2 exit, U3 wakeup, Warm Reset), timings of 6.9. SuperSpeedPlus LFPS signaling: SCD1/SCD2 and LBPM generators/checkers (6.9.4/6.9.5). |
| `TSUnit` | `training.py` | TSEQ/TS1/TS2 generation and detection (Link Configuration: reset, loopback, scrambling). |
| `Scrambler`/`Descrambler`, `Gen2Scrambler` | `scrambling.py` | Gen 1 scrambler (6.3.1.3, symbol level: SKP does not advance, COM initializes), Gen 2 scrambler (6.3.2.3). |
| `USB3PIPE` | `core.py` | PIPE: LFPS/TS units, scrambling, Logical Idle, Idle handshake, framing alignment (SHP/SDP/SLC). |
| `USB3LTSSM`, `SSPPolling` | `ltssm.py` | LTSSM (7.5): Polling, U0, Recovery, U1/U2/U3, Hot/Warm Reset, Compliance, Loopback, eSS.Inactive, PM handshake, timers. SuperSpeedPlus Polling substates (SCD/LBPM, PortMatch/PortConfig, 7.5.4.3-7.5.4.6) used with `USB3LTSSM(ssp=True)`. |
| `USB3PHY` | `phy.py` | PIPE + LTSSM with the Link interface (below). `lanes=2`: Gen 1x2 (one PIPE per lane, SuperSpeedPlus Polling, lanes deskew/striping, 64-bit link words, [gen2-x2-roadmap.md](gen2-x2-roadmap.md)). |
| `LaneStriper`/`LaneDestriper`/`LaneDeskew` | `lanes.py` | Gen 1x2 lanes striping/deskew (6.13). |
| `USB3PHYDebug`/`USB3SerDesDebug` | `debug.py` | State history, counters, measurements CSRs ([debugging-guide.md](debugging-guide.md)). |
| `usb3_pipe_gen` | `gen.py` | Standalone Verilog core generator. |

## Link interface (`USB3PHY`)

What a USB3 Link Layer needs from the PHY (the USB3 Device Core only uses this interface):

| Signals | Direction | Description |
|---|---|---|
| `sink` (data 32, ctrl 4) | Link -> PHY | TX symbols (first symbol on `data[31:24]`), Logical Idle sent when `sink.valid` is low, `sink.ready` low during SKP insertions. |
| `source` (data 32, ctrl 4) | PHY -> Link | RX symbols, framing words aligned, no back-pressure. |
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
$ ./sim.py --trace               # VCD (build/sim/gateware/sim.vcd).
$ pytest test                    # Unit/system tests (-n auto: parallel with pytest-xdist).
$ pytest test -m slow            # Long simulations (Verilator), not run by default.
```

## Limitations

- Gen 1x1 only on hardware (Gen 1x2: simulation only, Gen 2: building blocks,
  [gen2-x2-roadmap.md](gen2-x2-roadmap.md)).
- U1/U2 are rejected by the USB3 Device Core by default (LXU), accepted with `with_lpm` (simulation
  validated only).
- The Acorn has no transceiver reference clock without a PCIe host: the 200MHz oscillator is routed
  through the fabric ([debugging-guide.md](debugging-guide.md)).
