# Gen 2x1 (10Gbps) / Gen 1x2 (2x5Gbps) Roadmap

USB3PIPE implements USB 3.2 **Gen 1x1** (SuperSpeed, 5Gbps, 8b/10b, one lane). This document lists what
**Gen 2x1** (SuperSpeedPlus 10Gbps, 128b/132b, one lane) and **Gen 1x2** (SuperSpeedPlus 2x5Gbps,
8b/10b, two lanes) require, layer by layer, and what is already prepared. References are to the
USB 3.2 specification (Rev 1.1).

Both are **SuperSpeedPlus** operation: they change the PHY *and* the Link Layer (Type 1/Type 2 traffic
classes, 4-bit Header Sequence Numbers, Rx Buffer credits), the Protocol Layer is mostly unchanged.

## Hardware

| Mode     | Line rate     | Transceivers                         | Connector                          |
|----------|---------------|--------------------------------------|------------------------------------|
| Gen 1x1  | 5Gbps         | Artix7 GTP (Acorn), Kintex7 GTX      | Type-A/B, SFP2USB adapter          |
| Gen 2x1  | 10Gbps        | Kintex7 GTX (-2/-3), UltraScale GTH/GTY (GTP max 6.6Gbps: not possible on Artix7) | Type-A/B (10Gbps capable) or Type-C |
| Gen 1x2  | 2x5Gbps       | 2 GTP/GTX (Artix7 possible)          | **Type-C only** (two lanes, Configuration Lane from CC) |

The SFP2USB adapter carries a single lane: Gen 1x2 needs a Type-C breakout with both SuperSpeed lane
pairs routed to transceivers (and CC handling for the Configuration Lane/orientation).

## PHY (usb3_pipe)

### Gen 2x1
- **128b/132b block code** (6.3.2): 4-bit Block Header (`0011b` data, `1100b` control, single bit
  error correction), 128-bit payload, SKP control block with a 192-bit payload. Fabric gearbox
  (132-bit blocks on a 32/40/64-bit transceiver interface: GTX/GTH 64b/66b gearbox not usable) and
  **block aligner** (header search, Unaligned/Aligned/Locked phases, 6.4.x).
- **Gen 2 scrambler** (6.3.2.3): G(X) = X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1, seed 1DBFBCh,
  advance/bypass rules per block type, SYNC Ordered Set re-initialization.
- **Ordered Sets**: Gen 2 TS1/TS2/TSEQ (control blocks, symbols 14/15 DC balance), SYNC, SDS (Start
  of Data Stream), Gen 2 SKP (SKPEND, LFSR in SKP, 6.4.3.2).
- **Speed negotiation** (7.5.4): SCD1/SCD2 LFPS patterns in Polling.LFPS, **LBPM** (LFPS Based PWM
  Messaging) in Polling.PortMatch/PortConfig, Gen 1 fallback. New LTSSM substates.
- **Datapath**: 10Gbps = 1.25GB/s: 64-bit at 156.25MHz or 128-bit at 78.125MHz (32-bit is not
  sustainable).

### Gen 1x2
- Two `USB3SerDes`/PIPE lanes: 8b/10b, SKP, word alignment and polarity **per lane** (6.13.7).
- **Data striping** (6.13.4): data symbols striped across lanes aligned to Lane 0, control blocks /
  Ordered Sets duplicated on both lanes (6.13.6); **lane deskew** (Table 6-34: up to 6.4ns at the
  receiver, a few symbols) using the duplicated Ordered Sets (TS1/TS2, SKP).
- **Per lane scramblers** (6.13.5): Gen 1 seeds FFFFh (Lane 0) / 8000h (Lane 1).
- LFPS, Rx.Detect and Ux exit on the Configuration Lane only (6.13.10-6.13.13). x2 capability
  advertised/negotiated with LBPM (Polling.PortMatch, 6.13.2).
- Datapath: 2x32-bit at 125MHz (64-bit link words).

## Link Layer (usb3_core.link)

SuperSpeedPlus (7.2.4.1, 7.2.1):
- **4-bit Header Sequence Numbers** (modulo 16), LGOOD_n with 4-bit n.
- **Type 1 / Type 2 traffic classes** with separate Tx Header Buffers and **Rx Buffers credits**
  (LCRD1_x/LCRD2_x, 4 buffers in Gen 2x1/Gen 1x2, 7 in Gen 2x2), Type 2 = DPH (with DPP).
- Header Packets with a different Link Control Word, DPP padding/alignment rules, **nullified DPPs**
  on retransmission (7.2.1.2.2).
- Arbitration weights, Deferred/Delayed handling of the SSP hub model (device: optional).
- Timers values per speed (Table 7-7).

The native Link Layer is structured to make these changes local: sequence numbers/credits/buffers are
parameters of the flow control state, the framer/parser are word based (data_width).

## Protocol Layer (usb3_core.protocol/endpoints/control)

- Largely unchanged (TP/DP formats are the same). SuperSpeedPlus additions: Isochronous timestamp
  changes, PTM (optional), Arbitration Weight in DPHs.
- Bandwidth: Gen 2x1 needs 64-bit endpoints datapaths (or two 32-bit at 2x the clock).
- Descriptors: bcdUSB 0x0320, SuperSpeedPlus USB Device Capability (sublink speed attributes) in BOS.

## Prepared (this branch)

- `usb3_pipe/gen2.py`:
  - `Gen2Scrambler`: Gen 2 LFSR (X^23 polynomial, seed 1DBFBCh), parallel on N bytes, advance/xor,
    advance/bypass, hold modes.
  - `Gen2BlockGearbox`/`Gen2BlockAligner`: 132-bit blocks (Block Header + payload) to/from 32-bit
    transceiver words, header single bit error correction, block alignment (header search/slip).
- `usb3_pipe/lanes.py`:
  - `LaneStriper`/`LaneDestriper`: Gen 1x2 symbol striping (Lane 0 aligned) of link words over two
    lanes, ordered sets duplicated.
  - `LaneDeskew`: lanes deskew on a reference marker (COM/framing), per lane delay lines.
- Unit tests against Python reference models (`test/test_gen2.py`, `test/test_lanes.py`).

**Not done / to verify with hardware or captures**: the Gen 2 LFSR bit ordering is implemented from
the polynomial and the serialization order (LSB first) of 6.3.2; it must be checked against Figure 6-12
(or a Gen 2 capture) before use. No Gen 2/x2 LTSSM, LBPM, SSP Link Layer yet.

## Suggested order

1. Gen 1x2 on Artix7/Kintex7 with a Type-C breakout: per-lane PHYs + striping/deskew + LBPM + SSP Link
   Layer (reuses the 8b/10b PHY, validates the SSP Link Layer at 5Gbps).
2. Gen 2x1 on Kintex7 GTX/UltraScale GTH: 128b/132b PHY (gearbox, aligner, scrambler, Gen 2 Ordered
   Sets), SCD/LBPM speed negotiation, 64-bit datapath.
3. Gen 2x2 (20Gbps) on UltraScale+ GTY: combination of both, 7 buffers.
