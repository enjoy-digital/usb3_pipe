# Gen 2x1 (10Gbps) / Gen 1x2 (2x5Gbps) Roadmap

USB3PIPE implements USB 3.2 **Gen 1x1** (SuperSpeed, 5Gbps, 8b/10b, one lane). This document lists what
**Gen 2x1** (SuperSpeedPlus 10Gbps, 128b/132b, one lane) and **Gen 1x2** (SuperSpeedPlus 2x5Gbps,
8b/10b, two lanes) require, layer by layer, and what is already prepared. References are to the
USB 3.2 specification (Rev 1.1). Current Gen 1x1 PHY and Device Core: [usb3-pipe.md](usb3-pipe.md),
[usb3-core.md](usb3-core.md).

Both are **SuperSpeedPlus** operation: they change the PHY *and* the Link Layer (Type 1/Type 2 traffic
classes, 4-bit Header Sequence Numbers, Rx Buffer credits), the Protocol Layer is mostly unchanged.

## Hardware

| Mode     | Line rate     | Transceivers                         | Connector                          |
|----------|---------------|--------------------------------------|------------------------------------|
| Gen 1x1  | 5Gbps         | Artix7 GTP (Acorn), Kintex7 GTX, Ultrascale+ GTY | Type-A/B, SFP2USB adapter |
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

## Done (simulation only)

There is no Gen 2 (10Gbps) or dual-lane hardware on the current benches (Acorn GTP: 6.6Gbps max, single
lane): everything below is verified in simulation only.

### Gen 2x1 PHY building blocks (`usb3_pipe/gen2.py`, `test/test_gen2.py`)

- `Gen2Scrambler`: LFSR checked against **Figure 6-12** (Galois: D0 <- D22, D2/D5/D8/D16/D21 <- D(n-1)
  ^ D22, Data Out = Data In ^ D22, seed 1DBFBCh with D22 = bit 22), the test re-implements the figure
  register by register. Bit order from **Figure 6-11** (Block Header bit 0 first, Symbols LSB first).
- `Gen2TX`: block transmitter with the **6.3.2.3 scrambling rules** (Block Header bypass, TS Symbol 0
  bypass/advance, TS Symbols 1-13 scrambled, SDS bypass/advance, SKP bypass/no advance, SYNC bypass +
  LFSR re-initialization), **Gen 2 TSEQ/TS1/TS2/SYNC/SDS Ordered Sets** (Tables 6-7 to 6-11),
  **DC Balance** Symbols 14/15 (6.4.1.2.2 algorithm, running balance saturated at +-511), **SYNC
  every 32 TS1/TS2 / 16384 TSEQ** (6.4.1.2.1), **Gen 2 SKP Ordered Sets** (6.4.3.3: Y/40 scheduling
  with remainder, up to 2 pending, after SYNC/TS1/TS2/SDS/Logical Idle/end of packet only, never after
  TSEQ, SYNC priority, 20 SKP + SKPEND + LFSR Symbols of Table 6-13), SDS then Data Blocks (Logical
  Idle 00h blocks when no data).
- `Gen2BlockGearbox`: 132-bit and 196-bit (SKP) blocks to 32-bit words.
- `Gen2BlockAligner`: **SYNC based block alignment** (Unaligned/Aligned/Locked, 6.4.1.2.4, 32 offsets
  searched per cycle), **polarity inversion on SYNC** (6.4.2.2), Block Header single bit correction /
  undefined header -> Unaligned (6.3.2.4), **variable length SKP Ordered Sets** (0-36 SKP Symbols by
  groups of 4, LFSR Symbols extracted).
- `Gen2RX`: descrambling (re-initialized on SYNC), TSEQ/TS1/TS2 detection (Symbols 14/15 not checked),
  Link Functionality, SDS -> Data Stream.
- Tests: the TX output is decoded by an independent bit-serial reference model of the rules (TS
  contents, DC Balance Symbols, SKP LFSR Symbols = reference LFSR, SKP/SYNC scheduling, Data Blocks);
  TX -> channel (bit offset, polarity inversion, bit slip during TS1, SKP Ordered Sets shortened/
  lengthened by 4/16/20 SKP Symbols) -> RX loopback: realignment on the next SYNC, polarity detected,
  TS1/TS2/SDS detected, Data Blocks recovered, received SKP LFSR state equal to the RX descrambler LFSR.

### Gen 1x2 PHY (`usb3_pipe/ssp.py`, `usb3_pipe/x2.py`, `usb3_pipe/lanes.py`, `sim_x2.py`)

- `ssp.py`: **SuperSpeedPlus Polling** on the Configuration Lane: Polling.LFPS binary representation
  (tRepeat 6-9us = 0, 11-14us = 1, Table 6-32), **SCD1/SCD2** (6.9.4.2, LSb first), **LBPM** (PWM,
  delimiters, Table 6-33), Polling.LFPS (SCD1 handshake) -> Polling.LFPSPlus (SCD2 handshake, end of
  SCD) -> **Polling.PortMatch** (PHY Capability LBPM, Table 7-13, highest common capability Table 7-14,
  4 matched sent after 2 matched received) -> **Polling.PortConfig** (PHY Ready LBPM handshake, DFP
  bit 6, bit 7 Config Done) -> Polling.RxEQ. SuperSpeed partner (no SCD in 16 Polling.LFPS bursts):
  fallback to the SuperSpeed Polling.LFPS handshake. `USB3LTSSM(ssp=True)` runs it in Polling.LFPS
  (Gen 1x1 default unchanged).
- `x2.py` `USB3X2PHY`: two `USB3PIPE` lanes (`x2_lane=True`) under one LTSSM: LFPS on Lane 0 only
  (Lane 1 in electrical idle, 6.13.12), Ordered Sets on both lanes, TS1/TS2 required on both lanes
  (6.13.6), per lane scrambler seeds FFFFh/8000h (6.13.5), **per lane polarity** (6.13.7), per lane SKP
  removal, **lanes deskew** on the TS1/TS2 COM words (re-acquired after each TSEQ/TS1 phase, markers
  paired by arrival time within 1 word), **data striping** aligned to Lane 0 (6.13.4), 64-bit/8-bit
  link words.
- `lanes.py`: `LaneDeskew(max_skew)` pairs periodic markers by arrival timestamp.
- `sim_x2.py` (Verilator): Host and Device `USB3X2PHY` (4 SerDes models) negotiate Gen 1x2 (SCD1/SCD2,
  LBPM), train to U0 and exchange 64-bit PRBS packets without errors with lane-to-lane skew (Lane 0 or
  Lane 1 late, word shifts), Lane 1 TX polarity inverted, extra SKP Ordered Sets on a single lane at a
  time, +-300ppm (`test/test_phy_x2.py`, extra configurations with `pytest -m slow`).
- Unit tests: `test/test_ssp.py` (SCD, LBPM, x2/x2 -> Gen 1x2, x2/x1 -> Gen 1x1, SS partner fallback),
  `test/test_lanes.py` (periodic marker deskew in both skew directions).

### Approximated / not verified

- Gen 2 Block Header bit order: "0011b" read with bit 0 as LSB, bit 0 transmitted first (Figure 6-11);
  SKP LFSR Symbol 4N+1 bit 7 = ~LFSR[22] as Table 6-13 (6.4.3.3 rule 6 mentions an even parity
  instead). To check against a Gen 2 capture.
- `Gen2BlockAligner` does not re-align on a SYNC received with a different alignment in the Aligned
  phase (alignment only acquired in Unaligned), SKP/SKPEND Symbols are matched exactly (no multi-bit
  error tolerance). DC Balance includes the Block Header bits.
- `Gen2TX`/`Gen2RX` are block-level (one block per cycle), not yet a 64-bit 156.25MHz datapath, no Gen 2
  elastic buffer (SKP add/remove at the receiver), no Gen 2 LTSSM integration (TSEQ/TS count, SYNC
  in Polling/Recovery, Disable Scrambling from the first Data Block).
- SSP Polling: no tPollingSCDLFPSTimeout, no Gen 2 capability/speed switch, no re-timer presence
  announcement/RT Config (bit 7 always de-asserted), no Polling.PortMatch re-entry/fallback after a
  training failure (Table 7-15), no cPollingTimeout/eSS.Disabled handling.
- `USB3X2PHY`: Gen 1x1 negotiated (x2 = 0) is reported but the datapath stays x2 (no fallback to a
  single lane datapath), the two lanes TX must run in lockstep (same clock, same SKP schedule), the
  Recovery/U1/U2/U3 flows use the Gen 1x1 LTSSM on the Configuration Lane (Ux exit, Ping.LFPS on Lane
  0 as required, but not exercised in x2 simulation), Lane 1 Rx.Detect/Loopback/Compliance per lane
  not handled.
- SuperSpeedPlus Link Layer (Type 1/Type 2 traffic, 4-bit HSEQ, Rx Buffer credits, DPHP, framing
  alignment of the 64-bit link words): not started, the x2 simulation uses PRBS Link-Layer stand-ins.

### Hardware needed

- Gen 2x1: 10Gbps transceivers (Kintex7 GTX -2/-3, UltraScale GTH (ex: KCU105), UltraScale+ GTY (the
  alibaba_xcku3p target)) with a 10Gbps capable Type-A/B or Type-C connection and a USB 3.2 Gen 2 host.
- Gen 1x2/Gen 2x2: two transceivers per port with a **dual-lane USB Type-C** breakout (both SuperSpeed
  pairs routed, CC handling for the Configuration Lane) and a **USB 3.2 Gen 1x2/Gen 2x2 capable host**
  (Gen 2x2 hosts are rare: ASMedia ASM3242 based cards).
- A protocol analyzer/capture (or a Gen 2 PHY with PIPE access) to validate the Gen 2 bit order/LFSR
  against real traffic.

## Suggested order

1. Gen 1x2 on Artix7/Kintex7 with a Type-C breakout: PHY done in simulation (`USB3X2PHY`); remaining:
   single lane fallback, SSP Link Layer, Type-C CC/Configuration Lane, hardware validation.
2. Gen 2x1 on Kintex7 GTX/UltraScale GTH: 128b/132b blocks done in simulation; remaining: Gen 2 LTSSM
   integration (Gen 2 capability in LBPM, SYNC/TS counts, SDS), 64-bit datapath, elastic buffer, Gen 2
   transceiver configuration.
3. Gen 2x2 (20Gbps) on UltraScale+ GTY: combination of both, 7 buffers.
