# Gen 2x1 (10Gbps) / Gen 1x2 (2x5Gbps) Roadmap

**Status**: **Gen 1x1** (SuperSpeed, 5Gbps) and **Gen 2x1** (SuperSpeedPlus, 10Gbps) work on hardware
(Gen 2x1 on UltraScale+ GTY: enumeration, Bulk transfers up to ~880MB/s, full duplex, see the
[hardware milestones](#gen-2x1-hardware-milestones)). **Gen 1x2** (2x5Gbps): PHY in simulation only
(no SuperSpeedPlus Link Layer on two lanes, no hardware). **Gen 2x2** (2x10Gbps): not started.

This document lists what Gen 2x1 (SuperSpeedPlus 10Gbps, 128b/132b, one lane) and Gen 1x2
(SuperSpeedPlus 2x5Gbps, 8b/10b, two lanes) require, layer by layer, what was done and found during
the Gen 2x1 bring-up and what remains. References are to the USB 3.2 specification (Rev 1.1). PHY and
Device Core: [usb3-pipe.md](usb3-pipe.md), [usb3-core.md](usb3-core.md).

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
- **Datapath**: 10Gbps on 40-bit raw transceiver words at 250MHz (same TX/RX clocks as Gen 1 on
  20-bit words at 5Gbps), blocks (128-bit payloads) in the transceiver clock domains; Link Layer side:
  1.25GB/s = 64-bit at 156.25MHz or 128-bit at 78.125MHz (32-bit is not sustainable).

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

## Done in simulation (before the hardware bring-up)

The building blocks below were first developed and verified in simulation (Acorn GTP: 6.6Gbps max,
single lane), the Gen 2x1 ones were then validated on the XCKU3P ([hardware
milestones](#gen-2x1-hardware-milestones)); the Gen 1x2 ones are still simulation only.

### Gen 2x1 PHY building blocks (`usb3_pipe/common.py`/`coding.py`/`scrambling.py`/`datapath.py`)

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
- `Gen2BlockGearbox(data_width)`: 132-bit and 196-bit (SKP) blocks to 32/40/64-bit words.
- `Gen2BlockAligner(data_width)`: **SYNC based block alignment** (Unaligned/Aligned/Locked,
  6.4.1.2.4, data_width offsets searched per cycle, **realignment on a SYNC received with a different
  alignment**: a bit slip in TSEQ keeps valid looking Block Headers), **polarity inversion on SYNC**
  (6.4.2.2, the inverted SYNC is used for the alignment), Block Header single bit correction /
  undefined header -> Unaligned (6.3.2.4), **variable length SKP Ordered Sets** (0-36 SKP Symbols by
  groups of 4, LFSR Symbols extracted). Hardware structure: window of the last words, block read at a
  bit pointer in a 64 positions range (40-bit), registered per position decodes (Block Header, SKP
  Dwords/SKPEND, SYNC), short pointer loop, registered outputs.
- `Gen2RX`: descrambling (re-initialized on SYNC), TSEQ/TS1/TS2 detection (Symbols 14/15 not checked),
  Link Functionality, SDS -> Data Stream, per Ordered Set pulses (TSEQ/TS1/TS2/SYNC/SDS/unknown
  control block), register stages after the descrambler and the matching.
- `Gen2Scrambler(per_byte_advance=False)` for blocks: keys/next state from the current state (no per
  byte chain).
- `Gen2TXDatapath`/`Gen2RXDatapath` (`datapath.py`, `GEN2_DATA_WIDTH = 40`): transceiver words <->
  blocks at the hardware width in the transceiver TX/RX clock domains (Gen2TX -> register stage ->
  gearbox, aligner -> Gen2RX), continuous words (one per cycle, RX always ready), RX status and 32-bit
  counters (blocks, corrected headers, alignment losses, SYNC/TSEQ/TS1/TS2/SDS/SKP, unknown control
  blocks). `Gen2RXDebug` (`debug.py`): status/counters CSRs (sys clock domain), LiteScope probes list
  (`gen2_rx_analyzer_signals`).
- Tests (`test/test_scrambling.py`, `test/test_coding.py`, `test/test_datapath.py`, reference models
  in `test/model/gen2.py`): the TX output is decoded by an independent bit-serial reference model of
  the rules (TS contents, DC Balance Symbols, SKP LFSR Symbols = reference LFSR, SKP/SYNC scheduling,
  Data Blocks) at 32 and 40-bit; gearbox -> aligner roundtrip at 32/40/64-bit (SKP Ordered Sets of
  0-36 SKP Symbols, header bit error); `Gen2TXDatapath` -> channel model (`gen2_channel`: bit offset,
  polarity inversion, bit slips during TSEQ/TS1, Block Header bit error, SKP Ordered Sets
  shortened/lengthened) -> `Gen2RXDatapath` loopback at 32 and 40-bit with continuous words:
  realignment on the next SYNC, polarity detected, TSEQ/TS1/TS2/SDS detected, Data Blocks recovered,
  received SKP LFSR state equal to the RX descrambler LFSR, counters checked against the reference
  decode.

### Gen 1x2 PHY (`usb3_pipe/lfps.py`/`ltssm.py`/`phy.py`/`lanes.py`, `sim.py --lanes 2`)

- `lfps.py` (`SCDGenerator`/`SCDChecker`/`LBPMGenerator`/`LBPMChecker`) and `ltssm.py`
  (`SSPPolling`): **SuperSpeedPlus Polling** on the Configuration Lane: Polling.LFPS binary
  representation (tRepeat 6-9us = 0, 11-14us = 1, Table 6-32), **SCD1/SCD2** (6.9.4.2, LSb first),
  **LBPM** (PWM, delimiters, Table 6-33), Polling.LFPS (SCD1 handshake) -> Polling.LFPSPlus (SCD2 handshake, end of
  SCD) -> **Polling.PortMatch** (PHY Capability LBPM, Table 7-13, highest common capability Table 7-14,
  4 matched sent after 2 matched received) -> **Polling.PortConfig** (PHY Ready LBPM handshake, DFP
  bit 6, bit 7 Config Done) -> Polling.RxEQ. SuperSpeed partner (no SCD in 16 Polling.LFPS bursts):
  fallback to the SuperSpeed Polling.LFPS handshake. `USB3LTSSM(ssp=True)` runs it in Polling.LFPS
  (Gen 1x1 default unchanged). **Gen 2 capability**: `SSPPolling(gen2_capable=True)` advertises Gen 2
  (PHY Capability LBPM bit 2) and reports the negotiated `gen2`; `USB3PHY(lanes=1, gen2_capable=True)`
  runs the SuperSpeedPlus Polling on a single lane as a Gen 2x1 port (`phy.gen2`), then continues the
  training at Gen 1 speed (no speed switch yet). Tests: `test/test_ltssm_ssp.py` (Gen 2x1/Gen 2x1,
  Gen 2x2 host/Gen 2x1 device, Gen 1x1 SSP host/Gen 2x1 device, LBPMs decoded on the wire),
  `sim.py --gen2-capable` (`test/test_phy.py`: Gen 2 negotiated by both PHYs, then U0 at Gen 1).
- `phy.py` `USB3PHY(lanes=2)`: two `USB3PIPE` lanes (`x2_lane=True`) under one LTSSM: LFPS on Lane 0
  only (SuperSpeedPlus Polling LFPS through the Lane 0 `LFPSUnit`, Lane 1 in electrical idle,
  6.13.12), Ordered Sets on both lanes, TS1/TS2 required on both lanes (6.13.6), per lane scrambler
  seeds FFFFh/8000h (6.13.5), **per lane polarity** (6.13.7), per lane SKP removal, **lanes deskew**
  on the TS1/TS2 COM words (re-acquired after each TSEQ/TS1 phase, markers paired by arrival time
  within 1 word), **data striping** aligned to Lane 0 (6.13.4), 64-bit/8-bit link words.
- `lanes.py`: `LaneDeskew(max_skew)` pairs periodic markers by arrival timestamp.
- `sim.py --lanes 2` (Verilator): Host and Device `USB3PHY(lanes=2)` (4 SerDes models) negotiate
  Gen 1x2 (SCD1/SCD2, LBPM), train to U0 and exchange 64-bit PRBS packets without errors with
  lane-to-lane skew (Lane 0 or Lane 1 late, word shifts), Lane 1 TX polarity inverted, extra SKP
  Ordered Sets on a single lane at a time, +-300ppm (`test/test_phy.py` `TestPHYX2`, extra
  configurations with `pytest -m slow`).
- Unit tests: `test/test_lfps_ssp.py` (SCD, LBPM), `test/test_ltssm_ssp.py` (x2/x2 -> Gen 1x2,
  x2/x1 -> Gen 1x1, SS partner fallback), `test/test_lanes.py` (periodic marker deskew in both skew
  directions).

### Approximated / not verified

Confirmed on hardware since (Gen 2 host, [M1/M2](#gen-2x1-hardware-milestones)): the Gen 2 Block
Header bit order and SKP LFSR Symbols, the LBPM PHY Capability Gen 2 bit, the speed switch to Gen 2
after Polling.PortConfig, timing at 250MHz (`Gen2TX` DC Balance pipelined). Remaining:

- `Gen2BlockAligner`: a SYNC with a different alignment received while a SKP Ordered Set is consumed
  does not realign (the next SYNC does), polarity only toggled in Unaligned, SKP/SKPEND Symbols are
  matched exactly (no multi-bit error tolerance). DC Balance includes the Block Header bits.
- No Gen 2 elastic buffer (SKP add/remove at the receiver, the Gen 2 RX path runs in the recovered
  clock domain), no Disable Scrambling in Gen 2 operation.
- SSP Polling: no tPollingSCDLFPSTimeout, no re-timer presence announcement/RT Config (bit 7 always
  de-asserted), no Polling.PortMatch re-entry/fallback after a training failure (Table 7-15), no
  cPollingTimeout/eSS.Disabled handling.
- `USB3PHY(lanes=2)`: Gen 1x1 negotiated (x2 = 0) is reported but the datapath stays x2 (no fallback
  to a single lane datapath), the two lanes TX must run in lockstep (same clock, same SKP schedule),
  the Recovery/U1/U2/U3 flows use the Gen 1x1 LTSSM on the Configuration Lane (Ux exit, Ping.LFPS on
  Lane 0 as required, but not exercised in x2 simulation), Lane 1 Rx.Detect/Loopback/Compliance per
  lane not handled. The x2 simulation PRBS packets are separated by 0-3 cycles of Logical Idle:
  Logical Idle gaps of 4+ cycles with the extra SKP Ordered Sets on a single lane give Device PRBS
  errors (also seen with the former sim_x2.py, to investigate).
- Gen 1x2 Link Layer: the SuperSpeedPlus Link Layer (done for Gen 2x1, M3) and the 64-bit link
  words (done for Gen 2x1, M4b) are not connected to the x2 datapath yet (framing alignment of the
  striped 64-bit link words), the x2 simulation uses PRBS Link-Layer stand-ins.

### Hardware needed

- Gen 2x1: done on UltraScale+ GTY (alibaba_xcku3p target, SFP2USB adapter, USB 3.2 Gen 2 host);
  other 10Gbps transceivers (Kintex7 GTX -2/-3, UltraScale GTH) would need their Gen 2 SerDes
  clocking (`gen2_capable`, only on `USPUSB3SerDes` today).
- Gen 1x2/Gen 2x2: two transceivers per port with a **dual-lane USB Type-C** breakout (both SuperSpeed
  pairs routed, CC handling for the Configuration Lane) and a **USB 3.2 Gen 1x2/Gen 2x2 capable host**
  (Gen 2x2 hosts are rare: ASMedia ASM3242 based cards).

## Gen 2x1 hardware milestones

Hardware: Alibaba Cloud XCKU3P (UltraScale+ GTY, `bench/alibaba_xcku3p.py --gen2-capable`), SFP2USB
adapter, USB 3.2 Gen 2 xHCI root hub (a Gen 1 only root hub negotiates Gen 1, as expected).

**M1: Gen 2 capability exchange and Gen 2 RX** (done): the device advertises Gen 2x1 in the
SuperSpeedPlus Polling, the host selects Gen 2 and the training continues at 10Gbps
(`USPUSB3SerDes(gen2_capable=True)`: QPLL at 10Gbps, 40-bit raw words, Gen 1 at the /2 rate, RX
re-initialization on rate changes). The host Gen 2 SYNC/TSEQ/TS1/SKP Ordered Sets are block-aligned,
descrambled and decoded (Gen 2 bit order, polarity inversion, SKP LFSR Symbols confirmed): 31.6M
blocks, 146 corrected Block Headers over 4 host Gen 2 attempts. Found on hardware:
- `popcount` compared to constants was evaluated on 1-bit operands in the generated Verilog (all
  Block Headers seen as corrected Control headers, no alignment loss on undefined headers), not
  visible in the Migen simulation: fixed, tested with yosys on the generated Verilog.
- GTY CDR: the `USPUSB3SerDes` RXCDR_CFG2/3 overrides (0x263/0x1a) also track the host SSC at 10Gbps
  (as the PCIe Gen3 0x164/0x12), the Transceivers Wizard 10Gbps default (0x269) does not.

**M2: Gen 2 link training to U0** (done): `Gen2PIPE` (Gen 2 TSEQ/TS1/TS2 bursts, 8 consecutive
TS1/TS2 detection, SDS then Logical Idle, Idle Symbol handshake) under the LTSSM when Gen 2 is
negotiated (`USB3PHY.add_gen2`: 10Gbps once Gen 2 is negotiated, LFPS stay on the Gen 1 PIPE). On
hardware: Polling.RxEQ (524288 TSEQ, 6.9ms) -> Polling.Active -> Polling.Configuration ->
Polling.Idle -> **U0 at 10Gbps**, Recovery at Gen 2 to U0. Found on hardware: the Gen 2 Idle Symbol is
5Ah (7.2.3, the host Data Blocks after SDS descramble to 5Ah). Simulation: `sim.py --gen2` (Host and
Device to U0 at Gen 2). The Gen 1 Link Layer receives nothing in U0 at Gen 2: the link goes to
Recovery a few us after U0 (no Header Sequence Number Advertisement exchange).

**M3: SuperSpeedPlus Link Layer, enumeration and Bulk transfers at Gen 2** (done): Gen 2 packet
framing (`usb3_pipe/framing.py`: Link Layer words <-> Data Blocks, framing Symbols as scrambled bytes,
DPHSTART/Length Field Replicas, store and forward of the TX units) and SuperSpeedPlus Link Layer
(`usb3_core/link.py` at Gen 2: 4-bit Header Sequence Numbers, Type 1/Type 2 credits, Transfer Types),
SuperSpeedPlus USB Device Capability in the BOS. On hardware (`lsusb`: 10000M, SuperSpeedPlus):
Bulk IN ~320MB/s, Bulk OUT ~466MB/s, full duplex 98MB/s IN + 457MB/s OUT (60s), no errors, no
Recovery. Found on hardware:
- Warm Reset LFPS detected from the 8b/10b decode errors at 10Gbps (always set on raw words): Gen 2
  block alignment lost instead.
- Verilog expression width of the RX framing byte shifts (`8*p` truncated, as `popcount` in M1).
- PENDING_HP_TIMER is 10us (Table 7-7), Bulk IN stale ERDY race, SKP Ordered Sets starvation with
  units packed back to back (host Recovery every few ms in full duplex).

**M4a: Gen 2 full duplex throughput and robustness** (done): measured with a throughput profile
(`bench/usb3_soc.py` `add_usb3_profile`: Link Layer TX/Bulk IN cycles, Tx Header Buffers hold, LGOOD_n
round trip, Bulk IN ACK TP latency) and the Link Layer/core traces:
- Type 1/Type 2 Tx Header Buffers (4 + 4, 7.2.4.1): the Bulk OUT ACK TPs no longer block the Bulk IN DPHs.
- LGOOD_n/LCRD_x received from the Gen 2 RX framing without waiting behind the RX words (~7us with
  Bulk OUT DPs queued: Tx Header Buffers full, credits missing).
- Nullified DPP after a retransmitted DPH (Gen 2 framing: a DPP after each DPH).
- Gen2TXPacker: Logical Idle Data Block inside a unit when a block was requested while the next unit
  was started in a partially filled block (rare LBADs, Recovery/transfer errors in full duplex).

XCKU3P, 64KB transfers: Bulk IN 426MB/s, Bulk OUT 466MB/s, full duplex 325MB/s IN + 430MB/s OUT (120s,
0 errors, 0 LBAD, 0 Recovery; was 100 + 457MB/s). With 1MB transfers the host is slower on Bulk IN
(~330MB/s, host side). Bulk IN bursts are limited by the 32-bit Link Layer TX (one 1024-byte DP every
~2.2us, ~470MB/s).

**M4b: 64-bit Link Layer words at Gen 2** (done): `USB3PHY(link_words=2)` Link Layer interface in
words pairs (Gen 2 framing 8 bytes per cycle in the DPPs, adapters to the Gen 1 PIPE words), Link Layer
DPP words of 64-bit (RX words window, TX DPPSplitter, CRC-32 on 8 bytes), 64-bit Bulk endpoints/packet
slots (`data_width=64`), 32-bit endpoints (Control Endpoint) through Protocol Layer adapters. Bench:
`--link-words 2`. XCKU3P, 64KB transfers, 0 errors:

| Link Layer        | Bulk IN  | Bulk OUT | Full duplex (IN + OUT)     |
|-------------------|----------|----------|----------------------------|
| 32-bit            | 426MB/s  | 466MB/s  | 325 + 430MB/s              |
| 64-bit            | 744MB/s  | 881MB/s  | 619 + 736MB/s (120s soak)  |
| 64-bit, MaxBurst 16 | 785MB/s | 873MB/s | 644 + 757MB/s             |

Gen 1 operation of the same design (Gen 2 capability cleared, `usb3_gen2_capable` bench CSR): 441MB/s
IN/OUT. Limits: Bulk OUT ~87% of the 64-bit words at 125MHz (1GB/s), Bulk IN waiting for the host
requests (ACK TPs behind the RX DPPs), the host Bulk IN rate with large transfers (~330MB/s at 1MB).

**M4c: Gen 2 robustness** (done): link retraining at Gen 2 after a PHY reset (Gen2PIPE tx/rx clock
domains reset with the PHY reset; no stale LGOOD_n before the Advertisement at the U0 entry), Gen 1
operation of a Gen 2 capable design, U1/U2 entries/exits with Recovery at Gen 2 (simulation: the bench
host, an xHCI without hardware LPM support in Linux, never enables U1/U2). On hardware: 10 PHY resets
alternating Gen 2/Gen 1 trainings, traffic checked after each (one Gen 2 -> Gen 1 capability change
needed a host port power cycle: capability changes between trainings are a test-only case).

## Next steps

1. Gen 2x1: Bulk OUT at ~87% of the 64-bit Link Layer words (8 bytes per cycle at 125MHz, ~1GB/s vs
   ~1.18GB/s on the line): wider/faster Link Layer words for the last ~25%; 64-bit words for the other
   endpoints/device classes (UVC, MSC); U1/U2 on hardware (not enabled by the bench host); Gen 2
   SerDes clocking on Kintex7 GTX/UltraScale GTH.
2. Gen 1x2 on Artix7/Kintex7 with a Type-C breakout: PHY done in simulation (`USB3PHY(lanes=2)`);
   remaining: single lane fallback, SSP Link Layer on the x2 datapath, Type-C CC/Configuration Lane,
   hardware validation.
3. Gen 2x2 (20Gbps) on UltraScale+ GTY: combination of both, 7 buffers.
4. Gen 2x1 on Gowin GW5AST (Tang Mega 138K Pro, `bench/sipeed_tang_mega_138k_pro.py`): Gen 1 is
   validated on hardware with the PIPE in its own clock domain (`pipe_cd`, GTR12 TX clock at 125MHz)
   and the LTSSM/Link Layer/Core at 62.5MHz on 64-bit words. Remaining for 10Gbps:
   - GTR12 rate switch: 5Gbps -> 10Gbps is 6 register writes (CDR divider, QPLL1 output divider, TX
     lanes clock tree, TX/RX gear FIFOs: 1:2 -> 1:4) with the same 5GHz VCO and 125MHz fabric clock
     (40-bit -> 80-bit raw words), back to 5Gbps for the next SuperSpeedPlus Polling. Done:
     `GW5RateSequencer` writes them through the GTR12 runtime register access (UPAR, LiteICLink
     `GTR12UPAR`) with the lane held in reset (`GW5USB3SerDes(gen2_capable=True)`).
   - 80-bit raw words in LiteICLink's GTR12 (8b/10b on 40 bits at 5Gbps, raw 80 bits at 10Gbps) and
     `GW5USB3SerDes` as a Gen 2 capable SerDes (`gen2_data_width = 80`).
   - Gen 2 with `pipe_cd` (follow-up branch `gw5/gen2`): done in simulation (`sim.py --gen2 --dev-pipe-cd`, Gen2PIPE in the PIPE
     clock domain, RX credits throttling sized for Gen 2), including 80-bit raw words for the
     gearbox/aligner and the TX DC balance (`Gen2TXDCBalance(one_per_cycle=True)`).
   - Gen 2 framing at 16 bytes per cycle (done in simulation): with raw words wider than 40 bits
     (`Gen2PIPE`), `Gen2RXFraming` processes 16 bytes per cycle (4 words entries, narrowed to words
     pairs after the RX buffer), the framer entries are paired before the TX units buffer and
     `Gen2TXPacker` reads 16 bytes per cycle (80-bit words at 125MHz: a Data Block every 1.65
     cycles). The block aligner consumes up to 4 SKP Dwords per cycle on words wider than 64 bits
     (else the pointer falls behind in long SKP Ordered Sets, alignment lost). `sim.py --gen2
     --dev-pipe-cd --gen2-data-width 80` (also with `--host-itp`/`--ssc`): host script done, no
     Recovery.
   - Throughput: the 64-bit Link Layer at 62.5MHz is ~500MB/s (credit throttled at Gen 2, about half
     of the 10Gbps rate): 128-bit Link Layer words (or a faster Link Layer clock) for full rate.
   - Timing (in progress): the 80-bit Gen 2 datapaths at 125MHz on the GW5A fabric. Paths are
     routing dominated (about 1.3ns per hop, 2/3 of the delays): restructured for few logic levels
     and low fanout (one-hot block aligner pointer with per transition decodes registered and
     shifted with the window, `Gen2TXPacker16`, gearbox and DC balance loops on registered values,
     registered valid/ready buffers with `usb3_pipe.common.PipeReady`, data registers without
     reset). Gen 2 capable Tang Mega 138K Pro build: tx/rx 41 -> 81-95MHz, sys 45 -> 55-57MHz
     (125/62.5MHz needed, about +-15% between place and route runs, Gowin `route_option 0`: the
     timing driven router leaves unrouted nets). Remaining: the loops of the 16 bytes framing (RX
     framing byte pointer/window, packer fill level), gearbox and DC balance at 125MHz, and the
     Link Layer at 62.5MHz; a framing at 32 bytes per cycle in the 62.5MHz domain could be needed.
