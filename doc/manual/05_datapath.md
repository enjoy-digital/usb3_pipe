# Datapath {#datapath}

This chapter follows a symbol from the link layer to the transceiver and back. It describes the
transmit and receive datapaths of the PHY at Gen 1 (8b/10b, 5 Gbps) and at Gen 2 (128b/132b,
10 Gbps): coding, scrambling, clock compensation, alignment, packet framing and the clock domain
crossings. Read it to understand what the PHY does to the data once the link is trained, or to
debug a datapath counter.

The concepts (symbols, K codes, Ordered Sets, scrambling, SKP) are introduced in
[Symbols, coding and scrambling](#primer-coding); the training that brings the link to U0 is in
[Link training](#link-training).

## At a glance {#dp-glance}

| Item | Gen 1 (5 Gbps) | Gen 2 (10 Gbps) |
|------------------------|----------------------------------|----------------------------------|
| Line code | 8b/10b, in the transceiver | 128b/132b, in the fabric |
| Unit on the wire | Symbol: 8 bits + K flag | Block: 4-bit header + 16 Symbols |
| Scrambler | X^16 + X^5 + X^4 + X^3 + 1 | X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1 |
| Clock compensation | SKP Ordered Sets (2 Symbols) | SKP Ordered Sets (SKPEND, LFSR) |
| Packet framing | K Symbols at word boundaries | Scrambled bytes, any position |
| Transceiver words | 20 or 40 bits | 40 or 80 raw bits |
| Link layer words | 32-bit (64-bit with `link_words=2`) | 64-bit (128-bit with `pipe_cd`) |
| Classes | `TXDatapath`, `RXDatapath` | `Gen2TXDatapath`, `Gen2RXDatapath` |

The Gen 1 datapath is split between two modules. The transceiver wrapper (`USB3SerDes`, see
[Transceivers](#transceivers)) holds the parts tied to the transceiver: width conversion, SKP
insertion and removal, clock domain crossing and word alignment. The PIPE (`USB3PIPE`) holds
scrambling, Logical Idle, Training Sequences and the framing alignment. The chapter follows Gen 1
first, then shows what Gen 2 changes.

## Gen 1 symbols and words {#dp-gen1-words}

At Gen 1 the line carries 10-bit codes at 5 Gbps, that is 500 million symbols per second. Each
symbol decodes to a byte and a flag telling a data symbol (Dxx.y) from a control symbol (Kxx.y).
The PHY moves four symbols per cycle: a 32-bit data word with a 4-bit control mask, at 125 MHz in
the `sys` clock domain.

At the link interface, the first symbol of a word is on `data[31:24]`. Inside the PHY the order
is the reverse, symbol 0 on `data[7:0]`, and a byte swap at the edge of the PIPE converts between
the two (`EndiannessSwap`, disabled with `with_endianness_swap=False`).

The 8b/10b encoder and decoder are those of the transceiver or of LiteICLink: the PHY does not
re-implement them. It adds the checks that USB 3 asks for on reception, described in
[Gen 1 RX datapath](#dp-gen1-rx).

The control symbols are listed in [Symbols, coding and scrambling](#primer-coding). The PHY
itself acts on three of them: COM for alignment, SKP for clock compensation and SUB for decode
errors. It passes the framing symbols (SHP, SDP, SLC, END, EDB, EPF) to the link layer, and uses
SHP, SDP and SLC only to realign the received packets.

## Gen 1 TX datapath {#dp-gen1-tx}

The figure below shows the transmit path. The link layer words enter in the `sys` clock domain and
leave as 20-bit or 40-bit transceiver words in the transceiver TX clock domain.

![Gen 1 TX datapath: the PIPE (byte swap to TS1/TS2 select) and the transceiver wrapper (SKP
inserter to width conversion), by clock domain.](datapath_tx.svg)

The stages are, in order:

1. **Byte swap**: link interface order to PHY order.
2. **Logical Idle**: when the link layer has no word to send, the PIPE sends Idle Symbols (`00h`
   data symbols). They are scrambled like data, so the partner's descrambler keeps advancing.
3. **Scrambler**: data symbols are XORed with the LFSR output; K symbols pass unchanged.
4. **TS1/TS2 select**: during training, the Training Sequences replace the scrambled stream. They
   are never scrambled.
5. **SKP inserter**: adds SKP Ordered Sets for clock compensation, see
   [Clock compensation](#dp-skp).
6. **CDC FIFO**: an 8-word asynchronous FIFO from `sys` to the transceiver TX clock.
7. **Width conversion**: 32-bit words to two (20-bit) or four (40-bit) symbols per transceiver
   clock cycle.

The PIPE accepts link layer words only in U0 (`tx_ready`). The SKP inserter pauses the link layer
for one cycle when it sends a SKP word (`sink.ready` low): the link layer must hold its word.

## Gen 1 scrambling {#dp-scrambling}

The Gen 1 scrambler is a 16-bit LFSR seeded with `FFFFh` (USB 3.2 section 6.3.1.3 and Appendix B).
It follows the rules of [Symbols, coding and scrambling](#primer-coding): only data symbols are
scrambled, SKP does not advance the LFSR, and COM resets it to the seed. The last rule is what
synchronizes the two sides: the Training Sequences start with COM, so both ends enter U0 with the
same LFSR state.

The descrambler is the same circuit on the receive side (`Descrambler`). It processes every
received symbol, including the Training Sequences before U0. Its LFSR is therefore already
synchronized with the partner's scrambler when the link enters U0; only its output to the link
layer waits for `rx_ready`.

Scrambling can be disabled for compliance testing: the Disable Scrambling bit of the TS2 received
during training, or the `disable_scrambling` input of the PIPE. The bit is decoded in
Polling.Idle and Recovery.Idle only.

## Gen 1 clock compensation {#dp-skp}

The two ends of a USB 3 link run from independent clocks, and the host often adds spread spectrum
clocking ([The wire](#primer-wire)). The receiver recovers the partner's clock, but its logic runs
from the local clock: without a margin, its FIFO would overflow or run dry. SKP Ordered Sets give
that margin: two SKP symbols that carry no data, sent on average once every 354 symbols, that a
receiver may drop (USB 3.2 section 6.4.3).

**Transmit.** The SKP inserter sends one 32-bit word of four SKP symbols, that is two Ordered Sets,
after every 176 words (704 symbols). The average matches the specification. The interval between
SKP words is twice the nominal one, a small deviation that keeps the SKP symbols on word
boundaries. Up to two SKP words are queued (`TXSKPInserter`).

SKP words are never inserted inside a packet: the link layer marks the first and last words of its
packets, and the inserter waits for the end of the packet. No SKP is scheduled while TSEQ is sent;
the symbol count restarts at the entry into Polling.Active.

**Receive.** The SKP remover drops every SKP symbol in the transceiver RX clock domain, before the
CDC FIFO (`RXSKPRemover`), so only data words cross to the `sys` clock domain. When the partner's
clock is faster than the local one, the dropped SKP symbols are the cycles the local side needs to
catch up, as the figure below shows.

![Gen 1 clock compensation: the SKP word received in the rx clock domain is not written to the CDC
FIFO. That skipped write is the margin a slower local clock needs.](datapath_skp.svg)

The RX path has no back-pressure: the transceiver delivers words at the line rate, and the `sys`
side reads them at least as fast, which is why `sys_clk_freq` is 125 MHz or more. A SKP removal
can leave a word with fewer than four symbols; the remover re-packs the remaining symbols into
full words.

## Gen 1 RX datapath {#dp-gen1-rx}

The figure below shows the receive path, from the transceiver to the link layer words.

![Gen 1 RX datapath: the transceiver wrapper (error substitution to word aligner) and the PIPE
(descrambler to byte swap), by clock domain.](datapath_rx.svg)

The stages are, in order:

1. **Comma alignment** in the transceiver: the RX words start on a symbol boundary. The
   transceiver may align a comma on any byte (`ALIGN_COMMA_WORD=1` on the Xilinx parts), since
   half of the COMs of a TS1 fall on odd bytes with two symbols per word.
2. **Error substitution**: a symbol received with a decode error, or with the wrong running
   disparity, becomes SUB (K28.4) (USB 3.2 section 6.3.1.4). The link layer then sees a corrupted
   packet and its CRC check fails (`RXErrorSubstitution`). The disparity check is optional and
   off on the Gowin and ECP5 fabrics (shorter paths at 125 MHz).
3. **Width conversion**: 2 or 4 symbols per cycle to 32-bit words.
4. **SKP removal and CDC FIFO**: see [Clock compensation](#dp-skp).
5. **Word aligner**: rotates the words so that a COM falls on symbol 0 (`RXWordAligner`). The
   alignment is updated only at the start of a run of COMs.
6. **Descrambler**: see [Gen 1 scrambling](#dp-scrambling).
7. **Framing aligner**: a second word aligner, on the framing symbols SHP, SDP and SLC. In U0 no COM
   is received to realign the words. The partner aligns its packets on its own words, which
   include the SKP symbols it inserted, so once the PHY drops one SKP Ordered Set the next packet
   starts two symbols off. The framing aligner puts each packet back on a word boundary; the
   symbols it drops or repeats are Logical Idle.
8. **rx_ready gate**: words reach the link layer only in U0.

Two signals leave the RX path for the LTSSM. The RX error monitor reports when 8b/10b decode
errors are being received (`RXErrorMonitor`): an LFPS burst never decodes as valid 8b/10b while
data does, which tells a Warm Reset LFPS from data. The RX idle filter reports electrical idle only
after ten consecutive idle cycles, so that the bursts of an LFPS do not toggle it (`RXIdleFilter`).
It can also ignore non-idle spikes shorter than a few cycles, described in
[Electrical idle and LFPS](#gt-lfps).

## Polarity {#dp-polarity}

The two wires of a differential pair may be swapped on the board. USB 3 asks the receiver to fix
this: a TS1 received with inverted data symbols (D21.5 instead of D10.2) tells the receiver to
invert its input (USB 3.2 section 6.4.2). The training unit detects the inverted TS1 and the LTSSM
sets the RX polarity of the transceiver (`rx_polarity`). The TX polarity is a fixed board
property, set at build time if needed.

At Gen 2 the block aligner applies the same rule to an inverted SYNC Ordered Set, see
[Gen 2 RX](#dp-gen2-rx).

## Gen 1 packet framing {#dp-gen1-framing}

At Gen 1 the link layer frames its own packets with K symbols; the PHY carries them as words. A
packet always starts on a word boundary, with a framing word of three start symbols and EPF. The
figure below shows a Header Packet: the framing word, the 12-byte header, its CRC-16 and the Link
Control Word, 20 symbols in five words.

![A Header Packet on the wire: five words at Gen 1, then the same bytes across two Data Blocks at
Gen 2, with Idle Symbols around the packet.](datapath_framing.svg)

The other packets follow the same pattern. A Link Command is a SLC framing word and one word (two
copies of the 16-bit command). A Data Packet Payload is a SDP framing word, the data, a CRC-32 and
an END framing word, or EDB when the packet is nullified. The link layer computes the CRCs; the
CRC generators are in `usb3_pipe/crc.py`, shared with the device core.

## Gen 2 blocks {#dp-gen2-blocks}

At Gen 2 the line code changes from 8b/10b (20 % overhead) to 128b/132b (3 % overhead). The unit is
a block: a 4-bit Block Header followed by 16 symbols of 8 bits, 132 bits in all (USB 3.2 section
6.3.2). The figure below shows the three block shapes the PHY handles.

![Gen 2 blocks: a data or control block, the SYNC Ordered Set and the longer SKP Ordered Set.](datapath_gen2_block.svg)

- The Block Header is `0011b` for a data block and `1100b` for a control block (bit 0
  transmitted first). The two values differ in all four bits, so a single bit error is corrected.
- Data blocks carry the packets and the Idle Symbols (`5Ah`).
- Control blocks carry the Ordered Sets: TSEQ, TS1, TS2, SYNC, SDS (Start of Data Stream) and SKP.
  Symbol 0 identifies them.
- The SYNC Ordered Set alternates `00h` and `FFh`. It is the alignment pattern of the receiver and
  it re-initializes the scramblers.
- The SKP Ordered Set is the only variable-length block: SKP symbols (`CCh`) in multiples of four,
  then SKPEND and three symbols of the transmitter's LFSR state. The PHY transmits 20 SKP symbols,
  that is 196 bits.

Bits are serialized LSB first: header bit 0 first, then symbol 0 bit 0, and so on. There are no K
symbols at Gen 2; framing symbols become ordinary bytes, described in
[Gen 2 packet framing](#dp-gen2-framing).

## Gen 2 scrambling {#dp-gen2-scrambling}

The Gen 2 scrambler uses a 23-bit LFSR, X^23 + X^21 + X^16 + X^8 + X^5 + X^2 + 1, seeded with
`1DBFBCh` (USB 3.2 section 6.3.2.3). Its rules are block based:

- Data blocks are scrambled and advance the LFSR.
- Control blocks advance the LFSR; their symbol 0, the identifier, is never scrambled.
- SYNC Ordered Sets re-initialize the LFSR to the seed.
- SKP Ordered Sets do not advance the LFSR; they carry its state for the receiver's checks.

The scrambler processes 16 bytes per cycle (`Gen2Scrambler`). TS1, TS2 and TSEQ also carry DC
balance symbols in symbols 14 and 15: the transmitter tracks the running difference between ones
and zeros and, above a threshold, replaces those symbols to bring it back (`Gen2TXDCBalance`,
USB 3.2 section 6.4.1.2.2). The running difference saturates at ±511, as the specification
asks.

## Gen 2 datapaths {#dp-gen2}

The figure below shows the Gen 2 transmit and receive paths. They take over from the Gen 1 PIPE
once Gen 2 is negotiated (`Gen2PIPE`, see [PHY architecture](#phy-architecture)). LFPS stays on the
Gen 1 path at both rates.

![Gen 2 datapaths: packet framing, blocks and scrambling, DC balance and gearbox on transmit;
block alignment, descrambling and packet parsing on receive.](datapath_gen2.svg)

### Transmit {#dp-gen2-tx}

1. **Packet framer**: link layer words to transmission units of bytes, with the Gen 2 framing
   (`Gen2TXFramer`, `Gen2LinkTX`). A unit is stored completely before it is sent: inside a unit,
   no Idle Symbol may appear.
2. **Packer**: units packed back to back into 16-byte Data Block payloads, Idle Symbols after a
   unit when the next one is not ready (`Gen2TXPacker`).
3. **Blocks and scrambler**: Data Blocks after an SDS, or the Ordered Sets requested by the LTSSM
   during training, scrambled (`Gen2TX`). A SYNC Ordered Set opens each Training Sequence burst
   and repeats every 32 TS1/TS2 or 16384 TSEQ.
4. **SKP insertion**: one SKP Ordered Set per 40 blocks on average, after a SYNC, a Training
   Sequence, an SDS, a Logical Idle block or the last block of a packet, never inside a packet.
5. **DC balance**: symbols 14 and 15 of TS1, TS2 and TSEQ.
6. **Gearbox**: 132-bit (or 196-bit SKP) blocks to continuous transceiver words of 40 or 80 bits
   (`Gen2BlockGearbox`). The transceiver's own 8b/10b is bypassed ("raw" words).

### Receive {#dp-gen2-rx}

1. **Block aligner** (`Gen2BlockAligner`): searches the 132-bit SYNC pattern at every bit offset of
   the incoming words. It moves through three phases: Unaligned, Aligned, then Locked after an
   SDS. An inverted SYNC toggles the RX polarity. In Aligned and Locked, a Block Header two bits
   or more away from both valid values sends it back to Unaligned. SKP Ordered Sets are consumed
   here and never leave the aligner.
2. **Descrambler and Ordered Sets** (`Gen2RX`): blocks descrambled, TSEQ, TS1 and TS2 detected on
   symbols 0 to 13 (eight in a row report to the LTSSM), SYNC and SDS reported, Data Blocks after
   an SDS passed on.
3. **Packet parser** (`Gen2RXFraming`, `Gen2LinkRX`): the bytes of the Data Blocks back to link layer
   words, with Idle Symbols dropped and each packet re-aligned on a word.

The Gen 2 RX path has no elastic buffer: it runs in the recovered clock domain, and a FIFO moves
the packet words (or, with 80-bit words, the Data Blocks) to `sys`. The receive path counts blocks,
corrected headers, alignment losses and each Ordered Set type for bring-up (`GEN2_RX_COUNTERS`,
read through the debug registers of [Debugging](#debugging)).

### Widths and rates {#dp-gen2-widths}

At 10 Gbps a 40-bit word needs a 250 MHz transceiver clock; a block arrives every 3.3 cycles. With
80-bit words (Gowin GTR12) the clock is 125 MHz and a block arrives every 1.65 cycles, so several
stages process 16 bytes per cycle.

The useful Gen 2 rate is about 1.2 GB/s. A 32-bit link layer at 125 MHz moves only 500 MB/s, so the
PHY offers wider link words (`link_words` in [PHY architecture](#phy-architecture)):

| `link_words` | Link layer words | Bytes per cycle | Gen 2 use |
|----------------|---------------------|---------------|------------------------------------|
| 1 | 32-bit | 4 | Works, 466 MB/s Bulk OUT |
| 2 | Pairs, 64-bit | 8 | 881 MB/s Bulk OUT |
| 4 | Quads, 128-bit | 16 | With `pipe_cd` and Gen 2 only |

The throughputs are measured on the XCKU3P board; [Gen 2 (10 Gbps)](#perf-gen2) has the full
results.

## Gen 2 packet framing {#dp-gen2-framing}

At Gen 2 there are no K symbols. The framing symbols are ordinary bytes, scrambled with the data,
and a packet may start at any byte of a Data Block (USB 3.2 section 7.2.1). Idle Symbols (`5Ah`)
fill the gaps between packets. The lower half of the framing figure above shows a Header Packet
starting at byte 3 of a Data Block and ending in the next one.

| Framing | Gen 1 | Gen 2 byte | Content |
|-----------|---------|------------|----------------------------------------------------|
| HPSTART | SHP x3, EPF | `9Ah` x3, `36h` | Header Packet, 20 bytes in all |
| DPHSTART | SHP x3, EPF | `95h` x3, `36h` | Data Packet Header, then 2 Length Field Replicas |
| DPPSTART | SDP x3, EPF | `96h` x3, `36h` | Data Packet Payload, then CRC-32 |
| DPPEND | END x3, EPF | `65h` x3, `36h` | End of a Data Packet Payload |
| DPPABORT | EDB x3, EPF | `69h` x3, `36h` | End of a nullified Data Packet Payload |
| LCSTART | SLC x3, EPF | `4Bh` x3, `36h` | Link Command, 8 bytes in all |

The link interface stays the Gen 1 one: 32-bit words with K flags, framing at word boundaries.
The packet framer and parser translate between the two formats, so the link layer of the device
core handles both generations with one framing logic. The parser also finds the length of a Data
Packet Payload from the Length Field Replicas of its header, since nothing else marks its end.

Gen 2 adds a fast path for Link Commands. With `source_lc_enable`, the LGOOD_n and LCRD_x Link
Commands are delivered on `source_lc` as soon as they are parsed, ahead of the packet words queued
in the RX FIFO. The link layer then returns credits and acknowledges without waiting.

## Slower fabrics: PIPE clock domain {#dp-pipe-cd}

On fabrics where the link layer cannot run at 125 MHz (Gowin GW5A, Lattice ECP5-5G), the PIPE
runs in its own clock domain at 125 MHz and hands 64-bit word pairs to a slower `sys`. The stages
of that crossing (`LinkWordsCDC`: Logical Idle filter, packets buffer, receive throttling) and the
Gen 2 words around it are described with the option in [Slower fabrics: `pipe_cd`](#phy-pipe-cd).

## Gen 1x2 lanes {#dp-lanes}

Gen 1x2 runs two Gen 1 lanes in parallel, each with its own PIPE, 8b/10b, SKP handling and
scrambler. The scramblers use a different seed per lane: `FFFFh` on lane 0 and `8000h` on lane 1
(USB 3.2 section 6.13.5). Striping, deskew and the status of this experimental mode are
described in [Gen 1x2 lanes](#phy-lanes).

## In the code {#dp-code}

| Function | File | Classes |
|------------------------|-------------------|--------------------------------------------|
| Gen 1 TX/RX datapaths | `datapath.py` | `TXDatapath`, `RXDatapath`, `TXSKPInserter`, `RXSKPRemover`, `RXWordAligner` |
| Gen 1 RX checks | `datapath.py` | `RXErrorSubstitution`, `RXErrorMonitor`, `RXIdleFilter` |
| 8b/10b tables, checker | `coding.py` | `RX8b10bChecker` |
| Gen 2 blocks | `coding.py` | `Gen2BlockGearbox`, `Gen2BlockAligner` |
| Gen 2 TX/RX | `datapath.py` | `Gen2TX`, `Gen2RX`, `Gen2TXDCBalance` |
| Scramblers | `scrambling.py` | `Scrambler`, `Descrambler`, `Gen2Scrambler` |
| Gen 2 framing | `framing.py` | `Gen2LinkTX`, `Gen2LinkRX` |
| CRCs | `crc.py` | `CRC5`, `CRC16`, `CRC32` |
| `pipe_cd` crossing | `phy.py`, `framing.py` | `LinkWordsCDC`, `LogicalIdleFilter`, `PacketsBuffer` |
| Lanes | `lanes.py` | `LaneStriper`, `LaneDestriper`, `LaneDeskew` |
| Symbols, Ordered Sets | `common.py` | `TS1`, `TS2`, `TSEQ`, Gen 2 constants |

The datapath unit tests are in `test/test_datapath.py`, `test_rx_datapath.py`, `test_skp.py`,
`test_tx_skp.py`, `test_rx_idle_filter.py`, `test_scrambling.py`, `test_coding.py` and
`test_framing.py`.

Where to go next: [Transceivers](#transceivers) for the wrappers around these paths,
[Link training](#link-training) for the LTSSM that drives them.
