# USB 3 in a nutshell {#usb3-primer}

This chapter explains USB 3 to an FPGA engineer who has never read the specification. It follows
the data from the wire up to an endpoint: the signals on the cable, the training of the link, the
packets and their acknowledgement, and the transfers a host schedules. Each section starts with
the idea, then gives the mechanism, and ends with the module of the project that implements it.

The reference is the USB 3.2 specification. Gen 1 (5 Gbps) is described first, because every
port starts there; [Gen 2 at a glance](#primer-gen2) lists what changes at 10 Gbps.

## Three layers {#primer-layers}

A USB 3 port is a small network stack. Each layer solves one problem and talks only to the same
layer on the other side of the cable, through the layer below it. The figure below shows the three
layers and the units each pair exchanges.

![The three layers of a USB 3 port. Each layer exchanges its own units with its peer on the
other side: symbols and LFPS at the bottom, packets and link commands in the middle, transactions
at the top.](primer_layers.svg)

| Layer | Moves | Main jobs |
|----------------|---------------------|---------------------------------------------------------|
| Physical layer | Symbols | Signal before training (LFPS), train the receiver, code and scramble, compensate the clocks, run the LTSSM |
| Link layer | Packets, commands | Frame and check packets, number and acknowledge them, retry the bad ones, manage the power states |
| Protocol layer | Transfers | Address endpoints, request and send data, flow control between the host and each endpoint |

In the project, the PHY (`usb3_pipe`) is the physical layer and the device core (`usb3_core`) holds
the two upper layers. They meet at the link interface of `USB3PHY`: a stream of 32-bit words of
four symbols (or 64-bit words), plus status and power management signals. [PHY
architecture](#phy-architecture) describes this interface.

The name *PIPE* comes from the PHY Interface for PCI Express, USB and SATA: the interface between
an external USB 3 PHY chip and its controller. The project keeps the same split, but both sides
live in the FPGA.

## The wire {#primer-wire}

A USB 3 cable carries the USB 2 pair and two SuperSpeed differential pairs, one per direction.
Each side drives its own transmit pair, so both directions run at the same time at the full rate:
5 Gbps each way at Gen 1, 10 Gbps at Gen 2. The figure below shows the connections.

![The pairs of a USB 3 link. Each side drives its own transmit pair, through a series capacitor
(AC coupling). The USB 2 pair is in the cable but the project does not use it.](primer_wire.svg)

The pairs are AC coupled: a series capacitor blocks the DC level, so each side sets its own common
mode voltage. The data must therefore stay DC balanced on average, which is one job of the line
coding.

A port signals its presence with the termination of its receiver. The partner finds it by
measuring the load seen by its transmit pair: this is *receiver detection*, the first step of a
link.

Each side runs from its own reference clock, within 300 ppm of nominal. Many hosts also use
spread spectrum clocking (SSC): their clock wanders slowly, up to 5000 ppm below nominal, to
spread the radiated energy. The receiver recovers the partner's clock from the data transitions
(clock and data recovery, CDR) and must follow this wander. Clock compensation, described in
[Symbols, coding and scrambling](#primer-coding), absorbs the remaining difference.

In the project, the FPGA transceiver is the electrical layer: serializer, CDR, termination and
electrical idle. [Transceivers](#transceivers) gives its USB 3 settings. Since the USB 2 pair is
left unconnected, a USB3-PIPE device appears only on a SuperSpeed port.

## LFPS: talking before training {#primer-lfps}

Before the high-speed receivers are locked, both sides need a simpler way to talk. Low Frequency
Periodic Signaling (LFPS) is a square wave of 10 to 50 MHz, sent in bursts separated by electrical
idle. A receiver detects it with a simple squelch detector, without any clock recovery. The figure
below shows Polling.LFPS, the first exchange of a link.

![Polling.LFPS: bursts of about 1 µs (tBurst) repeated every 10 µs (tRepeat). Both sides send
them; receiving the partner's bursts starts the training.](primer_lfps.svg)

The meaning of an LFPS is carried by the length of its burst and by its repeat period. The table
below lists the patterns the project generates and detects, with the nominal values used in
`usb3_pipe/lfps.py`; the specification allows a range around each.

| Pattern | Burst | Purpose |
|------------------|---------------------|------------------------------------------------|
| Polling.LFPS | 1 µs, every 10 µs | Start of training after a partner is detected |
| Ping.LFPS | 190 ns, every 200 ms | Presence signal in U1 and Loopback |
| U1 exit | 900 ns | Wake the link from U1 |
| U2 exit | 100 µs | Wake the link from U2 |
| U3 wakeup | 1 ms | Wake the link from U3 (suspend) |
| Warm Reset | 100 ms | Reset the link, the strongest reset |

SuperSpeedPlus ports also use LFPS to agree on the speed. During Polling.LFPS they vary the repeat
period to encode the bits of two patterns, SCD1 and SCD2, which announce Gen 2 capability. They
then exchange short LFPS Based PWM Messages (LBPM) to settle the PHY capability.

In the project, `LFPSUnit` generates and detects the patterns. Four further modules of
`usb3_pipe/lfps.py` send and check SCD1, SCD2 and LBPM ([Link training](#link-training)).

## Symbols, coding and scrambling {#primer-coding}

A serial link needs three things from its data: as many ones as zeros (DC balance), frequent
transitions (for clock recovery) and a way to mark boundaries that data can never imitate. At
Gen 1, 8b/10b coding gives all three: every byte is sent as a 10-bit *symbol*. The 256 data
symbols are written Dx.y; a few extra codes, the *K symbols* (Kx.y), never appear in data and
serve as markers.

| Symbol | Code | Use |
|--------|-------|---------------------------------------------------------------------|
| COM | K28.5 | Comma: symbol alignment, start of TS1, TS2 and TSEQ |
| SKP | K28.1 | Skip: clock compensation, inserted and removed by the PHYs |
| SHP | K27.7 | Start of a header packet |
| SDP | K28.2 | Start of a data packet payload |
| SLC | K30.7 | Start of a link command |
| EPF | K23.7 | End of packet framing |
| END | K29.7 | End of a data packet payload |
| EDB | K28.3 | End bad: a nullified data packet payload |
| SUB | K28.4 | Substitutes a symbol received with a decode error |

Long runs of identical data would still concentrate the radiated energy at a few frequencies. The
transmitter therefore *scrambles* each data symbol: it XORs it with the output of a 16-bit LFSR
(polynomial x^16^ + x^5^ + x^4^ + x^3^ + 1). The receiver runs the same LFSR and removes it. K
symbols are not scrambled; COM resets both LFSRs and SKP does not advance them, so both sides
stay in step.

Clock compensation uses SKP. Each transmitter inserts SKP ordered sets (two SKP symbols) at a
fixed average rate of one per 354 symbols. Each receiver passes the data through an *elastic
buffer* that crosses from the recovered clock to its own clock. It drops SKP symbols when it
fills up, and passes them on when it runs low. The figure below shows where each step sits in
the transmit and receive chains.

![Gen 1 symbol chains. On transmit, SKP ordered sets are inserted, data symbols are scrambled
and every symbol is 8b/10b coded. On receive, the aligner finds the symbol boundaries on COM,
then the same steps run in reverse order.](primer_coding.svg)

Gen 2 replaces 8b/10b by 128b/132b coding: 16 bytes are sent as one *block* with a 4-bit header
that says whether the block holds data or an ordered set. The overhead drops from 20% to 3%.
Gen 2 scrambles with a 23-bit LFSR and has its own SKP, SYNC and SDS ordered sets. The packets
keep the Gen 1 framing symbols, now sent as scrambled bytes inside the data blocks.

In the project, the coding and scrambling live in three files ([Datapath](#datapath)):

| File | Content |
|-----------------------------------|------------------------------------------------------|
| `usb3_pipe/coding.py` | 8b/10b model, 128b/132b gearbox and block aligner |
| `usb3_pipe/scrambling.py` | Gen 1 and Gen 2 scramblers |
| `usb3_pipe/datapath.py` | SKP insertion and removal, receive alignment |

## Training {#primer-training}

A receiver cannot decode anything until it has adapted its equalizer to the channel, locked on
the bits and found the symbol boundaries. Training gives it each of these in turn. After
Polling.LFPS, both sides send fixed *ordered sets* in a scripted order:

1. **TSEQ** (Polling.RxEQ): a fixed sequence with a rich spectrum, sent 65536 times. The receiver
   trains its equalizer on it.
2. **TS1** (Polling.Active): the receiver locks on the COMs and finds the symbol boundaries. A
   port moves on once it has received eight TS1 or TS2 in a row.
3. **TS2** (Polling.Configuration): both sides confirm the link configuration, again with eight
   in a row.
4. **Idle symbols** (Polling.Idle): each side sends scrambled idle symbols until it has received
   eight in a row and sent sixteen. This proves the scramblers are in step. Both sides then enter
   U0.

TS1 and TS2 are 16 symbols long, numbered from 0, as the figure below shows. Four COMs start
them. Symbol 5 is the *link configuration*: its bits request a Hot Reset, Loopback or no
scrambling. The last ten symbols identify the set: D10.2 for TS1, D5.2 for TS2.

![The TS1 ordered set. Look at symbol 5, the link configuration, and its request bits. TS2
differs only by its ten identifier symbols.](primer_ts1.svg)

Training also corrects the lane polarity. If the D+ and D- wires of a pair are swapped, the
receiver sees TS1 with every bit inverted: D21.5 instead of D10.2. It then inverts its input.

In the project, `TSUnit` (`usb3_pipe/training.py`) generates and detects TSEQ, TS1 and TS2.
`USB3PIPE` (`usb3_pipe/core.py`) adds the scrambling and the idle symbol handshake
([Link training](#link-training)).

## The LTSSM {#primer-ltssm}

The Link Training and Status State Machine (LTSSM) is the heart of the physical layer. It decides
when to send LFPS, when to train and when the link can carry packets. The figure below shows its
main states; each is a group of substates in the specification.

![Simplified LTSSM. Follow the thick arrows: a port trains through Polling into U0. Recovery
retrains from U0 and is also the way back from U1, U2 and U3. Hot Reset is requested in the TS2 of
Recovery. A timeout of Recovery ends in SS.Inactive, which a Warm Reset
leaves.](primer_ltssm.svg)

| State | What happens |
|----------------------|---------------------------------------------------------------------|
| Rx.Detect | The port looks for a partner termination on its transmit pair |
| Polling | LFPS exchange, then TSEQ, TS1, TS2 and idle symbols: the full training; a timeout returns to Rx.Detect |
| U0 | The link is up: packets and link commands flow |
| Recovery | A short retraining with TS1 and TS2, entered on errors or on the exit of U1, U2 or U3 |
| U1, U2, U3 | Power states, left with an exit or wakeup LFPS handshake |
| Hot Reset | A reset of the upper layers, requested in the TS2 of Recovery; the link stays trained |
| SS.Inactive | An error state, entered on a Recovery timeout; the specification leaves it on a Warm Reset or a disconnect, the PHY also retrains after 12 ms |
| Compliance, Loopback | Test modes: a fixed test pattern, or the received data sent back |

A healthy link spends almost all its time in U0. Recovery is normal at the exit of a power state
and rare otherwise. A *Recovery storm*, many Recovery entries per second, usually means a signal
integrity or clocking problem.

In the project, `USB3LTSSM` (`usb3_pipe/ltssm.py`) implements these states except the Rx.Detect
family. The transceiver termination stays enabled, and an exit to Rx.Detect restarts the LTSSM at
Polling.Entry after a short delay. SS.Inactive also ends after 12 ms of quiet with such a
restart, so the device retrains even without a Warm Reset ([Link training](#link-training)).

## Power states {#primer-power}

The link has four power states. U0 is the active state; U1, U2 and U3 save more and more power
and take longer and longer to leave.

| State | Meaning | Exit | Typical exit time |
|-------|---------------------------------------|------------------|-------------------|
| U0 | Active, packets flow | Not applicable | Not applicable |
| U1 | Standby, fast exit | U1 exit LFPS | microseconds |
| U2 | Standby, slower exit, more saving | U2 exit LFPS | hundreds of µs |
| U3 | Suspend, the device is idle | U3 wakeup LFPS | milliseconds |

A port enters a power state by a handshake of link commands. One side sends LGO_U1, LGO_U2 or
LGO_U3. The partner accepts with LAU or rejects with LXU, and the requester confirms an
acceptance with LPMA. A device may reject U1 and U2 but must accept U3. The exit is an LFPS
handshake followed by Recovery.

In the project, the LTSSM runs the handshake timers and the device core decides whether to accept
a request. U1 and U2 are rejected unless the device core is built with `with_lpm`
([Device core architecture](#core-architecture)).

## Packets and link commands {#primer-packets}

In U0 the link carries three kinds of units, each framed by K symbols. A frame starts with three
copies of a start symbol and an EPF, so that a single corrupted symbol cannot hide a boundary. The
figure below shows the three frames.

![Gen 1 framing. A header packet is 20 symbols; a data packet payload follows its own header
packet; a link command is 8 symbols. The grey cells are K symbols: they mark the start and the
end of each frame.](primer_packets.svg)

| Unit | Content | Size | Seen by |
|-------------------|-------------------------------------------------------|----------------|-----------------|
| Header packet | 12-byte header, CRC-16, 2-byte link control word | 20 symbols | Both layers |
| Data packet payload | Up to 1024 data bytes and a CRC-32 | Up to 1036 symbols | Protocol layer |
| Link command | 16-bit command word with its CRC-5, sent twice | 8 symbols | Link layer only |

Every protocol unit starts with a header packet. Its link control word carries a header sequence
number and a CRC-5, used by the link layer. A data packet is a header packet followed by its
payload.

The header has a 5-bit type field. The protocol layer uses three types, and the link layer one:

| Type | Name | Purpose |
|------|------------------------------|------------------------------------------------------|
| TP | Transaction packet | Control of a transfer: ACK, NRDY, ERDY, STATUS, STALL, PING |
| DP | Data packet | A data packet header (DPH) and its payload (DPP) |
| ITP | Isochronous timestamp packet | The bus time, broadcast by the host |
| LMP | Link management packet | Link settings between the two ports |

In the project, `usb3_core/common.py` defines the packet types, the framing words and the link
command codes, and `usb3_pipe/crc.py` the three CRCs.

## Link layer flow control {#primer-link}

The link layer guarantees that every header packet arrives, in order, and only when the receiver
has room for it. It does so with two mechanisms: acknowledgements and credits.

Each header packet carries a 3-bit sequence number. The receiver answers each good one with
LGOOD_n, where n is its sequence number, and each bad one with LBAD. After an LBAD, the sender
sends LRTY and retransmits from the first unacknowledged packet.

A receiver has four header buffers, named A to D. A sender may send a header packet only while it
holds a *credit* for a free buffer. The receiver returns one credit with LCRD_x each time it frees
buffer x. The figure below shows both mechanisms.

![Header packet flow control. On each U0 entry both sides advertise their last received sequence
number and their free buffers. Then each header packet is acknowledged with LGOOD_n and its buffer
returned with LCRD_x; a bad one is answered with LBAD and sent again after
LRTY.](primer_link_flow.svg)

The table below lists the link commands, including those that keep the link alive.

| Command | Meaning |
|------------------------|-----------------------------------------------------------------|
| LGOOD_n, LBAD | Header packet n received good, or a header packet received bad |
| LCRD_x | Header buffer x freed: one credit back to the sender |
| LRTY | Retransmission starts |
| LGO_Ux, LAU, LXU, LPMA | Power state request, accept, reject, acknowledge |
| LUP | Keep-alive of the device: sent after 10 µs with nothing else to send in U0 |
| LDN | Keep-alive of the downstream port (host or hub) |

A missing acknowledgement or credit times out, and the link layer then forces Recovery to clear the
problem. On U0 entry, link management packets also exchange the port capabilities: link speed,
number of header buffers and direction.

SuperSpeedPlus operation keeps the same scheme with a 4-bit sequence number. It has two kinds of
buffers and credits: Type 1 for packets that must stay in order, Type 2 for the asynchronous data
packets of Control and Bulk transfers.

In the project, `USB3LinkLayer` (`usb3_core/link.py`) implements the framing, the CRC checks, the
sequence numbers, the credits, the retries, the timers and the link management packets
([Device core architecture](#core-architecture)).

## Protocol layer and transfers {#primer-protocol}

The host owns the bus: it starts every transfer. A transfer is addressed to an *endpoint* of the
device, a numbered buffer with a direction and a type. Endpoint 0 is the control endpoint every
device has; the others are declared in the descriptors.

| Transfer type | Use | Guarantees |
|---------------|--------------------------------------------|--------------------------------------|
| Control | Enumeration, configuration, class requests | Delivery, small payloads |
| Bulk | Large data: storage, streams, test data | Delivery, any bandwidth left |
| Interrupt | Small periodic data: notifications | Delivery, bounded latency |
| Isochronous | Video, audio | Reserved bandwidth, no retry |

For a Bulk IN transfer, the host sends an ACK transaction packet that asks for a number of
packets (NumP). The device answers with up to that many data packets, a *burst*, without waiting
for each acknowledgement. When it has no data, it answers NRDY. Unlike USB 2, the host then stops
asking until the device sends ERDY to say it is ready. The figure below shows a Bulk IN exchange.

![Bulk IN. Look at the sequence numbers: each ACK TP acknowledges the packets received so far and
asks for NumP more. NRDY parks the endpoint until ERDY. A device that runs out of data before NumP
flags its last packet end of burst (EOB).](primer_bulk_in.svg)

Bulk OUT is the mirror image: the host sends data packets and the device acknowledges each one
with an ACK TP. The NumP of that acknowledgement tells how many more packets the device can take.
A NumP of zero stops the host until the device sends ERDY, as the figure below shows.

![Bulk OUT. The device acknowledges each data packet with the next expected sequence number and
its free space as NumP. With no space left it answers NumP 0, then ERDY when space is
back.](primer_bulk_out.svg)

The size of a burst is limited by the endpoint's *max burst*, up to 16 packets of 1024 bytes,
declared in the SuperSpeed endpoint companion descriptor. A longer burst hides the round-trip
latency of the acknowledgements: it is the key to throughput.

Isochronous IN transfers have no NRDY, ERDY or retry. The host requests the packets of each
service interval, and the device sends what it has. The host also broadcasts ITPs with the bus
time, so that isochronous devices can align their data.

In the project, `USB3ProtocolLayer` (`usb3_core/protocol.py`) dispatches the headers to the
endpoints and arbitrates their responses. The Bulk endpoints are in `usb3_core/bulk.py` and the
Isochronous IN endpoint in `usb3_core/iso.py` ([Device core architecture](#core-architecture)).

## Enumeration {#primer-enum}

When a device comes up, the host discovers it in a fixed order, shown in the figure below. The
link layers first exchange their port capabilities and configuration. Then the host gives the
device an address, reads its descriptors on endpoint 0 and selects a configuration.

![Enumeration. Link management packets settle the link first. Then each cyan arrow is a control
transfer on endpoint 0: assign the address, read the descriptors, select the
configuration.](primer_enum.svg)

An xHCI host assigns the address before it reads the device descriptor; the exact list of
requests varies from one host to another. Every request is a control transfer of up to three
stages:

1. The *setup* stage is a data packet that holds the 8-byte request.
2. The optional *data* stage moves the response, or the data of the request.
3. The *status* stage starts with a STATUS transaction packet from the host and ends with the
   device's acknowledgement.

The descriptors describe the device:

| Descriptor | Content |
|----------------------------|--------------------------------------------------------------|
| Device | Vendor and product IDs, class, number of configurations |
| Configuration | Interfaces and their endpoints |
| SuperSpeed endpoint companion | Max burst of each endpoint |
| BOS | Device capabilities; the SuperSpeedPlus capability marks a Gen 2 device |
| String | Human-readable names |

In the project, `USB3EndpointControl` (`usb3_core/control.py`) answers the standard requests from
a table built by `USB3Descriptors` (`usb3_core/descriptors.py`). A class adds its own requests to
the same table ([Device classes](#classes)).

## Gen 2 at a glance {#primer-gen2}

Gen 2x1 doubles the rate to 10 Gbps on the same pair. The table below sums up what changes, layer
by layer.

| Layer | Gen 1 (SuperSpeed) | Gen 2 (SuperSpeedPlus) |
|------------------|------------------------------------|----------------------------------------------|
| Coding | 8b/10b, 20% overhead | 128b/132b blocks, 3% overhead |
| Scrambling | 16-bit LFSR | 23-bit LFSR |
| Speed choice | None | SCD1 and SCD2 in Polling.LFPS, then LBPM |
| Framing | Packets aligned on 4-symbol words | Packets at any byte of a data block |
| Sequence numbers | 3 bits, 4 header buffers | 4 bits, Type 1 and Type 2 buffers |
| Byte rate after coding | 500 MB/s per direction | about 1200 MB/s per direction |

A Gen 2 capable device starts like any SuperSpeedPlus port and settles the rate with its
partner in Polling, through SCD1, SCD2 and LBPM. With a Gen 1 host it falls back to Gen 1, so
the same bitstream works on both kinds of ports.

In the project, `USB3PHY(gen2_capable=True)` adds `SSPPolling` to the LTSSM and a `Gen2PIPE`
datapath. The link layer switches to SuperSpeedPlus operation when the PHY reports `gen2_active`
([Link training](#link-training), [Datapath](#datapath)).

## Where to go next {#primer-next}

[PHY architecture](#phy-architecture) maps these concepts to the PHY modules;
[Device core architecture](#core-architecture) does the same for the link and protocol layers.
