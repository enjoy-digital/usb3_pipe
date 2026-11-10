# Glossary {#glossary}

The terms below are used throughout the manual. USB 3 terms follow the USB 3.2 specification;
[USB 3 in a nutshell](#usb3-primer) explains them in context.

| Term | Meaning |
|----------------------|-------------------------------------------------------------------------|
| 8b/10b | Gen 1 line code: each byte is sent as a 10-bit symbol, DC balanced, with K symbols for control |
| 128b/132b | Gen 2 line code: 16 bytes sent as a 132-bit block with a 4-bit block header |
| ACK | Transaction packet that requests IN data packets or acknowledges OUT ones |
| BOS | Binary device Object Store: descriptor listing the device capabilities |
| Burst | Data packets sent without waiting for their acknowledgement, up to the max burst |
| CDC-ACM | Communications Device Class, Abstract Control Model: a virtual serial port |
| CDC-ECM | Communications Device Class, Ethernet Control Model: a USB network adapter |
| CDR | Clock and Data Recovery: the receiver circuit that recovers the clock from the data |
| COM | K28.5 comma symbol: symbol alignment and start of the training ordered sets |
| CRC | Cyclic Redundancy Check: CRC-5 of link commands, CRC-16 of headers, CRC-32 of payloads |
| CSR | Control and Status Register of a LiteX design, on the CSR bus |
| DCU | Dual Channel Unit: the transceiver of the Lattice ECP5-5G |
| DP, DPH, DPP | Data packet, made of a data packet header and a data packet payload |
| DRP | Dynamic Reconfiguration Port of a Xilinx transceiver |
| Elastic buffer | Receive buffer that absorbs the clock difference of the two sides by dropping SKP |
| EOB | End Of Burst: flag of a Bulk IN data packet, the device ends the burst before NumP |
| EP0 | Endpoint 0, the control endpoint of every device |
| ERDY | Transaction packet by which an endpoint says it is ready after an NRDY |
| Gen 1, Gen 2 | Signaling rates of 5 Gbps and 10 Gbps per lane |
| Gen 1x1, Gen 2x1, Gen 1x2 | Rate and lane count: one lane at 5 or 10 Gbps, or two lanes at 5 Gbps |
| GTP, GTX, GTY | Transceivers of the Xilinx Artix-7, Kintex-7 and UltraScale+ families |
| GTR12 | Transceiver of the Gowin GW5AST and GW5AT families |
| Header packet | 12-byte header with CRC-16 and link control word; starts every protocol unit |
| Hot Reset | Reset of the upper layers, requested in the TS2 of Recovery; the link stays trained |
| ITP | Isochronous Timestamp Packet: the bus time broadcast by the host |
| K symbol | 8b/10b control symbol that never appears in data (COM, SKP, SHP, END) |
| LAU, LXU, LPMA | Link commands: power state request accepted, rejected, acknowledged |
| LBAD | Link command: a header packet was received with an error |
| LBPM | LFPS Based PWM Message: SuperSpeedPlus capability messages on LFPS |
| LCRD_x | Link command: header buffer x is free again (one credit) |
| LDN, LUP | Link commands: keep-alives of the downstream port (host or hub) and of the upstream port (device) |
| LFPS | Low Frequency Periodic Signaling: square wave bursts for out-of-band signaling |
| LFSR | Linear Feedback Shift Register, the scrambler generator |
| LGO_Ux | Link command: request to enter U1, U2 or U3 |
| LGOOD_n | Link command: header packet with sequence number n received good |
| Link command | 8-symbol link layer control unit, invisible to the protocol layer |
| Link control word | Last 2 bytes of a header packet: sequence number and CRC-5 |
| Link partner | The port at the other end of the cable: the host or a hub |
| LiteScope | LiteX logic analyzer, captures signals of a design on hardware |
| LiteX, Migen | Python frameworks in which the project is written |
| LMP | Link Management Packet: port capability, configuration and link functions |
| LRTY | Link command: retransmission of header packets starts |
| LTSSM | Link Training and Status State Machine |
| LUN | Logical Unit Number: one disk of a Mass Storage device, selected in each command |
| Max burst | Number of packets an endpoint can send or take in one burst (1 to 16), in the SuperSpeed endpoint companion descriptor |
| MSC | Mass Storage Class: a disk over USB |
| NRDY | Transaction packet: the endpoint has no data or no space now |
| NumP | Number of packets field of an ACK or ERDY transaction packet |
| Ordered set | Fixed sequence of symbols: TSEQ, TS1, TS2, SKP, and at Gen 2 SYNC and SDS |
| PIPE | PHY Interface for PCI Express, USB and SATA: the PHY to controller split |
| Recovery | LTSSM state that retrains the link quickly, also used to leave U1, U2 and U3 |
| Recovery storm | Many Recovery entries per second: a sign of a signal integrity or clocking problem |
| SCD1, SCD2 | SuperSpeedPlus capability declarations encoded in Polling.LFPS |
| SFP | Small Form-factor Pluggable cage, used with an SFP to USB adapter |
| SKP | Skip symbol or ordered set used for clock compensation |
| SS.Inactive | LTSSM error state entered on a Recovery timeout, left by a Warm Reset or a disconnect; called eSS.Inactive in USB 3.2 and in the code, where the PHY also retrains after 12 ms |
| SSC | Spread Spectrum Clocking: slow modulation of the clock frequency |
| SuperSpeed | Gen 1 operation (5 Gbps) |
| SuperSpeedPlus | Gen 2 or multi-lane operation |
| TP | Transaction Packet: ACK, NRDY, ERDY, STATUS, STALL, PING |
| TS1, TS2 | Training ordered sets: lock and configuration |
| TSEQ | Training ordered set used for receiver equalization |
| U0, U1, U2, U3 | Link power states: active, standby, deeper standby, suspend |
| UVC | USB Video Class: a camera over USB |
| Warm Reset | Link reset signaled by a long LFPS; restarts training |
| xHCI | Extensible Host Controller Interface: the USB 3 host controller of a PC |
| Zero-length packet | Data packet without payload: ends a transfer whose last packet has the maximum size |
