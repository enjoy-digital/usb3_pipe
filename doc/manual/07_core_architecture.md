```{=latex}
\part{The device core}
```

```{=html}
<h1 class="part">Part III. The device core</h1>
```

# Device core architecture {#core-architecture}

This chapter describes the device core, `usb3_core`: the layers and endpoints that turn the PHY
into a working USB 3 device. It starts with what an integrator uses first, the endpoints and their
streams, then goes down to the control endpoint, the protocol layer and the link layer. The
protocol concepts are explained in [USB 3 in a nutshell](#usb3-primer).

## At a glance {#core-glance}

| Item | Value |
|--------------------------|-----------------------------------------------------------------|
| Speeds | Gen 1x1 (5 Gbps, SuperSpeed), Gen 2x1 (10 Gbps, SuperSpeedPlus) |
| Role | Device (upstream port), one configuration |
| Clock | The `sys` clock domain: 125 MHz, or 62.5 MHz or 83.33 MHz with 64-bit words on slower fabrics |
| Link layer words | 32-bit (default), 64-bit or 128-bit (PHY `link_words` of 1, 2 or 4) |
| Endpoints | Control (EP0), Bulk IN, Bulk OUT, Isochronous IN |
| Max burst | 2, 4, 8 or 16 packets (Bulk); 1, 2, 4, 8 or 16 packets (Isochronous IN) |
| Max packet size | 1024 bytes (Bulk, Isochronous), 512 bytes (EP0) |
| Power management | U3 (suspend) always; U1 and U2 with `with_lpm` |
| Device classes | Vendor Bulk, CDC-ACM, UVC, Mass Storage, composite devices |
| Top-level class | `USB3DeviceCore` in `usb3_core/device.py` |

## Architecture {#core-overview}

The device core is a stack of three parts, as the figure below shows. The link layer makes the
link reliable: it frames packets, checks their CRCs and resends lost headers. The protocol layer
routes each header to its endpoint and arbitrates what is sent back. The endpoints buffer the data
of user logic.

![Device core architecture. Look at the stack from left to right: the link and protocol layers
(cyan) sit between the PHY and the endpoints; EP0 answers from a descriptors ROM, the other
endpoints connect to user logic or to a device class.](core_architecture.svg)

The core talks to the PHY only through the PHY's link interface. Any PHY that provides the same
interface can replace `usb3_pipe`; the simulation stand-in of the tests is one. The whole core
runs in the `sys` clock domain of the PHY, so user logic connected to the endpoints needs no clock
domain crossing.

The device core uses little-endian words: the first byte of a packet is on `data[0:8]`. The PHY
puts the first symbol on the upper byte instead, so the core swaps the bytes at its boundary.

## Usage {#core-usage}

A device needs four things: its descriptors, its endpoints, the core, and user logic on the
endpoint streams. The example below builds a vendor Bulk device that sends back, on Bulk IN, the
packets the host writes on Bulk OUT.

```python
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

# 1. Descriptors: one vendor interface, EP1 Bulk IN, EP2 Bulk OUT.
descriptors = bulk_descriptors(in_ept=1, out_ept=2, max_burst=8)

# 2-3. Endpoints (same numbers and max burst) and the core.
self.usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq,
    descriptors = descriptors,
    endpoints   = [
        USB3BulkIN(ept=1,  max_burst=8),
        USB3BulkOUT(ept=2, max_burst=8),
    ])
self.usb3_core.add_csr(with_trace=True)

# 4. User logic: Bulk OUT packets sent back on Bulk IN.
ep_in, ep_out = self.usb3_core.endpoints
self.comb += ep_out.source.connect(ep_in.sink)
```

The descriptors and the endpoint list must agree: same endpoint numbers, directions and max
bursts. The core does not cross-check them, and the host only sees the descriptors. The
`loopback` example of [Examples](#examples) is this design with a FIFO between the two endpoints.

| Parameter | Default | Meaning |
|--------------------------|---------|-------------------------------------------------------|
| `descriptors` | none | A `USB3Descriptors` instance |
| `endpoints` | none | Endpoint list (not EP0), in transmit priority order |
| `with_ep0_handler` | `False` | Pass unknown class and vendor requests to user logic |
| `with_lpm` | `False` | Accept U1 and U2 |
| `crc32_cascade` | `False` | Smaller CRC-32 logic with longer paths, for small FPGAs |
| `timer_scale` | 1 | Divide the link timers (simulation) |

Two outputs give the state of the device: `ready` (U0, link initialized) and `configured` (the host
has selected the configuration). On a link reset, the core resets EP0, the protocol layer and the
sequence numbers of the endpoints.

With a 64-bit link layer (`USB3PHY(link_words=2)`), give the Bulk endpoints `data_width=64`; their
streams then carry 64-bit words with an 8-bit `be`. The same holds for 128-bit words with
`link_words=4`. See [Wide link words](#core-link-wide) for when to use them.

A Bulk OUT endpoint must be as wide as the link layer; the core refuses a 32-bit one. User logic
that works on 32-bit words connects to wide Bulk endpoints through the
[Endpoint converters](#core-converters).

## Endpoints {#core-endpoints}

### The user stream {#core-stream}

User logic sees an endpoint as a LiteX stream: `sink` for an IN endpoint (device to host),
`source` for an OUT endpoint (host to device). One stream packet, ended by `last`, is one USB
packet or a run of them. The table below lists the fields; the layout is `endpoint_layout()`.

| Field | Width (32, 64, 128-bit) | Meaning |
|-------------|------------------------|-----------------------------------------------------|
| `data` | 32, 64, 128 | Packet bytes, first byte on `data[0:8]` |
| `be` | 4, 8, 16 | Valid bytes, contiguous from byte 0; all ones except on the last word |
| `last` | 1 | Last word of the packet |
| `valid` | 1 | Word present |
| `ready` | 1 | Word accepted |

The figure below shows an 18-byte packet written into a 32-bit Bulk IN endpoint. The fifth word
holds the last two bytes, so its `be` is `0x3`; the endpoint holds `ready` low for one cycle.

![An 18-byte packet on the user stream of a 32-bit IN endpoint. Look at the last word: `last` is
set and `be` marks two valid bytes; D2 stays on the bus while `ready` is
low.](core_stream_timing.svg)

The two directions do not treat packet boundaries the same way:

- **Bulk IN**: the endpoint cuts a stream packet longer than 1024 bytes into maximum-size packets.
  A last packet shorter than 1024 bytes is a short packet and ends the host transfer. The endpoint
  never adds a zero-length packet, so a stream packet of exactly N x 1024 bytes does not end the
  transfer.
- **Bulk OUT**: each USB packet the host sends, 1024 bytes at most, comes out as one stream packet
  ended by `last`. The host transfer boundaries are not visible; a protocol on top of Bulk OUT
  carries its own lengths. Zero-length packets are dropped.

### Packet slots {#core-slots}

Bulk and Isochronous endpoints buffer packets in a ring of 1024-byte packet slots, one per packet
of a burst (`USB3PacketSlots`). The number of slots is the max burst of the endpoint, so a Bulk
endpoint with a max burst of 8 holds 8 KiB of block RAM.

The figure below shows the Bulk IN datapath. The packet writer fills the slots from the user
stream. The payload reader sends them when the host asks. A slot is freed only when the host has
acknowledged its packet, so that it can be resent after an error.

![Bulk IN endpoint. Follow the data from user logic through the packet writer and the slot ring to
the protocol layer (cyan arrow); the host ACK TPs free the slots for the
writer.](core_bulk_in.svg)

### Bursts and flow control {#core-bursts}

Bursts and flow control follow the USB 3 rules described in
[Protocol layer and transfers](#primer-protocol): the host requests NumP packets with an ACK TP,
the endpoint sends up to NumP packets without waiting, and NRDY and ERDY park and wake the
endpoint. The figure below shows how a Bulk IN endpoint applies them to its slots.

![Bulk IN burst and flow control. Look at the notes on the right: the second ACK TP frees the four
slots, NRDY answers a request with empty slots, ERDY follows the next user packet, and a short
packet ends the transfer.](core_bulk_in_burst.svg)

A Bulk IN endpoint keeps a sent packet in its slot until an ACK TP acknowledges it. An ACK TP
with the retry bit makes it resend from the requested packet. While user logic is writing, the
endpoint holds back the last full packet for up to 64 idle cycles, so that a long write continues
the burst instead of closing it.

A Bulk OUT endpoint acknowledges each received packet with an ACK TP whose NumP is its number of
free slots. After a NumP of 0, it sends ERDY once a slot is free again. A packet with a CRC error
is dropped, and the ACK TP asks for it again with the retry bit.

### Isochronous IN {#core-iso}

An Isochronous IN endpoint (`USB3IsoIN`) sends one transfer per service interval, with no retry
and no flow control; its companion descriptor has a Mult of 0. User logic writes each transfer as
one stream packet of up to max burst x 1024 bytes. The endpoint sends it as 1024-byte packets, the
last one with the Last Packet Flag.

The host requests the packets of each interval with ACK TPs, the first one with sequence number 0.
The endpoint answers with the oldest complete transfer, or with a zero-length packet when none is
buffered. A transfer cut by the next interval is dropped. The outputs `n_transfers`, `n_zlps` and
`n_drops` count these three cases.

The endpoint is declared in an interface alternate setting, because the host reserves bandwidth
for it. Alternate setting 0, without the endpoint, uses none. The host selects alternate setting 1
with SET_INTERFACE before the first interval, so user logic starts its stream on
`ep0.set_interface`. `iso_in_descriptors()` builds these descriptors.

### Flush {#core-flush}

IN endpoints have a `flush` input that drops the buffered packets, for example when a video stream
restarts. Packets already sent stay until the host acknowledges them, so the sequence numbers stay
in step with the host.

### Endpoint converters {#core-converters}

A function written for 32-bit words, such as the Mass Storage SCSI engine or the CDC-ACM byte
packer, can run on a 64-bit or 128-bit link layer. Its Bulk endpoints are built at the link layer
width, and two converters adapt their user streams to 32-bit words, as the figure below shows.

![Endpoint converters on a wide link layer. Follow the cyan arrows: the up converter packs the
32-bit words of the function into the Bulk IN words, the down converter splits the Bulk OUT words
for the function.](core_converters.svg)

| Class or helper | Role |
|--------------------------------------|---------------------------------------------------|
| `USB3EndpointUpConverter` | 32-bit words to 64 or 128-bit Bulk IN words |
| `USB3EndpointDownConverter` | 64 or 128-bit Bulk OUT words to 32-bit words |
| `bulk_endpoint_32()` | Adds the right converter to a Bulk endpoint, or returns a 32-bit endpoint as is |

The up converter ends a wide word early when a packet ends inside it; the upper bytes are then
not valid. The down converter skips the 32-bit words that hold no valid byte after the end of a
packet. The CDC-ACM and Mass Storage device classes use them; see [Device classes](#classes).

```python
from usb3_core.bulk import USB3BulkIN, USB3BulkOUT, bulk_endpoint_32

dw = 32*usb3_phy.link_words
ep_in  = USB3BulkIN(ept=1,  max_burst=8, data_width=dw)
ep_out = USB3BulkOUT(ept=2, max_burst=8, data_width=dw)
# The core is built with [ep_in, ep_out], as in Usage.
# in32.sink: 32-bit words to the host.
in32  = bulk_endpoint_32(self, ep_in)
# out32.source: 32-bit words from the host.
out32 = bulk_endpoint_32(self, ep_out)
```

## Control endpoint {#core-control}

The control endpoint, EP0, answers the requests the host sends during enumeration and afterwards
(see [Enumeration](#primer-enum)). The figure below shows how a request is served. Its class is
`USB3EndpointControl`.

![Control endpoint. Look at the three paths out of the SETUP decoder: a constant answer from the
requests table, a standard request applied to the device state, or a class or vendor request
passed to the user handler outside the core.](core_control.svg)

### Requests {#core-control-requests}

The descriptors builder produces a requests table: one entry per request the device answers, with
its constant response. The responses are stored in a ROM, so enumeration needs no processor. The
table below lists what EP0 handles.

| Request | Handling |
|------------------------------------|---------------------------------------------------------|
| GET_DESCRIPTOR | Device, configuration, BOS and string descriptors from the ROM |
| GET_STATUS | Device (with the U1 and U2 Enable bits), interface, endpoint |
| GET_CONFIGURATION | Returns 1 |
| SET_ADDRESS | Applied in the Status stage (`dev_addr`) |
| SET_CONFIGURATION | Applied (`configured`); resets sequence numbers and alternate settings |
| SET_FEATURE, CLEAR_FEATURE | U1_ENABLE and U2_ENABLE tracked; other features accepted |
| CLEAR_FEATURE(ENDPOINT_HALT) | Resets the sequence number of the endpoint |
| SET_INTERFACE, GET_INTERFACE | Alternate settings (`set_interface`, `interface`, `alt_setting`); SET_INTERFACE resets the sequence numbers of the endpoints of the interface |
| SET_SEL, SET_ISOCH_DELAY | Accepted, data ignored |
| Class requests of the table | Constant responses (`add_class_request`) |
| Other class or vendor requests | User handler with `with_ep0_handler`, else STALL |
| Other standard requests | STALL |

An endpoint is never halted: SET_FEATURE(ENDPOINT_HALT) is accepted without effect and GET_STATUS
always reports the endpoint as running.

### Descriptors {#core-descriptors}

`USB3Descriptors` builds the device descriptor, one configuration with its interfaces and
endpoints, the BOS descriptor and the strings. Each endpoint gets its SuperSpeed Endpoint
Companion with its max burst. Interfaces can carry class-specific descriptors and alternate
settings; raw byte lists insert Interface Association Descriptors (IADs).

The BOS always carries the USB 2.0 Extension and the SuperSpeed USB Device Capability. With a
Gen 2 capable PHY, the core adds the SuperSpeedPlus USB Device Capability. With `with_lpm`, it
advertises non-zero U1 and U2 exit latencies.

| Argument | Default | Meaning |
|----------------|----------------------|------------------------------------------|
| `vid`, `pid` | `0x1d50`, `0x605a` | Vendor and product IDs |
| `manufacturer` | `"Enjoy-Digital"` | Manufacturer string |
| `product` | `"USB3-PIPE"` | Product string |
| `serial` | `"0001"` | Serial number string |
| `interfaces` | none | `USB3InterfaceDescriptor` list, or raw bytes |
| `device_class` | `(0, 0, 0)` | Class, subclass, protocol; `DEVICE_CLASS_IAD` with IADs |
| `max_power_ma` | 0 | Bus power; the device is declared self-powered |

Helpers build the descriptors of the usual devices. The device classes of
[Device classes](#classes) use the last three.

| Helper | Device |
|------------------------------|------------------------------------------------------------|
| `bulk_descriptors()` | Vendor interface with one Bulk IN and one Bulk OUT endpoint |
| `iso_in_descriptors()` | Vendor interface with an Isochronous IN endpoint in alternate setting 1 |
| `cdc_acm_descriptors()` | CDC-ACM serial port (`usb3_core/cdc_acm.py`) |
| `uvc_descriptors()` | UVC camera (`usb3_core/uvc.py`) |
| `msc_descriptors()` | Mass Storage (`usb3_core/msc.py`) |

### Request handler {#core-handler}

With `with_ep0_handler=True`, EP0 passes the class and vendor requests that are not in the table
to user logic on `core.ep0`. The device classes use it for their controls, for example the UVC
PROBE and COMMIT requests. The table below lists the handler signals.

| Signal | Direction | Meaning |
|---------------------|-----------|-------------------------------------------------------------|
| `req_setup` | out | Pulse: a new request |
| `req_valid` | out | Level: the request fields below are valid |
| `req_type` | out | `bmRequestType` |
| `req_request` | out | `bRequest` |
| `req_value` | out | `wValue` |
| `req_index` | out | `wIndex` |
| `req_length` | out | `wLength` |
| `req_stall` | in | Stall the request; set within a few cycles of `req_setup` |
| `req_in` | in | IN Data stage response, 32-bit stream, up to 512 bytes |
| `req_out` | out | OUT Data stage data, 32-bit stream, no back-pressure |
| `req_done` | out | Pulse: Status stage completed, for every request |

The handler below answers one vendor IN request with a constant word and stalls the other class
and vendor requests.

```python
core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints,
    with_ep0_handler=True)
ep0  = core.ep0
is_mine = Signal()
self.comb += [
    # Vendor, device-to-host, bRequest 0x01.
    is_mine.eq((ep0.req_type == 0xc0) & (ep0.req_request == 0x01)),
    ep0.req_stall.eq(ep0.req_valid & ~is_mine),
]
self.sync += [
    If(ep0.req_setup & is_mine,
        ep0.req_in.valid.eq(1),
        ep0.req_in.data.eq(0x12345678),
        ep0.req_in.be.eq(0b1111),
        ep0.req_in.last.eq(1),
    ).Elif(ep0.req_in.ready,
        ep0.req_in.valid.eq(0),
    )
]
```

### Wishbone bridge {#core-wishbone}

`USB3EP0Wishbone` is a ready-made request handler that gives the host a Wishbone master on the SoC
bus. It speaks the protocol of `litex_server --usb`, so the LiteX host tools reach the registers
of the design over USB 3. Add its `bus` with `soc.bus.add_master`.

| Field | Read | Write |
|-------------------|----------------------------------|----------------------------------|
| `bmRequestType` | `0xc3` | `0x43` |
| `bRequest` | `0x00` | `0x00` |
| `wValue`, `wIndex` | Byte address, low and high halves | Byte address, low and high halves |
| `wLength` | 4 x N bytes, N up to 128 | 4 x N bytes, N up to 128 |

Writes are buffered, two full-size requests deep, and executed before any later read. Other
requests are stalled.

## Protocol layer {#core-protocol}

The protocol layer sits between the link layer and the endpoints. Its class is
`USB3ProtocolLayer`; it receives the endpoints as a list, EP0 first.

On receive, it hands each header packet, in order, to every endpoint at once; the endpoint the
header addresses reacts. A data packet header is processed together with the status of its
payload: received without error, and stored by its endpoint. Payloads go to their endpoint by
endpoint number, and SETUP payloads only go to EP0.

On transmit, responses go first: the ACK and STALL TPs of the endpoints and the PING_RESPONSE to a
host PING. Then the endpoints are served by fixed priority, in the order of the list. The device
address goes into every header, and the payloads follow the order of their headers.

When the link layer is wider than 32 bits, some 32-bit endpoints still work: the protocol layer
adapts their payloads at their own rate (`DPPPacker`, `DPPRXAdapter`). EP0, the Isochronous IN
endpoint and 32-bit Bulk IN endpoints can be adapted this way.

A 32-bit Bulk OUT endpoint cannot, and the core refuses it. The endpoint would acknowledge a
packet before the adapter had stored its payload. Bulk OUT endpoints are therefore built at the
link layer width, with an [Endpoint converter](#core-converters) for 32-bit user logic.

## Link layer {#core-link}

The link layer turns the PHY symbol stream into reliable header packets and data payloads. The
figure below shows its receive path on top, its transmit path at the bottom and the flow control
between them. The class is `USB3LinkLayer`.

![Link layer. Look at the flow control block in the middle: it acknowledges received headers with
LGOOD and LCRD, and makes the Tx Header Buffers resend a header after a
retry.](core_link_layer.svg)

### Framing and CRCs {#core-link-framing}

On receive, the parser recognizes header packets (SHP), link commands (SLC) and data packet
payloads (SDP) by their framing symbols. It checks the CRC-16 of each header, the CRC-5 of its
link control word and the CRC-32 of each payload. A payload with a bad CRC is reported to its
endpoint, which asks the host to send it again.

On transmit, the framer adds the framing symbols and the CRCs. A payload follows its data packet
header (DPH) and ends with the END symbol. A payload that cannot be completed is nullified with
the EDB symbol.

### Header flow control and retries {#core-link-flow}

The link layer implements the sequence numbers, credits and retries of
[Link layer flow control](#primer-link). It keeps four Rx Header Buffers and four Tx Header
Buffers. A sent header stays in a Tx Header Buffer until its LGOOD arrives, so that it can be
resent after an LBAD.

At each U0 entry, both sides advertise their sequence number and credits. The flow control state
is kept across Recovery, so headers in flight are not lost. A link reset (Polling or Hot Reset)
clears the sequence numbers and the buffers.

### SuperSpeedPlus operation {#core-link-ssp}

When the PHY trains at Gen 2 (`phy.gen2_active`), the link layer switches to SuperSpeedPlus
operation. The header sequence numbers grow to 4 bits. The buffers split into Type 1 and Type 2,
with separate credits; Type 2 holds the data packets of Control and Bulk endpoints. ACK TPs and
DPHs carry the transfer type of their endpoint, taken from the endpoint list.

### Timers and link management packets {#core-link-timers}

| Timer or packet | Value or role |
|-------------------------|---------------------------------------------------------------|
| LUP keep-alive | Sent in U0 when nothing was sent for 10 µs |
| `PENDING_HP_TIMER` | 10 µs without LGOOD for a sent header: Recovery |
| `CREDIT_HP_TIMER` | 5 ms without the missing LCRD credits: Recovery |
| Port Capability LMP | Sent after link initialization: 4 header buffers, upstream port |
| Port Configuration LMP | Answered with a Port Configuration Response |
| U2 Inactivity Timeout LMP | Programs the U1 to U2 timeout of the LTSSM |
| Set Link Function LMP | `Force_LinkPM_Accept` handled |

At Gen 2, the framer sends a DPH only once its payload is ready, so the pending header timer is
extended by the time to buffer one 1024-byte payload.

### Power management {#core-link-pm}

The host asks for a low-power link state with the LGO_U1, LGO_U2 or LGO_U3 link commands (see
[Power states](#primer-power)). The link layer always accepts U3, the suspend state, with LAU.
Without `with_lpm`, it rejects U1 and U2 with LXU and the host stops asking.

With `with_lpm=True`, the descriptors advertise U1 and U2 exit latencies of 10 µs and 2047 µs,
because hosts only enable U1 and U2 when they are non-zero. The link layer then accepts LGO_U1 and
LGO_U2 only when the link is idle: everything acknowledged and credited in both directions,
nothing to send, and no endpoint waiting for an ACK TP.

U1 and U2 are left through Recovery, by the host or by the device when it has packets to send.
The device never asks for U1 or U2 itself.

### Wide link words {#core-link-wide}

At 125 MHz, 32-bit words carry 500 MB/s per direction: enough for Gen 1, not for Gen 2. With a PHY
built with `link_words=2`, the link layer processes two PHY words per cycle and moves 64-bit
payload words. With `link_words=4`, it moves four; this needs a Gen 2 capable PHY with its own
clock domain (`pipe_cd`).

Wide words have a second use on slower fabrics. A 64-bit link layer at 62.5 MHz carries the same
500 MB/s as a 32-bit one at 125 MHz, so it relaxes timing without losing Gen 1 bandwidth. The
Gowin targets run it at 62.5 MHz, the ECPIX-5 at 83.33 MHz.

## Registers and traces {#core-registers}

`core.add_csr()` adds the status registers of the core. They report link ready, configured, the
device address, the LBAD and retry counts, the Recovery requests with their causes, the receive
error counts and the link layer state.

`add_csr(with_trace=True)` also adds two traces in block RAM. `USB3CoreTrace` records the header
packets exchanged with the host, with a timestamp. `USB3LinkTrace` records the link commands, the
U0 entries and exits, and the Recovery requests.

The host scripts `bench/usb3_debug.py` and `bench/usb3_trace.py` read them; see
[Debugging](#debugging). The register reference is in [Registers](#registers).

## In the code {#core-code}

The table below lists every module of `usb3_core/`.

| File | Main classes | Role |
|--------------------------|---------------------------------------|--------------------------------------|
| `device.py` | `USB3DeviceCore` | Top level: link, protocol, EP0, endpoints, registers |
| `link.py` | `USB3LinkLayer` | Framing, CRCs, header flow control, retries, LMPs, power management |
| `protocol.py` | `USB3ProtocolLayer` | Header dispatch, TP responses, TX arbitration |
| `dpp.py` | `DPPReader`, `DPPSplitter`, `DPPPacker` | Data payload readers, splitters and width adapters |
| `endpoints.py` | `USB3Endpoint` | Base class: protocol layer interface of an endpoint |
| `bulk.py` | `USB3BulkIN`, `USB3BulkOUT` | Bulk endpoints with bursts and flow control, endpoint converters |
| `iso.py` | `USB3IsoIN` | Isochronous IN endpoint |
| `slots.py` | `USB3PacketSlots`, `USB3PacketWriter` | Packet buffers of the Bulk and Isochronous endpoints |
| `control.py` | `USB3EndpointControl` | Control endpoint (EP0), standard requests, request handler |
| `descriptors.py` | `USB3Descriptors` | Descriptors builder and requests table |
| `wishbone.py` | `USB3EP0Wishbone` | Wishbone master over EP0 vendor requests |
| `bridge.py` | `USB3BytePacker`, `USB3ByteUnpacker` | Endpoint words to and from byte streams |
| `cdc_acm.py` | `USB3CDCACM` | CDC-ACM serial port device |
| `uvc.py` | `USB3UVC` | UVC camera device |
| `msc.py` | `USB3MSC`, `USB3MSCSCSI` | Mass Storage device and SCSI engine |
| `msc_storage.py` | `USB3MSCRAMStorage`, `USB3MSCDRAM` | Block RAM and DRAM storages of the Mass Storage device |
| `bench.py` | `USB3Bench` | Bulk bandwidth source and checker |
| `trace.py` | `USB3CoreTrace`, `USB3LinkTrace` | Header packet and link event traces (debug) |
| `crc.py` | `CRC5`, `CRC16`, `CRC32` | Parallel CRCs, shared with the PHY |
| `common.py` | layouts, constants | Packet types, link commands, stream layouts |
| `examples.py` | `add_usb3_example` | Example designs shared by the targets |
| `host_model.py` | `USB3HostModel` | Host models for simulation and tests |

## Limitations {#core-limits}

- Gen 1x1 and Gen 2x1 only; no Gen 1x2 or Gen 2x2 link layer.
- SuperSpeedPlus device features are limited to what a device needs: no hub support, no deferred
  packets, no arbitration weight handling.
- 64-bit and 128-bit words for the Bulk endpoints only; the other endpoints stay 32-bit. The
  CDC-ACM and Mass Storage device classes run on any link layer width through the endpoint
  converters. The `uvc`, `uvc_iso` and `composite` examples are built on the 32-bit link layer
  only.
- No Isochronous OUT endpoint, no Streams. An Interrupt IN endpoint is declared with a Bulk IN
  endpoint behind it, as the CDC-ACM notification endpoint, which never sends.
- No endpoint halt: SET_FEATURE(ENDPOINT_HALT) is accepted without effect.
- U1 and U2 entry is host-initiated only, and validated in simulation only. U3 entry and exit
  (Linux autosuspend and resume) are validated on hardware.
- No function remote wakeup, no Latency Tolerance Messaging (LTM), no Precision Time Measurement
  (PTM).

Where to go next: [Device classes](#classes) for complete devices, [Integration](#integration)
to add the core to a design, [Performance](#performance) for throughput and resources.
