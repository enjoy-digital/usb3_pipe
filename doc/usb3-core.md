# USB3 Device Core (`usb3_core`)

Migen/LiteX USB 3.2 Gen 1x1 (SuperSpeed, 5Gbps) and Gen 2x1 (SuperSpeedPlus, 10Gbps) device core:
Link Layer, Protocol Layer, Control Endpoint, Bulk/Isochronous Endpoints and device classes (CDC-ACM,
UVC, Mass Storage). It runs in the `sys` clock domain of the PHY ([usb3-pipe.md](usb3-pipe.md)) on
32-bit Link Layer words at 125MHz (~500MB/s per direction), or 64-bit words with a PHY in words pairs
(`USB3PHY(link_words=2)`, Gen 2 bandwidth). With a Gen 2 capable PHY (`phy.gen2_active`), the Link
Layer switches to SuperSpeedPlus operation at Gen 2 and the BOS advertises the SuperSpeedPlus USB
Device Capability.

```
            USB3PHY (usb3_pipe)                         USB3DeviceCore (usb3_core)
 SerDes -> PIPE -> LTSSM                       link.py        protocol.py           endpoints
   source (data/ctrl) ------------------>  RX parser  --> Header dispatch ---> EP0 (control.py)
   sink   (data/ctrl) <------------------  TX framer  <-- TX arbitration <--- Bulk IN/OUT
   u0/polling_idle/hot_reset ----------->  Header flow control (seq/credits/retry)     |
   lgo_u1/2/3, tx_lau/tx_lxu <---------->  Link PM (U3 accepted, U1/U2 with_lpm)       |
   directed_recovery, link_activity <---   Timers / Recovery requests              Functions:
                                                                                   CDC-ACM, UVC, MSC
```

## Usage

```python
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

self.usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq,
    descriptors = bulk_descriptors(vid=0x1d50, pid=0x605a, max_burst=8),
    endpoints   = [USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)])
self.usb3_core.add_csr(with_trace=True) # Status/statistics CSRs, Header Packets trace.
# endpoints[0].sink: Device -> Host packets, endpoints[1].source: Host -> Device packets (32-bit words,
# first byte on data[7:0], be: valid bytes (full except on the last word), last: end of packet).
```

64-bit endpoints (with a `USB3PHY(link_words=2)`): `USB3BulkIN(ept, max_burst, data_width=64)`/
`USB3BulkOUT(..., data_width=64)`, user streams in `endpoint_layout(64)` (64-bit words, 8-bit `be`).
32-bit endpoints (the Control Endpoint, Isochronous IN, ...) are adapted by the Protocol Layer
(`DPPPacker`/`DPPRXAdapter`, `dpp.py`) at their 32-bit rate.

U1/U2 Link Power Management (`with_lpm=True`, off by default): the BOS advertises non-zero U1/U2
exit latencies (10us/2047us, hosts only enable U1/U2 with non-zero latencies), the Link Layer accepts
the host LGO_U1/LGO_U2 (LAU) when idle (7.2.4.2.2: all packets acknowledged/credited both ways,
nothing to transmit, no ACK TP expected by an endpoint), else rejects them (LXU). U1/U2 are entered
on the host LPMA, exited by the host or by the device when it has packets to transmit (ex: ERDY),
through Recovery (Advertisements on U0 re-entry). The U2 Inactivity Timeout LMP programs the LTSSM
U1 -> U2 timeout. SET/CLEAR_FEATURE(U1_ENABLE/U2_ENABLE) are tracked (`u1_enable`/`u2_enable`,
GET_STATUS(Device) D2/D3) but the device never initiates U1/U2 entry. Without `with_lpm`,
LGO_U1/LGO_U2 are rejected (LXU).

Device classes (complete devices, or functions to compose a device):

```python
from usb3_core.cdc_acm import USB3CDCACM # LiteX UART PHY: self.uart = UART(USB3CDCACM(phy, sys_clk_freq))
from usb3_core.uvc     import USB3UVC    # UVC camera (test pattern, transfer="bulk" or "iso").
from usb3_core.msc     import USB3MSC    # Mass Storage RAM disk (or storage=USB3MSCDRAM(...)).
```

## Modules

| Module | File | Description |
|---|---|---|
| `USB3LinkLayer` | `link.py` | Framing (HP/LC/DPP), CRCs, header flow control: 3-bit Header Sequence Numbers, 4 Rx/Tx Header Buffers, LGOOD_n/LCRD_x, LBAD/LRTY retries (DL bit on resent DPHs), Advertisements on each U0 entry (state kept across Recovery), PENDING_HP/CREDIT_HP timers (Recovery), LUP keep-alive, LMPs (Port Capability/Configuration, Set Link Function, U2 Inactivity Timeout), LGO_U3 accepted, LGO_U1/U2 accepted when idle with `with_lpm` (else LXU), LPMA forwarded, U1/U2 exit on pending packets. SuperSpeedPlus operation at Gen 2 (7.2.4.1): 4-bit Header Sequence Numbers, Type 1/Type 2 Tx Header Buffers (4 + 4) and Rx Buffer credits (LCRD1_x/LCRD2_x), Transfer Type in the ACK TPs/DPHs, nullified DPP after a retransmitted DPH, LGOOD_n/LCRD_x from the PHY `source_lc`. 32 or 64-bit DPP words (`data_width` from the PHY: RX words window, TX `DPPSplitter`). |
| `USB3ProtocolLayer` | `protocol.py` | Received headers dispatch (in order, broadcast to the endpoints), DPP status pairing/routing, TP responses (with the Device Address of their generation), PING_RESPONSE, TX arbitration, DPPs sent in DPH order, 32-bit endpoints on a 64-bit Link Layer through `DPPPacker`/`DPPRXAdapter`. |
| `DPPReader`/`DPPSplitter`/`DPPPacker`/`DPPRXAdapter` | `dpp.py` | Endpoints memories to continuous DPP streams (prefetched), 64-bit DPP words splitting (Link Layer TX), 32 <-> 64-bit DPP adapters. |
| `USB3BulkIN`/`USB3BulkOUT` | `bulk.py` | Bulk endpoints with bursts (MaxBurst 2-16, packet slots rings), NumP flow control, retries, EOB/NRDY/ERDY, short packets ending bursts, `flush`, 32/64-bit words (`data_width`). |
| `USB3IsoIN` | `iso.py` | Isochronous IN endpoint (MaxBurst 1-16, Mult 0): one transfer per service interval, sent on the host ACK TPs (Seq Num 0, then Seq Num + NumP), LPF on the last packet, zero-length DP when no data, no retry/flow control, `flush`. |
| `USB3PacketSlots`/`USB3PacketWriter` | `slots.py` | Packet slots of the Bulk/Iso endpoints (n x 1024-byte slots memory, per-slot lengths, slot addressing) and the IN endpoints user-side writer (packets split at Max Packet Size, `flush`, optional `enable` gating and per-transfer LPF). |
| `USB3EndpointControl` | `control.py` | EP0: standard requests and constant class requests from a table (descriptors ROM), U1/U2 Enable (SET/CLEAR_FEATURE, GET_STATUS), SET/GET_INTERFACE (alternate settings), class/vendor request handler (`with_ep0_handler`). |
| `USB3Descriptors` | `descriptors.py` | Descriptors builder (device, configuration with SuperSpeed companions, alternate settings, BOS, strings, IADs), Chapter 9 requests constants, `bulk_descriptors`/`iso_in_descriptors` helpers. |
| `USB3DeviceCore` | `device.py` | Link + Protocol + EP0 + endpoints (`with_lpm`, `with_ep0_handler`), `add_csr()`. |
| `USB3CDCACM(Function)` | `cdc_acm.py` | CDC-ACM serial port (descriptors/class requests), LiteX UART PHY interface (`sink`/`source` bytes, `connect`). |
| `USB3ByteUnpacker`/`USB3BytePacker` | `bridge.py` | 32-bit endpoint words (`be`/`last`) to/from byte streams (CDC-ACM). |
| `USB3UVC(Function)` | `uvc.py` | UVC 1.1 camera: PROBE/COMMIT, payload headers (FID/EOF/PTS/SCR), YUY2 test pattern, several modes, Bulk (default) or Isochronous (`transfer="iso"`) streaming. |
| `USB3MSC` | `msc.py` | Mass Storage Bulk-Only Transport, SCSI command engine (`USB3MSCSCSI`) on a storage (`USB3MSCStorage` stream interface, `msc_storage.py`): block RAM disk (FAT image, `USB3MSCRAMStorage`) or DRAM disk (LiteDRAM DMAs, `USB3MSCDRAM`). |
| `USB3Bench` | `bench.py` | Bandwidth benchmark source/checker. |
| `USB3CoreTrace`/`USB3LinkTrace` | `trace.py` | Header Packets trace (endpoint filter, circular mode) and Link Layer events trace (Link Commands, Header Packets, U0 entry/exit, Recovery requests with causes), debug. |
| `CRC5/CRC16/CRC16Words/CRC32` | `crc.py` | Parallel CRCs (equations generated from the polynoms), CRC-32 on 32 or 64-bit words. |
| Host models | `host_model.py` | Scripted Host gateware model (`sim.py`), Python host models for Migen tests. |
| Example designs | `examples.py` | `add_usb3_example()`/`USB3_EXAMPLES`: example designs shared by the targets (`bench/acorn.py`, `bench/kc705.py`, `bench/alibaba_xcku3p.py`), [examples.md](examples.md). |

## Control requests handler

With `USB3DeviceCore(..., with_ep0_handler=True)`, class/vendor requests not in the requests table are
delegated to user logic on `core.ep0`:

| Signals | Description |
|---|---|
| `req_setup`, `req_valid`, `req_type`, `req_request`, `req_value`, `req_index`, `req_length` | Request (pulse/level, SETUP fields). |
| `req_stall` | The handler stalls the request. |
| `req_in` (data 32, be, last) | IN Data stage response (up to 512 bytes), written after `req_setup`. |
| `req_out` (data 32, be, last) | OUT Data stage data (no back-pressure). |
| `req_done` | Status stage completed (all requests, ex: SET_INTERFACE/SET_CONTROL_LINE_STATE tracking). |

SET_INTERFACE is also decoded by EP0 for all devices: `set_interface` (pulse, Status stage),
`interface`, `alt_setting`. GET_INTERFACE returns the current alternate setting of the interface
(reset on SET_CONFIGURATION).

`USB3EP0Wishbone(ep0)` (`usb3_core/wishbone.py`) is a request handler giving a Wishbone master
(`bus`, add it with `soc.bus.add_master`) wire-compatible with LiteX CommUSB (`litex_server --usb`):
vendor requests `bmRequestType` 0xC3 (read)/0x43 (write), `bRequest` 0, `wValue`/`wIndex` = 32-bit
byte address, `wLength` = 4*N bytes for incrementing bursts of N words (N <= 128, CommUSB uses N=1).
Writes are buffered (2 full-size requests) and executed before the following reads, other requests
are stalled.

## Isochronous IN endpoints

`USB3IsoIN(ept, max_burst)` (USB 3.2 8.12.6, Mult of 0): `sink` takes service interval transfers
(`last`: end of the transfer, up to `max_burst` x 1024 bytes), each one sent in one service interval
as 1024 bytes packets, the last one (short or not) with the Last Packet Flag (DPH bit 1:6). The host
requests the packets with ACK TPs (first one with a Seq Num of 0, then previous Seq Num + NumP):
a complete buffered transfer is sent from Seq Num 0 within the requested window, or a zero-length DP
(Seq Num 0, LPF) when no complete transfer is buffered (Table 8-33). No TP response, no retry, no
NRDY/ERDY; the end of a transfer interrupted by the next service interval is dropped. PINGs are
answered by the Protocol Layer (PING_RESPONSE), ITPs are ignored. Statistics: `n_transfers`,
`n_zlps`, `n_drops`.

Descriptors: `USB3EndpointDescriptor(ept, 1, kind=EP_ISOCHRONOUS, max_burst, interval, sync=EP_SYNC_ASYNC)`
(SuperSpeed companion: bMaxBurst, Mult 0, wBytesPerInterval = max_burst x 1024) in an interface
alternate setting (`USB3InterfaceDescriptor(..., alt=1)`, alternate setting 0 without endpoint: zero
bandwidth), see `iso_in_descriptors()`. The host selects the alternate setting (SET_INTERFACE) before
starting the service intervals: the user logic starts its stream on `ep0.set_interface`.

## Composite devices

Functions attach to a device's EP0/endpoints, descriptors are concatenated (see the `composite` example
in `usb3_core/examples.py`):

```python
descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD,
    interfaces = uvc_interfaces(UVC_MODES, vc_interface=0, ept=1) +
        cdc_acm_interfaces(comm_interface=2, notif_ept=4, in_ept=2, out_ept=3))
add_cdc_acm_requests(descriptors)
core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[USB3BulkIN(1, 16), USB3BulkIN(2, 2),
    USB3BulkOUT(3, 2), USB3BulkIN(4, 2)], with_ep0_handler=True)
uvc = USB3UVCFunction(core.ep0, core.endpoints[0], core.link.reset, sys_clk_freq)
acm = USB3CDCACMFunction(core.ep0, core.endpoints[1], core.endpoints[2])
```

## Mass Storage storages

`USB3MSCSCSI` (Bulk-Only Transport/SCSI) reads/writes blocks through a `USB3MSCStorage` interface:
`cmd` stream (`write`, `lba`, `blocks`), `source` (read data) and `sink` (write data) 32-bit streams,
`ready` (commands fail with NOT READY when low) and `idle` (write data committed: the WRITE(10) CSW
is sent once the storage is idle, so a following READ(10) returns the written data).

- `USB3MSCRAMStorage(size, image)`: block RAM disk, optionally initialized with a disk image
  (`msc_fat_image`).
- `USB3MSCDRAM(rd_port, wr_port, size, base, ready)`: DRAM disk on two LiteDRAM native ports
  (`crossbar.get_port("read")`/`get_port("write")`, DMA reader/writer + width converters). The disk
  is stored little-endian at `base` (visible from the CPU at `main_ram + base`), `ready` is typically
  the DRAM initialization status. Not initialized: format it from the host (`mkfs.vfat /dev/sdX`).

```python
storage = USB3MSCDRAM(soc.sdram.crossbar.get_port("read"), soc.sdram.crossbar.get_port("write"),
    size=256*1024*1024, base=256*1024*1024)
soc.usb3_msc = USB3MSC(phy, sys_clk_freq, storage=storage)
```

## Performance (Acorn CLE-215+, SFP2USB, Linux xHCI host)

| Test | Result |
|---|---|
| Bulk IN (`usb3_bench.py in --no-check`) | 446 MB/s |
| Bulk OUT (`usb3_bench.py out`) | 449 MB/s |
| Full duplex (`usb3_bench.py both`) | IN 376 MB/s + OUT 400 MB/s |
| 3 min full duplex soak, data checked | 139 GB, 0 errors, no LBAD/retry/Recovery |
| UVC 1920x1080@60 YUY2 | 60.00 fps (~249 MB/s) |
| MSC READ(10)/WRITE(10) | 186/227 MB/s (single command) |
| CDC-ACM UARTBone | ~6000 CSR reads/s |

Gen 2x1 (Alibaba XCKU3P, UltraScale+ GTY, SFP2USB, Linux xHCI host, `bench` example,
`usb3_bench.py --transfer-size 65536`, data checked, 0 errors, 0 LBAD, 0 Recovery):

| Link Layer | Bulk IN | Bulk OUT | Full duplex |
|---|---|---|---|
| 32-bit (default) | 426 MB/s | 466 MB/s | IN 325 + OUT 430 MB/s |
| 64-bit (`--link-words 2`) | 744 MB/s | 881 MB/s | IN 619 + OUT 736 MB/s (120s soak, 156 GB) |
| 64-bit, MaxBurst 16 (`--max-burst 16`) | 785 MB/s | 873 MB/s | IN 644 + OUT 757 MB/s |

At Gen 2, Bulk OUT with 64-bit words uses ~87% of the Link Layer bandwidth (8 bytes per cycle at
125MHz), Bulk IN waits for the host requests (ACK TPs behind the received DPPs). The same Gen 2
capable design (64-bit) also runs at Gen 1: 441 MB/s IN/OUT with the Gen 2 capability cleared
(`usb3_gen2_capable` bench CSR).

## Resources

Out-of-context synthesis/placement, xc7a200t-3, 125MHz (`bench/usb3_resources.py`):

| Block | LUTs | FFs | BRAM36 | WNS |
|---|---|---|---|---|
| Link Layer | 1649 | 1034 | 0 | +1.86ns |
| Control Endpoint (bulk descriptors) | 176 | 131 | 0 | +3.34ns |
| Bulk IN (MaxBurst 8) | 258 | 191 | 2 | +1.71ns |
| Bulk OUT (MaxBurst 8) | 171 | 165 | 2 | +2.09ns |
| Device (Link + Protocol + EP0 + Bulk IN/OUT) | 2550 | 1682 | 4 | +0.93ns |

A complete Acorn design (PHY + core + debug + bench) uses ~6000 LUTs (4.5% of the XC7A200T).

## Limitations

- Gen 1x1 and Gen 2x1 (SuperSpeedPlus device: no hub/deferred/arbitration weight handling), no
  Gen 1x2/Gen 2x2 Link Layer yet ([gen2-x2-roadmap.md](gen2-x2-roadmap.md)).
- 64-bit words for the Bulk endpoints only (`link_words=2`): the other endpoints/device classes are
  32-bit (adapted by the Protocol Layer, at their 32-bit rate); the `cdc_*`/`console`/`uvc*`/`msc*`/
  `composite` examples use the 32-bit Link Layer.
- Bulk/Control endpoints, Isochronous IN (Mult of 0): no Isochronous OUT, no Streams. Interrupt IN
  endpoints can be declared with a Bulk IN endpoint (ex: CDC-ACM notifications, never sent).
- U1/U2 rejected by default (descriptors advertise 0 exit latencies: hosts disable LPM). With
  `with_lpm`: host initiated U1/U2 entry only (no device initiated entry, U1/U2 Enable only reported),
  validated in simulation only (not on hardware yet: the AMD xHCI host of the bench never enables
  USB3 LPM). U3 entry/exit (Linux autosuspend/resume) validated on hardware.
- Function remote wakeup, LTM, PTM not supported.
