# USB3 Device Core (`usb3_core`)

Migen/LiteX USB 3.2 Gen 1x1 (SuperSpeed, 5Gbps) device core: Link Layer, Protocol Layer, Control
Endpoint, Bulk/Isochronous Endpoints and device classes (CDC-ACM, UVC, Mass Storage). It runs at the PIPE rate
(32-bit words at 125MHz) in the `sys` clock domain of the PHY ([usb3-pipe.md](usb3-pipe.md)).

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
| `USB3LinkLayer` | `link.py` | Framing (HP/LC/DPP), CRCs, header flow control: 3-bit Header Sequence Numbers, 4 Rx/Tx Header Buffers, LGOOD_n/LCRD_x, LBAD/LRTY retries (DL bit on resent DPHs), Advertisements on each U0 entry (state kept across Recovery), PENDING_HP/CREDIT_HP timers (Recovery), LUP keep-alive, LMPs (Port Capability/Configuration, Set Link Function, U2 Inactivity Timeout), LGO_U3 accepted, LGO_U1/U2 accepted when idle with `with_lpm` (else LXU), LPMA forwarded, U1/U2 exit on pending packets. |
| `USB3ProtocolLayer` | `protocol.py` | Received headers dispatch (in order, broadcast to the endpoints), DPP status pairing/routing, TP responses (with the Device Address of their generation), PING_RESPONSE, TX arbitration, DPPs sent in DPH order. |
| `USB3BulkIN`/`USB3BulkOUT` | `bulk.py` | Bulk endpoints with bursts (MaxBurst 2-16, packet slots rings), NumP flow control, retries, EOB/NRDY/ERDY, short packets ending bursts, `flush`. |
| `USB3IsoIN` | `iso.py` | Isochronous IN endpoint (MaxBurst 1-16, Mult 0): one transfer per service interval, sent on the host ACK TPs (Seq Num 0, then Seq Num + NumP), LPF on the last packet, zero-length DP when no data, no retry/flow control, `flush`. |
| `USB3EndpointControl` | `control.py` | EP0: standard requests and constant class requests from a table (descriptors ROM), U1/U2 Enable (SET/CLEAR_FEATURE, GET_STATUS), SET/GET_INTERFACE (alternate settings), class/vendor request handler (`with_ep0_handler`). |
| `USB3Descriptors` | `descriptors.py` | Descriptors builder (device, configuration with SuperSpeed companions, alternate settings, BOS, strings, IADs), Chapter 9 requests constants, `bulk_descriptors`/`iso_in_descriptors` helpers. |
| `USB3DeviceCore` | `device.py` | Link + Protocol + EP0 + endpoints (`with_lpm`, `with_ep0_handler`), `add_csr()`. |
| `USB3CDCACM(Function)` | `cdc_acm.py` | CDC-ACM serial port (descriptors/class requests), LiteX UART PHY interface (`sink`/`source` bytes, `connect`). |
| `USB3ByteUnpacker`/`USB3BytePacker` | `bridge.py` | 32-bit endpoint words (`be`/`last`) to/from byte streams (CDC-ACM). |
| `USB3UVC(Function)` | `uvc.py` | UVC 1.1 camera: PROBE/COMMIT, payload headers (FID/EOF/PTS/SCR), YUY2 test pattern, several modes, Bulk (default) or Isochronous (`transfer="iso"`) streaming. |
| `USB3MSC` | `msc.py` | Mass Storage Bulk-Only Transport, SCSI command engine (`USB3MSCSCSI`) on a storage (`USB3MSCStorage` stream interface, `msc_storage.py`): block RAM disk (FAT image, `USB3MSCRAMStorage`) or DRAM disk (LiteDRAM DMAs, `USB3MSCDRAM`). |
| `USB3Bench` | `bench.py` | Bandwidth benchmark source/checker. |
| `USB3CoreTrace` | `trace.py` | Header Packets trace (debug). |
| `CRC5/CRC16/CRC16Words/CRC32` | `crc.py` | Parallel CRCs (equations generated from the polynoms). |
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

- Gen 1x1 only ([gen2-x2-roadmap.md](gen2-x2-roadmap.md)).
- Bulk/Control endpoints, Isochronous IN (Mult of 0): no Isochronous OUT, no Streams. Interrupt IN
  endpoints can be declared with a Bulk IN endpoint (ex: CDC-ACM notifications, never sent).
- U1/U2 rejected by default (descriptors advertise 0 exit latencies: hosts disable LPM). With
  `with_lpm`: host initiated U1/U2 entry only (no device initiated entry, U1/U2 Enable only reported),
  validated in simulation only (not on hardware yet). U3 entry/exit through the LTSSM (not validated
  on hardware).
- Function remote wakeup, LTM, PTM not supported.
