# Device classes {#classes}

This chapter describes the complete devices built on the device core: vendor Bulk, CDC-ACM serial
port, UVC camera and Mass Storage, and how to combine them into one composite device. Each one
works with the standard Linux drivers, so the host needs no software beyond the usual tools. Read
[Device core architecture](#core-architecture) first for the endpoints and the control endpoint.

## Overview {#classes-overview}

Each class comes in up to two forms. The *device* class builds a whole device core on a PHY, with
its descriptors and endpoints: one line of code gives a working device. The *function* class
attaches to the EP0 and endpoints of an existing core, so several functions can share one device.

| Class | Device class | Function class | Linux driver, host view |
|------------|---------------|---------------------------|-------------------------|
| Vendor Bulk | none | none | usbfs (libusb) |
| CDC-ACM | `USB3CDCACM` | `USB3CDCACMFunction` | `cdc_acm`, `/dev/ttyACMx` |
| UVC | `USB3UVC` | `USB3UVCFunction` | `uvcvideo`, `/dev/videoX` |
| Mass Storage | `USB3MSC` | none | `usb-storage`, `/dev/sdX` |

The examples of [Examples](#examples) instantiate each class on the boards. The descriptors use
the vendor ID `0x1d50` and product ID `0x605a` by default.

The CDC-ACM and Mass Storage device classes follow the width of the link layer, taken from the PHY
`link_words`. Their Bulk endpoints are as wide as the link layer, and their 32-bit logic connects
through the [Endpoint converters](#core-converters). The UVC classes and the `composite` example
use 32-bit endpoints, so these examples are built on the 32-bit link layer.

## Vendor Bulk {#classes-bulk}

The simplest device is a vendor-specific interface with one Bulk IN and one Bulk OUT endpoint. The
host reaches it with libusb, without a kernel driver. `bulk_descriptors()` builds its descriptors,
and the [Usage](#core-usage) section of the previous chapter shows the code. The `loopback`,
`wishbone` and `bench` examples use it.

| Parameter | Default | Meaning |
|---------------|---------|----------------------------------|
| `in_ept` | 1 | Bulk IN endpoint number |
| `out_ept` | 2 | Bulk OUT endpoint number |
| `max_burst` | 8 | Max burst of both endpoints |
| `vid`, `pid` | `0x1d50`, `0x605a` | Vendor and product IDs |

The bandwidth benchmark `USB3Bench` (`usb3_core/bench.py`) plugs into these endpoints. Its source
sends incrementing 32-bit counter words at full rate on Bulk IN, one, two or four per endpoint
word of 32, 64 or 128 bits. Its checker verifies the same
counter on Bulk OUT and counts the errors. `bench/usb3_bench.py` drives it from the host.

## CDC-ACM serial port {#classes-cdc}

A CDC-ACM device appears on Linux as a serial port, `/dev/ttyACMx`, with the standard `cdc_acm`
driver. `USB3CDCACM` exposes it as a LiteX UART PHY: an 8-bit `sink` toward the host and an 8-bit
`source` from the host. It can be the SoC console of the LiteX BIOS, or carry the UARTBone protocol
of `litex_server --uart`.

| Item | Value |
|-----------------------|-------------------------------------------------------------------|
| Interfaces | Communication (class `0x02`), Data (class `0x0a`), grouped by an IAD |
| Endpoints | EP1 Bulk IN, EP2 Bulk OUT (data), EP3 Interrupt IN (notifications) |
| Endpoint width | Link layer width; 32-bit function through the converters |
| Class requests | SET_LINE_CODING, GET_LINE_CODING (115200 8N1), SET_CONTROL_LINE_STATE |
| Max burst | 2 by default (`max_burst`) |
| Byte stream | 8-bit `sink` and `source` |
| Status | `connect`: a terminal has opened the port |

The notification endpoint is declared as Interrupt IN but never sends. The line coding is accepted
and ignored: the data moves at USB 3 speed whatever baud rate the terminal sets.

The device drops the bytes toward the host until a terminal opens the port. A console that writes
before the host is ready therefore never stalls the SoC. The terminal is detected from the DTR bit
of SET_CONTROL_LINE_STATE and reported on `connect`.

Bytes toward the host are packed into 32-bit words (`USB3BytePacker`). A packet ends after 64 idle
cycles or at 1024 bytes, so a short message reaches the host without waiting for a full packet.

```python
from litex.soc.cores.uart import UART
from usb3_core.cdc_acm    import USB3CDCACM

self.usb3_acm = USB3CDCACM(usb3_phy, sys_clk_freq)
self.uart     = UART(self.usb3_acm)
```

On the host, open the port with `litex_term /dev/ttyACM0`. For the `cdc_uartbone` example, bridge
the SoC bus with `litex_server --uart --uart-port /dev/ttyACM0` instead.

## UVC camera {#classes-uvc}

A UVC device appears on Linux as a video device, `/dev/videoX`, with the standard `uvcvideo`
driver. `USB3UVC` streams an uncompressed YUY2 test pattern: colour bars, a moving marker and a
moving luma band. It is a complete reference of a video source; a design with real video replaces
the pattern generator (`USB3UVCStream`) with its own payload source.

| Item | Value |
|-----------------------|-------------------------------------------------------------------|
| Specification | USB Video Class 1.1 |
| Interfaces | VideoControl and VideoStreaming, grouped by an IAD |
| Terminals | Camera input terminal, streaming output terminal |
| Format | Uncompressed YUY2, 16 bits per pixel |
| Bulk modes | 640x480 at 60 fps, 1280x720 at 60 fps, 1920x1080 at 30 and 60 fps |
| Isochronous modes | 640x480 at 60 fps, 1280x720 at 30 and 60 fps, 1920x1080 at 30 fps |
| Payload header | 12 bytes: FID, EOF, PTS, SCR |
| Controls | VideoStreaming PROBE and COMMIT |

The figure below shows the function. The host picks a mode with PROBE and COMMIT on EP0. The
pattern generator then produces frames, cut into payloads that each start with a payload header.

![UVC function. Follow the video from the test pattern through the payload header block to the
video endpoint of the core and the host (cyan arrow); PROBE and COMMIT on EP0 set the mode and
start the stream.](class_uvc.svg)

The host selects a mode by its frame descriptor and frame interval. The function answers SET_CUR,
GET_CUR, GET_MIN, GET_MAX, GET_DEF, GET_LEN and GET_INFO on the PROBE and COMMIT controls, and
stalls the other requests.

### Bulk streaming {#classes-uvc-bulk}

With Bulk streaming, the default, each payload is 32768 bytes, header included. The last payload
of a frame ends with a short packet, which marks the end of the frame; the build checks this for
every mode. The stream starts on COMMIT. It stops on a link reset or on
CLEAR_FEATURE(ENDPOINT_HALT) of the video endpoint, as `uvcvideo` does.

### Isochronous streaming {#classes-uvc-iso}

With `transfer="iso"`, the VideoStreaming interface has two alternate settings: 0 without an
endpoint and 1 with the Isochronous IN endpoint. Each service interval of 125 µs carries one
payload of max burst x 1024 bytes: 16 KiB with the default max burst of 16, about 131 MB/s.
Zero-length packets fill the intervals between frames.

That bandwidth sets the list of isochronous modes. 1920x1080 at 60 fps in YUY2 needs about
249 MB/s, so it only runs with Bulk streaming. In exchange, the host reserves the isochronous
bandwidth: the stream keeps its rate when other devices share the bus, as on most commercial
cameras.

The host starts the stream by selecting alternate setting 1 after COMMIT, and stops it with
alternate setting 0 or a link reset.

| Parameter | Default | Meaning |
|------------------|-------------------|----------------------------------------------|
| `modes` | `UVC_MODES` or `UVC_ISO_MODES` | List of (width, height, fps) |
| `transfer` | `"bulk"` | `"bulk"` or `"iso"` |
| `max_burst` | 16 | Max burst of the video endpoint |
| `payload_size` | 32768 (Bulk), max burst x 1024 (Isochronous) | UVC payload size in bytes |
| `interval` | 1 | Isochronous `bInterval`: 2^(interval - 1) x 125 µs |

A mode width must be a multiple of 16 and its height a multiple of 8. On the host:

```sh
v4l2-ctl --list-formats-ext
ffplay -f v4l2 -input_format yuyv422 -video_size 1920x1080 \
    -framerate 60 /dev/video0
```

## Mass Storage {#classes-msc}

A Mass Storage device appears on Linux as a disk, `/dev/sdX`, with the standard `usb-storage`
driver. `USB3MSC` implements the Bulk-Only Transport and a SCSI command engine on top of a
storage: block RAM for a small disk, or DRAM for a large one.

| Item | Value |
|-----------------------|-------------------------------------------------------------------|
| Interface | Class `0x08`, subclass `0x06` (SCSI transparent), protocol `0x50` (Bulk-Only) |
| Endpoints | EP1 Bulk IN, EP2 Bulk OUT, max burst 8 |
| Endpoint width | Link layer width; 32-bit SCSI engine through the converters |
| Class requests | Get Max LUN (one LUN), Bulk-Only Mass Storage Reset |
| Block size | 512 bytes |
| Storages | Block RAM (`USB3MSCRAMStorage`, 512 KiB by default), DRAM (`USB3MSCDRAM`) |

### Bulk-Only Transport {#classes-msc-bot}

Every command has three steps, as the figure below shows. The host sends a 31-byte Command Block
Wrapper (CBW) on Bulk OUT. The data phase, if any, moves blocks on Bulk IN or Bulk OUT. The device
then returns a 13-byte Command Status Wrapper (CSW) on Bulk IN.

![Bulk-Only Transport. Read each command top to bottom: CBW, data phase (cyan), then CSW; the
CSW of the WRITE(10) waits until the storage has committed the data.](class_msc_bot.svg)

The CSW of a WRITE(10) is sent only when the storage is idle, with the data committed. A READ(10)
that follows therefore returns the written data.

### SCSI commands {#classes-msc-scsi}

| Command | Handling |
|--------------------------------|------------------------------------------------------|
| TEST UNIT READY | Status from the storage `ready` |
| REQUEST SENSE | Sense of the last command |
| INQUIRY | Removable direct-access device, vendor `EnjoyDig` |
| MODE SENSE(6) | Constant response, not write-protected |
| PREVENT ALLOW MEDIUM REMOVAL | Accepted |
| READ FORMAT CAPACITIES | Formatted medium, capacity of the storage |
| READ CAPACITY(10) | Last block and block size |
| READ(10), WRITE(10) | Block transfers through the storage |
| SYNCHRONIZE CACHE(10) | Accepted |
| Other commands | Fail with ILLEGAL REQUEST; their data phase is padded or discarded |

A READ(10) or WRITE(10) beyond the last block fails with ILLEGAL REQUEST. When the storage is not
ready, for example a DRAM still being initialized, commands fail with NOT READY and the host
retries.

### Storages {#classes-msc-storage}

The SCSI engine reaches its storage through a small stream interface, `USB3MSCStorage`. A new
storage only needs to implement it:

- `cmd`: a stream of commands, with the direction (`write`), the logical block address and the
  block count;
- `source` and `sink`: 32-bit streams of read and write data;
- `ready`: the storage is available;
- `idle`: no command running, write data committed.

| Storage | Medium | Initial content |
|----------------------------|------------------------------------|-------------------------------|
| `USB3MSCRAMStorage` | Block RAM, 512 KiB in the `msc` example | Optional disk image |
| `USB3MSCDRAM` | DRAM, two LiteDRAM native ports | None: format from the host |

`msc_fat_image()` builds a FAT disk image with files at build time. It needs the `mkfs.fat` and
`mcopy` tools on the build machine; without them, the disk starts unformatted.

The DRAM storage keeps the disk little-endian at `base`, so the CPU sees it at `main_ram + base`.
Its content is lost at power-up: format it from the host before mounting it. The `msc_dram`
example places the disk in the upper half of the DRAM and leaves the lower half to the CPU.

```python
from usb3_core.msc         import USB3MSC
from usb3_core.msc_storage import USB3MSCDRAM

# 256 MiB disk in the upper half of a 512 MiB DRAM.
storage = USB3MSCDRAM(soc.sdram.crossbar.get_port("read"),
    soc.sdram.crossbar.get_port("write"),
    size=256*1024*1024, base=256*1024*1024)
soc.usb3_msc = USB3MSC(usb3_phy, sys_clk_freq, storage=storage)
```

```sh
sudo mkfs.vfat /dev/sdX
udisksctl mount -b /dev/sdX
```

## Composite devices {#classes-composite}

A composite device exposes several functions through one device core. Each function brings its
interfaces, grouped by an Interface Association Descriptor, and uses its own endpoints. The device
class is `DEVICE_CLASS_IAD`, so that the host binds one driver per function.

The figure below shows the `composite` example: a UVC camera on interfaces 0 and 1 and a CDC-ACM
console on interfaces 2 and 3.

![Composite device of the composite example. Look at the two groups: the UVC function owns
interfaces 0 and 1 and EP1; the CDC-ACM function owns interfaces 2 and 3 and EP2 to
EP4.](class_composite.svg)

To build one, follow four steps:

1. Concatenate the interface lists of the functions, with interface and endpoint numbers that do
   not overlap.
2. Add the constant class requests of the functions to the descriptors.
3. Create the core with the request handler and one endpoint per declared endpoint.
4. Attach each function to EP0 and to its endpoints.

```python
uvc_ifs = uvc_interfaces(UVC_MODES, vc_interface=0, ept=1, max_burst=16,
    clk_freq=sys_clk_freq)
acm_ifs = cdc_acm_interfaces(comm_interface=2, notif_ept=4, in_ept=2,
    out_ept=3, max_burst=2)
descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD,
    interfaces=uvc_ifs + acm_ifs)
add_cdc_acm_requests(descriptors)
core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
    endpoints = [
        USB3BulkIN(ept=1,  max_burst=16), # UVC video.
        USB3BulkIN(ept=2,  max_burst=2),  # CDC-ACM data IN.
        USB3BulkOUT(ept=3, max_burst=2),  # CDC-ACM data OUT.
        USB3BulkIN(ept=4,  max_burst=2),  # CDC-ACM notifications.
    ],
    with_ep0_handler = True)
uvc = USB3UVCFunction(core.ep0, core.endpoints[0], core.link.reset,
    sys_clk_freq)
acm = USB3CDCACMFunction(core.ep0, core.endpoints[1], core.endpoints[2])
```

Only one function may answer through the request handler, because `req_stall` and `req_in` have a
single driver. Here the CDC-ACM requests are constant entries of the requests table, so the UVC
function owns the handler and the CDC-ACM function only watches `req_done`. A UVC function that
is not on interfaces 0 and 1 needs its `vs_interface` argument.

The order of the endpoint list sets their transmit priority in the protocol layer.

As written, this code needs a 32-bit link layer: the core refuses the 32-bit CDC-ACM Bulk OUT
endpoint on a wider one. On a 64-bit or 128-bit link layer, build the CDC-ACM Bulk endpoints at
the link layer width and attach the function through `bulk_endpoint_32()`, as `USB3CDCACM` does.

Where to go next: [Examples](#examples) to build these devices on a board,
[Performance](#performance) for their measured throughput.
