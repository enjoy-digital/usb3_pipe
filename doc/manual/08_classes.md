# Device classes {#classes}

This chapter describes the complete devices built on the device core: vendor Bulk, CDC-ACM serial
port, CDC-ECM network adapter, UVC camera and Mass Storage, and how to combine them into one
composite device. Each one
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
| CDC-ECM | none | `USB3CDCECMFunction` | `cdc_ether`, network interface |
| CDC-NCM | none | `USB3CDCNCMFunction` | `cdc_ncm`, network interface |
| UVC | `USB3UVC` | `USB3UVCFunction` | `uvcvideo`, `/dev/videoX` |
| Mass Storage | `USB3MSC` | none | `usb-storage`, `/dev/sdX` |
| GUD display | none | `USB3GUDFunction` | `gud`, DRM display and `/dev/fbX` |

The examples of [Examples](#examples) instantiate each class on the boards. The descriptors use
the vendor ID `0x1d50` and product ID `0x605a` by default.

The CDC-ACM and Mass Storage device classes follow the width of the link layer, taken from the PHY
`link_words`. Their Bulk endpoints are as wide as the link layer, and their 32-bit logic connects
through the [Endpoint converters](#core-converters). The UVC classes, the CDC-ECM function and
the `composite` example use 32-bit endpoints, so they are built on the 32-bit link layer.

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

## Network adapters {#classes-net}

A CDC-ECM or CDC-NCM function makes the device a USB network adapter. The host sees a network
interface, and the design gets Ethernet frames on two 32-bit streams, for example to bridge them
to an Ethernet MAC. Both functions have the same user side, so a design switches from one to the
other without other change.

| Item | CDC-ECM | CDC-NCM |
|-----------------------|--------------------------------|---------------------------------------|
| Host drivers | Linux `cdc_ether`, macOS | Linux `cdc_ncm`, Windows 10+, macOS |
| Transfer | one frame per transfer | one NCM Transfer Block (NTB16) per transfer |
| Frames toward the host | one per transfer | one per NTB |
| Frames from the host | one per transfer | several per NTB |
| Class requests | SET_ETHERNET_PACKET_FILTER | NTB parameters, format and input size, packet filter |

Each function has a Communication interface with a notification endpoint and a Data interface
with Bulk IN and Bulk OUT endpoints. The Data interface has an empty alternate setting 0, and the
host selects setting 1 to start the traffic. A transfer ends with a short packet or a zero-length
packet, so the Bulk OUT endpoint is declared `with_zlp`.

The two sections below describe each function.

### CDC-ECM {#classes-ecm}

A CDC-ECM device appears on Linux as an Ethernet interface, with the standard `cdc_ether` driver.
Ethernet frames move between the host and the FPGA as on a network card, so the FPGA side can
bridge them to an Ethernet MAC or terminate them in its own network stack. `USB3CDCECMFunction`
is a function class: it attaches to the EP0 and endpoints of a core built as a composite device.

| Item | Value |
|-----------------------|-------------------------------------------------------------------|
| Specification | CDC ECM 1.2 (Ethernet Control Model) |
| Interfaces | Communication (class `0x02`, subclass `0x06`), Data (class `0x0a`), grouped by an IAD |
| Data alternate settings | 0 without endpoints, 1 with Bulk IN and Bulk OUT |
| Endpoints | EP1 Bulk IN, EP2 Bulk OUT (data), EP3 Interrupt IN (notifications) |
| Class requests | SET_ETHERNET_PACKET_FILTER (accepted, no filtering) |
| Max segment size | 1514 bytes: one Ethernet frame without its FCS |
| Frame streams | 32-bit `sink` and `source`, one stream packet per frame |
| Status | `link`: network connection reported to the host |

Each frame is one host transfer. Toward the host, the function sends the frame as maximum-size
packets and a short last packet. A frame of exactly N x 1024 bytes is followed by a zero-length
packet, so that the host sees its end.

From the host, a short packet or a zero-length packet ends the frame. The Bulk OUT endpoint
therefore needs `with_zlp=True`, which turns a zero-length packet into an end marker (see
[The user stream](#core-stream)). The function holds one word back, because a full last packet
only ends the frame when the zero-length packet that follows arrives.

The notification endpoint sends two notifications when the host configures the device and on
every change of `link`. NETWORK_CONNECTION carries the connection state; CONNECTION_SPEED_CHANGE
carries the `speed` parameter as both the upstream and the downstream bit rate.

| Parameter | Default | Meaning |
|--------------------|-----------------|----------------------------------------------|
| `comm_interface` | 0 | Communication interface; data interface on the next number |
| `speed` | 1000000000 | Bit rate in the CONNECTION_SPEED_CHANGE notification |
| `mac_string` | 4 | String index of the MAC address (`cdc_ecm_interfaces()`) |
| `max_burst` | 8 | Max burst of the data endpoints (`cdc_ecm_interfaces()`) |

The MAC address goes in a string descriptor of 12 hexadecimal digits. `add_cdc_ecm_strings()` adds
it and returns its index. The host gives that address to its network interface, so the FPGA side
uses another one. The code below builds the device, with the frames from the host looped back to
it through a FIFO.

```python
from litex.soc.interconnect import stream

from usb3_core.common      import endpoint_layout
from usb3_core.device      import USB3DeviceCore
from usb3_core.descriptors import USB3Descriptors, DEVICE_CLASS_IAD
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.cdc_ecm     import *

descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD)
mac = add_cdc_ecm_strings(descriptors, 0x02eccc000001)
descriptors.interfaces = cdc_ecm_interfaces(0, notif_ept=3, in_ept=1,
    out_ept=2, mac_string=mac)
add_cdc_ecm_requests(descriptors)
core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
    endpoints = [
        USB3BulkIN(ept=1,  max_burst=8),                # Frames to the host.
        USB3BulkOUT(ept=2, max_burst=8, with_zlp=True), # Frames from the host.
        USB3BulkIN(ept=3,  max_burst=2),                # Notifications.
    ],
    with_ep0_handler = True)
eps = core.endpoints
ecm  = USB3CDCECMFunction(core.ep0, eps[0], eps[1], eps[2])
echo = stream.SyncFIFO(endpoint_layout(32), 1024, buffered=True)
self.comb += [
    ecm.source.connect(echo.sink),
    echo.source.connect(ecm.sink),
]
```

The function works on 32-bit endpoints, so this device needs a 32-bit link layer. On the host,
with a locally administered MAC address as here, the interface appears as `usb0`, or as `enx`
followed by the MAC address with predictable interface names. Bring it up and configure it as any
Ethernet interface:

```sh
ip -br link
sudo ip link set usb0 up
```

### CDC-NCM {#classes-ncm}

A CDC-NCM device appears as a network interface on Linux, with the standard `cdc_ncm` driver, and
on Windows 10 and later without a vendor driver. It packs several frames in one transfer, which
keeps the host overhead low with small frames.

The NCM function buffers each NTB from the host in a 16 KiB block RAM (`NCMNTBParser`). It then
walks the NTB headers and sends each datagram as a frame. Toward the host, it stores each frame to
know its length, then sends it as one NTB (`NCMNTBBuilder`).

```python
mac = add_cdc_ecm_strings(descriptors, 0x02eccc000001)
descriptors.interfaces = cdc_ncm_interfaces(0, notif_ept=3, in_ept=1, out_ept=2,
    mac_string=mac)
add_cdc_ncm_requests(descriptors)
core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
    USB3BulkIN(ept=1), USB3BulkOUT(ept=2, with_zlp=True), USB3BulkIN(ept=3)],
    with_ep0_handler=True)
ncm = USB3CDCNCMFunction(core.ep0, *core.endpoints)
# ncm.sink: frames to the host, ncm.source: frames from the host, ncm.link: link status.
```

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
storage: block RAM for a small disk, or DRAM for a large one. With a list of storages, the device
has one logical unit (LUN) per storage, and the host sees one disk per LUN.

| Item | Value |
|-----------------------|-------------------------------------------------------------------|
| Interface | Class `0x08`, subclass `0x06` (SCSI transparent), protocol `0x50` (Bulk-Only) |
| Endpoints | EP1 Bulk IN, EP2 Bulk OUT, max burst 8 |
| Endpoint width | Link layer width; 32-bit SCSI engine through the converters |
| Class requests | Get Max LUN (number of storages minus one), Bulk-Only Mass Storage Reset |
| Logical units | One per storage, selected by the LUN field of each CBW |
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
| START STOP UNIT | Accepted, no effect |
| PREVENT ALLOW MEDIUM REMOVAL | Accepted |
| READ FORMAT CAPACITIES | Formatted medium, capacity of the storage |
| READ CAPACITY(10) | Last block of the storage `capacity`, block size |
| READ(10), WRITE(10) | Block transfers through the storage |
| SYNCHRONIZE CACHE(10) | Accepted |
| Other commands | Fail with ILLEGAL REQUEST; their data phase is padded or discarded |

A READ(10) or WRITE(10) beyond the last block fails with ILLEGAL REQUEST. A command to a LUN the
device does not have fails with ILLEGAL REQUEST and LOGICAL UNIT NOT SUPPORTED.

When the storage is not ready, commands fail with NOT READY and the host retries. The additional
sense tells why: LOGICAL UNIT NOT READY for a storage becoming ready, such as a DRAM still being
initialized, and MEDIUM NOT PRESENT for a removable storage without its medium, such as an empty
SD card slot. Linux sends START STOP UNIT to such a LUN, which is why the engine accepts it.

When a storage becomes ready, the next command to its LUN fails once with UNIT ATTENTION and
MEDIUM MAY HAVE CHANGED. The host then reads the capacity again, so a medium inserted at runtime
shows up with its size. INQUIRY and REQUEST SENSE do not consume this UNIT ATTENTION.

### Storages {#classes-msc-storage}

The SCSI engine reaches its storage through a small stream interface, `USB3MSCStorage`. A new
storage only needs to implement it:

- `cmd`: a stream of commands, with the direction (`write`), the logical block address and the
  block count;
- `source` and `sink`: 32-bit streams of read and write data;
- `ready`: the storage is available;
- `idle`: no command running, write data committed;
- `capacity`: the size in blocks, constant for a storage built with a `size`, driven by the
  storage itself when built with `size=None` (an SD card or a SATA drive detected at runtime).

A storage built with `removable=True` reports MEDIUM NOT PRESENT when it is not ready.

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

### Several LUNs {#classes-msc-luns}

`USB3MSC` takes a list of storages for `storage`, and then a list of INQUIRY product strings for
`inquiry_product`, one per LUN. Fixed storages and storages detected at runtime can share one
device, each with its own ready state and capacity.

```python
from usb3_core.msc         import USB3MSC
from usb3_core.msc_storage import USB3MSCRAMStorage

# LUN 0: 512 KiB block RAM disk; LUN 1: a removable storage of the design.
soc.usb3_msc = USB3MSC(usb3_phy, sys_clk_freq,
    storage         = [USB3MSCRAMStorage(512*1024), sd_storage],
    inquiry_product = ["USB3-PIPE RAMDisk", "USB3-PIPE SDCard"])
```

Here `sd_storage` stands for a `USB3MSCStorage` built with `size=None` and `removable=True`, which
drives `ready` and `capacity` from its medium.

## GUD display {#classes-gud}

A GUD device is a USB display for the Linux `gud` driver (Generic USB Display). The host sends
pixels over a Bulk OUT endpoint, and the device shows them, for example from a framebuffer on an
HDMI output. The driver binds on the vendor and product IDs `0x1d50` and `0x614d` and a
vendor-specific interface.

The function describes a fixed display, so all its requests are constant responses from the
requests table. It reports one mode, one connected HDMI connector and one pixel format, with no
EDID and no properties. `add_gud_requests()` builds these responses from LiteX video timings.

| Request | Response |
|-------------------------------|--------------------------------------------|
| `GET_DESCRIPTOR` | magic, version 1, `FULL_UPDATE`, one frame as buffer size |
| `GET_FORMATS` | one pixel format, `RGB565` by default |
| `GET_CONNECTORS`, `GET_CONNECTOR_STATUS` | one HDMI connector, connected |
| `GET_CONNECTOR_MODES` | one preferred mode from the video timings |
| `GET_PROPERTIES`, `GET_CONNECTOR_PROPERTIES`, `GET_CONNECTOR_EDID` | empty |
| `SET_*` | accepted |

The `FULL_UPDATE` flag asks the host for full frames. Each Bulk OUT transfer is then one whole frame,
in the pixel format and in raster order. The host sends `SET_BUFFER` only after a failed transfer,
to resynchronize. `USB3GUDFunction` gives the frame data as a stream and a pulse on `SET_BUFFER`:
the design writes the stream to its framebuffer and restarts at the frame start on the pulse.

```python
descriptors = USB3Descriptors(vid=GUD_VID, pid=GUD_PID,
    interfaces=gud_interfaces(0, out_ept=1, max_burst=16))
frame_size = add_gud_requests(descriptors, video_timings["1280x720@60Hz"])
core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
    endpoints=[USB3BulkOUT(ept=1, max_burst=16)])
gud  = USB3GUDFunction(core.ep0, core.endpoints[0])
# gud.source: frame data, gud.start: frame resynchronization.
```

The `RGB565` byte order of the host matches the `rgb565` format of the LiteX video framebuffer, so
the data goes to memory unchanged. On the ECPIX-5, a full 720p frame takes 8.5 ms (216 MB/s).
The X server of the NVIDIA proprietary driver does not drive GUD displays and can crash when one
appears. Test there without the kernel driver, from libusb.

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
