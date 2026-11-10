# Examples {#examples}

This chapter describes the example device designs shipped with the project and the host tools that
exercise them. Each example is a complete USB 3 device; most need no host driver beyond the
standard Linux ones. Use them to check a board, to measure throughput, or as a starting point for
your own device.

## At a glance {#ex-glance}

All examples live in `usb3_core/examples.py` and every board target builds them with
`--example <example>`. The device answers with the vendor and product IDs `1d50:605a`.

| Example | Device | Host side |
|---------------------|---------------------------------------|----------------------------------------|
| `loopback` | Vendor Bulk OUT looped back to Bulk IN | `usb3_bulk.py loopback` |
| `wishbone` | `loopback` plus a SoC bus bridge on EP0 | `litex_server --usb` |
| `bench` | Bulk IN counter source, Bulk OUT checker | `usb3_bench.py` |
| `cdc_uartbone` | CDC-ACM serial port carrying the SoC bus | `litex_server --uart` |
| `console` | CDC-ACM LiteX BIOS console | `litex_term /dev/ttyACM0` |
| `uvc` | UVC camera, test pattern, Bulk | `ffplay`, VLC |
| `uvc_iso` | UVC camera, test pattern, Isochronous | `ffplay`, VLC |
| `msc` | Mass storage, 512 KiB FAT RAM disk | `/dev/sdX`, mount |
| `msc_dram` | Mass storage, 512 MiB DDR3 disk | `mkfs.vfat`, mount |
| `composite` | UVC camera plus CDC-ACM console | both at once |

The commands of this chapter use the Acorn target; the other targets take the same options. The
results were measured on the Acorn with a Linux xHCI host, except where a section says otherwise.

```sh
./bench/acorn.py --build --load --with-jtagbone --example <example>
```

The `console` and `composite` examples need a CPU (`--cpu-type vexriscv`). Only the Acorn target
has a DRAM controller, so `msc_dram` is built on the Acorn only. The XCKU3P has no UART: its
debug access is JTAGBone only.

## Common build options {#ex-options}

The options below shape the examples on every board target. [Build options](#build-options)
lists all the options of the targets with their defaults, including the debug ones.

| Option | Effect |
|-----------------------------|-------------------------------------------------------------|
| `--example <example>` | Device design, `loopback` by default |
| `--cpu-type vexriscv` | Add a CPU (console, composite, DRAM initialization) |
| `--max-burst N` | Bulk max burst of the vendor examples: 2, 4, 8 (default) or 16 |
| `--link-words N` | Link layer width: 1 (32-bit, default), 2 (64-bit) or 4 (128-bit) |
| `--with-lpm` | Accept U1 and U2 (vendor Bulk examples) |
| `--lean` | No PHY debug, throughput profile or traces, for timing margin |
| `--with-jtagbone` | SoC bus bridge over JTAG (UART bridge by default) |

With `--link-words 2` or `4`, the Bulk endpoints of the vendor, CDC-ACM and Mass Storage
examples are as wide as the link layer. The `uvc`, `uvc_iso` and `composite` examples need the
32-bit link layer.

## `loopback`: Bulk OUT to Bulk IN {#ex-loopback}

The device has a vendor class interface with a Bulk IN endpoint (EP1) and a Bulk OUT endpoint
(EP2). A FIFO sends every packet received on EP2 back on EP1. It is the first example to run on a
new board: it checks the link, the enumeration and both directions with little logic.

```console
$ ./bench/usb3_bulk.py loopback --size 1024 --count 1000
1000 packets, 1024000 bytes, 0 errors
$ ./bench/usb3_bulk.py loopback --size 1024 --count 2000 --random-sizes
```

The vendor examples need the udev rule of [Getting started](#gs-host) for user access.

## `wishbone`: SoC bus over EP0 {#ex-wishbone}

This example adds a bridge from the control endpoint to the SoC bus to `loopback`. The host reads
and writes the SoC registers with vendor requests, using the USB support of the LiteX tools,
alongside the Bulk traffic.

```sh
litex_server --usb --usb-vid 0x1d50 --usb-pid 0x605a
litex_cli --csr-csv build/sqrl_acorn/csr.csv --regs
```

The bridge is `USB3EP0Wishbone`. It needs no extra cable: the USB 3 link itself carries the debug
access.

## `bench`: throughput {#ex-bench}

The device generates a counter on Bulk IN and checks the counter it receives on Bulk OUT. The host
script queues asynchronous libusb transfers to keep the link busy, checks the IN data and reads the
OUT checker through the SoC bus.

```sh
# Device to host, data checked by the host.
./bench/usb3_bench.py in
# Host to device, device checker read through litex_server.
./bench/usb3_bench.py out --port 1240 --csr-csv build/sqrl_acorn/csr.csv
# Full duplex.
./bench/usb3_bench.py both --port 1240 --csr-csv build/sqrl_acorn/csr.csv
```

On the Acorn this gives 446 MB/s IN, 449 MB/s OUT with 0 checker errors, and 376 MB/s IN plus
400 MB/s OUT in full duplex. The script options are `--duration`, `--transfer-size` (1 MiB by
default), `--queue` (transfers queued per direction) and `--no-check`.

**Gen 2 on the XCKU3P.** With `--gen2-capable --link-words 2` on a USB 3.2 Gen 2 port:

```sh
./bench/alibaba_xcku3p.py --build --load --example bench \
    --gen2-capable --link-words 2
./bench/usb3_bench.py in --transfer-size 65536
./bench/usb3_bench.py both --transfer-size 65536 --port 1240 \
    --csr-csv build/alibaba_xcku3p/csr.csv
```

This gives 744 MB/s IN, 881 MB/s OUT, and 619 plus 736 MB/s in full duplex; `--max-burst 16`
raises Bulk IN to 785 MB/s. Use 64 KiB transfers to measure the device: with 1 MiB transfers the
host itself limits Bulk IN to about 330 MB/s at Gen 2.

The SoC of the `bench` example also has a throughput profile: cycle counters of the link layer
transmit path and of the Bulk IN endpoint states. [Throughput profile](#debug-profile) explains
how to read it.

## `--with-lpm`: U1 and U2 {#ex-lpm}

By default the device rejects the host requests to enter the U1 and U2 low power states, and hosts
then stop asking. With `--with-lpm`, the vendor Bulk examples advertise U1 and U2 exit latencies
and the device accepts the host requests when idle.

```sh
./bench/acorn.py --build --load --with-jtagbone --example loopback --with-lpm
```

This mode is validated in simulation (`./sim.py --host-lpm`) and not yet on hardware.

## `cdc_uartbone`: SoC bus over a serial port {#ex-cdc-uartbone}

The device is a CDC-ACM serial port, which Linux supports without a driver. The port carries the
UARTBone protocol of LiteX, so the LiteX tools reach the SoC bus through it:

```sh
litex_server --uart --uart-port /dev/ttyACM0
litex_cli --csr-csv build/sqrl_acorn/csr.csv --regs
```

This gives about 6000 register reads per second.

## `console`: LiteX BIOS console {#ex-console}

The SoC gets a VexRiscv CPU with its console on a USB 3 CDC-ACM serial port. The CDC-ACM class
(`USB3CDCACM`) acts as a LiteX UART PHY, so the BIOS and any firmware use it as a normal UART.

```console
$ ./bench/acorn.py --build --load --example console --cpu-type vexriscv
$ litex_term /dev/ttyACM0
litex> mem_list
Available memory regions:
ROM    0x00000000 0x0000ffff 0x10000
SRAM   0x10000000 0x10001fff 0x2000
CSR    0xf0000000 0xf000ffff 0x10000
```

## `uvc`: USB Video Class camera {#ex-uvc}

The device is a UVC 1.1 camera streaming a YUY2 test pattern (colour bars, a moving marker and a
moving luma band) over a Bulk endpoint. The host selects the mode among 640x480 at 60 fps,
1280x720 at 60 fps and 1920x1080 at 30 or 60 fps.

```sh
v4l2-ctl --list-formats-ext
ffplay -f v4l2 -input_format yuyv422 -video_size 1920x1080 -framerate 60 /dev/video0
vlc v4l2:///dev/video0:width=1280:height=720
v4l2-ctl --set-fmt-video=width=1920,height=1080 --set-parm=60 \
    --stream-mmap --stream-count=300
```

The last command counts the frames: it reports 60.00 fps at 1920x1080.

## `uvc_iso`: Isochronous camera {#ex-uvc-iso}

This is the `uvc` camera with Isochronous streaming, the usual mode of UVC cameras. The video
interface has two alternate settings: setting 0 has no endpoint, setting 1 has an Isochronous IN
endpoint (EP1, max burst 16, one burst per 125 µs service interval). That is 16 KiB per interval,
about 131 MB/s. The host driver selects setting 1 to start the stream and setting 0 to stop it.

The modes that fit this bandwidth are 640x480 at 60 fps, 1280x720 at 30 or 60 fps and 1920x1080
at 30 fps. On the Acorn, all of them stream complete frames at 60.04 or 30.01 fps.

```sh
lsusb -v -d 1d50:605a | grep -A4 "bAlternateSetting\|Isochronous"
ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720 -framerate 60 /dev/video0
v4l2-ctl --set-fmt-video=width=1920,height=1080 --set-parm=30 \
    --stream-mmap --stream-count=300
# Bandwidth or alternate setting errors of the host driver.
sudo dmesg | grep -i uvc
```

## `msc`: mass storage RAM disk {#ex-msc}

The device is a USB mass storage disk of 512 KiB in block RAM, formatted as FAT at build time with
a `README.TXT` file. Linux sees it as a SCSI disk:

```console
usb-storage 6-1:1.0: USB Mass Storage device detected
scsi 2:0:0:0: Direct-Access     EnjoyDig USB3-PIPE RAMDis 1.00 PQ: 0 ANSI: 5
sd 2:0:0:0: [sda] 1024 512-byte logical blocks: (524 kB/512 KiB)
```

Mount it with `udisksctl mount -b /dev/sdX` or the desktop. Reads reach about 186 MB/s and writes
about 227 MB/s.

## `msc_dram`: mass storage DRAM disk {#ex-msc-dram}

The disk is 512 MiB in the upper half of the 1 GiB DDR3 of the Acorn, accessed through LiteDRAM
ports (`USB3MSCDRAM`). The lower half is left to the CPU. This example is not yet validated on
hardware.

```sh
# Enables the DDR3 and a VexRiscv CPU.
./bench/acorn.py --build --load --example msc_dram
```

The LiteX BIOS initializes and calibrates the DDR3 at power-up. The disk reports "not ready" until
the DRAM controller is back under hardware control without an initialization error; the
`ddrctrl_init_done` and `ddrctrl_init_error` registers show the result. The BIOS console goes to a
stub UART, since a real UART with no reader would block the initialization.

The DRAM content is lost at each power-up, so the disk is not formatted. Format it from the host
before mounting it:

```sh
sudo mkfs.vfat /dev/sdX
udisksctl mount -b /dev/sdX
```

The build meets timing at 125 MHz on the XC7A200T with the DDR3 at 500 MT/s, and uses about
11.7k LUTs, 8.5k flip-flops and 32 block RAM tiles.

## `composite`: camera and console {#ex-composite}

One USB 3 device carries a UVC camera (interfaces 0 and 1, EP1) and a CDC-ACM BIOS console
(interfaces 2 and 3, EP2 to EP4). It is built from the function classes `USB3UVCFunction` and
`USB3CDCACMFunction`, the pieces [Device classes](#classes) describes. Stream the camera with
ffplay or VLC while using `litex_term /dev/ttyACM0`.

```sh
./bench/acorn.py --build --load --example composite --cpu-type vexriscv
```

## Host tools {#ex-tools}

The host scripts of `bench/` drive the examples and the debug features. The scripts that read
registers connect to a `litex_server` (`--port`, `--csr-csv`); [Debugging](#debugging) details the
debug tools.

| Script | Purpose |
|-------------------------|------------------------------------------------------------------|
| `usb3_bulk.py` | Bulk loopback test and device information (`loopback` example) |
| `usb3_bench.py` | Bulk throughput, IN, OUT or both, with data checks (`bench` example) |
| `usb3_soak.py` | Long stability run: link counters, enumeration, kernel messages, transfers |
| `usb3_suspend.py` | Suspend and resume test from U3, with the LTSSM history of each exit |
| `usb3_trace.py` | Dump the header packet trace of the device core |
| `usb3_debug.py` | PHY status, LTSSM history, retrain, transceiver controls |
| `usb3_analyzer.py` | LiteScope captures of the PHY and link, and their analysis |
| `usb3_resources.py` | Resources and timing of each block, out-of-context synthesis |
| `acorn_run.py` | Load the Acorn and record N link trainings with the host view |

Where to go next: [Integration](#integration) to build your own device from these examples,
[Debugging](#debugging) to use the debug tools.
