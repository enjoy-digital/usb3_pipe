# Examples

Example designs (`usb3_core/examples.py`: `loopback`, `wishbone`, `bench`, `cdc_uartbone`, `console`, `uvc`,
`uvc_iso`, `msc`, `msc_dram`, `composite`) and their host tools, on `bench/acorn.py` (LiteX Acorn
Baseboard, Acorn CLE-215+, SFP2USB). Build and load (JTAG, SRAM) with:

```sh
$ ./bench/acorn.py --build --load --with-jtagbone --example <name> [--cpu-type vexriscv] [--with-lpm]
```

The results below were measured on a Linux host (xHCI controller). The device is `1d50:605a` (see
`bench/99-usb3-pipe.rules` for user access to the vendor class examples).

## `loopback`: Bulk OUT -> Bulk IN

Vendor class interface, Bulk OUT (EP2) data looped back to Bulk IN (EP1) through a FIFO.

```sh
$ ./bench/usb3_bulk.py loopback --size 1024 --count 1000
1000 packets, 1024000 bytes, 0 errors
$ ./bench/usb3_bulk.py loopback --size 1024 --count 2000 --random-sizes
```

## `wishbone`: Bulk loopback + EP0 Wishbone bridge

`loopback` with a Wishbone bridge on the Control Endpoint (`USB3EP0Wishbone`, vendor requests): SoC
bus/CSRs access over USB with LiteX CommUSB (pyusb), alongside the Bulk endpoints.

```sh
$ litex_server --usb --usb-vid 0x1d50 --usb-pid 0x605a
$ litex_cli --regs
```

## `bench`: bandwidth

Bulk IN (EP1) counter source, Bulk OUT (EP2) counter checker (CSRs), asynchronous libusb transfers:

```sh
$ ./bench/usb3_bench.py in --no-check                      # IN  446 MB/s
$ ./bench/usb3_bench.py out --csr-csv build/sqrl_acorn/csr.csv  # OUT 449 MB/s, device checker 0 errors
$ ./bench/usb3_bench.py both --csr-csv build/sqrl_acorn/csr.csv # IN 376 + OUT 400 MB/s
$ ./bench/usb3_soak.py --bench 20 --duration 300 --csr-csv build/sqrl_acorn/csr.csv
```

`usb3_bench.py` options: `--duration`, `--transfer-size` (bytes, default 1MB), `--queue` (queued
transfers per direction), `--no-check`. With 1MB transfers the host itself limits the Bulk IN rate at
Gen 2 (~330MB/s): use `--transfer-size 65536` to measure the device.

Build options of the vendor Bulk examples (`loopback`, `bench`):
- `--max-burst 2|4|8|16`: Bulk endpoints Max Burst (default 8).
- `--link-words 1|2`: Link Layer words, 2: 64-bit Link Layer and Bulk endpoints/`USB3Bench`
  (`USB3PHY(link_words=2)`, Gen 2 bandwidth).

With the `bench` example, the SoC also has a throughput profile (`usb3_profile_control`: clear/run,
`usb3_profile`: cycles counters of `USB3_PROFILE_EVENTS` in `bench/usb3_soc.py`: Link Layer TX
words/back-pressure/idle, Tx Header Buffers full, credit waits, Bulk IN states (no data, no host
window, flow control), Tx Header Buffers hold, Header Packet -> LGOOD_n round trip, Bulk IN DPH ->
ACK TP latency).

## Gen 2 (10Gbps): `bench` on the XCKU3P

`bench/alibaba_xcku3p.py --gen2-capable` (Gen 2 capable GTY clocking: QPLL at 10Gbps) advertises Gen
2x1 in the SuperSpeedPlus Polling: on a USB 3.2 Gen 2 host port the link trains at 10Gbps
(SuperSpeedPlus, `lsusb -t`: 10000M), on a Gen 1 port at 5Gbps.

```sh
$ ./bench/alibaba_xcku3p.py --build --load --example bench --gen2-capable --link-words 2
$ lsusb -t
$ ./bench/usb3_bench.py in   --transfer-size 65536                                         # IN  744 MB/s
$ ./bench/usb3_bench.py out  --transfer-size 65536 --csr-csv build/alibaba_xcku3p/csr.csv  # OUT 881 MB/s
$ ./bench/usb3_bench.py both --transfer-size 65536 --csr-csv build/alibaba_xcku3p/csr.csv  # IN 619 + OUT 736 MB/s
```

32-bit Link Layer (default `--link-words 1`): IN 426 MB/s, OUT 466 MB/s, full duplex 325 + 430 MB/s.
`--max-burst 16` with `--link-words 2`: IN 785 MB/s, full duplex 644 + 757 MB/s. The Gen 2 debug CSRs
(`usb3_gen2_*`) are added with `--gen2-capable`: Gen 2 status (`usb3_gen2_status`: negotiated, 10Gbps
rate), capability advertised (`usb3_gen2_capable`, 0: Gen 1 operation at the next link training, ex:
after a `usb3_debug` PHY reset), RX/TX snapshots and counters ([debugging-guide.md](debugging-guide.md)).

## `--with-lpm`: U1/U2 Link Power Management

With `--with-lpm`, the vendor Bulk examples (`loopback`, `bench`) are built with
`USB3DeviceCore(..., with_lpm=True)`: the BOS advertises non-zero U1/U2 exit latencies and the device
accepts the host U1/U2 entries when idle (see [usb3-core.md](usb3-core.md)). Off by default (U1/U2
rejected, hosts disable LPM); validated in simulation (`./sim.py --host-lpm`), not on hardware yet.

```sh
$ ./bench/acorn.py --build --load --with-jtagbone --example loopback --with-lpm
```

## `cdc_uartbone`: SoC bus over USB3

CDC-ACM serial port (no driver needed) carrying the UARTBone protocol:

```sh
$ litex_server --uart --uart-port /dev/ttyACM0
$ litex_cli --regs        # ~6000 CSR reads/s
```

## `console`: LiteX BIOS console (`--cpu-type vexriscv`)

VexRiscv SoC with its console on a USB3 CDC-ACM serial port (`USB3CDCACM`, a LiteX UART PHY, as
LiteX's `LunaCDCACM` over USB2):

```sh
$ litex_term /dev/ttyACM0
litex> help
LiteX BIOS, available commands:
...
litex> mem_list
Available memory regions:
ROM    0x00000000 0x0000ffff 0x10000
SRAM   0x10000000 0x10001fff 0x2000
CSR    0xf0000000 0xf000ffff 0x10000
```

## `uvc`: USB Video Class camera

UVC 1.1 camera streaming a YUY2 test pattern (color bars, moving marker, moving luma band) over Bulk,
modes selected by the host:

```sh
$ v4l2-ctl --list-formats-ext
	[0]: 'YUYV' (YUYV 4:2:2)
		Size: Discrete 640x480    Interval: 0.017s (60.000 fps)
		Size: Discrete 1280x720   Interval: 0.017s (60.000 fps)
		Size: Discrete 1920x1080  Interval: 0.033s (30.000 fps), 0.017s (60.000 fps)
$ ffplay -f v4l2 -input_format yuyv422 -video_size 1920x1080 -framerate 60 /dev/video0
$ vlc v4l2:///dev/video0:width=1280:height=720
$ v4l2-ctl --set-fmt-video=width=1920,height=1080 --set-parm=60 --stream-mmap --stream-count=300   # 60.00 fps
```

## `uvc_iso`: USB Video Class camera, Isochronous

Same camera as `uvc` with Isochronous streaming (the usual UVC camera mode): VideoStreaming interface
with alternate setting 0 (no endpoint) and 1 (Isochronous IN EP1, MaxBurst 16, Mult 0, bInterval 1:
16KB per 125us service interval, ~131MB/s), one UVC payload per service interval
(dwMaxPayloadTransferSize = wBytesPerInterval = 16384), zero-length packets between frames. uvcvideo
selects alternate setting 1 to start the stream, alternate setting 0 to stop it. Modes fitting the
bandwidth: 640x480@60, 1280x720@30/60, 1920x1080@30.

```sh
$ lsusb -v -d 1d50:605a | grep -A4 "bAlternateSetting\|Isochronous"
$ v4l2-ctl --list-formats-ext
$ ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720 -framerate 60 /dev/video0
$ v4l2-ctl --set-fmt-video=width=1920,height=1080 --set-parm=30 --stream-mmap --stream-count=300
$ sudo dmesg | grep -i uvc   # Bandwidth/alternate setting errors.
```

On the Acorn (Linux xHCI host): 640x480@60 and 1280x720@60 at 60.04 fps, 1280x720@30 and
1920x1080@30 at 30.01 fps, complete frames.

## `msc`: Mass Storage RAM disk

512KB block RAM disk, pre-formatted (FAT) with a `README.TXT` at build time:

```
usb-storage 6-1:1.0: USB Mass Storage device detected
scsi 2:0:0:0: Direct-Access     EnjoyDig USB3-PIPE RAMDis 1.00 PQ: 0 ANSI: 5
sd 2:0:0:0: [sda] 1024 512-byte logical blocks: (524 kB/512 KiB)
```

Mount it (`udisksctl mount -b /dev/sdX` or the desktop), READ(10) ~186 MB/s, WRITE(10) ~227 MB/s.

## `msc_dram`: Mass Storage DRAM disk (not yet validated on hardware)

512MB disk in the Acorn DDR3 (upper half of the 1GB DRAM, LiteDRAM DMAs: `USB3MSCDRAM`):

```sh
$ ./bench/acorn.py --build --load --example msc_dram   # Enables the DDR3 and a VexRiscv CPU.
```

Build (Vivado 2024.1, xc7a200t-3, sys 125MHz, DDR3 500MT/s): timing met (sys WNS +0.36ns), 11.7K LUTs,
8.5K FFs, 32 BRAM tiles.

The DDR3 is initialized/calibrated by the LiteX BIOS at power-up: the disk reports NOT READY until
the DRAM controller is back under hardware control after the BIOS initialization (and no
`ddrctrl_init_error`). The BIOS console goes to a stub UART (a crossover UART would block the BIOS,
and the DRAM initialization, until a host reads it), check the initialization with the
`ddrctrl_init_done`/`ddrctrl_init_error` CSRs (UARTBone/JTAGBone, `litex_cli --regs`).

The disk is not formatted (DRAM content is lost at each power-up/reload): format it from the host
before mounting it:

```sh
$ sudo mkfs.vfat /dev/sdX   # Whole disk, or partition it first (fdisk).
$ udisksctl mount -b /dev/sdX
```

## `composite`: camera + console (`--cpu-type vexriscv`)

One USB3 device with a UVC camera (interfaces 0-1, EP1) and a CDC-ACM BIOS console (interfaces 2-3,
EP2-EP4), built from the `USB3UVCFunction`/`USB3CDCACMFunction` functions: stream the camera with
ffplay/VLC while using `litex_term /dev/ttyACM0`.

## Other targets

`bench/kc705.py` (KC705, Kintex7 GTX, `--connector sfp|sma`) and `bench/alibaba_xcku3p.py` (Alibaba
Cloud XCKU3P, Kintex Ultrascale+ GTY, `--sfp 0|1`, `--refclk sfp|clk100`) share the SoC
(`bench/usb3_soc.py`: example designs and common options `--example`, `--cpu-type`, `--with-lpm`,
`--max-burst`, `--link-words`, `--with-serdes-debug`, `--with-analyzer`). Results above are from the
Acorn, except the Gen 2 section (XCKU3P).

- XCKU3P: validated on hardware with the SFP2USB adapter in the NIC1 SFP cage (`--sfp 0`, default),
  GTY reference clock from the SFP MGT clock (156.25MHz, `--refclk sfp`, default): bench example IN
  446MB/s, OUT 450MB/s, full duplex, 0 errors, retrains/soak without Recovery (RX polarity inverted
  on this board, detected by the LTSSM). Gen 2 (10Gbps) with `--gen2-capable`, see above. JTAG:
  Digilent JTAG-HS2 (`openFPGALoader -c digilent_hs2`,
  OpenOCD `cpld/xilinx-xcu.cfg` with `CHIP XCKU3P` for JTAGBone).
- KC705: build-tested, hardware validation in progress.

`bench/acorn.py --lanes 2 --role host|device` (Acorn Baseboard Mini, SFP-0/SFP-1) is not a USB3
Device Core example: Gen 1x2 PHY link test between two boards with PRBS Link-Layer stand-ins
(`bench/usb3_lanes.py`), see [gen2-x2-roadmap.md](gen2-x2-roadmap.md#gen-1x2-hardware-milestones).

| Target | Build directory | Not available |
|---|---|---|
| `bench/acorn.py` | `build/sqrl_acorn` | |
| `bench/kc705.py` | `build/xilinx_kc705` | `msc_dram` (no DRAM controller) |
| `bench/alibaba_xcku3p.py` | `build/alibaba_xcku3p` | `msc_dram` (no DRAM controller), UARTBone (JTAGBone only) |
