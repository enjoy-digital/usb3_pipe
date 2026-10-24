# Examples

Example designs of `acorn.py` (LiteX Acorn Baseboard + Acorn CLE-215+ + SFP2USB) and their host tools.
Build and load (JTAG, SRAM) with:

```sh
$ ./acorn.py --build --load --with-jtagbone --example <name> [--cpu-type vexriscv]
```

The results below were measured on a Linux host (xHCI controller). The device is `1d50:605a` (see
`scripts/99-usb3-pipe.rules` for user access to the vendor class examples).

## `loopback`: Bulk OUT -> Bulk IN

Vendor class interface, Bulk OUT (EP2) data looped back to Bulk IN (EP1) through a FIFO.

```sh
$ ./scripts/usb3_bulk.py loopback --size 1024 --count 1000
1000 packets, 1024000 bytes, 0 errors
$ ./scripts/usb3_bulk.py loopback --size 1024 --count 2000 --random-sizes
```

## `bench`: bandwidth

Bulk IN (EP1) counter source, Bulk OUT (EP2) counter checker (CSRs), asynchronous libusb transfers:

```sh
$ ./scripts/usb3_bench.py in --no-check                      # IN  446 MB/s
$ ./scripts/usb3_bench.py out --csr-csv build/sqrl_acorn/csr.csv  # OUT 449 MB/s, device checker 0 errors
$ ./scripts/usb3_bench.py both --csr-csv build/sqrl_acorn/csr.csv # IN 376 + OUT 400 MB/s
$ ./scripts/usb3_soak.py --bench 20 --duration 300 --csr-csv build/sqrl_acorn/csr.csv
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
$ ./acorn.py --build --load --example msc_dram   # Enables the DDR3 and a VexRiscv CPU.
```

Build (Vivado 2024.1, xc7a200t-3, sys 125MHz, DDR3 500MT/s): timing met (sys WNS +0.36ns), 11.7K LUTs,
8.5K FFs, 32 BRAM tiles.

The DDR3 is initialized/calibrated by the LiteX BIOS (the disk reports NOT READY until the BIOS
sets `ddrctrl_init_done`), the BIOS console is on a crossover UART reachable through UARTBone:

```sh
$ litex_server --uart --uart-port /dev/ttyUSBX
$ litex_term crossover   # DRAM initialization/memtest logs.
```

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
