# Performance and resources {#performance}

This appendix gathers the measured throughputs and the FPGA resources of the project. Use it to
size a design and to know what to expect on a board before building. All throughputs are
measured with a Linux xHCI host and the standard tools of the repository.

## Test conditions {#perf-conditions}

| Item | Value |
|------------------------|------------------------------------------------------------------|
| Host | Linux xHCI host controller, motherboard USB 3.2 root port |
| Connection | SFP2USB adapter in an SFP cage, or a direct USB-C cable (ECPIX-5, GW5AT-15) |
| Device design | `bench` example: Bulk counter source on IN, counter checker on OUT |
| Host tool | `bench/usb3_bench.py in`, `out` or `both` |
| Data check | IN and OUT data checked, unless noted |
| Clock | 125 MHz `sys` clock for the link layer and the core, unless noted |
| Throughput unit | MB/s, 10^6 bytes per second, as `usb3_bench.py` prints it |

`usb3_bench.py` queues 1 MiB transfers by default. At Gen 2 the host itself then limits Bulk IN to
about 330 MB/s, so the Gen 2 numbers are measured with `--transfer-size 65536`.

## Gen 1 (5 Gbps) {#perf-gen1}

A Gen 1 link carries 500 MB/s of raw data after 8b/10b coding. Framing, headers and link commands
take their share, so about 450 MB/s is the practical ceiling of a Bulk endpoint.

| Board | Transceiver | Bulk IN | Bulk OUT | Full duplex (IN + OUT) |
|---------------------|-----------------|-----------|-----------|---------------------------|
| Acorn CLE-215+ | Artix-7 GTP | 446 MB/s | 449 MB/s | 376 + 400 MB/s |
| Alibaba XCKU3P | Kintex UltraScale+ GTY | 446 MB/s | 450 MB/s | |
| ECPIX-5 | ECP5-5G DCU | 434 MB/s | 449 MB/s | 342 + 349 MB/s |
| Tang Mega 138K Pro | GW5AST GTR12 | 372 MB/s | 433 MB/s | 262 + 383 MB/s |
| GW5AT-15 | GW5AT GTR12 | about 400 MB/s | | |

The Acorn Bulk IN number is measured with `--no-check`, without the host data check. On the Tang
Mega 138K Pro, the ECPIX-5 and the GW5AT-15, the PHY runs at 125 MHz on the transceiver clock while
the link layer and the core run on 64-bit words: at 62.5 MHz on the Gowin devices, at 83.33 MHz
on the ECPIX-5. The ECPIX-5 and GW5AT-15 numbers are measured with a max burst of 16.

| Device class (Acorn CLE-215+) | Result |
|-------------------------------------------|-------------------------------------------|
| UVC, 1920x1080 at 60 fps, YUY2, Bulk | 60.00 fps, about 249 MB/s |
| UVC, Isochronous, 640x480 and 1280x720 at 60 fps | 60.04 fps, complete frames |
| UVC, Isochronous, 1280x720 and 1920x1080 at 30 fps | 30.01 fps, complete frames |
| Mass Storage RAM disk, READ(10) and WRITE(10) | 186 and 227 MB/s, single command |
| CDC-ACM with UARTBone | about 6000 register reads per second |

## Gen 2 (10 Gbps) {#perf-gen2}

A Gen 2 link carries about 1200 MB/s after 128b/132b coding. A 32-bit link layer at 125 MHz moves
only 500 MB/s per direction, so Gen 2 needs the 64-bit link layer to go beyond that.

| Alibaba XCKU3P, Gen 2x1 | Bulk IN | Bulk OUT | Full duplex (IN + OUT) |
|-------------------------------------------|-----------|-----------|---------------------------|
| 32-bit link layer (default) | 426 MB/s | 466 MB/s | 325 + 430 MB/s |
| 64-bit link layer (`--link-words 2`) | 744 MB/s | 881 MB/s | 619 + 736 MB/s |
| 64-bit, max burst 16 (`--max-burst 16`) | 785 MB/s | 873 MB/s | 644 + 757 MB/s |

With 64-bit words, Bulk OUT reaches about 88 % of the 1000 MB/s that the link layer moves at
8 bytes per cycle and 125 MHz. Bulk IN is lower because each burst waits for a host request, an
ACK TP, which the host sends only after processing the previous packets.

The same Gen 2 capable design also runs at Gen 1, at 441 MB/s in both directions, when its Gen 2
capability is cleared.

## Stability {#perf-stability}

| Board | Run | Result |
|-----------------------|----------------------------------------|------------------------------------|
| Acorn CLE-215+ | 3 min full duplex, data checked | 139 GB, 0 errors, no LBAD, retry or Recovery |
| Acorn CLE-215+ | Linux autosuspend (U3) and resume | 72 of 72 clean resumes, about 0.6 s each |
| Alibaba XCKU3P, Gen 2 | 120 s full duplex, 64-bit link layer | 156 GB, 0 errors, 0 LBAD, 0 Recovery |
| ECPIX-5 | 3 min full duplex, data checked | about 110 GB, 0 errors, 0 Recovery |
| Tang Mega 138K Pro | 20 s runs, data checked | 0 errors, 0 LBAD, 0 Recovery |

## What limits the throughput {#perf-limits}

| Limit | Effect | Lever |
|------------------------|---------------------------------------------------|-----------------------------------|
| Line coding | Gen 1: 500 MB/s raw; Gen 2: about 1200 MB/s raw | Gen 2 capable board and host port |
| Link layer width | 32-bit words at 125 MHz: 500 MB/s per direction | `link_words=2` (64-bit) |
| Max burst | Fewer packets in flight between host requests | `max_burst` up to 16 |
| Host requests | Bulk IN waits for ACK TPs from the host | Larger bursts, queued transfers |
| Host transfer size | Small or very large transfers lower the host rate | `--transfer-size`, `--queue` |
| Full duplex | Each direction also carries the ACK TPs of the other | Larger max burst |
| Fabric speed | Slow fabrics run the core at 62.5 or 83.33 MHz on 64-bit words | `pipe_cd` clock domain |

## Resources {#perf-resources}

The table below gives the size of each block of the device core alone, synthesized and placed out
of context by `bench/usb3_resources.py` on an XC7A200T-3 at 125 MHz. WNS is the worst negative
slack at that frequency; a positive value means the timing is met.

| Block | LUTs | FFs | BRAM36 | WNS |
|----------------------------------------------|------|------|--------|----------|
| Link layer | 1649 | 1034 | 0 | +1.86 ns |
| Control endpoint (Bulk descriptors) | 176 | 131 | 0 | +3.34 ns |
| Bulk IN, max burst 8 | 258 | 191 | 2 | +1.71 ns |
| Bulk OUT, max burst 8 | 171 | 165 | 2 | +2.09 ns |
| Device: link, protocol, EP0, Bulk IN and OUT | 2550 | 1682 | 4 | +0.93 ns |

A complete Acorn design with the PHY, the core, the debug logic and the `bench` example uses about
6000 LUTs, 4.5 % of the XC7A200T. The `msc_dram` example, with a DDR3 controller and a VexRiscv
CPU, uses 11.7K LUTs, 8.5K FFs and 32 BRAM tiles and meets timing at 125 MHz.

Each Bulk or Isochronous endpoint needs one 1 KiB packet slot per packet of its max burst. A max
burst of 16 therefore costs 16 KiB of block RAM per endpoint.

```sh
./bench/usb3_resources.py --sys-clk-freq 125e6
./bench/usb3_resources.py --blocks link endpoint_in
```

Where to go next: [Device core architecture](#core-architecture) for the parameters named here,
[Boards](#boards) for the board setups.
