# Getting started {#getting-started}

This chapter takes you from a fresh machine to a USB 3 device moving data with a host. It installs
the tools, runs a first simulation, then builds and loads a bitstream on a board and checks it from
the host. Each step gives the command, what you should see, and what to check when you do not.

## Prerequisites {#gs-prerequisites}

The project is a set of Python packages built on LiteX and needs Python 3.9 or newer. Install
LiteX first, following the installation page of its wiki, with the standard configuration
(`litex_setup.py --config=standard`). It installs Migen, LiteX, LiteICLink (the transceiver library
the PHY uses), LiteScope, LiteDRAM and LiteX-Boards. Then install the project in development mode:

```sh
git clone https://github.com/enjoy-digital/usb3_pipe
cd usb3_pipe
pip3 install -e .
```

The other tools depend on what you run. The table below lists them.

| Task | Tools |
|--------------------------------|---------------------------------------------------------------|
| Unit and system tests | Python 3, pytest, pytest-xdist (optional, parallel runs) |
| Full system simulation | Verilator and a C++ compiler (`sim.py`) |
| AMD 7-Series and UltraScale+ | Vivado; openFPGALoader or Vivado for JTAG loading |
| Gowin GW5AST, GW5AT | Gowin IDE, LiteICLink with GTR12 support, openFPGALoader |
| Lattice ECP5-5G | Yosys, nextpnr-ecp5, Project Trellis, LiteICLink with 40-bit `SerDesECP5` |
| Host side scripts | `pip3 install pyusb libusb1`, numpy, the LiteX tools (`litex_server`, `litex_cli`) |

The host scripts talk to the device with libusb. A Linux host with an xHCI controller is the
reference; every result of this manual was measured on one.

## Step 1: a first simulation {#gs-simulation}

The simulation connects two PHYs, a host and a device, through models of the transceivers. A
scripted host model trains the link, enumerates the device and runs Bulk transfers against the
device core. It is the fastest way to see the whole project work, and it needs no board.

```sh
./sim.py
```

**What you should see.** Verilator compiles the design into `build/sim`, then the run prints the
LTSSM state changes of both sides and the packets the host receives (`HOST_RX` lines). It ends
with `HOST script done`.

**If not.** A compile error usually means Verilator or the C++ compiler is missing. A run that
stops before `HOST script done` points to a broken installation: run the test suite below, which
checks every module on its own.

```sh
python3 -m pytest -v -n auto -m "not slow and not sim"
```

This default suite is fast and needs no Verilator. Options of `sim.py` change the scenario; the
most useful ones are below, and [Simulation](#simulation) lists the others.

| Option | Scenario |
|---------------------------------|--------------------------------------------------------|
| `--no-core --ppm 300` | PHYs only, PRBS traffic, 300 ppm clock offset |
| `--ssc` | Spread spectrum clocking on the device clock |
| `--gen2` | Gen 2 training to U0 at 10 Gbps, enumeration, Bulk transfers |
| `--gen2 --link-words 2` | Same with the 64-bit link interface |
| `--host-lpm` | U1 and U2 entries and exits after the enumeration |
| `--dev-pipe-cd` | Device link layer and core at half the PHY clock rate |
| `--trace` | VCD waveform in `build/sim/gateware/sim.vcd` |

## Step 2: build and load the Acorn {#gs-acorn}

The Acorn CLE-215+ on the LiteX Acorn Baseboard is the reference board (see [Boards](#boards)).
Plug the SFP2USB adapter into the SFP cage of the baseboard. Connect the baseboard JTAG port to
the host, and a USB 3 cable from the adapter to a SuperSpeed port of the host.

The command below builds the `loopback` example, loads it into the FPGA through JTAG and enables
the JTAG bridge to the SoC bus. Loading only writes the FPGA configuration memory: a power cycle
brings the board back to its previous bitstream.

```sh
./bench/acorn.py --build --load --with-jtagbone
```

**What you should see.** The build takes a few minutes with Vivado and writes everything to
`build/sqrl_acorn`, including `csr.csv`, the register map the host scripts use. After the load,
the first user LED (transceiver ready) lights, then the second one (U0) a fraction of a second
later. The third LED lights while the link is in Recovery.

**If not.** A build error is reported by Vivado in `build/sqrl_acorn/gateware`. A load error
usually means the JTAG cable is not found: with several FTDI adapters connected, see
[Setup](#debug-setup). No transceiver ready LED points to the reference clock; no U0 LED means
the link does not train: go to [Debugging](#debugging).

## Step 3: check the device from the host {#gs-host}

The device enumerates with the vendor and product IDs `1d50:605a`. The commands below show it on
the SuperSpeed bus and in the kernel log.

```console
$ lsusb -t
/:  Bus 06.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 5000M
    |__ Port 1: Dev 2, If 0, Class=Vendor Specific Class, Driver=, 5000M
$ sudo dmesg | tail
usb 6-1: new SuperSpeed USB device number 2 using xhci_hcd
usb 6-1: New USB device found, idVendor=1d50, idProduct=605a
```

**What you should see.** A device on a `5000M` bus at Gen 1, or `10000M` at Gen 2.

**If not.** Check that the cable goes to a SuperSpeed port: the device has no USB 2 pair and does
not appear on a USB 2 port. The kernel message "Cannot enable. Maybe the USB cable is bad?" means
the link never reached U0; [Debugging](#debugging) starts from there.

The vendor class examples need user access to the device. Install the udev rule of the
repository once, then re-plug the cable or reload the bitstream:

```sh
sudo cp bench/99-usb3-pipe.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

## Step 4: first transfers {#gs-transfers}

With the `loopback` example, the device returns on its Bulk IN endpoint every packet the host
sends on its Bulk OUT endpoint. The loopback script sends packets, reads them back and compares
them.

```console
$ ./bench/usb3_bulk.py loopback --size 1024 --count 1000
1000 packets, 1024000 bytes, 0 errors, <rate> Mbps (OUT+IN, one packet per transfer)
$ ./bench/usb3_bulk.py loopback --size 1024 --count 2000 --random-sizes
```

**What you should see.** `0 errors` on both runs. The rate printed is low by design: the script
sends one packet per transfer and waits for its echo.

**If not.** "Access denied" or "Device not found" errors come from a missing udev rule or a device
that did not enumerate (Step 3). Packet errors point to the link: see
[Bulk data and stability](#debug-bulk).

## Step 5: measure the throughput {#gs-bench}

The `bench` example measures the throughput. The device generates a counter on Bulk IN and checks
the counter the host sends on Bulk OUT. The host checks the IN data itself; the result of the
device OUT checker is read through the SoC bus. That needs a `litex_server` connected to the
board, on a port of its own, and the `csr.csv` of the build.

```sh
./bench/acorn.py --build --load --with-jtagbone --example bench
litex_server --jtag --bind-port 1240 &
litex_cli --port 1240 --csr-csv build/sqrl_acorn/csr.csv --ident
./bench/usb3_bench.py in
./bench/usb3_bench.py both --port 1240 --csr-csv build/sqrl_acorn/csr.csv
```

**What you should see.** The identifier starts with `USB3SoC`. On the Acorn, the benchmark gives
about 446 MB/s IN, 449 MB/s OUT and 376 plus 400 MB/s in full duplex, with 0 errors.

**If not.** Another identifier means another bridge answers on that port: the LiteX default port
1234 is often taken, which is why the commands use 1240. A lower throughput is analysed in
[Performance](#performance).

## Other boards {#gs-other-boards}

The other targets follow the same steps: build, load, start a `litex_server` on the bridge of
the board, then run the host scripts with the `csr.csv` of the build. The bridge and the loading
method of each target are in [Target scripts](#boards-targets).

The Alibaba XCKU3P has no UART: JTAGBone goes through OpenOCD with the configuration of the
JTAG-HS2 adapter and the XCKU3P (see [Alibaba XCKU3P](#boards-xcku3p)). The commands below build
Gen 2 with the 64-bit link interface.

```sh
./bench/alibaba_xcku3p.py --build --load --example bench \
    --gen2-capable --link-words 2
litex_server --jtag --jtag-config <openocd config> --bind-port 1240 &
./bench/usb3_bench.py both --transfer-size 65536 \
    --port 1240 --csr-csv build/alibaba_xcku3p/csr.csv
```

The host sees a 10 Gbps device only on a USB 3.2 Gen 2 port; on a Gen 1 port the link trains at
5 Gbps. Measure Gen 2 with 64 KiB transfers, as [Examples](#ex-bench) explains: with the 1 MiB
default, the host itself limits Bulk IN.

The Tang Mega 138K Pro builds with the Gowin IDE and loads with openFPGALoader, since `--load`
expects a `.bit` file. Its default bridge is UARTBone, on the serial port of the board.

```sh
./bench/sipeed_tang_mega_138k_pro.py --build --example bench
openFPGALoader -c ft2232 \
    build/sipeed_tang_mega_138k_pro/gateware/impl/pnr/project.fs
litex_server --uart --uart-port /dev/ttyUSBx --bind-port 1240 &
./bench/usb3_bench.py both --transfer-size 65536 \
    --port 1240 --csr-csv build/sipeed_tang_mega_138k_pro/csr.csv
```

The ECPIX-5 builds with Yosys and nextpnr and loads with `--load`. Use the on-board USB-C
connector: the target reads the cable orientation and sets the multiplexer itself. When timing
fails, try another `--seed`.

```sh
./bench/lambdaconcept_ecpix5.py --build --load --example bench
litex_server --uart --uart-port /dev/ttyUSBx --bind-port 1240 &
./bench/usb3_bench.py both --transfer-size 65536 \
    --port 1240 --csr-csv build/lambdaconcept_ecpix5/csr.csv
```

Where to go next: [Examples](#examples) for the other device designs, [Integration](#integration)
to add the project to your own SoC, [Debugging](#debugging) when a step fails.
