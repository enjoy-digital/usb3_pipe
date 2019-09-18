# High Speed Transceiver PIPE Wrapper

This repository contains a library which adapts [the High Speed Transceiver (SERDES) parts](https://en.wikipedia.org/wiki/Multi-gigabit_transceiver) found in popular FPGAs to a rough approximation of [the PIPE standard](https://www.intel.com/content/dam/www/public/us/en/documents/white-papers/phy-interface-pci-express-sata-usb30-architectures-3.1.pdf).

## Targets

While we hope this wrapper will eventually support multiple protocols through the PIPE interface (such as PCIe, SATA, DisplayPort) it is currently targetting support for [USB3.0 SuperSpeed](https://en.wikipedia.org/wiki/USB_3.0#Data_encoding) when used with a customized [the Daisho USB3 core](https://github.com/enjoy-digital/daisho).

It currently targets the following FPGA parts;
 - [ ] Xilinx Kintex 7
 - [ ] Lattice ECP5-5G
 
It is hoped to eventually expand the support beyond these initial parts to;
 - [ ] Xilinx Artix 7
 - [ ] Xilinx Ultrascale/Ultrascale+ line
 - [ ] Various Xilinx Zynq line
 - [ ] Altera Cyclone parts

## Test Hardware

One of the following boards;

 - [KONDOR AX](https://www.latticesemi.com/en/Products/DevelopmentBoardsAndKits/KONDORAX)
 - [KC705](https://www.xilinx.com/products/boards-and-kits/ek-k7-kc705-g.html)

paired with 
 - [3-Port USB 3 FMC Module from HiTechGlobal](https://hitechglobal.us/index.php?route=product/product&path=18_81&product_id=233).

These boards have been previously shown to work with [the Daisho Core](https://github.com/enjoy-digital/daisho) and the [TI TUSB1310A - SuperSpeed 5 Gbps USB 3.0 Transceiver with PIPE and ULPI Interfaces](http://www.ti.com/product/TUSB1310A).

## Toolchain

This project targets;
  - [ ] Xilinx Vivado for Kintex 7 support
  - [ ] Yosys + nextpnr for ECP5 support
 
There will also be a demo showing how to use a harness to expose the PIPE interface to the SymbiFlow Yosys + VPR flow.


