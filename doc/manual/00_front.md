```{=latex}
\frontmatter
```

# About this document {.unnumbered}

This manual is the reference of the USB3-PIPE project: a USB 3.2 PHY and a USB 3 device core
that run in the fabric of an FPGA and drive its transceivers directly, with no external USB chip.
It is written for FPGA engineers who want to reuse the design, and it teaches USB 3 along the way.

No prior knowledge of USB 3 is assumed. Part I explains how USB 3 works, from the electrical
signalling up to the transfers a host sees, and points at the module that implements each
mechanism. Parts II and III then describe the implementation, layer by layer: the PHY, then the
device core.

Part IV is the user guide: the supported FPGAs and boards, the first build, the integration in a
design, the examples, the simulation and the debugging. The appendices hold the register
reference, the build options, the glossary and the performance figures.

| If you want to | Read |
|------------------------------------------|----------------------------------------------|
| Know what the project does and does not | [Introduction](#introduction) |
| Learn how USB 3 works | [USB 3 in a nutshell](#usb3-primer) |
| Check that your FPGA or board is supported | [Supported FPGAs and boards](#boards) |
| Get a device enumerated on a host | [Getting started](#getting-started) |
| Add USB 3 to your own design | [Integration](#integration) |
| Understand a link that does not come up | [Debugging](#debugging) |

The manual follows the repository: every name, number and default is taken from the source code,
and the manual is generated from it on the date printed on the title page. When the code and the
manual disagree, the code is right and the manual has a bug.

## Conventions {.unnumbered}

- Numbers: `0x` prefix for hexadecimal, `0b` for binary.
- Rates: line rates in Gbps (5 Gbps for Gen 1, 10 Gbps for Gen 2). Throughputs in MB/s, that is
  10^6 bytes per second, as the host tools print them. Sizes in KiB and MiB.
- USB 3 names follow the USB 3.2 specification, cited as "USB 3.2 section 7.5". Link states are
  written U0 to U3; LTSSM states are written with their dotted names, such as *Polling.Active*.
- *The PHY* is the `usb3_pipe` package; *the device core* is the `usb3_core` package; *the
  project* is both. *The host* is the USB 3 host controller (xHCI) at the other end of the cable;
  *the link partner* is the port the PHY is connected to, the host or a hub; *the device* is the
  FPGA design.
- `<target>` stands for the name of a board target script of `bench/` without its extension, such
  as `acorn` or `alibaba_xcku3p`; a build lands in `build/<target>/`.
- `<example>` stands for the name of an example design, such as `loopback`, `bench` or `uvc`.
- Paths are relative to the repository root. Shell commands are given for a Linux host.
- When the text names a source file, it is the file to open to check or extend the fact.
