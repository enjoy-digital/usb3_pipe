# Standalone Core Generator (`usb3_pipe_gen`)

The USB3 PHY, optionally with the USB3 Device Core, can be generated as a standalone Verilog core to be
integrated in designs not built with LiteX (Vivado projects, other HDLs).

```sh
$ usb3_pipe_gen examples/a7_device.yml     --output-dir build/usb3_pipe_core # PHY only.
$ usb3_pipe_gen examples/a7_usb_device.yml --output-dir build/usb3_device    # PHY + Device Core.
$ usb3_pipe_gen examples/k7_device.yml     --output-dir build/usb3_k7        # Kintex7 (GTX).
$ usb3_pipe_gen examples/usp_device.yml    --output-dir build/usb3_usp       # Ultrascale+ (GTY), + Device Core.
```

`--name` sets the core/module name (default: `usb3_pipe_core`). Ultrascale+ (`USPUSB3SerDes`: GTY
channel PLL, OOB clock from a `USPMMCM` instead of a `S7PLL`, `XilinxUSPPlatform`) is generated the
same way (`examples/usp_device.yml`: xcku3p as the `alibaba_xcku3p.py` target, but with a dedicated
156.25MHz transceiver reference clock).

## Configuration (YAML)

| Parameter | Default | Description |
|---|---|---|
| `phy` | (required) | Transceiver: `A7USB3SerDes` (Artix7 GTP), `K7USB3SerDes` (Kintex7 GTX), `USPUSB3SerDes` (Ultrascale+ GTY). |
| `device` | PHY family | FPGA device (default: `xc7a`/`xc7k`/`xcku3p`). |
| `speedgrade` | `-1` | FPGA speedgrade (OOB clock PLL/MMCM). |
| `sys_clk_freq` | `125e6` | Core/user clock frequency (>= 125MHz). |
| `refclk`/`refclk_freq` | `pads`/`125e6` | Transceiver reference clock: `pads` (dedicated `refclk_p/n` ports) or `sys` (core clock). |
| `role` | `device` | `device` (Upstream Port) or `host` (Downstream Port). |
| `with_endianness_swap` | `True` | First symbol on `tdata[31:24]` (else `tdata[7:0]`). |
| `with_link_ctrl` | `False` | Expose the LTSSM Link-Layer control interface (PM handshake, directed transitions). |
| `timer_scale` | `1` | LTSSM/LFPS timers scaling (simulation only, `1` for hardware). |
| `usb_core` | `False` | Include the USB3 Device Core (device role): Bulk IN/OUT AXI-Stream endpoints. |
| `usb_vid`/`usb_pid`/`usb_manufacturer`/`usb_product`/`usb_serial` | | Device descriptors (`usb_core`). |
| `usb_max_burst` | `8` | Bulk endpoints MaxBurst (2/4/8/16, `usb_core`). |

## Interfaces

- **PHY only**: `sink_*`/`source_*` AXI-Stream symbol streams (32-bit data, 4-bit K-symbol flags in
  `tuser`), LTSSM status ports (`u0`, `ltssm_state`...), optional Link-Layer control ports.
- **PHY + Device Core** (`usb_core`): `bulk_in_*` (EP1 IN, device -> host) and `bulk_out_*` (EP2 OUT,
  host -> device) AXI-Stream (32-bit data, first byte on `tdata[7:0]`, `tkeep` on the last word of a
  packet, `tlast`: end of packet), `usb_ready`, `usb_configured`. Host side: any libusb application.

## Outputs

| File | Description |
|---|---|
| `gateware/<name>.v` | Verilog of the core (top module `<name>`). |
| `gateware/<name>.xdc` | Timing constraints of the core internal clocks/CDCs (to be scoped to the core). |
| `gateware/*.init` | Memories initialization files. |
| `<name>_inst.v` | Instantiation template. |
| `<name>_ltssm.vh` | LTSSM states encoding of the `ltssm_state` port (Verilog `localparam`s). |
| `<name>.md` | Documentation of the core for the selected configuration (ports, clocking, data format, link bring-up, integration notes). |

The user project constrains `clk` (and `refclk_p/n`) and provides the transceiver/pin placement.
