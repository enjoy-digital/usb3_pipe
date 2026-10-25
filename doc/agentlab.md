# AgentLab: USB3 Hardware Instrument for Agents (work in progress)

One USB3 cable giving a software agent "hands and eyes" on a Device Under Test (DUT): DUT consoles,
power/GPIO control, a streaming logic analyzer, JTAG/SWD flashing and a rendered view of the bench,
all through **standard USB classes** (no custom kernel driver) and a small host API (Python library,
CLI and MCP server). Every event shares one FPGA timebase.

Built on the USB3 Device Core (composite device) and LiteX (VexRiscv, LiteDRAM, CSRs).

## Targets

| Target | Transceiver | DUT IOs | Status |
|---|---|---|---|
| `agentlab/targets/kc705.py` | GTX (SFP, SFP2USB) | FMC LPC LA[00:33] P/N (68 single-ended, VADJ) | Main target (hardware validation pending) |
| `agentlab/targets/acorn.py` | GTP (SFP, SFP2USB) | 4 debug pins (loopback tests) | Development/validation target |

The gateware takes its DUT pins from the target (`dut_pads`: uarts, gpios, power, la, jtag/swd), so
the same instrument runs on both boards.

## USB Composite Device

Device class 0xEF/02/01 (IADs), one configuration:

| Interface(s) | Class | Endpoints | Function |
|---|---|---|---|
| 0-1 | CDC-ACM | EP1 IN notif, EP2 IN/OUT data | **Control**: UARTBone (LiteX Wishbone bridge, `litex_server --uart`), access to every CSR |
| 2-9 | CDC-ACM ×4 | EP3-EP6 IN notif, EP7-EP10 IN/OUT data | **DUT UARTs** 0-3 (`/dev/ttyACMx`), baudrate from SET_LINE_CODING |
| 10 | Vendor | EP11 IN bulk | **Events**: timestamped records (UART bytes, GPIO edges, power, triggers) |
| 11 | Vendor | EP12 IN bulk | **Logic analyzer** stream (samples from DDR3 / live) |
| 12 | Vendor (CMSIS-DAP v2) | EP13 IN/OUT bulk | **JTAG/SWD** probe (OpenOCD/pyOCD/probe-rs, no driver) |
| 13-14 | UVC | EP14 IN | **Eyes**: rendered view (framebuffer in DDR3) |

Endpoint numbers are indicative (max 15 IN/15 OUT): the gateware descriptors are the reference.

## Timebase and Events Stream

- `timestamp`: 64-bit free-running counter at `sys_clk_freq` (125MHz, 8ns), CSR readable, latched on
  every event.
- Event record (little-endian, 16 bytes):

| Bytes | Field |
|---|---|
| 0 | type: 0x01 UART RX byte, 0x02 UART TX byte, 0x03 GPIO edge, 0x04 power, 0x05 LA trigger, 0x06 marker (CSR write), 0xff overflow |
| 1 | source: UART index, GPIO index, ... |
| 2-7 | value (UART byte, GPIO new level, power state, ...) |
| 8-15 | timestamp (64-bit) |

## CSRs (names are the host API contract)

- `agentlab_timestamp` (64-bit, latch/read), `agentlab_marker` (write: marker event).
- `agentlab_power_out` (bit per power switch), `agentlab_power_status` (optional sense).
- `agentlab_gpio_oe`, `agentlab_gpio_out`, `agentlab_gpio_in`, `agentlab_gpio_edge_mask`.
- `agentlab_uart{n}_*`: DUT UART config (baudrate also from the CDC-ACM line coding).
- `agentlab_la_*`: channels, samplerate divider, trigger (mask/value/edge), pre-trigger depth,
  capture length, arm/status (phase 2).
- `agentlab_video_*`: framebuffer base/format (phase 4).

## Host

`agentlab/host/`: Python package `agentlab`:
- `Instrument`: finds the device (VID/PID, interfaces), CSR access through UARTBone
  (`litex.RemoteClient` via `litex_server --uart`, or a direct serial Wishbone client), DUT UARTs
  (pyserial), events (pyusb), logic analyzer captures (pyusb → VCD / sigrok srzip), power/GPIO,
  JTAG/SWD (OpenOCD with cmsis-dap), screenshots (UVC frame grab).
- `agentlab` CLI and an MCP server (`agentlab-mcp`) exposing tools: `uart_read`, `uart_write`,
  `uart_wait_for`, `power_set`, `power_cycle`, `gpio_set/get`, `capture`, `jtag_flash`,
  `screenshot`, `events`, `timestamp`.

## Phases

1. Control + DUT UARTs + GPIO/power + events, host library, CLI, MCP server.
2. Logic analyzer: streaming capture through DDR3, triggers, sigrok/VCD export.
3. JTAG/SWD: CMSIS-DAP v2 probe (VexRiscv firmware + SWD/JTAG IO core).
4. Eyes: UVC stream of a DDR3 framebuffer rendered by the firmware (LA waveforms, UART tails,
   status), for multimodal agents.
