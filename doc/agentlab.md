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

Phase 1 DUT pins (details in the targets docstrings):
- KC705 (LVCMOS25, VADJ 2.5V default): UART0-3 TX/RX on LA02-LA05 P/N, GPIO0-15 on LA06-LA13 P/N,
  power 0-3 on LA14-LA15 P/N; reserved: LA16-LA31 P/N (logic analyzer channels 0-31), LA32-LA33 P/N
  + LA01_CC_P (JTAG/SWD).
- Acorn (debug pins, LVCMOS33): UART0 TX/RX on H5/J5, GPIO0/1 on J2/K2 (loop H5-J5 and J2-K2 for
  tests), UART1-3 internally looped back, power switch 0 on user_led3. CSRs through the USB control
  CDC-ACM or JTAGBone (no UARTBone on the baseboard UART).

## USB Composite Device

Device class 0xEF/02/01 (IADs), one configuration:

| Interface(s) | Class | Endpoints | Function |
|---|---|---|---|
| 0-1 | CDC-ACM | EP1 IN notif, EP2 IN/OUT data | **Control**: UARTBone (LiteX Wishbone bridge, `litex_server --uart`), access to every CSR |
| 2-9 | CDC-ACM ×4 | EP3-EP6 IN notif, EP7-EP10 IN/OUT data | **DUT UARTs** 0-3 (`/dev/ttyACMx`), baudrate from SET_LINE_CODING |
| 10 | Vendor | EP11 IN bulk | **Events**: timestamped records (UART bytes, GPIO edges, power, triggers) |
| 11 | Vendor | EP12 IN bulk | **Logic analyzer** stream (samples from DDR3 / live) |
| 12 | Vendor (CMSIS-DAP v2) | EP13 IN/OUT bulk | **JTAG/SWD** probe (OpenOCD/pyOCD/probe-rs, no driver) |
| 13-14 | UVC | EP14 IN bulk | **Eyes**: rendered view of the bench (gateware text-mode renderer) |

Endpoint numbers are indicative (max 15 IN/15 OUT): the gateware descriptors are the reference.
Interface numbers are contiguous (USB requirement): an optional function takes the next free
interfaces (Eyes: 11-12 without the logic analyzer/JTAG functions, `agentlab_eyes_interface`), its
endpoint number is fixed (Eyes: EP14 IN).

Phase 1 (`agentlab/gateware/usb.py`, `agentlab_usb_map()`): interfaces 0-10/EP1-EP11 as above
(USB3 data endpoints: Max Burst 2 for the CDC-ACM, 4 for the events), VID/PID 0x1d50/0x605a
(USB3-PIPE), product string "AgentLab". The CDC-ACM class requests (SET/GET_LINE_CODING,
SET_CONTROL_LINE_STATE) are dispatched to the function of their interface (wIndex,
`usb3_core.control.USB3ControlDispatcher`): each DUT UART has its own line coding/DTR. A DUT UART
sends its RX bytes to the host only while the port is open (DTR set).

Phase 4 (`with_eyes`, `--with-eyes`/`--no-eyes` on the targets, default on): UVC function (IAD,
VideoControl + VideoStreaming, Uncompressed YUY2 1280x720 at 10 or 30 fps, bulk EP14 IN, Max Burst
8, `usb3_core.uvc.USB3UVCFunction` with `pattern=False`: pixels from the Eyes renderer). The UVC
class requests (PROBE/COMMIT) are dispatched to the function as the CDC-ACM ones. The configuration
descriptor (611 bytes) takes 2 EP0 packets.

## Eyes (phase 4)

A UVC camera (`/dev/videoN`, VLC/OBS/ffplay, no driver) showing a view of the bench, rendered by the
gateware on the fly (`agentlab/gateware/eyes.py`, no framebuffer, no CPU, no DRAM):

- Character grid: 160x45 cells of 8x16 pixels (font: Terminus Bold 16, Latin-1 + box drawing/blocks
  on codes 0x80-0x9f, `agentlab/gateware/eyes_font.py`), cell (16-bit): char [7:0], foreground
  [11:8], background [15:12] (16 colors, ANSI order: black, red, green, yellow, blue, magenta,
  cyan, white, then the bright variants). Written by the host through the `agentlab_eyes`
  Wishbone region (csr.csv `memory_region`, one cell per 32-bit word at `4*(row*160 + col)`,
  UARTBone burst writes: a full screen is ~29KB). Boot screen until the host writes it.
- Waveform lanes drawn over the grid: one character row per lane, 2 pixels per sample, 512 samples
  window (oldest on the left), live sampling of the DUT pins (sources, bit 0 first: GPIOs, power
  switches, DUT UARTs TX/RX lines, `agentlab_eyes_wave_lanes` sources). Sampling paused while a
  frame is rendered (coherent window), continuous (roll mode) or single capture. Phase 2 can feed
  the logic analyzer samples to the same lanes (`AgentLabEyes.samples`).
- Pixels generated in raster order (2 pixels/YUY2 word per cycle) when the UVC stream starts a
  frame: ~2 BRAM36 (grid), 1 (font), 1 (samples), a few hundred LUTs.

The status panel (time, power/GPIO states, DUT UARTs tails from the events stream, last events,
lanes labels) is drawn by the host (`agentlab display status`, `Instrument.display_status()` or
`start_display_refresh()`): the gateware only renders the grid.

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

Phase 1 details (`agentlab/gateware/events.py`):

- UART RX: timestamp at the middle of the stop bit; UART TX: end of the stop bit; value: the byte.
- GPIO edge (0x03) / power (0x04): one record per changed bit (source: bit index), value byte 2:
  new level of the bit, bytes 3-7: levels of all the GPIOs/power switches (bit n: GPIO/switch n),
  same timestamp for the changes of the same cycle. GPIO edges only for the GPIOs enabled in
  `agentlab_gpio_edge_mask` (none by default).
- Marker (0x06): source 0, value: the 32-bit value written to `agentlab_marker`.
- Overflow (0xff): source 0, value: number of lost events, timestamp of the first lost event.
- Sources merged round-robin, 512 records FIFO, records sent continuously on the Bulk IN endpoint
  (records never split; packets of up to 1024 bytes ended after ~1us without record: the host reads
  a stream of 16-byte records, short packets are normal).

## CSRs (names are the host API contract)

- `agentlab_timestamp_latch` (write 1: latch), `agentlab_timestamp_value` (64-bit, latched
  timestamp), `agentlab_marker` (32-bit, write: marker event with this value).
- `agentlab_power_out` (bit per power switch), `agentlab_power_status` (optional sense, else the
  outputs).
- `agentlab_gpio_oe`, `agentlab_gpio_out`, `agentlab_gpio_in`, `agentlab_gpio_edge_mask`.
- `agentlab_uart{n}_baudrate` (rw, also written by the CDC-ACM SET_LINE_CODING, default 115200, 8N1
  only: format fields ignored), `agentlab_uart{n}_tuning_word` (ro),
  `agentlab_uart{n}_status` (`connected`: host port open, DTR).
- `agentlab_events_enable` (default 1), `agentlab_events_clear` (write 1: drop the buffered records,
  reset the lost counter), `agentlab_events_lost` (lost events count), `agentlab_events_level`
  (buffered records).
- Constants (csr.csv): `agentlab_uarts`, `agentlab_gpios`, `agentlab_power`,
  `agentlab_events_interface`, `agentlab_events_ep`.
- `agentlab_la_*`: channels, samplerate divider, trigger (mask/value/edge), pre-trigger depth,
  capture length, arm/status (phase 2).
- `agentlab_eyes_wave_row`, `agentlab_eyes_wave_lanes`, `agentlab_eyes_wave_column`,
  `agentlab_eyes_wave_first` (sample bit of the first lane), `agentlab_eyes_wave_color`: lanes
  position/count/source/color (defaults: row 29, 16 lanes, column 32, bit 0, bright green);
  `agentlab_eyes_wave_divider` (sample period: divider + 1 cycles, default 10us),
  `agentlab_eyes_wave_control` (bit 0 `run`: roll mode, default 1; bit 1 `single`: capture one
  window), `agentlab_eyes_wave_status` (`running`), `agentlab_eyes_frames` (frames rendered)
  (phase 4).
- Eyes constants: `agentlab_eyes_interface`, `agentlab_eyes_ep`, `agentlab_eyes_width`/`height`,
  `agentlab_eyes_columns`/`rows`, `agentlab_eyes_wave_depth`, `agentlab_eyes_wave_lanes`; text grid
  region `agentlab_eyes` (0x90000000).

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
4. Eyes: UVC view of the bench (gateware text-mode renderer + waveform lanes, status drawn by the
   host), for multimodal agents and humans.

## Host Usage

Install (from the repository root; the extras only add host dependencies, the gateware packages do
not depend on them):

```sh
pip install -e .[agentlab]          # Library + agentlab CLI (pyserial, pyusb).
pip install -e .[agentlab-mcp]      # + agentlab-mcp MCP server (mcp, 2.x or 1.x).
```

The device is found in sysfs (VID/PID 1d50:605a and the interface layout above, since other
USB3-PIPE designs share the VID/PID); the CDC-ACM interfaces are mapped to their `/dev/ttyACMx`.
CSR features need the gateware `csr.csv` (or `csr.json`): `--csr-csv` or `$AGENTLAB_CSR_CSV`.
CSRs are accessed with a direct UARTBone client on the control tty (no server process), or through
a running `litex_server` with `--server host:port`. The vendor events interface needs USB access
rights (udev rule for 1d50:605a). All device-specific constants (IDs, interface numbers, endpoints,
event codes, CSR names) are in `agentlab/host/constants.py`.

CLI:

```sh
export AGENTLAB_CSR_CSV=build/kc705/csr.csv
agentlab info                                  # Device, ttys, CSRs.
agentlab power cycle 0 --off-time 2            # Power switch 0 off 2s, then on.
agentlab uart wait 0 "login:" --timeout 30     # Wait for a regex on DUT UART0.
agentlab uart write 0 "root\r"
agentlab uart read 0 --timeout 2 --baudrate 115200
agentlab gpio set 3 0 && agentlab gpio get 3   # Drive/read GPIO3 (gpio release 3: input).
agentlab marker 0x42                           # Marker event, prints its device time.
agentlab events --follow --types uart_rx,gpio  # Timestamped events (--lines: UART text lines).
agentlab display status --follow               # Eyes status panel, refreshed every second.
agentlab display text "Hello\nWorld" --row 44 --fg bright_yellow
agentlab display waves --lanes 11 --period-us 10 --run
agentlab screenshot bench.png --refresh        # One UVC frame to PNG (OpenCV, ffmpeg or v4l2-ctl).
```

Eyes from any video application: `ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720
/dev/videoN` (`agentlab info`: `video`), VLC (`v4l2:///dev/videoN`), OBS (Video Capture Device).

Python:

```python
from agentlab.host import Instrument

with Instrument.open(csr_csv="csr.csv", start_events=True) as lab:
    t0 = lab.marker(1)
    lab.power_cycle(0, off_time=1.0)
    r = lab.uart(0).wait_for(r"IP address: (\S+)", timeout=30)
    print(r.groups, r.elapsed)
    for line in lab.uart_lines(index=0, since=lab.ticks_to_s(t0)):
        print("{:.6f}s {}".format(lab.ticks_to_s(line.timestamp), line.text))
    lab.start_display_refresh(period=1.0)       # Eyes status panel.
    print(lab.screenshot("bench.png"))           # {"path", "width", "height", "bytes", "device"}.
```

MCP server (stdio), e.g. for Claude Code (`.mcp.json`) or any MCP client:

```json
{
  "mcpServers": {
    "agentlab": {
      "command": "agentlab-mcp",
      "args": ["--csr-csv", "/path/to/build/kc705/csr.csv"]
    }
  }
}
```

Tools: `uart_read`, `uart_write`, `uart_wait_for`, `uart_log`, `power_set`, `power_cycle`,
`gpio_set`, `gpio_get`, `events`, `timestamp`, `marker`, `screenshot` (PNG path of the bench view,
status panel refreshed first), `display_text`. Times are device seconds (`time_s`, shared
timebase); the instrument is opened on the first tool call and the events reader runs from then.

Tests: `pytest test/test_agentlab_host*.py` (fake backends, default suite); with a device
connected: `AGENTLAB_CSR_CSV=... pytest -m hw test/test_agentlab_host_hw.py`.
