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

Phase 2: logic analyzer channels 0-31 on the KC705 LA16-LA31 P/N (LA16_P: 0, LA16_N: 1, ...); on
the Acorn, internal probes (`AgentLab.la_probes`): 0/1 UART0 TX/RX (debug pins H5/J5), 2-7 UART1-3
TX/RX (internal loopbacks), 8/9 GPIO0/1 (debug pins J2/K2), 12 power switch 0, 16-31 timestamp bits
0-15 (counter: checks the samples/timebase integrity).

## USB Composite Device

Device class 0xEF/02/01 (IADs), one configuration:

| Interface(s) | Class | Endpoints | Function |
|---|---|---|---|
| 0-1 | CDC-ACM | EP1 IN notif, EP2 IN/OUT data | **Control**: UARTBone (LiteX Wishbone bridge, `litex_server --uart`), access to every CSR |
| 2-9 | CDC-ACM ×4 | EP3-EP6 IN notif, EP7-EP10 IN/OUT data | **DUT UARTs** 0-3 (`/dev/ttyACMx`), baudrate from SET_LINE_CODING |
| 10 | Vendor | EP11 IN bulk | **Events**: timestamped records (UART bytes, GPIO edges, power, triggers) |
| 11 | Vendor | EP12 IN bulk | **Logic analyzer** stream (samples from DDR3 / live) (`with_la`) |
| 12-13 | UVC | EP14 IN bulk | **Eyes**: rendered view of the bench (gateware text-mode renderer) (`with_eyes`) |
| next free | Vendor (CMSIS-DAP v2) | EP13 IN/OUT bulk | **JTAG/SWD** probe (OpenOCD/pyOCD/probe-rs, no driver) (phase 3, reserved) |

Endpoint numbers are indicative (max 15 IN/15 OUT): the gateware descriptors are the reference.
Interface numbers are contiguous (USB requirement): the optional functions take the next free
interfaces in this order (logic analyzer, then Eyes: 12-13 with the logic analyzer, 11-12 without,
`agentlab_la_interface`/`agentlab_eyes_interface`); their endpoint numbers are fixed (logic
analyzer: EP12 IN, JTAG/SWD: EP13 reserved, Eyes: EP14 IN). The table shows both optional
functions enabled (Acorn/KC705 targets defaults).

Phase 1/2 (`agentlab/gateware/usb.py`, `agentlab_usb_map()`): interfaces 0-11/EP1-EP12 as above
(USB3 data endpoints: Max Burst 2 for the CDC-ACM, 4 for the events, 16 for the logic analyzer;
interface 11 only with the logic analyzer, `with_la`), VID/PID 0x1d50/0x605a
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
- `agentlab_la_*`: logic analyzer (see below).
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

## DRAM

The logic analyzer uses the board DDR3 (`AgentLabSoC.add_agentlab_dram`,
both targets: 1GB, Acorn MT41K512M16/A7DDRPHY, KC705 MT8JTF12864 SODIMM/K7DDRPHY, sys_clk 125MHz
1:4). LiteDRAM needs a CPU for the initialization/calibration: VexRiscv (integrated ROM: BIOS, 8KB
SRAM, console on a stub UART, a crossover UART would block the BIOS until a host reads it), DRAM
also mapped as the CPU main_ram (0x40000000, L2 cache). The BIOS writes the `ddrctrl_init_done`/
`ddrctrl_init_error` CSRs: `soc.dram_ready` (init done without error) enables the logic analyzer
(arm ignored before, `agentlab_la_status.dram_ready`).

`soc.sdram` (LiteDRAM crossbar) is shared: the logic analyzer takes one write and one read native
port (`crossbar.get_port()`), ring buffer in the upper half of the DRAM (512MB at 512MB,
`agentlab_la_ring_base/size`); the lower half is left to the CPU/BIOS (main_ram) and the other
users (further crossbar ports). `--no-dram`: phase 1 instrument only (no CPU/DRAM/
logic analyzer).

## Logic Analyzer

`agentlab/gateware/la.py` (`AgentLabLA`): N channels (<= 32: KC705 32 FMC pins, Acorn 32 internal
probes), inputs synchronized (2 sys_clk cycles: timestamps are the synchronized samples instants),
sampled at sys_clk/divider (125MHz max), samples are 32-bit words (channel n on bit n).

- From the arm, every sample is written to the DRAM ring buffer (LiteDRAM DMA writer) and checked
  against the trigger: immediate, pattern (`(sample & mask) == (value & mask)`, optionally with an
  edge on one channel: rising/falling/any), or external (marker CSR write, GPIO edge event of the
  GPIOs enabled in `agentlab_gpio_edge_mask`, power switch change: on the next sample).
- On the trigger (sample t), the capture starts at `t - min(pre_trigger, t)` (offset in the header)
  and continues `length - pre_trigger` samples from the trigger (trigger included). The trigger
  also emits an event (type 0x05, value: trigger sample, timestamp: trigger sample time).
- Capture mode: the readout (LiteDRAM DMA reader -> Bulk IN) starts at the end of the capture
  (length <= ring buffer: 128M samples, ~1s at 125MHz).
- Stream mode: the readout starts on the trigger, the ring buffer is a FIFO absorbing the USB stalls
  (sustained rate limited by USB: ~350-400MB/s, divider >= 2 for 32-bit samples). A full ring (host
  not keeping up) ends the capture (overflow, all samples before it are sent); length 0: until stop.
- After the trigger the writer never overwrites unread samples; samples lost (DRAM/ring full) end
  the capture with the overflow flag. Arm restarts from any state (previous capture flushed, the
  Bulk IN endpoint flushed); stop: armed: abort (nothing sent), triggered: end of the capture.
- Timing: sample k is at `start_timestamp + k*divider` (shared timebase, sys_clk ticks).

Stream format (EP12 IN, 32-bit little-endian words), per capture:

| Words | Field |
|---|---|
| 0 | magic 0x31414c41 ("ALA1") |
| 1 | version (bits 0-7, 1), channels (8-15), mode (16-23, 0: capture, 1: stream), flags (24-31, bit 0: overflow, capture mode) |
| 2 | divider (samplerate = sys_clk_freq/divider) |
| 3 | block samples (B, 256) |
| 4 | trigger offset (samples from the capture start) |
| 5 | reserved (0) |
| 6-7 | sample count (64-bit; capture mode: exact, stream mode: expected, 0: until stop) |
| 8-9 | start timestamp (64-bit, sys_clk ticks, sample 0) |
| 10-11 | trigger timestamp (64-bit) |

Then blocks until the last one: a block word (count: bits 0-15, flags: bits 16-23, bit 0: last
block, bit 1: overflow, magic 0xb1: bits 24-31) followed by count samples (B, last block <= B, can
be 0). The stream is sent continuously (packets of up to 1024 bytes, ended after ~1us without data).

CSRs:

| CSR | Description |
|---|---|
| `agentlab_la_mode` | 0: capture, 1: stream |
| `agentlab_la_divider` | samplerate = sys_clk_freq/divider (0/1: sys_clk_freq) |
| `agentlab_la_trigger_source` | 0: immediate, 1: pattern (mask/value, edge), 2: external |
| `agentlab_la_trigger_mask`, `agentlab_la_trigger_value` | pattern: channels compared, values |
| `agentlab_la_trigger_edge` | enable (bit 0), channel (bits 1-5), polarity (bits 6-7: 0 rising, 1 falling, 2 any) |
| `agentlab_la_trigger_ext` | external sources: marker (bit 0), GPIO edge (bit 1), power (bit 2) |
| `agentlab_la_pre_trigger` | samples before the trigger |
| `agentlab_la_length` | samples (pre-trigger included); stream mode: 0, until stop |
| `agentlab_la_control` | arm (bit 0), stop (bit 1) (write 1: pulse) |
| `agentlab_la_status` | state (bits 0-2: 0 idle, 1 armed, 2 triggered, 3 readout, 4 done, 5 flush), triggered (3), overflow (4), dram_ready (5) |
| `agentlab_la_level` | DRAM words buffered (not read) |
| `agentlab_la_count` | samples captured (64-bit) |
| `agentlab_la_trigger_offset` | trigger sample from the capture start |
| `agentlab_la_start_timestamp`, `agentlab_la_trigger_timestamp` | 64-bit |

Constants (csr.csv): `agentlab_la_channels`, `agentlab_la_interface` (11), `agentlab_la_ep` (12),
`agentlab_la_ring_base`, `agentlab_la_ring_size` (bytes from the DRAM start),
`agentlab_la_ring_samples`, `agentlab_la_word_samples` (samples per DRAM word),
`agentlab_la_block_samples`.

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

# Logic analyzer: 25MHz, trigger on a falling edge of channel 0 with channel 1 high, 1000 samples
# before the trigger, 10ms in total, channels 0-7 to a VCD (or .sr for PulseView, .json): prints a
# JSON summary (per channel levels, edges, first edges, frequency, duty cycle).
agentlab la capture --rate 25M --trigger "falling:0,ch1=1" --pre 1000 --length 10ms \
    --channels 0-7 --out capture.vcd
agentlab la capture --trigger marker --length 1M --out boot.sr  # Trigger on the next marker.
agentlab la capture --mode stream --length 0 --duration 2 --rate 10M --out long.sr
agentlab la status
```

Logic analyzer files: VCD with a 1ns timescale and times in device time (shared with the events:
`#t` = timestamp/sys_clk_freq), sigrok srzip (`version`, `metadata`, `logic-1-N`, plus an
`agentlab.json` member with the start/trigger timestamps, ignored by sigrok).

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
    cap = lab.la_capture(rate="10M", trigger="rising:0", pre=100, length="1ms")
    print(cap.trigger_timestamp, len(cap.samples), cap.time(0)) # Shared timebase.
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
`gpio_set`, `gpio_get`, `events`, `timestamp`, `marker`, `la_capture`, `screenshot` (PNG path of
the bench view, status panel refreshed first), `display_text`. Times are device seconds (`time_s`,
shared timebase); the instrument is opened on the first tool call and the events reader runs from
then.

Tests: `pytest test/test_agentlab_host*.py` (fake backends, default suite), `test_agentlab_la.py`
(logic analyzer gateware simulations with a LiteDRAM native ports model); with a device
connected: `AGENTLAB_CSR_CSV=... pytest -m hw test/test_agentlab_host_hw.py`.
