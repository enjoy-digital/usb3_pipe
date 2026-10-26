#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Instrument: one object for CSRs, DUT UARTs, power, GPIOs, timestamp/markers, events,
logic analyzer and Eyes (text display, status panel, screenshots)."""

import time
import threading
from typing import Callable, Dict, List, Optional

from agentlab.host import constants as c
from agentlab.host.csr       import CSRMap, CSRs, UARTBoneBus, RemoteClientBus
from agentlab.host.uart      import DUTUart
from agentlab.host.events    import EventReader, filter_events, format_events, uart_lines, usb_read_fn
from agentlab.host.eyes      import (EyesDisplay, Frame, color_index, grab_frame, lane_names,
    render_status, uart_tails)
from agentlab.host.discovery import DeviceInfo, find_device, find_usb_device
from agentlab.host.la        import (LACapture, LAStreamParser, LATrigger, parse_trigger,
    parse_samples)

# Instrument ---------------------------------------------------------------------------------------

class Instrument:
    """AgentLab instrument.

    Built from its backends (CSRs, UART factory, events read function) so tests can inject fakes;
    use `Instrument.open()` to connect to a real device.
    """
    def __init__(self, csrs: Optional[CSRs] = None,
        uart_factory : Optional[Callable[[int], DUTUart]] = None,
        events_read  : Optional[Callable[[int, int], bytes]] = None,
        la_read      : Optional[Callable[[int, int], bytes]] = None,
        sys_clk_freq : Optional[float] = None,
        device       : Optional[DeviceInfo] = None,
        sleep        : Callable[[float], None] = time.sleep,
        clock        : Callable[[], float] = time.monotonic,
        frame_grabber: Optional[Callable[..., Frame]] = None):
        self.csrs          = csrs
        self.device        = device
        self.sleep         = sleep
        self.clock         = clock
        self.frame_grabber = frame_grabber or grab_frame
        self._display      = None
        self._refresh      = None
        self._uart_factory = uart_factory
        self._uarts        = {}
        self._events_read  = events_read
        self._events       = None
        self._la_read      = la_read
        if sys_clk_freq is None and csrs is not None:
            sys_clk_freq = csrs.csr_map.sys_clk_freq
        self.sys_clk_freq  = float(sys_clk_freq or c.DEFAULT_SYS_CLK_FREQ)

    # Open / Close ---------------------------------------------------------------------------------

    @classmethod
    def open(cls, csr_csv=None, serial=None, control_port=None, server=None, spawn_server=False,
        server_port=1234, sys_clk_freq=None, sysfs_root=None, start_events=False) -> "Instrument":
        """Connect to an AgentLab device.

        csr_csv      : LiteX csr.csv/csr.json of the gateware (required for CSR features).
        serial       : USB serial number (when several devices are connected).
        control_port : control tty override (default: discovered /dev/ttyACMx of interface 0).
        server       : "host:port" of a running litex_server (RemoteClient) instead of UARTBone.
        spawn_server : spawn `litex_server --uart` on the control tty and use RemoteClient.
        start_events : start the events reader now (else on first use of `events`).
        """
        kwargs = {} if sysfs_root is None else {"sysfs_root": sysfs_root}
        device = None
        try:
            device = find_device(serial=serial, **kwargs)
        except RuntimeError:
            if control_port is None and server is None:
                raise
        csrs = None
        if csr_csv is not None:
            csr_map = CSRMap.from_file(csr_csv)
            port    = control_port or (device.control_tty if device else None)
            if server is not None:
                host, _, p = server.partition(":")
                bus = RemoteClientBus(host=host or "localhost", port=int(p or 1234))
            elif spawn_server:
                bus = RemoteClientBus(port=server_port, spawn_uart=port)
            else:
                if port is None:
                    raise RuntimeError("Control tty not found (CDC-ACM interface {}).".format(
                        c.CONTROL_INTERFACE))
                bus = UARTBoneBus(port)
            csrs = CSRs(bus, csr_map)

        def uart_factory(index):
            tty = device.uart_tty(index) if device else None
            if tty is None:
                raise RuntimeError("DUT UART{} tty not found.".format(index))
            return DUTUart(tty, index=index)

        def events_read(size, timeout_ms):
            # Bound lazily: only claims the vendor interface when events are used.
            if not hasattr(events_read, "fn"):
                if device is None:
                    raise RuntimeError("Events need the USB device (not found).")
                events_read.fn = usb_read_fn(find_usb_device(device))
            return events_read.fn(size, timeout_ms)

        def la_read(size, timeout_ms):
            # Bound lazily: only claims the logic analyzer interface when captures are made.
            if not hasattr(la_read, "fn"):
                if device is None:
                    raise RuntimeError("Logic analyzer needs the USB device (not found).")
                la_read.fn = usb_read_fn(find_usb_device(device), interface=c.LA_INTERFACE,
                    endpoint=c.LA_ENDPOINT)
            return la_read.fn(size, timeout_ms)

        instrument = cls(csrs=csrs, uart_factory=uart_factory, events_read=events_read,
            la_read=la_read, sys_clk_freq=sys_clk_freq, device=device)
        if start_events and device is not None:
            instrument.events
        return instrument

    def close(self):
        self.stop_display_refresh()
        if self._events is not None:
            self._events.stop()
        for uart in self._uarts.values():
            uart.close()
        self._uarts.clear()
        if self.csrs is not None:
            self.csrs.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # Helpers --------------------------------------------------------------------------------------

    def _csrs(self) -> CSRs:
        if self.csrs is None:
            raise RuntimeError("CSR access needs a CSR map (csr.csv/csr.json).")
        return self.csrs

    def _set_bit(self, name, index, value):
        csrs = self._csrs()
        reg  = csrs.read(name)
        reg  = (reg | (1 << index)) if value else (reg & ~(1 << index))
        csrs.write(name, reg)

    def ticks_to_s(self, ticks: int) -> float:
        return ticks/self.sys_clk_freq

    def info(self) -> Dict[str, object]:
        d = {"sys_clk_freq": self.sys_clk_freq}
        if self.device is not None:
            d.update(self.device.describe())
        if self.csrs is not None:
            d["csrs"] = sorted(n for n in self.csrs.csr_map.registers if n.startswith("agentlab_"))
        return d

    # Timestamp / Marker ---------------------------------------------------------------------------

    def timestamp(self) -> int:
        """Current device timestamp (sys_clk ticks)."""
        csrs = self._csrs()
        if c.CSR_TIMESTAMP_LATCH in csrs:
            csrs.write(c.CSR_TIMESTAMP_LATCH, 1)
        return csrs.read(c.CSR_TIMESTAMP)

    def time(self) -> float:
        """Current device time (seconds)."""
        return self.ticks_to_s(self.timestamp())

    def marker(self, value: int = 0) -> int:
        """Emit a marker event (value) and return the device timestamp read just after."""
        self._csrs().write(c.CSR_MARKER, value)
        return self.timestamp()

    # Power ----------------------------------------------------------------------------------------

    def power_set(self, index: int, on: bool):
        self._set_bit(c.CSR_POWER_OUT, index, on)

    def power_get(self, index: int) -> bool:
        """Power switch state (sense when available, else the commanded state)."""
        csrs = self._csrs()
        name = c.CSR_POWER_STATUS if c.CSR_POWER_STATUS in csrs else c.CSR_POWER_OUT
        return bool((csrs.read(name) >> index) & 1)

    def power_cycle(self, index: int, off_time: float = 1.0):
        """Switch power `index` off, wait `off_time` seconds, switch it on."""
        self.power_set(index, False)
        self.sleep(off_time)
        self.power_set(index, True)

    # GPIOs ----------------------------------------------------------------------------------------

    def gpio_dir(self, index: int, output: bool):
        self._set_bit(c.CSR_GPIO_OE, index, output)

    def gpio_set(self, index: int, value: bool):
        """Drive GPIO `index` to `value` (and enable its output)."""
        self._set_bit(c.CSR_GPIO_OUT, index, value)
        self.gpio_dir(index, True)

    def gpio_release(self, index: int):
        """Release GPIO `index` (input, high impedance)."""
        self.gpio_dir(index, False)

    def gpio_get(self, index: int) -> bool:
        return bool((self._csrs().read(c.CSR_GPIO_IN) >> index) & 1)

    def gpio_edge_events(self, index: int, enable: bool = True):
        self._set_bit(c.CSR_GPIO_EDGE_MASK, index, enable)

    # DUT UARTs ------------------------------------------------------------------------------------

    def uart(self, index: int) -> DUTUart:
        if index not in self._uarts:
            if self._uart_factory is None:
                raise RuntimeError("No DUT UART backend.")
            self._uarts[index] = self._uart_factory(index)
        return self._uarts[index]

    def uart_set_baudrate(self, index: int, baudrate: int):
        """Set the DUT UART baudrate (SET_LINE_CODING, plus the CSR when present)."""
        self.uart(index).set_baudrate(baudrate)
        name = c.csr_uart_baudrate(index)
        if self.csrs is not None and name in self.csrs:
            self.csrs.write(name, int(baudrate))

    # Events ---------------------------------------------------------------------------------------

    @property
    def events(self) -> EventReader:
        """Events reader (background thread started on first access)."""
        if self._events is None:
            if self._events_read is None:
                raise RuntimeError("No events backend.")
            self._events = EventReader(self._events_read).start()
        return self._events

    def events_between(self, since=None, until=None, types=None, source=None) -> List:
        """Events received in [since, until] (device seconds), filtered by types/source."""
        return filter_events(self.events.snapshot(), since=since, until=until, types=types,
            source=source, sys_clk_freq=self.sys_clk_freq)

    def uart_lines(self, index=None, since=None, until=None):
        """DUT UART RX text lines (with device timestamps) from the events stream."""
        events = self.events_between(since=since, until=until, types="uart_rx", source=index)
        return uart_lines(events)

    # Logic Analyzer -------------------------------------------------------------------------------

    def _la_constant(self, name, default=None):
        return self._csrs().csr_map.constants.get(name, default)

    @property
    def la_channels(self) -> int:
        """Logic analyzer channels."""
        return int(self._la_constant(c.CONST_LA_CHANNELS, 32))

    @property
    def la_ring_samples(self) -> Optional[int]:
        """Logic analyzer DRAM ring buffer depth (samples): capture mode maximum length."""
        n = self._la_constant(c.CONST_LA_RING_SAMPLES)
        return None if n is None else int(n)

    def la_divider(self, rate=None) -> int:
        """Divider for a samplerate (Hz, k/M suffixes, None: sys_clk), samplerate = sys_clk/div."""
        if rate is None:
            return 1
        from agentlab.host.la import parse_number
        return max(1, int(round(self.sys_clk_freq/parse_number(rate))))

    def la_configure(self, rate=None, trigger=None, pre=0, length=4096, mode="capture") -> Dict:
        """Configure the logic analyzer (not armed). rate: samplerate (Hz, "25M"); trigger:
        LATrigger or spec string (see agentlab.host.la.parse_trigger); pre/length: samples or
        durations ("10us"); mode: "capture" (DRAM ring buffer, then readout) or "stream"
        (continuous, length 0: until la_stop). Returns the configuration (CSR values)."""
        csrs     = self._csrs()
        divider  = self.la_divider(rate)
        rate_hz  = self.sys_clk_freq/divider
        pre      = parse_samples(pre, rate_hz)
        length   = parse_samples(length, rate_hz)
        trig     = trigger if isinstance(trigger, LATrigger) else parse_trigger(trigger)
        if mode not in c.LA_MODES:
            raise ValueError("Invalid LA mode {} (capture or stream).".format(mode))
        ring = self.la_ring_samples
        if mode == "capture" and length <= 0:
            raise ValueError("Capture mode needs a length > 0.")
        if length and pre >= length:
            raise ValueError("Pre-trigger ({}) must be < length ({}).".format(pre, length))
        if ring is not None and mode == "capture" and length > ring:
            raise ValueError("Length {} exceeds the ring buffer ({} samples): use stream "
                "mode.".format(length, ring))
        if ring is not None and pre > ring//2:
            raise ValueError("Pre-trigger {} exceeds half the ring buffer ({} samples).".format(
                pre, ring//2))
        if length >= 2**32:
            raise ValueError("Length {} too large (32-bit).".format(length))
        csrs.write(c.CSR_LA_MODE,           c.LA_MODES[mode])
        csrs.write(c.CSR_LA_DIVIDER,        divider)
        csrs.write(c.CSR_LA_TRIGGER_SOURCE, trig.source)
        csrs.write(c.CSR_LA_TRIGGER_MASK,   trig.mask)
        csrs.write(c.CSR_LA_TRIGGER_VALUE,  trig.value)
        csrs.write(c.CSR_LA_TRIGGER_EDGE,   trig.edge_csr)
        csrs.write(c.CSR_LA_TRIGGER_EXT,    trig.ext)
        csrs.write(c.CSR_LA_PRE_TRIGGER,    pre)
        csrs.write(c.CSR_LA_LENGTH,         length)
        return {"mode": mode, "divider": divider, "samplerate_hz": rate_hz, "pre": pre,
            "length": length, "trigger": trig}

    def la_arm(self):
        self._csrs().write(c.CSR_LA_CONTROL, 0b01)

    def la_stop(self):
        """Stop: armed: abort (nothing sent), triggered: end the capture (stream mode)."""
        self._csrs().write(c.CSR_LA_CONTROL, 0b10)

    def la_status(self) -> Dict:
        csrs   = self._csrs()
        status = csrs.read(c.CSR_LA_STATUS)
        return {
            "state"      : c.LA_STATES.get(status & 0x7, "unknown"),
            "triggered"  : bool((status >> 3) & 1),
            "overflow"   : bool((status >> 4) & 1),
            "dram_ready" : bool((status >> 5) & 1),
            "count"      : csrs.read(c.CSR_LA_COUNT),
            "level"      : csrs.read(c.CSR_LA_LEVEL),
        }

    def la_capture(self, rate=None, trigger=None, pre=0, length=4096, mode="capture",
        timeout=10.0, duration=None, transfer_size=1 << 20) -> LACapture:
        """Configure, arm and read a capture (LACapture: samples in the shared timebase).

        timeout : seconds to wait for the trigger/end of the capture (no trigger: capture aborted,
                  TimeoutError).
        duration: stream mode with length 0: stop after duration seconds.
        """
        if self._la_read is None:
            raise RuntimeError("No logic analyzer backend.")
        self.la_configure(rate=rate, trigger=trigger, pre=pre, length=length, mode=mode)
        status = self.la_status()
        if not status["dram_ready"]:
            raise RuntimeError("Logic analyzer DRAM not ready (DRAM initialization failed?).")
        parser = LAStreamParser(self.sys_clk_freq)
        self.la_arm()
        start   = self.clock()
        stopped = False
        while not parser.done:
            data    = self._la_read(transfer_size, 100)
            elapsed = self.clock() - start
            # Duration (stream mode): checked on every read, the stream never pauses.
            if duration is not None and not stopped and elapsed > duration:
                self.la_stop()
                stopped = True
            if data:
                parser.feed(data)
                continue
            if elapsed > timeout + (duration or 0):
                status = self.la_status()
                if not status["triggered"]:
                    self.la_stop()
                    raise TimeoutError("Logic analyzer: no trigger after {:.1f}s.".format(
                        elapsed))
                raise TimeoutError("Logic analyzer: capture incomplete after {:.1f}s "
                    "({} samples received, status {}).".format(elapsed, len(parser.samples),
                    status))
        return parser.capture

    def la_capture_summary(self, channels=None, names=None, out=None, **kwargs) -> Dict:
        """la_capture, write `out` (.vcd/.sr/.json) and return a summary (agentlab.host.la.summarize)
        of the selected channels ("0-7,12", list or None: all) with the file path."""
        from agentlab.host.la import parse_channels, summarize, write_capture
        capture = self.la_capture(**kwargs)
        chans   = parse_channels(channels, capture.channels)
        if isinstance(names, str):
            names = names.split(",")
        if names is not None and len(names) == len(chans):
            full = ["ch{}".format(i) for i in range(capture.channels)]
            for ch, name in zip(chans, names):
                full[ch] = name
            names = full
        if out is not None:
            write_capture(capture, out, channels=chans, names=names)
        summary = summarize(capture, chans, names)
        summary["file"] = out
        return summary

    # Eyes -----------------------------------------------------------------------------------------

    def _constant(self, name, default=None):
        if self.csrs is None:
            return default
        return self.csrs.csr_map.constants.get(name, default)

    def eyes_info(self) -> Dict[str, int]:
        """Eyes geometry (csr.csv constants, defaults otherwise)."""
        return {k: int(self._constant("agentlab_eyes_" + k, v)) for k, v in c.EYES_DEFAULTS.items()}

    @property
    def display(self) -> EyesDisplay:
        """Eyes text display (shadow of the gateware text grid, see EyesDisplay)."""
        if self._display is None:
            csrs   = self._csrs()
            region = csrs.csr_map.regions.get(c.EYES_REGION, None)
            if region is None:
                raise RuntimeError("No Eyes in this gateware (memory region {} not in the CSR "
                    "map).".format(c.EYES_REGION))
            info = self.eyes_info()
            self._display = EyesDisplay(csrs.bus.write, region.origin,
                cols=info["columns"], rows=info["rows"])
        return self._display

    def display_text(self, text, col=0, row=0, fg="white", bg="black", clear=False):
        """Write text on the Eyes display at (col, row) (newlines: next rows), clear: whole screen
        cleared first."""
        d = self.display
        if clear:
            d.clear()
        d.text(col, row, text, fg, bg)
        d.flush()

    def display_clear(self, bg="black"):
        self.display.clear(bg)
        self.display.flush()

    def display_waves(self, lanes=None, first=None, divider=None, period_s=None, run=None,
        single=False, row=None, column=None, color=None):
        """Configure the Eyes waveform lanes: lanes (count, 0: off), first lane source bit, sample
        period (divider: sys_clk cycles - 1, or period_s), run (roll mode) / single (capture one
        window), row/column (character position), color."""
        csrs = self._csrs()
        if period_s is not None:
            divider = max(int(round(period_s*self.sys_clk_freq)) - 1, 0)
        for name, value in [(c.CSR_EYES_WAVE_LANES, lanes), (c.CSR_EYES_WAVE_FIRST, first),
            (c.CSR_EYES_WAVE_DIVIDER, divider), (c.CSR_EYES_WAVE_ROW, row),
            (c.CSR_EYES_WAVE_COLUMN, column)]:
            if value is not None:
                csrs.write(name, int(value))
        if color is not None:
            csrs.write(c.CSR_EYES_WAVE_COLOR, color_index(color))
        if run is not None or single:
            if run is None:
                run = csrs.read(c.CSR_EYES_WAVE_CONTROL) & 1
            csrs.write(c.CSR_EYES_WAVE_CONTROL, int(bool(run)) | (2 if single else 0))

    def status_state(self) -> Dict[str, object]:
        """Status panel state (device time, power/GPIOs, DUT UARTs tails, last events, lanes)."""
        state = {"title": "AgentLab Eyes"}
        csrs  = self._csrs()
        if self.device is not None and self.device.serial:
            state["title"] += " - " + self.device.serial
        state["time_s"] = self.time()
        n_power = int(self._constant("agentlab_power", 0))
        n_gpios = int(self._constant("agentlab_gpios", 0))
        n_uarts = int(self._constant("agentlab_uarts", c.DUT_UART_COUNT))
        if n_power:
            name  = c.CSR_POWER_STATUS if c.CSR_POWER_STATUS in csrs else c.CSR_POWER_OUT
            power = csrs.read(name)
            state["power"] = [bool((power >> i) & 1) for i in range(n_power)]
        if n_gpios:
            gpio_in, gpio_oe = csrs.read(c.CSR_GPIO_IN), csrs.read(c.CSR_GPIO_OE)
            state["gpio_in"] = [bool((gpio_in >> i) & 1) for i in range(n_gpios)]
            state["gpio_oe"] = [bool((gpio_oe >> i) & 1) for i in range(n_gpios)]
        try:
            events = self.events.snapshot()
            state["uarts"]  = uart_tails(events, n_uarts=n_uarts)
            others = [e for e in events if e.type not in [c.EVENT_UART_RX, c.EVENT_UART_TX]]
            state["events"] = format_events(others[-4:], self.sys_clk_freq) or ["(no events)"]
        except RuntimeError:
            pass
        if c.CSR_EYES_WAVE_LANES in csrs:
            lanes   = csrs.read(c.CSR_EYES_WAVE_LANES)
            first   = csrs.read(c.CSR_EYES_WAVE_FIRST)
            period  = (csrs.read(c.CSR_EYES_WAVE_DIVIDER) + 1)/self.sys_clk_freq
            depth   = self.eyes_info()["wave_depth"]
            running = csrs.read(c.CSR_EYES_WAVE_STATUS) & 1
            names   = lane_names(n_gpios, n_power, n_uarts)
            state["lanes"] = [names[i] if i < len(names) else "bit{}".format(i)
                for i in range(first, first + lanes)]
            state["wave"]  = "(sample period {:.3f} us, window {:.3f} ms, {})".format(
                period*1e6, period*depth*1e3, "running" if running else "stopped")
        return state

    def display_status(self) -> Dict[str, object]:
        """Draw the status panel on the Eyes display (one refresh), return its state."""
        state = self.status_state()
        render_status(self.display, state)
        self.display.flush()
        return state

    def start_display_refresh(self, period=1.0):
        """Refresh the status panel every period seconds (background thread)."""
        self.stop_display_refresh()
        stop = threading.Event()
        def run():
            while not stop.is_set():
                try:
                    self.display_status()
                except Exception: # Device unplugged...: keep trying.
                    pass
                stop.wait(period)
        thread = threading.Thread(target=run, name="agentlab-eyes", daemon=True)
        self._refresh = (stop, thread)
        thread.start()

    def stop_display_refresh(self):
        if self._refresh is not None:
            stop, thread = self._refresh
            stop.set()
            thread.join(timeout=5.0)
            self._refresh = None

    def video_device(self) -> str:
        """Eyes UVC capture node (/dev/videoN) of the device."""
        if self.device is None or self.device.video_device is None:
            raise RuntimeError("Eyes video node not found (gateware without Eyes, uvcvideo not "
                "loaded?).")
        return self.device.video_device

    def screenshot(self, path, refresh=False, backend="auto", video_device=None,
        timeout=10.0) -> Dict[str, object]:
        """Grab one Eyes frame to a PNG file (refresh: redraw the status panel first)."""
        if refresh:
            self.display_status()
        info   = self.eyes_info()
        device = video_device or self.video_device()
        frame  = self.frame_grabber(device, width=info["width"], height=info["height"],
            backend=backend, timeout=timeout)
        size   = frame.save_png(path)
        return {"path": path, "width": frame.width, "height": frame.height, "bytes": size,
            "device": device}
