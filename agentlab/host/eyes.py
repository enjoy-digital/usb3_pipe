#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Eyes host side: text display (character grid written over the CSR bus), status panel
and frame grabbing from the UVC function (/dev/videoN) to PNG.

- EyesDisplay   : shadow of the gateware text grid (160x45 cells of 8x16 pixels), text/box drawing,
                  flush of the changed cells only (Wishbone burst writes).
- render_status : status panel (time, power/GPIOs, DUT UARTs tails, last events, waveform lanes
                  labels) drawn on an EyesDisplay.
- grab_frame    : one frame from the UVC node (OpenCV, ffmpeg or v4l2-ctl), as RGB.
- Frame         : RGB frame, PNG export (pure Python, zlib).
"""

import os
import zlib
import shutil
import struct
import tempfile
import subprocess
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence

from agentlab.host import constants as c
from agentlab.gateware.eyes_font import eyes_char_code # Pure Python (no Migen).

# Cells --------------------------------------------------------------------------------------------

def color_index(color) -> int:
    """Palette index of a color (name from constants.EYES_COLORS or index 0-15)."""
    if isinstance(color, str):
        try:
            return c.EYES_COLORS.index(color)
        except ValueError:
            raise ValueError("Unknown color {} (one of {}).".format(color,
                ", ".join(c.EYES_COLORS))) from None
    if not (0 <= int(color) < 16):
        raise ValueError("Color index {} out of range (0-15).".format(color))
    return int(color)

def eyes_cell(char, fg="white", bg="black") -> int:
    """Text grid cell: font code [7:0], foreground [11:8], background [15:12]."""
    return (eyes_char_code(char) & 0xff) | (color_index(fg) << 8) | (color_index(bg) << 12)

# Display ------------------------------------------------------------------------------------------

class EyesDisplay:
    """Text grid shadow, flushed to the gateware text grid (only the changed cells).

    write(addr, datas) is the CSR bus write (byte address, 32-bit words); origin: text grid region
    base (csr.csv memory_region agentlab_eyes).
    """
    def __init__(self, write: Callable[[int, List[int]], None], origin: int,
        cols=c.EYES_DEFAULTS["columns"], rows=c.EYES_DEFAULTS["rows"]):
        self._write  = write
        self.origin  = origin
        self.cols    = cols
        self.rows    = rows
        self.cells   = [eyes_cell(" ")]*(cols*rows)
        self._device = [None]*(cols*rows) # Unknown: everything written on the first flush.

    # Drawing --------------------------------------------------------------------------------------

    def clear(self, bg="black"):
        self.cells = [eyes_cell(" ", "white", bg)]*(self.cols*self.rows)

    def put(self, col, row, char, fg="white", bg="black"):
        if 0 <= col < self.cols and 0 <= row < self.rows:
            self.cells[row*self.cols + col] = eyes_cell(char, fg, bg)

    def text(self, col, row, text, fg="white", bg="black", width=None):
        """Write text at (col, row) (newlines: next rows from col), clipped to the screen (and to
        width columns, padded with spaces)."""
        for i, line in enumerate(str(text).split("\n")):
            if width is not None:
                line = line[:width].ljust(width)
            for j, ch in enumerate(line):
                self.put(col + j, row + i, ch, fg, bg)

    def fill(self, col, row, width, height, char=" ", fg="white", bg="black"):
        for r in range(row, row + height):
            for k in range(col, col + width):
                self.put(k, r, char, fg, bg)

    def box(self, col, row, width, height, title=None, fg="gray", bg="black", title_fg="bright_white"):
        """Single line box (box drawing characters), optional title on the top border."""
        right, bottom = col + width - 1, row + height - 1
        for k in range(col + 1, right):
            self.put(k, row,    "─", fg, bg)
            self.put(k, bottom, "─", fg, bg)
        for r in range(row + 1, bottom):
            self.put(col,   r, "│", fg, bg)
            self.put(right, r, "│", fg, bg)
        for (k, r), ch in [((col, row), "┌"), ((right, row), "┐"),
            ((col, bottom), "└"), ((right, bottom), "┘")]:
            self.put(k, r, ch, fg, bg)
        if title:
            self.text(col + 2, row, " {} ".format(title)[:max(width - 4, 0)], title_fg, bg)

    def get_text(self, row) -> str:
        """Characters of a row (font codes, for tests/inspection)."""
        return "".join(chr(cell & 0xff) for cell in self.cells[row*self.cols:(row + 1)*self.cols])

    # Flush ----------------------------------------------------------------------------------------

    def invalidate(self):
        """Forget the device content (next flush rewrites everything)."""
        self._device = [None]*(self.cols*self.rows)

    def flush(self) -> int:
        """Write the changed cells (one burst per row, from the first to the last changed cell),
        return the number of words written."""
        written = 0
        for row in range(self.rows):
            base    = row*self.cols
            changed = [i for i in range(self.cols)
                if self.cells[base + i] != self._device[base + i]]
            if not changed:
                continue
            first, last = changed[0], changed[-1]
            datas = self.cells[base + first:base + last + 1]
            self._write(self.origin + 4*(base + first), list(datas))
            self._device[base + first:base + last + 1] = datas
            written += len(datas)
        return written

# Status Panel -------------------------------------------------------------------------------------

STATUS_UART_ROW   = 2  # 2x2 DUT UARTs boxes (80x10), rows 2-21.
STATUS_EVENTS_ROW = 22 # Last events box, rows 22-27.
STATUS_WAVE_ROW   = 28 # Waveforms title, lanes on rows 29-44 (gateware defaults).
STATUS_WAVE_COL   = 32 # Lanes labels on columns 0-31, waveforms from column 32.

def lane_names(n_gpios, n_power, n_uarts):
    """Waveform lanes sources (gateware order: GPIOs, power switches, DUT UARTs TX/RX lines)."""
    names  = ["gpio{}".format(i) for i in range(n_gpios)]
    names += ["power{}".format(i) for i in range(n_power)]
    for i in range(n_uarts):
        names += ["uart{}_tx".format(i), "uart{}_rx".format(i)]
    return names

def render_status(display: EyesDisplay, state: Dict[str, object]):
    """Draw the status panel from a state dict (missing entries: shown as unavailable):

    time_s, title, power (list of bools), gpio_in/gpio_oe (lists of bools), uarts (list of lists
    of text lines, tails), events (list of text lines), lanes (names of the displayed lanes),
    wave (text describing the sample period/window).
    """
    d = display
    d.clear()
    # Title bar.
    title = state.get("title", "AgentLab Eyes")
    now   = state.get("time_s", None)
    right = "t = {:.6f} s ".format(now) if now is not None else "t = n/a "
    d.text(0, 0, (" " + title).ljust(d.cols - len(right)) + right, "bright_white", "blue",
        width=d.cols)

    # Power / GPIOs.
    col = 1
    d.text(col, 1, "Power:", "bright_white")
    col += 7
    for i, on in enumerate(state.get("power", [])):
        s = "{}:{}".format(i, "ON " if on else "off")
        d.text(col, 1, s, "bright_green" if on else "red")
        col += len(s) + 1
    col += 2
    d.text(col, 1, "GPIO:", "bright_white")
    col += 6
    gpio_oe = state.get("gpio_oe", [])
    for i, level in enumerate(state.get("gpio_in", [])):
        out = i < len(gpio_oe) and gpio_oe[i]
        s   = "{}{}{}".format(i, ">" if out else "<", int(level))
        d.text(col, 1, s, "bright_yellow" if level else "cyan")
        col += len(s) + 1

    # DUT UARTs (2x2 boxes, tails).
    uarts = state.get("uarts", None)
    bw, bh = d.cols//2, 10
    for n in range(4):
        x, y = (n % 2)*bw, STATUS_UART_ROW + (n//2)*bh
        d.box(x, y, bw, bh, title="UART{}".format(n))
        lines = (uarts[n] if uarts is not None and n < len(uarts) else ["(events unavailable)"])
        for i, line in enumerate(list(lines)[-(bh - 2):]):
            d.text(x + 1, y + 1 + i, _printable(line), "bright_white", width=bw - 2)

    # Last events.
    events = state.get("events", ["(events unavailable)"])
    d.box(0, STATUS_EVENTS_ROW, d.cols, 6, title="Events")
    for i, line in enumerate(list(events)[-4:]):
        d.text(1, STATUS_EVENTS_ROW + 1 + i, _printable(line), "white", width=d.cols - 2)

    # Waveforms: title and lanes labels (lanes drawn by the gateware).
    d.text(0, STATUS_WAVE_ROW, (" Waveforms " + state.get("wave", "")).ljust(d.cols), "bright_white",
        "blue", width=d.cols)
    for i, name in enumerate(state.get("lanes", [])[:d.rows - STATUS_WAVE_ROW - 1]):
        d.text(1, STATUS_WAVE_ROW + 1 + i, name, "bright_cyan", width=STATUS_WAVE_COL - 2)

def _printable(text) -> str:
    return "".join(ch if ch.isprintable() else "." for ch in str(text).replace("\t", "    "))

def uart_tails(events, n_uarts=4, n_lines=8) -> List[List[str]]:
    """Last text lines received on each DUT UART (uart_rx events), last line possibly partial."""
    tails = []
    for n in range(n_uarts):
        data  = bytes(e.value & 0xff for e in events if e.type == c.EVENT_UART_RX and e.source == n)
        text  = data[-4096:].decode(errors="replace").replace("\r\n", "\n").replace("\r", "\n")
        lines = text.split("\n")
        if lines and lines[-1] == "":
            lines = lines[:-1]
        tails.append(lines[-n_lines:])
    return tails

# Frames -------------------------------------------------------------------------------------------

@dataclass
class Frame:
    width  : int
    height : int
    rgb    : bytes # width*height*3, row-major.

    def pixel(self, x, y):
        i = 3*(y*self.width + x)
        return tuple(self.rgb[i:i + 3])

    def png(self) -> bytes:
        return png_encode(self.rgb, self.width, self.height)

    def save_png(self, path) -> int:
        """Write a PNG file, return its size in bytes."""
        data = self.png()
        with open(path, "wb") as f:
            f.write(data)
        return len(data)

def _clip(v):
    return 0 if v < 0 else 255 if v > 255 else int(v)

def yuy2_to_rgb(data: bytes, width: int, height: int) -> bytes:
    """YUY2 (Y0 U Y1 V, BT.601 limited range) to RGB (numpy when available)."""
    if len(data) < width*height*2:
        raise ValueError("YUY2 frame too short ({} < {} bytes).".format(len(data), width*height*2))
    data = data[:width*height*2]
    try:
        import numpy as np
    except ImportError:
        np = None
    if np is not None:
        a  = np.frombuffer(data, dtype=np.uint8).reshape(-1, 4).astype(np.float32)
        y  = np.empty((a.shape[0], 2), dtype=np.float32)
        y[:, 0], y[:, 1] = a[:, 0], a[:, 2]
        y  = (y - 16)*1.164
        u  = (a[:, 1] - 128)[:, None]
        v  = (a[:, 3] - 128)[:, None]
        r  = y + 1.596*v
        g  = y - 0.392*u - 0.813*v
        b  = y + 2.017*u
        rgb = np.stack([r, g, b], axis=-1).round().clip(0, 255).astype(np.uint8)
        return rgb.tobytes()
    out = bytearray(width*height*3)
    o   = 0
    for i in range(0, len(data), 4):
        y0, u, y1, v = data[i], data[i + 1] - 128, data[i + 2], data[i + 3] - 128
        for yy in (y0, y1):
            yy = (yy - 16)*1.164
            out[o:o + 3] = bytes([_clip(round(yy + 1.596*v)), _clip(round(yy - 0.392*u - 0.813*v)),
                _clip(round(yy + 2.017*u))])
            o += 3
    return bytes(out)

def png_encode(rgb: bytes, width: int, height: int) -> bytes:
    """RGB 8-bit PNG (pure Python, zlib)."""
    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data +
            struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))
    stride = 3*width
    raw    = b"".join(b"\x00" + rgb[y*stride:(y + 1)*stride] for y in range(height))
    return (b"\x89PNG\r\n\x1a\n" +
        chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
        chunk(b"IDAT", zlib.compress(raw, 6)) +
        chunk(b"IEND", b""))

def png_decode(data: bytes):
    """Minimal decoder of the PNGs written by png_encode (tests): (width, height, rgb)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    i, idat, width, height = 8, b"", 0, 0
    while i < len(data):
        length = struct.unpack(">I", data[i:i + 4])[0]
        kind   = data[i + 4:i + 8]
        body   = data[i + 8:i + 8 + length]
        if kind == b"IHDR":
            width, height = struct.unpack(">II", body[:8])
        elif kind == b"IDAT":
            idat += body
        i += 12 + length
    raw    = zlib.decompress(idat)
    stride = 3*width
    rgb    = b"".join(raw[y*(stride + 1) + 1:(y + 1)*(stride + 1)] for y in range(height))
    return width, height, rgb

# Frame Grabbing -----------------------------------------------------------------------------------

GRAB_BACKENDS = ["opencv", "ffmpeg", "v4l2-ctl"]

def _grab_opencv(device, width, height, timeout):
    import cv2
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError("OpenCV: cannot open {}.".format(device))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, bgr = cap.read()
        if not ok:
            raise RuntimeError("OpenCV: no frame from {}.".format(device))
        h, w = bgr.shape[:2]
        return Frame(w, h, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes())
    finally:
        cap.release()

def _grab_ffmpeg(device, width, height, timeout, run=subprocess.run):
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2",
        "-input_format", "yuyv422", "-video_size", "{}x{}".format(width, height), "-i", device,
        "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "yuyv422", "-"]
    p = run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    if p.returncode != 0 or len(p.stdout) < width*height*2:
        raise RuntimeError("ffmpeg: no frame from {} ({}).".format(device,
            p.stderr.decode(errors="replace").strip()[-200:]))
    return Frame(width, height, yuy2_to_rgb(p.stdout, width, height))

def _grab_v4l2_ctl(device, width, height, timeout, run=subprocess.run):
    with tempfile.TemporaryDirectory() as tmp:
        raw = os.path.join(tmp, "frame.yuy2")
        cmd = ["v4l2-ctl", "-d", device,
            "--set-fmt-video=width={},height={},pixelformat=YUYV".format(width, height),
            "--stream-mmap", "--stream-count=1", "--stream-to={}".format(raw)]
        p = run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
        data = b""
        if os.path.exists(raw):
            with open(raw, "rb") as f:
                data = f.read()
        if p.returncode != 0 or len(data) < width*height*2:
            raise RuntimeError("v4l2-ctl: no frame from {} ({}).".format(device,
                p.stderr.decode(errors="replace").strip()[-200:]))
    return Frame(width, height, yuy2_to_rgb(data, width, height))

def available_backends(which=shutil.which) -> List[str]:
    backends = []
    try:
        import cv2 # noqa: F401
        backends.append("opencv")
    except ImportError:
        pass
    if which("ffmpeg"):
        backends.append("ffmpeg")
    if which("v4l2-ctl"):
        backends.append("v4l2-ctl")
    return backends

def grab_frame(device, width=c.EYES_DEFAULTS["width"], height=c.EYES_DEFAULTS["height"],
    backend="auto", timeout=10.0, run=subprocess.run, which=shutil.which) -> Frame:
    """Grab one frame from a UVC node (YUY2): backend "opencv", "ffmpeg", "v4l2-ctl" or "auto"
    (first available, in this order)."""
    if backend == "auto":
        backends = available_backends(which)
        if not backends:
            raise RuntimeError("No frame grabber: install OpenCV (python3-opencv), ffmpeg or "
                "v4l2-ctl (v4l-utils).")
    else:
        if backend not in GRAB_BACKENDS:
            raise ValueError("Unknown backend {} (one of {}).".format(backend,
                ", ".join(GRAB_BACKENDS)))
        backends = [backend]
    errors = []
    for b in backends:
        try:
            if b == "opencv":
                return _grab_opencv(device, width, height, timeout)
            if b == "ffmpeg":
                return _grab_ffmpeg(device, width, height, timeout, run)
            return _grab_v4l2_ctl(device, width, height, timeout, run)
        except (RuntimeError, OSError, subprocess.TimeoutExpired) as e:
            errors.append(str(e))
    raise RuntimeError("Frame grab failed: " + " / ".join(errors))
