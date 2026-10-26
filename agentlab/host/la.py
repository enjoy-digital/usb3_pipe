#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab logic analyzer (host side): configuration/trigger parsing, capture stream parser,
VCD and sigrok (.sr, srzip) writers/readers, per-channel summaries for agents.

Stream format: agentlab/gateware/la.py, doc/agentlab.md. Timestamps are device ticks (sys_clk,
shared with the events stream): sample k is at start_timestamp + k*divider.
"""

import io
import re
import json
import array
import struct
import zipfile
import configparser
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from agentlab.host import constants as c

# Configuration ------------------------------------------------------------------------------------

_SUFFIXES = {"": 1, "k": 1e3, "K": 1e3, "M": 1e6, "G": 1e9}

def parse_number(s) -> float:
    """Number with an optional k/M/G suffix ("10M", "2.5k", "1000", "0x10")."""
    if isinstance(s, (int, float)):
        return s
    s = s.strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    m = re.fullmatch(r"([0-9.eE+-]+)\s*([kKMG]?)(?:Hz|hz)?", s)
    if m is None:
        raise ValueError("Invalid number: {!r}.".format(s))
    return float(m.group(1))*_SUFFIXES[m.group(2)]

def parse_samples(s, samplerate) -> int:
    """Samples count: integer (k/M suffixes) or a duration ("10ms", "2us", "1s") at samplerate."""
    if isinstance(s, (int, float)):
        return int(s)
    s = s.strip()
    m = re.fullmatch(r"([0-9.eE+-]+)\s*(s|ms|us|ns)", s)
    if m is not None:
        scale = {"s": 1, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}[m.group(2)]
        return int(round(float(m.group(1))*scale*samplerate))
    return int(parse_number(s))

def parse_channels(s, n=32) -> List[int]:
    """Channels list: "0-7,12" -> [0..7, 12] (None/"all": every channel)."""
    if s is None or (isinstance(s, str) and s.strip() in ["", "all"]):
        return list(range(n))
    if isinstance(s, (list, tuple)):
        return [int(x) for x in s]
    r = []
    for part in str(s).split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            r += list(range(int(a), int(b) + 1))
        else:
            r.append(int(part))
    for ch in r:
        if not 0 <= ch < n:
            raise ValueError("Channel {} out of range (0-{}).".format(ch, n - 1))
    return r

@dataclass
class LATrigger:
    """Trigger configuration (CSR values)."""
    source   : int = c.LA_TRIGGER_IMMEDIATE
    mask     : int = 0
    value    : int = 0
    edge     : Optional[int] = None # Edge channel (pattern trigger).
    polarity : int = c.LA_EDGES["rising"]
    ext      : int = 0              # External sources bits.

    @property
    def edge_csr(self) -> int:
        if self.edge is None:
            return 0
        return 1 | (self.edge << 1) | (self.polarity << 6)

def parse_trigger(s) -> LATrigger:
    """Trigger spec: "immediate" (default), pattern/edge terms separated by commas:
    "ch3=1", "ch4=0", "rising:2", "falling:2", "edge:2" (any edge), or "mask=0x0f,value=0x05";
    external: "marker", "gpio", "power" (or "ext:marker+gpio")."""
    t = LATrigger()
    if s is None or s.strip() in ["", "immediate", "now"]:
        return t
    for term in [x.strip() for x in s.split(",") if x.strip()]:
        m = re.fullmatch(r"ch(\d+)=([01])", term)
        if m:
            ch, v = int(m.group(1)), int(m.group(2))
            t.source = c.LA_TRIGGER_PATTERN
            t.mask  |= 1 << ch
            t.value  = (t.value & ~(1 << ch)) | (v << ch)
            continue
        m = re.fullmatch(r"(mask|value)=(\S+)", term)
        if m:
            t.source = c.LA_TRIGGER_PATTERN
            setattr(t, m.group(1), int(m.group(2), 0))
            continue
        m = re.fullmatch(r"(rising|falling|edge):(\d+)", term)
        if m:
            t.source   = c.LA_TRIGGER_PATTERN
            t.edge     = int(m.group(2))
            t.polarity = c.LA_EDGES[m.group(1)]
            continue
        names = term[4:].split("+") if term.startswith("ext:") else [term]
        if all(name in c.LA_EXT_SOURCES for name in names):
            for name in names:
                t.ext |= 1 << c.LA_EXT_SOURCES[name]
            continue
        raise ValueError("Invalid trigger term {!r} (chN=0|1, rising:N, falling:N, edge:N, "
            "mask=/value=, marker, gpio, power, immediate).".format(term))
    if t.ext:
        t.source = c.LA_TRIGGER_EXTERNAL
    if t.ext and (t.mask or t.edge is not None):
        raise ValueError("External and pattern triggers can not be combined.")
    return t

# Capture ------------------------------------------------------------------------------------------

@dataclass
class LAHeader:
    version           : int
    channels          : int
    mode              : int
    flags             : int
    divider           : int
    block_samples     : int
    trigger_offset    : int
    count             : int # Capture mode: exact, stream mode: expected (0: until stop).
    start_timestamp   : int
    trigger_timestamp : int

    _STRUCT = struct.Struct("<IBBBBIIIIQQQ")

    @classmethod
    def unpack(cls, data: bytes) -> "LAHeader":
        (magic, version, channels, mode, flags, divider, block_samples, trigger_offset, _,
            count, start_ts, trigger_ts) = cls._STRUCT.unpack(data[:4*c.LA_HEADER_WORDS])
        if magic != c.LA_MAGIC:
            raise ValueError("Bad LA header magic 0x{:08x}.".format(magic))
        return cls(version, channels, mode, flags, divider, block_samples, trigger_offset, count,
            start_ts, trigger_ts)

    def pack(self) -> bytes:
        return self._STRUCT.pack(c.LA_MAGIC, self.version, self.channels, self.mode, self.flags,
            self.divider, self.block_samples, self.trigger_offset, 0, self.count,
            self.start_timestamp, self.trigger_timestamp)

@dataclass
class LACapture:
    """A logic analyzer capture: samples (array of 32-bit words, channel n on bit n) and timing."""
    header       : LAHeader
    samples      : array.array
    overflow     : bool  = False
    sys_clk_freq : float = c.DEFAULT_SYS_CLK_FREQ

    @property
    def channels(self) -> int:
        return self.header.channels

    @property
    def divider(self) -> int:
        return self.header.divider

    @property
    def samplerate(self) -> float:
        return self.sys_clk_freq/self.header.divider

    @property
    def start_timestamp(self) -> int:
        return self.header.start_timestamp

    @property
    def trigger_timestamp(self) -> int:
        return self.header.trigger_timestamp

    @property
    def trigger_offset(self) -> int:
        return self.header.trigger_offset

    def timestamp(self, k) -> int:
        """Device timestamp (ticks) of sample k."""
        return self.header.start_timestamp + k*self.header.divider

    def time(self, k) -> float:
        """Device time (seconds) of sample k."""
        return self.timestamp(k)/self.sys_clk_freq

    def channel(self, ch) -> List[int]:
        return [(s >> ch) & 1 for s in self.samples]

    def changes(self) -> List[int]:
        """Indexes k (> 0) where samples[k] != samples[k - 1]."""
        s = self.samples
        return [k for k in range(1, len(s)) if s[k] != s[k - 1]]

def _words(data):
    a = array.array("I")
    a.frombytes(bytes(data))
    if struct.pack("=I", 1) != struct.pack("<I", 1):
        a.byteswap()
    return a

class LAStreamParser:
    """Incremental parser of the LA stream: feed() bytes (any split), `capture` once complete.
    Data before a header magic (stale data, previous captures) is skipped."""
    def __init__(self, sys_clk_freq=c.DEFAULT_SYS_CLK_FREQ):
        self.sys_clk_freq = sys_clk_freq
        self._buffer      = b""
        self.header       = None
        self.samples      = array.array("I")
        self.overflow     = False
        self.done         = False
        self._block       = 0 # Remaining words of the current block.
        self._last        = False
        self.skipped      = 0 # Bytes skipped before the header.
        self.restarts     = 0 # Captures restarted (header received in a capture).

    @property
    def capture(self) -> Optional[LACapture]:
        if not self.done:
            return None
        return LACapture(self.header, self.samples, self.overflow, self.sys_clk_freq)

    def feed(self, data: bytes) -> bool:
        """Feed bytes, return True when the capture is complete."""
        if self.done:
            return True
        buf = self._buffer + bytes(data)
        pos = 0 # Parsed bytes of buf (buf only sliced once per feed: linear time on large feeds).
        try:
            while not self.done:
                if self.header is None:
                    magic = struct.pack("<I", c.LA_MAGIC)
                    i = buf.find(magic, pos)
                    while i > 0 and (i - pos) % 4:
                        i = buf.find(magic, i + 1)
                    if i < 0:
                        keep = (len(buf) - pos) & ~3
                        self.skipped += keep
                        pos          += keep
                        return False
                    self.skipped += i - pos
                    pos = i
                    if len(buf) - pos < 4*c.LA_HEADER_WORDS:
                        return False
                    self.header   = LAHeader.unpack(buf[pos:pos + 4*c.LA_HEADER_WORDS])
                    self.overflow = bool(self.header.flags & c.LA_HEADER_OVERFLOW)
                    pos += 4*c.LA_HEADER_WORDS
                elif self._block:
                    n = min(self._block, (len(buf) - pos)//4)
                    if n == 0:
                        return False
                    self.samples.extend(_words(buf[pos:pos + 4*n]))
                    pos         += 4*n
                    self._block -= n
                    if self._block == 0 and self._last:
                        self.done = True
                else:
                    if len(buf) - pos < 4:
                        return False
                    word = struct.unpack_from("<I", buf, pos)[0]
                    if word == c.LA_MAGIC:
                        # New capture (previous one aborted by a re-arm): restart on its header.
                        self.restarts += 1
                        self.header   = None
                        self.samples  = array.array("I")
                        self.overflow = False
                        continue
                    pos += 4
                    if (word >> 24) != c.LA_BLOCK_MAGIC:
                        raise ValueError("Bad LA block word 0x{:08x} after {} samples.".format(
                            word, len(self.samples)))
                    flags = (word >> 16) & 0xff
                    self._block = word & 0xffff
                    self._last  = bool(flags & c.LA_BLOCK_LAST)
                    if flags & c.LA_BLOCK_OVERFLOW:
                        self.overflow = True
                    if self._block == 0 and self._last:
                        self.done = True
            return True
        finally:
            self._buffer = buf[pos:]

def parse_stream(data: bytes, sys_clk_freq=c.DEFAULT_SYS_CLK_FREQ) -> LACapture:
    """Parse a complete LA capture stream."""
    parser = LAStreamParser(sys_clk_freq)
    if not parser.feed(data):
        raise ValueError("Incomplete LA stream ({} samples).".format(len(parser.samples)))
    return parser.capture

def pack_stream(header: LAHeader, samples: Sequence[int], overflow=False) -> bytes:
    """Build an LA stream (inverse of parse_stream, used by tests and fake devices)."""
    b = header.block_samples
    data = [header.pack()]
    i = 0
    while True:
        n    = min(b, len(samples) - i)
        last = (len(samples) - i) <= b
        flags = (c.LA_BLOCK_LAST if last else 0) | (c.LA_BLOCK_OVERFLOW if last and overflow else 0)
        data.append(struct.pack("<I", (c.LA_BLOCK_MAGIC << 24) | (flags << 16) | n))
        data.append(struct.pack("<{}I".format(n), *samples[i:i + n]))
        i += n
        if last:
            break
    return b"".join(data)

# Summary ------------------------------------------------------------------------------------------

def summarize(capture: LACapture, channels: Optional[Sequence[int]] = None, names=None,
    max_edges=8) -> dict:
    """Capture summary for agents: timing, and per channel levels, edges (counts, first edges in
    device seconds), frequency/duty cycle estimates."""
    channels = list(range(capture.channels)) if channels is None else list(channels)
    s        = capture.samples
    n        = len(s)
    period   = 1/capture.samplerate
    changes  = capture.changes()
    result   = {
        "samples"         : n,
        "samplerate_hz"   : capture.samplerate,
        "divider"         : capture.divider,
        "start_time_s"    : capture.time(0),
        "trigger_time_s"  : capture.trigger_timestamp/capture.sys_clk_freq,
        "trigger_offset"  : capture.trigger_offset,
        "duration_s"      : n*period,
        "start_timestamp" : capture.start_timestamp,
        "overflow"        : capture.overflow,
        "channels"        : {},
    }
    for ch in channels:
        name  = names[ch] if names and ch < len(names) else "ch{}".format(ch)
        bit   = 1 << ch
        edges = [k for k in changes if (s[k] ^ s[k - 1]) & bit]
        rising = [k for k in edges if s[k] & bit]
        high   = 0
        if n:
            level, last_k = s[0] & bit, 0
            for k in edges:
                if level:
                    high += k - last_k
                level, last_k = s[k] & bit, k
            if level:
                high += n - last_k
        d = {
            "initial"     : (s[0] >> ch) & 1 if n else None,
            "final"       : (s[-1] >> ch) & 1 if n else None,
            "edges"       : len(edges),
            "rising"      : len(rising),
            "falling"     : len(edges) - len(rising),
            "first_edges" : [{"time_s": capture.time(k), "level": (s[k] >> ch) & 1}
                for k in edges[:max_edges]],
            "duty"        : high/n if n else None,
        }
        if len(rising) >= 2:
            d["frequency_hz"] = (len(rising) - 1)/((rising[-1] - rising[0])*period)
        result["channels"][name] = d
    return result

# VCD ----------------------------------------------------------------------------------------------

def _vcd_id(i):
    chars = [chr(c_) for c_ in range(33, 127)]
    r = ""
    while True:
        r += chars[i % len(chars)]
        i //= len(chars)
        if i == 0:
            return r

def write_vcd(capture: LACapture, f, channels=None, names=None):
    """Write a VCD (1ns timescale, times = device time: shared timebase)."""
    if isinstance(f, str):
        with open(f, "w") as fd:
            return write_vcd(capture, fd, channels, names)
    channels = list(range(capture.channels)) if channels is None else list(channels)
    names    = names or ["ch{}".format(ch) for ch in range(capture.channels)]
    ids      = {ch: _vcd_id(i) for i, ch in enumerate(channels)}
    ns       = lambda k: int(round(capture.timestamp(k)*1e9/capture.sys_clk_freq))
    f.write("$comment AgentLab logic analyzer: start_timestamp {} ticks, trigger_timestamp {} "
        "ticks, sys_clk {} Hz, divider {}, trigger_offset {} $end\n".format(
        capture.start_timestamp, capture.trigger_timestamp, int(capture.sys_clk_freq),
        capture.divider, capture.trigger_offset))
    f.write("$timescale 1ns $end\n")
    f.write("$scope module agentlab $end\n")
    for ch in channels:
        f.write("$var wire 1 {} {} $end\n".format(ids[ch], names[ch]))
    f.write("$upscope $end\n$enddefinitions $end\n")
    s = capture.samples
    if len(s) == 0:
        return
    f.write("#{}\n$dumpvars\n".format(ns(0)))
    for ch in channels:
        f.write("{}{}\n".format((s[0] >> ch) & 1, ids[ch]))
    f.write("$end\n")
    mask = sum(1 << ch for ch in channels)
    for k in capture.changes():
        diff = (s[k] ^ s[k - 1]) & mask
        if diff:
            f.write("#{}\n".format(ns(k)))
            for ch in channels:
                if (diff >> ch) & 1:
                    f.write("{}{}\n".format((s[k] >> ch) & 1, ids[ch]))
    f.write("#{}\n".format(ns(len(s))))

def read_vcd(f) -> dict:
    """Minimal VCD reader (1-bit vars): {"timescale", "vars": {name: [(time, value)]}}."""
    if isinstance(f, str):
        with open(f) as fd:
            return read_vcd(fd)
    text   = f.read()
    ids    = {}
    vars_  = {}
    m = re.search(r"\$timescale\s+(\S+)\s+\$end", text)
    for m_ in re.finditer(r"\$var\s+wire\s+1\s+(\S+)\s+(\S+)\s+\$end", text):
        ids[m_.group(1)] = m_.group(2)
        vars_[m_.group(2)] = []
    body = text.split("$enddefinitions $end", 1)[1]
    t = 0
    for tok in body.split():
        if tok.startswith("#"):
            t = int(tok[1:])
        elif tok[0] in "01" and tok[1:] in ids:
            vars_[ids[tok[1:]]].append((t, int(tok[0])))
    return {"timescale": m.group(1) if m else None, "vars": vars_, "end": t}

# Sigrok (srzip) -----------------------------------------------------------------------------------

def write_sr(capture: LACapture, f, channels=None, names=None, chunk_samples=1 << 20):
    """Write a sigrok session file (srzip: version, metadata, logic-1-N chunks; PulseView,
    sigrok-cli). Selected channels are packed (unitsize bytes per sample); the device timing
    (start/trigger timestamps) is stored in an agentlab.json member (ignored by sigrok)."""
    channels = list(range(capture.channels)) if channels is None else list(channels)
    names    = names or ["ch{}".format(ch) for ch in range(capture.channels)]
    unitsize = max(1, (len(channels) + 7)//8)
    meta = io.StringIO()
    meta.write("[global]\nsigrok version=0.5.2\n\n[device 1]\ncapturefile=logic-1\n")
    meta.write("total probes={}\nsamplerate={}\ntotal analog=0\n".format(len(channels),
        _sr_rate(capture.samplerate)))
    for i, ch in enumerate(channels):
        meta.write("probe{}={}\n".format(i + 1, names[ch]))
    meta.write("unitsize={}\n".format(unitsize))
    identity = channels == list(range(len(channels)))
    s = capture.samples
    with zipfile.ZipFile(f, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("version", "2")
        z.writestr("metadata", meta.getvalue())
        z.writestr("agentlab.json", json.dumps({
            "sys_clk_freq"      : capture.sys_clk_freq,
            "divider"           : capture.divider,
            "start_timestamp"   : capture.start_timestamp,
            "trigger_timestamp" : capture.trigger_timestamp,
            "trigger_offset"    : capture.trigger_offset,
            "overflow"          : capture.overflow,
            "channels"          : channels,
        }, indent=2))
        for n, i in enumerate(range(0, max(len(s), 1), chunk_samples)):
            chunk = s[i:i + chunk_samples]
            if not identity:
                chunk = [sum(((v >> ch) & 1) << j for j, ch in enumerate(channels)) for v in chunk]
            mask = (1 << (8*unitsize)) - 1
            data = b"".join((v & mask).to_bytes(unitsize, "little") for v in chunk)
            z.writestr("logic-1-{}".format(n + 1), data)

def _sr_rate(rate):
    """Samplerate as written by sigrok (integer Hz, largest exact unit: "125 MHz", "41666667 Hz")."""
    rate = int(round(rate))
    for unit, scale in [("GHz", 10**9), ("MHz", 10**6), ("kHz", 10**3)]:
        if rate >= scale and rate % scale == 0:
            return "{} {}".format(rate//scale, unit)
    return "{} Hz".format(rate)

def read_sr(f) -> dict:
    """Read a sigrok srzip written by write_sr: {"samplerate", "probes", "unitsize", "samples",
    "agentlab"}."""
    with zipfile.ZipFile(f) as z:
        cp = configparser.ConfigParser()
        cp.read_string(z.read("metadata").decode())
        dev      = cp["device 1"]
        unitsize = int(dev["unitsize"])
        nprobes  = int(dev["total probes"])
        rate     = dev["samplerate"].split()
        samplerate = float(rate[0])*{"Hz": 1, "kHz": 1e3, "MHz": 1e6, "GHz": 1e9}[rate[1]]
        chunks = sorted((n for n in z.namelist() if n.startswith(dev["capturefile"] + "-")),
            key=lambda n: int(n.rsplit("-", 1)[1]))
        data = b"".join(z.read(n) for n in chunks)
        samples = [int.from_bytes(data[i:i + unitsize], "little")
            for i in range(0, len(data), unitsize)]
        extra = json.loads(z.read("agentlab.json")) if "agentlab.json" in z.namelist() else None
        return {
            "version"    : z.read("version").decode(),
            "samplerate" : samplerate,
            "probes"     : [dev["probe{}".format(i + 1)] for i in range(nprobes)],
            "unitsize"   : unitsize,
            "samples"    : samples,
            "agentlab"   : extra,
        }

def write_capture(capture: LACapture, path, channels=None, names=None):
    """Write a capture to path (.vcd, .sr, or .json: summary)."""
    if path.endswith(".vcd"):
        write_vcd(capture, path, channels, names)
    elif path.endswith(".sr"):
        write_sr(capture, path, channels, names)
    elif path.endswith(".json"):
        with open(path, "w") as f:
            json.dump(summarize(capture, channels, names), f, indent=2)
    else:
        raise ValueError("Unknown capture file extension (.vcd, .sr or .json): {}.".format(path))
