#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Video Class camera (UVC 1.1, Bulk streaming, YUY2 test pattern).

A USB3 device exposing a UVC camera (/dev/videoX, no driver needed) streaming an uncompressed YUY2
test pattern (color bars, moving marker, moving luma band) in several modes selected by the host
(PROBE/COMMIT), viewable with VLC/ffplay/GStreamer:

    ffplay -f v4l2 -input_format yuyv422 -video_size 1280x720 /dev/video0

- Descriptors: IAD, VideoControl (Camera Input Terminal -> Streaming Output Terminal), VideoStreaming
  (Uncompressed YUY2 format, one frame descriptor per mode, Bulk IN endpoint).
- USB3UVCControl: VS PROBE/COMMIT requests (EP0 request handler).
- USB3UVCStream  : test pattern generator and payloads (12-byte payload header: FID/EOF/PTS/SCR).
- USB3UVC        : complete device (USB3DeviceCore + control + stream).

Bulk streaming (UVC 1.1 2.4.3.2.2): payloads of dwMaxPayloadTransferSize bytes (header + data), the
last payload of a frame ends with a short packet (checked at build time for each mode).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN
from usb3_core.descriptors import *

# Constants ----------------------------------------------------------------------------------------

CC_VIDEO                       = 0x0e
SC_VIDEOCONTROL                = 0x01
SC_VIDEOSTREAMING              = 0x02
SC_VIDEO_INTERFACE_COLLECTION  = 0x03

VC_HEADER                      = 0x01
VC_INPUT_TERMINAL              = 0x02
VC_OUTPUT_TERMINAL             = 0x03
VS_INPUT_HEADER                = 0x01
VS_FORMAT_UNCOMPRESSED         = 0x04
VS_FRAME_UNCOMPRESSED          = 0x05
VS_COLORFORMAT                 = 0x0d

VS_PROBE_CONTROL               = 0x01
VS_COMMIT_CONTROL              = 0x02

UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_MIN = 0x82
UVC_GET_MAX = 0x83
UVC_GET_LEN = 0x85
UVC_GET_INF = 0x86
UVC_GET_DEF = 0x87

YUY2_GUID = [0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71]

UVC_HEADER_SIZE  = 12   # Payload header: bHeaderLength, bmHeaderInfo, PTS (4), SCR (6).
UVC_PROBE_SIZE   = 34   # UVC 1.1 Video Probe and Commit Controls.
UVC_PAYLOAD_SIZE = 32768

# Default modes: (width, height, fps), YUY2 (16 bits/pixel).
UVC_MODES = [(640, 480, 60), (1280, 720, 60), (1920, 1080, 30), (1920, 1080, 60)]

# Color bars (BT.601 YUV): white, yellow, cyan, green, magenta, red, blue, black.
UVC_BARS = [(235, 128, 128), (210, 16, 146), (170, 166, 16), (145, 54, 34), (106, 202, 222),
    (81, 90, 240), (41, 240, 110), (16, 128, 128)]

def frame_size(width, height):
    return width*height*2

def frame_interval(fps):
    return 10_000_000//fps # 100ns units.

def check_modes(modes, payload_size):
    """The last payload of each frame must end with a short packet (bulk payload delimitation)."""
    data_size = payload_size - UVC_HEADER_SIZE
    for width, height, fps in modes:
        assert (width % 16) == 0, "width must be a multiple of 16 (color bars, 2 pixels per word)."
        assert (height % 8) == 0, "height must be a multiple of 8."
        last = UVC_HEADER_SIZE + (frame_size(width, height) % data_size or data_size)
        assert (last % 1024) != 0 or last == payload_size, \
            f"{width}x{height}: last payload of {last} bytes not ended by a short packet."

# Descriptors --------------------------------------------------------------------------------------

def uvc_frames(modes):
    """Frame descriptors: resolutions (in order of first appearance) with their frame rates."""
    frames = {}
    for width, height, fps in modes:
        frames.setdefault((width, height), []).append(fps)
    return list(frames.items())

def uvc_frame_index(modes, mode):
    """Frame descriptor index (1-N) of a mode (0-N)."""
    width, height, _ = modes[mode]
    return [size for size, _ in uvc_frames(modes)].index((width, height)) + 1

def uvc_probe(modes, mode, payload_size, clk_freq):
    """Video Probe and Commit Controls (UVC 1.1, 34 bytes) of a mode (0-N)."""
    width, height, fps = modes[mode]
    return (le16(0) + [1, uvc_frame_index(modes, mode)] + le32(frame_interval(fps)) + le16(0)*4 + le16(0) +
        le32(frame_size(width, height)) + le32(payload_size) + le32(clk_freq) + [0x03, 0, 0, 0])

def uvc_interfaces(modes, vc_interface=0, ept=1, max_burst=16, clk_freq=125_000_000):
    """UVC function: IAD, VideoControl and VideoStreaming interfaces (VS: vc_interface + 1)."""
    vs_interface = vc_interface + 1
    iad = [8, DESC_IAD, vc_interface, 2, CC_VIDEO, SC_VIDEO_INTERFACE_COLLECTION, 0x00, 0]

    # VideoControl: Header, Camera Input Terminal (1), Streaming Output Terminal (2).
    vc_it  = [18, DESC_CS_INTERFACE, VC_INPUT_TERMINAL, 1] + le16(0x0201) + [0, 0] + le16(0)*3 + [3, 0, 0, 0]
    vc_ot  = [9, DESC_CS_INTERFACE, VC_OUTPUT_TERMINAL, 2] + le16(0x0101) + [0, 1, 0]
    vc_hdr = [13, DESC_CS_INTERFACE, VC_HEADER] + le16(0x0110) + le16(13 + len(vc_it) + len(vc_ot)) + \
        le32(clk_freq) + [1, vs_interface]
    vc = USB3InterfaceDescriptor(vc_interface, cls=CC_VIDEO, subclass=SC_VIDEOCONTROL,
        extra=vc_hdr + vc_it + vc_ot)

    # VideoStreaming: Input Header, Uncompressed YUY2 Format, Frames, Color Matching.
    vs_fmt    = [27, DESC_CS_INTERFACE, VS_FORMAT_UNCOMPRESSED, 1, len(uvc_frames(modes))] + YUY2_GUID + \
        [16, 1, 0, 0, 0, 0]
    # One frame descriptor per resolution, with its discrete frame intervals.
    vs_frames = []
    for i, ((width, height), rates) in enumerate(uvc_frames(modes), start=1):
        size = frame_size(width, height)
        vs_frames += [26 + 4*len(rates), DESC_CS_INTERFACE, VS_FRAME_UNCOMPRESSED, i, 0] + le16(width) + \
            le16(height) + le32(size*min(rates)*8) + le32(size*max(rates)*8) + le32(size) + \
            le32(frame_interval(rates[0])) + [len(rates)] + sum([le32(frame_interval(f)) for f in rates], [])
    vs_color  = [6, DESC_CS_INTERFACE, VS_COLORFORMAT, 1, 1, 4]
    vs_hdr    = [14, DESC_CS_INTERFACE, VS_INPUT_HEADER, 1] + \
        le16(14 + len(vs_fmt) + len(vs_frames) + len(vs_color)) + [0x80 | ept, 0, 2, 0, 0, 0, 1, 0]
    vs = USB3InterfaceDescriptor(vs_interface, cls=CC_VIDEO, subclass=SC_VIDEOSTREAMING,
        extra=vs_hdr + vs_fmt + vs_frames + vs_color,
        endpoints=[USB3EndpointDescriptor(ept, direction=1, max_burst=max_burst)])
    return [iad, vc, vs]

def uvc_descriptors(modes=UVC_MODES, ept=1, max_burst=16, clk_freq=125_000_000, **kwargs):
    kwargs.setdefault("product",      "USB3-PIPE Camera")
    kwargs.setdefault("device_class", DEVICE_CLASS_IAD)
    return USB3Descriptors(interfaces=uvc_interfaces(modes, ept=ept, max_burst=max_burst,
        clk_freq=clk_freq), **kwargs)

# UVC Control (PROBE/COMMIT) -----------------------------------------------------------------------

class USB3UVCControl(LiteXModule):
    """VideoStreaming PROBE/COMMIT controls on the EP0 request handler (other requests stalled).

    The mode (resolution and frame rate) is selected by bFrameIndex/dwFrameInterval of SET_CUR (frame
    index only if the interval does not match, first mode otherwise).
    commit (pulse)/mode: committed mode (0 to N-1), stream start.
    """
    def __init__(self, ep0, modes, payload_size, clk_freq, vs_interface=1):
        self.commit = Signal()                       # o
        self.mode   = Signal(max=max(len(modes), 2)) # o

        # # #

        n = len(modes)

        # Responses: N probes (9 words each), GET_LEN (34), GET_INFO (GET/SET supported).
        words = []
        for m in range(n):
            probe  = uvc_probe(modes, m, payload_size, clk_freq) + [0, 0]
            words += [int.from_bytes(bytes(probe[4*j:4*j+4]), "little") for j in range(9)]
        probes = Array(Constant(w, 32) for w in words)

        probe_mode  = Signal(max=max(n, 2))
        commit_mode = Signal(max=max(n, 2))

        # Request decoding.
        vs       = Signal()
        selector = Signal(8)
        valid    = Signal()
        self.comb += [
            selector.eq(ep0.req_value[8:16]),
            vs.eq((ep0.req_index[0:8] == vs_interface) &
                ((selector == VS_PROBE_CONTROL) | (selector == VS_COMMIT_CONTROL))),
            valid.eq(vs & ((ep0.req_request == UVC_SET_CUR) | (ep0.req_request == UVC_GET_CUR) |
                (ep0.req_request == UVC_GET_MIN) | (ep0.req_request == UVC_GET_MAX) |
                (ep0.req_request == UVC_GET_DEF) | (ep0.req_request == UVC_GET_LEN) |
                (ep0.req_request == UVC_GET_INF))),
            ep0.req_stall.eq(ep0.req_valid & ~valid),
        ]

        # IN responses.
        mode   = Signal(max=max(n, 2)) # Probe returned.
        count  = Signal(4)
        active = Signal()
        kind   = Signal(2)             # 0: probe, 1: GET_LEN, 2: GET_INFO.
        self.comb += Case(ep0.req_request, {
            UVC_GET_CUR : mode.eq(Mux(selector == VS_COMMIT_CONTROL, commit_mode, probe_mode)),
            UVC_GET_MAX : mode.eq(n - 1),
            "default"   : mode.eq(0),
        })
        self.sync += [
            If(ep0.req_setup,
                count.eq(0),
                active.eq(valid & ep0.req_type[7]),
                kind.eq(Mux(ep0.req_request == UVC_GET_LEN, 1, Mux(ep0.req_request == UVC_GET_INF, 2, 0))),
            ).Elif(ep0.req_in.valid & ep0.req_in.ready,
                count.eq(count + 1),
                If(ep0.req_in.last, active.eq(0)),
            )
        ]
        self.comb += [
            ep0.req_in.valid.eq(active),
            Case(kind, {
                0: [
                    ep0.req_in.data.eq(probes[mode*9 + count]),
                    ep0.req_in.last.eq(count == 8),
                    ep0.req_in.last_be.eq(0b0010), # 34 bytes.
                ],
                1: [
                    ep0.req_in.data.eq(UVC_PROBE_SIZE),
                    ep0.req_in.last.eq(1),
                    ep0.req_in.last_be.eq(0b0010), # 2 bytes.
                ],
                2: [
                    ep0.req_in.data.eq(0x03),
                    ep0.req_in.last.eq(1),
                    ep0.req_in.last_be.eq(0b0001), # 1 byte.
                ],
            }),
        ]

        # SET_CUR: bFrameIndex (byte 3)/dwFrameInterval (bytes 4-7) of the OUT data, applied at the
        # Status stage.
        word         = Signal(4)
        set_frame    = Signal(8)
        set_interval = Signal(32)
        self.comb += ep0.req_out.ready.eq(1)
        self.sync += [
            If(ep0.req_setup, word.eq(0)),
            If(ep0.req_out.valid,
                word.eq(word + 1),
                If(word == 0, set_frame.eq(ep0.req_out.data[24:32])),
                If(word == 1, set_interval.eq(ep0.req_out.data)),
            ),
        ]
        set_mode = Signal(max=max(n, 2))
        cases    = None
        for m in reversed(range(n)):
            # Frame index match first (lower priority), then frame index + interval match.
            match  = (set_frame == uvc_frame_index(modes, m))
            cases  = If(match, set_mode.eq(m)) if cases is None else If(match, set_mode.eq(m)).Else(cases)
        for m in reversed(range(n)):
            match  = (set_frame == uvc_frame_index(modes, m)) & (set_interval == frame_interval(modes[m][2]))
            cases  = If(match, set_mode.eq(m)).Else(cases)
        self.comb += [set_mode.eq(0), cases]
        apply = Signal()
        self.comb += apply.eq(ep0.req_done & vs & (ep0.req_request == UVC_SET_CUR) & ~ep0.req_type[7])
        self.sync += [
            If(apply & (selector == VS_PROBE_CONTROL),
                probe_mode.eq(set_mode)
            ),
            If(apply & (selector == VS_COMMIT_CONTROL),
                commit_mode.eq(set_mode)
            ),
        ]
        self.sync += self.commit.eq(apply & (selector == VS_COMMIT_CONTROL))
        self.comb += self.mode.eq(commit_mode)

# UVC Stream (pattern + payloads) ------------------------------------------------------------------

class USB3UVCStream(LiteXModule):
    """YUY2 test pattern frames in UVC Bulk payloads.

    start (pulse): start streaming in mode (stream restarted), stop (pulse): stop streaming.
    source: payloads (to the Bulk IN endpoint, last: end of payload).
    """
    def __init__(self, modes, clk_freq, payload_size=UVC_PAYLOAD_SIZE):
        check_modes(modes, payload_size)
        self.start  = Signal()                       # i
        self.stop   = Signal()                       # i
        self.mode   = Signal(max=max(len(modes), 2)) # i
        self.source = source = stream.Endpoint([("data", 32), ("last_be", 4)])
        self.frames = Signal(32)                     # o: Frames sent.

        # # #

        # Mode parameters (loaded on start).
        def param(values):
            return Array(Constant(v, bits_for(max(values))) for v in values)
        p_width    = param([w for w, h, f in modes])
        p_height   = param([h for w, h, f in modes])
        p_bar      = param([w//8 for w, h, f in modes])
        p_band     = param([h - h//8 for w, h, f in modes])
        p_interval = param([clk_freq//f for w, h, f in modes])
        width    = Signal(16)
        height   = Signal(16)
        bar_w    = Signal(16)
        band_y   = Signal(16)
        interval = Signal(32)

        payload_words = (payload_size - UVC_HEADER_SIZE)//4

        # Time (PTS/SCR: sys clock cycles).
        stc = Signal(32)
        self.sync += stc.eq(stc + 1)

        # Frame timer.
        timer   = Signal(32)
        tick    = Signal()
        self.comb += tick.eq(timer == 0)
        self.sync += If(~tick, timer.eq(timer - 1))

        # Pattern state.
        x       = Signal(16) # First pixel of the word.
        y       = Signal(16)
        bar     = Signal(3)
        bar_x   = Signal(16) # Position in the bar.
        marker  = Signal(16)
        fid     = Signal()
        pts     = Signal(32)
        p_words = Signal(32) # Data words left in the payload.
        f_words = Signal(32) # Data words left in the frame.
        hdr     = Signal(2)

        # Pixels (word: Y0 U Y1 V).
        bars   = Array(Constant(yy | (u << 8) | (v << 16), 24) for yy, u, v in UVC_BARS)
        color  = Signal(24)
        y0     = Signal(8)
        y1     = Signal(8)
        u      = Signal(8)
        v      = Signal(8)
        band   = Signal()
        ramp   = Signal(8)
        self.comb += [
            color.eq(bars[bar]),
            band.eq(y >= band_y),
            ramp.eq(x + 4*self.frames),
            If(band,
                y0.eq(ramp), y1.eq(ramp + 1), u.eq(128), v.eq(128),
            ).Else(
                y0.eq(Mux(x == marker,       235, color[0:8])),
                y1.eq(Mux((x + 1) == marker, 235, color[0:8])),
                u.eq(color[8:16]),
                v.eq(color[16:24]),
            ),
        ]

        # Payload header (bmHeaderInfo: EOH, SCR, PTS, EOF on the last payload of the frame, FID).
        eof    = Signal()
        info   = Signal(8)
        self.comb += [
            eof.eq(f_words <= payload_words),
            info.eq(0x80 | 0x08 | 0x04 | (eof << 1) | fid),
        ]

        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(self.start | self.stop)
        self.sync += If(self.start,
            width.eq(p_width[self.mode]),
            height.eq(p_height[self.mode]),
            bar_w.eq(p_bar[self.mode]),
            band_y.eq(p_band[self.mode]),
            interval.eq(p_interval[self.mode]),
            timer.eq(0),
        )
        # (Restarted on start: the FSM is reset on the start pulse, then leaves IDLE.)
        started = Signal()
        self.sync += started.eq(self.start)
        fsm.act("IDLE",
            If(started, NextState("WAIT"))
        )
        fsm.act("WAIT",
            If(tick,
                NextValue(timer, interval - 1),
                NextValue(x, 0),
                NextValue(y, 0),
                NextValue(bar, 0),
                NextValue(bar_x, 0),
                NextValue(pts, stc),
                NextValue(f_words, (width*height) >> 1),
                NextValue(hdr, 0),
                NextState("HEADER")
            )
        )
        fsm.act("HEADER",
            source.valid.eq(1),
            Case(hdr, {
                0: source.data.eq(Cat(C(UVC_HEADER_SIZE, 8), info, pts[0:16])),
                1: source.data.eq(Cat(pts[16:32], stc[0:16])),
                2: source.data.eq(Cat(stc[16:32], C(0, 16))),
            }),
            If(source.ready,
                NextValue(hdr, hdr + 1),
                If(hdr == 2,
                    NextValue(p_words, payload_words),
                    NextState("DATA")
                )
            )
        )
        fsm.act("DATA",
            source.valid.eq(1),
            source.data.eq(Cat(y0, u, y1, v)),
            source.last.eq((p_words == 1) | (f_words == 1)),
            If(source.ready,
                NextValue(p_words, p_words - 1),
                NextValue(f_words, f_words - 1),
                # Pattern position.
                NextValue(x, x + 2),
                NextValue(bar_x, bar_x + 2),
                If(bar_x + 2 == bar_w,
                    NextValue(bar_x, 0),
                    NextValue(bar, bar + 1),
                ),
                If(x + 2 == width,
                    NextValue(x, 0),
                    NextValue(bar, 0),
                    NextValue(bar_x, 0),
                    NextValue(y, y + 1),
                ),
                If(f_words == 1,
                    # End of frame.
                    NextValue(fid, ~fid),
                    NextValue(self.frames, self.frames + 1),
                    NextValue(marker, Mux(marker + 4 >= width, 0, marker + 4)),
                    NextState("WAIT")
                ).Elif(p_words == 1,
                    # End of payload.
                    NextValue(hdr, 0),
                    NextState("HEADER")
                )
            )
        )

# USB3 UVC Function -------------------------------------------------------------------------------

class USB3UVCFunction(LiteXModule):
    """UVC camera function of a USB3 device (EP0 with request handler, Bulk IN video endpoint).

    The stream is started on COMMIT, stopped by CLEAR_FEATURE(ENDPOINT_HALT) on the video endpoint
    (uvcvideo bulk stream off) or a link reset, the buffered packets are then flushed.
    """
    def __init__(self, ep0, bulk_in, link_reset, clk_freq, modes=UVC_MODES, payload_size=UVC_PAYLOAD_SIZE,
        vs_interface=1):
        self.control = control = USB3UVCControl(ep0, modes, payload_size, clk_freq, vs_interface)
        self.stream  = stream  = USB3UVCStream(modes, clk_freq, payload_size)

        # # #

        stop = Signal()
        self.comb += [
            stop.eq((ep0.clear_halt & (ep0.clear_ept == bulk_in.ept)) | link_reset),
            stream.start.eq(control.commit),
            stream.stop.eq(stop),
            stream.mode.eq(control.mode),
            bulk_in.flush.eq(control.commit | stop),
            stream.source.connect(bulk_in.sink),
        ]

# USB3 UVC -----------------------------------------------------------------------------------------

class USB3UVC(LiteXModule):
    """USB3 UVC camera device (test pattern, see USB3UVCFunction)."""
    def __init__(self, phy, sys_clk_freq, modes=UVC_MODES, payload_size=UVC_PAYLOAD_SIZE, max_burst=16,
        **kwargs):
        clk_freq = int(sys_clk_freq)
        descriptors = uvc_descriptors(modes, ept=1, max_burst=max_burst, clk_freq=clk_freq, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors,
            endpoints        = [USB3BulkIN(ept=1, max_burst=max_burst)],
            with_ep0_handler = True)
        self.function = USB3UVCFunction(core.ep0, core.endpoints[0], core.link.reset, clk_freq,
            modes, payload_size)
