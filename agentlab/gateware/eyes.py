#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Eyes (phase 4): gateware text-mode renderer + logic waveform lanes, streamed as YUY2
frames to a UVC function (no framebuffer, no CPU, no DRAM).

Screen (1280x720 by default): character grid of 8x16 cells (160x45), each cell a 16-bit word
(char code [7:0], foreground color [11:8], background color [15:12], 16 colors palette), written
by the host through a Wishbone region (one cell per 32-bit word, row-major, burst writes over
UARTBone), and waveform lanes drawn over the grid from a sample window in BRAM (live sampling of the
DUT pins, oldest sample on the left, 2 pixels per sample, one 16-pixel character row per lane).

Pixels are generated on the fly in raster order (2 pixels/YUY2 word per clock cycle) when the UVC
stream asks for a frame (frame_start), through a small FIFO: deterministic, ~2 BRAM36 for the text
grid, 1 for the font, 1 for the samples.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream
from litex.soc.interconnect     import wishbone

from agentlab.gateware.eyes_font import *

# Constants ----------------------------------------------------------------------------------------

EYES_WIDTH  = 1280
EYES_HEIGHT = 720
EYES_MODES  = [(EYES_WIDTH, EYES_HEIGHT, 10), (EYES_WIDTH, EYES_HEIGHT, 30)] # (w, h, fps).

EYES_WAVE_DEPTH = 512 # Samples (2 pixels each: 1024 pixels).
EYES_WAVE_WIDTH = 32  # Sample bits (lanes sources).
EYES_WAVE_HIGH  = 3   # Lane line of a high level (in the 16 lines of a character row).
EYES_WAVE_LOW   = 12  # Lane line of a low level.

# Palette (ANSI order, RGB): index = color code of the cells.
EYES_PALETTE = [
    (  0,   0,   0), # 0  black.
    (205,   0,   0), # 1  red.
    (  0, 205,   0), # 2  green.
    (205, 205,   0), # 3  yellow.
    ( 30,  80, 220), # 4  blue.
    (205,   0, 205), # 5  magenta.
    (  0, 205, 205), # 6  cyan.
    (229, 229, 229), # 7  white.
    (110, 110, 110), # 8  gray.
    (255,  70,  70), # 9  bright red.
    ( 70, 255,  70), # 10 bright green.
    (255, 255,  70), # 11 bright yellow.
    (100, 140, 255), # 12 bright blue.
    (255,  90, 255), # 13 bright magenta.
    ( 70, 255, 255), # 14 bright cyan.
    (255, 255, 255), # 15 bright white.
]

def rgb_to_yuv(r, g, b):
    """BT.601 limited range RGB to YUV (integers)."""
    y = 16  + round(( 65.481*r + 128.553*g +  24.966*b)/255)
    u = 128 + round((-37.797*r -  74.203*g + 112.000*b)/255)
    v = 128 + round((112.000*r -  93.786*g -  18.214*b)/255)
    return y, u, v

EYES_PALETTE_YUV = [rgb_to_yuv(*rgb) for rgb in EYES_PALETTE]

# Cells / Boot Screen ------------------------------------------------------------------------------

def eyes_cell(char, fg=7, bg=0):
    """Text grid cell (16-bit): char (str or font code), foreground/background color codes."""
    code = eyes_char_code(char) if isinstance(char, str) else char
    return (code & 0xff) | ((fg & 0xf) << 8) | ((bg & 0xf) << 12)

def eyes_boot_screen(cols, rows):
    """Initial text grid (until the host writes it)."""
    cells = [eyes_cell(" ")]*(cols*rows)
    def text(col, row, s, fg=7, bg=0):
        for i, ch in enumerate(s[:max(cols - col, 0)]):
            if row < rows:
                cells[row*cols + col + i] = eyes_cell(ch, fg, bg)
    text(0, 0, " AgentLab Eyes".ljust(cols), fg=15, bg=4)
    text(2, 2, "Waiting for the host: agentlab display status --follow", fg=11)
    return cells

# Reference Renderer -------------------------------------------------------------------------------

def eyes_render(width, height, cells, samples=None, wave_row=0, wave_lanes=0, wave_column=0,
    wave_first=0, wave_color=10, wave_base=0):
    """Python reference of the gateware renderer: YUY2 frame (bytes) of the text grid (cells) with
    the waveform lanes of samples (wave_base: index of the leftmost sample)."""
    font  = eyes_font()
    cols  = width//EYES_FONT_WIDTH
    depth = len(samples) if samples else 1
    x0    = wave_column*EYES_FONT_WIDTH
    frame = bytearray()
    for y in range(height):
        row, yy = y//EYES_FONT_HEIGHT, y % EYES_FONT_HEIGHT
        lane_row = (wave_row <= row < wave_row + wave_lanes) and samples
        for x in range(0, width, 2):
            cell   = cells[row*cols + x//EYES_FONT_WIDTH]
            glyph  = font[(cell & 0xff)*EYES_FONT_HEIGHT + yy]
            colors = []
            for p in range(2):
                bit = (glyph >> (7 - (x + p) % EYES_FONT_WIDTH)) & 1
                colors.append((cell >> 8) & 0xf if bit else (cell >> 12) & 0xf)
            if lane_row and x0 <= x < x0 + 2*depth:
                s     = (x - x0)//2
                lane  = (wave_first + row - wave_row) % EYES_WAVE_WIDTH
                level = (samples[(wave_base + s) % depth] >> lane) & 1
                prev  = (samples[(wave_base + s - 1) % depth] >> lane) & 1
                line  = (level and yy == EYES_WAVE_HIGH) or (not level and yy == EYES_WAVE_LOW)
                edge  = (s != 0) and (level != prev) and (EYES_WAVE_HIGH <= yy <= EYES_WAVE_LOW)
                if line or edge:
                    colors[0] = wave_color
                if line:
                    colors[1] = wave_color
            (y0, u0, v0), (y1, u1, v1) = [EYES_PALETTE_YUV[c] for c in colors]
            frame += bytes([y0, (u0 + u1)//2, y1, (v0 + v1)//2])
    return bytes(frame)

# AgentLab Eyes ------------------------------------------------------------------------------------

class AgentLabEyes(LiteXModule):
    """Text-mode renderer with waveform lanes (see module docstring).

    frame_start (i, pulse): restart the frame (first pixel), from the UVC stream.
    source (o): YUY2 words (Y0 U Y1 V), width*height/2 per frame (to the UVC function sink).
    samples (i): waveform sample (live DUT pins, bit n: lane source n).
    sample_we (o): samples written to the window (pulse: samples of this cycle).
    bus: Wishbone slave, text grid (cell of (col, row) at word row*cols + col, 16 LSBs).
    """
    def __init__(self, width=EYES_WIDTH, height=EYES_HEIGHT, wave_depth=EYES_WAVE_DEPTH,
        wave_width=EYES_WAVE_WIDTH, wave_lanes=16, wave_row=29, wave_column=32, wave_divider=1249,
        fifo_depth=16):
        fw, fh = EYES_FONT_WIDTH, EYES_FONT_HEIGHT
        assert (width % fw) == 0 and (height % fh) == 0
        assert wave_depth & (wave_depth - 1) == 0, "wave_depth must be a power of 2."
        assert wave_width <= 32
        self.width       = width
        self.height      = height
        self.cols = cols = width//fw
        self.rows = rows = height//fh
        self.wave_depth  = wave_depth
        self.wave_width  = wave_width

        self.frame_start = Signal()                          # i
        self.source      = stream.Endpoint([("data", 32)])   # o
        self.samples     = Signal(wave_width)                # i
        self.sample_we   = Signal()                          # o
        self.bus         = bus = wishbone.Interface(data_width=32, adr_width=30)

        self._wave_row     = CSRStorage(8,  reset=wave_row,     description="Waveform: first character row of the lanes.")
        self._wave_lanes   = CSRStorage(8,  reset=wave_lanes,   description="Waveform: number of lanes (character rows, 0: none).")
        self._wave_column  = CSRStorage(8,  reset=wave_column,  description="Waveform: first character column (x0 = 8*column), 2 pixels/sample.")
        self._wave_first   = CSRStorage(8,  reset=0,            description="Waveform: sample bit of the first lane.")
        self._wave_color   = CSRStorage(4,  reset=10,           description="Waveform: lanes color (palette index).")
        self._wave_divider = CSRStorage(32, reset=wave_divider, description="Waveform: sample period (divider + 1 sys_clk cycles).")
        self._wave_control = CSRStorage(fields=[
            CSRField("run",    size=1, reset=1, description="Continuous sampling (roll mode)."),
            CSRField("single", size=1, pulse=True, description="Capture one window (wave_depth samples)."),
        ])
        self._wave_status  = CSRStatus(fields=[
            CSRField("running", size=1, description="Sampling (run or single capture ongoing)."),
        ])
        self._frames       = CSRStatus(32, description="Frames rendered.")

        # # #

        # Memories ---------------------------------------------------------------------------------
        text_mem  = Memory(16, cols*rows, init=eyes_boot_screen(cols, rows))
        text_bus  = text_mem.get_port(write_capable=True)
        text_rd   = text_mem.get_port()
        font_mem  = Memory(8, 256*fh, init=eyes_font())
        font_rd   = font_mem.get_port()
        wave_mem  = Memory(wave_width, wave_depth, init=[0]*wave_depth)
        wave_wr   = wave_mem.get_port(write_capable=True)
        wave_rd   = wave_mem.get_port()
        self.specials += text_mem, text_bus, text_rd, font_mem, font_rd, wave_mem, wave_wr, wave_rd

        # Text grid Wishbone slave (one cell per word, single cycle writes, reads with 1 cycle
        # latency).
        self.comb += [
            text_bus.adr.eq(bus.adr),
            text_bus.dat_w.eq(bus.dat_w[:16]),
            text_bus.we.eq(bus.cyc & bus.stb & bus.we & ~bus.ack & (bus.adr < cols*rows)),
            bus.dat_r.eq(text_bus.dat_r),
        ]
        self.sync += [
            bus.ack.eq(0),
            If(bus.cyc & bus.stb & ~bus.ack, bus.ack.eq(1)),
        ]

        # Frame state ------------------------------------------------------------------------------
        rendering = Signal() # From frame_start to the last pixel (sampler paused).
        done      = Signal(reset=1)
        self.sync += [
            If(self.frame_start,
                rendering.eq(1),
            ).Elif(done,
                rendering.eq(0),
            ),
        ]

        # Sampler ----------------------------------------------------------------------------------
        wr_ptr  = Signal(max=wave_depth)
        count   = Signal(max=wave_depth + 1)
        single  = Signal()
        timer   = Signal(32)
        running = Signal()
        sample  = self.sample_we
        self.comb += [
            running.eq(self._wave_control.fields.run | single),
            sample.eq(running & ~rendering & (timer == 0)),
            self._wave_status.fields.running.eq(running),
            wave_wr.adr.eq(wr_ptr),
            wave_wr.dat_w.eq(self.samples),
            wave_wr.we.eq(sample),
        ]
        self.sync += [
            If(timer == 0,
                timer.eq(self._wave_divider.storage)
            ).Else(
                timer.eq(timer - 1)
            ),
            If(sample,
                wr_ptr.eq(wr_ptr + 1),
                count.eq(count - 1),
                If(count == 1, single.eq(0)),
            ),
            If(self._wave_control.fields.single,
                single.eq(1),
                count.eq(wave_depth),
            ),
        ]

        # Pixel generator (S0) ---------------------------------------------------------------------
        latency = 4
        self.fifo = fifo = ResetInserter()(stream.SyncFIFO([("data", 32)], fifo_depth))
        self.comb += [
            fifo.reset.eq(self.frame_start),
            fifo.source.connect(self.source),
        ]

        x        = Signal(max=width)
        y        = Signal(max=height)
        yy       = Signal(4)
        row      = Signal(max=max(rows, 2))
        row_base = Signal(max=cols*rows)
        base     = Signal(max=wave_depth)
        emit     = Signal()
        self.comb += emit.eq(~done & ~self.frame_start & (fifo.level < (fifo_depth - latency - 1)))
        self.sync += [
            If(self.frame_start,
                x.eq(0),
                y.eq(0),
                yy.eq(0),
                row.eq(0),
                row_base.eq(0),
                base.eq(wr_ptr), # Oldest sample on the left.
                done.eq(0),
            ).Elif(emit,
                x.eq(x + 2),
                If(x == (width - 2),
                    x.eq(0),
                    y.eq(y + 1),
                    yy.eq(yy + 1),
                    If(yy == (fh - 1),
                        row.eq(row + 1),
                        row_base.eq(row_base + cols),
                    ),
                    If(y == (height - 1),
                        done.eq(1),
                    ),
                ),
            ),
        ]
        self.sync += If(emit & (x == (width - 2)) & (y == (height - 1)),
            self._frames.status.eq(self._frames.status + 1)
        )

        # Waveform window.
        x0      = Signal(16)
        wave_x  = Signal(16)
        in_x    = Signal()
        in_rows = Signal()
        lane    = Signal(8)
        self.comb += [
            x0.eq(self._wave_column.storage*fw),
            wave_x.eq(x - x0),
            in_x.eq((x >= x0) & (wave_x < 2*wave_depth)),
            lane.eq(row - self._wave_row.storage),
            in_rows.eq((row >= self._wave_row.storage) & (lane < self._wave_lanes.storage)),
            text_rd.adr.eq(row_base + x[log2_int(fw):]),
            wave_rd.adr.eq(wave_x[1:] + base),
        ]

        # S1: cell/sample read.
        s1_valid = Signal()
        s1_xx    = Signal(log2_int(fw))
        s1_yy    = Signal(4)
        s1_wave  = Signal()
        s1_first = Signal()
        s1_bit   = Signal(5)
        self.sync += [
            s1_valid.eq(emit),
            s1_xx.eq(x[:log2_int(fw)]),
            s1_yy.eq(yy),
            s1_wave.eq(in_x & in_rows),
            s1_first.eq(wave_x == 0),
            s1_bit.eq(lane + self._wave_first.storage),
        ]
        cell = text_rd.dat_r
        self.comb += font_rd.adr.eq(Cat(s1_yy, cell[0:8]))

        # S2: glyph line read, waveform levels.
        s2_valid = Signal()
        s2_xx    = Signal(log2_int(fw))
        s2_yy    = Signal(4)
        s2_fg    = Signal(4)
        s2_bg    = Signal(4)
        s2_wave  = Signal()
        s2_level = Signal()
        s2_edge  = Signal()
        prev     = Signal(wave_width)
        level      = Signal()
        prev_level = Signal()
        self.comb += [
            level.eq((wave_rd.dat_r >> s1_bit)[0]),
            prev_level.eq((prev >> s1_bit)[0]),
        ]
        self.sync += [
            If(s1_valid, prev.eq(wave_rd.dat_r)),
            s2_valid.eq(s1_valid & ~self.frame_start),
            s2_xx.eq(s1_xx),
            s2_yy.eq(s1_yy),
            s2_fg.eq(cell[8:12]),
            s2_bg.eq(cell[12:16]),
            s2_wave.eq(s1_wave),
            s2_level.eq(level),
            s2_edge.eq(~s1_first & (level != prev_level)),
        ]

        # S3: pixels colors.
        s3_valid = Signal()
        s3_c0    = Signal(4)
        s3_c1    = Signal(4)
        glyph    = Array(font_rd.dat_r[fw - 1 - i] for i in range(fw)) # Pixels (bit 7: left).
        pix0     = Signal()
        pix1     = Signal()
        line     = Signal()
        edge     = Signal()
        self.comb += [
            pix0.eq(glyph[s2_xx]),
            pix1.eq(glyph[s2_xx | 1]),
            line.eq(s2_wave & ((s2_level & (s2_yy == EYES_WAVE_HIGH)) |
                              (~s2_level & (s2_yy == EYES_WAVE_LOW)))),
            edge.eq(s2_wave & s2_edge & (s2_yy >= EYES_WAVE_HIGH) & (s2_yy <= EYES_WAVE_LOW)),
        ]
        color = self._wave_color.storage
        self.sync += [
            s3_valid.eq(s2_valid & ~self.frame_start),
            s3_c0.eq(Mux(line | edge, color, Mux(pix0, s2_fg, s2_bg))),
            s3_c1.eq(Mux(line,        color, Mux(pix1, s2_fg, s2_bg))),
        ]

        # S4: palette (YUV), YUY2 word (U/V: average of the 2 pixels).
        palette = Array(Constant(y | (u << 8) | (v << 16), 24) for y, u, v in EYES_PALETTE_YUV)
        yuv0 = Signal(24)
        yuv1 = Signal(24)
        u    = Signal(9)
        v    = Signal(9)
        self.comb += [
            yuv0.eq(palette[s3_c0]),
            yuv1.eq(palette[s3_c1]),
            u.eq(yuv0[8:16]  + yuv1[8:16]),
            v.eq(yuv0[16:24] + yuv1[16:24]),
        ]
        self.sync += [
            fifo.sink.valid.eq(s3_valid & ~self.frame_start),
            fifo.sink.data.eq(Cat(yuv0[0:8], u[1:9], yuv1[0:8], v[1:9])),
        ]
