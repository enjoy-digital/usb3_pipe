#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Shared helpers of the device core figures (core_*.py, class_*.py): packet exchange lanes and
# stream waveforms. Not a figure (no build()).

from svgdiag import *

# Packet exchange lanes ----------------------------------------------------------------------------

class Lanes:
    """Two vertical lanes (left/right) with a title box on top of each; messages are horizontal
    arrows between them, one per row, the label above the arrow."""
    def __init__(self, d, left, right, xl=170, xr=590, top=20, row=34, rows=8, bw=150):
        self.d, self.xl, self.xr, self.row = d, xl, xr, row
        self.y0 = top + 40 + row*0.8
        bottom  = top + 40 + row*(rows + 0.6)
        d.add(Box(xl - bw/2, top, bw, 34, label=left,  fill=COLOR_HARDWARE))
        d.add(Box(xr - bw/2, top, bw, 34, label=right, fill=COLOR_ACCENT))
        d.add(Bus([(xl, top + 34), (xl, bottom)], dash="4,3"))
        d.add(Bus([(xr, top + 34), (xr, bottom)], dash="4,3"))
        self.n = 0

    def msg(self, label, to_right=True, kind="data", color=COLOR_TEXT, note=None):
        y = self.y0 + self.n*self.row
        a, b = ((self.xl, y), (self.xr, y)) if to_right else ((self.xr, y), (self.xl, y))
        self.d.add(Arrow(a, b, kind=kind, label=label, label_pos=((self.xl + self.xr)/2, y - 11),
            label_size=15, label_color=color))
        if note:
            # Notes on the right lane side (the side the figure is about).
            self.d.add(Text(self.xr + 10, y, note, size=14, anchor="start", color=COLOR_STROKE))
        self.n += 1
        return y

    def gap(self, n=0.5):
        self.n += n

# Stream waveforms ---------------------------------------------------------------------------------

class Waves:
    """Synchronous waveforms: a clock and named rows of bits or bus values, one column per cycle."""
    def __init__(self, d, x0=150, y0=20, cycles=10, cw=56, rh=34, label_x=135):
        self.d, self.x0, self.y, self.cycles, self.cw, self.rh = d, x0, y0, cycles, cw, rh
        self.label_x = label_x

    def _label(self, name):
        self.d.add(Text(self.label_x, self.y + self.rh/2, name, size=13, anchor="end"))

    def clock(self, name="clk"):
        self._label(name)
        lo, hi = self.y + self.rh - 8, self.y + 8
        pts = [(self.x0, lo)]
        for c in range(self.cycles):
            x = self.x0 + c*self.cw
            pts += [(x, hi), (x + self.cw/2, hi), (x + self.cw/2, lo), (x + self.cw, lo)]
        self.d.add(Bus(_dedup(pts)))
        self.y += self.rh

    def bit(self, name, values):
        """values: string of '0'/'1', one per cycle."""
        self._label(name)
        lo, hi = self.y + self.rh - 8, self.y + 8
        pts = []
        for c, v in enumerate(values):
            yv = hi if v == "1" else lo
            x  = self.x0 + c*self.cw
            pts += [(x, yv), (x + self.cw, yv)]
        self.d.add(Bus(_dedup(pts), kind="data" if False else "control"))
        self.y += self.rh

    def bus(self, name, values, accent=None):
        """values: list of labels (None: idle), one per cycle; equal neighbours are merged."""
        self._label(name)
        c = 0
        while c < len(values):
            n = 1
            while c + n < len(values) and values[c + n] == values[c]:
                n += 1
            v = values[c]
            x = self.x0 + c*self.cw
            if v is None:
                self.d.add(Bus([(x, self.y + self.rh/2), (x + n*self.cw, self.y + self.rh/2)],
                    color=COLOR_SHADE))
            else:
                fill = COLOR_ACCENT if (accent and v in accent) else COLOR_FILL
                self.d.add(Box(x + 2, self.y + 6, n*self.cw - 4, self.rh - 12, label=v, size=12,
                    radius=0, fill=fill))
            c += n
        self.y += self.rh

    def marker(self, cycle, label):
        """Note below the waveforms, centered on a cycle."""
        x = self.x0 + (cycle + 0.5)*self.cw
        self.d.add(Text(x, self.y + 12, label, size=12, color=COLOR_STROKE))


def _dedup(pts):
    out = []
    for p in pts:
        if out and abs(out[-1][0] - p[0]) < 0.01 and abs(out[-1][1] - p[1]) < 0.01:
            continue
        out.append(p)
    # Merge collinear points (no zero-length segments).
    merged = [out[0]]
    for p in out[1:]:
        if len(merged) >= 2:
            a, b = merged[-2], merged[-1]
            if (abs(a[0] - b[0]) < 0.01 and abs(b[0] - p[0]) < 0.01) or \
               (abs(a[1] - b[1]) < 0.01 and abs(b[1] - p[1]) < 0.01):
                merged[-1] = p
                continue
        merged.append(p)
    return merged
