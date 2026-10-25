#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Transceiver (SerDes) models for USB3PIPE/USB3PHY/LFPSUnit Migen simulations."""

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.serdes import RXIdleFilter

# Transceiver Model --------------------------------------------------------------------------------

class GTModel(Module):
    """Transceiver model: SerDes interface of USB3PIPE/USB3PHY.

    tx_ready     : TX always ready (like the GT, which also transmits the last presented word again
                   when the stream is not valid, see wire_monitor), RX only models leave it low.
    with_ready   : SerDes ready (USB3PHY), always 1.
    with_rx_reset: SerDes RX re-initialization request (USB3PHY drives it when present).

    RX words are driven on source (the GT ignores back-pressure: one received word per cycle)."""
    def __init__(self, tx_ready=True, with_ready=False, with_rx_reset=False):
        self.sink        = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source      = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.rx_align    = Signal()
        self.rx_polarity = Signal()
        self.rx_idle     = Signal()
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)
        if with_ready:
            self.ready = Signal(reset=1)
        if with_rx_reset:
            self.rx_reset = Signal()

        # # #

        if tx_ready:
            self.comb += self.sink.ready.eq(1)

def wire_monitor(gt, n):
    """Record n words transmitted on the wire (last word repeated when not valid), starting from the
    first valid word."""
    words = []
    last  = None
    for _ in range(n):
        yield
        if (yield gt.sink.valid):
            last = ((yield gt.sink.data), (yield gt.sink.ctrl))
        if last is not None:
            words.append(last)
    return words

# LFPS SerDes Stub ---------------------------------------------------------------------------------

class LFPSSerDesStub(Module):
    """LFPS face of the SerDes (LFPSUnit serdes interface).

    with_rx_idle_filter: raw transceiver electrical idle (rx_elecidle) through the RX idle filter, as
                         in the K7/A7 SerDes (RXELECIDLE + RXIdleFilter), rx_idle driven directly
                         otherwise.
    with_rx_errors     : rx_errors (8b/10b errors, Warm Reset detection).
    with_tx_lfps       : tx_lfps (LFPS transmission, driven by the LFPSUnit when present)."""
    def __init__(self, with_rx_idle_filter=False, with_rx_errors=False, with_tx_lfps=False):
        self.tx_idle    = Signal()
        self.tx_pattern = Signal(20)
        if with_tx_lfps:
            self.tx_lfps = Signal()
        if with_rx_errors:
            self.rx_errors = Signal()
        if with_rx_idle_filter:
            self.rx_elecidle = Signal(reset=1) # Raw transceiver electrical idle.
            self.submodules.rx_idle_filter = RXIdleFilter(self.rx_elecidle)
            self.rx_idle         = self.rx_idle_filter.idle
            self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles
        else:
            self.rx_idle = Signal(reset=1)
