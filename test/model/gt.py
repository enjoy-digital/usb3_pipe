#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Transceiver (SerDes) models for USB3PIPE/USB3PHY/LFPSUnit Migen simulations."""

from migen import *

from litex.soc.interconnect import stream

from usb3_pipe.datapath import RXIdleFilter

# Transceiver Model --------------------------------------------------------------------------------

class GTModel(Module):
    """Transceiver model: SerDes interface (usb3_pipe.serdes.USB3SerDes attributes) of
    USB3PIPE/USB3PHY/LFPSUnit.

    tx_ready: TX always ready (like the GT, which also transmits the last presented word again when
              the stream is not valid, see wire_monitor), RX only models leave it low.

    RX words are driven on source (the GT ignores back-pressure: one received word per cycle). The
    controls (enable, tx_*, rx_align/rx_polarity/rx_reset) are only observed, ready is always 1 and
    rx_errors is held at 1 (no decode errors model: Warm Reset LFPS detected on the electrical idle
    only)."""
    def __init__(self, tx_ready=True):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal(reset=1) # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i
        self.tx_lfps     = Signal()   # i

        self.rx_polarity = Signal()        # i
        self.rx_idle     = Signal()        # o
        self.rx_errors   = Signal(reset=1) # o

        self.tx_skp_enable = Signal(reset=1) # i
        self.rx_align      = Signal()        # i
        self.rx_reset      = Signal()        # i

        self.rx_idle_stretch = 0
        self.xcvr_name       = None

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

class LFPSSerDesStub(GTModel):
    """LFPS face of the SerDes (LFPSUnit serdes interface): GTModel with the RX electrical idle
    driven by the test.

    with_rx_idle_filter: raw transceiver electrical idle (rx_elecidle) through the RX idle filter, as
                         in the K7/A7 SerDes (RXELECIDLE + RXIdleFilter), rx_idle driven directly
                         otherwise.
    with_rx_errors     : rx_errors driven by the test (8b/10b errors, Warm Reset detection), else
                         held at 1 (electrical idle detection only)."""
    def __init__(self, with_rx_idle_filter=False, with_rx_errors=False):
        GTModel.__init__(self, tx_ready=False)
        if with_rx_errors:
            self.rx_errors = Signal()
        if with_rx_idle_filter:
            self.rx_elecidle = Signal(reset=1) # Raw transceiver electrical idle.
            self.submodules.rx_idle_filter = RXIdleFilter(self.rx_elecidle)
            self.rx_idle         = self.rx_idle_filter.idle
            self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles
        else:
            self.rx_idle = Signal(reset=1)
