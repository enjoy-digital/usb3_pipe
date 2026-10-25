#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 TX/RX datapaths (USB 3.2 6.3/6.4): word alignment, SKP Ordered Sets insertion/removal,
8b/10b errors substitution/monitoring, electrical idle filtering, clock domain crossing and
data-width adaptation between the transceiver and the sys clock domain.
"""

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common import K, COM, SKP
from usb3_pipe.coding import RX8b10bChecker

# RX SKP Remover (6.4.3) ---------------------------------------------------------------------------

class RXSKPRemover(LiteXModule):
    """RX SKP Remover

    SKP Ordered Sets are inserted in the stream for clock compensation between partners with an
    average of 1 SKP Ordered Set every 354 symbols. This module removes SKP Ordered Sets from
    the RX stream.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.skip   = Signal()

        # # #

        # Find SKP symbols -------------------------------------------------------------------------
        skp = Signal(4)
        for i in range(4):
            self.comb += skp[i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == SKP.value))
        self.comb += self.skip.eq(self.sink.valid & self.sink.ready & (skp != 0))

        # Select valid Data/Ctrl fragments ---------------------------------------------------------
        frag = stream.Endpoint([("data", 32), ("ctrl", 4), ("bytes", 3)])
        cases = {}
        for i in range(2**4):
            datas = []
            ctrls = []
            for j in range(4):
                if (i & 2**j) == 0:
                    datas.append(sink.data[8*j:8*(j+1)])
                    ctrls.append(sink.ctrl[1*j:1*(j+1)])
            cases[i] = [
                frag.data.eq(Cat(*datas) if len(datas) else 0),
                frag.ctrl.eq(Cat(*ctrls) if len(ctrls) else 0),
                frag.bytes.eq(len(ctrls)),
            ]
        self.comb += Case(skp, cases)

        # Register the fragments (timing: SKP detection/fragment selection and Shift Register update
        # in separate cycles, the RX clock domain runs at 250MHz with 2-symbols transceivers).
        self.pipe = pipe = stream.PipeValid([("data", 32), ("ctrl", 4), ("bytes", 3)])
        self.comb += [
            frag.valid.eq(sink.valid),
            sink.ready.eq(frag.ready),
            frag.connect(pipe.sink),
        ]
        frag_data  = pipe.source.data
        frag_ctrl  = pipe.source.ctrl
        frag_bytes = pipe.source.bytes

        # Store Data/Ctrl in a 64/8-bit Shift Register ---------------------------------------------
        sr_data  = Signal(64, reset_less=True)
        sr_ctrl  = Signal(8,  reset_less=True)
        sr_bytes = Signal(4)
        cases = {}
        cases[0] = [
            sr_data.eq(sr_data),
            sr_ctrl.eq(sr_ctrl),
        ]
        for i in range(1, 5):
            cases[i] = [
                sr_data.eq(Cat(sr_data[8*i:], frag_data[0:8*i])),
                sr_ctrl.eq(Cat(sr_ctrl[1*i:], frag_ctrl[0:1*i])),
            ]
        # Accept a fragment (up to 4 bytes) only if it fits in the Shift Register, taking into account
        # the word output on the same cycle.
        self.comb += pipe.source.ready.eq((sr_bytes <= 4) | (source.ready & (sr_bytes <= 8)))
        self.sync += [
            If(pipe.source.valid & pipe.source.ready,
                If(source.valid & source.ready,
                    sr_bytes.eq(sr_bytes + frag_bytes - 4)
                ).Else(
                    sr_bytes.eq(sr_bytes + frag_bytes)
                ),
                Case(frag_bytes, cases)
            ).Elif(source.valid & source.ready,
                sr_bytes.eq(sr_bytes - 4)
            )
        ]

        # Output Data/Ctrl when there is a full 32/4-bit word --------------------------------------
        self.comb += source.valid.eq(sr_bytes >= 4)
        cases = {}
        for i in range(4, 9):
            cases[i] = [
                source.data.eq(sr_data[8*(8-i):8*(8-i+4)]),
                source.ctrl.eq(sr_ctrl[1*(8-i):1*(8-i+4)]),
            ]
        self.comb += Case(sr_bytes, cases)


# RX Aligner ---------------------------------------------------------------------------------------

class RXWordAligner(LiteXModule):
    """RX Word Aligner

    Align RX Words by analyzing the location of the alignment markers (COM by default, configurable
    K-codes, or any K-code with check_ctrl_only) in the RX stream.
    """
    def __init__(self, check_ctrl_only=False, markers=(COM,)):
        self.enable = Signal(reset=1)
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        alignment   = Signal(2)
        alignment_d = Signal(2)

        self.buf = buf = stream.Buffer([("data", 32), ("ctrl", 4)])
        self.comb += [
            sink.connect(buf.sink),
            source.valid.eq(sink.valid & buf.source.valid),
            buf.source.ready.eq(sink.valid & source.ready),
        ]

        # Alignment markers (markers K-codes, or any K-code with check_ctrl_only).
        def is_marker(data, ctrl):
            if check_ctrl_only:
                return ctrl
            return ctrl & reduce(or_, [data == m.value for m in markers])
        marker   = Signal(4)
        marker_d = Signal(4)
        for i in range(4):
            self.comb += marker[i].eq(is_marker(sink.data[8*i:8*(i+1)], sink.ctrl[i]))
            self.comb += marker_d[i].eq(is_marker(buf.source.data[8*i:8*(i+1)], buf.source.ctrl[i]))

        # Alignment detection: first marker of the word.
        for i in reversed(range(4)):
            self.comb += If(marker[i], alignment.eq(i))

        # Alignment update: only at the start of a marker run (markers in the word and no marker on the
        # last symbol of the previous word), so that alignment is kept on non-marker K-codes (ex: Link
        # Layer framing in U0) and on marker runs spanning 2 words. The new alignment is applied on
        # the word with the marker run, so the symbols before the run are output in the same word
        # (without a word mixing symbols before the run and the start of the run).
        alignment_sel = Signal(2)
        self.comb += [
            alignment_sel.eq(alignment_d),
            If(self.enable & (marker != 0) & ~marker_d[3],
                alignment_sel.eq(alignment)
            )
        ]
        self.sync += If(sink.valid & sink.ready, alignment_d.eq(alignment_sel))

        # Data selection
        data = Cat(buf.source.data, sink.data)
        ctrl = Cat(buf.source.ctrl, sink.ctrl)
        cases = {}
        for i in range(4):
            cases[i] = [
                source.data.eq(data[8*i:]),
                source.ctrl.eq(ctrl[i:]),
            ]
        self.comb += Case(alignment_sel, cases)

# RXErrorSubstitution (6.3.1.4) --------------------------------------------------------------------

class RXErrorSubstitution(LiteXModule):
    """RX Error Substitution

    Substitutes received 8b/10b decode errors and disparity errors with K28.4 symbols (6.3.1.4).
    """
    def __init__(self, serdes, clock_domain):
        self.sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        self.comb += self.sink.connect(self.source)
        checker = RX8b10bChecker(serdes.decoders)
        if clock_domain != "sys":
            checker = ClockDomainsRenamer(clock_domain)(checker)
        self.checker = checker
        for i in range(len(serdes.decoders)):
            self.comb += [
                If(serdes.decoders[i].invalid | checker.invalid[i] | checker.disparity_error[i],
                    self.source.ctrl[i].eq(1),
                    self.source.data[8*i:8*(i+1)].eq(K(28, 4)),
                )
            ]

# RX Error Monitor ---------------------------------------------------------------------------------

class RXErrorMonitor(LiteXModule):
    """RX Error Monitor

    Reports (in the sys clock domain) whether 8b/10b decode errors are being received: `errors` is
    set while decode errors are seen with no more than `window` RX clock cycles between them.

    A LFPS signal (10-50MHz square wave, Table 6-29) sampled at the 5Gbps line rate never decodes as
    valid 8b/10b symbols while SuperSpeed data does: this is used to differentiate a Warm Reset LFPS
    from data, both seen as a non electrical idle line.
    """
    def __init__(self, decoders, clock_domain="sys", window=256):
        self.errors = Signal() # o

        # # #

        invalid = Signal()
        errors  = Signal()
        count   = Signal(max=window + 1)
        self.comb += invalid.eq(Reduce("OR", [decoder.invalid for decoder in decoders]))
        sync = getattr(self.sync, clock_domain)
        sync += [
            If(invalid,
                count.eq(window),
                errors.eq(1),
            ).Elif(count != 0,
                count.eq(count - 1),
            ).Else(
                errors.eq(0),
            )
        ]
        self.specials += MultiReg(errors, self.errors)

# RX Idle Filter -----------------------------------------------------------------------------------

class RXIdleFilter(LiteXModule):
    """RX Electrical Idle Filter

    Resynchronizes the transceiver's electrical idle detection and only reports electrical idle
    after `idle_cycles` consecutive idle cycles: filters the idle detection toggling during LFPS.
    A non-idle burst is seen stretched by `stretch_cycles` (filter latency on the idle edge only).
    """
    def __init__(self, rx_elecidle, idle_cycles=10):
        self.idle           = Signal() # o
        self.stretch_cycles = idle_cycles - 1

        # # #

        rx_elecidle_r = Signal(reset=1)
        count         = Signal(max=idle_cycles + 1, reset=idle_cycles) # Idle at reset.
        self.specials += MultiReg(rx_elecidle, rx_elecidle_r, reset=1)
        self.sync += [
            If(rx_elecidle_r,
                If(count != idle_cycles,
                    count.eq(count + 1)
                )
            ).Else(
                count.eq(0)
            )
        ]
        self.comb += self.idle.eq(count == idle_cycles)

# TX SKP Inserter (6.4.3) --------------------------------------------------------------------------

# 2 SKP Ordered Sets every 176 words (704 symbols): average of 1 SKP Ordered Set every 354 symbols
# (6.4.3).
SKP_INTERVAL_WORDS = 176

class TXSKPInserter(LiteXModule):
    """TX SKP Inserter

    SKP Ordered Sets are inserted in the stream for clock compensation between partners with an
    average of 1 SKP Ordered Set every 354 symbols. This module inserts SKP Ordered Sets to the
    TX stream. SKP Ordered Sets shall not be inserted inside a packet, so this packet delimiters
    (first/last) should be used to ensure SKP are inserted only between packets and not inside.

    Note: To simplify implementation and keep alignment, 2 SKP Ordered Sets are inserted every 708
    symbols, which is a small deviation from the specification (good average, but 2x interval between
    SKPs). More tests need to be done to ensure this deviation is acceptable with all hardwares.

    Up to 4 SKP Ordered Sets (2 SKP words) are buffered (6.4.3). When disabled (TSEQ: SKP are only
    transmitted after TS1/TS2, packets and Logical Idle, 6.4.3.1), no SKP is scheduled and the count
    of transmitted symbols (Y) is reset, so it restarts from 0 on Polling.Active entry.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.enable = Signal(reset=1) # i

        # # #

        data_count   = Signal(max=SKP_INTERVAL_WORDS)
        skip_grant   = Signal(reset=1)
        skip_queue   = Signal()
        skip_dequeue = Signal()
        skip_count   = Signal(2)

        # Queue 2 SKP Ordered Sets every SKP_INTERVAL_WORDS Data/Ctrl words.
        self.sync += [
            skip_queue.eq(0),
            If(~self.enable,
                data_count.eq(0)
            ).Elif(sink.valid & sink.ready,
                data_count.eq(data_count + 1),
                If(data_count == (SKP_INTERVAL_WORDS - 1),
                    data_count.eq(0),
                    skip_queue.eq(1)
                )
            )
        ]

        # SKP grant: SKP should not be inserted inside packets
        self.sync += [
            If(sink.valid & sink.ready,
                If(sink.last,
                    skip_grant.eq(1)
                ).Elif(sink.first,
                    skip_grant.eq(0)
                )
            )
        ]

        # SKP counter (saturated to 2 SKP words: 4 SKP Ordered Sets buffered max)
        self.sync += [
            If(~self.enable,
                skip_count.eq(0)
            ).Elif(skip_queue & ~skip_dequeue & (skip_count != 2),
                skip_count.eq(skip_count + 1)
            ).Elif(~skip_queue &  skip_dequeue,
                skip_count.eq(skip_count - 1)
            )
        ]

        # SKP insertion
        self.comb += [
            If(skip_grant & (skip_count != 0),
                source.valid.eq(1),
                source.data.eq(Replicate(Constant(SKP.value, 8), 4)),
                source.ctrl.eq(0b1111),
                skip_dequeue.eq(source.ready)
            ).Else(
                sink.connect(source)
            )
        ]

# Datapath (Clock Domain Crossing & Converter) -----------------------------------------------------

class TXDatapath(LiteXModule):
    """TX Datapath

    This module realizes the:
    - Clock compensation (SKP insertion).
    - Clock domain crossing (from system clock to transceiver's TX clock).
    - Data-width adaptation (from 32-bit to transceiver's data-width).
    """
    def __init__(self, clock_domain="sys", phy_dw=16):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])

        # # #

        # Clock compensation
        self.skip_inserter = skip_inserter = TXSKPInserter()
        self.skp_enable    = skip_inserter.enable

        # Clock domain crossing
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True)
        cdc = ClockDomainsRenamer({"write": "sys", "read": clock_domain})(cdc)
        self.cdc = cdc

        # Data-width adaptation
        converter = stream.StrideConverter(
            [("data", 32), ("ctrl", 4)],
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            reverse=False)
        converter = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.converter = converter

        # Flow
        self.comb += [
            self.sink.connect(skip_inserter.sink),
            skip_inserter.source.connect(cdc.sink),
            cdc.source.connect(converter.sink),
            converter.source.connect(self.source)
        ]

class RXDatapath(LiteXModule):
    """RX Datapath

    This module realizes the:
    - Data-width adaptation (from transceiver's data-width to 32-bit).
    - Clock compensation (SKP removing).
    - Clock domain crossing (from transceiver's RX clock to system clock).
    - Words alignment.

    SKP removal is done in the transceiver's RX clock domain, before the clock domain crossing: the
    partner's clock can be faster than the local one (6.4.3), SKP Ordered Sets are then needed to
    compensate the difference and only data symbols have to cross to the system clock domain.
    """
    def __init__(self, clock_domain="sys", phy_dw=16):
        self.sink   = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Data-width adaptation
        converter = stream.StrideConverter(
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            [("data", 32), ("ctrl", 4)],
            reverse=False)
        converter = stream.BufferizeEndpoints({"sink":   stream.DIR_SINK})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.converter = converter

        # Clock compensation
        skip_remover = RXSKPRemover()
        skip_remover = ClockDomainsRenamer(clock_domain)(skip_remover)
        self.skip_remover = skip_remover

        # Clock domain crossing
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8, buffered=True)
        cdc = ClockDomainsRenamer({"write": clock_domain, "read": "sys"})(cdc)
        self.cdc = cdc

        # Words alignment
        word_aligner = RXWordAligner()
        word_aligner = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(word_aligner)
        self.word_aligner = word_aligner

        # Flow
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(skip_remover.sink),
            skip_remover.source.connect(cdc.sink),
            cdc.source.connect(word_aligner.sink),
            word_aligner.source.connect(self.source),
        ]
