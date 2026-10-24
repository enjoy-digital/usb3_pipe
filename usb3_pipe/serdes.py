#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 SerDes (USB 3.2 chapter 6): LiteICLink transceivers (Artix7 GTP/Kintex7 GTX/Ultrascale+ GTY) at
5Gbps and the TX/RX datapaths (8b/10b, word alignment, polarity, SKP insertion/removal, electrical
idle/LFPS).
"""

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect import stream
from litex.soc.cores import code_8b10b

from usb3_pipe.common import K, COM, SKP

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
    def __init__(self, check_ctrl_only=False, markers=[COM]):
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

# 8b/10b Code Tables (6.3.1, Appendix A) -----------------------------------------------------------

# Valid K-codes (Table 6-2 / Appendix A).
K_CODES = [(28, 0), (28, 1), (28, 2), (28, 3), (28, 4), (28, 5), (28, 6), (28, 7),
           (23, 7), (27, 7), (29, 7), (30, 7)]

def encode_8b10b(d, k, rd):
    """Python model of LiteX's SingleEncoder (code_8b10b): returns the 10-bit code (MSB first,
    abcdei fghj) and the running disparity (0: RD-, 1: RD+) after the symbol."""
    t = code_8b10b
    code5b, code3b = d & 0x1f, d >> 5
    if k and code5b == 28:
        code6b, unb6, flip6 = 0b110000, True, True
    else:
        code6b, unb6, flip6 = t.table_5b6b[code5b], t.table_5b6b_unbalanced[code5b], t.table_5b6b_flip[code5b]
    code4b = t.table_3b4b[code3b]
    unb4   = t.table_3b4b_unbalanced[code3b]
    flip4  = True if k else t.table_3b4b_flip[code3b]
    alt7_rd0 = (code3b == 7) and ((code5b in [17, 18, 20]) or k)
    alt7_rd1 = (code3b == 7) and ((code5b in [11, 13, 14]) or k)
    disp_inter = rd ^ unb6
    out6 = (~code6b & 0x3f) if (not rd and flip6) else code6b
    if not disp_inter and alt7_rd0:
        rd_out, out4 = 1, 0b0111
    elif disp_inter and alt7_rd1:
        rd_out, out4 = 0, 0b1000
    else:
        rd_out = disp_inter ^ unb4
        out4   = (~code4b & 0xf) if (not disp_inter and flip4) else code4b
    return (out6 << 4) | out4, int(rd_out)

def code_8b10b_table(lsb_first=True):
    """1024 entries (indexed by the received 10-bit code): (valid with RD-, valid with RD+, RD after
    the symbol for RD-, RD after the symbol for RD+)."""
    table = [[0, 0, 0, 0] for _ in range(1024)]
    symbols = [(d, 0) for d in range(256)] + [(x | (y << 5), 1) for x, y in K_CODES]
    for d, k in symbols:
        for rd in [0, 1]:
            code, rd_out = encode_8b10b(d, k, rd)
            if lsb_first:
                code = int("{:010b}".format(code)[::-1], 2)
            table[code][rd]     = 1
            table[code][2 + rd] = rd_out
    return table

# RX 8b/10b Checker (6.3.1.2, 6.3.1.4) -------------------------------------------------------------

class RX8b10bChecker(LiteXModule):
    """RX 8b/10b Checker

    Checks the received 10-bit symbols (decoders inputs) against the valid 8b/10b codes, with the
    same latency as the decoders outputs:
    - invalid:         Symbol not matching any valid 8b/10b code (decode error, 6.3.1.4).
    - disparity_error: Valid code received in the wrong running disparity column (6.3.1.2).

    The running disparity is tracked over the symbols and re-initialized from the received code on
    errors (6.3.1.2: initial disparity is the disparity of the first received symbol).
    """
    def __init__(self, decoders, lsb_first=True):
        n = len(decoders)
        self.invalid         = Signal(n) # o
        self.disparity_error = Signal(n) # o

        # # #

        table = code_8b10b_table(lsb_first)
        rom   = Memory(4, 1024, init=[e[0] | (e[1] << 1) | (e[2] << 2) | (e[3] << 3) for e in table])
        self.specials += rom

        # Code lookup, registered (as the decoders outputs).
        lookups = []
        for decoder in decoders:
            port = rom.get_port(async_read=True)
            self.specials += port
            lookup = Signal(4)
            self.comb += port.adr.eq(decoder.input)
            self.sync += lookup.eq(port.dat_r)
            lookups.append(lookup)

        # Errors and running disparity.
        rd = Signal()
        rd_lane = rd
        for i, lookup in enumerate(lookups):
            valid_m, valid_p, rd_m, rd_p = [lookup[j] for j in range(4)]
            rd_next = Signal()
            self.comb += [
                self.invalid[i].eq(~valid_m & ~valid_p),
                self.disparity_error[i].eq(Mux(rd_lane, ~valid_p & valid_m, ~valid_m & valid_p)),
                If(rd_lane,
                    If(valid_p,  rd_next.eq(rd_p)).Elif(valid_m, rd_next.eq(rd_m)).Else(rd_next.eq(rd_lane))
                ).Else(
                    If(valid_m,  rd_next.eq(rd_m)).Elif(valid_p, rd_next.eq(rd_p)).Else(rd_next.eq(rd_lane))
                )
            ]
            rd_lane = rd_next
        self.sync += rd.eq(rd_lane)

# RXErrorSubstitution (6.3.1.4) --------------------------------------------------------------------

class RXErrorSubstitution(LiteXModule):
    """RX Error Substitution

    Substitutes received 8b/10b decode errors and disparity errors with K28.4 symbols (6.3.1.4).
    """
    def __init__(self, serdes, clock_domain, enable=True):
        self.sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        self.comb += self.sink.connect(self.source)
        checker = RX8b10bChecker(serdes.decoders)
        if clock_domain != "sys":
            checker = ClockDomainsRenamer(clock_domain)(checker)
        self.checker = checker
        if enable:
            for i in range(2):
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

        data_count   = Signal(8)
        skip_grant   = Signal(reset=1)
        skip_queue   = Signal()
        skip_dequeue = Signal()
        skip_count   = Signal(2)

        # Queue one 2 SKP Ordered Set every 176 Data/Ctrl words
        self.sync += [
            skip_queue.eq(0),
            If(~self.enable,
                data_count.eq(0)
            ).Elif(sink.valid & sink.ready,
                data_count.eq(data_count + 1),
                If(data_count == 175,
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

# USB3 Serializer/Deserializer --------------------------------------------------------------------

class USB3SerDes(LiteXModule):
    """USB3 SerDes on a LiteICLink transceiver.

    Common part of the transceivers: 5Gbps, 20-bit with 8b/10b, TX/RX buffers (no clock aligner, word
    alignment done in the RX datapath), TX/RX datapaths (SKP insertion/removal, CDC, word alignment),
    LFPS (TX pattern/electrical idle, filtered RX electrical idle), RX decode errors (Warm Reset LFPS
    detection) and timing constraints. Transceivers specific parts (PLL, USB3 parameters overrides,
    electrical idle/OOB signals) are done by the subclasses.
    """
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i
        self.tx_lfps     = Signal()   # i: LFPS burst, transmit tx_pattern raw.

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_errors   = Signal()   # o

        self.tx_skp_enable = Signal(reset=1) # i
        self.rx_align      = Signal()        # i
        self.rx_reset      = Signal()        # i: RX re-initialization (PMA/CDR/PCS reset sequence).

    def get_refclk(self, refclk_pads, buffer="IBUFDS_GTE2"):
        """Transceiver reference clock: from dedicated pads (buffer) or a fabric clock."""
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            return refclk_pads
        refclk = Signal()
        self.specials += Instance(buffer,
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk
        )
        return refclk

    def add_transceiver(self, name, xcvr_cls, pll, tx_pads, rx_pads, sys_clk_freq):
        """Transceiver channel (added as name) with the TX/RX datapaths."""
        xcvr = xcvr_cls(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 20,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            tx_polarity      = self.tx_polarity,
            rx_polarity      = self.rx_polarity)
        self.add_module(name=name, module=xcvr)
        self.xcvr_name = name
        xcvr.add_stream_endpoints()
        self.tx_datapath     = tx_datapath     = TXDatapath("tx")
        self.rx_substitution = rx_substitution = RXErrorSubstitution(xcvr, "rx")
        self.rx_datapath     = rx_datapath     = RXDatapath("rx")
        self.comb += [
            xcvr.tx_enable.eq(self.enable),
            xcvr.rx_enable.eq(self.enable & ~self.rx_reset),
            self.ready.eq(xcvr.tx_ready & xcvr.rx_ready),
            xcvr.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.skp_enable.eq(self.tx_skp_enable),
            tx_datapath.source.connect(xcvr.sink),
            xcvr.source.connect(rx_substitution.sink),
            rx_substitution.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]
        return xcvr

    def add_lfps(self, xcvr, rx_idle):
        """LFPS: TX pattern, filtered RX electrical idle (from the transceiver's OOB detector) and RX
        decode errors (Warm Reset LFPS detection)."""
        self.comb += [
            xcvr.tx_produce_pattern.eq(self.tx_lfps | (self.tx_pattern != 0)),
            xcvr.tx_pattern.eq(self.tx_pattern),
        ]
        self.rx_idle_filter  = RXIdleFilter(rx_idle)
        self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles
        self.comb += self.rx_idle.eq(self.rx_idle_filter.idle)
        self.rx_error_monitor = RXErrorMonitor(xcvr.decoders, clock_domain="rx")
        self.comb += self.rx_errors.eq(self.rx_error_monitor.errors)

    def add_timing_constraints(self, platform, sys_clk, xcvr):
        platform.add_period_constraint(xcvr.cd_tx.clk, 1e9/xcvr.tx_clk_freq)
        platform.add_period_constraint(xcvr.cd_rx.clk, 1e9/xcvr.rx_clk_freq)
        platform.add_false_path_constraints(sys_clk, xcvr.cd_tx.clk, xcvr.cd_rx.clk)

# Xilinx Kintex7 USB3 Serializer/Deserializer ------------------------------------------------------

class K7USB3SerDes(USB3SerDes):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gtx_7series import GTXChannelPLL, GTX

        # PLL / Transceiver ------------------------------------------------------------------------
        self.pll = pll = GTXChannelPLL(self.get_refclk(refclk_pads), refclk_freq, 5e9)
        gtx = self.add_transceiver("gtx", GTX, pll, tx_pads, rx_pads, sys_clk_freq)

        # Override GTX RX settings for USB3 ---------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings from
        # the Xilinx 7-Series Transceivers Wizard (SATA, SSC) and PCIe Gen2 (pcie_7x) GT wrappers:
        # - RXCDR_CFG: SSC tolerant CDR settings (RXOUT_DIV=1).
        # - RXCDR_HOLD_DURING_EIDLE/RX_DFE_LPM_HOLD_DURING_EIDLE: the CDR/LPM keep their state during
        #   Electrical Idle/LFPS.
        # - LPM mode (as PCIe Gen1/Gen2).
        # - RX_CM_SEL/RX_CM_TRIM: Programmable, 800mV.
        assert pll.config['d'] == 1 # 5Gbps.
        gtx.gtx_params.update(
            p_RXCDR_CFG                    = 0x03_8000_8BFF_4010_0008,
            p_RXCDR_HOLD_DURING_EIDLE      = 0b1,
            p_RX_DFE_LPM_HOLD_DURING_EIDLE = 0b1,
            p_RX_DFE_LPM_CFG               = 0x0904,
            i_RXLPMEN                      = 0b1,
            p_RX_CM_SEL                    = 0b11,
            p_RX_CM_TRIM                   = 0b010,
            p_PMA_RSV                      = 0x00018480,
            p_PMA_RSV2                     = 0x2050,
        )

        # Override GTX TX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtx.gtx_params.update(
            i_TXDIFFCTRL = 0b1100,
        )

        # Override GTX parameters/signals for USB3 LFPS --------------------------------------------
        # PCS_RSVD_ATTR: OOB powered up (bit 8), OOB clock from CLKRSVD[0] (bit 3).
        rx_idle = Signal()
        gtx.gtx_params.update(
            p_PCS_RSVD_ATTR    = 0x000000000108,
            p_RXOOB_CFG        = 0b0000110,
            i_CLKRSVD          = ClockSignal("oob"),
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gtx, rx_idle)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtx)

# Xilinx Artix7 USB3 Serializer/Deserializer -------------------------------------------------------

class A7USB3SerDes(USB3SerDes):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads,
        rxcdr_cfg=None):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP

        # PLL / Transceiver ------------------------------------------------------------------------
        self.pll = pll = GTPQuadPLL(self.get_refclk(refclk_pads), refclk_freq, 5e9)
        gtp = self.add_transceiver("gtp", GTP, pll, tx_pads, rx_pads, sys_clk_freq)

        # Override GTP RX settings for USB3 ---------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz):
        # - RXCDR_CFG: UG482 Table 4-14 (CDR Recommended Settings for Protocols with SSC, RXOUT_DIV=1).
        #   The 8B/10B (no SSC) settings (Table 4-15, +-1250ppm) saturate the CDR frequency tracking
        #   (recovered clock pinned at ~-1465ppm) and slip a bit every few hundred bits on hardware.
        # - RXCDR_HOLD_DURING_EIDLE/RXCDR_LOCK_CFG: Xilinx 7-Series PCIe Gen2 GTP wrapper settings
        #   (pcie_7x gt_wrapper): the CDR keeps its frequency during Electrical Idle/LFPS.
        assert pll.config['d'] == 1 # 5Gbps.
        gtp.gtp_params.update(
            p_RX_CM_SEL               = 0b11,   # Programmable, 800mV.
            p_RX_CM_TRIM              = 0b1010,
            p_RXCDR_CFG               = 0x0_0000_87FE_2060_2448_1010 if rxcdr_cfg is None else rxcdr_cfg,
            p_RXCDR_HOLD_DURING_EIDLE = 0b1,
            p_RXCDR_LOCK_CFG          = 0b010101,
            p_RXLPM_INCM_CFG          = 0b0,
            p_RXLPM_IPCM_CFG          = 0b1,
        )

        # Override GTP TX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtp.gtp_params.update(
            i_TXDIFFCTRL = 0b1100,
        )

        # Override GTP parameters/signals to allow LFPS --------------------------------------------
        rx_idle = Signal()
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR    = 0x000000000100,
            p_RXOOB_CLK_CFG    = "FABRIC",
            i_SIGVALIDCLK      = ClockSignal("oob"),
            p_RXOOB_CFG        = 0b0000110,
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gtp, rx_idle)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtp)

# Xilinx Ultrascale+ USB3 Serializer/Deserializer --------------------------------------------------

class USPUSB3SerDes(USB3SerDes):
    """Ultrascale+ GTY USB3 SerDes (CPLL, reference clock from dedicated pads or from the fabric)."""
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gty_ultrascale import GTYChannelPLL, GTY

        # PLL / Transceiver ------------------------------------------------------------------------
        refclk_from_fabric = isinstance(refclk_pads, (Signal, ClockSignal))
        refclk = self.get_refclk(refclk_pads, buffer="IBUFDS_GTE4")
        self.pll = pll = GTYChannelPLL(refclk, refclk_freq, 5e9)
        gty = self.add_transceiver("gty", GTY, pll, tx_pads, rx_pads, sys_clk_freq)
        assert pll.config['d'] == 1 # 5Gbps.

        # Fabric reference clock: CPLL from GTGREFCLK.
        if refclk_from_fabric:
            gty.gty_params.update(
                i_GTREFCLK0     = 0,
                i_GTGREFCLK     = refclk,
                i_CPLLREFCLKSEL = 0b111,
            )

        # Override GTY RX settings for USB3 ---------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings from
        # the Xilinx Ultrascale+ Transceivers Wizard (SATA preset at 5Gbps, SSC) and PCIe Gen2
        # (pcie4_uscale_plus) GT wrappers:
        # - RXCDR_CFG2/3: SSC tolerant CDR settings.
        # - RX_SUM_*/RX_DEGEN_CTRL/RX_DFE_AGC_CFG1: 5Gbps settings (LiteICLink: high line rates ones).
        # - RX/TX_CLK25_DIV: from the reference clock (also clocks the OOB detector).
        # - RXCDRHOLD: the CDR keeps its state during Electrical Idle/LFPS (as pcie4_uscale_plus).
        # - LPM mode (as PCIe Gen1/Gen2, LiteICLink default).
        clk25_div = int((refclk_freq + 25e6 - 1)//25e6)
        rx_idle   = Signal()
        gty.gty_params.update(
            p_RXCDR_CFG2                = 0x0263,
            p_RXCDR_CFG2_GEN2           = 0x263,
            p_RXCDR_CFG3                = 0x001a,
            p_RXCDR_CFG3_GEN2           = 0b011010,
            p_RX_SUM_VCMTUNE            = 0b0011,
            p_RX_SUM_DEGEN_AVTT_OVERITE = 0b0,
            p_RX_DEGEN_CTRL             = 0b100,
            p_RX_DFE_AGC_CFG1           = 0b010,
            p_RX_CLK25_DIV              = clk25_div,
            p_TX_CLK25_DIV              = clk25_div,
            i_RXCDRHOLD                 = rx_idle,
        )

        # Override GTY parameters/signals for USB3 LFPS --------------------------------------------
        gty.gty_params.update(
            p_OOB_PWRUP        = 0b1,
            p_OOBDIVCTL        = 0b01,
            p_RXOOB_CFG        = 0b000000110,
            p_RXOOB_CLK_CFG    = "PMA",
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gty, rx_idle)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gty)
