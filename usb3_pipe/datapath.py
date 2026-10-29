#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 TX/RX datapaths (USB 3.2 6.3/6.4).

- Gen 1: word alignment, SKP Ordered Sets insertion/removal, 8b/10b errors substitution/monitoring,
  electrical idle filtering, clock domain crossing and data-width adaptation between the
  transceiver and the sys clock domain.
- Gen 2: block transmitter/receiver (Gen2TX/Gen2RX): TSEQ/TS1/TS2/SYNC/SDS/SKP Ordered Sets, Data
  Blocks and scrambling rules (6.3.2.3), and the transceiver words <-> blocks datapaths
  (Gen2TXDatapath/Gen2RXDatapath, 40-bit at 250MHz for 10Gbps), see doc/gen2-x2-roadmap.md.
"""

from functools import reduce
from operator import or_, and_

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common     import K, COM, SKP, AsyncFIFO, Buffer
from usb3_pipe.common     import BLOCK_HEADER_DATA, BLOCK_HEADER_CONTROL, GEN2_SKP_TX_SYMBOLS
from usb3_pipe.common     import GEN2_SKP, GEN2_SKPEND, GEN2_SDS, GEN2_TS1, GEN2_TS2, GEN2_TSEQ, GEN2_IDLE
from usb3_pipe.common     import gen2_sync_payload, gen2_sds_payload, symbols_to_int
from usb3_pipe.coding     import RX8b10bChecker
from usb3_pipe.coding     import GEN2_DC_REDUCE_ONES, GEN2_DC_REDUCE_ZEROS, popcount
from usb3_pipe.coding     import gen2_block_layout, Gen2BlockGearbox, Gen2BlockAligner
from usb3_pipe.scrambling import GEN2_LFSR_SEED, Gen2Scrambler

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
    with_disparity=False: decode errors only (no running disparity tracking: shorter RX paths on
    slow fabrics, corrupted packets still detected by the Link Layer CRCs).
    """
    def __init__(self, serdes, clock_domain, with_disparity=True):
        n = len(serdes.decoders)
        self.sink   = stream.Endpoint([("data", 8*n), ("ctrl", n)])
        self.source = stream.Endpoint([("data", 8*n), ("ctrl", n)])

        # # #

        self.comb += self.sink.connect(self.source)
        checker = RX8b10bChecker(serdes.decoders)
        if clock_domain != "sys":
            checker = ClockDomainsRenamer(clock_domain)(checker)
        self.checker = checker
        for i in range(len(serdes.decoders)):
            error = serdes.decoders[i].invalid | checker.invalid[i]
            if with_disparity:
                error = error | checker.disparity_error[i]
            self.comb += [
                If(error,
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

    with_cdc=False: sys and clock_domain are the same clock (ex: SerDes logic renamed to the
    transceiver's TX clock domain), no clock domain crossing.
    """
    def __init__(self, clock_domain="sys", phy_dw=16, with_cdc=True):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])

        # # #

        # Input register (registered ready: the PIPE TX logic isolated from the SKP insertion).
        self.sink_buffer = sink_buffer = stream.Buffer([("data", 32), ("ctrl", 4)], pipe_ready=True)

        # Clock compensation
        self.skip_inserter = skip_inserter = TXSKPInserter()
        self.skp_enable    = skip_inserter.enable

        # Clock domain crossing (registered FIFO write and ready: timings).
        buffer = stream.Buffer([("data", 32), ("ctrl", 4)], pipe_ready=True)
        self.buffer = buffer
        if with_cdc:
            cdc = AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
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
            self.sink.connect(sink_buffer.sink),
            sink_buffer.source.connect(skip_inserter.sink),
            skip_inserter.source.connect(buffer.sink),
            converter.source.connect(self.source)
        ]
        if with_cdc:
            self.comb += [
                buffer.source.connect(cdc.sink),
                cdc.source.connect(converter.sink),
            ]
        else:
            self.comb += buffer.source.connect(converter.sink)

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

        # Clock domain crossing (registered FIFO write and ready: timings).
        buffer = ClockDomainsRenamer(clock_domain)(
            stream.Buffer([("data", 32), ("ctrl", 4)], pipe_ready=True))
        cdc    = AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
        cdc    = ClockDomainsRenamer({"write": clock_domain, "read": "sys"})(cdc)
        self.buffer = buffer
        self.cdc    = cdc

        # Words alignment
        word_aligner = RXWordAligner()
        word_aligner = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(word_aligner)
        self.word_aligner = word_aligner

        # Flow
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(skip_remover.sink),
            skip_remover.source.connect(buffer.sink),
            # No back-pressure on the received words (transceiver RX at the line rate, words
            # dropped if the CDC FIFO is full): shorter RX paths.
            buffer.source.connect(cdc.sink, omit={"ready"}),
            buffer.source.ready.eq(1),
            cdc.source.connect(word_aligner.sink),
            word_aligner.source.connect(self.source),
        ]

# Gen 2 SKP Ordered Sets (6.4.3.3) -----------------------------------------------------------------

GEN2_SKP_INTERVAL = 40 # Average interval between SKP Ordered Sets (blocks, 6.4.3.3).

# Gen 2 TX -----------------------------------------------------------------------------------------

def _symbols(symbols):
    return Cat(*[Constant(s, 8) for s in symbols])

class Gen2TX(LiteXModule):
    """Gen 2 block transmitter (one block per cycle on source).

    Controls (levels, one at a time): tx_tseq/tx_ts1/tx_ts2 (Ordered Sets, a SYNC Ordered Set first
    and then every sync_interval_ts TS1/TS2 or sync_interval_tseq TSEQ, 6.4.1.2.1) or tx_data (SDS
    Ordered Set then Data Blocks from sink, Logical Idle Data Blocks (5Ah Idle Symbols, 7.2.3) when
    sink has no data).

    The TSEQ/TS1/TS2 blocks are output with their scrambled Symbols 14/15 (ts flag): the DC Balance
    Symbols are applied by Gen2TXDCBalance (dc_reset: SDS/Data Blocks reset the running balance).

    SKP Ordered Sets (6.4.3.3): Y counts the non SKP blocks (reset while sending TSEQ: Y restarts on
    Polling.Active entry), Z = Y/skp_interval accumulated (2 max) and inserted after SYNC, TS1, TS2,
    SDS, Logical Idle and the last Data Block of a packet (sink.last), never inside a packet. SYNC has
    priority over SKP.
    """
    def __init__(self, seed=GEN2_LFSR_SEED, skp_interval=GEN2_SKP_INTERVAL, sync_interval_ts=32,
        sync_interval_tseq=16384):
        self.enable      = Signal()  # i
        self.tx_tseq     = Signal()  # i
        self.tx_ts1      = Signal()  # i
        self.tx_ts2      = Signal()  # i
        self.tx_data     = Signal()  # i
        self.link_config = Signal(8) # i: TS1/TS2 Link Functionality (Table 6-6).
        self.tx_os       = Signal()  # o: TSEQ/TS1/TS2 Ordered Set sent (pulse).
        self.tx_idle     = Signal()  # o: Logical Idle Data Block sent (pulse).
        self.tx_skp      = Signal()  # o: SKP Ordered Set sent (pulse, debug).
        self.skp_pending = Signal()  # o: SKP Ordered Set(s) scheduled (inserted after a sink.last
                                     #    Data Block or a Logical Idle block).
        self.tx_data_blk = Signal()  # o: Data Block with sink data sent (pulse, debug).

        self.sink   = sink   = stream.Endpoint([("data", 128)])
        self.source = source = stream.Endpoint(gen2_tx_block_layout())

        # # #

        self.scrambler = scrambler = Gen2Scrambler(nbytes=16, seed=seed, per_byte_advance=False)

        # Block selection --------------------------------------------------------------------------
        # SYNC/SKP selection conditions registered (sync_zero, skp_go: timings).
        tx_os        = Signal()
        sync_count   = Signal(max=max(sync_interval_ts, sync_interval_tseq))
        sync_zero    = Signal(reset=1) # sync_count == 0.
        sync_period  = Signal.like(sync_count)
        y            = Signal(max=skp_interval)
        skp_pending  = Signal(2)
        skp_allowed  = Signal()
        skp_next     = Signal(2) # skp_pending/skp_allowed after this cycle.
        allowed_next = Signal()
        skp_go       = Signal()  # (skp_pending != 0) & skp_allowed.
        sds_sent     = Signal()
        self.comb += [
            tx_os.eq(self.tx_tseq | self.tx_ts1 | self.tx_ts2),
            sync_period.eq(Mux(self.tx_tseq, sync_interval_tseq - 1, sync_interval_ts - 1)),
        ]

        sel_sync = Signal()
        sel_skp  = Signal()
        sel_sds  = Signal()
        sel_data = Signal()
        sel_ts   = Signal()
        self.comb += [
            If(self.enable,
                If(tx_os & sync_zero,
                    sel_sync.eq(1)
                ).Elif(skp_go,
                    sel_skp.eq(1)
                ).Elif(self.tx_data,
                    If(~sds_sent,
                        sel_sds.eq(1)
                    ).Else(
                        sel_data.eq(1)
                    )
                ).Elif(tx_os,
                    sel_ts.eq(1)
                )
            )
        ]

        # Payload / scrambling modes (6.3.2.3) -----------------------------------------------------
        ts_identifier = Signal(8)
        self.comb += [
            If(self.tx_tseq,
                ts_identifier.eq(GEN2_TSEQ)
            ).Elif(self.tx_ts1,
                ts_identifier.eq(GEN2_TS1)
            ).Else(
                ts_identifier.eq(GEN2_TS2)
            )
        ]
        ts_payload = Signal(128)
        self.comb += ts_payload.eq(Cat(
            Replicate(ts_identifier, 4),
            Constant(0, 8),
            Mux(self.tx_tseq, 0, self.link_config),
            Replicate(ts_identifier, 10),
        ))
        idle = Signal()
        self.comb += idle.eq(sel_data & ~sink.valid)
        self.comb += [
            source.valid.eq(sel_sync | sel_skp | sel_sds | sel_data | sel_ts),
            scrambler.ce.eq(source.valid & source.ready),
            If(sel_sync,
                # All Symbols bypass, LFSR re-initialized after the SYNC Ordered Set.
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(_symbols(gen2_sync_payload())),
                scrambler.init.eq(source.ready),
            ).Elif(sel_skp,
                # Bypass, no advance.
                source.header.eq(BLOCK_HEADER_CONTROL),
                source.skp.eq(1),
            ).Elif(sel_sds,
                # Bypass, advance.
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(_symbols(gen2_sds_payload())),
                scrambler.advance.eq(1),
            ).Elif(sel_data,
                # Scrambled, advance (Logical Idle: 5Ah Idle Symbols in Gen 2 operation, 7.2.3).
                source.header.eq(BLOCK_HEADER_DATA),
                scrambler.i.eq(Mux(idle, GEN2_IDLE_BLOCK, sink.data)),
                scrambler.advance.eq(1),
                scrambler.scramble.eq(0xffff),
                sink.ready.eq(source.ready),
            ).Elif(sel_ts,
                # Symbol 0 bypass/advance, Symbols 1-15 scrambled (14/15 DC Balance override below).
                source.header.eq(BLOCK_HEADER_CONTROL),
                scrambler.i.eq(ts_payload),
                scrambler.advance.eq(1),
                scrambler.scramble.eq(0xfffe),
            )
        ]

        # Output -----------------------------------------------------------------------------------
        lfsr = scrambler.state
        self.comb += [
            If(sel_skp,
                source.data.eq(Cat(
                    _symbols([GEN2_SKP]*GEN2_SKP_TX_SYMBOLS + [GEN2_SKPEND]),
                    lfsr[16:23], ~lfsr[22], lfsr[8:16], lfsr[0:8]))
            ).Else(
                source.data.eq(scrambler.o)
            ),
            source.ts.eq(sel_ts),
            source.dc_reset.eq(sel_sds | sel_data),
            self.tx_os.eq(sel_ts & source.ready),
            self.tx_idle.eq(idle & source.ready),
            self.skp_pending.eq(skp_pending != 0),
            self.tx_skp.eq(sel_skp & source.ready),
            self.tx_data_blk.eq(sel_data & ~idle & source.ready),
        ]

        # State ------------------------------------------------------------------------------------
        # SKP scheduling.
        self.comb += [
            skp_next.eq(skp_pending),
            allowed_next.eq(skp_allowed),
            If(~self.enable,
                skp_next.eq(0),
                allowed_next.eq(0),
            ).Elif(source.valid & source.ready,
                If(~sel_skp,
                    If(~(self.tx_tseq & ~self.tx_data) & (y == (skp_interval - 1)) &
                        (skp_pending != 2),
                        skp_next.eq(skp_pending + 1)
                    ),
                    allowed_next.eq(sel_sync | sel_sds | (sel_ts & ~self.tx_tseq) | idle |
                        (sel_data & sink.last)),
                ).Else(
                    skp_next.eq(skp_pending - 1)
                )
            )
        ]
        self.sync += [
            skp_pending.eq(skp_next),
            skp_allowed.eq(allowed_next),
            skp_go.eq((skp_next != 0) & allowed_next),
        ]
        self.sync += [
            If(~self.enable,
                sync_count.eq(0),
                sync_zero.eq(1),
                y.eq(0),
                sds_sent.eq(0),
            ).Elif(source.valid & source.ready,
                # SYNC scheduling (TS Ordered Sets count).
                If(sel_ts,
                    If(sync_count == sync_period,
                        sync_count.eq(0),
                        sync_zero.eq(1),
                    ).Else(
                        sync_count.eq(sync_count + 1),
                        sync_zero.eq(0),
                    )
                ).Elif(sel_sync,
                    sync_count.eq(1),
                    sync_zero.eq(0),
                ),
                # SKP interval.
                If(~sel_skp,
                    If(self.tx_tseq & ~self.tx_data,
                        y.eq(0)
                    ).Elif(y == (skp_interval - 1),
                        y.eq(0),
                    ).Else(
                        y.eq(y + 1)
                    ),
                ),
                # Data Stream.
                If(sel_sds,
                    sds_sent.eq(1)
                ).Elif(sel_sync | sel_ts,
                    sds_sent.eq(0)
                ),
            ),
            If(~self.tx_data & ~tx_os, sds_sent.eq(0)),
        ]

# Logical Idle Data Block payload (16 Idle Symbols).
GEN2_IDLE_BLOCK = int.from_bytes(bytes([GEN2_IDLE])*16, "little")

def gen2_tx_block_layout():
    """Gen2TX blocks: gen2_block_layout + TSEQ/TS1/TS2 (ts, DC Balance Symbols to apply) and running
    DC Balance reset (dc_reset: SDS/Data Blocks)."""
    return gen2_block_layout() + [("ts", 1), ("dc_reset", 1)]

# Gen 2 TX DC Balance ------------------------------------------------------------------------------

class Gen2TXDCBalance(LiteXModule):
    """DC Balance Symbols of the transmitted TSEQ/TS1/TS2 (6.4.1.2.2), on the Gen2TX blocks.

    Running DC Balance (1s - 0s) of the transmitted TSEQ/TS1/TS2 bits (Block Header included),
    saturated at +-511, reset at the start of the Data Blocks transmission (dc_reset). Evaluated at
    the end of Symbol 11: > 31: DC Balance Symbols 14 and 15, > 15: Symbol 15 only (the Gen2TX LFSR
    still advances on them).

    Pipelined over 3 cycles per block (popcounts, balance, output/update): the gearbox accepts a block
    every 3 cycles at most at 40-bit (132-bit blocks), the running balance loop is then a small adder
    and saturation. one_per_cycle: one block per cycle (wider transceiver words, ex: 80-bit at
    125MHz: a block every 1.65 cycles), the block registered with partial popcounts, then the
    balance increments of the possible DC Balance Symbols precomputed, the running balance loop is
    then two adders, a selection and the saturation.
    """
    def __init__(self, one_per_cycle=False):
        self.sink   = sink   = stream.Endpoint(gen2_tx_block_layout())
        self.source = source = stream.Endpoint(gen2_block_layout())

        # # #

        self.dc_balance = dc_balance = Signal((11, True))
        if one_per_cycle:
            self.add_one_per_cycle(dc_balance)
            return

        # Stage 1: block and popcounts (Symbols 0-11 with the Block Header, 12-13, 14, 15).
        header   = Signal(4)
        data     = Signal(192)
        skp      = Signal()
        ts       = Signal()
        dc_reset = Signal()
        ones_11  = Signal((8, True)) # Signed: balance arithmetic.
        ones_13  = Signal((6, True))
        ones_14  = Signal((5, True))
        ones_15  = Signal((5, True))
        # Stage 2: balance at the end of Symbol 11, DC Balance Symbols selection.
        balance  = Signal((12, True))
        fix_14   = Signal()
        fix_15   = Signal()
        positive = Signal()

        self.fsm = fsm = FSM(reset_state="CAPTURE")
        fsm.act("CAPTURE",
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(header,   sink.header),
                NextValue(data,     sink.data),
                NextValue(skp,      sink.skp),
                NextValue(ts,       sink.ts),
                NextValue(dc_reset, sink.dc_reset),
                NextValue(ones_11, popcount(Cat(sink.header, sink.data[0:96]))),
                NextValue(ones_13, popcount(sink.data[96:112])),
                NextValue(ones_14, popcount(sink.data[112:120])),
                NextValue(ones_15, popcount(sink.data[120:128])),
                NextState("BALANCE")
            )
        )
        new_balance = Signal((12, True))
        fsm.act("BALANCE",
            NextValue(balance, dc_balance + 2*ones_11 - 100),
            NextState("OUTPUT")
        )
        magnitude = Signal(11)
        self.comb += [
            magnitude.eq(Mux(balance < 0, -balance, balance)),
            positive.eq(balance > 0),
            fix_14.eq(magnitude > 31),
            fix_15.eq(magnitude > 15),
        ]
        sym14 = Signal(8)
        sym15 = Signal(8)
        o14   = Signal((5, True))
        o15   = Signal((5, True))
        def dc_ones(i):
            return Mux(positive, bin(GEN2_DC_REDUCE_ONES[i]).count("1"),
                                 bin(GEN2_DC_REDUCE_ZEROS[i]).count("1"))
        self.comb += [
            sym14.eq(Mux(fix_14, Mux(positive, GEN2_DC_REDUCE_ONES[0], GEN2_DC_REDUCE_ZEROS[0]),
                data[112:120])),
            sym15.eq(Mux(fix_15, Mux(positive, GEN2_DC_REDUCE_ONES[1], GEN2_DC_REDUCE_ZEROS[1]),
                data[120:128])),
            o14.eq(Mux(fix_14, dc_ones(0), ones_14)),
            o15.eq(Mux(fix_15, dc_ones(1), ones_15)),
            # dc_balance + 2*ones(block) - 132.
            new_balance.eq(balance + 2*(ones_13 + o14 + o15) - 32),
        ]
        fsm.act("OUTPUT",
            source.valid.eq(1),
            source.header.eq(header),
            source.skp.eq(skp),
            If(ts,
                source.data.eq(Cat(data[0:112], sym14, sym15, data[128:]))
            ).Else(
                source.data.eq(data)
            ),
            If(source.ready,
                If(ts,
                    If(new_balance > 511,
                        NextValue(dc_balance, 511)
                    ).Elif(new_balance < -511,
                        NextValue(dc_balance, -511)
                    ).Else(
                        NextValue(dc_balance, new_balance)
                    )
                ).Elif(dc_reset,
                    NextValue(dc_balance, 0)
                ),
                NextState("CAPTURE")
            )
        )

    def add_one_per_cycle(self, dc_balance):
        sink   = self.sink
        source = self.source
        ones   = lambda v: bin(v).count("1")

        # Stage 0: block and partial popcounts (Block Header + Symbols 0-11 in 8 chunks, 12-13, 14,
        # 15).
        v0        = Signal()
        header0   = Signal(4, reset_less=True)
        data0     = Signal(192, reset_less=True)
        skp0      = Signal(reset_less=True)
        ts0       = Signal(reset_less=True)
        dc_reset0 = Signal(reset_less=True)
        ones_11   = [Signal(4, reset_less=True) for _ in range(8)]
        ones_13   = Signal(5, reset_less=True)
        ones_14   = Signal(4, reset_less=True)
        ones_15   = Signal(4, reset_less=True)
        ready1    = Signal()
        self.comb += sink.ready.eq(~v0 | ready1)
        bits_11 = Cat(sink.header, sink.data[0:96])
        self.sync += If(sink.ready,
            v0.eq(sink.valid),
            header0.eq(sink.header),
            data0.eq(sink.data),
            skp0.eq(sink.skp),
            ts0.eq(sink.ts),
            dc_reset0.eq(sink.dc_reset),
            [ones_11[i].eq(popcount(bits_11[13*i:min(13*(i+1), 100)])) for i in range(8)],
            ones_13.eq(popcount(sink.data[96:112])),
            ones_14.eq(popcount(sink.data[112:120])),
            ones_15.eq(popcount(sink.data[120:128])),
        )

        # Stage 1: block, balance increment at the end of Symbol 11 and the balance increments of
        # Symbols 12-15 for each DC Balance Symbols choice (none, Symbol 15, Symbols 14/15; reduce
        # ones/zeros).
        v1       = Signal()
        header   = Signal(4, reset_less=True)
        data     = Signal(192, reset_less=True)
        skp      = Signal(reset_less=True)
        ts       = Signal(reset_less=True)
        dc_reset = Signal(reset_less=True)
        inc_11   = Signal((9, True), reset_less=True)
        inc      = {k: Signal((8, True), reset_less=True)
            for k in ["none", "15p", "15n", "14p", "14n"]}
        advance  = Signal()
        self.comb += ready1.eq(~v1 | advance)
        self.sync += If(ready1,
            v1.eq(v0),
            header.eq(header0),
            data.eq(data0),
            skp.eq(skp0),
            ts.eq(ts0),
            dc_reset.eq(dc_reset0),
            inc_11.eq(2*(((ones_11[0] + ones_11[1]) + (ones_11[2] + ones_11[3])) +
                         ((ones_11[4] + ones_11[5]) + (ones_11[6] + ones_11[7]))) - 100),
            inc["none"].eq(2*(ones_13 + ones_14 + ones_15) - 32),
            inc["15p"].eq(2*(ones_13 + ones_14 + ones(GEN2_DC_REDUCE_ONES[1])) - 32),
            inc["15n"].eq(2*(ones_13 + ones_14 + ones(GEN2_DC_REDUCE_ZEROS[1])) - 32),
            inc["14p"].eq(2*(ones_13 + ones(GEN2_DC_REDUCE_ONES[0]) + ones(GEN2_DC_REDUCE_ONES[1])) - 32),
            inc["14n"].eq(2*(ones_13 + ones(GEN2_DC_REDUCE_ZEROS[0]) + ones(GEN2_DC_REDUCE_ZEROS[1])) - 32),
        )

        # Stage 2: thresholds of the running balance (DC Balance Symbols choice, balance at the end of
        # Symbol 11 = dc_balance + inc_11: > 0, > 15, < -15, > 31, < -31) and the update of each choice
        # (inc_11 + inc[k]) with its saturation thresholds (+-511): the running balance loop is then
        # comparisons/adders with registered values and a selection.
        v2        = Signal()
        header2   = Signal(4, reset_less=True)
        data2     = Signal(192, reset_less=True)
        skp2      = Signal(reset_less=True)
        ts2       = Signal(reset_less=True)
        dc_reset2 = Signal(reset_less=True)
        th        = {k: Signal((12, True), reset_less=True)
            for k in ["pos", "15p", "15n", "14p", "14n"]}
        upd       = {k: Signal((12, True), reset_less=True) for k in inc}
        sat_hi    = {k: Signal((12, True), reset_less=True) for k in inc}
        sat_lo    = {k: Signal((12, True), reset_less=True) for k in inc}
        ready2    = Signal()
        self.comb += advance.eq(v1 & ready2)
        self.sync += If(ready2,
            v2.eq(v1),
            header2.eq(header),
            data2.eq(data),
            skp2.eq(skp),
            ts2.eq(ts),
            dc_reset2.eq(dc_reset),
            th["pos"].eq(  0 - inc_11),
            th["15p"].eq( 15 - inc_11),
            th["15n"].eq(-15 - inc_11),
            th["14p"].eq( 31 - inc_11),
            th["14n"].eq(-31 - inc_11),
            [upd[k].eq(inc_11 + inc[k]) for k in inc],
            [sat_hi[k].eq( 511 - (inc_11 + inc[k])) for k in inc],
            [sat_lo[k].eq(-511 - (inc_11 + inc[k])) for k in inc],
        )

        # Stage 3: DC Balance Symbols, running balance update, output. Running balance copies for the
        # thresholds and the updates (fanout).
        dcb_th   = Signal((11, True), attr={("syn_preserve", 1)})
        dcb_upd  = Signal((11, True), attr={("syn_preserve", 1)})
        positive = Signal()
        fix_14   = Signal()
        fix_15   = Signal()
        value    = {k: Signal((12, True)) for k in inc} # Saturated running balance of each choice.
        updated  = Signal((12, True))
        self.comb += [
            positive.eq(dcb_th > th["pos"]),
            fix_14.eq((dcb_th > th["14p"]) | (dcb_th < th["14n"])),
            fix_15.eq((dcb_th > th["15p"]) | (dcb_th < th["15n"])),
            [If(dcb_upd > sat_hi[k],
                value[k].eq(511)
            ).Elif(dcb_upd < sat_lo[k],
                value[k].eq(-511)
            ).Else(
                value[k].eq(dcb_upd + upd[k])
            ) for k in inc],
            If(fix_14,
                updated.eq(Mux(positive, value["14p"], value["14n"]))
            ).Elif(fix_15,
                updated.eq(Mux(positive, value["15p"], value["15n"]))
            ).Else(
                updated.eq(value["none"])
            ),
        ]
        sym14 = Mux(fix_14, Mux(positive, GEN2_DC_REDUCE_ONES[0], GEN2_DC_REDUCE_ZEROS[0]), data2[112:120])
        sym15 = Mux(fix_15, Mux(positive, GEN2_DC_REDUCE_ONES[1], GEN2_DC_REDUCE_ZEROS[1]), data2[120:128])
        advance2 = Signal()
        self.comb += [
            ready2.eq(~v2 | advance2),
            advance2.eq(v2 & (~source.valid | source.ready)),
        ]
        self.sync += [
            If(source.ready, source.valid.eq(0)),
            If(advance2,
                source.valid.eq(1),
                source.header.eq(header2),
                source.skp.eq(skp2),
                If(ts2,
                    source.data.eq(Cat(data2[0:112], sym14, sym15, data2[128:]))
                ).Else(
                    source.data.eq(data2)
                ),
                If(ts2,
                    dc_balance.eq(updated),
                    dcb_th.eq(updated),
                    dcb_upd.eq(updated),
                ).Elif(dc_reset2,
                    dc_balance.eq(0),
                    dcb_th.eq(0),
                    dcb_upd.eq(0),
                ),
            ),
        ]

# Gen 2 RX -----------------------------------------------------------------------------------------

class Gen2RX(LiteXModule):
    """Gen 2 block receiver (from Gen2BlockAligner).

    Descrambling (6.3.2.3): Data Blocks advance/descramble, control blocks advance the LFSR (Symbol 0
    bypass), SYNC Ordered Sets re-initialize it (SKP Ordered Sets are removed by the aligner and do not
    advance it). TSEQ/TS1/TS2 detection on Symbols 0-13 (Symbols 14/15 not checked, 6.4.1.2.2, Link
    Functionality excluded): rx_tseq/rx_ts1/rx_ts2 pulse every n_ts consecutive Ordered Sets (SKP and
    SYNC Ordered Sets are not interruptions), tseq/ts1/ts2/sync/sds/unknown pulse on each received
    control block. The Data Blocks following an SDS are output on source.

    Pipeline (250MHz with one block every 3.3 cycles): descrambling, then the Ordered Sets matching
    (registered), the outputs follow the input blocks by 1 (source, rx_sync, rx_sds) to 3 cycles.
    """
    def __init__(self, seed=GEN2_LFSR_SEED, n_ts=8):
        self.sink   = sink   = stream.Endpoint([("header", 4), ("data", 128), ("header_error", 1)])
        self.source = source = stream.Endpoint([("data", 128)])

        self.rx_tseq     = Signal()   # o: n_ts consecutive TSEQ received (pulse).
        self.rx_ts1      = Signal()   # o: n_ts consecutive TS1 received (pulse).
        self.rx_ts2      = Signal()   # o: n_ts consecutive TS2 received (pulse).
        self.rx_sync     = Signal()   # o: SYNC Ordered Set received (pulse).
        self.rx_sds      = Signal()   # o: SDS Ordered Set received (pulse).
        self.tseq        = Signal()   # o: TSEQ Ordered Set received (pulse).
        self.ts1         = Signal()   # o: TS1 Ordered Set received (pulse).
        self.ts2         = Signal()   # o: TS2 Ordered Set received (pulse).
        self.unknown     = Signal()   # o: Unknown control block received (pulse).
        self.link_config = Signal(8)  # o: Link Functionality of the last TS1/TS2.
        self.data_stream = Signal()   # o: Receiving the Data Stream (after an SDS).
        self.lfsr        = Signal(23) # o: Descrambler LFSR state.

        # # #

        # Descrambling -----------------------------------------------------------------------------
        self.descrambler = descrambler = Gen2Scrambler(nbytes=16, seed=seed, per_byte_advance=False)
        self.comb += self.lfsr.eq(descrambler.state)

        control = Signal()
        sym0    = Signal(8)
        is_sync = Signal()
        is_sds  = Signal()
        self.comb += [
            sink.ready.eq(1),
            control.eq(sink.header == BLOCK_HEADER_CONTROL),
            sym0.eq(sink.data[0:8]),
            is_sync.eq(control & (sink.data == symbols_to_int(gen2_sync_payload()))),
            is_sds.eq( control & (sym0 == GEN2_SDS)),
            descrambler.ce.eq(sink.valid),
            descrambler.i.eq(sink.data),
            If(control,
                If(is_sync,
                    descrambler.init.eq(sink.valid)
                ).Elif(is_sds,
                    descrambler.advance.eq(1)
                ).Else(
                    descrambler.advance.eq(1),
                    descrambler.scramble.eq(0xfffe)
                )
            ).Else(
                descrambler.advance.eq(1),
                descrambler.scramble.eq(0xffff)
            )
        ]

        # Descrambled block (registered).
        valid   = Signal()
        ctl     = Signal()
        sync    = Signal()
        sds     = Signal()
        o       = Signal(128)
        self.sync += [
            valid.eq(sink.valid),
            ctl.eq(control),
            sync.eq(is_sync),
            sds.eq(is_sds),
            o.eq(descrambler.o),
        ]
        self.comb += [
            source.valid.eq(valid & ~ctl & self.data_stream),
            source.data.eq(o),
            self.rx_sync.eq(valid & sync),
            self.rx_sds.eq(valid & sds),
        ]
        self.sync += [
            If(valid & ctl,
                self.data_stream.eq(sds)
            )
        ]

        # Training Sequences detection -------------------------------------------------------------
        def ts_match(identifier, with_link_config):
            sym = [o[8*i:8*(i+1)] for i in range(14)]
            checks = [sym[i] == identifier for i in list(range(4)) + list(range(6, 14))]
            checks.append(sym[4] == 0)
            if not with_link_config:
                checks.append(sym[5] == 0)
            return reduce(and_, checks)
        valid_d = Signal()
        sync_d  = Signal()
        lc_d    = Signal(8)
        self.sync += [
            valid_d.eq(valid),
            sync_d.eq(sync),
            lc_d.eq(o[40:48]),
        ]
        matches = []
        for name, identifier in [("tseq", GEN2_TSEQ), ("ts1", GEN2_TS1), ("ts2", GEN2_TS2)]:
            m     = getattr(self, name)
            count = Signal(max=n_ts, name=f"{name}_count")
            pulse = getattr(self, f"rx_{name}")
            match = ts_match(identifier, name != "tseq")
            self.sync += [
                m.eq(valid & ctl & ~sync & ~sds & match),
                pulse.eq(0),
                If(m,
                    If(count == (n_ts - 1),
                        count.eq(0),
                        pulse.eq(1)
                    ).Else(
                        count.eq(count + 1)
                    )
                ).Elif(valid_d & ~sync_d,
                    count.eq(0)
                )
            ]
            if name != "tseq":
                self.sync += If(m, self.link_config.eq(lc_d))
            matches.append(match)
        self.sync += self.unknown.eq(valid & ctl & ~sync & ~sds & ~reduce(or_, matches))

# Gen 2 TX/RX Datapaths ----------------------------------------------------------------------------

# Gen 2 transceiver words: 40-bit at 250MHz for 10Gbps (raw data, transceiver 8b/10b bypassed), same
# TX/RX clocks as Gen 1 (20-bit at 250MHz for 5Gbps).
GEN2_DATA_WIDTH = 40

class Gen2TXDatapath(LiteXModule):
    """Gen 2 TX datapath: Gen2TX -> Gen2TXDCBalance -> Gen2BlockGearbox: continuous data_width-bit
    transceiver words (bit 0 transmitted first), in the transceiver TX clock domain. Controls/sink
    on tx (Gen2TX)."""
    def __init__(self, data_width=GEN2_DATA_WIDTH, **kwargs):
        self.tx         = tx         = Gen2TX(**kwargs)
        # One block per cycle in the DC balance when the words need a block more often than every 3
        # cycles (132-bit blocks, ex: 80-bit words).
        self.dc_balance = dc_balance = Gen2TXDCBalance(one_per_cycle=(3*data_width > 132))
        self.gearbox    = gearbox    = Gen2BlockGearbox(data_width)
        self.sink    = tx.sink
        self.source  = gearbox.source

        # # #

        if 3*data_width > 132:
            # Registered valid/ready between Gen2TX and the DC balance and between the DC balance and
            # the gearbox (timings: the Gen2TX block selection and the gearbox/DC balance readies at
            # one block per cycle).
            self.buffer      = buffer      = Buffer(gen2_tx_block_layout(), pipe_valid=True,
                pipe_ready=True)
            self.gear_buffer = gear_buffer = Buffer(gen2_block_layout(), pipe_valid=True,
                pipe_ready=True, ready_first=True) # Registered blocks to the gearbox placement.
            self.comb += [
                tx.source.connect(buffer.sink),
                buffer.source.connect(dc_balance.sink),
                dc_balance.source.connect(gear_buffer.sink),
                gear_buffer.source.connect(gearbox.sink),
            ]
        else:
            self.comb += [
                tx.source.connect(dc_balance.sink),
                dc_balance.source.connect(gearbox.sink),
            ]

GEN2_RX_COUNTERS = [
    # (name, description).
    ("blocks",        "Blocks (SKP Ordered Sets excluded)."),
    ("header_errors", "Block Headers with a corrected bit error."),
    ("align_errors",  "Alignment losses (undefined Block Header/SKP Ordered Set)."),
    ("sync",          "SYNC Ordered Sets."),
    ("tseq",          "TSEQ Ordered Sets."),
    ("ts1",           "TS1 Ordered Sets."),
    ("ts2",           "TS2 Ordered Sets."),
    ("sds",           "SDS Ordered Sets."),
    ("skp",           "SKP Ordered Sets."),
    ("unknown",       "Unknown control blocks."),
]

class Gen2RXDatapath(LiteXModule):
    """Gen 2 RX datapath: data_width-bit transceiver words (bit 0 received first, continuous) ->
    Gen2BlockAligner (SYNC block alignment, polarity, SKP Ordered Sets) -> Gen2RX (descrambling,
    Ordered Sets detection), in the transceiver RX clock domain.

    Status (aligned/locked/polarity/data_stream/link_config) and 32-bit event counters
    (GEN2_RX_COUNTERS, `{name}_count`, cleared with clear) for the hardware bring-up (Gen2RXDebug).
    Data Blocks (after an SDS) on source, Training Sequences pulses on rx (Gen2RX).
    """
    def __init__(self, data_width=GEN2_DATA_WIDTH, n_ts=8):
        self.sink   = sink   = stream.Endpoint([("data", data_width)])
        self.source = source = stream.Endpoint([("data", 128)])
        self.clear  = Signal() # i: Counters clear.

        # # #

        self.aligner = aligner = Gen2BlockAligner(data_width)
        self.rx      = rx      = Gen2RX(n_ts=n_ts)
        self.comb += [
            sink.connect(aligner.sink),
            aligner.source.connect(rx.sink),
            rx.source.connect(source),
        ]

        # Status.
        self.aligned     = aligner.aligned
        self.locked      = aligner.locked
        self.polarity    = aligner.polarity
        self.data_stream = rx.data_stream
        self.link_config = rx.link_config

        # Counters.
        events = {
            "blocks"        : aligner.source.valid,
            "header_errors" : aligner.source.valid & aligner.source.header_error,
            "align_errors"  : aligner.error,
            "sync"          : rx.rx_sync,
            "tseq"          : rx.tseq,
            "ts1"           : rx.ts1,
            "ts2"           : rx.ts2,
            "sds"           : rx.rx_sds,
            "skp"           : aligner.skp,
            "unknown"       : rx.unknown,
        }
        for name, _ in GEN2_RX_COUNTERS:
            count = Signal(32, name=f"{name}_count")
            setattr(self, f"{name}_count", count)
            self.sync += If(self.clear,
                count.eq(0)
            ).Elif(events[name],
                count.eq(count + 1)
            )
