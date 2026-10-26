#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Logic Analyzer (doc/agentlab.md, phase 2): N channels (<= 32) sampled at
sys_clk/divider, triggers, pre-trigger, DRAM ring buffer, capture or continuous streaming to a USB3
Bulk IN endpoint.

Datapath:

    inputs -> MultiReg -> sampler (divider, trigger) -> 32-bit to DRAM words -> sample FIFO ->
    DMA writer (ring buffer) ... DMA reader -> DRAM words to 32-bit -> skip/length filter ->
    output FIFO -> framer (capture header, blocks) -> word packer -> source (Bulk IN).

- From arm, every sample is written to the ring buffer (sample k at ring position k, modulo the
  ring), the trigger is searched on each sample. On the trigger (sample t), the capture starts at
  sample s = t - min(pre_trigger, t) and continues length - pre_trigger samples from the trigger
  (trigger sample included).
- Capture mode: the readout starts when the capture ends (exact sample count in the header), the
  ring buffer holds the whole capture (length <= ring samples).
- Stream mode: the readout starts on the trigger, the ring buffer is used as a FIFO (DRAM absorbs
  the USB stalls). A full ring (host not keeping up) ends the capture (overflow flagged), length 0
  streams until stop.
- After the trigger, the writer never overwrites the samples not yet read (ring full: samples lost,
  capture ended, overflow).

Samples are 32-bit words (channel n on bit n, unused bits 0). Every sample k is taken
divider sys_clk cycles after sample k-1: timestamp(k) = start_timestamp + k*divider (timestamps of
the synchronized inputs, 2 sys_clk cycles after the pins).

Stream format (32-bit little-endian words), per capture:

    Header (LA_HEADER_WORDS words):
        0     : magic 0x31414c41 ("ALA1")
        1     : version (bits 0-7), channels (8-15), mode (16-23), flags (24-31, bit 0: overflow)
        2     : divider (samplerate = sys_clk_freq/divider)
        3     : block samples (B)
        4     : trigger offset (samples from the capture start)
        5     : reserved (0)
        6-7   : sample count (capture mode: exact, stream mode: expected, 0: until stop)
        8-9   : start timestamp (sys_clk ticks, sample 0)
        10-11 : trigger timestamp (sys_clk ticks)
    Blocks, until the last one:
        word  : count (bits 0-15), flags (16-23, bit 0: last block, bit 1: overflow), 0xb1 (24-31)
        count sample words (B, last block: <= B, can be 0).
"""

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

from litedram.frontend.dma import LiteDRAMDMAWriter, LiteDRAMDMAReader

from agentlab.gateware.events import AgentLabWordPacker

# Constants ----------------------------------------------------------------------------------------

LA_MAGIC        = 0x31414c41 # "ALA1" (little-endian bytes).
LA_VERSION      = 1
LA_HEADER_WORDS = 12

LA_BLOCK_MAGIC    = 0xb1
LA_BLOCK_LAST     = 0x01
LA_BLOCK_OVERFLOW = 0x02

LA_HEADER_OVERFLOW = 0x01

LA_MODE_CAPTURE = 0
LA_MODE_STREAM  = 1

LA_TRIGGER_IMMEDIATE = 0
LA_TRIGGER_PATTERN   = 1
LA_TRIGGER_EXTERNAL  = 2

LA_EDGE_RISING  = 0
LA_EDGE_FALLING = 1
LA_EDGE_ANY     = 2

LA_EXT_MARKER = 0 # External trigger sources (bits of agentlab_la_trigger_ext).
LA_EXT_GPIO   = 1
LA_EXT_POWER  = 2
LA_EXT_SOURCES = 3

LA_STATE_IDLE      = 0
LA_STATE_ARMED     = 1 # Sampling, waiting for the trigger.
LA_STATE_TRIGGERED = 2 # Sampling after the trigger (stream mode: readout running).
LA_STATE_READOUT   = 3 # Capture ended, readout running.
LA_STATE_DONE      = 4 # Readout done (last block sent).
LA_STATE_FLUSH     = 5 # Arm: previous capture flushed.

LA_COUNTER_BITS = 48 # Samples/DRAM words counters (since arm).

# Logic Analyzer -----------------------------------------------------------------------------------

class AgentLabLA(LiteXModule):
    """Logic analyzer on `inputs` (<= 32 channels, asynchronous), DRAM ring buffer on a native write
    port/read port pair (litedram crossbar ports) at ring_base (bytes from the DRAM start, ring_size
    bytes, power of 2 DRAM words).

    source        : capture stream (32-bit words packets, to a USB3 Bulk IN endpoint sink).
    flush         : pulse on arm (flush the Bulk IN endpoint buffered packets).
    ext_trigger   : external trigger events (pulses, LA_EXT_* bits).
    dram_ready    : DRAM initialized (arm ignored when 0).
    trigger_event : pulse on the trigger (trigger_value: trigger sample), for the events stream.
    """
    def __init__(self, inputs, timestamp, write_port, read_port, ring_base, ring_size,
        block_samples      = 256,
        sample_fifo_depth  = 1024,
        output_fifo_depth  = 1024,
        dma_fifo_depth     = 64,
        idle_cycles        = 128):
        n = len(inputs)
        assert 1 <= n <= 32
        dw         = write_port.data_width
        assert read_port.data_width == dw
        assert dw % 32 == 0
        ratio      = dw//32 # Samples per DRAM word.
        ratio_bits = log2_int(ratio)
        word_bytes = dw//8
        assert ring_base % word_bytes == 0
        assert ring_size % word_bytes == 0
        ring_words = ring_size//word_bytes
        assert ring_words >= 4
        log2_int(ring_words) # Power of 2.
        assert block_samples < 2**16
        assert output_fifo_depth >= block_samples
        self.n             = n
        self.ratio         = ratio
        self.ring_base     = ring_base
        self.ring_size     = ring_size
        self.ring_samples  = ring_words*ratio
        self.block_samples = block_samples

        self.source        = stream.Endpoint([("data", 32), ("be", 4)]) # o
        self.flush         = Signal()                                   # o
        self.ext_trigger   = Signal(LA_EXT_SOURCES)                     # i
        self.dram_ready    = Signal(reset=1)                            # i
        self.trigger_event = Signal()                                   # o
        self.trigger_value = Signal(32)                                 # o

        # CSRs -------------------------------------------------------------------------------------
        self._mode     = CSRStorage(fields=[
            CSRField("mode", size=1, values=[
                ("``0``", "Capture: ring buffer, readout after the capture."),
                ("``1``", "Stream: readout from the trigger, ring buffer used as a FIFO."),
            ]),
        ])
        self._divider  = CSRStorage(32, reset=1,
            description="Samplerate divider: samplerate = sys_clk_freq/divider (0/1: sys_clk_freq).")
        self._trigger_source = CSRStorage(fields=[
            CSRField("source", size=2, values=[
                ("``0``", "Immediate (first sample)."),
                ("``1``", "Pattern: (sample & mask) == (value & mask), and the edge if enabled."),
                ("``2``", "External: events selected by trigger_ext."),
            ]),
        ])
        self._trigger_mask  = CSRStorage(32, description="Pattern trigger: channels compared.")
        self._trigger_value = CSRStorage(32, description="Pattern trigger: channels values.")
        self._trigger_edge  = CSRStorage(fields=[
            CSRField("enable",   size=1, description="Pattern trigger also requires an edge."),
            CSRField("channel",  size=5, description="Edge channel."),
            CSRField("polarity", size=2, values=[
                ("``0``", "Rising."),
                ("``1``", "Falling."),
                ("``2``", "Any."),
            ]),
        ])
        self._trigger_ext = CSRStorage(fields=[
            CSRField("marker", size=1, description="Marker event (agentlab_marker write)."),
            CSRField("gpio",   size=1, description="GPIO edge event (agentlab_gpio_edge_mask)."),
            CSRField("power",  size=1, description="Power switch change."),
        ])
        self._pre_trigger = CSRStorage(32,
            description="Samples before the trigger (<= length, capture mode/stream mode).")
        self._length  = CSRStorage(32, reset=1024,
            description="Samples of the capture (pre-trigger included); stream mode: 0, until stop.")
        self._control = CSRStorage(fields=[
            CSRField("arm",  size=1, pulse=True, description="Arm (restart) a capture."),
            CSRField("stop", size=1, pulse=True,
                description="Stop: armed: abort (no readout), triggered: end of the capture."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("state", size=3, values=[
                ("``0``", "Idle."),
                ("``1``", "Armed (waiting for the trigger)."),
                ("``2``", "Triggered (capturing)."),
                ("``3``", "Readout (capture ended)."),
                ("``4``", "Done (stream sent)."),
                ("``5``", "Flush (arm in progress)."),
            ]),
            CSRField("triggered",  size=1, description="Trigger seen."),
            CSRField("overflow",   size=1, description="Samples lost (capture ended early)."),
            CSRField("dram_ready", size=1, description="DRAM initialized."),
        ])
        self._level             = CSRStatus(32, description="DRAM words buffered (not read).")
        self._count             = CSRStatus(64, description="Samples captured (from the start).")
        self._trigger_offset    = CSRStatus(32, description="Trigger sample (from the start).")
        self._start_timestamp   = CSRStatus(64, description="Timestamp of the first sample.")
        self._trigger_timestamp = CSRStatus(64, description="Timestamp of the trigger sample.")

        # # #

        cbits = LA_COUNTER_BITS

        # Configuration (latched on arm) -----------------------------------------------------------
        mode        = Signal()
        divider     = Signal(32)
        pre         = Signal(32)
        post        = Signal(32) # Samples from the trigger (trigger included).
        unbounded   = Signal()
        arm         = Signal()
        stop        = Signal()
        self.comb += [
            arm.eq(self._control.fields.arm & self.dram_ready),
            stop.eq(self._control.fields.stop),
        ]
        self.sync += If(arm,
            mode.eq(self._mode.fields.mode),
            divider.eq(Mux(self._divider.storage == 0, 1, self._divider.storage)),
            pre.eq(self._pre_trigger.storage),
            post.eq(Mux(self._length.storage > self._pre_trigger.storage,
                self._length.storage - self._pre_trigger.storage, 1)),
            unbounded.eq(self._mode.fields.mode & (self._length.storage == 0)),
        )

        # Datapath reset (flush on arm).
        reset = Signal()

        # Inputs (synchronized) / Sampler ----------------------------------------------------------
        data = Signal(n)
        self.specials += MultiReg(inputs, data)
        sample   = Signal(32)
        previous = Signal(32)
        has_prev = Signal()
        self.comb += sample.eq(data)

        sampling = Signal()
        strobe   = Signal()
        div_cnt  = Signal(32)
        self.comb += strobe.eq(sampling & (div_cnt == 0))
        self.sync += [
            If(sampling,
                div_cnt.eq(div_cnt - 1),
                If(div_cnt == 0, div_cnt.eq(divider - 1)),
            ).Else(
                div_cnt.eq(0),
            ),
            If(strobe,
                previous.eq(sample),
                has_prev.eq(1),
            ),
            If(arm, has_prev.eq(0)),
        ]

        # Trigger ----------------------------------------------------------------------------------
        ext_pending = Signal(LA_EXT_SOURCES)
        ext_now     = Signal(LA_EXT_SOURCES)
        edge_ch     = self._trigger_edge.fields.channel
        ch_now      = Signal()
        ch_prev     = Signal()
        edge_ok     = Signal()
        pattern_ok  = Signal()
        hit         = Signal()
        self.comb += [
            ext_now.eq(ext_pending | (self.ext_trigger & self._trigger_ext.storage)),
            ch_now.eq((sample   >> edge_ch)[0]),
            ch_prev.eq((previous >> edge_ch)[0]),
            Case(self._trigger_edge.fields.polarity, {
                LA_EDGE_RISING  : edge_ok.eq(has_prev & ~ch_prev &  ch_now),
                LA_EDGE_FALLING : edge_ok.eq(has_prev &  ch_prev & ~ch_now),
                "default"       : edge_ok.eq(has_prev & (ch_prev ^ ch_now)),
            }),
            pattern_ok.eq(((sample ^ self._trigger_value.storage) &
                self._trigger_mask.storage) == 0),
            Case(self._trigger_source.fields.source, {
                LA_TRIGGER_IMMEDIATE : hit.eq(1),
                LA_TRIGGER_PATTERN   : hit.eq(pattern_ok &
                    (~self._trigger_edge.fields.enable | edge_ok)),
                LA_TRIGGER_EXTERNAL  : hit.eq(ext_now != 0),
                "default"            : hit.eq(0),
            }),
        ]
        self.sync += [
            ext_pending.eq(ext_now),
            If(strobe | arm, ext_pending.eq(0)),
        ]

        # DRAM words / Sample FIFO -----------------------------------------------------------------
        # Samples grouped in DRAM words before the FIFO: the FIFO drains at up to one DRAM word per
        # cycle (ratio x the full samplerate), the DRAM stalls (refresh, bank/row changes) absorbed
        # at the full samplerate are caught up (a 32-bit FIFO drains at most at the samplerate).
        self.upconverter = upconverter = ResetInserter()(stream.Converter(32, dw))
        self.sample_fifo = sample_fifo = ResetInserter()(
            stream.SyncFIFO([("data", dw)], max(sample_fifo_depth//ratio, 2), buffered=True))
        self.comb += [
            upconverter.reset.eq(reset),
            sample_fifo.reset.eq(reset),
            upconverter.source.connect(sample_fifo.sink),
        ]

        # Capture FSM ------------------------------------------------------------------------------
        k              = Signal(cbits) # Samples pushed since arm (index of the next one).
        start_ts       = Signal(64)    # Start timestamp candidate (sample max(0, k - pre)).
        cand_ts        = Signal(64)
        pre_actual     = Signal(32)
        pre_cand       = Signal(32)
        start          = Signal(cbits) # Start sample of the capture.
        start_cand     = Signal(cbits)
        post_cnt       = Signal(cbits)
        total          = Signal(cbits) # Samples captured (from start).
        triggered      = Signal()
        started        = Signal() # Capture start known (trigger, or overflow before the trigger).
        ended          = Signal()
        overflow       = Signal()
        trigger_ts     = Signal(64)
        rd_enable      = Signal()
        framer_start   = Signal()
        framer_done    = Signal()
        flushed        = Signal()
        push           = Signal()
        accepted       = Signal()
        lost           = Signal()
        do_trigger     = Signal()
        do_end         = Signal()
        do_end_armed   = Signal()

        self.comb += [
            cand_ts.eq(Mux(k == 0, timestamp, start_ts)),
            pre_cand.eq(Mux(k < pre, k, pre)),
            start_cand.eq(k - pre_cand),
            accepted.eq(push & upconverter.sink.ready),
            lost.eq(push & ~upconverter.sink.ready & sampling),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(arm, NextState("FLUSH")),
        )
        fsm.act("FLUSH",
            reset.eq(1),
            If(flushed, NextState("ARMED")),
        )
        fsm.act("ARMED",
            sampling.eq(1),
            push.eq(strobe),
            upconverter.sink.data.eq(sample),
            If(stop,
                NextState("IDLE"),
            ).Elif(accepted & hit,
                do_trigger.eq(1),
                If(~unbounded & (post == 1),
                    do_end.eq(1),
                    NextState("ALIGN"),
                ).Else(
                    NextState("TRIGGERED"),
                ),
            ).Elif(lost,
                do_end_armed.eq(1),
                do_end.eq(1),
                NextState("ALIGN"),
            ),
        )
        fsm.act("TRIGGERED",
            sampling.eq(1),
            push.eq(strobe),
            upconverter.sink.data.eq(sample),
            If(lost | stop | (accepted & ~unbounded & (post_cnt == (post - 1))),
                do_end.eq(1),
                NextState("ALIGN"),
            ),
        )
        # Pad the last DRAM word (padding samples not counted, dropped at the readout).
        fsm.act("ALIGN",
            push.eq(k[:ratio_bits] != 0) if ratio_bits else push.eq(0),
            upconverter.sink.data.eq(0),
            If(~push, NextState("READOUT")),
        )
        fsm.act("READOUT",
            If(framer_done, NextState("DONE")),
        )
        fsm.act("DONE")
        # Arm (restart) from any state.
        for state in ["ARMED", "TRIGGERED", "ALIGN", "READOUT", "DONE"]:
            fsm.act(state, If(arm, NextState("FLUSH")))
        self.comb += [
            upconverter.sink.valid.eq(push),
            self.flush.eq(arm),
        ]

        self.sync += [
            # Samples counter / start timestamp candidate.
            If(accepted,
                k.eq(k + 1),
                start_ts.eq(cand_ts + Mux(k >= pre, divider, 0)),
            ),
            # Trigger.
            If(do_trigger,
                triggered.eq(1),
                trigger_ts.eq(timestamp),
                post_cnt.eq(1),
                total.eq(pre_cand + 1),
            ),
            If(do_trigger | do_end_armed,
                pre_actual.eq(pre_cand),
                start.eq(start_cand),
                self._start_timestamp.status.eq(cand_ts),
            ),
            If(do_end_armed,
                trigger_ts.eq(timestamp),
                total.eq(pre_cand),
            ),
            If(fsm.ongoing("TRIGGERED") & accepted,
                post_cnt.eq(post_cnt + 1),
                total.eq(total + 1),
            ),
            If(do_trigger | do_end_armed, started.eq(1)),
            If(do_end, ended.eq(1)),
            If(lost, overflow.eq(1)),
            If(fsm.ongoing("FLUSH"),
                k.eq(0),
                post_cnt.eq(0),
                total.eq(0),
                pre_actual.eq(0),
                start.eq(0),
                triggered.eq(0),
                started.eq(0),
                ended.eq(0),
                overflow.eq(0),
                trigger_ts.eq(0),
                self._start_timestamp.status.eq(0),
            ),
        ]
        self.comb += [
            self.trigger_event.eq(do_trigger),
            self.trigger_value.eq(sample),
            rd_enable.eq(Mux(mode,
                started & ~fsm.ongoing("FLUSH") & ~fsm.ongoing("IDLE"),
                ended     & ~fsm.ongoing("FLUSH") & ~fsm.ongoing("IDLE"))),
        ]
        rd_enable_d = Signal()
        self.sync += rd_enable_d.eq(rd_enable)
        self.comb += framer_start.eq(rd_enable & ~rd_enable_d)

        # DRAM Writer (ring buffer) ----------------------------------------------------------------
        base_word = ring_base//word_bytes
        ring_mask = ring_words - 1
        wr_words  = Signal(cbits) # DRAM words written (commands accepted) since arm.
        rd_issue  = Signal(cbits) # DRAM words read (commands issued).
        rd_done   = Signal(cbits) # DRAM words read (data out of the DMA reader).
        full      = Signal()
        self.writer = writer = LiteDRAMDMAWriter(write_port, fifo_depth=dma_fifo_depth,
            fifo_buffered=True)
        self.comb += [
            full.eq(started & (wr_words >= (rd_done + ring_words))),
            writer.sink.valid.eq(sample_fifo.source.valid & ~full & ~reset),
            writer.sink.address.eq(base_word + (wr_words & ring_mask)),
            writer.sink.data.eq(sample_fifo.source.data),
            sample_fifo.source.ready.eq(writer.sink.ready & ~full),
        ]
        self.sync += [
            If(writer.sink.valid & writer.sink.ready,
                wr_words.eq(wr_words + 1),
            ),
            If(reset, wr_words.eq(0)),
        ]

        # DRAM Reader ------------------------------------------------------------------------------
        self.reader = reader = LiteDRAMDMAReader(read_port, fifo_depth=dma_fifo_depth,
            fifo_buffered=True)
        rd_pending = Signal(max=2*dma_fifo_depth + 2) # Reads issued, data not out (not reset).
        self.comb += [
            reader.sink.valid.eq(rd_enable & (rd_issue < wr_words) & ~reset),
            reader.sink.address.eq(base_word + (rd_issue & ring_mask)),
        ]
        rd_issued   = Signal()
        rd_returned = Signal()
        self.comb += [
            rd_issued.eq(reader.sink.valid & reader.sink.ready),
            rd_returned.eq(reader.source.valid & reader.source.ready),
        ]
        self.sync += [
            If(rd_issued, rd_issue.eq(rd_issue + 1)),
            If(rd_returned, rd_done.eq(rd_done + 1)),
            If(do_trigger | do_end_armed,
                rd_issue.eq(start_cand >> ratio_bits),
                rd_done.eq(start_cand  >> ratio_bits),
            ),
            If(reset,
                rd_issue.eq(0),
                rd_done.eq(0),
            ),
            rd_pending.eq(rd_pending + rd_issued - rd_returned),
        ]
        self.comb += [
            flushed.eq((rd_pending == 0) & ~writer.fifo.source.valid & ~reader.source.valid),
            self._level.status.eq(Mux(triggered, wr_words - rd_done, 0)),
        ]

        # DRAM words to samples, skip/length filter ------------------------------------------------
        self.downconverter = downconverter = ResetInserter()(stream.Converter(dw, 32))
        self.output_fifo = output_fifo = ResetInserter()(
            stream.SyncFIFO([("data", 32)], output_fifo_depth, buffered=True))
        self.comb += [
            downconverter.reset.eq(reset),
            output_fifo.reset.eq(reset),
            reader.source.connect(downconverter.sink),
            If(reset, reader.source.ready.eq(1)), # Flush: data dropped.
        ]
        skip   = Signal(max=max(ratio, 2))
        passed = Signal(cbits)
        drop   = Signal()
        self.comb += [
            drop.eq((skip != 0) | (ended & (passed >= total))),
            output_fifo.sink.valid.eq(downconverter.source.valid & ~drop),
            output_fifo.sink.data.eq(downconverter.source.data),
            downconverter.source.ready.eq(drop | output_fifo.sink.ready),
        ]
        self.sync += [
            If(downconverter.source.valid & downconverter.source.ready,
                If(skip != 0,
                    skip.eq(skip - 1),
                ).Elif(~drop,
                    passed.eq(passed + 1),
                ),
            ),
            If(do_trigger | do_end_armed,
                skip.eq(start_cand[:ratio_bits] if ratio_bits else 0),
            ),
            If(reset,
                skip.eq(0),
                passed.eq(0),
            ),
        ]

        # Framer -----------------------------------------------------------------------------------
        header_count = Signal(64)
        self.comb += header_count.eq(Mux(mode,
            Mux(unbounded, 0, pre_actual + post),
            total))
        header = [
            LA_MAGIC,
            Cat(C(LA_VERSION, 8), C(n, 8), mode, C(0, 7), overflow & ~mode, C(0, 7)),
            divider,
            C(block_samples, 32),
            pre_actual,
            C(0, 32),
            header_count[:32],
            header_count[32:],
            self._start_timestamp.status[:32],
            self._start_timestamp.status[32:],
            trigger_ts[:32],
            trigger_ts[32:],
        ]
        assert len(header) == LA_HEADER_WORDS

        framed     = stream.Endpoint([("data", 32)])
        emitted    = Signal(cbits)
        remaining  = Signal(cbits)
        index      = Signal(max=LA_HEADER_WORDS)
        blk_count  = Signal(16)
        blk_cnt    = Signal(16)
        blk_last   = Signal()
        last_ready = Signal()
        self.comb += [
            remaining.eq(total - emitted),
            last_ready.eq(ended & (remaining <= block_samples) & (output_fifo.level >= remaining)),
            blk_count.eq(Mux(last_ready, remaining, block_samples)),
        ]
        self.framer = framer = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += framer.reset.eq(reset)
        framer.act("IDLE",
            NextValue(index, 0),
            If(framer_start, NextState("HEADER")),
        )
        framer.act("HEADER",
            framed.valid.eq(1),
            Case(index, {i: framed.data.eq(w) for i, w in enumerate(header)}),
            If(framed.ready,
                NextValue(index, index + 1),
                If(index == (LA_HEADER_WORDS - 1), NextState("BLOCK")),
            ),
        )
        framer.act("BLOCK",
            framed.valid.eq(last_ready | (output_fifo.level >= block_samples)),
            framed.data.eq(Cat(blk_count,
                last_ready, overflow & last_ready, C(0, 6),
                C(LA_BLOCK_MAGIC, 8))),
            If(framed.valid & framed.ready,
                NextValue(blk_cnt,  blk_count),
                NextValue(blk_last, last_ready),
                If(blk_count == 0,
                    NextState("DONE"),
                ).Else(
                    NextState("DATA"),
                ),
            ),
        )
        framer.act("DATA",
            framed.valid.eq(output_fifo.source.valid),
            framed.data.eq(output_fifo.source.data),
            output_fifo.source.ready.eq(framed.ready),
            If(framed.valid & framed.ready,
                NextValue(blk_cnt, blk_cnt - 1),
                If(blk_cnt == 1,
                    If(blk_last,
                        NextState("DONE"),
                    ).Else(
                        NextState("BLOCK"),
                    ),
                ),
            ),
        )
        framer.act("DONE",
            framer_done.eq(1),
        )
        self.sync += [
            If(framer.ongoing("DATA") & framed.valid & framed.ready,
                emitted.eq(emitted + 1),
            ),
            If(reset, emitted.eq(0)),
        ]

        # Packets (USB3 Bulk IN) -------------------------------------------------------------------
        self.packer = packer = ResetInserter()(AgentLabWordPacker(idle_cycles=idle_cycles))
        self.comb += [
            packer.reset.eq(reset),
            framed.connect(packer.sink),
            packer.source.connect(self.source),
        ]

        # Status -----------------------------------------------------------------------------------
        state = Signal(3)
        self.comb += [
            If(fsm.ongoing("ARMED"),
                state.eq(LA_STATE_ARMED),
            ).Elif(fsm.ongoing("TRIGGERED"),
                state.eq(LA_STATE_TRIGGERED),
            ).Elif(fsm.ongoing("ALIGN") | fsm.ongoing("READOUT"),
                state.eq(LA_STATE_READOUT),
            ).Elif(fsm.ongoing("DONE"),
                state.eq(LA_STATE_DONE),
            ).Elif(fsm.ongoing("FLUSH"),
                state.eq(LA_STATE_FLUSH),
            ).Else(
                state.eq(LA_STATE_IDLE),
            ),
            self._status.fields.state.eq(state),
            self._status.fields.triggered.eq(triggered),
            self._status.fields.overflow.eq(overflow),
            self._status.fields.dram_ready.eq(self.dram_ready),
            self._count.status.eq(total),
            self._trigger_offset.status.eq(pre_actual),
            self._trigger_timestamp.status.eq(trigger_ts),
        ]
