#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Events: timestamped event records (doc/agentlab.md), merged from several sources,
buffered and streamed as 32-bit words to a USB3 Bulk IN endpoint.

Record (16 bytes, little-endian): type (byte 0), source (byte 1), value (bytes 2-7), timestamp
(bytes 8-15, latched when the event occurs). Events lost (source or main FIFO full) are reported by
an overflow record (type 0xff, value: number of lost events, timestamp of the first lost event).
"""

from operator  import add
from functools import reduce

from migen import *
from migen.genlib.misc       import WaitTimer
from migen.genlib.roundrobin import RoundRobin, SP_CE

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

# Constants ----------------------------------------------------------------------------------------

EVENT_UART_RX    = 0x01
EVENT_UART_TX    = 0x02
EVENT_GPIO       = 0x03
EVENT_POWER      = 0x04
EVENT_LA_TRIGGER = 0x05
EVENT_MARKER     = 0x06
EVENT_OVERFLOW   = 0xff

EVENT_RECORD_BYTES = 16

def event_layout():
    """Event record (Cat order: little-endian record bytes)."""
    return [
        ("type",       8),
        ("source",     8),
        ("value",     48),
        ("timestamp", 64),
    ]

def event_record(type, source, value, timestamp):
    """Event record bytes (host side/tests)."""
    return bytes([type, source]) + value.to_bytes(6, "little") + timestamp.to_bytes(8, "little")

# Event Port ---------------------------------------------------------------------------------------

class AgentLabEventPort(LiteXModule):
    """Single events source (UART bytes, markers...): an event (valid pulse, value) is timestamped
    and buffered (depth records) until merged, overflow pulse when lost."""
    def __init__(self, timestamp, type, source=0, depth=4):
        self.valid    = Signal()                             # i
        self.value    = Signal(48)                           # i
        self.source   = stream.Endpoint(event_layout())      # o
        self.overflow = Signal()                             # o

        # # #

        self.fifo = fifo = stream.SyncFIFO(event_layout(), depth)
        self.comb += [
            fifo.sink.valid.eq(self.valid),
            fifo.sink.type.eq(type),
            fifo.sink.source.eq(source),
            fifo.sink.value.eq(self.value),
            fifo.sink.timestamp.eq(timestamp),
            fifo.source.connect(self.source),
            self.overflow.eq(self.valid & ~fifo.sink.ready),
        ]

# Vector Events ------------------------------------------------------------------------------------

class AgentLabVectorEvents(LiteXModule):
    """Events of a vector of signals (GPIO edges, power changes): the changes of a cycle are
    timestamped and buffered (depth), then emitted as one record per changed bit (source: bit index,
    value[0]: new level of the bit, value[8:8+n]: levels of all the bits)."""
    def __init__(self, timestamp, n, type, depth=8):
        assert n <= 40
        self.changes  = Signal(n)                            # i
        self.levels   = Signal(n)                            # i
        self.source   = source = stream.Endpoint(event_layout()) # o
        self.overflow = Signal()                             # o

        # # #

        # Capture.
        layout = [("changes", n), ("levels", n), ("timestamp", 64)]
        self.fifo = fifo = stream.SyncFIFO(layout, depth)
        self.comb += [
            fifo.sink.valid.eq(self.changes != 0),
            fifo.sink.changes.eq(self.changes),
            fifo.sink.levels.eq(self.levels),
            fifo.sink.timestamp.eq(timestamp),
            self.overflow.eq(fifo.sink.valid & ~fifo.sink.ready),
        ]

        # Serializer (one record per changed bit, lowest index first).
        mask   = Signal(n)
        levels = Signal(n,  reset_less=True)
        ts     = Signal(64, reset_less=True)
        index  = Signal(max=max(n, 2))
        for i in reversed(range(n)):
            self.comb += If(mask[i], index.eq(i))
        remaining = Signal(n)
        self.comb += [
            remaining.eq(mask & ~(1 << index)),
            fifo.source.ready.eq((mask == 0) | (source.ready & (remaining == 0))),
            source.valid.eq(mask != 0),
            source.type.eq(type),
            source.source.eq(index),
            source.value.eq(Cat((levels >> index)[0], C(0, 7), levels)),
            source.timestamp.eq(ts),
        ]
        self.sync += [
            If(source.valid & source.ready,
                mask.eq(remaining)
            ),
            If(fifo.source.valid & fifo.source.ready,
                mask.eq(fifo.source.changes),
                levels.eq(fifo.source.levels),
                ts.eq(fifo.source.timestamp),
            ),
        ]

# Word Packer --------------------------------------------------------------------------------------

class AgentLabWordPacker(LiteXModule):
    """32-bit words stream to USB3 packets: a word is held until the next one (packet continues) or
    idle_cycles without word (packet ended: last, the host gets the data promptly); packets are also
    ended at max_bytes.

    min_period: packets ended on idle (short packets: they end the host transfers) at most every
    min_period cycles (data held up to min_period cycles): a slow stream (ex: data in bursts) fills
    the host transfers instead of ending them at each burst.
    """
    def __init__(self, max_bytes=1024, idle_cycles=128, min_period=0):
        self.sink   = sink   = stream.Endpoint([("data", 32)])
        self.source = source = stream.Endpoint([("data", 32), ("be", 4)])

        # # #

        out       = Signal(32, reset_less=True)
        pending   = Signal() # Word held.
        pkt_words = Signal(max=max_bytes//4)
        at_max    = Signal()
        idle_end  = Signal() # Packet ended on idle.
        self.idle = idle = WaitTimer(idle_cycles)
        if min_period:
            period = Signal(max=min_period + 1, reset=min_period) # Since the last idle end.
            self.sync += [
                If(period != min_period, period.eq(period + 1)),
                If(source.valid & source.ready & ~sink.valid & ~at_max, period.eq(0)),
            ]
            self.comb += idle_end.eq(idle.done & (period == min_period))
        else:
            self.comb += idle_end.eq(idle.done)
        self.comb += [
            idle.wait.eq(~sink.valid & pending),
            at_max.eq(pkt_words == (max_bytes//4 - 1)),
            source.valid.eq(pending & (sink.valid | idle_end | at_max)),
            source.data.eq(out),
            source.last.eq(~sink.valid | at_max),
            source.be.eq(0b1111),
            sink.ready.eq(~pending | (source.valid & source.ready)),
        ]
        self.sync += [
            If(source.valid & source.ready,
                pending.eq(0),
                pkt_words.eq(pkt_words + 1),
                If(source.last, pkt_words.eq(0)),
            ),
            If(sink.valid & sink.ready,
                out.eq(sink.data),
                pending.eq(1),
            ),
        ]

# AgentLab Events ----------------------------------------------------------------------------------

class AgentLabEvents(LiteXModule):
    """Events merger: ports (event record streams) merged (round-robin), buffered (fifo_depth
    records, overflow records on lost events) and streamed as 32-bit words packets (source: USB3
    Bulk IN endpoint sink).

    overflows: optional overflow pulses of the ports (lost events, counted in the overflow record).
    """
    def __init__(self, timestamp, ports, overflows=[], fifo_depth=512, idle_cycles=128):
        self.source = stream.Endpoint([("data", 32), ("be", 4)])

        self._enable  = CSRStorage(1, reset=1, description="Events recording enable.")
        self._clear   = CSRStorage(fields=[
            CSRField("clear", size=1, pulse=True,
                description="Clear the buffered events and the lost events counter."),
        ])
        self._lost    = CSRStatus(32, description="Lost events (since the last clear).")
        self._level   = CSRStatus(16, description="Buffered events.")

        # # #

        enable = self._enable.storage
        clear  = self._clear.fields.clear

        # Arbiter (round-robin).
        n = len(ports)
        self.rr = rr = RoundRobin(n, SP_CE)
        merged = stream.Endpoint(event_layout())
        self.comb += [
            rr.request.eq(Cat(*[port.valid for port in ports])),
            rr.ce.eq(~merged.valid | merged.ready),
        ]
        for i, port in enumerate(ports):
            self.comb += If(rr.grant == i, *port.connect(merged))

        # Main FIFO (with clear).
        self.fifo = fifo = ResetInserter()(stream.SyncFIFO(event_layout(), fifo_depth,
            buffered=True))
        self.comb += fifo.reset.eq(clear)

        # Lost events: counted, overflow record inserted (with priority) when space is available.
        lost      = Signal(48)
        lost_ts   = Signal(64)
        lost_new  = Signal(max=len(overflows) + 2)
        insert    = Signal()
        dropped   = Signal()
        self.comb += [
            insert.eq((lost != 0) & fifo.sink.ready),
            merged.ready.eq(~insert),
            dropped.eq(merged.valid & merged.ready & enable & ~fifo.sink.ready),
            lost_new.eq(reduce(add, [ovf & enable for ovf in overflows] + [dropped])),
            If(insert,
                fifo.sink.valid.eq(1),
                fifo.sink.type.eq(EVENT_OVERFLOW),
                fifo.sink.source.eq(0),
                fifo.sink.value.eq(lost),
                fifo.sink.timestamp.eq(lost_ts),
            ).Else(
                fifo.sink.valid.eq(merged.valid & enable),
                fifo.sink.type.eq(merged.type),
                fifo.sink.source.eq(merged.source),
                fifo.sink.value.eq(merged.value),
                fifo.sink.timestamp.eq(merged.timestamp),
            ),
        ]
        self.sync += [
            If(insert,
                lost.eq(lost_new),
            ).Else(
                lost.eq(lost + lost_new),
            ),
            If((lost_new != 0) & ((lost == 0) | insert),
                lost_ts.eq(timestamp),
            ),
            self._lost.status.eq(self._lost.status + lost_new),
            If(clear,
                lost.eq(0),
                self._lost.status.eq(0),
            ),
        ]

        # Level.
        level = Signal(16)
        self.sync += [
            If(fifo.sink.valid & fifo.sink.ready & ~(fifo.source.valid & fifo.source.ready),
                level.eq(level + 1)
            ).Elif(~(fifo.sink.valid & fifo.sink.ready) & fifo.source.valid & fifo.source.ready,
                level.eq(level - 1)
            ),
            If(clear, level.eq(0)),
        ]
        self.comb += self._level.status.eq(level)

        # Records to 32-bit words (record held: not affected by a clear while serialized).
        record       = Signal(128, reset_less=True)
        record_valid = Signal()
        word         = Signal(2)
        self.packer = packer = AgentLabWordPacker(idle_cycles=idle_cycles)
        record_done = Signal()
        self.comb += [
            record_done.eq(packer.sink.valid & packer.sink.ready & (word == 3)),
            fifo.source.ready.eq(~record_valid | record_done),
            packer.sink.valid.eq(record_valid),
            Case(word, {i: packer.sink.data.eq(record[32*i:32*(i+1)]) for i in range(4)}),
            packer.source.connect(self.source),
        ]
        self.sync += [
            If(packer.sink.valid & packer.sink.ready,
                word.eq(word + 1)
            ),
            If(record_done,
                record_valid.eq(0)
            ),
            If(fifo.source.valid & fifo.source.ready,
                record_valid.eq(1),
                record.eq(Cat(fifo.source.type, fifo.source.source, fifo.source.value,
                    fifo.source.timestamp)),
            ),
        ]
