#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage storages (backends of the SCSI command engine, see usb3_core.msc).

- USB3MSCRAMStorage: FPGA block RAM (USB3MSCRAM), optionally initialized with a disk image.
- USB3MSCDRAM      : DRAM through LiteDRAM DMAs (large disk, not initialized).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

# Constants ----------------------------------------------------------------------------------------

MSC_BLOCK_SIZE = 512

# USB3 MSC Storage --------------------------------------------------------------------------------

# Storage command: read/write blocks (MSC_BLOCK_SIZE) from lba.
msc_storage_cmd_layout = [
    ("write",  1),
    ("lba",   32),
    ("blocks", 16),
]

class USB3MSCStorage(LiteXModule):
    """Storage interface of the SCSI command engine (BRAM/DRAM backends).

    - cmd    : Read/write command (msc_storage_cmd_layout), accepted when the storage is idle.
    - source : Read data (32-bit words, blocks*MSC_BLOCK_SIZE//4 words per read command).
    - sink   : Write data (32-bit words, blocks*MSC_BLOCK_SIZE//4 words per write command).
    - ready  : Storage available (ex: DRAM initialized), commands fail with NOT READY when low.
    - idle   : No command ongoing, write data committed (a following read returns it).
    """
    def __init__(self, size):
        assert size % MSC_BLOCK_SIZE == 0
        self.size   = size
        self.blocks = size//MSC_BLOCK_SIZE
        self.cmd    = stream.Endpoint(msc_storage_cmd_layout) # i
        self.source = stream.Endpoint([("data", 32)])         # o
        self.sink   = stream.Endpoint([("data", 32)])         # i
        self.ready  = Signal()                                # o
        self.idle   = Signal()                                # o

# USB3 MSC RAM Disk --------------------------------------------------------------------------------

class USB3MSCRAM(LiteXModule):
    """Disk RAM (32-bit words): read port (adr/re/dat_r, 1 cycle latency), write port (adr/dat_w/we).

    Built from banks of bank_words (8K words: RAMB36 without cascades, Vivado 7-Series cascaded RAMB36
    on large memories can fail the REQP-1962 DRC).
    """
    def __init__(self, words, init=None, bank_words=8192):
        init       = [] if init is None else init
        bank_words = min(words, bank_words)
        assert words % bank_words == 0
        nbanks = words//bank_words
        abits  = log2_int(words, need_pow2=False)
        self.rd_port = Record([("adr", abits), ("re", 1), ("dat_r", 32)])
        self.wr_port = Record([("adr", abits), ("dat_w", 32), ("we", 1)])

        # # #

        bbits   = log2_int(bank_words)
        rd_bank = Signal(max=max(nbanks, 2))
        rd_dats = []
        for n in range(nbanks):
            mem = Memory(32, bank_words, init=init[n*bank_words:(n+1)*bank_words])
            rd  = mem.get_port(has_re=True)
            wr  = mem.get_port(write_capable=True)
            self.specials += mem, rd, wr
            self.comb += [
                rd.adr.eq(self.rd_port.adr[:bbits]),
                rd.re.eq(self.rd_port.re),
                wr.adr.eq(self.wr_port.adr[:bbits]),
                wr.dat_w.eq(self.wr_port.dat_w),
                wr.we.eq(self.wr_port.we & (self.wr_port.adr[bbits:] == n)),
            ]
            rd_dats.append(rd.dat_r)
        self.sync += If(self.rd_port.re, rd_bank.eq(self.rd_port.adr[bbits:]))
        self.comb += self.rd_port.dat_r.eq(Array(rd_dats)[rd_bank])

class USB3MSCRAMStorage(USB3MSCStorage):
    """Block RAM disk storage (USB3MSCRAM), optionally initialized with a disk image."""
    def __init__(self, size, image=None):
        USB3MSCStorage.__init__(self, size)
        words           = size//4
        words_per_block = MSC_BLOCK_SIZE//4

        # # #

        # Disk RAM.
        init = []
        if image is not None:
            image = bytes(image[:size]).ljust(size, b"\x00")
            init  = [int.from_bytes(image[4*i:4*i+4], "little") for i in range(words)]
        self.ram = ram = USB3MSCRAM(words, init=init)

        # Always ready.
        self.comb += self.ready.eq(1)

        # Address/Count.
        adr   = Signal(max=max(words, 2), reset_less=True)
        count = Signal(32, reset_less=True) # Words to read/write.

        # RAM reader (1 cycle latency, prefetch FIFO).
        self.fifo = fifo = stream.SyncFIFO([("data", 32)], 8)
        rd_issue = Signal()
        rd_valid = Signal()
        self.comb += [
            ram.rd_port.adr.eq(adr),
            ram.rd_port.re.eq(rd_issue),
        ]
        self.sync += rd_valid.eq(rd_issue)
        self.comb += [
            fifo.sink.valid.eq(rd_valid),
            fifo.sink.data.eq(ram.rd_port.dat_r),
            fifo.source.connect(self.source),
        ]

        # RAM writer.
        self.comb += [
            ram.wr_port.adr.eq(adr),
            ram.wr_port.dat_w.eq(self.sink.data),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.idle.eq(~rd_valid & ~fifo.source.valid),
            self.cmd.ready.eq(self.idle),
            NextValue(adr,   self.cmd.lba*words_per_block),
            NextValue(count, self.cmd.blocks*words_per_block),
            If(self.cmd.valid & self.cmd.ready,
                If(self.cmd.write,
                    NextState("WRITE")
                ).Else(
                    NextState("READ")
                )
            )
        )
        fsm.act("READ",
            rd_issue.eq(fifo.level < 4),
            If(rd_issue,
                NextValue(adr,   adr   + 1),
                NextValue(count, count - 1),
                If(count == 1, NextState("IDLE"))
            )
        )
        fsm.act("WRITE",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                ram.wr_port.we.eq(1),
                NextValue(adr,   adr   + 1),
                NextValue(count, count - 1),
                If(count == 1, NextState("IDLE"))
            )
        )

# USB3 MSC DRAM Disk -------------------------------------------------------------------------------

class USB3MSCDRAM(USB3MSCStorage):
    """DRAM disk storage: LiteDRAM DMA reader/writer on native ports (from the LiteDRAM crossbar).

    The disk (size bytes) starts at base (bytes) in the DRAM, data stored little-endian (as seen from
    the CPU/Wishbone bus at main_ram + base). Port data width: multiple of 32 bits, block multiple.

    rd_port/wr_port: separate ports (each DMA drives its port's cmd), ex: crossbar.get_port("read")
    and crossbar.get_port("write").
    ready: optional storage ready signal (ex: DRAM initialized by the BIOS), 1 when None.

    The disk content is not initialized: format it from the host (ex: mkfs.vfat /dev/sdX).
    """
    def __init__(self, rd_port, wr_port, size, base=0, ready=None, fifo_depth=64):
        from litedram.frontend.dma import LiteDRAMDMAReader, LiteDRAMDMAWriter
        USB3MSCStorage.__init__(self, size)
        data_width = rd_port.data_width
        port_bytes = data_width//8
        assert wr_port.data_width == data_width
        assert data_width % 32 == 0
        assert MSC_BLOCK_SIZE % port_bytes == 0
        assert base % MSC_BLOCK_SIZE == 0
        assert (base + size) <= (2**rd_port.address_width)*port_bytes
        words_per_block = MSC_BLOCK_SIZE//port_bytes

        # # #

        # Ready.
        self.comb += self.ready.eq(1 if ready is None else ready)

        # Address/Count (DRAM port words).
        adr   = Signal(rd_port.address_width, reset_less=True)
        count = Signal(32, reset_less=True)

        # DMA Reader (port words -> 32-bit words).
        self.reader  = reader  = LiteDRAMDMAReader(rd_port, fifo_depth=fifo_depth, fifo_buffered=True)
        self.rd_conv = rd_conv = stream.Converter(data_width, 32)
        self.comb += [
            reader.sink.address.eq(adr),
            reader.source.connect(rd_conv.sink),
            rd_conv.source.connect(self.source, keep={"valid", "ready", "data"}),
        ]

        # DMA Writer (32-bit words -> port words).
        self.wr_conv = wr_conv = stream.Converter(32, data_width)
        self.writer  = writer  = LiteDRAMDMAWriter(wr_port, fifo_depth=fifo_depth, fifo_buffered=True)
        self.comb += [
            self.sink.connect(wr_conv.sink, keep={"valid", "ready", "data"}),
            writer.sink.address.eq(adr),
            writer.sink.data.eq(wr_conv.source.data),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            # Idle when the write data are all handed to the DRAM controller.
            self.idle.eq(~wr_conv.source.valid & ~writer.fifo.source.valid),
            self.cmd.ready.eq(self.idle),
            NextValue(adr,   (base//port_bytes) + self.cmd.lba*words_per_block),
            NextValue(count, self.cmd.blocks*words_per_block),
            If(self.cmd.valid & self.cmd.ready,
                If(self.cmd.write,
                    NextState("WRITE")
                ).Else(
                    NextState("READ")
                )
            )
        )
        fsm.act("READ",
            reader.sink.valid.eq(1),
            If(reader.sink.ready,
                NextValue(adr,   adr   + 1),
                NextValue(count, count - 1),
                If(count == 1, NextState("IDLE"))
            )
        )
        fsm.act("WRITE",
            writer.sink.valid.eq(wr_conv.source.valid),
            wr_conv.source.ready.eq(writer.sink.ready),
            If(writer.sink.valid & writer.sink.ready,
                NextValue(adr,   adr   + 1),
                NextValue(count, count - 1),
                If(count == 1, NextState("IDLE"))
            )
        )
