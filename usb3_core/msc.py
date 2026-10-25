#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage (Bulk-Only Transport, SCSI transparent command set) RAM/DRAM disk.

A USB3 device exposing a disk (/dev/sdX, no driver needed) backed by a storage:
- USB3MSCRAMStorage: FPGA block RAM, optionally initialized with a disk image (ex: a FAT file system
  with files, see msc_fat_image).
- USB3MSCDRAM: DRAM through LiteDRAM DMAs (large disk, not initialized: mkfs.vfat from the host).

- Bulk-Only Transport (USB MSC BOT 1.0): CBW (31 bytes) on Bulk OUT, optional data phase, CSW (13 bytes)
  on Bulk IN. Get Max LUN/Bulk-Only Mass Storage Reset class requests (constant responses).
- SCSI commands: TEST UNIT READY, REQUEST SENSE, INQUIRY, MODE SENSE(6), PREVENT ALLOW MEDIUM
  REMOVAL, READ FORMAT CAPACITIES, READ CAPACITY(10), READ(10), WRITE(10), SYNCHRONIZE CACHE(10).
  Other commands fail (ILLEGAL REQUEST sense), their data phase is padded (IN) or discarded (OUT).
"""

import os
import subprocess
import tempfile

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import bytes_to_be
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import *

# Constants ----------------------------------------------------------------------------------------

MSC_BLOCK_SIZE = 512
CBW_SIGNATURE  = 0x43425355 # "USBC".
CSW_SIGNATURE  = 0x53425355 # "USBS".

SCSI_TEST_UNIT_READY       = 0x00
SCSI_REQUEST_SENSE         = 0x03
SCSI_INQUIRY               = 0x12
SCSI_MODE_SENSE_6          = 0x1a
SCSI_PREVENT_ALLOW_REMOVAL = 0x1e
SCSI_READ_FORMAT_CAPACITY  = 0x23
SCSI_READ_CAPACITY_10      = 0x25
SCSI_READ_10               = 0x28
SCSI_WRITE_10              = 0x2a
SCSI_SYNCHRONIZE_CACHE_10  = 0x35

# Descriptors --------------------------------------------------------------------------------------

def msc_descriptors(in_ept=1, out_ept=2, max_burst=8, **kwargs):
    kwargs.setdefault("product", "USB3-PIPE RAM Disk")
    interface = USB3InterfaceDescriptor(0, cls=0x08, subclass=0x06, protocol=0x50, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    descriptors = USB3Descriptors(interfaces=[interface], **kwargs)
    descriptors.add_class_request(0x21, 0xff)      # Bulk-Only Mass Storage Reset.
    descriptors.add_class_request(0xa1, 0xfe, [0]) # Get Max LUN: 1 LUN.
    return descriptors

# Disk Image ---------------------------------------------------------------------------------------

def msc_fat_image(size, files={}, label="USB3PIPE"):
    """FAT disk image (mkfs.fat + mtools) with files ({name: content}), None if the tools are missing."""
    try:
        with tempfile.TemporaryDirectory() as d:
            img = os.path.join(d, "disk.img")
            subprocess.run(["mkfs.fat", "-C", "-n", label, img, str(size//1024)], check=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for name, content in files.items():
                src = os.path.join(d, name)
                with open(src, "wb") as f:
                    f.write(content.encode() if isinstance(content, str) else content)
                subprocess.run(["mcopy", "-i", img, src, "::" + name], check=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return open(img, "rb").read()
    except (OSError, subprocess.CalledProcessError):
        return None

# SCSI Responses -----------------------------------------------------------------------------------

def scsi_responses(blocks, vendor="EnjoyDig", product="USB3-PIPE RAMDisk", revision="1.00"):
    """Constant responses: {opcode: bytes}."""
    def pad(s, n):
        return list(s.encode()[:n].ljust(n, b" "))
    last_lba = blocks - 1
    return {
        # Removable direct access device, SPC-3, response format 2.
        SCSI_INQUIRY : [0x00, 0x80, 0x05, 0x02, 31, 0, 0, 0] + pad(vendor, 8) + pad(product, 16) +
            pad(revision, 4),
        SCSI_MODE_SENSE_6 : [3, 0, 0, 0],
        SCSI_READ_CAPACITY_10 : list(last_lba.to_bytes(4, "big")) + list(MSC_BLOCK_SIZE.to_bytes(4, "big")),
        # Formatted media: capacity list header + current descriptor.
        SCSI_READ_FORMAT_CAPACITY : [0, 0, 0, 8] + list(blocks.to_bytes(4, "big")) + [0x02, 0] +
            list(MSC_BLOCK_SIZE.to_bytes(2, "big")),
    }

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
    def __init__(self, words, init=[], bank_words=8192):
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

# USB3 MSC SCSI ------------------------------------------------------------------------------------

class USB3MSCSCSI(LiteXModule):
    """Bulk-Only Transport/SCSI command engine on a storage (USB3MSCStorage: BRAM/DRAM)."""
    def __init__(self, bulk_in, bulk_out, storage, inquiry_product="USB3-PIPE RAMDisk"):
        blocks = storage.blocks

        # # #

        # Responses ROM (opcode -> offset/length).
        responses = scsi_responses(blocks, product=inquiry_product)
        rom   = []
        table = {}
        for opcode, data in responses.items():
            table[opcode] = (len(rom), len(data))
            data = data + [0]*((-len(data)) % 4)
            rom += [int.from_bytes(bytes(data[i:i+4]), "little") for i in range(0, len(data), 4)]
        rom = Array(Constant(w, 32) for w in rom)

        # CBW.
        cbw   = Array(Signal(32, reset_less=True) for _ in range(8))
        count = Signal(4)
        sig   = cbw[0]
        tag   = cbw[1]
        dlen  = cbw[2]
        din   = Signal()
        op    = Signal(8)
        lba   = Signal(32)
        nblk  = Signal(16)
        self.comb += [
            din.eq(cbw[3][7]),
            op.eq(cbw[3][24:32]),
            # CB[2:6]: LBA (big endian), CB[7:9]: transfer length (big endian).
            lba.eq(Cat(cbw[5][0:8], cbw[4][24:32], cbw[4][16:24], cbw[4][8:16])),
            nblk.eq(Cat(cbw[5][24:32], cbw[5][16:24])),
        ]

        # Sense data (REQUEST SENSE): key/ASC of the last failed command.
        sense_key = Signal(4)
        sense_asc = Signal(8)

        # Status.
        residue = Signal(32, reset_less=True)
        status  = Signal(8, reset_less=True)

        # Data phase.
        adr       = Signal(max=max(len(rom), 2), reset_less=True)
        remaining = Signal(32, reset_less=True) # Bytes to send.
        out       = Signal(32, reset_less=True) # Bytes to receive/discard.

        # Storage command (READ(10)/WRITE(10)).
        self.comb += [
            storage.cmd.write.eq(op == SCSI_WRITE_10),
            storage.cmd.lba.eq(lba),
            storage.cmd.blocks.eq(nblk),
        ]

        # Sense response: fixed format.
        # Fixed format: byte 0: 0x70, byte 2: sense key, byte 7: additional length (10), byte 12: ASC.
        sense = Array([
            Cat(C(0x70, 8), C(0, 8), sense_key, C(0, 4), C(0, 8)),
            Cat(C(0, 8), C(0, 8), C(0, 8), C(10, 8)),
            C(0, 32),
            Cat(sense_asc, C(0, 8), C(0, 16)),
            C(0, 32),
        ])
        sense_word = Signal(3, reset_less=True)

        self.fsm = fsm = FSM(reset_state="CBW")
        fsm.act("CBW",
            bulk_out.source.ready.eq(1),
            If(bulk_out.source.valid,
                Case(count, {i: NextValue(cbw[i], bulk_out.source.data) for i in range(8)}),
                NextValue(count, count + 1),
                If(bulk_out.source.last,
                    NextValue(count, 0),
                    # Valid CBW: 31 bytes (8 words) with the signature.
                    If((count == 7) & (sig == CBW_SIGNATURE),
                        NextState("DECODE")
                    )
                )
            )
        )
        decode_cases = {}
        for opcode, (offset, length) in table.items():
            decode_cases[opcode] = [
                NextValue(adr, offset),
                NextValue(remaining, Mux(dlen < length, dlen, length)),
                NextValue(residue, Mux(dlen < length, 0, dlen - length)),
                NextState("ROM"),
            ]
        decode_cases[SCSI_REQUEST_SENSE] = [
            NextValue(sense_word, 0),
            NextValue(remaining, Mux(dlen < 18, dlen, 18)),
            NextValue(residue, Mux(dlen < 18, 0, dlen - 18)),
            NextState("SENSE"),
        ]
        for opcode in [SCSI_PREVENT_ALLOW_REMOVAL, SCSI_SYNCHRONIZE_CACHE_10]:
            decode_cases[opcode] = [NextValue(residue, dlen), NextState("CSW")]
        # Storage not ready (ex: DRAM not yet initialized): NOT READY/LOGICAL UNIT NOT READY.
        not_ready = [
            NextValue(sense_key, 0x02),
            NextValue(sense_asc, 0x04),
            NextValue(status, 1),
        ]
        decode_cases[SCSI_TEST_UNIT_READY] = [
            NextValue(residue, dlen),
            If(~storage.ready, *not_ready),
            NextState("CSW"),
        ]
        decode_cases[SCSI_READ_10] = [
            NextValue(remaining, nblk*MSC_BLOCK_SIZE),
            NextValue(residue, dlen - nblk*MSC_BLOCK_SIZE),
            If(~storage.ready,
                *not_ready,
                NextValue(remaining, dlen),
                NextState("PAD"),
            ).Elif((lba + nblk) > blocks,
                NextValue(sense_key, 0x05), NextValue(sense_asc, 0x21), # LBA out of range.
                NextValue(status, 1),
                NextValue(remaining, dlen),
                NextState("PAD"),
            ).Elif(nblk == 0,
                NextState("CSW"),
            ).Else(
                NextState("READ-CMD")
            ),
        ]
        decode_cases[SCSI_WRITE_10] = [
            NextValue(out, nblk*MSC_BLOCK_SIZE),
            NextValue(residue, dlen - nblk*MSC_BLOCK_SIZE),
            If(~storage.ready,
                *not_ready,
                NextValue(out, dlen),
                NextState("DISCARD"),
            ).Elif((lba + nblk) > blocks,
                NextValue(sense_key, 0x05), NextValue(sense_asc, 0x21),
                NextValue(status, 1),
                NextValue(out, dlen),
                NextState("DISCARD"),
            ).Elif(nblk == 0,
                NextState("CSW"),
            ).Else(
                NextState("WRITE-CMD")
            ),
        ]
        decode_cases["default"] = [
            # Unsupported: ILLEGAL REQUEST/INVALID COMMAND OPERATION CODE, data phase padded/discarded.
            NextValue(sense_key, 0x05),
            NextValue(sense_asc, 0x20),
            NextValue(status, 1),
            NextValue(residue, dlen),
            If(dlen == 0,
                NextState("CSW")
            ).Elif(din,
                NextValue(remaining, dlen),
                NextState("PAD")
            ).Else(
                NextValue(out, dlen),
                NextState("DISCARD")
            ),
        ]
        fsm.act("DECODE",
            NextValue(status, 0),
            If(op != SCSI_REQUEST_SENSE,
                NextValue(sense_key, 0),
                NextValue(sense_asc, 0),
            ),
            Case(op, decode_cases)
        )
        fsm.act("ROM",
            bulk_in.sink.valid.eq(remaining != 0),
            bulk_in.sink.data.eq(rom[adr]),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.be.eq(Mux(remaining <= 4, bytes_to_be(remaining), 0b1111)),
            If(remaining == 0,
                NextState("CSW")
            ).Elif(bulk_in.sink.ready,
                NextValue(adr, adr + 1),
                NextValue(remaining, Mux(remaining <= 4, 0, remaining - 4)),
            )
        )
        fsm.act("SENSE",
            bulk_in.sink.valid.eq(remaining != 0),
            bulk_in.sink.data.eq(sense[sense_word]),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.be.eq(Mux(remaining <= 4, bytes_to_be(remaining), 0b1111)),
            If(remaining == 0,
                NextValue(sense_key, 0),
                NextValue(sense_asc, 0),
                NextState("CSW")
            ).Elif(bulk_in.sink.ready,
                NextValue(sense_word, sense_word + 1),
                NextValue(remaining, Mux(remaining <= 4, 0, remaining - 4)),
            )
        )
        fsm.act("PAD",
            bulk_in.sink.valid.eq(remaining != 0),
            bulk_in.sink.data.eq(0),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.be.eq(Mux(remaining <= 4, bytes_to_be(remaining), 0b1111)),
            If(remaining == 0,
                NextState("CSW")
            ).Elif(bulk_in.sink.ready,
                NextValue(remaining, Mux(remaining <= 4, 0, remaining - 4)),
            )
        )
        fsm.act("READ-CMD",
            storage.cmd.valid.eq(1),
            If(storage.cmd.ready,
                NextState("READ")
            )
        )
        fsm.act("READ",
            bulk_in.sink.valid.eq(storage.source.valid),
            bulk_in.sink.data.eq(storage.source.data),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.be.eq(0b1111),
            storage.source.ready.eq(bulk_in.sink.ready),
            If(storage.source.valid & bulk_in.sink.ready,
                NextValue(remaining, remaining - 4),
                If(remaining <= 4, NextState("CSW"))
            )
        )
        fsm.act("WRITE-CMD",
            storage.cmd.valid.eq(1),
            If(storage.cmd.ready,
                NextState("WRITE")
            )
        )
        fsm.act("WRITE",
            storage.sink.valid.eq(bulk_out.source.valid),
            storage.sink.data.eq(bulk_out.source.data),
            bulk_out.source.ready.eq(storage.sink.ready),
            If(bulk_out.source.valid & storage.sink.ready,
                NextValue(out, out - 4),
                If(out <= 4, NextState("WRITE-DONE"))
            )
        )
        fsm.act("WRITE-DONE",
            # Data committed to the storage before the CSW (ordering with the following reads).
            If(storage.idle,
                NextState("CSW")
            )
        )
        fsm.act("DISCARD",
            bulk_out.source.ready.eq(1),
            If(out == 0,
                NextState("CSW")
            ).Elif(bulk_out.source.valid,
                NextValue(out, Mux(out <= 4, 0, out - 4)),
            )
        )
        csw = Array([C(CSW_SIGNATURE, 32), tag, residue, Cat(status, C(0, 24))])
        csw_word = Signal(2)
        fsm.act("CSW",
            bulk_in.sink.valid.eq(1),
            bulk_in.sink.data.eq(csw[csw_word]),
            bulk_in.sink.last.eq(csw_word == 3),
            bulk_in.sink.be.eq(Mux(csw_word == 3, 0b0001, 0b1111)), # 13 bytes.
            If(bulk_in.sink.ready,
                NextValue(csw_word, csw_word + 1),
                If(csw_word == 3,
                    NextValue(csw_word, 0),
                    NextState("CBW")
                )
            )
        )

# USB3 MSC -----------------------------------------------------------------------------------------

class USB3MSC(LiteXModule):
    """USB3 Mass Storage device: block RAM disk (size, optional image) or storage (ex: USB3MSCDRAM)."""
    def __init__(self, phy, sys_clk_freq, size=512*1024, image=None, storage=None, max_burst=8,
        inquiry_product="USB3-PIPE RAMDisk", **kwargs):
        descriptors = msc_descriptors(max_burst=max_burst, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=max_burst),
            USB3BulkOUT(ept=2, max_burst=max_burst),
        ])
        if storage is None:
            storage = USB3MSCRAMStorage(size, image)
        self.storage = storage
        self.scsi    = USB3MSCSCSI(core.endpoints[0], core.endpoints[1], storage, inquiry_product)
