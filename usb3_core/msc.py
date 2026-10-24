#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage (Bulk-Only Transport, SCSI transparent command set) RAM disk.

A USB3 device exposing a disk (/dev/sdX, no driver needed) backed by FPGA block RAM, optionally
initialized with a disk image (ex: a FAT file system with files, see msc_fat_image).

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

# USB3 MSC RAM Disk ----------------------------------------------------------------------------------

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

# USB3 MSC SCSI ------------------------------------------------------------------------------------

class USB3MSCSCSI(LiteXModule):
    """Bulk-Only Transport/SCSI command engine on a block RAM disk."""
    def __init__(self, bulk_in, bulk_out, size, image=None):
        assert size % MSC_BLOCK_SIZE == 0
        blocks = size//MSC_BLOCK_SIZE
        words  = size//4

        # # #

        # Disk RAM.
        init = []
        if image is not None:
            image = bytes(image[:size]).ljust(size, b"\x00")
            init  = [int.from_bytes(image[4*i:4*i+4], "little") for i in range(words)]
        self.ram = ram = USB3MSCRAM(words, init=init)
        rd_port  = ram.rd_port
        wr_port  = ram.wr_port

        # Responses ROM (opcode -> offset/length).
        responses = scsi_responses(blocks)
        rom   = []
        table = {}
        for opcode, data in responses.items():
            table[opcode] = (len(rom), len(data))
            data = data + [0]*((-len(data)) % 4)
            rom += [int.from_bytes(bytes(data[i:i+4]), "little") for i in range(0, len(data), 4)]
        rom = Array(Constant(w, 32) for w in rom)

        # CBW.
        cbw   = Array(Signal(32) for _ in range(8))
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
        residue = Signal(32)
        status  = Signal(8)

        # Data phase.
        adr       = Signal(max=max(words, len(rom), 2))
        remaining = Signal(32) # Bytes to send.
        out       = Signal(32) # Bytes to receive/discard.

        # RAM reader (1 cycle latency, prefetch FIFO).
        self.fifo = fifo = stream.SyncFIFO([("data", 32), ("last_be", 4)], 8)
        rd_issue = Signal()
        rd_valid = Signal()
        rd_last  = Signal()
        rd_rem   = Signal(32)
        self.comb += [
            rd_port.adr.eq(adr),
            rd_port.re.eq(rd_issue),
        ]
        self.sync += [
            rd_valid.eq(rd_issue),
            rd_last.eq(rd_issue & (rd_rem <= 4)),
        ]
        self.comb += [
            fifo.sink.valid.eq(rd_valid),
            fifo.sink.data.eq(rd_port.dat_r),
            fifo.sink.last.eq(rd_last),
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
        sense_word = Signal(3)

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
        for opcode in [SCSI_TEST_UNIT_READY, SCSI_PREVENT_ALLOW_REMOVAL, SCSI_SYNCHRONIZE_CACHE_10]:
            decode_cases[opcode] = [NextValue(residue, dlen), NextState("CSW")]
        decode_cases[SCSI_READ_10] = [
            NextValue(adr, lba*(MSC_BLOCK_SIZE//4)),
            NextValue(remaining, nblk*MSC_BLOCK_SIZE),
            NextValue(rd_rem, nblk*MSC_BLOCK_SIZE),
            NextValue(residue, dlen - nblk*MSC_BLOCK_SIZE),
            If((lba + nblk) > blocks,
                NextValue(sense_key, 0x05), NextValue(sense_asc, 0x21), # LBA out of range.
                NextValue(status, 1),
                NextValue(remaining, dlen),
                NextState("PAD"),
            ).Elif(nblk == 0,
                NextState("CSW"),
            ).Else(
                NextState("READ")
            ),
        ]
        decode_cases[SCSI_WRITE_10] = [
            NextValue(adr, lba*(MSC_BLOCK_SIZE//4)),
            NextValue(out, nblk*MSC_BLOCK_SIZE),
            NextValue(residue, dlen - nblk*MSC_BLOCK_SIZE),
            If((lba + nblk) > blocks,
                NextValue(sense_key, 0x05), NextValue(sense_asc, 0x21),
                NextValue(status, 1),
                NextValue(out, dlen),
                NextState("DISCARD"),
            ).Elif(nblk == 0,
                NextState("CSW"),
            ).Else(
                NextState("WRITE")
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
        def last_be(n):
            return Mux(n >= 4, 0, Mux(n == 1, 0b0001, Mux(n == 2, 0b0010, 0b0100)))
        fsm.act("ROM",
            bulk_in.sink.valid.eq(remaining != 0),
            bulk_in.sink.data.eq(rom[adr]),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.last_be.eq(last_be(remaining)),
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
            bulk_in.sink.last_be.eq(last_be(remaining)),
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
            bulk_in.sink.last_be.eq(last_be(remaining)),
            If(remaining == 0,
                NextState("CSW")
            ).Elif(bulk_in.sink.ready,
                NextValue(remaining, Mux(remaining <= 4, 0, remaining - 4)),
            )
        )
        fsm.act("READ",
            rd_issue.eq((rd_rem != 0) & (fifo.level < 4)),
            If(rd_issue,
                NextValue(adr, adr + 1),
                NextValue(rd_rem, rd_rem - 4),
            ),
            fifo.source.connect(bulk_in.sink),
            If(fifo.source.valid & fifo.source.ready & fifo.source.last,
                NextState("CSW")
            )
        )
        fsm.act("WRITE",
            bulk_out.source.ready.eq(1),
            wr_port.adr.eq(adr),
            wr_port.dat_w.eq(bulk_out.source.data),
            If(bulk_out.source.valid,
                wr_port.we.eq(1),
                NextValue(adr, adr + 1),
                NextValue(out, out - 4),
                If(out <= 4, NextState("CSW"))
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
            bulk_in.sink.last_be.eq(0b0001), # 13 bytes.
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
    """USB3 Mass Storage RAM disk device."""
    def __init__(self, phy, sys_clk_freq, size=512*1024, image=None, max_burst=8, **kwargs):
        descriptors = msc_descriptors(max_burst=max_burst, **kwargs)
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=max_burst),
            USB3BulkOUT(ept=2, max_burst=max_burst),
        ])
        self.scsi = USB3MSCSCSI(core.endpoints[0], core.endpoints[1], size, image)
