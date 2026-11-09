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

- Bulk-Only Transport (USB MSC BOT 1.0): CBW (31 bytes) on Bulk OUT, optional data phase, CSW (13
  bytes) on Bulk IN. Get Max LUN/Bulk-Only Mass Storage Reset class requests (constant responses).
- SCSI commands: TEST UNIT READY, REQUEST SENSE, INQUIRY, MODE SENSE(6), START STOP UNIT (no
  effect), PREVENT ALLOW MEDIUM REMOVAL, READ FORMAT CAPACITIES, READ CAPACITY(10), READ(10), WRITE(10), SYNCHRONIZE CACHE(10).
  Other commands fail (ILLEGAL REQUEST sense), their data phase is padded (IN) or discarded (OUT).
"""

import os
import tempfile
import subprocess

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import bytes_to_be
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT, bulk_endpoint_32
from usb3_core.descriptors import *
from usb3_core.msc_storage import *

# Constants ----------------------------------------------------------------------------------------

CBW_SIGNATURE = 0x43425355 # "USBC".
CSW_SIGNATURE = 0x53425355 # "USBS".

SCSI_TEST_UNIT_READY       = 0x00
SCSI_REQUEST_SENSE         = 0x03
SCSI_INQUIRY               = 0x12
SCSI_MODE_SENSE_6          = 0x1a
SCSI_START_STOP_UNIT       = 0x1b
SCSI_PREVENT_ALLOW_REMOVAL = 0x1e
SCSI_READ_FORMAT_CAPACITY  = 0x23
SCSI_READ_CAPACITY_10      = 0x25
SCSI_READ_10               = 0x28
SCSI_WRITE_10              = 0x2a
SCSI_SYNCHRONIZE_CACHE_10  = 0x35

# Sense data (SPC-4 4.5): fixed format response code, sense keys, additional sense codes (ASC).
SENSE_FIXED_CURRENT        = 0x70
SENSE_KEY_NO_SENSE         = 0x0
SENSE_KEY_NOT_READY        = 0x2
SENSE_KEY_ILLEGAL_REQUEST  = 0x5
SENSE_KEY_UNIT_ATTENTION   = 0x6
SENSE_ASC_NONE             = 0x00
SENSE_ASC_NOT_READY        = 0x04 # LOGICAL UNIT NOT READY.
SENSE_ASC_INVALID_OPCODE   = 0x20 # INVALID COMMAND OPERATION CODE.
SENSE_ASC_LBA_OUT_OF_RANGE = 0x21 # LOGICAL BLOCK ADDRESS OUT OF RANGE.
SENSE_ASC_LUN_UNSUPPORTED  = 0x25 # LOGICAL UNIT NOT SUPPORTED.
SENSE_ASC_MEDIUM_CHANGED   = 0x28 # NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED.
SENSE_ASC_NO_MEDIUM        = 0x3a # MEDIUM NOT PRESENT.

# Descriptors --------------------------------------------------------------------------------------

def msc_descriptors(in_ept=1, out_ept=2, max_burst=8, luns=1, **kwargs):
    kwargs.setdefault("product", "USB3-PIPE RAM Disk")
    interface = USB3InterfaceDescriptor(0, cls=0x08, subclass=0x06, protocol=0x50, endpoints=[
        USB3EndpointDescriptor(in_ept,  direction=1, max_burst=max_burst),
        USB3EndpointDescriptor(out_ept, direction=0, max_burst=max_burst),
    ])
    descriptors = USB3Descriptors(interfaces=[interface], **kwargs)
    add_msc_requests(descriptors, luns)
    return descriptors

def add_msc_requests(descriptors, luns=1):
    """Bulk-Only Transport class requests: Bulk-Only Mass Storage Reset, Get Max LUN."""
    descriptors.add_class_request(0x21, 0xff)             # Bulk-Only Mass Storage Reset.
    descriptors.add_class_request(0xa1, 0xfe, [luns - 1]) # Get Max LUN.

# Disk Image ---------------------------------------------------------------------------------------

def msc_fat_image(size, files=None, label="USB3PIPE"):
    """FAT disk image (mkfs.fat + mtools) with files ({name: content}), None without the tools."""
    files = {} if files is None else files
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
    """Responses: {opcode: bytes} (READ CAPACITY(10)/READ FORMAT CAPACITIES: for a fixed capacity,
    see scsi_capacity_words for the runtime capacity)."""
    def pad(s, n):
        return list(s.encode()[:n].ljust(n, b" "))
    last_lba = blocks - 1
    return {
        # Removable direct access device, SPC-3, response format 2.
        SCSI_INQUIRY : [0x00, 0x80, 0x05, 0x02, 31, 0, 0, 0] + pad(vendor, 8) + pad(product, 16) +
            pad(revision, 4),
        SCSI_MODE_SENSE_6 : [3, 0, 0, 0],
        SCSI_READ_CAPACITY_10 : list(last_lba.to_bytes(4, "big")) +
            list(MSC_BLOCK_SIZE.to_bytes(4, "big")),
        # Formatted media: capacity list header + current descriptor.
        SCSI_READ_FORMAT_CAPACITY : [0, 0, 0, 8] + list(blocks.to_bytes(4, "big")) + [0x02, 0] +
            list(MSC_BLOCK_SIZE.to_bytes(2, "big")),
    }

def bswap32(v):
    """32-bit big-endian bytes of v (Signal), little-endian word (first byte on [0:8])."""
    return Cat(v[24:32], v[16:24], v[8:16], v[0:8])

def scsi_capacity_words(capacity):
    """READ CAPACITY(10)/READ FORMAT CAPACITIES responses words (little-endian 32-bit words) from a
    capacity Signal (blocks)."""
    block_size = MSC_BLOCK_SIZE.to_bytes(4, "big")
    return {
        SCSI_READ_CAPACITY_10 : [bswap32(capacity - 1), int.from_bytes(block_size, "little")],
        # Formatted media: capacity list header + current descriptor.
        SCSI_READ_FORMAT_CAPACITY : [int.from_bytes(bytes([0, 0, 0, 8]), "little"), bswap32(capacity),
            int.from_bytes(bytes([0x02, 0]) + block_size[2:], "little")],
    }

# USB3 MSC SCSI ------------------------------------------------------------------------------------

class USB3MSCSCSI(LiteXModule):
    """Bulk-Only Transport/SCSI command engine on storages (USB3MSCStorage: BRAM/DRAM/...).

    storage: a storage, or a list of storages (one per LUN, CBW bCBWLUN), inquiry_product: INQUIRY
    product (or one per LUN). Capacities at runtime (storage.capacity), UNIT ATTENTION (medium may
    have changed) on the first command after a storage becomes ready (ex: SD card inserted).
    """
    def __init__(self, bulk_in, bulk_out, storage, inquiry_product="USB3-PIPE RAMDisk"):
        storages = storage if isinstance(storage, (list, tuple)) else [storage]
        products = inquiry_product if isinstance(inquiry_product, (list, tuple)) else \
            [inquiry_product]*len(storages)
        assert len(products) == len(storages)
        n = len(storages)

        # # #

        # LUN (latched at the CBW decode), selected storage.
        lun     = Signal(max=max(n, 2))
        cbw_lun = Signal(4)
        def sel(values, width=1):
            # Selected LUN value (Signal: operators on Array proxies, ex: ~, not width safe in
            # simulation).
            v = Signal(width)
            self.comb += v.eq(Array(values)[lun])
            return v
        st_ready    = sel([s.ready    for s in storages])
        st_idle     = sel([s.idle     for s in storages])
        st_capacity = sel([s.capacity for s in storages], 32)
        st_no_asc   = sel([C(SENSE_ASC_NO_MEDIUM if s.removable else SENSE_ASC_NOT_READY, 8)
            for s in storages], 8)

        # Responses ROM (lun, opcode -> offset/length, same length for all the LUNs): constant
        # responses, capacities from the selected storage.
        rom   = []
        table = {}
        for l, product in enumerate(products):
            responses = scsi_responses(1, product=product)
            words     = scsi_capacity_words(st_capacity)
            for opcode, data in responses.items():
                offset = len(rom)
                length = len(data)
                if opcode in words:
                    rom += words[opcode]
                else:
                    data = data + [0]*((-len(data)) % 4)
                    rom += [int.from_bytes(bytes(data[i:i+4]), "little")
                        for i in range(0, len(data), 4)]
                table.setdefault(opcode, (length, {}))[1][l] = offset
        rom = Array(Constant(w, 32) if isinstance(w, int) else w for w in rom)

        # UNIT ATTENTION: storage becoming ready (one pending flag per LUN).
        ua       = Signal(n)
        ua_clear = Signal() # UNIT ATTENTION of the selected LUN reported.
        for i, st in enumerate(storages):
            ready_d = Signal(reset=1) # Ready at reset: no UNIT ATTENTION (fixed storages).
            self.sync += [
                ready_d.eq(st.ready),
                If(st.ready & ~ready_d,
                    ua[i].eq(1)
                ).Elif(ua_clear & (lun == i),
                    ua[i].eq(0)
                )
            ]
        ua_lun = sel([ua[i] for i in range(n)])

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
            cbw_lun.eq(cbw[3][8:12]),
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

        # Storage command (READ(10)/WRITE(10)) and data, to/from the selected storage.
        st_cmd_valid = Signal()
        st_src_ready = Signal()
        st_snk_valid = Signal()
        st_snk_data  = Signal(32)
        for i, st in enumerate(storages):
            self.comb += [
                st.cmd.valid.eq(st_cmd_valid & (lun == i)),
                st.cmd.write.eq(op == SCSI_WRITE_10),
                st.cmd.lba.eq(lba),
                st.cmd.blocks.eq(nblk),
                st.source.ready.eq(st_src_ready & (lun == i)),
                st.sink.valid.eq(st_snk_valid & (lun == i)),
                st.sink.data.eq(st_snk_data),
            ]
        st_cmd_ready = sel([s.cmd.ready    for s in storages])
        st_src_valid = sel([s.source.valid for s in storages])
        st_src_data  = sel([s.source.data  for s in storages], 32)
        st_snk_ready = sel([s.sink.ready   for s in storages])

        # Sense response, fixed format: byte 0: response code, byte 2: sense key, byte 7: additional
        # length (10), byte 12: ASC.
        sense = Array([
            Cat(C(SENSE_FIXED_CURRENT, 8), C(0, 8), sense_key, C(0, 4), C(0, 8)),
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
                        NextState("LUN")
                    )
                )
            )
        )
        # LUN selection (before the decode: selected storage status).
        fsm.act("LUN",
            NextValue(lun, Mux(cbw_lun < n, cbw_lun, 0)),
            NextState("DECODE")
        )
        decode_cases = {}
        for opcode, (length, offsets) in table.items():
            decode_cases[opcode] = [
                Case(lun, {l: NextValue(adr, offset) for l, offset in offsets.items()}),
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
        for opcode in [SCSI_START_STOP_UNIT, SCSI_PREVENT_ALLOW_REMOVAL, SCSI_SYNCHRONIZE_CACHE_10]:
            decode_cases[opcode] = [NextValue(residue, dlen), NextState("CSW")]
        # Storage not ready: NOT READY/LOGICAL UNIT NOT READY (ex: DRAM not yet initialized) or
        # MEDIUM NOT PRESENT (removable storages, ex: no SD card).
        not_ready = [
            NextValue(sense_key, SENSE_KEY_NOT_READY),
            NextValue(sense_asc, st_no_asc),
            NextValue(status, 1),
        ]
        decode_cases[SCSI_TEST_UNIT_READY] = [
            NextValue(residue, dlen),
            If(~st_ready, *not_ready),
            NextState("CSW"),
        ]
        decode_cases[SCSI_READ_10] = [
            NextValue(remaining, nblk*MSC_BLOCK_SIZE),
            NextValue(residue, dlen - nblk*MSC_BLOCK_SIZE),
            If(~st_ready,
                *not_ready,
                NextValue(remaining, dlen),
                NextState("PAD"),
            ).Elif((lba + nblk) > st_capacity,
                NextValue(sense_key, SENSE_KEY_ILLEGAL_REQUEST),
                NextValue(sense_asc, SENSE_ASC_LBA_OUT_OF_RANGE),
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
            If(~st_ready,
                *not_ready,
                NextValue(out, dlen),
                NextState("DISCARD"),
            ).Elif((lba + nblk) > st_capacity,
                NextValue(sense_key, SENSE_KEY_ILLEGAL_REQUEST),
                NextValue(sense_asc, SENSE_ASC_LBA_OUT_OF_RANGE),
                NextValue(status, 1),
                NextValue(out, dlen),
                NextState("DISCARD"),
            ).Elif(nblk == 0,
                NextState("CSW"),
            ).Else(
                NextState("WRITE-CMD")
            ),
        ]
        def failed(key, asc):
            # Failed command: sense key/ASC, data phase padded/discarded.
            return [
                NextValue(sense_key, key),
                NextValue(sense_asc, asc),
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
        # Unsupported: ILLEGAL REQUEST/INVALID COMMAND OPERATION CODE.
        decode_cases["default"] = failed(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_INVALID_OPCODE)
        fsm.act("DECODE",
            NextValue(status, 0),
            If(op != SCSI_REQUEST_SENSE,
                NextValue(sense_key, SENSE_KEY_NO_SENSE),
                NextValue(sense_asc, SENSE_ASC_NONE),
            ),
            If(cbw_lun >= n,
                # Unsupported LUN: ILLEGAL REQUEST/LOGICAL UNIT NOT SUPPORTED.
                *failed(SENSE_KEY_ILLEGAL_REQUEST, SENSE_ASC_LUN_UNSUPPORTED)
            ).Elif(ua_lun & (op != SCSI_INQUIRY) & (op != SCSI_REQUEST_SENSE),
                # Storage became ready: UNIT ATTENTION/MEDIUM MAY HAVE CHANGED (once).
                ua_clear.eq(1),
                *failed(SENSE_KEY_UNIT_ATTENTION, SENSE_ASC_MEDIUM_CHANGED)
            ).Else(
                Case(op, decode_cases)
            )
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
                NextValue(sense_key, SENSE_KEY_NO_SENSE),
                NextValue(sense_asc, SENSE_ASC_NONE),
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
            st_cmd_valid.eq(1),
            If(st_cmd_ready,
                NextState("READ")
            )
        )
        fsm.act("READ",
            bulk_in.sink.valid.eq(st_src_valid),
            bulk_in.sink.data.eq(st_src_data),
            bulk_in.sink.last.eq(remaining <= 4),
            bulk_in.sink.be.eq(0b1111),
            st_src_ready.eq(bulk_in.sink.ready),
            If(st_src_valid & bulk_in.sink.ready,
                NextValue(remaining, remaining - 4),
                If(remaining <= 4, NextState("CSW"))
            )
        )
        fsm.act("WRITE-CMD",
            st_cmd_valid.eq(1),
            If(st_cmd_ready,
                NextState("WRITE")
            )
        )
        fsm.act("WRITE",
            st_snk_valid.eq(bulk_out.source.valid),
            st_snk_data.eq(bulk_out.source.data),
            bulk_out.source.ready.eq(st_snk_ready),
            If(bulk_out.source.valid & st_snk_ready,
                NextValue(out, out - 4),
                If(out <= 4, NextState("WRITE-DONE"))
            )
        )
        fsm.act("WRITE-DONE",
            # Data committed to the storage before the CSW (ordering with the following reads).
            If(st_idle,
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
        csw      = Array([C(CSW_SIGNATURE, 32), tag, residue, Cat(status, C(0, 24))])
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
    """USB3 Mass Storage device: block RAM disk (size, optional image) or storage (USB3MSCDRAM), or
    a list of storages (one per LUN, inquiry_product: one per LUN)."""
    def __init__(self, phy, sys_clk_freq, size=512*1024, image=None, storage=None, max_burst=8,
        inquiry_product="USB3-PIPE RAMDisk", **kwargs):
        luns = len(storage) if isinstance(storage, (list, tuple)) else 1
        descriptors = msc_descriptors(max_burst=max_burst, luns=luns, **kwargs)
        dw = 32*getattr(phy, "link_words", 1) # Bulk endpoints at the Link Layer width.
        self.core = core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=max_burst, data_width=dw),
            USB3BulkOUT(ept=2, max_burst=max_burst, data_width=dw),
        ])
        if storage is None:
            storage = USB3MSCRAMStorage(size, image)
        if isinstance(storage, (list, tuple)):
            for i, st in enumerate(storage):
                self.add_module(name=f"storage{i}", module=st)
            storage = list(storage)
        else:
            self.storage = storage
        self.scsi    = USB3MSCSCSI(
            bulk_endpoint_32(self, core.endpoints[0]),
            bulk_endpoint_32(self, core.endpoints[1]),
            storage, inquiry_product)
