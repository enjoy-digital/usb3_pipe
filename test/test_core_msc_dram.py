#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage DRAM disk: storage backends and SCSI engine on a DRAM (Migen simulation)."""

import random
import struct
import unittest

from migen import *

from litex.gen import *

from litedram.common     import LiteDRAMNativePort
from litedram.modules    import MT41K512M16
from litedram.phy.model  import SDRAMPHYModel
from litedram.core       import LiteDRAMCore

from usb3_core.host_model import *
from usb3_core.msc        import *

from test.model.host import cbw

# Helpers ------------------------------------------------------------------------------------------

@passive
def native_port_model(port, mem, seed=0, latency=8, rate=0.7):
    """LiteDRAM native port model: sparse memory ({port word address: data}), random back-pressure
    on cmd/wdata, read data returned in order after latency cycles (with random gaps)."""
    rng        = random.Random(seed)
    data_bytes = port.data_width//8
    reads      = [] # [cycle, data].
    writes     = [] # Addresses waiting their write data.
    cycle      = 0
    while True:
        cmd_ready   = rng.random() < rate
        wdata_ready = (len(writes) > 0) and (rng.random() < rate)
        rdata_valid = (len(reads) > 0) and (reads[0][0] <= cycle) and (rng.random() < rate)
        yield port.cmd.ready.eq(cmd_ready)
        yield port.wdata.ready.eq(wdata_ready)
        yield port.rdata.valid.eq(rdata_valid)
        yield port.rdata.data.eq(reads[0][1] if rdata_valid else 0)
        yield
        cycle += 1
        if cmd_ready and (yield port.cmd.valid):
            adr = (yield port.cmd.addr)
            if (yield port.cmd.we):
                writes.append(adr)
            else:
                reads.append([cycle + latency, mem.get(adr, 0)])
        if wdata_ready and (yield port.wdata.valid):
            adr  = writes.pop(0)
            data = (yield port.wdata.data)
            we   = (yield port.wdata.we)
            mask = sum(0xff << (8*i) for i in range(data_bytes) if (we >> i) & 1)
            mem[adr] = (mem.get(adr, 0) & ~mask) | (data & mask)
        if rdata_valid and (yield port.rdata.ready):
            reads.pop(0)

def storage_write(storage, lba, words, rng):
    yield storage.cmd.valid.eq(1)
    yield storage.cmd.write.eq(1)
    yield storage.cmd.lba.eq(lba)
    yield storage.cmd.blocks.eq(len(words)//(MSC_BLOCK_SIZE//4))
    yield
    while not (yield storage.cmd.ready):
        yield
    yield storage.cmd.valid.eq(0)
    i = 0
    while i < len(words):
        valid = rng.random() < 0.8
        yield storage.sink.valid.eq(valid)
        yield storage.sink.data.eq(words[i])
        yield
        if valid and (yield storage.sink.ready):
            i += 1
    yield storage.sink.valid.eq(0)
    yield
    while not (yield storage.idle):
        yield

def storage_read(storage, lba, nwords, rng):
    yield storage.cmd.valid.eq(1)
    yield storage.cmd.write.eq(0)
    yield storage.cmd.lba.eq(lba)
    yield storage.cmd.blocks.eq(nwords//(MSC_BLOCK_SIZE//4))
    yield
    while not (yield storage.cmd.ready):
        yield
    yield storage.cmd.valid.eq(0)
    words = []
    while len(words) < nwords:
        ready = rng.random() < 0.8
        yield storage.source.ready.eq(ready)
        yield
        if ready and (yield storage.source.valid):
            words.append((yield storage.source.data))
    yield storage.source.ready.eq(0)
    return words

class SimDDR3(MT41K512M16):
    nrows = 64 # Small DRAM (1MB) for the simulation model.

# DUTs ---------------------------------------------------------------------------------------------

class DRAMStorageDUT(LiteXModule):
    """USB3MSCDRAM on a LiteDRAM controller with an SDRAM PHY model (1MB DDR3)."""
    def __init__(self, size, base):
        module = SimDDR3(125e6, "1:4")
        module.geom_settings.addressbits = 16
        self.phy  = SDRAMPHYModel(module, data_width=16, clk_freq=125e6)
        self.sdram = LiteDRAMCore(self.phy, module.geom_settings, module.timing_settings, 125e6)
        self.storage = USB3MSCDRAM(
            rd_port = self.sdram.crossbar.get_port("read"),
            wr_port = self.sdram.crossbar.get_port("write"),
            size    = size,
            base    = base)

class MSCDRAMDUT(LiteXModule):
    """USB3 MSC device on USB3MSCDRAM with native port models (sparse 512MB DRAM)."""
    def __init__(self, size, base):
        self.phy     = phy = PHYStandIn()
        self.ready   = Signal()
        self.rd_port = LiteDRAMNativePort("read",  address_width=25, data_width=128)
        self.wr_port = LiteDRAMNativePort("write", address_width=25, data_width=128)
        storage = USB3MSCDRAM(self.rd_port, self.wr_port, size=size, base=base, ready=self.ready)
        self.msc = USB3MSC(phy, 125e6, storage=storage, max_burst=2, product="USB3-PIPE DRAM Disk")

# Tests --------------------------------------------------------------------------------------------

class TestCoreMSCDRAM(unittest.TestCase):
    def storage_test(self, dut, ops):
        # Writes (lba, blocks), read back each region, check against a model.
        rng  = random.Random(42)
        disk = {}
        def generator():
            for lba, nblk in ops:
                words = [rng.getrandbits(32) for _ in range(nblk*MSC_BLOCK_SIZE//4)]
                yield from storage_write(dut.storage, lba, words, rng)
                for i, w in enumerate(words):
                    disk[lba*MSC_BLOCK_SIZE//4 + i] = w
                # Read back just after the write (write committed when idle).
                data = yield from storage_read(dut.storage, lba, len(words), rng)
                self.assertEqual(data, words)
            for lba, nblk in ops:
                n    = nblk*MSC_BLOCK_SIZE//4
                data = yield from storage_read(dut.storage, lba, n, rng)
                self.assertEqual(data, [disk[lba*MSC_BLOCK_SIZE//4 + i] for i in range(n)])
        return generator()

    def test_ram_storage(self):
        dut = LiteXModule()
        dut.storage = USB3MSCRAMStorage(32*MSC_BLOCK_SIZE)
        run_simulation(dut, self.storage_test(dut, [(0, 1), (29, 3), (5, 4), (6, 1), (16, 2)]))

    def test_dram_storage(self):
        # 256KB disk at 512KB in the DRAM (LiteDRAM controller, SDRAM PHY model).
        dut = DRAMStorageDUT(size=256*1024, base=512*1024)
        # Overlapping writes, last blocks of the disk.
        run_simulation(dut, self.storage_test(dut, [(1, 2), (2, 1), (511, 1)]))

    def test_dram_scsi_commands(self):
        size   = 256*1024*1024 # 256MB disk.
        base   = 256*1024*1024 # At 256MB in the DRAM.
        blocks = size//MSC_BLOCK_SIZE
        mem    = {}
        dut    = MSCDRAMDUT(size, base)
        host   = PyHost(dut.phy)
        state  = {"out_seq": 0, "in_seq": 0, "tag": 1}

        def command(cb, length=0, din=True, data_out=None):
            # CBW, optional data phase, CSW: returns (data in, residue, status).
            tag = state["tag"]
            state["tag"] += 1
            host.send_hp(dph, length=31, ept=2, dp_seq=state["out_seq"])
            host.send(data_packet_payload(cbw(tag, length, din, cb)))
            state["out_seq"] = (state["out_seq"] + 1) % 32
            if data_out is not None:
                for i in range(0, len(data_out), 1024):
                    chunk = data_out[i:i+1024]
                    host.send_hp(dph, length=len(chunk), ept=2, dp_seq=state["out_seq"])
                    host.send(data_packet_payload(chunk))
                    state["out_seq"] = (state["out_seq"] + 1) % 32
            count = 1 + ((-(-length//1024)) if (din and length) else 0)
            packets, state["in_seq"] = yield from pyhost_bulk_in(host, ept=1, count=count, seq=state["in_seq"])
            data = sum(packets[:-1], [])
            csw  = packets[-1]
            self.assertEqual(len(csw), 13)
            sig, csw_tag, residue, status = struct.unpack("<IIIB", bytes(csw))
            self.assertEqual((sig, csw_tag), (CSW_SIGNATURE, tag))
            return data, residue, status

        def rw10(op, lba, nblk):
            return [op, 0] + list(lba.to_bytes(4, "big")) + [0] + list(nblk.to_bytes(2, "big")) + [0]

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # DRAM not ready: TEST UNIT READY fails, REQUEST SENSE: NOT READY/LOGICAL UNIT NOT READY.
            data, residue, status = yield from command([SCSI_TEST_UNIT_READY])
            self.assertEqual(status, 1)
            data, residue, status = yield from command([SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0], length=18)
            self.assertEqual((data[2] & 0xf, data[12]), (0x02, 0x04))
            # READ(10) while not ready: padded, failed.
            data, residue, status = yield from command(rw10(SCSI_READ_10, 0, 1), length=512)
            self.assertEqual((data, status), ([0]*512, 1))
            # DRAM ready.
            yield dut.ready.eq(1)
            data, residue, status = yield from command([SCSI_TEST_UNIT_READY])
            self.assertEqual((data, residue, status), ([], 0, 0))
            # READ CAPACITY(10): DRAM disk size.
            data, residue, status = yield from command([SCSI_READ_CAPACITY_10], length=8)
            self.assertEqual(struct.unpack(">II", bytes(data)), (blocks - 1, 512))
            # WRITE(10) 3 blocks at the end of the disk, READ(10) back.
            lba   = blocks - 3
            block = [(13*i + 5) & 0xff for i in range(3*512)]
            data, residue, status = yield from command(rw10(SCSI_WRITE_10, lba, 3), length=3*512,
                din=False, data_out=block)
            self.assertEqual((residue, status), (0, 0))
            data, residue, status = yield from command(rw10(SCSI_READ_10, lba, 3), length=3*512)
            self.assertEqual((data, residue, status), (block, 0, 0))
            # DRAM layout: little endian, disk at base.
            adr = (base + lba*512)//16
            self.assertEqual(mem[adr], int.from_bytes(bytes(block[:16]), "little"))
            # WRITE(10)/READ(10) 2 blocks at a large LBA (> 16-bit).
            lba   = 0x12345
            block = [(7*i + 1) & 0xff for i in range(2*512)]
            data, residue, status = yield from command(rw10(SCSI_WRITE_10, lba, 2), length=2*512,
                din=False, data_out=block)
            self.assertEqual((residue, status), (0, 0))
            data, residue, status = yield from command(rw10(SCSI_READ_10, lba - 1, 4), length=4*512)
            self.assertEqual((data, residue, status), ([0]*512 + block + [0]*512, 0, 0))
            # Out of range READ(10): failed, ILLEGAL REQUEST/LBA OUT OF RANGE.
            data, residue, status = yield from command(rw10(SCSI_READ_10, blocks - 1, 2), length=1024)
            self.assertEqual(status, 1)
            data, residue, status = yield from command([SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0], length=18)
            self.assertEqual((data[2] & 0xf, data[12]), (0x05, 0x21))

        run_simulation(dut, [
            host.driver(),
            host.monitor(1.0),
            native_port_model(dut.rd_port, mem, seed=1),
            native_port_model(dut.wr_port, mem, seed=2),
            scenario(),
        ], vcd_name=None)

if __name__ == "__main__":
    unittest.main()
