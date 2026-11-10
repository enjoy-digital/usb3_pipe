#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage with several LUNs: Get Max LUN, INQUIRY per LUN, runtime capacity/ready (ex:
SD card inserted), UNIT ATTENTION, data per LUN, unsupported LUN (Migen simulation, reactive
host)."""

import struct
import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.msc         import *
from usb3_core.msc_storage import USB3MSCStorage, USB3MSCRAMStorage

from test.model.host import *

SIZE = 16*1024 # 32 blocks per RAM.

# Runtime Storage (test) ---------------------------------------------------------------------------

class RuntimeStorage(USB3MSCStorage):
    """RAM storage with a runtime ready/capacity (ex: SD card): ready/capacity driven by the test."""
    def __init__(self, size):
        USB3MSCStorage.__init__(self, removable=True)
        self.ram = ram = USB3MSCRAMStorage(size)
        self.ctrl_ready    = Signal()
        self.ctrl_capacity = Signal(32)
        self.comb += [
            self.cmd.connect(ram.cmd),
            ram.source.connect(self.source),
            self.sink.connect(ram.sink),
            self.idle.eq(ram.idle),
            self.ready.eq(self.ctrl_ready),
            self.capacity.eq(self.ctrl_capacity),
        ]

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        self.lun1 = lun1 = RuntimeStorage(SIZE)
        self.msc = USB3MSC(phy, 125e6, max_burst=2,
            storage         = [USB3MSCRAMStorage(SIZE), lun1],
            inquiry_product = ["USB3-PIPE LUN0", "USB3-PIPE LUN1"])

# Tests --------------------------------------------------------------------------------------------

class TestCoreMSCLUNs(unittest.TestCase):
    def test_luns(self):
        dut   = DUT()
        host  = PyHost(dut.phy)
        state = {"out_seq": 0, "in_seq": 0, "tag": 1}

        def command(cb, length=0, din=True, data_out=None, lun=0):
            tag = state["tag"]
            state["tag"] += 1
            host.send_hp(dph, length=31, ept=2, dp_seq=state["out_seq"])
            host.send(data_packet_payload(cbw(tag, length, din, cb, lun=lun)))
            state["out_seq"] = (state["out_seq"] + 1) % 32
            if data_out is not None:
                for i in range(0, len(data_out), 1024):
                    chunk = data_out[i:i+1024]
                    host.send_hp(dph, length=len(chunk), ept=2, dp_seq=state["out_seq"])
                    host.send(data_packet_payload(chunk))
                    state["out_seq"] = (state["out_seq"] + 1) % 32
            count = 1 + ((-(-length//1024)) if (din and length) else 0)
            packets, state["in_seq"] = yield from pyhost_bulk_in(host, ept=1, count=count,
                seq=state["in_seq"])
            data = sum(packets[:-1], [])
            sig, csw_tag, residue, status = struct.unpack("<IIIB", bytes(packets[-1]))
            self.assertEqual((sig, csw_tag), (CSW_SIGNATURE, tag))
            return data, status

        def sense(lun):
            data, status = yield from command([SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0], length=18,
                lun=lun)
            return (data[2] & 0xf, data[12])

        def capacity(lun):
            data, status = yield from command([SCSI_READ_CAPACITY_10], length=8, lun=lun)
            return struct.unpack(">II", bytes(data))

        def rw10(op, lba, nblk):
            return ([op, 0] + list(lba.to_bytes(4, "big")) + [0] + list(nblk.to_bytes(2, "big")) +
                [0])

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # Get Max LUN: 2 LUNs.
            self.assertEqual((yield from pyhost_control_in(host,
                setup_packet(0xa1, 0xfe, 0, 0, 1))), [1])
            # INQUIRY per LUN.
            for lun in range(2):
                data, status = yield from command([SCSI_INQUIRY, 0, 0, 0, 36, 0], length=36,
                    lun=lun)
                self.assertEqual(bytes(data[16:32]).rstrip(), f"USB3-PIPE LUN{lun}".encode())
            # LUN0 ready, LUN1 not ready (removable: MEDIUM NOT PRESENT), START STOP UNIT accepted.
            self.assertEqual((yield from command([SCSI_TEST_UNIT_READY], lun=0))[1], 0)
            self.assertEqual((yield from command([SCSI_TEST_UNIT_READY], lun=1))[1], 1)
            self.assertEqual((yield from sense(1)), (0x02, 0x3a))
            self.assertEqual((yield from command([SCSI_START_STOP_UNIT, 0, 0, 0, 1, 0], lun=1))[1], 0)
            # LUN1 ready with a runtime capacity: UNIT ATTENTION once, then ready, capacity.
            yield dut.lun1.ctrl_capacity.eq(24)
            yield dut.lun1.ctrl_ready.eq(1)
            self.assertEqual((yield from command([SCSI_TEST_UNIT_READY], lun=1))[1], 1)
            self.assertEqual((yield from sense(1)), (0x06, 0x28))
            self.assertEqual((yield from command([SCSI_TEST_UNIT_READY], lun=1))[1], 0)
            self.assertEqual((yield from capacity(0)), (SIZE//512 - 1, 512))
            self.assertEqual((yield from capacity(1)), (24 - 1, 512))
            # Data per LUN: block written to LUN1 LBA 3, LUN0 LBA 3 unchanged.
            block = [(5*i + 3) & 0xff for i in range(512)]
            data, status = yield from command(rw10(SCSI_WRITE_10, 3, 1), length=512, din=False,
                data_out=block, lun=1)
            self.assertEqual(status, 0)
            self.assertEqual((yield from command(rw10(SCSI_READ_10, 3, 1), length=512, lun=1)),
                (block, 0))
            self.assertEqual((yield from command(rw10(SCSI_READ_10, 3, 1), length=512, lun=0)),
                ([0]*512, 0))
            # LUN1 out of its runtime capacity: failed.
            data, status = yield from command(rw10(SCSI_READ_10, 24, 1), length=512, lun=1)
            self.assertEqual(status, 1)
            self.assertEqual((yield from sense(1)), (0x05, 0x21))
            # Unsupported LUN: ILLEGAL REQUEST/LOGICAL UNIT NOT SUPPORTED.
            self.assertEqual((yield from command([SCSI_TEST_UNIT_READY], lun=2))[1], 1)
            self.assertEqual((yield from sense(0)), (0x05, 0x25))
        run_pyhost(dut, host, scenario())

if __name__ == "__main__":
    unittest.main()
