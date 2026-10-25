#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Mass Storage RAM disk: Bulk-Only Transport/SCSI commands (Migen simulation, reactive host)."""

import struct
import unittest

import pytest

from migen import *

from litex.gen import *

from usb3_core.host_model import *
from usb3_core.msc        import *

from test.model.host import *

SIZE = 16*1024 # 32 blocks.

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        self.msc = USB3MSC(phy, 125e6, size=SIZE, max_burst=2)

class TestCoreMSC(unittest.TestCase):
    def scsi_commands_test(self, with_data=True):
        dut  = DUT()
        host = PyHost(dut.phy)
        state = {"out_seq": 0, "in_seq": 0, "tag": 1}

        def command(cb, length=0, din=True, data_out=None):
            # CBW, optional data phase, CSW: returns (data in, csw).
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

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # Get Max LUN.
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0xa1, 0xfe, 0, 0, 1))), [0])
            # TEST UNIT READY.
            data, residue, status = yield from command([SCSI_TEST_UNIT_READY])
            self.assertEqual((data, residue, status), ([], 0, 0))
            # INQUIRY.
            data, residue, status = yield from command([SCSI_INQUIRY, 0, 0, 0, 36, 0], length=36)
            self.assertEqual(data, scsi_responses(SIZE//512)[SCSI_INQUIRY])
            self.assertEqual(status, 0)
            # READ CAPACITY(10).
            data, residue, status = yield from command([SCSI_READ_CAPACITY_10], length=8)
            self.assertEqual(struct.unpack(">II", bytes(data)), (SIZE//512 - 1, 512))
            # WRITE(10) 2 blocks at LBA 5, READ(10) back.
            if with_data:
                block = [(7*i + 1) & 0xff for i in range(1024)]
                data, residue, status = yield from command([SCSI_WRITE_10, 0, 0, 0, 0, 5, 0, 0, 2, 0], length=1024,
                    din=False, data_out=block)
                self.assertEqual((residue, status), (0, 0))
                data, residue, status = yield from command([SCSI_READ_10, 0, 0, 0, 0, 5, 0, 0, 2, 0], length=1024)
                self.assertEqual((data, residue, status), (block, 0, 0))
            # Unsupported command (IN 8 bytes): padded, failed, REQUEST SENSE: ILLEGAL REQUEST/0x20.
            data, residue, status = yield from command([0xff], length=8)
            self.assertEqual((data, status), ([0]*8, 1))
            data, residue, status = yield from command([SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0], length=18)
            self.assertEqual((data[0], data[2] & 0xf, data[12]), (0x70, 0x05, 0x20))
            # Out of range READ(10): failed.
            data, residue, status = yield from command([SCSI_READ_10, 0, 0, 0, 0, 31, 0, 0, 2, 0], length=1024)
            self.assertEqual(status, 1)
        run_pyhost(dut, host, scenario())

    def test_scsi_commands(self):
        # Short: no WRITE(10)/READ(10) data phases.
        self.scsi_commands_test(with_data=False)

    @pytest.mark.slow
    def test_scsi_commands_data(self):
        self.scsi_commands_test(with_data=True)

    def test_ram_banks(self):
        # Disk RAM built from several banks: initialization, writes and reads across the banks.
        words = 64
        init  = [(0x1000 + i) for i in range(words)]
        dut   = USB3MSCRAM(words, init=init, bank_words=16)
        mem   = list(init)
        def generator():
            # Write odd words.
            for i in range(1, words, 2):
                yield dut.wr_port.adr.eq(i)
                yield dut.wr_port.dat_w.eq(0xcafe0000 + i)
                yield dut.wr_port.we.eq(1)
                yield
                mem[i] = 0xcafe0000 + i
            yield dut.wr_port.we.eq(0)
            # Read all words (1 cycle latency).
            for i in range(words):
                yield dut.rd_port.adr.eq(i)
                yield dut.rd_port.re.eq(1)
                yield
                yield dut.rd_port.re.eq(0)
                yield
                self.assertEqual((yield dut.rd_port.dat_r), mem[i])
        run_simulation(dut, generator())

    def test_fat_image(self):
        image = msc_fat_image(512*1024, {"README.TXT": "Hello\n"})
        if image is None:
            self.skipTest("mkfs.fat/mtools not available")
        self.assertEqual(len(image), 512*1024)
        self.assertEqual(image[510:512], b"\x55\xaa") # Boot sector signature.

if __name__ == "__main__":
    unittest.main()
