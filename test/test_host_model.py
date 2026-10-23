#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_core.host_model import *

def words(s):
    """Link Layer words from a capture string ("fbfbfbf7 08000004 ..."), framing words as K."""
    r = []
    for w in s.split():
        data = int(w, 16)
        r.append((data, 0b1111 if data in [0xfefefef7, 0xfbfbfbf7, 0x5c5c5cf7, 0xfdfdfdf7] else 0))
    return r

# Packets captured on hardware (Acorn + xHCI host, LiteScope at the USB3 Core interface).
HW_PACKETS = [
    (link_command(LGOOD(7)),                                   "fefefef7 07680768"),
    (link_command(LCRD(0)),                                    "fefefef7 80a080a0"),
    (link_command(LUP()),                                      "fefefef7 00b400b4"),
    (link_command(LDN()),                                      "fefefef7 802d802d"),
    (lmp_port_capability(seq=0),                               "fbfbfbf7 80020000 04000100 00000000 45180010"),
    (lmp_port_configuration(seq=1),                            "fbfbfbf7 a0020000 00000000 00000000 644301e8"),
    (dph(seq=3, length=8, addr=2, setup=1),                    "fbfbfbf7 08000004 00800800 00000000 af7f0350"),
    (data_packet_payload(setup_packet(0x00, 0x09, 1, 0, 0)),   "5c5c5cf7 00090100 00000000 1f49e63b fdfdfdf7"),
    (tp(TP_SUB_ACK, seq=3, addr=2, nump=1, tp_seq=0, pp=1),    "fbfbfbf7 04000004 01000100 00000008 24130350"),
    (tp(TP_SUB_STATUS, seq=4, addr=2),                         "fbfbfbf7 04000004 04000000 00000000 ba480428"),
    (tp(TP_SUB_ACK, seq=4, addr=2, tp_seq=1),                  "fbfbfbf7 04000004 01002000 00000000 27e80428"),
]

# Host Model -------------------------------------------------------------------------------------

class TestHostModel(unittest.TestCase):
    def test_packets_vs_hardware(self):
        # Packets encoders (framing, CRC-5/16/32, Link Control Word) vs packets of an xHCI host.
        for packet, capture in HW_PACKETS:
            with self.subTest(capture=capture):
                self.assertEqual(packet, words(capture))

    def test_decode_packets(self):
        rx  = link_command(LGOOD(3)) + dph(seq=2, length=8) + data_packet_payload(range(18))
        rx += tp(TP_SUB_ACK, seq=5, nump=0, tp_seq=1)
        packets = decode_packets(rx)
        self.assertEqual(packets[0], ("LC", LGOOD(3), True))
        self.assertEqual(packets[1], ("HP", 0x08, 8 << 16, 0, 2, True))
        self.assertEqual(packets[2], ("DPP", list(range(18)), True))
        self.assertEqual(packets[3], ("HP", 0x04, TP_SUB_ACK | (1 << 21), 0, 5, True))

    def test_model_send_wait(self):
        # Script: send a link command, wait a received word, request a SKP, end.
        s = HostScript()
        s.send(link_command(LUP()))
        s.wait(0xfbfbfbf7, ctrl=0b1111)
        s.skp()
        s.end()
        dut = USB3HostModel(s.steps, wait_timeout=64)

        def generator(dut):
            yield dut.enable.eq(1)
            dut.tx, dut.skp_cycles = [], 0
            yield dut.source.ready.eq(1)
            for i in range(200):
                if i == 20:
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.data.eq(0xfbfbfbf7)
                    yield dut.sink.ctrl.eq(0b1111)
                elif i == 21:
                    yield dut.sink.valid.eq(0)
                yield
                if (yield dut.source.valid):
                    dut.tx.append(((yield dut.source.data), (yield dut.source.ctrl)))
                dut.skp_cycles += (yield dut.skp)
            dut.result = ((yield dut.done), (yield dut.error))

        run_simulation(dut, generator(dut))
        self.assertEqual(dut.tx, link_command(LUP()))
        self.assertEqual(dut.skp_cycles, 1)
        self.assertEqual(dut.result, (1, 0))

    def test_model_wait_timeout(self):
        s = HostScript()
        s.wait(0xfbfbfbf7, ctrl=0b1111)
        s.end()
        dut = USB3HostModel(s.steps, wait_timeout=16)

        def generator(dut):
            yield dut.enable.eq(1)
            for i in range(64):
                yield
            dut.result = ((yield dut.done), (yield dut.error))

        run_simulation(dut, generator(dut))
        self.assertEqual(dut.result, (0, 1))

if __name__ == "__main__":
    unittest.main()
