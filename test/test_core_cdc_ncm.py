#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 CDC-NCM function: GET_NTB_PARAMETERS, an NTB16 with three datagrams from the host (two
NDP16s chained, 4-byte aligned datagrams) and an NTB16 padded to dwNtbOutMaxSize without
zero-length packet (as Linux cdc_ncm) echoed back as one NTB16 per frame, invalid NTB dropped
(Migen simulation, reactive host)."""

import struct
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.host_model  import *
from usb3_core.common      import endpoint_layout
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import USB3Descriptors, DEVICE_CLASS_IAD
from usb3_core.cdc_ecm     import add_cdc_ecm_strings
from usb3_core.cdc_ncm     import *

from test.model.host import *

# Helpers ------------------------------------------------------------------------------------------

def align4(n):
    return (n + 3) & ~3

def ntb16(datagrams, seq=0, split=1):
    """NTB16 (NTH16, datagrams 4-byte aligned, NDP16s at the end: split datagrams per NDP16)."""
    body    = bytearray(NTH16_LENGTH)
    entries = []
    for d in datagrams:
        body += bytes(align4(len(body)) - len(body))
        entries.append((len(body), len(d)))
        body += bytes(d)
    groups = [entries[i:i+split] for i in range(0, len(entries), split)]
    ndps   = []
    for g in groups:
        body += bytes(align4(len(body)) - len(body))
        ndps.append(len(body))
        body += bytes(8 + 4*len(g) + 4)
    for i, (g, ndp) in enumerate(zip(groups, ndps)):
        nxt = ndps[i + 1] if i + 1 < len(ndps) else 0
        struct.pack_into("<IHH", body, ndp, NDP16_SIGNATURE, 8 + 4*len(g) + 4, nxt)
        for j, (index, length) in enumerate(g):
            struct.pack_into("<HH", body, ndp + 8 + 4*j, index, length)
    struct.pack_into("<IHHHH", body, 0, NTH16_SIGNATURE, NTH16_LENGTH, seq, len(body), ndps[0])
    return list(body)

def ntb16_linux(datagrams, seq=0, max_datagrams=40, pad_to=None):
    """NTB16 as Linux cdc_ncm builds it: NDP16 right after the NTH16 (room for max_datagrams entries,
    zeros after the used ones), 4-byte aligned datagrams after it, optional padding."""
    ndp      = NTH16_LENGTH
    ndp_size = 8 + 4*(max_datagrams + 1)
    body     = bytearray(ndp + ndp_size)
    entries  = []
    for d in datagrams:
        body += bytes(align4(len(body)) - len(body))
        entries.append((len(body), len(d)))
        body += bytes(d)
    struct.pack_into("<IHH", body, ndp, NDP16_SIGNATURE, 8 + 4*(len(entries) + 1), 0)
    for j, (index, length) in enumerate(entries):
        struct.pack_into("<HH", body, ndp + 8 + 4*j, index, length)
    if pad_to is not None:
        body += bytes(pad_to - len(body))
    struct.pack_into("<IHHHH", body, 0, NTH16_SIGNATURE, NTH16_LENGTH, seq, len(body), ndp)
    return list(body)

def parse_ntb16(ntb):
    """Datagrams of an NTB16."""
    b = bytes(ntb)
    sig, hlen, seq, blen, ndp = struct.unpack_from("<IHHHH", b, 0)
    assert (sig, hlen, blen) == (NTH16_SIGNATURE, NTH16_LENGTH, len(b))
    datagrams = []
    while ndp:
        sig, nlen, nxt = struct.unpack_from("<IHH", b, ndp)
        assert sig == NDP16_SIGNATURE
        i = ndp + 8
        while True:
            index, length = struct.unpack_from("<HH", b, i)
            if index == 0 or length == 0:
                break
            datagrams.append(list(b[index:index + length]))
            i += 4
        ndp = nxt
    return datagrams

# DUT (CDC-NCM with echo) --------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        descriptors = USB3Descriptors(device_class=DEVICE_CLASS_IAD)
        mac = add_cdc_ecm_strings(descriptors, 0x02eccc000001)
        descriptors.interfaces = cdc_ncm_interfaces(0, notif_ept=3, in_ept=1, out_ept=2,
            mac_string=mac, max_burst=2)
        add_cdc_ncm_requests(descriptors)
        self.core = core = USB3DeviceCore(phy, 125e6, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=2),
            USB3BulkOUT(ept=2, max_burst=8, with_zlp=True),
            USB3BulkIN(ept=3,  max_burst=2),
        ], with_ep0_handler=True)
        eps = core.endpoints
        self.ncm  = ncm  = USB3CDCNCMFunction(core.ep0, eps[0], eps[1], eps[2])
        self.echo = echo = stream.SyncFIFO(endpoint_layout(32), 1024, buffered=True)
        self.comb += [
            ncm.source.connect(echo.sink),
            echo.source.connect(ncm.sink),
        ]

# Tests --------------------------------------------------------------------------------------------

class TestCoreCDCNCM(unittest.TestCase):
    def test_ntbs(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        frames = [
            [(3*i + 1) & 0xff for i in range(1514)],
            [(5*i + 2) & 0xff for i in range(61)],  # Not a multiple of 4.
            [(7*i + 3) & 0xff for i in range(300)],
        ]
        res = {}

        def send(transfer, seq, zlp=True):
            # Bulk OUT transfer: max size packets, a zero-length packet after a multiple of 1024.
            for i in range(0, len(transfer), 1024):
                chunk = transfer[i:i+1024]
                host.send_hp(dph, length=len(chunk), ept=2, dp_seq=seq)
                host.send(data_packet_payload(chunk))
                seq = (seq + 1) % 32
            if zlp and (len(transfer) % 1024 == 0):
                host.send_hp(dph, length=0, ept=2, dp_seq=seq)
                host.send(data_packet_payload([]))
                seq = (seq + 1) % 32
            return seq

        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            res["params"] = yield from pyhost_control_in(host,
                setup_packet(0xa1, NCM_GET_NTB_PARAMETERS, 0, 0, 28))
            # SET_INTERFACE (data interface, alternate setting 1), SET_NTB_INPUT_SIZE.
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, 1, 1, 0))))
            self.assertTrue((yield from pyhost_control_out(host,
                setup_packet(0x21, NCM_SET_NTB_INPUT_SIZE, 0, 0, 4), list(le32(16384)))))
            # Notifications.
            yield from pyhost_bulk_in(host, ept=3, count=2, seq=0)
            # Invalid NTB (bad NTH16 signature), then one NTB with 3 datagrams in 2 NDP16s.
            bad = ntb16(frames[1:2])
            bad[0] ^= 0xff
            seq = send(bad, 0)
            seq = send(ntb16(frames, split=2), seq)
            # NTB as Linux cdc_ncm: NDP16 first, padded to dwNtbOutMaxSize (16 max size packets), no
            # zero-length packet.
            seq = send(ntb16_linux([frames[1]], pad_to=NCM_NTB_OUT_MAX_SIZE), seq, zlp=False)
            # Echoed frames: one NTB each (1514 + 28 bytes: 2 packets, then 1, 1 and 1).
            packets, _ = yield from pyhost_bulk_in(host, ept=1, count=5, seq=0)
            res["in"] = packets
        run_pyhost(dut, host, scenario())

        p = res["params"]
        self.assertEqual(struct.unpack("<HHIHHHHIHHHH", bytes(p)),
            (28, 1, 16384, 4, 2, 4, 0, 16384, 4, 2, 4, 0))
        p = res["in"]
        self.assertEqual([len(x) for x in p], [1024, 1514 + 28 - 1024, 61 + 28, 300 + 28, 61 + 28])
        ntbs = [p[0] + p[1], p[2], p[3], p[4]]
        self.assertEqual([parse_ntb16(n) for n in ntbs], [[f] for f in frames + [frames[1]]])
        self.assertEqual([struct.unpack_from("<H", bytes(n), 6)[0] for n in ntbs], [0, 1, 2, 3]) # wSequence.

if __name__ == "__main__":
    unittest.main()
