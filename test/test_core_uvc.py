#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 UVC camera: descriptors, PROBE/COMMIT, payloads (Migen simulation with small modes, reactive
host)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model import *
from usb3_core.uvc        import *

# Small modes (simulation): 32x8 and 64x8 (2 frame rates), 256 bytes payloads.
MODES        = [(32, 8, 100_000), (64, 8, 100_000), (64, 8, 50_000)]
PAYLOAD_SIZE = 256
CLK_FREQ     = 125_000_000

class DUT(LiteXModule):
    def __init__(self):
        self.phy = phy = PHYStandIn()
        self.uvc = USB3UVC(phy, CLK_FREQ, modes=MODES, payload_size=PAYLOAD_SIZE, max_burst=4)

# Descriptors parser -------------------------------------------------------------------------------

def parse_descriptors(data):
    descs = []
    while data:
        length = data[0]
        descs.append(data[:length])
        data = data[length:]
    return descs

class TestCoreUVC(unittest.TestCase):
    def test_descriptors(self):
        d      = uvc_descriptors(UVC_MODES)
        config = d.configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        descs  = parse_descriptors(config)
        self.assertEqual(sum(len(x) for x in descs), len(config))
        # VC Header wTotalLength: Header + Input/Output Terminals.
        vc_hdr = [x for x in descs if x[1] == DESC_CS_INTERFACE and x[2] == VC_HEADER][0]
        self.assertEqual(vc_hdr[5] | (vc_hdr[6] << 8), 13 + 18 + 9)
        # VS Input Header wTotalLength: Input Header + Format + Frames + Color Matching.
        # (VC_HEADER/VS_INPUT_HEADER: same subtype, VC first.)
        vs_hdr = [x for x in descs if x[1] == DESC_CS_INTERFACE and x[2] == VS_INPUT_HEADER][1]
        self.assertEqual(vs_hdr[4] | (vs_hdr[5] << 8), 14 + 27 + 30 + 30 + 34 + 6)
        # One frame descriptor per resolution, with its frame intervals.
        frames = [x for x in descs if x[1] == DESC_CS_INTERFACE and x[2] == VS_FRAME_UNCOMPRESSED]
        self.assertEqual([(f[5] | f[6] << 8, f[7] | f[8] << 8, f[25]) for f in frames],
            [(640, 480, 1), (1280, 720, 1), (1920, 1080, 2)])
        # Device class: IAD.
        self.assertEqual(d.device()[4:7], [0xef, 0x02, 0x01])

    def test_modes_payloads(self):
        # Default modes: last payload of each frame ended by a short packet.
        check_modes(UVC_MODES, UVC_PAYLOAD_SIZE)

    def test_probe_commit_stream(self):
        dut  = DUT()
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # GET_CUR PROBE: mode 1, GET_MAX: mode 2.
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, uvc_probe(MODES, 0, PAYLOAD_SIZE, CLK_FREQ))
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_MAX, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, uvc_probe(MODES, 2, PAYLOAD_SIZE, CLK_FREQ))
            # GET_LEN, GET_INFO.
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_LEN, VS_PROBE_CONTROL << 8, 1, 2))), [34, 0])
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_INF, VS_PROBE_CONTROL << 8, 1, 1))), [3])
            # Unsupported request (VideoControl interface): stalled.
            self.assertIsNone((yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, 0x0200, 0, 1))))
            # SET_CUR PROBE: mode selected by bFrameIndex + dwFrameInterval (64x8@50000: mode 2).
            probe3 = uvc_probe(MODES, 2, PAYLOAD_SIZE, CLK_FREQ)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_PROBE_CONTROL << 8, 1, 34), probe3)))
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, probe3)
            # SET_CUR PROBE (mode 1), GET_CUR PROBE, SET_CUR COMMIT.
            probe2 = uvc_probe(MODES, 1, PAYLOAD_SIZE, CLK_FREQ)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_PROBE_CONTROL << 8, 1, 34), probe2)))
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, probe2)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_COMMIT_CONTROL << 8, 1, 34), probe2)))

            # Two frames of 64x8 (1024 bytes): 5 payloads of 244 bytes of data per frame.
            width, height, _ = MODES[1]
            data_size = PAYLOAD_SIZE - UVC_HEADER_SIZE
            n = -(-frame_size(width, height)//data_size)
            payloads, seq = yield from pyhost_bulk_in(host, ept=1, count=2*n)
            fids = []
            for frame in range(2):
                pixels = []
                for i, p in enumerate(payloads[frame*n:(frame + 1)*n]):
                    self.assertEqual(p[0], UVC_HEADER_SIZE)
                    info = p[1]
                    self.assertEqual(info & 0x8c, 0x8c)                 # EOH, SCR, PTS.
                    self.assertEqual((info >> 1) & 1, int(i == n - 1))  # EOF on the last payload.
                    fids.append(info & 1)
                    pixels += p[UVC_HEADER_SIZE:]
                self.assertEqual(len(pixels), frame_size(width, height))
                # First line: color bars (8 pixels each), marker at x = 4*frame.
                for x in range(0, width, 2):
                    y_ref = [235, 210, 170, 145, 106, 81, 41, 16][x//8]
                    y0 = 235 if x == 4*frame else y_ref
                    y1 = 235 if x + 1 == 4*frame else y_ref
                    self.assertEqual((pixels[2*x], pixels[2*x + 2]), (y0, y1), f"frame {frame} x {x}")
            # FID toggles per frame.
            self.assertEqual(fids, [fids[0]]*n + [1 - fids[0]]*n)
        run_pyhost(dut, host, scenario())

if __name__ == "__main__":
    unittest.main()
