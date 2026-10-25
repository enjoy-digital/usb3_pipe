#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 UVC camera: descriptors, PROBE/COMMIT, payloads (Migen simulation with small modes, reactive
host)."""

import unittest

import pytest

from migen import *

from litex.gen import *

from usb3_core.host_model import *
from usb3_core.uvc        import *

from test.model.host import *

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

    def probe_commit_stream_test(self, with_requests=True, n_frames=2):
        dut  = DUT()
        host = PyHost(dut.phy)
        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            if with_requests:
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
            if not n_frames:
                return
            # SET_CUR PROBE (mode 1), GET_CUR PROBE, SET_CUR COMMIT.
            probe2 = uvc_probe(MODES, 1, PAYLOAD_SIZE, CLK_FREQ)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_PROBE_CONTROL << 8, 1, 34), probe2)))
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, probe2)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_COMMIT_CONTROL << 8, 1, 34), probe2)))

            # Frames of 64x8 (1024 bytes): 5 payloads of 244 bytes of data per frame.
            width, height, _ = MODES[1]
            data_size = PAYLOAD_SIZE - UVC_HEADER_SIZE
            n = -(-frame_size(width, height)//data_size)
            payloads, seq = yield from pyhost_bulk_in(host, ept=1, count=n_frames*n)
            fids = []
            for frame in range(n_frames):
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
            self.assertEqual(fids, sum([[fids[0] ^ (frame % 2)]*n for frame in range(n_frames)], []))
        run_pyhost(dut, host, scenario())

    def test_probe_requests(self):
        # Short: PROBE requests only (no streaming).
        self.probe_commit_stream_test(with_requests=True, n_frames=0)

    def test_probe_commit_stream(self):
        # Short: SET_CUR PROBE/COMMIT, one frame.
        self.probe_commit_stream_test(with_requests=False, n_frames=1)

    @pytest.mark.slow
    def test_probe_commit_stream_frames(self):
        self.probe_commit_stream_test(with_requests=True, n_frames=2)

# Isochronous --------------------------------------------------------------------------------------

class ISODUT(LiteXModule):
    def __init__(self, max_burst=1):
        self.phy = phy = PHYStandIn()
        self.uvc = USB3UVC(phy, CLK_FREQ, modes=MODES, max_burst=max_burst, transfer="iso")

class TestCoreUVCIso(unittest.TestCase):
    def test_iso_descriptors(self):
        d      = uvc_descriptors(UVC_ISO_MODES, max_burst=16, transfer="iso")
        config = d.configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertEqual(config[4], 2) # VideoControl + VideoStreaming (2 alternate settings).
        descs = parse_descriptors(config)
        interfaces = [x for x in descs if x[1] == DESC_INTERFACE]
        self.assertEqual([(x[2], x[3], x[4], x[5], x[6]) for x in interfaces],
            [(0, 0, 0, CC_VIDEO, SC_VIDEOCONTROL), (1, 0, 0, CC_VIDEO, SC_VIDEOSTREAMING),
             (1, 1, 1, CC_VIDEO, SC_VIDEOSTREAMING)])
        # Class specific VS descriptors on alternate setting 0, VS Input Header bEndpointAddress.
        vs_hdr = [x for x in descs if x[1] == DESC_CS_INTERFACE and x[2] == VS_INPUT_HEADER][1]
        self.assertEqual(vs_hdr[6], 0x81)
        self.assertLess(descs.index(vs_hdr), descs.index(interfaces[2]))
        # Isochronous IN endpoint (Asynchronous, 1024 bytes, bInterval 1), companion: bMaxBurst 15,
        # Mult 0, wBytesPerInterval 16384 (= dwMaxPayloadTransferSize).
        ep   = [x for x in descs if x[1] == DESC_ENDPOINT][0]
        comp = [x for x in descs if x[1] == DESC_SS_EP_COMP][0]
        self.assertEqual(ep[2:7], [0x81, 0x05, 0x00, 0x04, 1])
        self.assertEqual(comp[2:6], [15, 0, 0x00, 0x40])
        probe = uvc_probe(UVC_ISO_MODES, 0, 16*1024, 125_000_000)
        self.assertEqual(int.from_bytes(bytes(probe[22:26]), "little"), 16*1024)
        # Isochronous modes: no bulk short packet constraint.
        check_modes(UVC_ISO_MODES, 16*1024, bulk=False)
        # uvcvideo (uvc_video_start_transfer): alternate setting with wBytesPerInterval >=
        # dwMaxPayloadTransferSize (-EIO otherwise), frame period from dwFrameInterval (S_PARM).
        bpis = {1: comp[4] | (comp[5] << 8)} # VS alternate setting 0: no endpoint.
        for m, (width, height, fps) in enumerate(UVC_ISO_MODES):
            probe = uvc_probe(UVC_ISO_MODES, m, 16*1024, 125_000_000)
            self.assertEqual(len(probe), UVC_PROBE_SIZE)
            le32 = lambda o: int.from_bytes(bytes(probe[o:o+4]), "little")
            self.assertEqual(le32(4), 10_000_000//fps)                      # dwFrameInterval.
            self.assertEqual(le32(18), frame_size(width, height))           # dwMaxVideoFrameSize.
            self.assertEqual(le32(26), 125_000_000)                         # dwClockFrequency.
            alts = [alt for alt, bpi in bpis.items() if bpi >= le32(22)]   # dwMaxPayloadTransferSize.
            self.assertEqual(alts, [1])

    def test_memories_initialized(self):
        # Initialized memories with write ports (EP0 descriptors ROM + request handler buffer) must be
        # initialized over their whole depth: on hardware, the handler buffer read back the
        # isochronous probes with bit 31 set (never written to 1, undefined init resolved to 1).
        for transfer in ["bulk", "iso"]:
            dut = ISODUT() if transfer == "iso" else DUT()
            mems = [s for s in dut.get_fragment().specials if isinstance(s, Memory)]
            for mem in mems:
                if mem.init and any(port.we is not None for port in mem.ports):
                    self.assertEqual(len(mem.init), mem.depth, f"{transfer}: {mem.name_override}")

    def iso_stream_test(self, n_frames):
        # 1 packet per service interval (1024 bytes payloads): 64x8 frames (1024 bytes) in 2 payloads
        # (1012 + 12 bytes of data), zero-length packets between frames.
        dut  = ISODUT(max_burst=1)
        host = PyHost(dut.phy)
        payload_size = 1024
        def scenario():
            yield from pyhost_link_init(host)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # GET_CUR PROBE: dwMaxPayloadTransferSize = wBytesPerInterval.
            probe = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR, VS_PROBE_CONTROL << 8, 1, 34))
            self.assertEqual(probe, uvc_probe(MODES, 0, payload_size, CLK_FREQ))
            # SET_CUR PROBE/COMMIT (mode 1): no streaming on alternate setting 0.
            probe2 = uvc_probe(MODES, 1, payload_size, CLK_FREQ)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_PROBE_CONTROL << 8, 1, 34), probe2)))
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR, VS_COMMIT_CONTROL << 8, 1, 34), probe2)))
            for _ in range(512):
                yield
            self.assertEqual((yield dut.uvc.function.stream.frames), 0)
            # SET_INTERFACE(1, 1): stream start.
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0x81, 0x0a, 0, 1, 1))), [0])
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, 1, 1, 0))))
            self.assertEqual((yield from pyhost_control_in(host, setup_packet(0x81, 0x0a, 0, 1, 1))), [1])
            # Service intervals until n_frames frames received.
            payloads = []
            zlps     = 0
            for _ in range(64):
                dps = yield from pyhost_iso_in(host, ept=1, bursts=[1], ping=False)
                self.assertEqual(len(dps), 1)
                d, p, crc = dps[0]
                self.assertTrue(crc)
                self.assertEqual((d["seq"], d["lpf"]), (0, 1))
                if len(p) == 0:
                    zlps += 1
                    continue
                self.assertLessEqual(len(p), payload_size)
                payloads.append(p)
                if sum((p[1] >> 1) & 1 for p in payloads) == n_frames:
                    break
                for _ in range(16):
                    yield
            self.assertGreater(zlps, 0)
            # Frames reassembly (FID/EOF).
            width, height, _ = MODES[1]
            frames = []
            pixels = []
            fids   = []
            for p in payloads:
                self.assertEqual(p[0], UVC_HEADER_SIZE)
                self.assertEqual(p[1] & 0x8c, 0x8c) # EOH, SCR, PTS.
                fids.append(p[1] & 1)
                pixels += p[UVC_HEADER_SIZE:]
                if p[1] & 0x2:
                    frames.append((fids[-1], pixels))
                    pixels = []
            self.assertEqual(len(frames), n_frames)
            self.assertEqual([len(p) for p in payloads], [payload_size, UVC_HEADER_SIZE + 12]*n_frames)
            for frame, (fid, pixels) in enumerate(frames):
                self.assertEqual(len(pixels), frame_size(width, height))
                self.assertEqual(fid, (frames[0][0] + frame) % 2)
                # First line: color bars, marker at x = 4*frame.
                for x in range(0, width, 2):
                    y_ref = [235, 210, 170, 145, 106, 81, 41, 16][x//8]
                    y0 = 235 if x == 4*frame else y_ref
                    y1 = 235 if x + 1 == 4*frame else y_ref
                    self.assertEqual((pixels[2*x], pixels[2*x + 2]), (y0, y1), f"frame {frame} x {x}")
            # SET_INTERFACE(1, 0): stream stop, zero-length packets.
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x01, 0x0b, 0, 1, 0))))
            frames_stop = (yield dut.uvc.function.stream.frames)
            for _ in range(2):
                dps = yield from pyhost_iso_in(host, ept=1, bursts=[1], ping=False)
                self.assertEqual(dps[0][0]["length"], 0)
                for _ in range(256):
                    yield
            self.assertEqual((yield dut.uvc.function.stream.frames), frames_stop)
        run_pyhost(dut, host, scenario())

    def test_iso_stream(self):
        # Short: one frame.
        self.iso_stream_test(n_frames=1)

    @pytest.mark.slow
    def test_iso_stream_frames(self):
        self.iso_stream_test(n_frames=2)

if __name__ == "__main__":
    unittest.main()
