#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Eyes: renderer (Migen simulation) against the Python reference (text grid written
through the Wishbone slave, waveform lanes from a single capture, random back-pressure), UVC
composite descriptors consistency with the phase 1 functions, and a frame streamed end to end by
the AgentLab USB3 composite device (reactive host)."""

import random
import unittest

import pytest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.descriptors import *
from usb3_core.uvc         import *

from agentlab.gateware.eyes       import *
from agentlab.gateware.eyes_font  import *
from agentlab.gateware.usb        import *
from agentlab.gateware.instrument import AgentLab

from test.model.host import *
from test.test_agentlab_usb import parse_descriptors

# Small screen (simulation): 64x48 (8x3 characters), 16 samples of 8 bits.
WIDTH, HEIGHT = 64, 48
WAVE_DEPTH    = 16
WAVE_WIDTH    = 8

def random_cells(cols, rows, seed=0):
    prng = random.Random(seed)
    return [eyes_cell(prng.choice([0x20, 0x41, 0x67, 0x91, 0x82, prng.randrange(256)]),
        fg=prng.randrange(16), bg=prng.randrange(16)) for _ in range(cols*rows)]

# Reference ----------------------------------------------------------------------------------------

class TestEyesReference(unittest.TestCase):
    def test_palette_font(self):
        self.assertEqual(len(EYES_PALETTE_YUV), 16)
        self.assertEqual(EYES_PALETTE_YUV[0],  (16, 128, 128))
        self.assertEqual(EYES_PALETTE_YUV[15], (235, 128, 128))
        font = eyes_font()
        self.assertEqual(len(font), 256*16)
        self.assertEqual(font[0:16], [0]*16)                                   # Blank.
        self.assertEqual(font[0x91*16:0x92*16], [0xff]*16)                     # Full block.
        self.assertEqual(eyes_char_code("A"), 0x41)
        self.assertEqual(eyes_char_code("█"), 0x91)
        self.assertEqual(eyes_char_code("中"), ord("?"))

    def test_render_text_wave(self):
        # Text: "A" (white on blue) at (0, 0); one lane on row 1 from column 1 (x0 = 8).
        cols  = WIDTH//8
        cells = [eyes_cell(" ")]*(cols*3)
        cells[0] = eyes_cell("A", fg=7, bg=4)
        frame = eyes_render(WIDTH, HEIGHT, cells, samples=[0, 0, 1, 1] + [0]*12, wave_row=1,
            wave_lanes=1, wave_column=1, wave_color=10)
        self.assertEqual(len(frame), WIDTH*HEIGHT*2)
        pixel = lambda x, y: frame[2*(y*WIDTH + x)] # Luma.
        ylum  = lambda c: EYES_PALETTE_YUV[c][0]
        # "A" line 2: 01111100 (blue background, white foreground).
        self.assertEqual([pixel(x, 2) for x in range(8)], [ylum(4)] + [ylum(7)]*5 + [ylum(4)]*2)
        # Lane: low line (y = 16 + 12) for samples 0-1 (x 8-11), high line (y = 16 + 3) for
        # samples 2-3 (x 12-15), rising edge at x = 12 (lines 3-12).
        self.assertEqual([pixel(x, 16 + 12) for x in range(8, 16)], [ylum(10)]*5 + [ylum(0)]*3)
        self.assertEqual([pixel(x, 16 + 3)  for x in range(8, 16)], [ylum(0)]*4 + [ylum(10)]*4)
        self.assertEqual([pixel(12, 16 + y) for y in range(3, 13)], [ylum(10)]*10)
        self.assertEqual(pixel(13, 16 + 6), ylum(0))

# Renderer -----------------------------------------------------------------------------------------

class RendererDUT(LiteXModule):
    def __init__(self):
        self.eyes = AgentLabEyes(WIDTH, HEIGHT, wave_depth=WAVE_DEPTH, wave_width=WAVE_WIDTH,
            wave_lanes=0, wave_divider=0)
        self.count = count = Signal(16)
        self.sync += count.eq(count + 1)
        self.comb += self.eyes.samples.eq(count[1:])

class TestEyesRenderer(unittest.TestCase):
    def render_test(self, ready_prob=0.7, seed=0):
        dut   = RendererDUT()
        eyes  = dut.eyes
        cols  = WIDTH//8
        rows  = HEIGHT//16
        cells = random_cells(cols, rows, seed)
        prng  = random.Random(seed)
        result = {}
        def generator():
            # Text grid through the Wishbone slave (+ read back).
            for i, cell in enumerate(cells):
                yield from eyes.bus.write(0x24000000 + i, cell | 0xdead0000) # Full SoC word address.
            for i in [0, 5, len(cells) - 1]:
                self.assertEqual((yield from eyes.bus.read(0x24000000 + i)), cells[i])
            # Waveform: 2 lanes from bit 1 on rows 1-2, column 2, single capture.
            yield eyes._wave_row.storage.eq(1)
            yield eyes._wave_lanes.storage.eq(2)
            yield eyes._wave_column.storage.eq(2)
            yield eyes._wave_first.storage.eq(1)
            yield eyes._wave_color.storage.eq(14)
            # (CSRs fields driven directly: no CSR bank in simulation.)
            yield eyes._wave_control.fields.run.eq(0)
            yield eyes._wave_control.fields.single.eq(1)
            yield
            yield eyes._wave_control.fields.single.eq(0)
            samples = []
            for _ in range(4*WAVE_DEPTH):
                if (yield eyes.sample_we):
                    samples.append((yield eyes.samples))
                yield
            self.assertEqual(len(samples), WAVE_DEPTH)
            self.assertEqual((yield eyes._wave_status.fields.running), 0)
            result["samples"] = samples
            # Two frames (the second one restarted in the middle of the first).
            for frame in range(2):
                yield eyes.frame_start.eq(1)
                yield
                yield eyes.frame_start.eq(0)
                words = []
                n     = WIDTH*HEIGHT//2 if frame else WIDTH*HEIGHT//4
                while len(words) < n:
                    ready = prng.random() < ready_prob
                    yield eyes.source.ready.eq(ready)
                    yield
                    if ready and (yield eyes.source.valid):
                        words.append((yield eyes.source.data))
                yield eyes.source.ready.eq(0)
            result["frame"] = b"".join(w.to_bytes(4, "little") for w in words)
            for _ in range(32):
                yield
            # No extra pixels after the frame.
            self.assertEqual((yield eyes.source.valid), 0)
            self.assertEqual((yield eyes._frames.status), 1)
        run_simulation(dut, generator())
        reference = eyes_render(WIDTH, HEIGHT, cells, result["samples"], wave_row=1, wave_lanes=2,
            wave_column=2, wave_first=1, wave_color=14)
        self.assertEqual(result["frame"], reference)

    def test_render(self):
        self.render_test(ready_prob=1.0)

    def test_render_backpressure(self):
        self.render_test(ready_prob=0.5, seed=1)

# USB ----------------------------------------------------------------------------------------------

class TestEyesDescriptors(unittest.TestCase):
    def test_descriptors(self):
        m           = agentlab_usb_map(with_eyes=True)
        descriptors = agentlab_descriptors(with_eyes=True)
        config      = descriptors.configuration()
        self.assertEqual(config[2] | (config[3] << 8), len(config))
        self.assertLessEqual(len(config), 1024) # 2 EP0 packets (checked in TestEyesUSB).
        descs      = parse_descriptors(config)[1:]
        iads       = [d for d in descs if d[1] == DESC_IAD]
        interfaces = [d for d in descs if d[1] == DESC_INTERFACE]
        endpoints  = [d for d in descs if d[1] == DESC_ENDPOINT]
        # Interfaces 0-12 contiguous: phase 1 (0-10) + UVC VideoControl/VideoStreaming (11-12).
        self.assertEqual([d[2] for d in interfaces], list(range(13)))
        self.assertEqual(config[4], 13)
        self.assertEqual(m["eyes"], {"interface": 11, "in_ept": 14})
        self.assertEqual([(d[5], d[6]) for d in interfaces[11:]],
            [(CC_VIDEO, SC_VIDEOCONTROL), (CC_VIDEO, SC_VIDEOSTREAMING)])
        # IADs: 5 CDC-ACM + UVC (interfaces 11-12).
        self.assertEqual((iads[-1][2], iads[-1][3], iads[-1][4]), (11, 2, CC_VIDEO))
        # VC Header bInCollection/baInterfaceNr: VideoStreaming interface 12.
        # (subtype 0x01 also used by the CDC Call Management descriptors: after interface 11.)
        uvc    = descs[descs.index(interfaces[11]):]
        vc_hdr = [d for d in uvc if d[1] == DESC_CS_INTERFACE and d[2] == VC_HEADER][0]
        self.assertEqual(vc_hdr[11:13], [1, 12])
        # VS Input Header bEndpointAddress: EP14 IN.
        vs_hdr = [d for d in uvc if d[1] == DESC_CS_INTERFACE and d[2] == VS_INPUT_HEADER][1]
        self.assertEqual(vs_hdr[6], 0x80 | 14)
        # Endpoints: unique, <= 15 IN/OUT, interfaces endpoints count.
        addresses = [d[2] for d in endpoints]
        self.assertEqual(len(addresses), len(set(addresses)))
        self.assertIn(0x80 | 14, addresses)
        self.assertLessEqual(len([a for a in addresses if a & 0x80]), 15)
        self.assertLessEqual(len([a for a in addresses if not a & 0x80]), 15)
        self.assertEqual(sum(d[4] for d in interfaces), len(endpoints))
        # Phase 1 functions unchanged.
        self.assertEqual({k: v for k, v in m.items() if k != "eyes"}, agentlab_usb_map())
        self.assertEqual(config[:len(agentlab_descriptors().configuration())][9:],
            agentlab_descriptors().configuration()[9:])
        # Default modes: bulk payloads ended by short packets.
        check_modes(EYES_MODES, UVC_PAYLOAD_SIZE)

    def test_core_endpoints(self):
        dut = AgentLabUSB(PHYStandIn(), sys_clk_freq=125e6, with_eyes=True)
        descs = parse_descriptors(agentlab_descriptors(with_eyes=True).configuration())
        addresses = sorted(d[2] for d in descs if d[1] == DESC_ENDPOINT)
        core      = sorted(ep.ept | (ep.direction << 7) for ep in dut.core.endpoints)
        self.assertEqual(addresses, core)

# Small UVC mode (simulation): 32x16 (4x1 characters), 1024 bytes payloads.
USB_WIDTH, USB_HEIGHT = 32, 16
USB_MODES    = [(USB_WIDTH, USB_HEIGHT, 100_000)]
PAYLOAD_SIZE = 1024

class USBDUT(LiteXModule):
    def __init__(self):
        self.phy      = phy = PHYStandIn()
        self.usb      = usb = AgentLabUSB(phy, sys_clk_freq=125e6, with_eyes=True,
            eyes_modes=USB_MODES, eyes_payload_size=PAYLOAD_SIZE)
        self.agentlab = agentlab = AgentLab(sys_clk_freq=125e6, uart_pads=[None]*4, n_gpios=2,
            n_power=1, with_eyes=True, eyes_kwargs=dict(width=USB_WIDTH, height=USB_HEIGHT,
            wave_depth=WAVE_DEPTH, wave_lanes=0))
        self.comb += agentlab_usb_connect(usb, agentlab)

def pyhost_control_in_packets(host, setup, n):
    """Control read with a Data stage of n packets (512 bytes max each): returns the data."""
    n0 = len(host.hps)
    d0 = len(host.dpps)
    host.send_hp(dph, length=8, setup=1)
    host.send(data_packet_payload(setup))
    yield from pyhost_wait_tp(host, n0)
    data = []
    for i in range(n):
        host.send_hp(tp, TP_SUB_ACK, nump=n - i, tp_seq=i, pp=1)
        yield from host.wait(lambda: len(host.dpps) > d0 + i)
        data += host.dpps[d0 + i][0]
    host.send_hp(tp, TP_SUB_ACK, nump=0, tp_seq=n)
    n2 = len(host.hps)
    host.send_hp(tp, TP_SUB_STATUS)
    yield from pyhost_wait_tp(host, n2)
    return data

class TestEyesUSB(unittest.TestCase):
    def usb_test(self, with_frame):
        dut  = USBDUT()
        host = PyHost(dut.phy)
        vs   = 12
        def scenario():
            yield from pyhost_link_init(host)
            # Configuration descriptor: 2 EP0 packets.
            config = agentlab_descriptors(with_eyes=True, eyes_modes=USB_MODES).configuration()
            self.assertGreater(len(config), 512)
            data = yield from pyhost_control_in_packets(host, setup_packet(0x80, 0x06, 0x0200, 0,
                len(config)), 2)
            self.assertEqual(data, config)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            # PROBE/COMMIT on the VideoStreaming interface (12).
            probe = uvc_probe(USB_MODES, 0, PAYLOAD_SIZE, 125_000_000)
            data  = yield from pyhost_control_in(host, setup_packet(0xa1, UVC_GET_CUR,
                VS_PROBE_CONTROL << 8, vs, 34))
            self.assertEqual(data, probe)
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x21, UVC_SET_CUR,
                VS_COMMIT_CONTROL << 8, vs, 34), probe)))
            # CDC-ACM requests still routed to their function.
            data = yield from pyhost_control_in(host, setup_packet(0xa1, 0x21, 0, 2, 7))
            self.assertEqual(data, list((115200).to_bytes(4, "little")) + [0, 0, 8])
            if not with_frame:
                return
            # One frame (boot screen).
            data_size = PAYLOAD_SIZE - UVC_HEADER_SIZE
            n = -(-frame_size(USB_WIDTH, USB_HEIGHT)//data_size)
            payloads, _ = yield from pyhost_bulk_in(host, ept=14, count=n, nump=4)
            pixels = []
            for i, p in enumerate(payloads):
                self.assertEqual(p[0], UVC_HEADER_SIZE)
                self.assertEqual((p[1] >> 1) & 1, int(i == n - 1)) # EOF.
                pixels += p[UVC_HEADER_SIZE:]
            cols, rows = USB_WIDTH//8, USB_HEIGHT//16
            self.assertEqual(bytes(pixels), eyes_render(USB_WIDTH, USB_HEIGHT,
                eyes_boot_screen(cols, rows)))
        run_pyhost(dut, host, scenario())

    def test_requests(self):
        # Configuration descriptor (2 EP0 packets), PROBE/COMMIT, CDC-ACM requests.
        self.usb_test(with_frame=False)

    @pytest.mark.slow
    def test_frame(self):
        self.usb_test(with_frame=True)

if __name__ == "__main__":
    unittest.main()
