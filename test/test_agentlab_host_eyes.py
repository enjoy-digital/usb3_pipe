#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab Eyes host side tests (fake backends, no hardware): YUY2/PNG conversions against the
gateware reference renderer, text display flush, status panel, frame grabbers (fake commands),
video node discovery, Instrument/CLI/MCP screenshot and display."""

import io
import os
import sys
import json
import tempfile
import unittest
import contextlib
import subprocess

from agentlab.host import constants as c
from agentlab.host.eyes       import *
from agentlab.host.discovery  import DeviceInfo, find_device
from agentlab.host.events     import Event
from agentlab.host.mcp_server import AgentLabTools

from agentlab.gateware.eyes import EYES_PALETTE, EYES_PALETTE_YUV, eyes_render
from agentlab.gateware.eyes import eyes_cell as gateware_eyes_cell

from test.agentlab_host_fake import *
from test.test_agentlab_host     import make_sysfs
from test.test_agentlab_host_cli import run_cli

# Conversions --------------------------------------------------------------------------------------

def yuy2_palette_frame():
    """2 lines of 16 pixels: palette colors (pairs of identical pixels: no U/V averaging)."""
    data = b""
    for line in range(2):
        for color in range(16):
            y, u, v = EYES_PALETTE_YUV[color]
            data += bytes([y, u, y, v])
    return data

class TestConversions(unittest.TestCase):
    def check_palette(self, rgb):
        for color in range(16):
            for line in range(2):
                for p in range(2):
                    i = 3*(line*32 + 2*color + p)
                    for a, b in zip(rgb[i:i + 3], EYES_PALETTE[color]):
                        self.assertLessEqual(abs(a - b), 3, (color, rgb[i:i + 3]))

    def test_yuy2_to_rgb(self):
        self.check_palette(yuy2_to_rgb(yuy2_palette_frame(), 32, 2))

    def test_yuy2_to_rgb_pure_python(self):
        numpy = sys.modules.get("numpy", None)
        sys.modules["numpy"] = None # ImportError on import.
        try:
            self.check_palette(yuy2_to_rgb(yuy2_palette_frame(), 32, 2))
        finally:
            if numpy is None:
                del sys.modules["numpy"]
            else:
                sys.modules["numpy"] = numpy
        with self.assertRaises(ValueError):
            yuy2_to_rgb(b"\x00"*10, 32, 2)

    def test_png_roundtrip(self):
        rgb   = bytes(range(256))*3 + bytes(48)
        frame = Frame(16, 17, rgb[:16*17*3])
        data  = frame.png()
        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(png_decode(data), (16, 17, frame.rgb))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "f.png")
            self.assertEqual(frame.save_png(path), len(data))

    def test_reference_frame(self):
        # Gateware reference renderer -> YUY2 -> RGB: "A" white on blue at (0, 0).
        cells = [gateware_eyes_cell(" ")]*(8*2)
        cells[0] = gateware_eyes_cell("A", fg=15, bg=4)
        frame = Frame(64, 32, yuy2_to_rgb(eyes_render(64, 32, cells), 64, 32))
        close = lambda a, b: all(abs(x - y) <= 3 for x, y in zip(a, b))
        self.assertTrue(close(frame.pixel(0, 0), EYES_PALETTE[4]))   # Background.
        self.assertTrue(close(frame.pixel(2, 2), EYES_PALETTE[15]))  # "A" top bar (01111100).
        self.assertTrue(close(frame.pixel(20, 20), EYES_PALETTE[0])) # Black.
        # Host cell encoding matches the gateware one.
        self.assertEqual(eyes_cell("A", "bright_white", "blue"), cells[0])
        self.assertEqual(eyes_cell("─", 7, 0), gateware_eyes_cell("─", 7, 0))

# Display ------------------------------------------------------------------------------------------

class TestDisplay(unittest.TestCase):
    def test_flush_diff(self):
        writes = []
        d = EyesDisplay(lambda addr, datas: writes.append((addr, datas)), 0x1000, cols=10, rows=3)
        self.assertEqual(d.flush(), 30) # First flush: everything.
        self.assertEqual([a for a, _ in writes], [0x1000, 0x1000 + 40, 0x1000 + 80])
        writes.clear()
        self.assertEqual(d.flush(), 0)
        d.text(2, 1, "Hi", fg="red")
        d.put(8, 1, "!")
        self.assertEqual(d.flush(), 7) # Columns 2-8 of row 1.
        self.assertEqual(writes, [(0x1000 + 4*12, d.cells[12:19])])
        self.assertEqual(d.cells[12], eyes_cell("H", "red"))
        self.assertTrue(d.get_text(1).startswith("  Hi"))
        # Clipping, multi-line text, width padding.
        d.text(8, 2, "abcd\nef", width=3)
        self.assertEqual(d.get_text(2)[8:], "ab")
        with self.assertRaises(ValueError):
            d.put(0, 0, "x", fg="purple")

    def test_box_and_status(self):
        d = EyesDisplay(lambda a, x: None, 0, cols=160, rows=45)
        state = {"time_s": 12.5, "power": [True, False], "gpio_in": [True, False],
            "gpio_oe": [True, False], "uarts": [["login: root", "# ls"], [], ["x"*200], []],
            "events": ["marker"], "lanes": ["gpio0", "gpio1"], "wave": "(test)"}
        render_status(d, state)
        self.assertIn("AgentLab Eyes", d.get_text(0))
        self.assertIn("t = 12.500000 s", d.get_text(0))
        self.assertIn("0:ON", d.get_text(1))
        self.assertIn("1:off", d.get_text(1))
        self.assertIn("0>1", d.get_text(1))
        self.assertIn("UART0", d.get_text(STATUS_UART_ROW))
        self.assertIn("# ls", d.get_text(STATUS_UART_ROW + 2))
        # UART2 (bottom left box): line clipped to the box.
        self.assertEqual(d.get_text(STATUS_UART_ROW + 11)[:80], "\x81" + "x"*78 + "\x81")
        self.assertIn("gpio1", d.get_text(STATUS_WAVE_ROW + 2))
        # Box corners (font codes 0x82-0x85).
        self.assertEqual(d.cells[STATUS_UART_ROW*160] & 0xff, 0x82)

    def test_uart_tails_lane_names(self):
        events = uart_events("a\r\nb\nc", source=1) + uart_events("zz", source=0)
        tails  = uart_tails(events)
        self.assertEqual(tails, [["zz"], ["a", "b", "c"], [], []])
        self.assertEqual(lane_names(2, 1, 1), ["gpio0", "gpio1", "power0", "uart0_tx", "uart0_rx"])

# Grabbers -----------------------------------------------------------------------------------------

class FakeRun:
    """subprocess.run stand-in: ffmpeg (YUY2 on stdout) or v4l2-ctl (YUY2 to --stream-to file)."""
    def __init__(self, data, returncode=0):
        self.data       = data
        self.returncode = returncode
        self.cmds       = []

    def __call__(self, cmd, stdout=None, stderr=None, timeout=None):
        self.cmds.append(cmd)
        out = b""
        if cmd[0] == "ffmpeg":
            out = self.data
        elif cmd[0] == "v4l2-ctl":
            path = [a for a in cmd if a.startswith("--stream-to=")][0].split("=", 1)[1]
            with open(path, "wb") as f:
                f.write(self.data)
        return subprocess.CompletedProcess(cmd, self.returncode, out, b"error")

class TestGrab(unittest.TestCase):
    def test_backends(self):
        data = yuy2_palette_frame()
        for backend in ["ffmpeg", "v4l2-ctl"]:
            run   = FakeRun(data)
            frame = grab_frame("/dev/video7", 32, 2, backend=backend, run=run)
            self.assertEqual((frame.width, frame.height), (32, 2))
            self.assertEqual(frame.rgb, yuy2_to_rgb(data, 32, 2))
            self.assertIn("/dev/video7", run.cmds[0])
        # Errors: short frame, command failure, unknown backend.
        with self.assertRaises(RuntimeError):
            grab_frame("/dev/video7", 32, 2, backend="ffmpeg", run=FakeRun(data[:10]))
        with self.assertRaises(RuntimeError):
            grab_frame("/dev/video7", 32, 2, backend="v4l2-ctl", run=FakeRun(data, returncode=1))
        with self.assertRaises(ValueError):
            grab_frame("/dev/video7", 32, 2, backend="vlc")

    def test_auto(self):
        data = yuy2_palette_frame()
        cv2  = sys.modules.get("cv2", None)
        sys.modules["cv2"] = None # No OpenCV: ffmpeg/v4l2-ctl from PATH.
        try:
            run   = FakeRun(data)
            which = lambda name: "/usr/bin/" + name if name == "v4l2-ctl" else None
            grab_frame("/dev/video7", 32, 2, run=run, which=which)
            self.assertEqual(run.cmds[0][0], "v4l2-ctl")
            with self.assertRaises(RuntimeError):
                grab_frame("/dev/video7", 32, 2, run=run, which=lambda name: None)
        finally:
            if cv2 is None:
                del sys.modules["cv2"]
            else:
                sys.modules["cv2"] = cv2

# Discovery ----------------------------------------------------------------------------------------

class TestVideoDiscovery(unittest.TestCase):
    def test_video_node(self):
        with tempfile.TemporaryDirectory() as root:
            classes = {i: "02" if i < 10 and i % 2 == 0 else "0a" for i in range(10)}
            classes.update({10: "ff", 11: "0e", 12: "0e"})
            make_sysfs(root, classes=classes, ttys={0: "ttyACM0"})
            for node, index in [("video5", "1"), ("video4", "0")]: # Metadata node: index 1.
                path = os.path.join(root, "2-1:1.11", "video4linux", node)
                os.makedirs(path)
                with open(os.path.join(path, "index"), "w") as f:
                    f.write(index + "\n")
            d = find_device(sysfs_root=root)
            self.assertEqual(d.video_device, "/dev/video4")
            self.assertEqual(d.describe()["video"], "/dev/video4")
            self.assertIsNone(DeviceInfo("1-1", 1, 1).video_device)

# Instrument / CLI / MCP ---------------------------------------------------------------------------

def make_eyes_instrument(**kwargs):
    i, f = make_instrument(with_eyes=True, **kwargs)
    i.device = DeviceInfo("2-1", 2, 3, serial="AL0001", videos={11: "/dev/video4"})
    return i, f

class TestInstrumentEyes(unittest.TestCase):
    def test_display_text(self):
        i, f = make_eyes_instrument()
        self.assertEqual(i.eyes_info()["columns"], 160)
        i.display_text("Hello", col=3, row=2, fg="bright_green")
        cells = f["camera"].cells()
        self.assertEqual(cells[2*160 + 3], eyes_cell("H", "bright_green"))
        # Burst writes on the text grid region.
        self.assertTrue(any(name == "0x{:08x}".format(EYES_ORIGIN) for _, name, _ in f["bus"].log))
        i.display_clear()
        self.assertEqual(f["camera"].cells()[2*160 + 3], eyes_cell(" "))

    def test_no_eyes(self):
        i, f = make_instrument()
        with self.assertRaises(RuntimeError):
            i.display_text("x")

    def test_waves(self):
        i, f = make_eyes_instrument()
        csrs = i.csrs
        i.display_waves(lanes=4, first=2, period_s=1e-6, run=False, single=True)
        self.assertEqual(csrs.read(c.CSR_EYES_WAVE_LANES), 4)
        self.assertEqual(csrs.read(c.CSR_EYES_WAVE_FIRST), 2)
        self.assertEqual(csrs.read(c.CSR_EYES_WAVE_DIVIDER), 124)
        self.assertEqual(csrs.read(c.CSR_EYES_WAVE_CONTROL), 0b10)
        i.display_waves(color="red")
        self.assertEqual(csrs.read(c.CSR_EYES_WAVE_COLOR), 1)

    def test_status_screenshot(self):
        t0 = int(1.0*SYS_CLK_FREQ)
        i, f = make_eyes_instrument(events=uart_events("boot ok\n", source=2, t0=t0) +
            [Event(c.EVENT_MARKER, 0, 7, 2*t0)])
        try:
            i.events.wait_for(lambda e: e.type == c.EVENT_MARKER, timeout=2.0)
            i.power_set(0, True)
            i.csrs.write(c.CSR_EYES_WAVE_LANES, 3)
            f["clock"].now = 3.0
            state = i.display_status()
            self.assertEqual(state["power"], [True])
            self.assertEqual(state["uarts"][2], ["boot ok"])
            self.assertEqual(state["lanes"], ["gpio0", "gpio1", "power0"])
            self.assertIn("marker", state["events"][-1])
            self.assertIn("boot ok", i.display.get_text(STATUS_UART_ROW + 10 + 1)[:80])
            with tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "shot.png")
                r = i.screenshot(path)
                self.assertEqual((r["width"], r["height"], r["device"]), (1280, 720, "/dev/video4"))
                self.assertEqual(r["bytes"], os.path.getsize(path))
                width, height, rgb = png_decode(open(path, "rb").read())
                frame = Frame(width, height, rgb)
                # Title bar (blue background) at the top left.
                blue = EYES_PALETTE[c.EYES_COLORS.index("blue")]
                self.assertTrue(all(abs(a - b) <= 3 for a, b in zip(frame.pixel(0, 0), blue)))
        finally:
            i.close()

    def test_refresh_thread(self):
        i, f = make_eyes_instrument()
        i.start_display_refresh(period=0.01)
        try:
            for _ in range(200):
                if f["camera"].cells()[0] == eyes_cell(" ", "bright_white", "blue"):
                    break
                f["clock"].sleep(0)
                import time; time.sleep(0.01)
            self.assertEqual(f["camera"].cells()[0], eyes_cell(" ", "bright_white", "blue"))
        finally:
            i.stop_display_refresh()

class TestCLIEyes(unittest.TestCase):
    def test_display_screenshot(self):
        i, f = make_eyes_instrument()
        r, out = run_cli(["display", "text", "Hi\\nthere", "--row", "1", "--fg", "10"], i)
        self.assertEqual(r, 0)
        cells = f["camera"].cells()
        self.assertEqual(cells[160], eyes_cell("H", 10))
        self.assertEqual(cells[2*160], eyes_cell("t", 10))
        r, out = run_cli(["display", "waves", "--lanes", "2", "--period-us", "8", "--no-run"], i)
        self.assertEqual((r, i.csrs.read(c.CSR_EYES_WAVE_DIVIDER)), (0, 999))
        self.assertEqual(i.csrs.read(c.CSR_EYES_WAVE_CONTROL), 0)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "out.png")
            r, out = run_cli(["screenshot", path, "--refresh"], i)
            self.assertEqual(r, 0)
            self.assertIn("1280x720", out)
            self.assertTrue(os.path.exists(path))
        r, out = run_cli(["display", "status"], i)
        self.assertEqual(r, 0)

class TestMCPEyes(unittest.TestCase):
    def test_tools(self):
        i, f  = make_eyes_instrument()
        tools = AgentLabTools(lambda: i)
        try:
            r = tools.display_text("Agent was here", row=44, clear=True)
            self.assertEqual(r["chars"], 14)
            self.assertEqual(f["camera"].cells()[44*160], eyes_cell("A"))
            with tempfile.TemporaryDirectory() as tmp:
                r = tools.screenshot(path=os.path.join(tmp, "s.png"), refresh=False)
                json.dumps(r)
                self.assertEqual((r["width"], r["height"]), (1280, 720))
                self.assertGreater(r["bytes"], 0)
            # Default path (temporary file).
            r = tools.screenshot(refresh=False)
            self.assertTrue(os.path.exists(r["path"]))
            os.remove(r["path"])
        finally:
            tools.close()

if __name__ == "__main__":
    unittest.main()
