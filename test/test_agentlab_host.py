#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""AgentLab host library tests (fake backends, no hardware)."""

import os
import json
import time
import struct
import tempfile
import unittest

from agentlab.host import constants as c
from agentlab.host.csr       import CSRMap, CSRs, UARTBoneBus
from agentlab.host.uart      import DUTUart
from agentlab.host.events    import (Event, EventParser, EventReader, pack_event, parse_record,
    parse_records, filter_events, first_after, uart_lines, usb_read_fn)
from agentlab.host.discovery import find_device, find_devices

from test.agentlab_host_fake import *

# Events -------------------------------------------------------------------------------------------

class TestEvents(unittest.TestCase):
    def test_parse_record(self):
        record = bytes([0x01, 0x02, 0x41, 0, 0, 0, 0, 0]) + struct.pack("<Q", 0x0123456789abcdef)
        e = parse_record(record)
        self.assertEqual(e, Event(c.EVENT_UART_RX, 2, 0x41, 0x0123456789abcdef))
        self.assertEqual(e.name, "uart_rx")
        self.assertEqual(e.to_dict(SYS_CLK_FREQ)["char"], "A")

    def test_value_48bit_little_endian(self):
        record = bytes([0x06, 0x00, 1, 2, 3, 4, 5, 6]) + struct.pack("<Q", 8)
        e = parse_record(record)
        self.assertEqual(e.value, 0x060504030201)
        self.assertEqual(e.name, "marker")
        self.assertEqual(pack_event(e), record)

    def test_parse_errors(self):
        with self.assertRaises(ValueError):
            parse_record(bytes(15))
        with self.assertRaises(ValueError):
            parse_records(bytes(17))

    def test_parser_split_transfers(self):
        events = [Event(c.EVENT_GPIO_EDGE, i, i & 1, 100*i) for i in range(10)]
        data   = b"".join(pack_event(e) for e in events)
        parser = EventParser()
        out    = []
        for i in range(0, len(data), 7):
            out += parser.feed(data[i:i + 7])
        self.assertEqual(out, events)

    def test_time_conversion(self):
        e = Event(c.EVENT_POWER, 0, 1, 125_000_000)
        self.assertAlmostEqual(e.time(SYS_CLK_FREQ), 1.0)
        self.assertAlmostEqual(e.to_dict(SYS_CLK_FREQ)["time_s"], 1.0)

    def test_filter_and_correlation(self):
        events = [
            Event(c.EVENT_MARKER,    0, 1, 1000),
            Event(c.EVENT_GPIO_EDGE, 3, 1, 2000),
            Event(c.EVENT_POWER,     0, 0, 3000),
            Event(c.EVENT_GPIO_EDGE, 3, 0, 4000),
            Event(c.EVENT_GPIO_EDGE, 4, 1, 5000),
        ]
        self.assertEqual(len(filter_events(events, since=2000, until=4000)), 3)
        self.assertEqual(len(filter_events(events, types="gpio", source=3)), 2)
        self.assertEqual(len(filter_events(events, types=["gpio", c.EVENT_POWER])), 4)
        # Seconds with sys_clk_freq.
        r = filter_events(events, since=2500/SYS_CLK_FREQ, sys_clk_freq=SYS_CLK_FREQ)
        self.assertEqual([e.timestamp for e in r], [3000, 4000, 5000])
        self.assertEqual(first_after(events, 2500, types="gpio").timestamp, 4000)
        self.assertIsNone(first_after(events, 6000))
        with self.assertRaises(ValueError):
            filter_events(events, types="nope")

    def test_uart_lines(self):
        events  = uart_events("boot\r\nlogin: ", source=0, t0=100)
        events += uart_events("hello\n", source=1, t0=150, dt=1)
        events.sort(key=lambda e: e.timestamp)
        lines = uart_lines(events)
        self.assertEqual([(l.source, l.text, l.timestamp) for l in lines], [
            (0, "boot", 100), (1, "hello", 150), (0, "login: ", 100 + 6*1000)])
        self.assertEqual([l.text for l in uart_lines(events, source=0, flush_partial=False)],
            ["boot"])

    def test_reader_ring_buffer(self):
        endpoint = FakeEventsEndpoint()
        endpoint.push_events([Event(c.EVENT_MARKER, 0, i, i) for i in range(10)], split=24)
        endpoint.push_events([Event(c.EVENT_OVERFLOW, 0, 0, 10)])
        reader = EventReader(endpoint, depth=4)
        while endpoint.transfers:
            reader.poll()
        self.assertEqual([e.timestamp for e in reader.snapshot()], [7, 8, 9, 10])
        self.assertEqual(reader.dropped, 7)
        self.assertEqual(reader.overflows, 1)

    def test_reader_thread(self):
        endpoint = FakeEventsEndpoint()
        reader   = EventReader(endpoint).start()
        try:
            endpoint.push_events([Event(c.EVENT_GPIO_EDGE, 5, 1, 42)])
            e = reader.wait_for(lambda e: e.type == c.EVENT_GPIO_EDGE, timeout=2.0)
            self.assertEqual(e.timestamp, 42)
            self.assertIsNone(reader.wait_for(lambda e: e.source == 9, timeout=0.05))
        finally:
            reader.stop()
        self.assertFalse(reader.running)

    def test_reader_error(self):
        def read_fn(size, timeout_ms):
            raise OSError("unplugged")
        reader = EventReader(read_fn).start()
        self.assertIsNone(reader.wait_for(lambda e: True, timeout=2.0))
        self.assertIsInstance(reader.error, OSError)
        reader.stop()

    def test_usb_read_fn_error_clear_halt(self):
        # USB I/O error: endpoint halt cleared (host/device Sequence Numbers resynchronized), error
        # raised; timeout: no data.
        import usb.core
        import usb.util
        class Device:
            def __init__(self):
                self.errors  = [usb.core.USBTimeoutError("timeout"), usb.core.USBError("io", 5)]
                self.cleared = []
            def read(self, endpoint, size, timeout):
                if self.errors:
                    raise self.errors.pop(0)
                return [1, 2, 3]
            def clear_halt(self, endpoint):
                self.cleared.append(endpoint)
        device = Device()
        claim  = usb.util.claim_interface
        usb.util.claim_interface = lambda device, interface: None
        try:
            read = usb_read_fn(device, interface=2, endpoint=0x8c)
        finally:
            usb.util.claim_interface = claim
        self.assertEqual(read(16, 100), b"")
        with self.assertRaises(usb.core.USBError):
            read(16, 100)
        self.assertEqual(device.cleared, [0x8c])
        self.assertEqual(read(16, 100), bytes([1, 2, 3]))

# CSRs ---------------------------------------------------------------------------------------------

class TestCSR(unittest.TestCase):
    def test_csv(self):
        m = CSRMap.from_csv(csr_csv())
        self.assertEqual(m[c.CSR_TIMESTAMP].size, 2)
        self.assertEqual(m[c.CSR_POWER_OUT].addr, 0xf0000000 + 4*4)
        self.assertEqual(m.sys_clk_freq, SYS_CLK_FREQ)
        self.assertEqual(m.data_width, 32)
        with self.assertRaises(KeyError):
            m["nope"]

    def test_json_and_file(self):
        d = {
            "csr_registers" : {"agentlab_marker": {"addr": 0x800, "size": 1, "type": "wo"}},
            "constants"     : {"config_clock_frequency": 100000000, "config_csr_data_width": 8},
        }
        with tempfile.TemporaryDirectory() as tmp:
            for ext, content in [(".json", json.dumps(d)), (".csv", csr_csv())]:
                fn = os.path.join(tmp, "csr" + ext)
                with open(fn, "w") as f:
                    f.write(content)
                m = CSRMap.from_file(fn)
                self.assertIn(c.CSR_MARKER, m)
        m = CSRMap.from_json(json.dumps(d))
        self.assertEqual((m.data_width, m.sys_clk_freq), (8, 100e6))

    def test_multiword(self):
        m    = CSRMap.from_csv(csr_csv())
        bus  = FakeCSRBus(m)
        csrs = CSRs(bus, m)
        bus.clock.now = 100.0 # 12.5e9 ticks: > 32-bit.
        self.assertEqual(csrs.read(c.CSR_TIMESTAMP), int(100*SYS_CLK_FREQ))
        csrs.write(c.CSR_POWER_OUT, 0x5)
        self.assertEqual(csrs.read(c.CSR_POWER_OUT), 0x5)

    def test_csr_data_width_8(self):
        m = CSRMap.from_csv(csr_csv().replace("csr_data_width,32", "csr_data_width,8"))
        port = FakeUARTBonePort()
        csrs = CSRs(UARTBoneBus(port), m)
        csrs.write(c.CSR_TIMESTAMP, 0x1234) # 2 words of 8 bits, MSB first.
        addr = m[c.CSR_TIMESTAMP].addr//4
        self.assertEqual((port.mem[addr], port.mem[addr + 1]), (0x12, 0x34))
        self.assertEqual(csrs.read(c.CSR_TIMESTAMP), 0x1234)

    def test_uartbone_wire_protocol(self):
        port = FakeUARTBonePort()
        bus  = UARTBoneBus(port)
        bus.write(0xf0000010, [0xdeadbeef, 0x01020304])
        self.assertEqual(port.frames[0],
            bytes([0x01, 2, 0x3c, 0x00, 0x00, 0x04]) + bytes.fromhex("deadbeef01020304"))
        self.assertEqual(bus.read(0xf0000010, 2), [0xdeadbeef, 0x01020304])
        self.assertEqual(port.frames[1], bytes([0x02, 2, 0x3c, 0x00, 0x00, 0x04]))
        # Bursts split at 255 words.
        bus.write(0x0, list(range(300)))
        self.assertEqual([f[1] for f in port.frames[2:]], [255, 45])
        self.assertEqual(bus.read(0x0, 300), list(range(300)))

    def test_uartbone_timeout(self):
        class Silent(FakeUARTBonePort):
            def write(self, data):
                return len(data)
        bus = UARTBoneBus(Silent(), timeout=0.01)
        with self.assertRaises(TimeoutError):
            bus.read(0)

# Discovery ----------------------------------------------------------------------------------------

def make_sysfs(root, name="2-1", serial="AL0001", vid="1d50", pid="605a", ttys=None,
    classes=None):
    dev = os.path.join(root, name)
    os.makedirs(dev)
    for k, v in [("idVendor", vid), ("idProduct", pid), ("busnum", "2"), ("devnum", "3"),
        ("serial", serial), ("product", "AgentLab"), ("speed", "5000")]:
        with open(os.path.join(dev, k), "w") as f:
            f.write(v + "\n")
    if classes is None:
        classes = {i: "02" if i < 10 and i % 2 == 0 else "0a" for i in range(10)}
        classes.update({10: "ff", 11: "ff", 12: "ff", 13: "0e", 14: "0e"})
    for intf, cls in classes.items():
        intf_dir = os.path.join(root, "{}:1.{}".format(name, intf))
        os.makedirs(intf_dir)
        with open(os.path.join(intf_dir, "bInterfaceClass"), "w") as f:
            f.write(cls + "\n")
    for intf, tty in (ttys or {}).items():
        os.makedirs(os.path.join(root, "{}:1.{}".format(name, intf), "tty", tty))

class TestDiscovery(unittest.TestCase):
    def test_find(self):
        with tempfile.TemporaryDirectory() as root:
            make_sysfs(root, ttys={0: "ttyACM3", 2: "ttyACM4", 4: "ttyACM5", 8: "ttyACM7"})
            make_sysfs(root, name="1-4", vid="1234")
            devices = find_devices(sysfs_root=root)
            self.assertEqual(len(devices), 1)
            d = find_device(sysfs_root=root)
            self.assertEqual(d.control_tty, "/dev/ttyACM3")
            self.assertEqual([d.uart_tty(i) for i in range(4)],
                ["/dev/ttyACM4", "/dev/ttyACM5", None, "/dev/ttyACM7"])
            self.assertEqual(d.describe()["speed"], "5000")
            with self.assertRaises(IndexError):
                d.uart_tty(4)
            with self.assertRaises(RuntimeError):
                find_device(serial="other", sysfs_root=root)

    def test_other_layout(self):
        # Same VID/PID, other USB3-PIPE design (UVC camera): ignored unless check_layout=False.
        with tempfile.TemporaryDirectory() as root:
            make_sysfs(root, classes={0: "0e", 1: "0e"})
            self.assertEqual(find_devices(sysfs_root=root), [])
            self.assertEqual(len(find_devices(sysfs_root=root, check_layout=False)), 1)

    def test_several(self):
        with tempfile.TemporaryDirectory() as root:
            make_sysfs(root, name="2-1", serial="A")
            make_sysfs(root, name="2-2", serial="B")
            with self.assertRaises(RuntimeError):
                find_device(sysfs_root=root)
            self.assertEqual(find_device(serial="B", sysfs_root=root).sysfs_name, "2-2")

    def test_none(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(RuntimeError):
                find_device(sysfs_root=root)

# UART ---------------------------------------------------------------------------------------------

class TestUART(unittest.TestCase):
    def make(self, chunks):
        clock = FakeClock()
        port  = FakeSerial(clock, chunks)
        return DUTUart(port, clock=clock), port, clock

    def test_wait_for_match(self):
        uart, port, clock = self.make([
            (0.1, b"U-Boot 2026\r\n"),
            (0.3, b"IP: 10.0.0.2\r\nlogin: "),
        ])
        r = uart.wait_for(r"IP: (\S+)", timeout=5)
        self.assertTrue(r.matched)
        self.assertEqual(r.groups, ("10.0.0.2",))
        self.assertTrue(r.text.startswith("U-Boot") and r.text.endswith("10.0.0.2"))
        self.assertGreaterEqual(r.elapsed, 0.3)
        # Remaining text kept for next read/wait.
        self.assertEqual(uart.read(timeout=0.2), b"\r\nlogin: ")

    def test_wait_for_timeout(self):
        uart, port, clock = self.make([(0.1, b"booting...")])
        r = uart.wait_for("login:", timeout=1.0)
        self.assertFalse(r.matched)
        self.assertEqual(r.text, "booting...")
        self.assertGreaterEqual(r.elapsed, 1.0)
        self.assertEqual(r.to_dict()["match"], None)

    def test_wait_for_invalid_utf8(self):
        uart, port, clock = self.make([(0, b"\xff\xfe>> ok\xc3")])
        r = uart.wait_for(">> ok", timeout=1.0)
        self.assertTrue(r.matched)
        self.assertEqual(uart.read(timeout=0.1), b"\xc3")

    def test_read_size_and_write(self):
        uart, port, clock = self.make([(0, b"abcdef")])
        self.assertEqual(uart.read(timeout=1, size=4), b"abcd")
        self.assertEqual(uart.read(timeout=0.1), b"ef")
        self.assertEqual(uart.write("ls\r"), 3)
        self.assertEqual(port.tx, b"ls\r")
        uart.set_baudrate(9600)
        self.assertEqual(port.baudrate, 9600)

# Instrument ---------------------------------------------------------------------------------------

class TestInstrument(unittest.TestCase):
    def test_power_cycle_sequencing(self):
        i, f = make_instrument()
        i.power_set(0, True)
        i.power_set(2, True)
        f["bus"].log.clear()
        i.power_cycle(2, off_time=0.5)
        writes = [(t, v) for t, n, v in f["bus"].log if n == c.CSR_POWER_OUT]
        self.assertEqual(writes, [(0.0, 0b001), (0.5, 0b101)])
        self.assertTrue(i.power_get(2))
        i.power_set(0, False)
        self.assertFalse(i.power_get(0))

    def test_gpio(self):
        i, f = make_instrument()
        self.assertTrue(i.gpio_get(3)) # Released: pulled-up.
        i.gpio_set(3, False)
        self.assertFalse(i.gpio_get(3))
        i.gpio_set(3, True)
        self.assertTrue(i.gpio_get(3))
        i.gpio_set(3, False)
        i.gpio_release(3)
        self.assertTrue(i.gpio_get(3))
        i.gpio_edge_events(3)
        self.assertEqual(f["bus"].read(f["bus"].csr_map[c.CSR_GPIO_EDGE_MASK].addr), [0b1000])

    def test_timestamp_marker(self):
        i, f = make_instrument()
        f["clock"].now = 2.0
        self.assertEqual(i.timestamp(), int(2*SYS_CLK_FREQ))
        self.assertAlmostEqual(i.time(), 2.0)
        self.assertEqual(i.marker(0x55), int(2*SYS_CLK_FREQ))
        names = [n for t, n, v in f["bus"].log]
        self.assertEqual(names, [c.CSR_TIMESTAMP_LATCH, c.CSR_TIMESTAMP_LATCH, c.CSR_MARKER,
            c.CSR_TIMESTAMP_LATCH])
        self.assertEqual(i.sys_clk_freq, SYS_CLK_FREQ)

    def test_uart_baudrate(self):
        i, f = make_instrument()
        i.uart_set_baudrate(0, 921600)
        self.assertEqual(f["ports"][0].baudrate, 921600)
        self.assertEqual(i.csrs.read(c.csr_uart_baudrate(0)), 921600)
        i.uart_set_baudrate(1, 9600) # No CSR for UART1: line coding only.
        self.assertEqual(f["ports"][1].baudrate, 9600)

    def test_events_correlation(self):
        events  = [Event(c.EVENT_MARKER, 0, 1, int(1.0*SYS_CLK_FREQ))]
        events += uart_events("hi\n", t0=int(1.5*SYS_CLK_FREQ))
        events += [Event(c.EVENT_POWER, 0, 1, int(3.0*SYS_CLK_FREQ))]
        i, f = make_instrument(events=events)
        try:
            i.events.wait_for(lambda e: e.type == c.EVENT_POWER, timeout=2.0)
            self.assertEqual(len(i.events_between(since=1.2, until=2.0)), 3)
            self.assertEqual(len(i.events_between(types="marker")), 1)
            lines = i.uart_lines(index=0)
            self.assertEqual(lines[0].text, "hi")
            self.assertAlmostEqual(i.ticks_to_s(lines[0].timestamp), 1.5)
        finally:
            i.close()

    def test_no_csr(self):
        from agentlab.host.instrument import Instrument
        i = Instrument()
        self.assertEqual(i.sys_clk_freq, c.DEFAULT_SYS_CLK_FREQ)
        with self.assertRaises(RuntimeError):
            i.power_set(0, True)
        with self.assertRaises(RuntimeError):
            i.uart(0)
        with self.assertRaises(RuntimeError):
            i.events

    def test_open_with_fake_sysfs(self):
        from agentlab.host.instrument import Instrument
        with tempfile.TemporaryDirectory() as root:
            make_sysfs(root, ttys={0: "ttyACM0"})
            i = Instrument.open(sysfs_root=root)
            self.assertEqual(i.info()["control"], "/dev/ttyACM0")
            with self.assertRaises(RuntimeError):
                i.uart(1) # No tty for UART1.
            with self.assertRaises(RuntimeError):
                i.timestamp() # No CSR map.

if __name__ == "__main__":
    unittest.main()
