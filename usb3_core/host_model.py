#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Host model (simulation).

Scripted Host (Downstream Port) Link/Protocol Layer, used to test the USB3 Core (device) and the PHY
against a host behaving like a real xHCI host: Header Sequence Number/Credits Advertisements right
after U0 entry, Port Capability/Configuration LMPs, control transfers. The packets formats/CRCs are
checked against packets captured on hardware (xHCI host, see test/test_host_model.py).

- Packets encoders (Link Layer words: 32-bit data/4-bit ctrl, first symbol on data[31:24]).
- USB3HostModel: plays a script (send words, wait for received words, request SKP Ordered Sets
  insertion) and logs the received words with Display ("HOST_RX <data> <ctrl>") while running.
- Scripts builders (link initialization, control transfers).
"""

import zlib
import random

from collections import deque

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import phy_layout

# CRCs (USB 3.2 7.2.1/7.2.2: bits transmitted LSB first, inverted remainder) -----------------------

def _crc(data_bits, width, poly, init):
    crc = init
    for bit in data_bits:
        msb = (crc >> (width - 1)) & 0x1
        crc = (crc << 1) & ((1 << width) - 1)
        if bit ^ msb:
            crc ^= poly
    crc ^= (1 << width) - 1
    return int("{:0{}b}".format(crc, width)[::-1], 2)

def crc5(value, nbits=11):
    """CRC-5 of Link Commands/Link Control Words (x^5+x^2+1)."""
    return _crc([(value >> i) & 0x1 for i in range(nbits)], 5, 0x05, 0x1f)

def crc16(data):
    """CRC-16 of Header Packets (x^16+x^12+x^3+x+1)."""
    return _crc([(byte >> i) & 0x1 for byte in data for i in range(8)], 16, 0x100b, 0xffff)

def crc32(data):
    """CRC-32 of Data Packet Payloads."""
    return zlib.crc32(bytes(data))

# Symbols <-> Link Layer Words ---------------------------------------------------------------------

SHP = 0xfb # K27.7.
SDP = 0x5c # K28.2.
SLC = 0xfe # K30.7.
END = 0xfd # K29.7.
EPF = 0xf7 # K23.7.

def words_from_symbols(symbols):
    """(byte, k) symbols -> Link Layer (data, ctrl) words (padded with Logical Idle)."""
    symbols = symbols + [(0x00, 0)]*((-len(symbols)) % 4)
    words   = []
    for i in range(0, len(symbols), 4):
        data = ctrl = 0
        for byte, k in symbols[i:i+4]:
            data = (data << 8) | byte
            ctrl = (ctrl << 1) | k
        words.append((data, ctrl))
    return words

def symbols_from_words(words):
    symbols = []
    for data, ctrl in words:
        for i in reversed(range(4)):
            symbols.append(((data >> 8*i) & 0xff, (ctrl >> i) & 0x1))
    return symbols

def framing(k):
    return [(k, 1)]*3 + [(EPF, 1)]

def le_bytes(value, n):
    return list(value.to_bytes(n, "little"))

# Link Commands (USB 3.2 7.2.2) --------------------------------------------------------------------

def LGOOD(n): return 0x000 | (n & 0x7)
def LCRD(x):  return 0x080 | (x & 0x3)
def LRTY():   return 0x100
def LUP():    return 0x400
def LDN():    return 0x580
def LGO_U(n): return 0x200 | (n & 0x3)
def LAU():    return 0x280
def LXU():    return 0x300
def LPMA():   return 0x380

def link_command(info):
    lc = info | (crc5(info) << 11)
    return words_from_symbols(framing(SLC) + [(b, 0) for b in le_bytes(lc, 2)*2])

# Header Packets (USB 3.2 7.2.1, 8.3) --------------------------------------------------------------

def header_packet(dw0, dw1, dw2, seq):
    """Header Packet: SHP framing, 3 DWords, CRC-16 and Link Control Word (Header Sequence Number)."""
    header = le_bytes(dw0, 4) + le_bytes(dw1, 4) + le_bytes(dw2, 4)
    lcw    = (seq & 0x7) | (crc5(seq & 0x7) << 11)
    return words_from_symbols(framing(SHP) + [(b, 0) for b in header + le_bytes(crc16(header), 2) + le_bytes(lcw, 2)])

def data_packet_payload(payload):
    """Data Packet Payload: SDP framing, payload, CRC-32 and END framing."""
    payload = list(payload)
    return words_from_symbols(framing(SDP) + [(b, 0) for b in payload + le_bytes(crc32(payload), 4)] + framing(END))

# LMPs (USB 3.2 8.4).
LMP_SUB_PORT_CAP      = 4
LMP_SUB_PORT_CFG      = 5
LMP_SUB_PORT_CFG_RESP = 6

def lmp_port_capability(seq):
    # Downstream Port: 5Gbps, 4 Rx Header Buffers, Downstream capable (as sent by an xHCI host).
    return header_packet(0x00000000 | (LMP_SUB_PORT_CAP << 5) | (1 << 9), 0x00010004, 0, seq)

def lmp_port_configuration(seq):
    return header_packet(0x00000000 | (LMP_SUB_PORT_CFG << 5) | (1 << 9), 0, 0, seq)

# Transaction Packets (USB 3.2 8.5).
TP_SUB_ACK    = 1
TP_SUB_NRDY   = 2
TP_SUB_ERDY   = 3
TP_SUB_STATUS = 4
TP_SUB_STALL  = 5

def tp(subtype, seq, addr=0, ept=0, direction=0, nump=0, tp_seq=0, pp=0):
    dw0 = 0x04 | (addr << 25)
    dw1 = subtype | (direction << 7) | (ept << 8) | (nump << 16) | (tp_seq << 21)
    dw2 = pp << 27
    return header_packet(dw0, dw1, dw2, seq)

def dph(seq, length, addr=0, ept=0, dp_seq=0, setup=0):
    dw0 = 0x08 | (addr << 25)
    dw1 = dp_seq | (ept << 8) | (setup << 15) | (length << 16)
    return header_packet(dw0, dw1, 0, seq)

# Received Packets Decoding ------------------------------------------------------------------------

def decode_packets(words, positions=None):
    """Decode received Link Layer words into packets: ("LC", info), ("HP", dw0, dw1, dw2, seq, crc_ok),
    ("DPP", payload, crc_ok). The index of the first word of each packet is appended to positions."""
    symbols = [s for s in symbols_from_words(words)]
    packets = []
    def append(packet, i):
        packets.append(packet)
        if positions is not None:
            positions.append(i//4)
    i = 0
    while i < len(symbols):
        if symbols[i:i+4] == framing(SLC):
            lc   = symbols[i+4][0] | (symbols[i+5][0] << 8)
            info = lc & 0x7ff
            append(("LC", info, (lc >> 11) == crc5(info)), i)
            i += 8
        elif symbols[i:i+4] == framing(SHP):
            data = [b for b, k in symbols[i+4:i+20]]
            dws  = [int.from_bytes(bytes(data[4*j:4*j+4]), "little") for j in range(3)]
            crc  = int.from_bytes(bytes(data[12:14]), "little")
            lcw  = int.from_bytes(bytes(data[14:16]), "little")
            append(("HP", *dws, lcw & 0x7, crc == crc16(data[:12])), i)
            i += 20
        elif symbols[i:i+4] == framing(SDP):
            j = i + 4
            while j < len(symbols) and symbols[j:j+4] != framing(END):
                j += 1
            data = [b for b, k in symbols[i+4:j]]
            payload, crc = data[:-4], int.from_bytes(bytes(data[-4:]), "little")
            append(("DPP", payload, crc == crc32(payload)), i)
            i = j + 4
        else:
            i += 1
    return packets

# USB3 Host Model (gateware) -----------------------------------------------------------------------

OP_SEND = 0 # Send data/ctrl word (first/last packet delimiters).
OP_WAIT = 1 # Wait a received word matching data (under mask)/ctrl.
OP_SKP  = 2 # Request a SKP Ordered Set insertion (SerDes model).
OP_MARK = 3 # Display a step marker.
OP_END  = 4 # End of script.
OP_IDLE = 5 # Logical Idle for data cycles.

class USB3HostModel(LiteXModule):
    """Scripted USB3 Host Link/Protocol Layer model.

    The script is a list of (op, data, ctrl, mask, first, last) steps, see the scripts builders. The
    received words are displayed ("HOST_RX <data> <ctrl>") while the script runs, the received
    packets are then checked by the test from the simulation output.
    """
    def __init__(self, script, wait_timeout=2**16):
        self.enable   = Signal() # i: Run the script (ex: LTSSM in U0), restarts it when deasserted.
        self.sink     = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source   = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.skp      = Signal() # o: SKP Ordered Set insertion request (SerDes model).
        self.done     = Signal() # o
        self.error    = Signal() # o

        # # #

        # Script ROM.
        layout = [("op", 3), ("data", 32), ("ctrl", 4), ("mask", 32), ("first", 1), ("last", 1)]
        width  = sum(w for _, w in layout)
        def encode(step):
            value, offset = 0, 0
            for (name, w), v in zip(layout, step):
                value  |= (v & ((1 << w) - 1)) << offset
                offset += w
            return value
        mem  = Memory(width, len(script) + 1, init=[encode(s) for s in script] + [encode((OP_END, 0, 0, 0, 0, 0))])
        port = mem.get_port(async_read=True)
        self.specials += mem, port
        step  = Signal(max=len(script) + 1)
        fields = {}
        offset = 0
        for name, w in layout:
            fields[name] = Signal(w, name=f"script_{name}")
            self.comb += fields[name].eq(port.dat_r[offset:offset + w])
            offset += w
        self.comb += port.adr.eq(step)

        # RX logging.
        self.comb += sink.ready.eq(1)
        self.sync += If(self.enable & ~self.done & sink.valid,
            Display("HOST_RX %08x %01x", sink.data, sink.ctrl)
        )

        # Script execution.
        timer = WaitTimer(wait_timeout)
        self.submodules += timer
        idle_count = Signal(16)
        match = Signal()
        self.comb += match.eq(sink.valid &
            ((sink.data & fields["mask"]) == (fields["data"] & fields["mask"])) &
            (sink.ctrl == fields["ctrl"]))
        self.comb += [
            source.data.eq(fields["data"]),
            source.ctrl.eq(fields["ctrl"]),
            source.first.eq(fields["first"]),
            source.last.eq(fields["last"]),
        ]
        run = Signal()
        self.comb += run.eq(self.enable & ~self.error)
        self.comb += If(self.enable, Case(fields["op"], {
            OP_SEND: [
                source.valid.eq(run),
            ],
            OP_WAIT: [
                timer.wait.eq(run),
            ],
            OP_SKP: [
                self.skp.eq(run),
            ],
            OP_END: [
                self.done.eq(1),
            ],
            "default": []
        }))
        self.sync += If(~self.enable,
            step.eq(0),
            idle_count.eq(0),
            self.error.eq(0),
        ).Elif(~self.error,
            Case(fields["op"], {
                OP_SEND: If(source.valid & source.ready, step.eq(step + 1)),
                OP_WAIT: [
                    If(match,
                        step.eq(step + 1)
                    ).Elif(timer.done,
                        self.error.eq(1),
                        Display("HOST_TIMEOUT step %d", step)
                    )
                ],
                OP_SKP:  step.eq(step + 1),
                OP_MARK: [
                    Display("HOST_MARK %d", fields["data"]),
                    step.eq(step + 1)
                ],
                OP_IDLE: [
                    idle_count.eq(idle_count + 1),
                    If(idle_count == fields["data"],
                        idle_count.eq(0),
                        step.eq(step + 1)
                    )
                ],
                "default": []
            })
        )

# Scripts Builders ---------------------------------------------------------------------------------

class HostScript:
    """Host script builder (USB3HostModel steps), tracking the Header Sequence Numbers and the Rx
    Header Buffer Credits returned to the device."""
    def __init__(self):
        self.steps     = []
        self.offset    = 0 # Packets offset in the words (symbols, SKP Ordered Sets accounting).
        self.tx_seq    = 0 # Host Header Sequence Number.
        self.rx_seq    = 0 # Device Header Sequence Number (expected).
        self.marks     = []

    def send(self, words):
        # Packets offset by the SKP Ordered Sets accounting (Logical Idle symbols before the packet).
        if self.offset:
            words = words_from_symbols([(0x00, 0)]*self.offset + symbols_from_words(words))
        for i, (data, ctrl) in enumerate(words):
            self.steps.append((OP_SEND, data, ctrl, 0, int(i == 0), int(i == len(words) - 1)))

    def send_hp(self, builder, *args, **kwargs):
        self.send(builder(*args, seq=self.tx_seq, **kwargs))
        self.tx_seq = (self.tx_seq + 1) % 8

    def wait(self, data, ctrl=0, mask=0xffffffff):
        self.steps.append((OP_WAIT, data, ctrl, mask, 0, 0))

    def wait_hp(self):
        # Wait a device Header Packet and acknowledge it (LGOOD_n + LCRD_x, 7.2.4).
        self.wait(0xfbfbfbf7, ctrl=0b1111)
        self.send(link_command(LGOOD(self.rx_seq)))
        self.send(link_command(LCRD(self.rx_seq % 4)))
        self.rx_seq = (self.rx_seq + 1) % 8

    def idle(self, cycles):
        self.steps.append((OP_IDLE, cycles, 0, 0, 0, 0))

    def skp(self):
        # SKP Ordered Set between packets: Logical Idle around the request (SerDes model latency).
        # A host aligns its packets on its symbols count, which includes the SKP Ordered Sets: after
        # a single SKP Ordered Set (2 symbols), its packets are offset by 2 symbols in the words
        # received by the device (after the SKP removal).
        self.idle(32)
        self.steps.append((OP_SKP, 0, 0, 0, 0, 0))
        self.idle(32)
        self.offset = (self.offset + 2) % 4

    def mark(self, name):
        self.marks.append(name)
        self.steps.append((OP_MARK, len(self.marks) - 1, 0, 0, 0, 0))

    # Link initialization (7.2.4.1.1, 8.4.5): Advertisements right after U0, Port Capability and
    # Port Configuration LMPs.
    def link_init(self):
        self.mark("link_init")
        self.send(link_command(LGOOD(7)))
        for x in range(4):
            self.send(link_command(LCRD(x)))
        self.send_hp(lmp_port_capability)
        self.wait_hp() # Device Port Capability LMP.
        self.send_hp(lmp_port_configuration)
        self.wait_hp() # Device Port Configuration Response LMP.

    # Control transfers (8.12.2).
    def control_read(self, setup, name, latency=256, status_gap=64):
        self.mark(name)
        self.send_hp(dph, length=8, setup=1)
        self.send(data_packet_payload(setup))
        self.wait_hp()                                                # ACK TP (SETUP).
        self.idle(latency)                                            # Host latency (xHCI: ~1.5us).
        self.send_hp(tp, TP_SUB_ACK, nump=1, tp_seq=0, pp=1)          # Data stage request.
        self.wait_hp()                                                # DPH (+DPP).
        self.idle(128)                                                # DPP (END framing not aligned).
        self.send_hp(tp, TP_SUB_ACK, nump=0, tp_seq=1)                # Data acknowledgement.
        self.idle(status_gap)
        self.send_hp(tp, TP_SUB_STATUS)                               # Status stage.
        self.wait_hp()                                                # ACK TP (Status).

    def control_write(self, setup, name):
        self.mark(name)
        self.send_hp(dph, length=8, setup=1)
        self.send(data_packet_payload(setup))
        self.wait_hp()                                                # ACK TP (SETUP).
        self.send_hp(tp, TP_SUB_STATUS)                               # Status stage.
        self.wait_hp()                                                # ACK TP (Status).

    # Bulk transfers (8.10, 8.11).
    def bulk_out(self, payload, ept, dp_seq, name):
        # DPH + DPP, the device acknowledges with an ACK TP (next sequence number).
        self.mark(name)
        self.send_hp(dph, length=len(payload), ept=ept, dp_seq=dp_seq)
        self.send(data_packet_payload(payload))
        self.wait_hp()                                                # ACK TP.

    def bulk_in(self, ept, dp_seq, name, latency=256):
        # ACK TP (NumP=1) requesting a data packet, DPH + DPP from the device, ACK TP acknowledging it.
        self.mark(name)
        self.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1, nump=1, tp_seq=dp_seq, pp=1)
        self.wait_hp()                                                # DPH (+DPP).
        self.idle(latency)                                            # DPP (END framing not aligned).
        self.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1, nump=0, tp_seq=(dp_seq + 1) % 32)

    # Link Power Management (7.2.4.2): LGO_Ux request, LAU/LXU response.
    def lgo(self, n, response, name):
        self.mark(name)
        self.send(link_command(LGO_U(n)))
        self.wait(*link_command(response)[1])

    def end(self):
        self.mark("end")
        self.steps.append((OP_END, 0, 0, 0, 0, 0))

def setup_packet(bmRequestType, bRequest, wValue, wIndex, wLength):
    return [bmRequestType, bRequest] + le_bytes(wValue, 2) + le_bytes(wIndex, 2) + le_bytes(wLength, 2)

def enumeration_script(skp=True, latency=256, status_gap=64, bulk=True, pm=False):
    """Link initialization + enumeration-like control transfers (as Linux on an xHCI host)."""
    s = HostScript()
    s.link_init()
    if skp:
        # SKP Ordered Set (2 symbols) between packets: the framing is then shifted by 2 symbols in
        # the received words (re-aligned by the PHY).
        s.skp()
    s.control_read(setup_packet(0x80, 0x06, 0x0100, 0, 18), "get_device_descriptor", latency, status_gap)
    if skp:
        s.skp()
    s.control_read(setup_packet(0x80, 0x06, 0x0200, 0,  9), "get_configuration_descriptor", latency, status_gap)
    s.control_read(setup_packet(0x80, 0x06, 0x0301, 0x0409, 255), "get_string_descriptor", latency, status_gap)
    s.control_write(setup_packet(0x00, 0x09, 0x0001, 0, 0), "set_configuration")
    if bulk:
        # Bulk OUT (EP2) then Bulk IN (EP1): looped back by the device.
        for i, payload in enumerate(bulk_payloads()):
            s.bulk_out(payload, ept=2, dp_seq=i, name=f"bulk_out_{i}")
        s.idle(1024)
        for i in range(len(bulk_payloads())):
            s.bulk_in(ept=1, dp_seq=i, name=f"bulk_in_{i}", latency=latency + 256)
    if pm:
        # U1/U2 requests (rejected by the device: LXU, link stays in U0), then a control transfer.
        s.lgo(1, LXU(), "lgo_u1")
        s.lgo(2, LXU(), "lgo_u2")
        s.control_read(setup_packet(0x80, 0x06, 0x0100, 0, 18), "get_device_descriptor_after_pm", latency)
    s.end()
    return s

def bulk_payloads():
    """Bulk test packets (short and max size packets)."""
    return [[(7*i + 3) & 0xff for i in range(64)], [(i ^ (i >> 8)) & 0xff for i in range(1024)]]

# Simulation Output Parsing ------------------------------------------------------------------------

def parse_sim_output(output, script):
    """Received packets per script section (from the "HOST_MARK"/"HOST_RX" simulation output), a packet
    belongs to the section in which it starts (the script can move to the next section while the
    end of a packet is received)."""
    words, sections_at = [], []
    section = None
    for line in output.splitlines():
        p = line.split()
        if len(p) == 2 and p[0] == "HOST_MARK":
            section = script.marks[int(p[1])]
        elif len(p) == 3 and p[0] == "HOST_RX" and section is not None:
            words.append((int(p[1], 16), int(p[2], 16)))
            sections_at.append(section)
    positions = []
    packets   = decode_packets(words, positions)
    sections  = {name: [] for name in script.marks}
    for packet, position in zip(packets, positions):
        sections[sections_at[position]].append(packet)
    return sections

# Python Host (Migen simulation) -------------------------------------------------------------------
# Python models of the Host Link Layer to test a USB3 Device Core directly connected (no PHY) in a
# Migen simulation: a script player (HostScript steps) and a reactive host (acknowledges the device
# Header Packets like an xHCI host, test scenarios as generators).

class PHYStandIn:
    """USB3PHY Link interface stand-in."""
    def __init__(self):
        self.source = stream.Endpoint(phy_layout())
        self.sink   = stream.Endpoint(phy_layout())
        for name in ["u0", "polling_idle", "hot_reset", "recovery", "lgo_u1", "lgo_u2", "lgo_u3",
            "tx_lau", "tx_lxu", "directed_recovery", "link_activity"]:
            setattr(self, name, Signal(name=name))

def play_script(dut, script, max_cycles=200000, tx_ready_rate=1.0, seed=0):
    """Play a HostScript against the DUT, returns the simulation output (HOST_MARK/HOST_RX lines)."""
    phy    = dut.phy
    log    = []
    rx     = []
    state  = {"done": False}
    rng    = random.Random(seed)

    def host(dut):
        yield phy.polling_idle.eq(1)
        yield
        yield phy.polling_idle.eq(0)
        yield phy.u0.eq(1)
        yield phy.source.valid.eq(1)
        for op, data, ctrl, mask, first, last in script.steps:
            if op == OP_SEND:
                yield phy.source.data.eq(data)
                yield phy.source.ctrl.eq(ctrl)
                yield
            elif op == OP_IDLE:
                yield phy.source.data.eq(0)
                yield phy.source.ctrl.eq(0)
                for _ in range(data):
                    yield
            elif op == OP_WAIT:
                yield phy.source.data.eq(0)
                yield phy.source.ctrl.eq(0)
                start = len(rx)
                while True:
                    if any(((d & mask) == (data & mask)) and (c == ctrl) for d, c in rx[start:]):
                        break
                    yield
            elif op == OP_MARK:
                log.append(f"HOST_MARK {data}")
            elif op == OP_END:
                break
        yield phy.source.data.eq(0)
        yield phy.source.ctrl.eq(0)
        for _ in range(256):
            yield
        state["done"] = True

    @passive
    def monitor(dut):
        while True:
            ready = rng.random() < tx_ready_rate
            yield phy.sink.ready.eq(int(ready))
            yield
            if ready and (yield phy.sink.valid):
                d = (yield phy.sink.data)
                c = (yield phy.sink.ctrl)
                rx.append((d, c))
                log.append(f"HOST_RX {d:08x} {c:x}")

    @passive
    def timeout(dut):
        for _ in range(max_cycles):
            yield
        raise TimeoutError("host script timeout")

    run_simulation(dut, [host(dut), monitor(dut), timeout(dut)])
    return "\n".join(log)

FRAMING = {0xfbfbfbf7: "HP", 0xfefefef7: "LC", 0x5c5c5cf7: "DPP"}

class PyHost:
    """Host Link Layer: sends packets (atomic), acknowledges the device Header Packets (LGOOD_n +
    LCRD_x), records the received packets."""
    def __init__(self, phy, corrupt_lgood=False):
        self.phy     = phy
        self.queue   = deque() # Packets (lists of words) to send.
        self.urgent  = deque() # Link Commands (sent first, between packets).
        self.tx_seq  = 0
        self.lcrd    = 0
        self.hps     = []      # (dw0, dw1, dw2, seq, crc_ok).
        self.dpps    = []      # (payload, crc_ok).
        self.lcs     = []
        self.cycles  = 0

    def send_hp(self, builder, *args, **kwargs):
        self.queue.append(builder(*args, seq=self.tx_seq, **kwargs))
        self.tx_seq = (self.tx_seq + 1) % 8

    def send(self, words):
        self.queue.append(words)

    def driver(self):
        yield "passive"
        phy = self.phy
        yield phy.polling_idle.eq(1)
        yield
        yield phy.polling_idle.eq(0)
        yield phy.u0.eq(1)
        yield phy.source.valid.eq(1)
        current = deque()
        while True:
            if not current:
                if self.urgent:
                    current = deque(self.urgent.popleft())
                elif self.queue:
                    current = deque(self.queue.popleft())
            if current:
                data, ctrl = current.popleft()
            else:
                data, ctrl = 0, 0
            yield phy.source.data.eq(data)
            yield phy.source.ctrl.eq(ctrl)
            yield
            self.cycles += 1

    def monitor(self, tx_ready_rate=1.0, seed=0):
        yield "passive"
        phy   = self.phy
        rng   = random.Random(seed)
        state = None
        words = []
        while True:
            ready = rng.random() < tx_ready_rate
            yield phy.sink.ready.eq(int(ready))
            yield
            if not (ready and (yield phy.sink.valid)):
                continue
            w = ((yield phy.sink.data), (yield phy.sink.ctrl))
            if state is None:
                if w[1] == 0b1111 and w[0] in FRAMING:
                    state = FRAMING[w[0]]
                    words = [w]
                continue
            words.append(w)
            if state == "LC" and len(words) == 2:
                self.lcs.append(decode_packets(words)[0][1])
                state = None
            elif state == "HP" and len(words) == 5:
                packet = decode_packets(words)[0]
                self.hps.append(packet[1:])
                if packet[-1]:
                    self.urgent.append(link_command(LGOOD(packet[4])))
                    self.urgent.append(link_command(LCRD(self.lcrd)))
                    self.lcrd = (self.lcrd + 1) % 4
                state = None
            elif state == "DPP" and any(c & (1 << (3 - i)) and ((d >> 8*(3 - i)) & 0xff) == 0xf7
                for d, c in [w] for i in range(4)) and len(words) > 1:
                packets = decode_packets(words + [(0, 0)])
                if packets:
                    self.dpps.append(tuple(packets[0][1:]))
                state = None

    def wait(self, cond, timeout=200000):
        for _ in range(timeout):
            if cond():
                return
            yield
        raise TimeoutError("PyHost wait timeout")

# Received Packets Helpers ------------------------------------------------------------------------

def tps(packets):
    """Transaction Packets: (subtype, nump, seq)."""
    return [(dw1 & 0xf, (dw1 >> 16) & 0x1f, (dw1 >> 21) & 0x1f)
        for kind, *p in packets if kind == "HP" for dw0, dw1, dw2, seq, crc in [p] if (dw0 & 0x1f) == 0x04]

def lmps(packets):
    return [(dw0 >> 5) & 0xf for kind, *p in packets if kind == "HP" for dw0, dw1, dw2, seq, crc in [p]
        if (dw0 & 0x1f) == 0x00]

def dpps(packets):
    return [p[0] for kind, *p in packets if kind == "DPP"]

# Tests --------------------------------------------------------------------------------------------

def tp_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "subtype": dw1 & 0xf, "nump": (dw1 >> 16) & 0x1f,
        "seq": (dw1 >> 21) & 0x1f, "retry": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf}

def dph_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "seq": dw1 & 0x1f, "eob": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf,
        "length": dw1 >> 16}

def run_pyhost(dut, host, scenario, tx_ready_rate=1.0):
    run_simulation(dut, [host.driver(), host.monitor(tx_ready_rate), scenario], vcd_name=None)

def pyhost_link_init(host):
    host.send(link_command(LGOOD(7)))
    for x in range(4):
        host.send(link_command(LCRD(x)))
    host.send_hp(lmp_port_capability)
    host.send_hp(lmp_port_configuration)
    yield from host.wait(lambda: len(host.hps) >= 2)

# Control transfers (reactive host).

def pyhost_wait_tp(host, n0, subtypes=(TP_SUB_ACK, TP_SUB_STALL)):
    """Wait a Transaction Packet received after host.hps[n0], returns its fields."""
    def find():
        for hp in host.hps[n0:]:
            f = tp_fields(hp)
            if f["type"] == 0x04 and f["subtype"] in subtypes:
                return f
    yield from host.wait(lambda: find() is not None)
    return find()

def pyhost_control_in(host, setup):
    """Control read (one Data stage packet): returns the data or None if stalled."""
    n0 = len(host.hps)
    d0 = len(host.dpps)
    host.send_hp(dph, length=8, setup=1)
    host.send(data_packet_payload(setup))
    yield from pyhost_wait_tp(host, n0)
    n1 = len(host.hps)
    host.send_hp(tp, TP_SUB_ACK, nump=1, tp_seq=0, pp=1)
    yield from host.wait(lambda: (len(host.dpps) > d0) or
        any(tp_fields(hp)["subtype"] == TP_SUB_STALL for hp in host.hps[n1:] if (hp[0] & 0x1f) == 0x04))
    if len(host.dpps) == d0:
        return None
    host.send_hp(tp, TP_SUB_ACK, nump=0, tp_seq=1)
    n2 = len(host.hps)
    host.send_hp(tp, TP_SUB_STATUS)
    yield from pyhost_wait_tp(host, n2)
    return host.dpps[d0][0]

def pyhost_control_out(host, setup, data=None):
    """Control write (optional one packet Data stage): returns False if stalled."""
    n0 = len(host.hps)
    host.send_hp(dph, length=8, setup=1)
    host.send(data_packet_payload(setup))
    yield from pyhost_wait_tp(host, n0)
    if data:
        n1 = len(host.hps)
        host.send_hp(dph, length=len(data), dp_seq=0)
        host.send(data_packet_payload(data))
        f = yield from pyhost_wait_tp(host, n1)
        if f["subtype"] == TP_SUB_STALL:
            return False
    n2 = len(host.hps)
    host.send_hp(tp, TP_SUB_STATUS)
    f = yield from pyhost_wait_tp(host, n2)
    return f["subtype"] == TP_SUB_ACK

