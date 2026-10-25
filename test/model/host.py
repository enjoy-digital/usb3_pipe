#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Pure Python USB3 Host models for Migen simulations of the USB3 Device Core (PHY stand-in, no PHY).

Python models of the Host Link Layer to test a USB3 Device Core directly connected to a PHYStandIn:
a script player (usb3_core.host_model HostScript steps) and a reactive host (PyHost: acknowledges
the device Header Packets like an xHCI host, test scenarios as generators). Packets builders/decoders
and the gateware host model (USB3HostModel, used by sim.py) are in usb3_core/host_model.py.
"""

import random
import struct

from collections import deque

from migen import *

from usb3_core.host_model import *
from usb3_core.msc        import CBW_SIGNATURE

# Script Player ------------------------------------------------------------------------------------

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

# Reactive Host ------------------------------------------------------------------------------------

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

# Reactive Host Helpers ----------------------------------------------------------------------------

def tp_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "subtype": dw1 & 0xf, "nump": (dw1 >> 16) & 0x1f,
        "seq": (dw1 >> 21) & 0x1f, "retry": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf}

def dph_fields(hp):
    dw0, dw1, dw2, seq, crc = hp
    return {"type": dw0 & 0x1f, "seq": dw1 & 0x1f, "eob": (dw1 >> 6) & 1, "ept": (dw1 >> 8) & 0xf,
        "length": dw1 >> 16, "lpf": (dw1 >> 6) & 1, "direction": (dw1 >> 7) & 1}

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

def pyhost_bulk_in(host, ept, count, seq=0, nump=8):
    """Bulk IN: read count packets (DPPs) from the endpoint, acknowledging them and requesting again
    after ERDY (flow control). Returns (packets, next sequence number)."""
    d0     = len(host.dpps)
    erdys  = lambda: sum(1 for hp in host.hps if (hp[0] & 0x1f) == 0x04 and
        tp_fields(hp)["subtype"] == TP_SUB_ERDY and tp_fields(hp)["ept"] == ept)
    e0     = erdys()
    host.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1, nump=nump, tp_seq=seq, pp=1)
    while len(host.dpps) - d0 < count:
        n = len(host.dpps)
        e = erdys()
        yield from host.wait(lambda: (len(host.dpps) > n) or (erdys() > e))
        if len(host.dpps) > n:
            seq = (seq + len(host.dpps) - n) % 32
            host.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1,
                nump=(nump if len(host.dpps) - d0 < count else 0), tp_seq=seq, pp=1)
        else:
            host.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1, nump=nump, tp_seq=seq, pp=1)
    return [d[0] for d in host.dpps[d0:d0 + count]], seq

# Isochronous transfers (reactive host).

def pyhost_dps(host):
    """Received Data Packets: (DPH fields, payload, crc_ok), DPHs paired in order with their DPPs."""
    dphs = [dph_fields(hp) for hp in host.hps if (hp[0] & 0x1f) == 0x08]
    return [(d, *p) for d, p in zip(dphs, host.dpps)]

def pyhost_ping(host, ept, direction=1):
    """PING TP (8.5.7): returns the PING_RESPONSE TP fields."""
    n0 = len(host.hps)
    host.send_hp(tp, TP_SUB_PING, ept=ept, direction=direction)
    return (yield from pyhost_wait_tp(host, n0, subtypes=(TP_SUB_PING_RESP,)))

def pyhost_iso_in(host, ept, bursts=[1], ping=True):
    """Isochronous IN service interval (8.12.6.1): PING/PING_RESPONSE, then ACK TPs requesting the
    packets of the service interval (bursts: NumP of each ACK TP, first ACK TP with a Seq Num of 0,
    next ones with the previous Seq Num + NumP, sent once the previous burst received), until a
    packet with the Last Packet Flag. No response to the DPs (no retry). Returns the received DPs
    [(DPH fields, payload, crc_ok)]."""
    if ping:
        yield from pyhost_ping(host, ept)
    d0  = len(pyhost_dps(host))
    seq = 0
    def dps():
        return [dp for dp in pyhost_dps(host)[d0:] if dp[0]["ept"] == ept]
    for nump in bursts:
        host.send_hp(tp, TP_SUB_ACK, ept=ept, direction=1, nump=nump, tp_seq=seq)
        seq += nump
        yield from host.wait(lambda: (len(dps()) >= seq) or any(dp[0]["lpf"] for dp in dps()))
        if any(dp[0]["lpf"] for dp in dps()):
            break
    return dps()

# Mass Storage (Bulk-Only Transport) ---------------------------------------------------------------

def cbw(tag, length, din, cb):
    """Command Block Wrapper (31 bytes)."""
    cb = bytes(cb).ljust(16, b"\x00")
    return list(struct.pack("<IIIBBB16s", CBW_SIGNATURE, tag, length, 0x80 if din else 0, 0, len(cb), cb))
