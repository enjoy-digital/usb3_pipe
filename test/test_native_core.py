#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Native USB3 Device Core (Link/Protocol Layers, endpoints) against the host model scripts, in a
Migen simulation with a PHY stand-in (U0, data/ctrl words)."""

import random
import unittest

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common      import phy_layout
from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT

# PHY stand-in -------------------------------------------------------------------------------------

class PHYStandIn:
    """USB3PHY Link interface stand-in."""
    def __init__(self):
        self.source = stream.Endpoint(phy_layout())
        self.sink   = stream.Endpoint(phy_layout())
        for name in ["u0", "polling_idle", "hot_reset", "recovery", "lgo_u1", "lgo_u2", "lgo_u3",
            "tx_lau", "tx_lxu", "directed_recovery", "link_activity"]:
            setattr(self, name, Signal(name=name))

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self, max_burst=8):
        self.phy = phy = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors = bulk_descriptors(max_burst=max_burst),
            endpoints   = [USB3BulkIN(ept=1, max_burst=max_burst), USB3BulkOUT(ept=2, max_burst=max_burst)])
        # Bulk loopback: EP2 OUT -> EP1 IN.
        self.loopback = loopback = stream.SyncFIFO([("data", 32), ("last_be", 4)], 2*256, buffered=True)
        self.comb += [
            core.endpoints[1].source.connect(loopback.sink),
            loopback.source.connect(core.endpoints[0].sink),
        ]

# Host script player -------------------------------------------------------------------------------

def run_script(dut, script, max_cycles=200000, tx_ready_rate=1.0, seed=0):
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

# Helpers ------------------------------------------------------------------------------------------

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

class TestNativeCore(unittest.TestCase):
    def run_enumeration(self, **kwargs):
        script   = enumeration_script(**kwargs)
        dut      = DUT()
        out      = run_script(dut, script)
        sections = parse_sim_output(out, script)
        return dut, sections

    def check_crcs(self, sections):
        for name, packets in sections.items():
            for packet in packets:
                self.assertTrue(packet[-1], f"{name}: bad CRC {packet}")

    def test_enumeration(self):
        dut, sections = self.run_enumeration(skp=False)
        self.check_crcs(sections)

        # Link initialization: Advertisements, Port Capability, Port Configuration Response.
        link_init = sections["link_init"]
        lcs = [p[1] for p in link_init if p[0] == "LC"]
        self.assertEqual(lcs[:5], [LGOOD(7)] + [LCRD(x) for x in range(4)])
        self.assertEqual(lmps(link_init), [LMP_SUB_PORT_CAP, LMP_SUB_PORT_CFG_RESP])

        # GET_DESCRIPTOR (Device).
        descriptors = bulk_descriptors()
        packets = sections["get_device_descriptor"]
        self.assertEqual(tps(packets), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])
        self.assertEqual(dpps(packets), [descriptors.device()])

        # GET_DESCRIPTOR (Configuration, 9 bytes).
        self.assertEqual(dpps(sections["get_configuration_descriptor"]), [descriptors.configuration()[:9]])

        # GET_DESCRIPTOR (String 1).
        self.assertEqual(dpps(sections["get_string_descriptor"]), [descriptors.string(1)])

        # SET_CONFIGURATION.
        self.assertEqual(tps(sections["set_configuration"]), [(TP_SUB_ACK, 1, 1), (TP_SUB_ACK, 0, 1)])

        # Bulk OUT -> loopback -> Bulk IN.
        for i, payload in enumerate(bulk_payloads()):
            acks = tps(sections[f"bulk_out_{i}"])
            self.assertEqual(len(acks), 1)
            self.assertEqual((acks[0][0], acks[0][2]), (TP_SUB_ACK, i + 1))
            self.assertEqual(dpps(sections[f"bulk_in_{i}"]), [payload])

if __name__ == "__main__":
    unittest.main()
