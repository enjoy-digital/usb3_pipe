#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core Link Power Management (U1/U2, USB 3.2 7.2.4.2, 9.4.5): SET/CLEAR_FEATURE(U1/U2
_ENABLE) and GET_STATUS, LGO_U1/LGO_U2 accepted (LAU) or rejected (LXU), U1/U2 entry (LPMA), exit
(host or device initiated) through Recovery and Advertisements on U0 re-entry (Migen simulation,
reactive host, LTSSM PM handshake emulated on the PHY stand-in)."""

import unittest

from migen import *

from litex.gen import *

from usb3_core.host_model  import *
from usb3_core.descriptors import bulk_descriptors
from usb3_core.device      import USB3DeviceCore
from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT

# DUT ----------------------------------------------------------------------------------------------

class DUT(LiteXModule):
    def __init__(self, with_lpm=True):
        self.phy  = phy  = PHYStandIn()
        self.core = core = USB3DeviceCore(phy, sys_clk_freq=125e6,
            descriptors = bulk_descriptors(max_burst=2),
            endpoints   = [USB3BulkIN(ept=1, max_burst=2), USB3BulkOUT(ept=2, max_burst=2)],
            with_lpm    = with_lpm)

# LTSSM PM handshake emulation ---------------------------------------------------------------------

LMP_SUB_U2_INACT = 2

def lmp_u2_inactivity_timeout(value, seq):
    return header_packet(0x00000000 | (LMP_SUB_U2_INACT << 5) | (value << 9), 0, 0, seq)

class LTSSMEmulator:
    """LTSSM (Upstream Port) PM handshake on the PHY stand-in: forwarded LGO_U1/LGO_U2 -> LAU, Ux on
    LPMA (or PM_ENTRY_TIMER), exit (host request or device exit_ux_req) through Recovery to U0, then
    host Advertisements (Header Sequence Number, Rx Header Buffer Credits)."""
    def __init__(self, dut, host, pm_entry=1000, recovery=64):
        self.dut      = dut
        self.host     = host
        self.pm_entry = pm_entry
        self.recovery = recovery
        self.state    = "U0"
        self.lgos     = []    # Forwarded LGO_Ux.
        self.lpma     = 0     # LPMA forwarded.
        self.exit     = False # Host initiated exit request.
        self.exits    = []    # Exits: "host"/"device".
        self.entries  = 0     # U0 re-entries.

    def run(self):
        yield "passive"
        phy = self.dut.phy
        while True:
            lgo = 0
            for n, signal in [(1, phy.lgo_u1), (2, phy.lgo_u2)]:
                if (yield signal):
                    lgo = n
            yield
            if not lgo:
                continue
            self.lgos.append(lgo)
            # LAU, then Ux on LPMA (or PM_ENTRY_TIMER timeout).
            yield phy.tx_lau.eq(1)
            yield
            yield phy.tx_lau.eq(0)
            for _ in range(self.pm_entry):
                if (yield phy.lpma):
                    self.lpma += 1
                    break
                yield
            ux = {1: phy.u1, 2: phy.u2}[lgo]
            yield phy.u0.eq(0)
            yield ux.eq(1)
            self.state = f"U{lgo}"
            yield
            # Exit: host or device initiated.
            exit_req = {1: phy.exit_u1_req, 2: phy.exit_u2_req}[lgo]
            while True:
                if self.exit:
                    self.exits.append("host")
                    break
                if (yield exit_req):
                    self.exits.append("device")
                    break
                yield
            self.exit = False
            # Recovery -> U0.
            yield ux.eq(0)
            yield phy.recovery.eq(1)
            self.state = "Recovery"
            for _ in range(self.recovery):
                yield
            yield phy.recovery.eq(0)
            yield phy.u0.eq(1)
            self.state = "U0"
            # Host Advertisements (7.2.4.1.1): LGOOD_n of the last device Header Packet received,
            # LCRD_A-D, Rx Header Buffer Credit index initialized to A.
            host = self.host
            host.urgent.clear()
            host.send(link_command(LGOOD(host.hps[-1][3] if host.hps else 7)))
            for x in range(4):
                host.send(link_command(LCRD(x)))
            host.lcrd = 0
            self.entries += 1
            yield

# Helpers ------------------------------------------------------------------------------------------

def get_status(host):
    return (yield from pyhost_control_in(host, setup_packet(0x80, 0x00, 0, 0, 2)))

def set_feature(host, feature, enable=True):
    return (yield from pyhost_control_out(host, setup_packet(0x00, 0x03 if enable else 0x01, feature, 0, 0)))

def lgo(host, n):
    """Send LGO_Un, returns the response (LAU/LXU), LPMA sent after LAU."""
    n0 = len(host.lcs)
    host.send(link_command(LGO_U(n)))
    yield from host.wait(lambda: any(lc in [LAU(), LXU()] for lc in host.lcs[n0:]))
    response = [lc for lc in host.lcs[n0:] if lc in [LAU(), LXU()]][0]
    if response == LAU():
        host.send(link_command(LPMA()))
    return response

def wait_advertisement(test, host, n0):
    """Device Advertisements after U0 re-entry: LGOOD_n (last host Header Packet), LCRD_A-D."""
    expected = [LGOOD((host.tx_seq - 1) % 8)] + [LCRD(x) for x in range(4)]
    yield from host.wait(lambda: host.lcs[n0:n0 + 5] == expected)

# Tests --------------------------------------------------------------------------------------------

class TestCoreLPM(unittest.TestCase):
    def run_scenario(self, scenario, with_lpm=True):
        dut   = DUT(with_lpm=with_lpm)
        host  = PyHost(dut.phy)
        ltssm = LTSSMEmulator(dut, host)
        def wrapper():
            yield from pyhost_link_init(host)
            yield from scenario(dut, host, ltssm)
        run_simulation(dut, [host.driver(), host.monitor(), ltssm.run(), wrapper()])

    def test_bos_exit_latencies(self):
        # U1/U2 Device Exit Latencies: 0 without LPM (hosts do not enable U1/U2), non-zero with.
        for with_lpm, u1, u2 in [(False, 0, 0), (True, 0x0a, 0x07ff)]:
            bos = bulk_descriptors(with_lpm=with_lpm).bos()
            ss  = bos[5 + 7:]
            self.assertEqual(ss[:3], [10, 0x10, 0x03])
            self.assertEqual((ss[7], ss[8] | (ss[9] << 8)), (u1, u2))
        descriptors = bulk_descriptors()
        USB3DeviceCore(PHYStandIn(), sys_clk_freq=125e6, descriptors=descriptors, with_lpm=True)
        self.assertEqual(descriptors.bos(), bulk_descriptors(with_lpm=True).bos())

    def test_u1_u2_enable_get_status(self):
        # SET/CLEAR_FEATURE(U1_ENABLE/U2_ENABLE) (Device) reported by GET_STATUS (D2/D3, 9.4.5).
        def scenario(dut, host, ltssm):
            self.assertEqual((yield from get_status(host)), [0x00, 0x00])
            self.assertTrue((yield from set_feature(host, 48)))
            self.assertEqual((yield dut.core.u1_enable), 1)
            self.assertEqual((yield from get_status(host)), [0x04, 0x00])
            self.assertTrue((yield from set_feature(host, 49)))
            self.assertEqual((yield from get_status(host)), [0x0c, 0x00])
            self.assertTrue((yield from set_feature(host, 48, enable=False)))
            self.assertEqual((yield from get_status(host)), [0x08, 0x00])
            self.assertEqual(((yield dut.core.u1_enable), (yield dut.core.u2_enable)), (0, 1))
            # GET_DESCRIPTOR still served from the ROM.
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, bulk_descriptors().device())
        self.run_scenario(scenario)

    def test_lgo_rejected_without_lpm(self):
        # Default (without LPM): LGO_U1/LGO_U2 rejected with LXU, not forwarded to the LTSSM.
        def scenario(dut, host, ltssm):
            self.assertTrue((yield from set_feature(host, 48)))
            self.assertTrue((yield from set_feature(host, 49)))
            for n in [1, 2]:
                self.assertEqual((yield from lgo(host, n)), LXU())
            for _ in range(64):
                yield
            self.assertEqual(ltssm.lgos, [])
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, bulk_descriptors().device())
        self.run_scenario(scenario, with_lpm=False)

    def test_u1_u2_host_exit(self):
        # LGO_Ux accepted (LAU), Ux entered on LPMA, host initiated exit through Recovery, U0
        # re-entry Advertisements, then control transfers work.
        def scenario(dut, host, ltssm):
            self.assertTrue((yield from set_feature(host, 48)))
            self.assertTrue((yield from set_feature(host, 49)))
            for n in [1, 2]:
                self.assertEqual((yield from lgo(host, n)), LAU())
                yield from host.wait(lambda: ltssm.state == f"U{n}")
                self.assertEqual(ltssm.lgos[-1], n)
                self.assertEqual(ltssm.lpma, n) # Entered on LPMA.
                self.assertEqual((yield dut.core.ready), 0)
                n0 = len(host.lcs)
                for _ in range(256):
                    yield
                self.assertEqual(len(host.lcs), n0) # Nothing sent in Ux.
                ltssm.exit = True
                yield from host.wait(lambda: ltssm.state == "U0")
                yield from wait_advertisement(self, host, n0)
                self.assertEqual(ltssm.exits[-1], "host")
                data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
                self.assertEqual(data, bulk_descriptors().device())
            self.assertEqual((yield from get_status(host)), [0x0c, 0x00])
        self.run_scenario(scenario)

    def test_lgo_accepted_without_u1_u2_enable(self):
        # U1/U2 Enable only allow device initiated entry (9.4.5): LGO_Ux accepted when idle
        # (7.2.4.2.2) even when not enabled.
        def scenario(dut, host, ltssm):
            self.assertEqual((yield from lgo(host, 1)), LAU())
            yield from host.wait(lambda: ltssm.state == "U1")
            n0 = len(host.lcs)
            ltssm.exit = True
            yield from host.wait(lambda: ltssm.state == "U0")
            yield from wait_advertisement(self, host, n0)
            data = yield from pyhost_control_in(host, setup_packet(0x80, 0x06, 0x0100, 0, 18))
            self.assertEqual(data, bulk_descriptors().device())
        self.run_scenario(scenario)

    def test_device_exit(self):
        # Bulk IN request without data (NRDY), U1 entry, data written: the device exits U1 to send
        # its ERDY (7.2.4.2.7), then the host reads the data.
        payload = list(range(16))
        def scenario(dut, host, ltssm):
            ep = dut.core.endpoints[0]
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            n0 = len(host.hps)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=1, tp_seq=0, pp=1)
            yield from pyhost_wait_tp(host, n0, subtypes=(TP_SUB_NRDY,))
            self.assertEqual((yield from lgo(host, 1)), LAU())
            yield from host.wait(lambda: ltssm.state == "U1")
            for _ in range(256):
                yield
            self.assertEqual(ltssm.state, "U1")
            # Data written to the endpoint.
            n1 = len(host.lcs)
            n2 = len(host.hps)
            for i in range(0, len(payload), 4):
                yield ep.sink.valid.eq(1)
                yield ep.sink.data.eq(int.from_bytes(bytes(payload[i:i+4]), "little"))
                yield ep.sink.be.eq(0b1111)
                yield ep.sink.last.eq(i == len(payload) - 4)
                yield
                while not (yield ep.sink.ready):
                    yield
            yield ep.sink.valid.eq(0)
            yield from host.wait(lambda: ltssm.state == "U0")
            self.assertEqual(ltssm.exits, ["device"])
            yield from wait_advertisement(self, host, n1)
            yield from pyhost_wait_tp(host, n2, subtypes=(TP_SUB_ERDY,))
            packets, _ = yield from pyhost_bulk_in(host, ept=1, count=1, seq=0)
            self.assertEqual(packets, [payload])
        self.run_scenario(scenario)

    def test_lgo_rejected_when_busy(self):
        # DP sent and not acknowledged by the host (ACK TP expected): LGO_U1 rejected (LXU,
        # 7.2.4.2.2), accepted once acknowledged.
        payload = list(range(16))
        def scenario(dut, host, ltssm):
            ep = dut.core.endpoints[0]
            self.assertTrue((yield from pyhost_control_out(host, setup_packet(0x00, 0x09, 1, 0, 0))))
            for i in range(0, len(payload), 4):
                yield ep.sink.valid.eq(1)
                yield ep.sink.data.eq(int.from_bytes(bytes(payload[i:i+4]), "little"))
                yield ep.sink.be.eq(0b1111)
                yield ep.sink.last.eq(i == len(payload) - 4)
                yield
            yield ep.sink.valid.eq(0)
            d0 = len(host.dpps)
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=1, tp_seq=0, pp=1)
            yield from host.wait(lambda: len(host.dpps) > d0)
            self.assertEqual(host.dpps[d0][0], payload)
            for _ in range(64):
                yield
            self.assertEqual((yield from lgo(host, 1)), LXU())
            self.assertEqual(ltssm.lgos, [])
            # Acknowledged (window closed): accepted.
            host.send_hp(tp, TP_SUB_ACK, ept=1, direction=1, nump=0, tp_seq=1)
            for _ in range(64):
                yield
            self.assertEqual((yield from lgo(host, 1)), LAU())
            yield from host.wait(lambda: ltssm.state == "U1")
        self.run_scenario(scenario)

    def test_u2_inactivity_timeout_lmp(self):
        # U2 Inactivity Timeout LMP (8.4.3): value forwarded to the LTSSM (U1 -> U2 timeout).
        def scenario(dut, host, ltssm):
            self.assertEqual((yield dut.phy.u2_timeout), 0)
            n0 = len(host.lcs)
            host.send_hp(lmp_u2_inactivity_timeout, 0x10)
            yield from host.wait(lambda: LGOOD((host.tx_seq - 1) % 8) in host.lcs[n0:])
            for _ in range(16):
                yield
            self.assertEqual((yield dut.phy.u2_timeout), 0x10)
        self.run_scenario(scenario)

if __name__ == "__main__":
    unittest.main()
