#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# LTSSM Link Power Management coverage (USB 3.2 Rev 1.1):
# - 7.2.4.2   LGO_Ux / LAU / LXU / LPMA handshake, PM_LC_TIMER (4us), PM_ENTRY_TIMER (8us),
#             Ux_EXIT_TIMER (6ms), U3 entry/wakeup retries (Table 7-13).
# - 7.5.7-9   U1 / U2 / U3 states and their exits (Table 7-12 timeouts).
#
# Simulation speed: migen simulates the LTSSM at a few k cycles/s, so the sys_clk_freq and
# timer_scale are chosen per test to bring the timers involved down to tens/hundreds of cycles
# while keeping their relative order.

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM
from usb3_pipe.lfps import (U1ExitLFPSBurst, U2ExitLFPSBurst, U3WakeupLFPSBurst,
    exit_burst_cycles, exit_u1_min_cycles, exit_u2_min_cycles, exit_u3_min_cycles)

from test import test_ltssm
from test.test_ltssm_link import LTSSMLinkHarness

# Configurations -----------------------------------------------------------------------------------

# PM handshake: unscaled timers, PM_LC_TIMER = 8 cycles, PM_ENTRY_TIMER = 16 cycles.
PM_CLK_FREQ   = int(2e6)
PM_LC_CYCLES    = int(4e-6*PM_CLK_FREQ)
PM_ENTRY_CYCLES = int(8e-6*PM_CLK_FREQ)

# Ux exit: Ux_EXIT_TIMER = 600 cycles, tNoLFPSResponseTimeout U1/U2 = 200, U3 = 1000.
EXIT_CLK_FREQ = int(1e6)
EXIT_SCALE    = 10

# Long timers (tU3WakeupRetryDelay = 1000 cycles, tU1PingTimeout = 3000 cycles).
LONG_CLK_FREQ = int(1e6)
LONG_SCALE    = 100

def scaled(t, sys_clk_freq, timer_scale=1):
    # Same convention as USB3LTSSM.
    return max(1, int(t*sys_clk_freq)//timer_scale)

def burst_cycles(kind, sys_clk_freq, timer_scale):
    timing, min_cycles = {
        1: (U1ExitLFPSBurst,   exit_u1_min_cycles),
        2: (U2ExitLFPSBurst,   exit_u2_min_cycles),
        3: (U3WakeupLFPSBurst, exit_u3_min_cycles),
    }[kind]
    return exit_burst_cycles(sys_clk_freq, timer_scale, timing, min_cycles(sys_clk_freq, timer_scale))

# Helpers ------------------------------------------------------------------------------------------

class PMTestCase(unittest.TestCase):
    wait_for    = test_ltssm.TestLTSSM.wait_for
    drive_to_u0 = test_ltssm.TestLTSSM.drive_to_u0

    def make_dut(self, sys_clk_freq=PM_CLK_FREQ, timer_scale=1, with_timers=False, **kwargs):
        return USB3LTSSM(sys_clk_freq=sys_clk_freq, with_timers=with_timers,
            timer_scale=timer_scale, **kwargs)

    def go_u0(self, dut, ds_port=0):
        yield dut.ds_port.eq(ds_port)
        yield from self.drive_to_u0(dut)
        yield dut.lfps_rx_polling.eq(0) # Partner Polling.LFPS ended (U1/U2/U3 escape otherwise).
        yield
        self.assertEqual((yield dut.u0), 1)

    def pulse(self, signal):
        yield signal.eq(1)
        yield
        yield signal.eq(0)

    def count_pulses(self, dut, signal, cycles):
        n = 0
        for _ in range(cycles):
            n += (yield signal)
            yield
        return n

    def ux_signal(self, dut, kind):
        return {1: dut.u1, 2: dut.u2, 3: dut.u3}[kind]

    def lgo_in(self, dut, kind):
        return {1: dut.lgo_u1, 2: dut.lgo_u2, 3: dut.lgo_u3}[kind]

    def lgo_out(self, dut, kind):
        return {1: dut.tx_lgo_u1, 2: dut.tx_lgo_u2, 3: dut.tx_lgo_u3}[kind]

    def req(self, dut, kind):
        return {1: dut.req_u1, 2: dut.req_u2, 3: dut.req_u3}[kind]

    def enter_ux_as_acceptor(self, dut, kind, timeout=256):
        # Partner LGO_Ux -> LAU, Ux on PM_ENTRY_TIMER timeout (7.2.4.2.3).
        yield from self.pulse(self.lgo_in(dut, kind))
        self.assertTrue((yield from self.wait_for(dut, self.ux_signal(dut, kind), timeout=timeout)))

    def recover_to_u0(self, dut, timeout=64):
        # Recovery.Active (TS1) -> Recovery.Configuration (TS2) -> Recovery.Idle -> U0.
        self.assertTrue((yield from self.wait_for(dut, dut.ts_tx_ts1, timeout=timeout)))
        self.assertEqual((yield dut.recovery), 1)
        yield from self.pulse(dut.ts_rx_ts1)
        yield
        yield from self.pulse(dut.ts_tx_done)
        self.assertTrue((yield from self.wait_for(dut, dut.ts_tx_ts2, timeout=timeout)))
        yield from self.pulse(dut.ts_rx_ts2)
        yield
        for _ in range(2):
            yield from self.pulse(dut.ts_tx_done)
            yield
        self.assertTrue((yield from self.wait_for(dut, dut.u0, timeout=timeout)))

    def partner_burst(self, dut, signal, cycles):
        # Partner LFPS burst (level as seen by the LFPS checkers).
        yield signal.eq(1)
        for _ in range(cycles):
            yield
        yield signal.eq(0)

    def lfps_tx(self, dut, kind):
        return {1: dut.lfps_tx_exit_u1, 2: dut.lfps_tx_exit_u2, 3: dut.lfps_tx_wakeup_u3}[kind]

    def lfps_rx(self, dut, kind):
        return {1: dut.lfps_rx_exit_u1, 2: dut.lfps_rx_exit_u2, 3: dut.lfps_rx_wakeup_u3}[kind]

    def exit_req(self, dut, kind):
        return {1: dut.exit_u1_req, 2: dut.exit_u2_req, 3: dut.wakeup_u3_req}[kind]

# PM Entry: Initiator (7.2.4.2.2/7.2.4.2.3) --------------------------------------------------------

class TestLTSSMPMInitiator(PMTestCase):
    def test_lau_enters_ux(self):
        # Requester sends LGO_Ux, on LAU sends LPMA and enters Ux (7.2.4.2.3).
        for kind in [1, 2, 3]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield self.req(dut, kind).eq(1)
                    self.assertTrue((yield from self.wait_for(dut, self.lgo_out(dut, kind), timeout=4)))
                    yield self.req(dut, kind).eq(0)
                    # Only the requested LGO, a single pulse.
                    for other in {1, 2, 3} - {kind}:
                        self.assertEqual((yield self.lgo_out(dut, other)), 0)
                    yield
                    self.assertEqual((yield self.lgo_out(dut, kind)), 0)
                    self.assertEqual((yield dut.pm_sent_lgo), 1)
                    self.assertEqual((yield dut.u0), 1)
                    # LAU before PM_LC_TIMER.
                    yield dut.lau.eq(1)
                    yield
                    yield dut.lau.eq(0)
                    lpma = (yield dut.tx_lpma)
                    yield
                    lpma |= (yield dut.tx_lpma)
                    self.assertEqual(lpma, 1)
                    self.assertEqual((yield self.ux_signal(dut, kind)), 1)
                    self.assertEqual((yield dut.u0), 0)
                    self.assertEqual((yield dut.recovery), 0)
                    # Ux is an electrical idle state.
                    self.assertEqual((yield dut.lfps_tx_idle), 1)
                    self.assertEqual((yield dut.tx_ready), 0)

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

    def test_lxu_stays_u0(self):
        # Rejected request (LXU): remain in U0, no LPMA, a new request is possible (7.2.4.2.3).
        for kind in [1, 2, 3]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield from self.pulse(self.req(dut, kind))
                    self.assertTrue((yield from self.wait_for(dut, dut.pm_sent_lgo, timeout=4)))
                    yield from self.pulse(dut.lxu)
                    for _ in range(4*PM_LC_CYCLES):
                        self.assertEqual((yield dut.u0), 1)
                        self.assertEqual((yield dut.tx_lpma), 0)
                        yield
                    self.assertEqual((yield dut.pm_sent_lgo), 0)
                    self.assertEqual((yield dut.recovery), 0)
                    # New request accepted.
                    yield from self.pulse(self.req(dut, kind))
                    self.assertTrue((yield from self.wait_for(dut, self.lgo_out(dut, kind), timeout=4)))
                    yield from self.pulse(dut.lau)
                    self.assertTrue((yield from self.wait_for(dut, self.ux_signal(dut, kind), timeout=4)))

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

    def test_pm_lc_timer_timeout_recovery(self):
        # No LAU/LXU within PM_LC_TIMER (4us) -> Recovery (7.2.4.2.3), then back to U0.
        for kind in [1, 2, 3]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield self.req(dut, kind).eq(1)
                    self.assertTrue((yield from self.wait_for(dut, self.lgo_out(dut, kind), timeout=4)))
                    yield self.req(dut, kind).eq(0)
                    for _ in range(PM_LC_CYCLES - 2):
                        yield
                        self.assertEqual((yield dut.u0), 1)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4)))
                    self.assertEqual((yield self.ux_signal(dut, kind)), 0)
                    self.assertEqual((yield dut.pm_sent_lgo), 0)
                    yield from self.recover_to_u0(dut)

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

    def test_late_lau_after_pm_lc_timer_ignored(self):
        # An LAU after PM_LC_TIMER expiry must not take the port to Ux (it is in Recovery).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.pulse(dut.req_u1)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=2*PM_LC_CYCLES)))
            yield from self.pulse(dut.lau)
            for _ in range(8):
                self.assertEqual((yield dut.u1), 0)
                self.assertEqual((yield dut.tx_lpma), 0)
                yield

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_ufp_cannot_request_u3(self):
        # LGO_U3 is only issued by a downstream port (7.2.4.2.4).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield dut.req_u3.eq(1)
            for _ in range(2*PM_LC_CYCLES):
                self.assertEqual((yield dut.tx_lgo_u3), 0)
                self.assertEqual((yield dut.pm_sent_lgo), 0)
                self.assertEqual((yield dut.u0), 1)
                yield

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_u3_entry_three_failures_ess_inactive(self):
        # DS LGO_U3 PM_LC_TIMER timeouts: Recovery and retry, 3rd consecutive failure ->
        # eSS.Inactive (7.2.4.2.4).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            for attempt in range(3):
                yield from self.pulse(dut.req_u3)
                self.assertTrue((yield from self.wait_for(dut, dut.tx_lgo_u3, timeout=4)))
                if attempt < 2:
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=2*PM_LC_CYCLES)))
                    self.assertEqual((yield dut.ess_inactive), 0)
                    yield from self.recover_to_u0(dut)
                else:
                    self.assertTrue((yield from self.wait_for(dut, dut.ess_inactive, timeout=2*PM_LC_CYCLES)))
                    self.assertEqual((yield dut.u3), 0)

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_u3_entry_failure_streak_reset(self):
        # The failure count is for consecutive failures: a resolved handshake (LXU/LAU) or a
        # U1/U2 PM_LC_TIMER failure does not contribute.
        for breaker in ["lxu", "lau"]:
            with self.subTest(breaker=breaker):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    def fail_u3():
                        yield from self.pulse(dut.req_u3)
                        self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=2*PM_LC_CYCLES)))
                        yield from self.recover_to_u0(dut)
                    # 2 failures.
                    yield from fail_u3()
                    yield from fail_u3()
                    # Resolved handshake.
                    yield from self.pulse(dut.req_u3)
                    self.assertTrue((yield from self.wait_for(dut, dut.pm_sent_lgo, timeout=4)))
                    yield from self.pulse(getattr(dut, breaker))
                    if breaker == "lau":
                        self.assertTrue((yield from self.wait_for(dut, dut.u3, timeout=4)))
                        # DS wakeup, partner responds.
                        yield from self.pulse(dut.wakeup_u3_req)
                        self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_wakeup_u3, timeout=4)))
                        yield from self.partner_burst(dut, dut.lfps_rx_wakeup_u3,
                            burst_cycles(3, PM_CLK_FREQ, 1))
                        yield from self.recover_to_u0(dut, timeout=4*burst_cycles(3, PM_CLK_FREQ, 1))
                    else:
                        for _ in range(4):
                            yield
                        self.assertEqual((yield dut.u0), 1)
                    # 2 new failures: still Recovery (not eSS.Inactive).
                    yield from fail_u3()
                    yield from fail_u3()
                    self.assertEqual((yield dut.u0), 1)

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

# PM Entry: Acceptor (7.2.4.2.3) -------------------------------------------------------------------

class TestLTSSMPMAcceptor(PMTestCase):
    def test_lgo_lau_enters_ux_on_pm_entry_timer(self):
        # LGO_Ux received -> LAU sent, Ux entered on PM_ENTRY_TIMER (8us) timeout.
        for ds_port, kind in [(0, 1), (0, 2), (0, 3), (1, 1), (1, 2)]:
            with self.subTest(ds_port=ds_port, kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=ds_port)
                    yield from self.pulse(self.lgo_in(dut, kind))
                    lau = 0
                    for _ in range(PM_ENTRY_CYCLES - 2):
                        lau += (yield dut.tx_lau)
                        self.assertEqual((yield dut.tx_lxu), 0)
                        self.assertEqual((yield dut.tx_lpma), 0)
                        self.assertEqual((yield dut.u0), 1)
                        yield
                    self.assertEqual(lau, 1) # single LAU pulse.
                    self.assertTrue((yield from self.wait_for(dut, self.ux_signal(dut, kind), timeout=6)))
                    self.assertEqual((yield dut.recovery), 0)
                    self.assertEqual((yield dut.lfps_tx_idle), 1)

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

    def test_ts1_during_pm_entry_timer_recovery(self):
        # TS1 received while PM_ENTRY_TIMER runs (LAU lost, requester went to Recovery) ->
        # Recovery (7.2.4.2.3).
        for kind in [1, 2, 3]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield from self.pulse(self.lgo_in(dut, kind))
                    for _ in range(PM_ENTRY_CYCLES//2):
                        yield
                    yield from self.pulse(dut.ts_rx_ts1)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4)))
                    for _ in range(2*PM_ENTRY_CYCLES):
                        self.assertEqual((yield self.ux_signal(dut, kind)), 0)
                        yield
                    yield from self.recover_to_u0(dut)

                dut = self.make_dut()
                run_simulation(dut, generator(dut))

    def test_lgo_lau_enters_ux_on_lpma(self):
        # LGO_Ux received -> LAU sent, Ux entered on LPMA reception, before the PM_ENTRY_TIMER
        # timeout (7.2.4.2.3).
        sys_clk_freq = int(20e6)
        pm_entry     = scaled(8e-6, sys_clk_freq)
        for kind in [1, 2]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield from self.pulse(self.lgo_in(dut, kind))
                    self.assertTrue((yield from self.wait_for(dut, dut.tx_lau, timeout=8)))
                    for _ in range(8):
                        yield
                    self.assertEqual((yield dut.u0), 1)
                    yield from self.pulse(dut.lpma)
                    self.assertTrue((yield from self.wait_for(dut, self.ux_signal(dut, kind), timeout=4)))
                    self.assertEqual((yield dut.recovery), 0)

                dut = self.make_dut(sys_clk_freq=sys_clk_freq, with_timers=True)
                self.assertGreater(pm_entry, 64)
                run_simulation(dut, generator(dut))

    def test_lgo_ignored_while_acceptor_pending(self):
        # A second LGO while PM_ENTRY_TIMER runs does not produce a second LAU nor change Ux.
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.pulse(dut.lgo_u1)
            yield
            yield
            yield from self.pulse(dut.lgo_u2)
            n = yield from self.count_pulses(dut, dut.tx_lau, 4)
            self.assertEqual(n, 0)
            self.assertTrue((yield from self.wait_for(dut, dut.u1, timeout=2*PM_ENTRY_CYCLES)))

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_ufp_never_rejects_lgo_u3(self):
        # An upstream port shall not reject LGO_U3 (7.2.4.2.4).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.pulse(dut.lgo_u3)
            self.assertTrue((yield from self.wait_for(dut, dut.tx_lau, timeout=4)))
            self.assertTrue((yield from self.wait_for(dut, dut.u3, timeout=2*PM_ENTRY_CYCLES)))

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

# PM Concurrency (7.2.4.2.5) -----------------------------------------------------------------------

class TestLTSSMPMConcurrency(PMTestCase):
    def test_concurrent_lgo_rejected(self):
        # Our LGO pending + partner LGO received: partner request rejected (LXU), no Ux entry
        # on the partner request.
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.pulse(dut.req_u1)
            self.assertTrue((yield from self.wait_for(dut, dut.pm_sent_lgo, timeout=4)))
            yield from self.pulse(dut.lgo_u2)
            self.assertTrue((yield from self.wait_for(dut, dut.tx_lxu, timeout=4)))
            self.assertEqual((yield dut.tx_lau), 0)
            for _ in range(2*PM_ENTRY_CYCLES):
                self.assertEqual((yield dut.u2), 0)
                yield

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_ds_concurrent_lgo_keeps_own_request(self):
        # 7.2.4.2.5: the downstream port rejects the upstream port LGO_Ux (LXU) but its own
        # LGO_Uy stays outstanding: the upstream port answers it (LAU -> DS enters Uy).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.pulse(dut.req_u1)
            self.assertTrue((yield from self.wait_for(dut, dut.pm_sent_lgo, timeout=4)))
            yield from self.pulse(dut.lgo_u2)
            self.assertTrue((yield from self.wait_for(dut, dut.tx_lxu, timeout=4)))
            self.assertEqual((yield dut.pm_sent_lgo), 1)
            yield from self.pulse(dut.lau)
            self.assertTrue((yield from self.wait_for(dut, dut.u1, timeout=4)))

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_ufp_concurrent_lgo_u3_accepted(self):
        # 7.2.4.2.4/7.2.4.2.5: an upstream port with its own LGO_Ux outstanding that receives an
        # LGO_U3 shall wait until the reception of an LXU (the DS rejecting its request) and then send
        # an LAU (an upstream port shall not reject U3) -> U3.
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.pulse(dut.req_u1)
            self.assertTrue((yield from self.wait_for(dut, dut.pm_sent_lgo, timeout=4)))
            yield from self.pulse(dut.lgo_u3)
            # No answer before the DS LXU.
            for _ in range(4):
                self.assertEqual((yield dut.tx_lau), 0)
                self.assertEqual((yield dut.tx_lxu), 0)
                yield
            yield from self.pulse(dut.lxu)
            lau = lxu = 0
            for _ in range(4):
                lau += (yield dut.tx_lau)
                lxu += (yield dut.tx_lxu)
                yield
            self.assertEqual(lxu, 0)
            self.assertEqual(lau, 1)
            self.assertTrue((yield from self.wait_for(dut, dut.u3, timeout=4*PM_ENTRY_CYCLES)))

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

# Ux Exit (6.9.2, 7.2.4.2.7, 7.5.7.2/7.5.8.2/7.5.9.2) ------------------------------------------------

class TestLTSSMUxExit(PMTestCase):
    def make_exit_dut(self, **kwargs):
        return self.make_dut(sys_clk_freq=EXIT_CLK_FREQ, timer_scale=EXIT_SCALE, **kwargs)

    def test_local_exit_handshake_to_u0(self):
        # Local exit/wakeup: own LFPS until the partner response is seen -> Recovery -> U0.
        for kind, ds_port in [(1, 0), (1, 1), (2, 0), (2, 1), (3, 1)]:
            with self.subTest(kind=kind, ds_port=ds_port):
                burst = burst_cycles(kind, EXIT_CLK_FREQ, EXIT_SCALE)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=ds_port)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    for _ in range(8): # U1_MIN_RESIDENCY (UFP).
                        yield
                    yield from self.pulse(self.exit_req(dut, kind))
                    self.assertTrue((yield from self.wait_for(dut, self.lfps_tx(dut, kind), timeout=4)))
                    # Late partner response: own LFPS still ongoing.
                    for _ in range(2*burst):
                        yield
                    self.assertEqual((yield self.lfps_tx(dut, kind)), 1)
                    self.assertEqual((yield self.ux_signal(dut, kind)), 1)
                    yield from self.partner_burst(dut, self.lfps_rx(dut, kind), burst)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
                    self.assertEqual((yield self.lfps_tx(dut, kind)), 0)
                    yield from self.recover_to_u0(dut)
                    self.assertEqual((yield dut.rx_ready), 1)

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_partner_exit_handshake_to_u0(self):
        # Partner exit/wakeup LFPS: respond with LFPS -> Recovery.Active -> U0.
        for kind, ds_port in [(1, 0), (1, 1), (2, 0), (2, 1), (3, 0)]:
            with self.subTest(kind=kind, ds_port=ds_port):
                burst = burst_cycles(kind, EXIT_CLK_FREQ, EXIT_SCALE)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=ds_port)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    yield self.lfps_rx(dut, kind).eq(1)
                    self.assertTrue((yield from self.wait_for(dut, self.lfps_tx(dut, kind), timeout=8)))
                    yield self.lfps_rx(dut, kind).eq(0)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
                    yield from self.recover_to_u0(dut)

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_partner_u0_transmission_not_exit_request(self):
        # Ux entered while the partner is still seen transmitting (burst checkers measuring its U0
        # transmission, ex: U1/U2 entered on LPMA reception): not an exit request, a burst started
        # after the partner's electrical idle is.
        for kind in [1, 2, 3]:
            with self.subTest(kind=kind):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield self.lfps_rx(dut, kind).eq(1)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    for _ in range(32):
                        self.assertEqual((yield self.ux_signal(dut, kind)), 1)
                        self.assertEqual((yield self.lfps_tx(dut, kind)), 0)
                        yield
                    yield self.lfps_rx(dut, kind).eq(0)
                    for _ in range(4):
                        yield
                    yield from self.partner_burst(dut, self.lfps_rx(dut, kind), 4)
                    self.assertTrue((yield from self.wait_for(dut, self.lfps_tx(dut, kind), timeout=8)))

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_wrong_exit_lfps_class_ignored(self):
        # Each Ux only responds to its own exit/wakeup LFPS class.
        for kind, other in [(1, 3), (2, 3), (3, 1), (3, 2)]:
            with self.subTest(kind=kind, other=other):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    yield from self.partner_burst(dut, self.lfps_rx(dut, other), 32)
                    for _ in range(32):
                        self.assertEqual((yield self.ux_signal(dut, kind)), 1)
                        for k in [1, 2, 3]:
                            self.assertEqual((yield self.lfps_tx(dut, k)), 0)
                        yield

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_ufp_u1_exit_waits_min_residency(self):
        # UFP: U1 exit not initiated before U1_MIN_RESIDENCY (3us, 7.2.4.2.7).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.enter_ux_as_acceptor(dut, 1)
            yield dut.exit_u1_req.eq(1)
            residency = scaled(3e-6, PM_CLK_FREQ)
            for _ in range(residency - 2):
                self.assertEqual((yield dut.lfps_tx_exit_u1), 0)
                yield
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_exit_u1, timeout=6)))

        dut = self.make_dut()
        run_simulation(dut, generator(dut))

    def test_u1_u2_exit_no_response_ess_inactive(self):
        # tNoLFPSResponseTimeout (2ms) without partner response -> eSS.Inactive (Table 7-12).
        for kind in [1, 2]:
            with self.subTest(kind=kind):
                timeout = scaled(2e-3, EXIT_CLK_FREQ, EXIT_SCALE)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    yield from self.pulse(self.exit_req(dut, kind))
                    for _ in range(timeout - 2):
                        self.assertEqual((yield dut.ess_inactive), 0)
                        yield
                    self.assertTrue((yield from self.wait_for(dut, dut.ess_inactive, timeout=6)))

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_ux_exit_timer_timeout_ess_inactive(self):
        # Ux_EXIT_TIMER (6ms) started with the U1/U2 exit LFPS: U0 not reached in time ->
        # eSS.Inactive (7.2.4.2.1). Recovery timers disabled (with_timers=False) so only
        # Ux_EXIT_TIMER can fire.
        for kind, initiator in [(1, True), (2, True)]:
            with self.subTest(kind=kind, initiator=initiator):
                ux_exit = scaled(6e-3, EXIT_CLK_FREQ, EXIT_SCALE)
                burst   = burst_cycles(kind, EXIT_CLK_FREQ, EXIT_SCALE)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    if initiator:
                        yield from self.pulse(self.exit_req(dut, kind))
                        self.assertTrue((yield from self.wait_for(dut, self.lfps_tx(dut, kind), timeout=4)))
                        yield from self.partner_burst(dut, self.lfps_rx(dut, kind), burst)
                    else:
                        yield from self.partner_burst(dut, self.lfps_rx(dut, kind), burst)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
                    # Partner never answers TS1: stuck in Recovery.Active.
                    for _ in range(ux_exit - 6*burst):
                        self.assertEqual((yield dut.ess_inactive), 0)
                        yield
                    self.assertTrue((yield from self.wait_for(dut, dut.ess_inactive, timeout=8*burst)))

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_ux_exit_timer_not_applied_to_responder(self):
        # 7.2.4.2.1/7.2.4.2.7: Ux_EXIT_TIMER is only started by the port initiating the U1/U2
        # exit; a responder stuck in Recovery.Active stays there (until its own Recovery timeouts,
        # disabled here).
        for kind in [1, 2]:
            with self.subTest(kind=kind):
                ux_exit = scaled(6e-3, EXIT_CLK_FREQ, EXIT_SCALE)
                burst   = burst_cycles(kind, EXIT_CLK_FREQ, EXIT_SCALE)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=1)
                    yield from self.enter_ux_as_acceptor(dut, kind)
                    yield from self.partner_burst(dut, self.lfps_rx(dut, kind), burst)
                    self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
                    for _ in range(ux_exit + 8*burst):
                        self.assertEqual((yield dut.ess_inactive), 0)
                        yield

                dut = self.make_exit_dut()
                run_simulation(dut, generator(dut))

    def test_ux_exit_timer_disarmed_in_u0(self):
        # Successful exit to U0 stops the Ux_EXIT_TIMER: a later Recovery is not cut short.
        ux_exit = scaled(6e-3, EXIT_CLK_FREQ, EXIT_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 1)
            yield from self.partner_burst(dut, dut.lfps_rx_exit_u1, 16)
            yield from self.recover_to_u0(dut)
            yield from self.pulse(dut.directed_recovery)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4)))
            for _ in range(ux_exit + 32):
                self.assertEqual((yield dut.ess_inactive), 0)
                yield

        dut = self.make_exit_dut()
        run_simulation(dut, generator(dut))

    def test_u3_wakeup_not_bounded_by_ux_exit_timer(self):
        # Ux_EXIT_TIMER only applies to U1/U2 exit (7.2.4.2.1): a U3 wakeup Recovery longer
        # than 6ms is not cut short (Recovery timers disabled).
        ux_exit = scaled(6e-3, EXIT_CLK_FREQ, EXIT_SCALE)
        burst   = burst_cycles(3, EXIT_CLK_FREQ, EXIT_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 3, timeout=4)
            yield from self.pulse(dut.wakeup_u3_req)
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_wakeup_u3, timeout=4)))
            yield from self.partner_burst(dut, dut.lfps_rx_wakeup_u3, burst)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
            for _ in range(ux_exit + 32):
                self.assertEqual((yield dut.ess_inactive), 0)
                yield

        dut = self.make_exit_dut()
        run_simulation(dut, generator(dut))

    def test_ufp_u3_remote_wakeup(self):
        # 7.5.9.2 / 7.2.4.2.7: either port may initiate the U3 wakeup handshake (upstream port:
        # function remote wakeup).
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.enter_ux_as_acceptor(dut, 3)
            yield from self.pulse(dut.wakeup_u3_req)
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_wakeup_u3, timeout=8)))

        dut = self.make_exit_dut()
        run_simulation(dut, generator(dut))

# U3 Wakeup retries (7.2.4.2.4, tU3WakeupRetryDelay) ------------------------------------------------

class TestLTSSMU3WakeupRetry(PMTestCase):
    def test_ds_wakeup_retries_stays_u3(self):
        # 7.2.4.2.7: a port initiating U3 wakeup shall remain in U3 when the LFPS handshake fails
        # (tNoLFPSResponseTimeout) and may initiate it again after tU3WakeupRetryDelay (100ms):
        # no failure limit (the 3 consecutive failures -> eSS.Inactive rule is for U3 entry,
        # 7.2.4.2.4).
        guard = scaled(10e-3,  LONG_CLK_FREQ, LONG_SCALE)
        delay = scaled(100e-3, LONG_CLK_FREQ, LONG_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 3, timeout=4)
            yield from self.pulse(dut.wakeup_u3_req)
            starts = []
            prev   = 0
            for cycle in range(4*(guard + delay) + 64):
                self.assertEqual((yield dut.ess_inactive), 0)
                tx = (yield dut.lfps_tx_wakeup_u3)
                if tx and not prev:
                    starts.append(cycle)
                if not tx and cycle > 2:
                    # Between attempts the port stays in U3 (electrical idle).
                    self.assertEqual((yield dut.u3), 1)
                prev = tx
                yield
            self.assertGreaterEqual(len(starts), 1)
            for a, b in zip(starts, starts[1:]):
                self.assertGreaterEqual(b - a, guard + delay - 4)
            self.assertEqual((yield dut.u3), 1)

        dut = self.make_dut(sys_clk_freq=LONG_CLK_FREQ, timer_scale=LONG_SCALE)
        run_simulation(dut, generator(dut))

    def test_ds_wakeup_retry_succeeds(self):
        # A partner response on the 2nd attempt completes the wakeup -> Recovery -> U0.
        guard = scaled(10e-3,  LONG_CLK_FREQ, LONG_SCALE)
        delay = scaled(100e-3, LONG_CLK_FREQ, LONG_SCALE)
        burst = burst_cycles(3, LONG_CLK_FREQ, LONG_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 3, timeout=4)
            yield from self.pulse(dut.wakeup_u3_req)
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_wakeup_u3, timeout=4)))
            # 1st attempt fails.
            for _ in range(guard + 4):
                yield
            self.assertEqual((yield dut.u3), 1)
            self.assertEqual((yield dut.lfps_tx_wakeup_u3), 0)
            # 2nd attempt after tU3WakeupRetryDelay.
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_wakeup_u3, timeout=delay + 8)))
            yield from self.partner_burst(dut, dut.lfps_rx_wakeup_u3, burst)
            self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=4*burst)))
            yield from self.recover_to_u0(dut)

        dut = self.make_dut(sys_clk_freq=LONG_CLK_FREQ, timer_scale=LONG_SCALE)
        run_simulation(dut, generator(dut))

# U1 timers (7.5.7.2): U2 inactivity, DS Ping timeout ----------------------------------------------

class TestLTSSMU1Timers(PMTestCase):
    def test_u1_to_u2_on_inactivity_timeout(self):
        # PORT_U2_TIMEOUT expiry in U1 -> U2 directly (7.2.4.2.3, 7.5.7.2), U2 exit afterwards.
        for enable in [0, 1]:
            with self.subTest(enable=enable):
                inact = scaled(256e-6, PM_CLK_FREQ)
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield dut.u2_timeout.eq(1 if enable else 0)
                    yield from self.enter_ux_as_acceptor(dut, 1)
                    for _ in range(inact - 2):
                        self.assertEqual((yield dut.u1), 1)
                        yield
                    if enable:
                        self.assertTrue((yield from self.wait_for(dut, dut.u2, timeout=6)))
                        self.assertEqual((yield dut.lfps_tx_idle), 1)
                        # U2 exit then works as usual.
                        yield from self.partner_burst(dut, dut.lfps_rx_exit_u2,
                            burst_cycles(2, PM_CLK_FREQ, 1))
                        self.assertTrue((yield from self.wait_for(dut, dut.recovery, timeout=1024)))
                    else:
                        for _ in range(2*inact):
                            self.assertEqual((yield dut.u1), 1)
                            yield

                dut = self.make_dut(with_timers=True)
                run_simulation(dut, generator(dut))

    def test_u1_to_u2_on_u2_timeout(self):
        # U2 Inactivity Timeout (U2 Inactivity Timeout LMP value, 256us units, 00h/FFh: disabled)
        # expiry in U1 -> U2 directly (10.6.2.4, Table 10-17).
        tick = scaled(256e-6, PM_CLK_FREQ)
        for u2_timeout in [0x00, 0x02, 0xff]:
            with self.subTest(u2_timeout=u2_timeout):
                def generator(dut):
                    yield from self.go_u0(dut, ds_port=0)
                    yield dut.u2_timeout.eq(u2_timeout)
                    yield from self.enter_ux_as_acceptor(dut, 1)
                    for _ in range(2*tick - 4):
                        self.assertEqual((yield dut.u1), 1)
                        yield
                    if u2_timeout == 0x02:
                        self.assertTrue((yield from self.wait_for(dut, dut.u2, timeout=8)))
                    else:
                        for _ in range(4*tick):
                            self.assertEqual((yield dut.u1), 1)
                            yield

                dut = self.make_dut(with_timers=True)
                run_simulation(dut, generator(dut))

    def test_ds_u1_ping_timeout_rx_detect(self):
        # DS in U1 without Ping.LFPS for tU1PingTimeout (300ms) -> Rx.Detect (7.5.7.2).
        ping_timeout = scaled(300e-3, LONG_CLK_FREQ, LONG_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 1, timeout=4)
            for _ in range(ping_timeout - 2):
                self.assertEqual((yield dut.u1), 1)
                yield
            self.assertTrue((yield from self.wait_for(dut, dut.exit_to_rx_detect, timeout=6)))

        dut = self.make_dut(sys_clk_freq=LONG_CLK_FREQ, timer_scale=LONG_SCALE)
        run_simulation(dut, generator(dut))

    def test_ds_u1_ping_received_stays_u1(self):
        # Periodic Ping.LFPS (tU1Ping < tU1PingTimeout) keeps the DS in U1.
        ping_timeout = scaled(300e-3, LONG_CLK_FREQ, LONG_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=1)
            yield from self.enter_ux_as_acceptor(dut, 1, timeout=4)
            for _ in range(3):
                for _ in range(ping_timeout*2//3):
                    self.assertEqual((yield dut.u1), 1)
                    yield
                yield from self.pulse(dut.lfps_rx_ping)
            self.assertEqual((yield dut.u1), 1)

        dut = self.make_dut(sys_clk_freq=LONG_CLK_FREQ, timer_scale=LONG_SCALE)
        run_simulation(dut, generator(dut))

    def test_ufp_u1_sends_ping_no_timeout(self):
        # UFP in U1: transmits Ping.LFPS (after U1_MIN_RESIDENCY) and has no Ping timeout.
        ping_timeout = scaled(300e-3, LONG_CLK_FREQ, LONG_SCALE)
        def generator(dut):
            yield from self.go_u0(dut, ds_port=0)
            yield from self.enter_ux_as_acceptor(dut, 1, timeout=4)
            self.assertTrue((yield from self.wait_for(dut, dut.lfps_tx_ping, timeout=8)))
            for _ in range(ping_timeout + 16):
                self.assertEqual((yield dut.u1), 1)
                yield

        dut = self.make_dut(sys_clk_freq=LONG_CLK_FREQ, timer_scale=LONG_SCALE)
        run_simulation(dut, generator(dut))

# Two LTSSMs back-to-back ---------------------------------------------------------------------------

class LTSSMPMLinkHarness(LTSSMLinkHarness):
    """LTSSMLinkHarness + LCMD (LGO/LAU/LXU) and exit/wakeup/Ping LFPS cross connections.

    host is the DS port, dev the UFP. LCMDs are delivered with 1 cycle of latency, LFPS as
    levels (checker latency not modelled)."""
    def __init__(self, sys_clk_freq=PM_CLK_FREQ):
        LTSSMLinkHarness.__init__(self, sys_clk_freq=sys_clk_freq)
        host, dev = self.host, self.dev
        self.comb += host.ds_port.eq(1)
        for a, b in [(host, dev), (dev, host)]:
            self.sync += [
                b.lgo_u1.eq(a.tx_lgo_u1),
                b.lgo_u2.eq(a.tx_lgo_u2),
                b.lgo_u3.eq(a.tx_lgo_u3),
                b.lau.eq(a.tx_lau),
                b.lxu.eq(a.tx_lxu),
            ]
            self.comb += [
                b.lfps_rx_exit_u1.eq(a.lfps_tx_exit_u1),
                b.lfps_rx_exit_u2.eq(a.lfps_tx_exit_u2),
                b.lfps_rx_wakeup_u3.eq(a.lfps_tx_wakeup_u3),
                b.lfps_rx_ping.eq(a.lfps_tx_ping),
            ]


class TestLTSSMPMLink(unittest.TestCase):
    wait_for = test_ltssm.TestLTSSM.wait_for

    def both(self, dut, name):
        return getattr(dut.host, name) & getattr(dut.dev, name)

    def test_link_u0_u1_u0(self):
        for initiator, exiter in [("host", "host"), ("host", "dev"), ("dev", "host"), ("dev", "dev")]:
            with self.subTest(pm_initiator=initiator, exit_initiator=exiter):
                dut = LTSSMPMLinkHarness()
                def generator():
                    self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
                    yield getattr(dut, initiator).req_u1.eq(1)
                    yield
                    yield getattr(dut, initiator).req_u1.eq(0)
                    self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u1"), timeout=4*PM_ENTRY_CYCLES)))
                    self.assertEqual((yield dut.host.recovery | dut.dev.recovery), 0)
                    for _ in range(16): # U1_MIN_RESIDENCY.
                        yield
                    yield getattr(dut, exiter).exit_u1_req.eq(1)
                    yield
                    yield getattr(dut, exiter).exit_u1_req.eq(0)
                    self.assertTrue((yield from self.wait_for(dut, self.both(dut, "recovery"), timeout=256)))
                    self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
                    self.assertEqual((yield dut.host.ess_inactive | dut.dev.ess_inactive), 0)

                run_simulation(dut, generator())

    def test_link_u0_u2_u0(self):
        burst = burst_cycles(2, PM_CLK_FREQ, 1)
        dut = LTSSMPMLinkHarness()
        def generator():
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
            yield dut.dev.req_u2.eq(1)
            yield
            yield dut.dev.req_u2.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u2"), timeout=4*PM_ENTRY_CYCLES)))
            yield dut.host.exit_u2_req.eq(1)
            yield
            yield dut.host.exit_u2_req.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "recovery"), timeout=4*burst)))
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))

        run_simulation(dut, generator())

    def test_link_u0_u3_u0(self):
        burst = burst_cycles(3, PM_CLK_FREQ, 1)
        dut = LTSSMPMLinkHarness()
        def generator():
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
            yield dut.host.req_u3.eq(1)
            yield
            yield dut.host.req_u3.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u3"), timeout=4*PM_ENTRY_CYCLES)))
            yield dut.host.wakeup_u3_req.eq(1)
            yield
            yield dut.host.wakeup_u3_req.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "recovery"), timeout=4*burst)))
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
            self.assertEqual((yield dut.host.ess_inactive | dut.dev.ess_inactive), 0)

        run_simulation(dut, generator())

    def test_link_u1_exit_concurrent(self):
        # Both partners initiate U1 exit at the same time (concurrent-safe handshake, 6.9.2).
        dut = LTSSMPMLinkHarness()
        def generator():
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=256)))
            yield dut.host.req_u1.eq(1)
            yield
            yield dut.host.req_u1.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u1"), timeout=4*PM_ENTRY_CYCLES)))
            for _ in range(16):
                yield
            yield dut.host.exit_u1_req.eq(1)
            yield dut.dev.exit_u1_req.eq(1)
            yield
            yield dut.host.exit_u1_req.eq(0)
            yield dut.dev.exit_u1_req.eq(0)
            self.assertTrue((yield from self.wait_for(dut, self.both(dut, "u0"), timeout=512)))

        run_simulation(dut, generator())

if __name__ == "__main__":
    unittest.main()
