#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from usb3_pipe.ltssm import USB3LTSSM

# Single-instance LTSSM state/transition/timeout coverage (USB 3.2 section 7.5) ---------------------
#
# Timers are accelerated with timer_scale (all timeouts divided by timer_scale): at 125MHz and
# TIMER_SCALE=10000, 1ms = 12 cycles, 2ms = 25, 6ms = 75, 12ms = 150, 100ms = 1250, 360ms = 4500.
# Compliance entry (360ms) uses COMPLIANCE_SCALE to keep the tests fast.
#
# Tests marked @unittest.expectedFailure assert the specification behavior where the current
# implementation deviates from it (bug candidates, see the comment of each test).

SYS_CLK_FREQ     = int(125e6)
TIMER_SCALE      = 10000
COMPLIANCE_SCALE = 100000

# Helpers ------------------------------------------------------------------------------------------

def cycles(t, timer_scale=TIMER_SCALE):
    # Same scaling as USB3LTSSM.
    return max(1, int(t*SYS_CLK_FREQ)//timer_scale)

def new_dut(timer_scale=TIMER_SCALE, **kwargs):
    dut = USB3LTSSM(sys_clk_freq=SYS_CLK_FREQ, timer_scale=timer_scale, **kwargs)
    dut.scale = timer_scale
    return dut

def run(dut, generator):
    run_simulation(dut, generator(dut))

def pulse(signal):
    yield signal.eq(1)
    yield
    yield signal.eq(0)
    yield

def state(dut):
    return dut.fsm.decoding[(yield dut.fsm.state)]

def wait_state(dut, name, timeout=64):
    # Returns the number of cycles waited before the FSM is in state name (None on timeout).
    for n in range(timeout):
        if (yield from state(dut)) == name:
            return n
        yield
    return None

def stay_in(dut, name, n):
    # True if the FSM stays in state name for n cycles.
    for _ in range(n):
        if (yield from state(dut)) != name:
            return False
        yield
    return True

OUTPUTS = [
    "u0", "rx_ready", "tx_ready", "u1", "u2", "u3", "compliance", "exit_to_compliance",
    "exit_to_rx_detect", "loopback", "hot_reset", "ess_inactive", "polling_idle",
    "lfps_tx_idle", "lfps_tx_polling", "ts_rx_enable", "ts_tx_enable",
    "ts_tx_tseq", "ts_tx_ts1", "ts_tx_ts2", "idle_handshake", "ts2_decode",
]

def outputs(dut):
    r = {}
    for name in OUTPUTS:
        r[name] = (yield getattr(dut, name))
    return r

# State driver -------------------------------------------------------------------------------------

class LTSSMDriver:
    def expect(self, dut, name, timeout=64):
        n = yield from wait_state(dut, name, timeout)
        self.assertIsNotNone(n, "{} not reached (in {})".format(name, (yield from state(dut))))
        return n

    def goto(self, dut, target):
        """Drive a fresh USB3LTSSM to target. ds_port/compliance_enable/loopback_capable must be set
        before. Leaves rx_idle8=0 (Idle handshakes not completed) and all pulsed inputs low."""
        yield dut.rx_idle8.eq(0)
        yield
        if target == "Compliance":
            yield from self.expect(dut, "Compliance", cycles(360e-3, dut.scale) + 16)
            return
        yield from self.expect(dut, "Polling.LFPS")
        if target == "Polling.LFPS":
            return

        # Polling.LFPS -> Polling.RxEQ (7.5.4.3.2).
        yield dut.lfps_tx_count.eq(20)
        yield dut.lfps_rx_polling.eq(1)
        yield
        yield dut.lfps_tx_count.eq(24)
        yield from self.expect(dut, "Polling.RxEQ")
        yield dut.lfps_rx_polling.eq(0)
        if target == "Polling.RxEQ":
            return

        # Polling.RxEQ -> Polling.Active (7.5.4.7.2).
        yield from pulse(dut.ts_tx_done)
        yield from self.expect(dut, "Polling.Active")
        if target == "Polling.Active":
            return

        # Polling.Active -> Polling.Configuration (7.5.4.8.2).
        yield from pulse(dut.ts_rx_ts1)
        yield from pulse(dut.ts_tx_done)
        yield from self.expect(dut, "Polling.Configuration")
        if target == "Polling.Configuration":
            return

        # Polling.Configuration -> Polling.Idle (7.5.4.9.2).
        yield from pulse(dut.ts_rx_ts2)
        yield from pulse(dut.ts_tx_done)
        yield from pulse(dut.ts_tx_done)
        yield from self.expect(dut, "Polling.Idle")
        if target == "Polling.Idle":
            return

        # Polling.Idle -> Loopback (slave on TS2 Loopback bit / master when directed).
        if target in ["Loopback", "Loopback.Master", "Loopback.Exit"]:
            if target == "Loopback":
                yield from pulse(dut.rx_ts2_loopback)
            else:
                yield dut.loopback_capable.eq(1)
                yield dut.directed_loopback.eq(1)
            yield from self.expect(dut, "Loopback")
            if target == "Loopback.Exit":
                yield dut.directed_loopback.eq(0)
                yield from self.expect(dut, "Loopback.Exit")
            return

        # Polling.Idle -> HotReset.Active (US: TS2 Reset bit / DS: directed).
        if target in ["HotReset.Active", "HotReset.Exit"]:
            if (yield dut.ds_port):
                yield from pulse(dut.hot_reset_req)
            else:
                yield from pulse(dut.rx_ts2_reset)
            yield from self.expect(dut, "HotReset.Active")
            if target == "HotReset.Exit":
                yield from pulse(dut.rx_ts2_reset) # Partner Reset bit seen, then cleared.
                yield from self.expect(dut, "HotReset.Exit", 64)
            return

        # Polling.Idle -> U0 (Idle handshake, 7.5.4.10.2).
        yield dut.rx_idle8.eq(1)
        yield from self.expect(dut, "U0")
        yield dut.rx_idle8.eq(0)
        yield
        if target == "U0":
            return

        # U0 -> U1/U2/U3 (partner LGO_Ux accepted, PM_ENTRY_TIMER, 7.2.4.2).
        if target in ["U1", "U2", "U3"]:
            yield from pulse(getattr(dut, "lgo_" + target.lower()))
            yield from self.expect(dut, target)
            return

        # U0 -> Recovery.Active (directed, 7.5.6.2).
        yield from pulse(dut.directed_recovery)
        yield from self.expect(dut, "Recovery.Active")
        if target == "Recovery.Active":
            return

        # Recovery.Active -> Recovery.Configuration (7.5.10.3.2).
        yield from pulse(dut.ts_rx_ts1)
        yield from pulse(dut.ts_tx_done)
        yield from self.expect(dut, "Recovery.Configuration")
        if target == "Recovery.Configuration":
            return

        # Recovery.Configuration -> Recovery.Idle (7.5.10.4.2).
        yield from pulse(dut.ts_rx_ts2)
        yield from pulse(dut.ts_tx_done)
        yield from pulse(dut.ts_tx_done)
        yield from self.expect(dut, "Recovery.Idle")
        if target == "Recovery.Idle":
            return

        # Recovery.Idle -> eSS.Inactive (tRecoveryIdleTimeout).
        if target == "eSS.Inactive":
            yield from self.expect(dut, "eSS.Inactive", cycles(2e-3) + 16)
            return

        raise ValueError(target)

# Outputs per state --------------------------------------------------------------------------------

STATE_OUTPUTS = {
    "Polling.LFPS"           : ["lfps_tx_idle", "lfps_tx_polling"],
    "Polling.RxEQ"           : ["ts_rx_enable", "ts_tx_enable", "ts_tx_tseq"],
    "Polling.Active"         : ["ts_rx_enable", "ts_tx_enable", "ts_tx_ts1"],
    "Polling.Configuration"  : ["ts_rx_enable", "ts_tx_enable", "ts_tx_ts2"],
    "Polling.Idle"           : ["idle_handshake", "polling_idle", "ts2_decode"],
    "U0"                     : ["u0", "rx_ready", "tx_ready"],
    "U1"                     : ["lfps_tx_idle", "u1"],
    "U2"                     : ["lfps_tx_idle", "u2"],
    "U3"                     : ["lfps_tx_idle", "u3"],
    "Recovery.Active"        : ["ts_rx_enable", "ts_tx_enable", "ts_tx_ts1"],
    "Recovery.Configuration" : ["ts_rx_enable", "ts_tx_enable", "ts_tx_ts2"],
    "Recovery.Idle"          : ["idle_handshake", "ts2_decode"],
    "Loopback"               : ["loopback"],
    "Loopback.Exit"          : ["lfps_tx_idle", "loopback"],
    "HotReset.Active"        : ["hot_reset", "ts_rx_enable", "ts_tx_enable", "ts_tx_ts2"],
    "HotReset.Exit"          : ["hot_reset", "idle_handshake"],
    "eSS.Inactive"           : ["lfps_tx_idle", "ess_inactive"],
    "Compliance"             : ["lfps_tx_idle", "compliance", "exit_to_compliance"],
}

class TestLTSSMStateOutputs(unittest.TestCase, LTSSMDriver):
    def test_outputs_per_state(self):
        # Status/control outputs are a function of the state only: rx_ready/tx_ready only in U0,
        # SuperSpeed transmission (lfps_tx_idle=0) in Polling.RxEQ..Idle, U0, Recovery, Hot Reset
        # and Loopback; TX electrical idle/LFPS in U1/U2/U3/eSS.Inactive (7.5).
        for target, high in STATE_OUTPUTS.items():
            with self.subTest(state=target):
                scale = COMPLIANCE_SCALE if target == "Compliance" else TIMER_SCALE
                def generator(dut):
                    yield from self.goto(dut, target)
                    self.assertEqual((yield from state(dut)), target)
                    expected = {name: int(name in high) for name in OUTPUTS}
                    self.assertEqual((yield from outputs(dut)), expected)
                run(new_dut(scale), generator)

    def test_exit_to_rx_detect_outputs(self):
        # Transient Rx.Detect equivalent: TX idle, exit_to_rx_detect, then back to Polling.Entry.
        def generator(dut):
            yield from self.goto(dut, "eSS.Inactive")
            yield from self.expect(dut, "Polling.ExitToRxDetect", cycles(12e-3) + 16)
            expected = {name: int(name in ["lfps_tx_idle", "exit_to_rx_detect"]) for name in OUTPUTS}
            self.assertEqual((yield from outputs(dut)), expected)
            yield from self.expect(dut, "Polling.Entry", 16)
            yield from self.expect(dut, "Polling.LFPS", 4)
        run(new_dut(), generator)

    def test_recovery_status(self):
        # recovery status set on Recovery entry, cleared on U0 return.
        def generator(dut):
            yield from self.goto(dut, "Recovery.Idle")
            self.assertEqual((yield dut.recovery), 1)
            yield dut.rx_idle8.eq(1)
            yield from self.expect(dut, "U0")
            yield
            self.assertEqual((yield dut.recovery), 0)
        run(new_dut(), generator)

# Timeouts (USB 3.2 Table 7-12) --------------------------------------------------------------------

class TestLTSSMTimeouts(unittest.TestCase, LTSSMDriver):
    def check_timeout(self, dut, src, dst, t, margin=4):
        # Must stay in src until ~t, then transition to dst.
        n = yield from wait_state(dut, dst, cycles(t, dut.scale) + 32)
        self.assertIsNotNone(n, "{} -> {} timeout not seen".format(src, dst))
        self.assertAlmostEqual(n, cycles(t, dut.scale), delta=margin)

    def test_polling_lfps_360ms_timeout(self):
        # tPollingLFPSTimeout (7.5.4.3.2): UFP -> Compliance, DS -> Compliance only when enabled,
        # otherwise Rx.Detect.
        for ds_port, compliance_enable, dst in [
            (0, 0, "Compliance"),
            (0, 1, "Compliance"),
            (1, 0, "Polling.ExitToRxDetect"),
            (1, 1, "Compliance")]:
            with self.subTest(ds_port=ds_port, compliance_enable=compliance_enable):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield dut.compliance_enable.eq(compliance_enable)
                    yield from self.expect(dut, "Polling.LFPS", 4)
                    self.assertEqual((yield dut.lfps_tx_polling), 1)
                    yield from self.check_timeout(dut, "Polling.LFPS", dst, 360e-3)
                    if dst == "Polling.ExitToRxDetect":
                        yield from self.expect(dut, "Polling.LFPS", 16)
                    else:
                        self.assertTrue((yield from stay_in(dut, "Compliance", 64)))
                run(new_dut(COMPLIANCE_SCALE), generator)

    def test_polling_lfps_without_timers_stays(self):
        def generator(dut):
            yield from self.expect(dut, "Polling.LFPS", 4)
            self.assertTrue((yield from stay_in(dut, "Polling.LFPS", cycles(360e-3, dut.scale) + 64)))
        run(new_dut(COMPLIANCE_SCALE, with_timers=False), generator)

    def test_polling_active_12ms_timeout(self):
        def generator(dut):
            yield from self.goto(dut, "Polling.Active")
            yield from self.check_timeout(dut, "Polling.Active", "Polling.ExitToRxDetect", 12e-3)
            yield from self.expect(dut, "Polling.LFPS", 16)
        run(new_dut(), generator)

    def test_polling_configuration_12ms_timeout(self):
        def generator(dut):
            yield from self.goto(dut, "Polling.Configuration")
            n = yield from wait_state(dut, "Polling.ExitToRxDetect", cycles(12e-3) + 32)
            self.assertIsNotNone(n)
            self.assertLessEqual(n, cycles(12e-3) + 4)
        run(new_dut(), generator)

    @unittest.expectedFailure
    def test_polling_configuration_timer_starts_on_entry(self):
        # SPEC: tPollingConfigurationTimeout (12ms, Table 7-12) is started upon entry to
        # Polling.Configuration (7.5.4.9.1). Polling.Active and Polling.Configuration share the same
        # 12ms WaitTimer with wait held continuously across the transition, so the timer is not
        # restarted: time spent in Polling.Active is deducted from Polling.Configuration.
        def generator(dut):
            yield from self.goto(dut, "Polling.Active")
            for _ in range(cycles(12e-3)*3//4):
                yield
            yield from pulse(dut.ts_rx_ts1)
            yield from pulse(dut.ts_tx_done)
            yield from self.expect(dut, "Polling.Configuration")
            self.assertTrue((yield from stay_in(dut, "Polling.Configuration", cycles(12e-3) - 4)))
        run(new_dut(), generator)

    def test_polling_idle_2ms_timeout(self):
        def generator(dut):
            yield from self.goto(dut, "Polling.Idle")
            yield from self.check_timeout(dut, "Polling.Idle", "Polling.ExitToRxDetect", 2e-3)
        run(new_dut(), generator)

    def test_recovery_timeouts(self):
        # tRecoveryActiveTimeout 12ms, tRecoveryConfigurationTimeout 6ms, tRecoveryIdleTimeout 2ms
        # -> eSS.Inactive (Table 7-12).
        for src, t in [
            ("Recovery.Active",        12e-3),
            ("Recovery.Configuration",  6e-3),
            ("Recovery.Idle",           2e-3)]:
            with self.subTest(state=src):
                def generator(dut):
                    yield from self.goto(dut, src)
                    yield from self.check_timeout(dut, src, "eSS.Inactive", t)
                run(new_dut(), generator)

    def test_hot_reset_active_12ms_timeout(self):
        # Partner never clears its Reset bit -> tHotResetActiveTimeout -> eSS.Inactive.
        def generator(dut):
            yield from self.goto(dut, "Polling.Idle")
            yield dut.rx_ts2_reset.eq(1)
            yield from self.expect(dut, "HotReset.Active")
            yield from self.check_timeout(dut, "HotReset.Active", "eSS.Inactive", 12e-3)
        run(new_dut(), generator)

    def test_hot_reset_exit_2ms_timeout(self):
        # No Idle handshake -> tHotResetExitTimeout -> eSS.Inactive.
        def generator(dut):
            yield from self.goto(dut, "HotReset.Exit")
            yield from self.check_timeout(dut, "HotReset.Exit", "eSS.Inactive", 2e-3)
        run(new_dut(), generator)

    def test_ess_inactive_quiet_timeout(self):
        # teSSInactiveQuietTimeout (12ms) -> Rx.Detect (sandbox: transient re-train).
        def generator(dut):
            yield from self.goto(dut, "eSS.Inactive")
            yield from self.check_timeout(dut, "eSS.Inactive", "Polling.ExitToRxDetect", 12e-3)
            yield from self.expect(dut, "Polling.LFPS", 16)
        run(new_dut(), generator)

    def test_u0_recovery_timeout(self):
        # tU0RecoveryTimeout (1ms) without link activity -> Recovery (7.5.6.2 trigger 4).
        def generator(dut):
            yield from self.goto(dut, "U0")
            # Link activity keeps U0 / restarts the timer.
            for _ in range(4):
                yield dut.link_activity.eq(0)
                for _ in range(cycles(1e-3) - 4):
                    yield
                self.assertEqual((yield from state(dut)), "U0")
                yield dut.link_activity.eq(1)
                yield
                yield
            yield dut.link_activity.eq(0)
            yield
            yield from self.check_timeout(dut, "U0", "Recovery.Active", 1e-3)
            yield
            self.assertEqual((yield dut.recovery), 1)
        run(new_dut(), generator)

    def test_u0_recovery_timeout_disabled_without_timers(self):
        def generator(dut):
            yield from self.goto(dut, "U0")
            yield dut.link_activity.eq(0)
            self.assertTrue((yield from stay_in(dut, "U0", 4*cycles(1e-3))))
        run(new_dut(with_timers=False), generator)

    def test_u1_to_u2_inactivity_timer(self):
        # PORT_U2_TIMEOUT: U1 -> U2 directly on U2 inactivity timer expiry (7.5.7.2).
        for enable in [0, 1]:
            with self.subTest(u2_inactivity_enable=enable):
                def generator(dut):
                    yield dut.u2_inactivity_enable.eq(enable)
                    yield from self.goto(dut, "U1")
                    if enable:
                        yield from self.check_timeout(dut, "U1", "U2", 1e-3)
                        self.assertTrue((yield from stay_in(dut, "U2", 32)))
                    else:
                        self.assertTrue((yield from stay_in(dut, "U1", 8*cycles(1e-3))))
                run(new_dut(), generator)

    def test_ux_exit_lfps_handshake_timeout(self):
        # tNoLFPSResponseTimeout (2ms) on U1/U2 exit without partner response -> eSS.Inactive.
        for ux, req, lfps in [("U1", "exit_u1_req", "lfps_tx_exit_u1"),
                              ("U2", "exit_u2_req", "lfps_tx_exit_u2")]:
            with self.subTest(state=ux):
                def generator(dut):
                    yield from self.goto(dut, ux)
                    for _ in range(4): # U1_MIN_RESIDENCY.
                        yield
                    yield getattr(dut, req).eq(1)
                    yield
                    yield getattr(dut, req).eq(0)
                    yield
                    self.assertEqual((yield getattr(dut, lfps)), 1)
                    yield from self.check_timeout(dut, ux, "eSS.Inactive", 2e-3)
                run(new_dut(), generator)

    def test_ux_exit_timer_in_recovery(self):
        # Ux_EXIT_TIMER (6ms, 7.2.4.2.1) armed on U1 exit LFPS: Recovery not completed -> eSS.Inactive
        # 6ms after the exit start (before tRecoveryActiveTimeout).
        def generator(dut):
            yield from self.goto(dut, "U1")
            for _ in range(4):
                yield
            yield dut.exit_u1_req.eq(1)
            yield
            yield dut.exit_u1_req.eq(0)
            yield dut.lfps_rx_exit_u1.eq(1) # Partner response.
            n0 = yield from self.expect(dut, "Recovery.Active", cycles(2e-3))
            yield dut.lfps_rx_exit_u1.eq(0)
            n1 = yield from self.expect(dut, "eSS.Inactive", cycles(12e-3))
            self.assertAlmostEqual(n0 + n1, cycles(6e-3), delta=6)
        run(new_dut(), generator)

    def test_u1_ds_ping_timeout(self):
        # tU1PingTimeout (300ms) without Ping.LFPS -> Rx.Detect for a DS port (7.5.7.2); a received
        # Ping restarts the timer.
        def generator(dut):
            yield dut.ds_port.eq(1)
            yield from self.goto(dut, "U1")
            for _ in range(cycles(300e-3, dut.scale)*3//4):
                yield
            yield from pulse(dut.lfps_rx_ping)
            self.assertTrue((yield from stay_in(dut, "U1", cycles(300e-3, dut.scale)*3//4)))
            n = yield from wait_state(dut, "Polling.ExitToRxDetect", cycles(300e-3, dut.scale))
            self.assertIsNotNone(n)
        # tU1PingTimeout is not gated by with_timers: disable the other timers to use a large scale.
        run(new_dut(COMPLIANCE_SCALE, with_timers=False), generator)

    def test_u3_ds_wakeup_retries_then_ess_inactive(self):
        # DS U3 wakeup without response: tNoLFPSResponseTimeout (10ms), 2 retries after
        # tU3WakeupRetryDelay (100ms), then eSS.Inactive (7.2.4.2.4).
        def generator(dut):
            yield dut.ds_port.eq(1)
            yield from self.goto(dut, "U3")
            yield from pulse(dut.wakeup_u3_req)
            bursts = 0
            prev   = 0
            for _ in range(3*cycles(10e-3) + 2*cycles(100e-3) + 64):
                s   = yield from state(dut)
                cur = yield dut.lfps_tx_wakeup_u3
                bursts += (cur & ~prev) & 1
                prev = cur
                if s == "eSS.Inactive":
                    break
                yield
            self.assertEqual((yield from state(dut)), "eSS.Inactive")
            self.assertEqual(bursts, 3)
        run(new_dut(), generator)

    def test_pm_lc_timer_timeout(self):
        # LGO_Ux sent, no LAU/LXU -> PM_LC_TIMER -> Recovery (7.2.4.2.3).
        def generator(dut):
            yield from self.goto(dut, "U0")
            yield from pulse(dut.req_u1)
            yield from self.expect(dut, "Recovery.Active", 16)
        run(new_dut(), generator)

# Warm Reset (USB 3.2 6.9, 7.5) --------------------------------------------------------------------

def warm_reset_dut(src):
    return new_dut(COMPLIANCE_SCALE if src == "Compliance" else TIMER_SCALE)

class WarmResetChecks(LTSSMDriver):
    def check_rx_warm_reset(self, src):
        # UFP: Warm Reset LFPS detected -> respond until the DS stops -> Rx.Detect.
        def generator(dut):
            yield from self.goto(dut, src)
            yield dut.lfps_rx_reset.eq(1)
            yield from self.expect(dut, "WarmReset", 4)
            yield
            self.assertEqual((yield dut.lfps_tx_reset), 1)
            self.assertEqual((yield dut.lfps_tx_idle),  1)
            self.assertTrue((yield from stay_in(dut, "WarmReset", 32)))
            yield dut.lfps_rx_reset.eq(0)
            yield from self.expect(dut, "Polling.ExitToRxDetect", 4)
            self.assertEqual((yield dut.lfps_tx_reset), 0)
            yield from self.expect(dut, "Polling.LFPS", 16)
        run(warm_reset_dut(src), generator)

    def check_ds_warm_reset(self, src):
        # DS: directed Warm Reset -> Warm Reset LFPS (tReset 100ms) -> Rx.Detect.
        def generator(dut):
            yield dut.ds_port.eq(1)
            yield dut.compliance_enable.eq(1)
            yield from self.goto(dut, src)
            yield from pulse(dut.warm_reset_req)
            yield from self.expect(dut, "WarmReset", 4)
            yield
            self.assertEqual((yield dut.lfps_tx_reset), 1)
            n = yield from wait_state(dut, "Polling.ExitToRxDetect", cycles(100e-3, dut.scale) + 16)
            self.assertIsNotNone(n)
            self.assertAlmostEqual(n, cycles(100e-3, dut.scale), delta=4)
            self.assertEqual((yield dut.lfps_tx_reset), 0)
            yield from self.expect(dut, "Polling.LFPS", 16)
        run(warm_reset_dut(src), generator)

class TestLTSSMWarmReset(unittest.TestCase, WarmResetChecks):
    def test_ufp_warm_reset_detect(self):
        for src in ["U0", "U1", "U2", "U3", "Compliance", "Loopback", "Loopback.Master"]:
            with self.subTest(state=src):
                self.check_rx_warm_reset(src)

    def test_ds_directed_warm_reset(self):
        for src in ["Polling.Idle", "U0", "U3", "Compliance", "Loopback", "Loopback.Master"]:
            with self.subTest(state=src):
                self.check_ds_warm_reset(src)

    def test_ds_directed_hot_reset_in_u3_loopback_is_warm_reset(self):
        # PORT_RESET directed in U3/Loopback -> Warm Reset (Inband Reset selection, 7.4.2).
        for src in ["U3", "Loopback", "Loopback.Master"]:
            with self.subTest(state=src):
                def generator(dut):
                    yield dut.ds_port.eq(1)
                    yield from self.goto(dut, src)
                    yield from pulse(dut.hot_reset_req)
                    yield from self.expect(dut, "WarmReset", 4)
                    yield
                    self.assertEqual((yield dut.lfps_tx_reset), 1)
                run(new_dut(), generator)

    def test_warm_reset_req_ignored_by_ufp(self):
        # Only a DS originates Warm Reset (6.9).
        for src in ["U0", "Compliance", "Polling.Idle"]:
            with self.subTest(state=src):
                def generator(dut):
                    yield from self.goto(dut, src)
                    yield dut.warm_reset_req.eq(1)
                    for _ in range(16):
                        self.assertNotEqual((yield from state(dut)), "WarmReset")
                        self.assertEqual((yield dut.lfps_tx_reset), 0)
                        yield
                scale = COMPLIANCE_SCALE if src == "Compliance" else TIMER_SCALE
                run(new_dut(scale, with_timers=(src == "Compliance")), generator)

# SPEC deviations: Warm Reset must be honored in every state (except eSS.Disabled). An upstream port
# shall transition to Rx.Detect upon detection of Warm Reset, a downstream port when directed to
# issue Warm Reset (USB 3.2 6.9 / 7.4.2 / exit conditions of 7.5.2.2 eSS.Inactive, 7.5.4.x Polling,
# 7.5.7.2 U1, 7.5.8.2 U2, 7.5.10.x Recovery, 7.5.11.3 Loopback.Exit, 7.5.12.x Hot Reset). The
# implementation only checks lfps_rx_reset in U0/U1/U2/U3/Compliance/Loopback and warm_reset_req in
# Polling.Idle/U0/U3/Compliance/Loopback.

UFP_WARM_RESET_MISSING = [
    "Polling.LFPS", "Polling.RxEQ", "Polling.Active", "Polling.Configuration", "Polling.Idle",
    "Recovery.Active", "Recovery.Configuration", "Recovery.Idle",
    "HotReset.Active", "HotReset.Exit", "Loopback.Exit", "eSS.Inactive",
]
DS_WARM_RESET_MISSING = [
    "Polling.LFPS", "Polling.RxEQ", "Polling.Active", "Polling.Configuration",
    "U1", "U2",
    "Recovery.Active", "Recovery.Configuration", "Recovery.Idle",
    "HotReset.Active", "HotReset.Exit", "Loopback.Exit", "eSS.Inactive",
]

class TestLTSSMWarmResetSpec(unittest.TestCase, WarmResetChecks):
    pass

def _name(s):
    return s.lower().replace(".", "_")

def _ufp_test(src):
    return unittest.expectedFailure(lambda self: self.check_rx_warm_reset(src))

def _ds_test(src):
    return unittest.expectedFailure(lambda self: self.check_ds_warm_reset(src))

for _src in UFP_WARM_RESET_MISSING:
    setattr(TestLTSSMWarmResetSpec, "test_ufp_warm_reset_detect_in_" + _name(_src), _ufp_test(_src))
for _src in DS_WARM_RESET_MISSING:
    setattr(TestLTSSMWarmResetSpec, "test_ds_directed_warm_reset_in_" + _name(_src), _ds_test(_src))

# Polling.Idle / Recovery.Idle exits (USB 3.2 7.5.4.10.2, 7.5.10.5.2) ------------------------------

class TestLTSSMIdleExits(unittest.TestCase, LTSSMDriver):
    def test_polling_idle_exits(self):
        cases = [
            # (ds_port, inputs,                                              dst)
            (0, {"rx_ts2_loopback": 1},                                   "Loopback"),
            (1, {"rx_ts2_loopback": 1},                                   "Loopback"),
            (0, {"directed_loopback": 1, "loopback_capable": 1},          "Loopback"),
            (0, {"directed_loopback": 1, "rx_idle8": 1},                  "U0"),       # Not capable.
            (0, {"rx_ts2_reset": 1},                                      "HotReset.Active"),
            (1, {"rx_ts2_reset": 1, "rx_idle8": 1},                       "U0"),       # DS ignores.
            (1, {"hot_reset_req": 1},                                     "HotReset.Active"),
            (0, {"hot_reset_req": 1, "rx_idle8": 1},                      "U0"),       # UFP ignores.
            (1, {"warm_reset_req": 1},                                    "WarmReset"),
            (0, {"rx_idle8": 1},                                          "U0"),
            # Priorities: Loopback bit > Reset bit > Idle handshake.
            (0, {"rx_ts2_loopback": 1, "rx_ts2_reset": 1, "rx_idle8": 1}, "Loopback"),
            (0, {"rx_ts2_reset": 1, "rx_idle8": 1},                       "HotReset.Active"),
        ]
        for ds_port, inputs, dst in cases:
            with self.subTest(ds_port=ds_port, inputs=inputs):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield from self.goto(dut, "Polling.Idle")
                    for k, v in inputs.items():
                        yield getattr(dut, k).eq(v)
                    yield from self.expect(dut, dst, 4)
                run(new_dut(), generator)

    def test_polling_idle_loopback_bit_latched_in_configuration(self):
        # TS2 Loopback bit seen in Polling.Configuration is latched -> slave Loopback.
        def generator(dut):
            yield from self.goto(dut, "Polling.Configuration")
            yield from pulse(dut.rx_ts2_loopback)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield from pulse(dut.ts_tx_done)
            yield dut.rx_idle8.eq(1)
            yield from self.expect(dut, "Loopback", 8)
        run(new_dut(), generator)

    def test_recovery_idle_exits(self):
        cases = [
            # (ds_port, inputs,                                     dst)
            (0, {"rx_ts2_loopback": 1},                          "Loopback"),
            (0, {"directed_loopback": 1, "loopback_capable": 1}, "Loopback"),
            (0, {"directed_loopback": 1, "rx_idle8": 1},         "U0"),       # Not capable.
            (0, {"rx_ts2_reset": 1},                             "HotReset.Active"),
            (1, {"hot_reset_req": 1},                            "HotReset.Active"),
            (0, {"hot_reset_req": 1, "rx_idle8": 1},             "U0"),       # UFP ignores.
            (0, {"rx_idle8": 1},                                 "U0"),
            (0, {"rx_ts2_reset": 1, "rx_idle8": 1},              "HotReset.Active"),
        ]
        for ds_port, inputs, dst in cases:
            with self.subTest(ds_port=ds_port, inputs=inputs):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield from self.goto(dut, "Recovery.Idle")
                    for k, v in inputs.items():
                        yield getattr(dut, k).eq(v)
                    yield from self.expect(dut, dst, 4)
                run(new_dut(), generator)

    def test_recovery_idle_loopback_bit_latched(self):
        # TS2 Loopback bit seen in Recovery.Configuration is latched -> slave Loopback.
        def generator(dut):
            yield from self.goto(dut, "Recovery.Configuration")
            yield from pulse(dut.rx_ts2_loopback)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield from pulse(dut.ts_tx_done)
            yield dut.rx_idle8.eq(1)
            yield from self.expect(dut, "Loopback", 8)
            # Slave: does not exit on its own.
            self.assertTrue((yield from stay_in(dut, "Loopback", 32)))
        run(new_dut(), generator)

    def test_u0_directed_loopback_via_recovery(self):
        # Directed loopback from U0: Recovery with the TS2 Loopback bit set (7.5.11), master entry.
        def generator(dut):
            yield from self.goto(dut, "U0")
            yield dut.loopback_capable.eq(1)
            yield dut.directed_loopback.eq(1)
            yield from self.expect(dut, "Recovery.Active", 4)
            self.assertEqual((yield dut.tx_ts2_loopback), 0)
            yield from pulse(dut.ts_rx_ts1)
            yield from pulse(dut.ts_tx_done)
            yield from self.expect(dut, "Recovery.Configuration")
            self.assertEqual((yield dut.tx_ts2_loopback), 1)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield from pulse(dut.ts_tx_done)
            yield from self.expect(dut, "Loopback", 8)
            self.assertEqual((yield dut.tx_ts2_loopback), 0)
            # Master: exits when no longer directed.
            yield dut.directed_loopback.eq(0)
            yield from self.expect(dut, "Loopback.Exit", 4)
        run(new_dut(), generator)

    @unittest.expectedFailure
    def test_polling_configuration_directed_loopback_sets_ts2_loopback_bit(self):
        # SPEC: a port directed to Loopback as master sets the Loopback bit in the Link
        # Configuration field of the TS2 it transmits in Polling.Configuration (7.5.4.9.1, Table
        # 6-6), which is how the slave learns it must enter Loopback from Polling.Idle (7.5.4.10.2).
        # tx_ts2_loopback is only asserted in Recovery.Configuration/Idle (lb_pending from U0), so a
        # master directed during Polling enters Loopback alone and the partner goes to U0.
        def generator(dut):
            yield dut.loopback_capable.eq(1)
            yield dut.directed_loopback.eq(1)
            yield from self.goto(dut, "Polling.Configuration")
            yield
            self.assertEqual((yield dut.tx_ts2_loopback), 1)
        run(new_dut(), generator)

# Hot Reset (USB 3.2 7.5.12) -----------------------------------------------------------------------

class TestLTSSMHotReset(unittest.TestCase, LTSSMDriver):
    def test_hot_reset_active_exit_u0(self):
        # HotReset.Active (TS2 Reset=1 until own reset done, then Reset=0) -> HotReset.Exit when the
        # partner's Reset bit falls -> Idle handshake -> U0.
        for ds_port in [0, 1]:
            with self.subTest(ds_port=ds_port):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield from self.goto(dut, "HotReset.Active")
                    yield
                    self.assertEqual((yield dut.tx_ts2_reset), 1)
                    self.assertEqual((yield dut.hot_reset), 1)
                    yield dut.rx_ts2_reset.eq(1)
                    # Own reset done after hr_delay: Reset=0 TS2.
                    n = yield from self.wait_for_value(dut, dut.tx_ts2_reset, 0, 64)
                    self.assertAlmostEqual(n, 16, delta=4)
                    # Stay in Active while the partner still sends Reset=1.
                    self.assertTrue((yield from stay_in(dut, "HotReset.Active", 16)))
                    yield dut.rx_ts2_reset.eq(0)
                    yield from self.expect(dut, "HotReset.Exit", 4)
                    self.assertEqual((yield dut.hot_reset), 1)
                    self.assertEqual((yield dut.idle_handshake), 1)
                    self.assertTrue((yield from stay_in(dut, "HotReset.Exit", 8)))
                    yield dut.rx_idle8.eq(1)
                    yield from self.expect(dut, "U0", 4)
                    self.assertEqual((yield dut.hot_reset), 0)
                    self.assertEqual((yield dut.rx_ready), 1)
                run(new_dut(), generator)

    def wait_for_value(self, dut, signal, value, timeout):
        for n in range(timeout):
            if (yield signal) == value:
                return n
            yield
        return None

    def test_hot_reset_no_partner_reset_bit_times_out(self):
        # Partner Reset bit never seen -> stays in HotReset.Active until the 12ms timeout.
        def generator(dut):
            yield dut.ds_port.eq(1)
            yield from self.goto(dut, "HotReset.Active")
            self.assertTrue((yield from stay_in(dut, "HotReset.Active", cycles(12e-3) - 8)))
            yield from self.expect(dut, "eSS.Inactive", 16)
        run(new_dut(), generator)

    def test_u0_hot_reset_req_ignored_by_ufp(self):
        def generator(dut):
            yield from self.goto(dut, "U0")
            yield dut.hot_reset_req.eq(1)
            self.assertTrue((yield from stay_in(dut, "U0", 16)))
        run(new_dut(), generator)

    def test_ds_hot_reset_from_u0_goes_through_recovery(self):
        # Hot Reset is signaled with the Reset bit of TS2 ordered sets (7.4.2): a DS directed to Hot
        # Reset in U0 transitions to Recovery (7.5.6.2), sends TS2 with Reset=1 in
        # Recovery.Configuration and enters Hot Reset from Recovery.Idle (7.5.10.5.2).
        def generator(dut):
            yield dut.ds_port.eq(1)
            yield from self.goto(dut, "U0")
            yield dut.hot_reset_req.eq(1)
            yield from self.expect(dut, "Recovery.Active", 4)
            self.assertEqual((yield dut.tx_ts2_reset), 0)
            yield from pulse(dut.ts_rx_ts1)
            yield from pulse(dut.ts_tx_done)
            yield from self.expect(dut, "Recovery.Configuration")
            self.assertEqual((yield dut.ts_tx_ts2), 1)
            self.assertEqual((yield dut.tx_ts2_reset), 1)
            yield from pulse(dut.ts_rx_ts2)
            yield from pulse(dut.ts_tx_done)
            yield from pulse(dut.ts_tx_done)
            yield from self.expect(dut, "HotReset.Active", 8)
            yield
            self.assertEqual((yield dut.tx_ts2_reset), 1)
        run(new_dut(), generator)

    def test_ds_hot_reset_from_u1_u2_through_recovery(self):
        # DS-directed PORT_RESET in U1/U2: LFPS exit handshake -> Recovery -> Recovery.Idle -> Hot
        # Reset (7.4.2).
        for ux, rx in [("U1", "lfps_rx_exit_u1"), ("U2", "lfps_rx_exit_u2")]:
            with self.subTest(state=ux):
                def generator(dut):
                    yield dut.ds_port.eq(1)
                    yield from self.goto(dut, ux)
                    yield dut.hot_reset_req.eq(1)
                    yield
                    yield getattr(dut, rx).eq(1)
                    yield from self.expect(dut, "Recovery.Active", cycles(2e-3))
                    yield getattr(dut, rx).eq(0)
                    yield from pulse(dut.ts_rx_ts1)
                    yield from pulse(dut.ts_tx_done)
                    yield from self.expect(dut, "Recovery.Configuration")
                    yield from pulse(dut.ts_rx_ts2)
                    yield from pulse(dut.ts_tx_done)
                    yield from pulse(dut.ts_tx_done)
                    yield from self.expect(dut, "HotReset.Active", 8)
                run(new_dut(), generator)

# Loopback (USB 3.2 7.5.11) ------------------------------------------------------------------------

class TestLTSSMLoopback(unittest.TestCase, LTSSMDriver):
    def test_loopback_master_exit(self):
        # Master: Loopback exit LFPS (U2 exit timing) -> handshake -> Rx.Detect (7.5.11.3.2).
        def generator(dut):
            yield from self.goto(dut, "Loopback.Master")
            self.assertTrue((yield from stay_in(dut, "Loopback", 16)))
            yield dut.directed_loopback.eq(0)
            yield from self.expect(dut, "Loopback.Exit", 4)
            self.assertEqual((yield dut.lfps_tx_exit_u2), 1)
            self.assertEqual((yield dut.loopback),        1)
            yield dut.lfps_rx_exit_u2.eq(1) # Slave response.
            yield from self.expect(dut, "Polling.ExitToRxDetect", cycles(2e-3))
            yield dut.lfps_rx_exit_u2.eq(0)
            yield from self.expect(dut, "Polling.LFPS", 16)
        run(new_dut(), generator)

    def test_loopback_slave_exit(self):
        # Slave: master's Loopback exit LFPS -> respond -> Rx.Detect.
        def generator(dut):
            yield from self.goto(dut, "Loopback")
            self.assertTrue((yield from stay_in(dut, "Loopback", 16)))
            yield dut.lfps_rx_exit_u2.eq(1)
            yield from self.expect(dut, "Loopback.Exit", 4)
            yield
            self.assertEqual((yield dut.lfps_tx_exit_u2), 1)
            yield from self.expect(dut, "Polling.ExitToRxDetect", cycles(2e-3))
            yield dut.lfps_rx_exit_u2.eq(0)
        run(new_dut(), generator)

    def test_loopback_slave_ignores_u1_exit_lfps(self):
        def generator(dut):
            yield from self.goto(dut, "Loopback")
            yield dut.lfps_rx_exit_u1.eq(1)
            self.assertTrue((yield from stay_in(dut, "Loopback", 32)))
        run(new_dut(), generator)

    def test_loopback_exit_timeout(self):
        # tLoopbackExitTimeout (2ms) without handshake -> eSS.Inactive.
        def generator(dut):
            yield from self.goto(dut, "Loopback.Exit")
            n = yield from wait_state(dut, "eSS.Inactive", cycles(2e-3) + 16)
            self.assertIsNotNone(n)
            self.assertLessEqual(n, cycles(2e-3) + 2)
        run(new_dut(), generator)

    def test_loopback_lfps_polling_escape(self):
        # Sandbox escape: partner restarted training.
        def generator(dut):
            yield from self.goto(dut, "Loopback")
            yield dut.lfps_rx_polling.eq(1)
            yield from self.expect(dut, "Polling.ExitToRxDetect", 4)
        run(new_dut(), generator)

# Compliance (USB 3.2 7.5.5) -----------------------------------------------------------------------

class TestLTSSMCompliance(unittest.TestCase, LTSSMDriver):
    def test_compliance_counts_ping(self):
        # Ping.LFPS advances the compliance pattern (count only in the sandbox).
        def generator(dut):
            yield from self.goto(dut, "Compliance")
            self.assertEqual((yield dut.compliance_ping_count), 0)
            for i in range(3):
                yield from pulse(dut.lfps_rx_ping)
                yield
                self.assertEqual((yield dut.compliance_ping_count), i + 1)
            self.assertEqual((yield from state(dut)), "Compliance")
        run(new_dut(COMPLIANCE_SCALE), generator)

    def test_compliance_only_warm_reset_exits(self):
        # Only Warm Reset exits Compliance (7.5.5.2): training LFPS, directed Recovery/Hot Reset,
        # LFPS exit bursts and TS are ignored.
        for ds_port in [0, 1]:
            with self.subTest(ds_port=ds_port):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield dut.compliance_enable.eq(1)
                    yield from self.goto(dut, "Compliance")
                    for name in ["lfps_rx_polling", "directed_recovery", "hot_reset_req",
                                 "lfps_rx_exit_u1", "lfps_rx_exit_u2", "lfps_rx_wakeup_u3",
                                 "ts_rx_ts1", "ts_rx_ts2", "rx_ts2_reset", "rx_ts2_loopback",
                                 "rx_idle8"]:
                        yield getattr(dut, name).eq(1)
                    self.assertTrue((yield from stay_in(dut, "Compliance", 64)))
                run(new_dut(COMPLIANCE_SCALE), generator)

    @unittest.expectedFailure
    def test_ufp_polling_lfps_timeout_after_link_up_is_not_compliance(self):
        # SPEC: Polling.LFPS -> Compliance Mode on the 360ms timeout only if the port has not yet
        # successfully completed Polling.LFPS since power-on / Warm Reset (first-time entry,
        # 7.5.4.3.2); otherwise the port goes to Rx.Detect (DS) / eSS.Disabled (UFP), never to
        # Compliance. The implementation has no such qualifier: a UFP that was in U0 and re-trains
        # (e.g. after eSS.Inactive) enters Compliance when the partner is silent for 360ms.
        def generator(dut):
            yield from self.goto(dut, "eSS.Inactive") # Was in U0 before.
            yield from self.expect(dut, "Polling.LFPS", cycles(12e-3, dut.scale) + 32)
            yield dut.rx_idle8.eq(0)
            n = yield from wait_state(dut, "Compliance", cycles(360e-3, dut.scale) + 32)
            self.assertIsNone(n)
        run(new_dut(), generator)

# U-states misc (USB 3.2 7.5.7 - 7.5.9) ------------------------------------------------------------

class TestLTSSMUStates(unittest.TestCase, LTSSMDriver):
    def test_ux_lfps_polling_escape(self):
        # Sandbox escape: partner restarted training (Polling.LFPS) -> Polling.Entry.
        for ux in ["U1", "U2", "U3"]:
            with self.subTest(state=ux):
                def generator(dut):
                    yield from self.goto(dut, ux)
                    yield dut.lfps_rx_polling.eq(1)
                    yield from self.expect(dut, "Polling.Entry", 4)
                run(new_dut(), generator)

    def test_ux_exit_handshake_to_recovery(self):
        # U1/U2 exit (initiator and responder) and U3 wakeup (DS initiator / UFP responder)
        # LFPS handshakes -> Recovery.
        cases = [
            # (ux,  ds_port, req,             rx)
            ("U1", 0, "exit_u1_req",   "lfps_rx_exit_u1"),
            ("U1", 0, None,            "lfps_rx_exit_u1"),
            ("U2", 0, "exit_u2_req",   "lfps_rx_exit_u2"),
            ("U2", 1, None,            "lfps_rx_exit_u2"),
            ("U3", 1, "wakeup_u3_req", "lfps_rx_wakeup_u3"),
            ("U3", 0, None,            "lfps_rx_wakeup_u3"),
        ]
        for ux, ds_port, req, rx in cases:
            with self.subTest(state=ux, ds_port=ds_port, initiator=req is not None):
                def generator(dut):
                    yield dut.ds_port.eq(ds_port)
                    yield from self.goto(dut, ux)
                    for _ in range(4):
                        yield
                    if req is not None:
                        yield from pulse(getattr(dut, req))
                        yield
                    yield getattr(dut, rx).eq(1)
                    yield from self.expect(dut, "Recovery.Active", cycles(10e-3))
                    yield getattr(dut, rx).eq(0)
                    yield
                    self.assertEqual((yield dut.recovery), 1)
                run(new_dut(), generator)

    def test_u1_ignores_u3_wakeup_lfps(self):
        # Each U-state only listens to its own exit burst class: a U3 wakeup burst seen in U1 must
        # not start a U1 exit handshake.
        def generator(dut):
            yield from self.goto(dut, "U1")
            yield dut.lfps_rx_wakeup_u3.eq(1)
            self.assertTrue((yield from stay_in(dut, "U1", 16)))
            self.assertEqual((yield dut.lfps_tx_exit_u1), 0)
        run(new_dut(), generator)

if __name__ == "__main__":
    unittest.main()
