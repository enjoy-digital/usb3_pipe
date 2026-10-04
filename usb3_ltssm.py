#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from usb3_pipe.lfps import (U1ExitLFPSBurst, U2ExitLFPSBurst, U3WakeupLFPSBurst,
    exit_burst_cycles, exit_rx_quiet_cycles,
    exit_u1_min_cycles, exit_u2_min_cycles, exit_u3_min_cycles)

# Link Training and Status State Machine -----------------------------------------------------------

# USB 3.2 Rev 1.1 references (PDF page = printed page + 31):
# - Section 7.5   (LTSSM states, PDF p190-p236): Polling.Idle p214-217, U1 p220-222, U2 p223,
#   U3 p224, Compliance p218-219, Loopback p229-232, Hot Reset p233-235, Recovery p226-229,
#   SS.Inactive p191 (Table 7-12 timeouts).
# - Section 7.2.4.2 (Link Power Management, PDF p173-177): LGO_Ux/LAU/LXU/LPMA handshake and
#   PM_LC_TIMER (4us) / PM_ENTRY_TIMER (8us) / Ux_EXIT_TIMER (6ms) / U1_MIN_RESIDENCY (3us).
# - Section 6.9   (LFPS, PDF p130-134): Table 6-30 patterns, Table 6-31 handshake timings.
# Sandbox simplifications:
# - Rx.Detect family is not modelled (Polling.ExitToRxDetect is a transient state towards
#   Polling.Entry, i.e. instant re-training).
# - TS2 bit-fields (Reset/Loopback) are not decoded from the TS unit: the rx_ts2_reset /
#   rx_ts2_loopback / rx_idle8 inputs are virtual interconnect proxies driven by the sim.
# - 20ns gap rule / electrical parameters are not modelled. Concurrent U1 exit is modelled
#   (both partners send a full 900ns burst).
# - U3 DS failure retry limited to 2 retries then eSS.Inactive.

@ResetInserter()
class USB3LTSSM(LiteXModule):
    """ Link Training and Status State Machine (section 7.5)"""
    def __init__(self, sys_clk_freq, with_timers=True, timer_scale=1, u2_inactivity_timeout=1e-3):
        # Status -----------------------------------------------------------------------------------
        self.u0                 = Signal()
        self.recovery           = Signal()
        self.rx_ready           = Signal()
        self.tx_ready           = Signal()
        self.exit_to_compliance = Signal()
        self.exit_to_rx_detect  = Signal()
        self.u1                 = Signal()
        self.u2                 = Signal()
        self.u3                 = Signal()
        self.compliance         = Signal()
        self.loopback           = Signal()
        self.hot_reset          = Signal()
        self.ess_inactive       = Signal()
        self.polling_idle       = Signal()

        # SerDes control ---------------------------------------------------------------------------
        self.serdes_rx_align    = Signal()         # o
        self.serdes_rx_polarity = Signal(reset=0)  # o

        # LFPS control/status ----------------------------------------------------------------------
        self.lfps_tx_polling = Signal()     # o
        self.lfps_tx_idle    = Signal()     # o
        self.lfps_rx_polling = Signal()     # i
        self.lfps_tx_count   = Signal(16)   # i

        self.lfps_tx_ping      = Signal()   # o
        self.lfps_tx_exit_u1   = Signal()   # o
        self.lfps_tx_exit_u2   = Signal()   # o
        self.lfps_tx_wakeup_u3 = Signal()   # o
        self.lfps_tx_reset     = Signal()   # o

        self.lfps_rx_ping      = Signal()   # i
        self.lfps_rx_exit_u1   = Signal()   # i
        self.lfps_rx_exit_u2   = Signal()   # i
        self.lfps_rx_wakeup_u3 = Signal()   # i
        self.lfps_rx_reset     = Signal()   # i

        # TS control/status ------------------------------------------------------------------------
        self.ts_rx_enable = Signal()  # o
        self.ts_tx_enable = Signal()  # o
        self.ts_tx_tseq   = Signal()  # o
        self.ts_tx_ts1    = Signal()  # o
        self.ts_tx_ts2    = Signal()  # o

        self.ts_rx_ts1     = Signal() # i
        self.ts_rx_ts1_inv = Signal() # i
        self.ts_rx_ts2     = Signal() # i
        self.ts_tx_done    = Signal() # i

        # Role & enables (7.5) ---------------------------------------------------------------------
        self.ds_port           = Signal(reset=0) # i: 1 = DS/host role, 0 = UFP/device role
        self.compliance_enable = Signal(reset=0) # i: DS Compliance migration path enable (7.5.4.3)

        # PM handshake interface (7.2.4.2, LCMD transport owned by the Link Layer) ------------------
        self.lgo_u1 = Signal() # i: partner LGO_U1 decoded (pulse)
        self.lgo_u2 = Signal() # i: partner LGO_U2 decoded (pulse)
        self.lgo_u3 = Signal() # i: partner LGO_U3 decoded (pulse)
        self.lau    = Signal() # i: our LGO_Ux accepted (LAU received)
        self.lxu    = Signal() # i: our LGO_Ux rejected (LXU received) -> stay in U0
        self.req_u1 = Signal() # i: upper layer wants U1
        self.req_u2 = Signal() # i: upper layer wants U2
        self.req_u3 = Signal() # i: upper layer wants U3 (DS only)

        self.tx_lgo_u1 = Signal() # o: pulse, Link Layer should send LGO_U1
        self.tx_lgo_u2 = Signal() # o
        self.tx_lgo_u3 = Signal() # o
        self.tx_lau    = Signal() # o: pulse, Link Layer should send LAU (accept)
        self.tx_lxu    = Signal() # o: pulse, Link Layer should send LXU (reject)
        self.tx_lpma   = Signal() # o: pulse, Link Layer should send LPMA (ack-ack)

        # Link activity (tU0RecoveryTimeout trigger, 7.5.6.2) ---------------------------------------
        self.link_activity = Signal(reset=1) # i: 1 when link active in current <1ms window

        # Upper layer directs (7.5) -----------------------------------------------------------------
        self.directed_recovery = Signal()        # i: directed to Recovery
        self.directed_loopback = Signal()        # i: directed as Loopback master
        self.loopback_capable  = Signal(reset=0) # i: loopback master capability
        self.hot_reset_req     = Signal()        # i: DS directed PORT_RESET (Hot Reset)
        self.warm_reset_req    = Signal()        # i: DS directed BH_PORT_RESET (Warm Reset LFPS)
        self.exit_u1_req       = Signal()        # i: initiate U1 exit (upper layer/traffic)
        self.exit_u2_req       = Signal()        # i: initiate U2 exit
        self.wakeup_u3_req     = Signal()        # i: initiate U3 wakeup (DS)

        # U2 inactivity (PORT_U2_TIMEOUT LMP, 0 = disabled, 7.5.7.2) --------------------------------
        self.u2_inactivity_enable = Signal(reset=0) # i

        # TS2 bit-plane (sandbox: bit fields driven by sim-level proxies) ---------------------------
        self.rx_ts2_reset    = Signal() # i: partner TS2 Reset=1 seen (Hot Reset)
        self.rx_ts2_loopback = Signal() # i: partner TS2 Loopback=1 seen (slave entry)
        self.tx_ts2_reset    = Signal() # o: TS unit should set Reset bit in TS2
        self.tx_ts2_loopback = Signal() # o: TS unit should set Loopback bit in TS2
        self.rx_idle8        = Signal(reset=1) # i: Idle handshake done (8 received + 16 sent model)
                                             #     reset=1: short path when no proxy is connected

        # Compliance testability (7.5.4.3) ----------------------------------------------------------
        self.compliance_ping_count = Signal(16) # o: Ping.LFPS received count in Compliance

        # # #

        # Timer scaling: all timeouts divided by timer_scale (sim acceleration), specification
        # exact values for timer_scale=1.
        scaled = lambda t: max(1, int(t*sys_clk_freq)//timer_scale)

        tx_lfps_count   = Signal(16)
        rx_lfps_seen    = Signal()
        rx_ts1_seen     = Signal()
        rx_ts1_inv_seen = Signal()
        rx_ts2_seen     = Signal()
        rx_lb_seen      = Signal() # partner TS2 Loopback bit latched during training/recovery
        self.lb_pending = lb_pending = Signal() # directed loopback via Recovery (TS2 Loopback bit)
        loopback_master = Signal() # Loopback role
        hr_seen         = Signal() # partner TS2 Reset bit seen in HotReset.Active

        # 360ms Timer (tPollingLFPSTimeout, Table 7-12) ----------------------------------------------
        self._360_ms_timer = _360_ms_timer = WaitTimer(scaled(360e-3))

        # 12ms Timer (tPollingActive/tPollingConfiguration/tRecoveryActive/tHotResetActive) ----------
        self._12_ms_timer = _12_ms_timer = WaitTimer(scaled(12e-3))

        # 6ms Timer (tRecoveryConfigurationTimeout) --------------------------------------------------
        self._6_ms_timer = _6_ms_timer = WaitTimer(scaled(6e-3))

        # 2ms Timer (tPollingIdleTimeout) ------------------------------------------------------------
        self.t_polling_idle = t_polling_idle = WaitTimer(scaled(2e-3))

        # 1ms Timer (tU0RecoveryTimeout) -------------------------------------------------------------
        self.t_u0_recovery = t_u0_recovery = WaitTimer(scaled(1e-3))

        # 2ms Timer (tRecoveryIdleTimeout) -----------------------------------------------------------
        self.t_recov_idle = t_recov_idle = WaitTimer(scaled(2e-3))

        # 2ms Timer (tHotResetExitTimeout) -----------------------------------------------------------
        self.t_hotreset_exit = t_hotreset_exit = WaitTimer(scaled(2e-3))

        # 6ms Timer (Ux_EXIT_TIMER, 7.2.4.2.1) -------------------------------------------------------
        self.t_ux_exit = t_ux_exit = WaitTimer(scaled(6e-3))

        # 4us Timer (PM_LC_TIMER) --------------------------------------------------------------------
        self.t_pm_lc = t_pm_lc = WaitTimer(scaled(4e-6))

        # 8us Timer (PM_ENTRY_TIMER) -----------------------------------------------------------------
        self.t_pm_entry = t_pm_entry = WaitTimer(scaled(8e-6))

        # 1ms Timer (sandbox PORT_U2_TIMEOUT value) --------------------------------------------------
        self.t_u2_inact = t_u2_inact = WaitTimer(scaled(u2_inactivity_timeout))

        # 12ms Timer (teSSInactiveQuietTimeout) ------------------------------------------------------
        self.t_ss_quiet = t_ss_quiet = WaitTimer(scaled(12e-3))

        # 100ms Timer (tU3WakeupRetryDelay) ----------------------------------------------------------
        self.t_u3_retry = t_u3_retry = WaitTimer(scaled(100e-3))

        # Transient RxDetect equivalent timer (sandbox) -----------------------------------------------
        self.t_rx_detect = t_rx_detect = WaitTimer(max(1, 64//timer_scale))

        timers = [
            _360_ms_timer, _12_ms_timer, _6_ms_timer,
            t_polling_idle, t_u0_recovery, t_recov_idle, t_hotreset_exit,
            t_ux_exit, t_pm_lc, t_pm_entry, t_u2_inact, t_ss_quiet, t_u3_retry, t_rx_detect,
        ]

        # LFPS Exit Handshake (6.9.2, shared by U1/U2/Loopback exit and U3 wakeup) -------------------
        # Modes: 0=none, 1=U1 exit, 2=U2/Loopback exit, 3=U3 wakeup.
        # Each partner sends a full burst (concurrent-safe): success = own burst completed and
        # partner burst seen; failure = tNoLFPSResponseTimeout (2ms, 2ms, 10ms) without success.
        # Burst lengths share the same source as the partner burst-checker detection windows
        # (usb3_pipe.lfps helpers): burst = max(scaled typ, detect_min + RX_DETECT_LATENCY), so
        # the responder still sees the initiator burst in flight at any timer_scale (Table 6-31
        # order preserved by the floor convention below). The tNoLFPSResponse
        # guard below (2ms/2ms/10ms) is >= the responder's own burst + detection latency.
        hs_mode      = Signal(2)
        self.hs_run  = hs_run = Signal()
        hs_send      = Signal() # burst ongoing
        hs_rx        = Signal() # partner burst seen
        hs_cnt       = Signal(32)
        hs_gcnt      = Signal(32)
        hs_done_ok   = Signal() # pulse
        hs_done_err  = Signal() # pulse

        hs_burst_len = Array([0,
            exit_burst_cycles(sys_clk_freq, timer_scale, U1ExitLFPSBurst,
                              exit_u1_min_cycles(sys_clk_freq, timer_scale)),
            exit_burst_cycles(sys_clk_freq, timer_scale, U2ExitLFPSBurst,
                              exit_u2_min_cycles(sys_clk_freq, timer_scale)),
            exit_burst_cycles(sys_clk_freq, timer_scale, U3WakeupLFPSBurst,
                              exit_u3_min_cycles(sys_clk_freq, timer_scale)),
        ])
        hs_guard_len = Array([0, scaled(2e-3),   scaled(2e-3),   scaled(10e-3)])
        hs_rx_pulse  = Array([0, self.lfps_rx_exit_u1, self.lfps_rx_exit_u2, self.lfps_rx_wakeup_u3])

        self.sync += [
            hs_done_ok.eq(0),
            hs_done_err.eq(0),
            If(hs_run,
                hs_gcnt.eq(hs_gcnt + 1),
                If(hs_send,
                    hs_cnt.eq(hs_cnt + 1),
                    If((hs_cnt + 1) >= hs_burst_len[hs_mode],
                        hs_send.eq(0)
                    )
                ),
                If(hs_rx_pulse[hs_mode],
                    hs_rx.eq(1)
                ),
                If(hs_rx & ~hs_send,
                    hs_done_ok.eq(1),
                    hs_run.eq(0),
                ).Elif((hs_gcnt + 1) >= hs_guard_len[hs_mode],
                    hs_done_err.eq(1),
                    hs_run.eq(0),
                )
            ).Else(
                hs_cnt.eq(0),
                hs_gcnt.eq(0),
                hs_rx.eq(0),
                hs_send.eq(0),
            )
        ]

        # Warm Reset burst (6.9: DS only originator, UFP responds until DS stops) --------------------
        wr_run  = Signal() # Warm Reset burst ongoing
        wr_orig = Signal() # 1 = originator (DS), 0 = responder (UFP)
        wr_rx   = Signal()
        wr_cnt  = Signal(32)
        wr_gcnt = Signal(32)
        wr_done = Signal() # pulse: burst phase finished

        self.sync += [
            wr_done.eq(0),
            If(wr_run,
                If(wr_orig,
                    wr_cnt.eq(wr_cnt + 1),
                    If((wr_cnt + 1) >= scaled(100e-3), # tReset typ 100ms (80-120ms)
                        wr_done.eq(1),
                        wr_run.eq(0)
                    )
                ).Else(
                    If(self.lfps_rx_reset,
                        wr_rx.eq(1)
                    ),
                    wr_gcnt.eq(wr_gcnt + 1),
                    If(wr_rx & ~self.lfps_rx_reset,      # DS stopped -> stop responding
                        wr_done.eq(1),
                        wr_run.eq(0)
                    ).Elif((wr_gcnt + 1) >= scaled(200e-3), # guard (max burst 120ms)
                        wr_done.eq(1),
                        wr_run.eq(0)
                    )
                )
            ).Else(
                wr_cnt.eq(0),
                wr_gcnt.eq(0),
                wr_rx.eq(0),
            )
        ]

        # U1 DS Ping timeout hand counter (tU1PingTimeout 300ms, reset on Ping reception, 7.5.7.2) ---
        self.u1_ping_cnt = u1_ping_cnt = Signal(32)
        u1_ping_lim  = scaled(300e-3)
        self.u1_ping_done = u1_ping_done = u1_ping_cnt >= u1_ping_lim

        # U1 UFP min residency (U1_MIN_RESIDENCY 3us, 7.2.4.2.7) -------------------------------------
        self.u1_res_cnt = u1_res_cnt = Signal(32)
        u1_res_lim    = scaled(3e-6)
        self.u1_residency_done = u1_residency_done = u1_res_cnt >= u1_res_lim

        # Hot Reset Active reset-done delay (sandbox: US clears Reset bit after own reset) -----------
        self.hr_cnt = hr_cnt = Signal(16)
        # Floor convention: the Reset=1 TS2 window must outlast the
        # partner's HotReset.Active entry lag (~2 cycles via the TS2 plane at any scale), or the
        # partner never latches hr_seen and both sides fall to the 12ms timeout. 16 cycles gives
        # an 8x margin over that lag at scale>=16; scale=1 keeps the exact 256.
        hr_delay = max(16, 256//timer_scale)

        # U3 wakeup retry (DS, 7.2.4.2.4: 2 retries then eSS.Inactive) -------------------------------
        u3_wakeup_pending = Signal()
        u3_retry_cnt      = Signal(2)
        u3_retry_wait     = Signal()
        u3_entry_cnt      = Signal(2) # consecutive U3 entry PM_LC failures (7.2.4.2.4)

        # PM handshake registers (7.2.4.2) -----------------------------------------------------------
        self.pm_kind = pm_kind = Signal(2)  # requested Ux: 1/2/3
        self.pm_sent_lgo = pm_sent_lgo = Signal() # LGO sent, LAU/LXU pending (PM_LC_TIMER armed)
        self.pm_acc_kind = pm_acc_kind = Signal(2) # partner request accepted (LAU sent)
        pm_lgo_fire  = Signal(2)
        pm_lau_fire  = Signal()
        pm_lxu_fire  = Signal()
        pm_lpma_fire = Signal()

        # Ux_EXIT_TIMER arming (armed on U1/U2/Loopback exit LFPS, disarmed on U0 entry) -------------
        ux_armed = Signal()

        # FSM ----------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="Polling.Entry")

        # Single-burst qualification of partner exit/wakeup bursts (Table 6-30/6-31) -----------------
        # A detected burst only counts as a handshake REQUEST once it ended and the line stayed
        # quiet for one (softened) Polling repeat period: a Polling.LFPS train bursts again within
        # that window and re-arms the qualifier, so a partner restarting training while we are in
        # U1/U2/U3 goes through the lfps_rx_polling escape instead of a bogus exit handshake. The
        # commit delay is far inside the initiator's tNoLFPSResponseTimeout at every timer_scale.
        # The handshake originators (exit_u*_req / wakeup_u3_req) are not qualified: they know.
        # The qualified level is only raised while in a handshake-capable state (U1/U2/U3/Loopback:
        # a qualification reached in any other state is dropped) and is consumed by the handshake
        # start. Wired in finalize() (needs the frozen FSM encoding).
        self.rxq_state      = Signal(2)        # 0=idle, 1=burst, 2=quiet
        self.rxq_cnt        = Signal(16)
        self.rx_burst_ok    = Signal()         # level: one qualified burst, waits for use
        self._rx_detect_any = self.lfps_rx_exit_u1 | self.lfps_rx_exit_u2 | self.lfps_rx_wakeup_u3
        self._rx_quiet_lim  = exit_rx_quiet_cycles(sys_clk_freq, timer_scale)

        # Timer/outputs preambles --------------------------------------------------------------------
        timers_off = [t.wait.eq(0) for t in timers]

        # Entry State ------------------------------------------------------------------------------
        fsm.act("Polling.Entry", # 0.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(tx_lfps_count,  16),
            NextValue(rx_lfps_seen,    0),
            NextValue(rx_ts1_seen,     0),
            NextValue(rx_ts1_inv_seen, 0),
            NextValue(rx_ts2_seen,     0),
            NextValue(rx_lb_seen,      0),
            NextState("Polling.LFPS"),
        )

        # LFPS State (7.5.4.3) ---------------------------------------------------------------------
        fsm.act("Polling.LFPS", # 1.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            _360_ms_timer.wait.eq(with_timers),
            self.lfps_tx_polling.eq(1),
            If(_360_ms_timer.done,
                # 360ms timeout: Compliance for UFP (always enabled) and DS when directed
                # (compliance_enable); DS without enable retries training (7.5.4.3).
                If(self.ds_port & ~self.compliance_enable,
                    NextState("Polling.ExitToRxDetect")
                ).Else(
                    NextState("Compliance")
                )
            ).Elif(self.lfps_tx_count >= tx_lfps_count,
                If(self.lfps_rx_polling & ~rx_lfps_seen,
                    NextValue(rx_lfps_seen, 1),
                    NextValue(tx_lfps_count, self.lfps_tx_count + 4)
                ),
                If(rx_lfps_seen,
                    NextState("Polling.RxEQ"),
                )
            )
        )

        # RxEQ State (7.5.4.4) ---------------------------------------------------------------------
        fsm.act("Polling.RxEQ", # 2.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.serdes_rx_align.eq(1),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_tseq.eq(1),
            If(self.ts_tx_done,
                NextState("Polling.Active")
            ),
        )

        # Active State (7.5.4.5) -------------------------------------------------------------------
        fsm.act("Polling.Active", # 3.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts1.eq(1),

            # Latch what we saw from the host (rx_ts1 / rx_ts1_inv are pulses).
            NextValue(rx_ts1_seen,     rx_ts1_seen     | self.ts_rx_ts1),
            NextValue(rx_ts1_inv_seen, rx_ts1_inv_seen | self.ts_rx_ts1_inv),

            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),

            If(self.ts_tx_done & (rx_ts1_seen | rx_ts1_inv_seen),
                # Polarity (7.5.4.5): TS1_INV evidence (latched during this training attempt)
                # flips the polarity, normal TS1 evidence keeps the current one. On a re-train
                # the previously negotiated polarity is exactly why TS1 decodes normally, so it
                # must be kept (forcing it back to 0 makes every re-training miss TS2 and stall
                # in Polling.Configuration until the next retry).
                If(rx_ts1_inv_seen & ~self.recovery,
                    NextValue(self.serdes_rx_polarity, ~self.serdes_rx_polarity)
                ),
                NextState("Polling.Configuration")
            ),
        )

        # Configuration State (7.5.4.6) ------------------------------------------------------------
        fsm.act("Polling.Configuration", # 4.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),
            NextValue(rx_lb_seen,  rx_lb_seen  | self.rx_ts2_loopback),
            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),
            If(self.ts_tx_done,
                If(rx_ts2_seen,
                    # 8 consecutive TS2 received + 16 sent -> Polling.Idle (7.5.4.9.2).
                    NextState("Polling.Idle")
                )
            )
        )

        # Polling.Idle State (7.5.4.10) --------------------------------------------------------------
        fsm.act("Polling.Idle", # 5.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.polling_idle.eq(1),
            t_polling_idle.wait.eq(with_timers),
            If(rx_lb_seen | self.rx_ts2_loopback,
                # Loopback bit in TS2: slave entry, forced (7.5.4.10.2). Checked before the idle
                # handshake: a directed/bit-configured partner must not bounce through U0.
                NextValue(loopback_master, 0),
                NextState("Loopback")
            ).Elif(self.directed_loopback & self.loopback_capable,
                # Directed loopback master (7.5.4.10.2).
                NextValue(loopback_master, 1),
                NextState("Loopback")
            ).Elif(self.rx_ts2_reset & ~self.ds_port,
                # Reset bit in TS2: US to Hot Reset (7.5.4.10.2).
                NextValue(hr_seen, 0),
                NextState("HotReset.Active")
            ).Elif(self.hot_reset_req & self.ds_port,
                # DS directed Hot Reset (7.5.4.10.2).
                NextValue(hr_seen, 0),
                NextState("HotReset.Active")
            ).Elif(self.warm_reset_req & self.ds_port,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 1),
                NextState("WarmReset")
            ).Elif(self.rx_idle8,
                # Idle handshake done (8 Idle received + 16 Idle sent model) -> U0.
                NextState("U0")
            ).Elif(t_polling_idle.done,
                # tPollingIdleTimeout (2ms): Rx.Detect (sandbox: transient re-train).
                NextState("Polling.ExitToRxDetect")
            )
        )

        # U0 State -----------------------------------------------------------------------------------
        fsm.act("U0", # 6.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.u0.eq(1),
            self.rx_ready.eq(1),
            self.tx_ready.eq(1),
            # U0 is a TS-receiving state (7.5.6.2 trigger 2: TS1 in U0 -> Recovery, also the
            # directed-Loopback TS1 face): keep RX word re-alignment live. The RXWordAligner
            # re-latches its rotation only while enabled (usb3_pipe/serdes.py:
            # word_aligner.enable == rx_align); a rotation latched during earlier
            # LFPS/Recovery/SKP-removal traffic otherwise persists frozen through U0, so every
            # incoming TS1 mismatches the checker and the port goes deaf to trigger 2 (observed
            # at timer_scale=100: first-U0 detects, any retrained-U0 does not).
            self.serdes_rx_align.eq(1),
            t_pm_lc.wait.eq(pm_sent_lgo),
            t_pm_entry.wait.eq(pm_acc_kind != 0),
            t_u0_recovery.wait.eq(with_timers & ~self.link_activity),
            NextValue(self.recovery, 0),
            NextValue(lb_pending, 0),
            NextValue(ux_armed, 0),
            NextValue(rx_lb_seen, 0),
            NextValue(u3_wakeup_pending, 0),
            NextValue(u3_retry_cnt, 0),
            NextValue(u3_retry_wait, 0),
            NextValue(pm_lgo_fire,  0),
            NextValue(pm_lau_fire,  0),
            NextValue(pm_lxu_fire,  0),
            NextValue(pm_lpma_fire, 0),

            # Warm Reset LFPS detect (6.9: UFP must detect it in any state, responds until DS stops).
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            # Hot Reset: US forced by TS2 Reset bit / DS directed (7.5.6.2).
            ).Elif(self.rx_ts2_reset | (self.hot_reset_req & self.ds_port),
                NextValue(hr_seen, 0),
                NextState("HotReset.Active")
            # Real Recovery triggers (7.5.6.2): TS1 detect (trigger 2), tU0RecoveryTimeout (4),
            # directed (3).
            ).Elif(self.ts_rx_ts1, # FIXME: trigger 2 of 7.5.6.2 (legacy bring-up trigger).
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(t_u0_recovery.done,
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(self.directed_recovery,
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(self.directed_loopback & self.loopback_capable,
                # Directed loopback from U0: via Recovery with TS2 Loopback bit (7.5.11).
                NextValue(lb_pending, 1),
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(self.warm_reset_req & self.ds_port,
                # DS directed Warm Reset: send Warm Reset LFPS burst (6.9).
                NextValue(wr_run,  1),
                NextValue(wr_orig, 1),
                NextState("WarmReset")
            # PM: accept partner request (UFP shall not reject U3, 7.2.4.2.4).
            ).Elif(self.lgo_u3 & ~pm_sent_lgo & (pm_acc_kind == 0),
                NextValue(pm_acc_kind, 3),
                NextValue(pm_lau_fire, 1)
            ).Elif(self.lgo_u1 & ~pm_sent_lgo & (pm_acc_kind == 0),
                NextValue(pm_acc_kind, 1),
                NextValue(pm_lau_fire, 1)
            ).Elif(self.lgo_u2 & ~pm_sent_lgo & (pm_acc_kind == 0),
                NextValue(pm_acc_kind, 2),
                NextValue(pm_lau_fire, 1)
            # PM concurrency (7.2.4.2.5, simplified): LGO sent + LGO received -> LXU, stay U0.
            ).Elif(pm_sent_lgo & (self.lgo_u1 | self.lgo_u2 | self.lgo_u3),
                NextValue(pm_sent_lgo, 0),
                NextValue(pm_lxu_fire, 1)
            # PM: requester trigger (conditions of 7.2.4.2.2 assumed met by upper layers).
            ).Elif(~pm_sent_lgo & (pm_acc_kind == 0) & (self.req_u1 | self.req_u2 | (self.req_u3 & self.ds_port)),
                If(self.req_u3 & self.ds_port,
                    NextValue(pm_kind, 3)
                ).Elif(self.req_u2,
                    NextValue(pm_kind, 2)
                ).Else(
                    NextValue(pm_kind, 1)
                ),
                NextValue(pm_lgo_fire, 1),
                NextValue(pm_sent_lgo, 1)
            ).Elif(pm_sent_lgo,
                # Requester resolution (7.2.4.2.3).
                If(self.lau,
                    NextValue(pm_sent_lgo, 0),
                    NextValue(pm_lpma_fire, 1),
                    NextValue(u3_entry_cnt, 0), # entry success breaks the failure streak
                    If(pm_kind == 1,
                        NextState("U1")
                    ).Elif(pm_kind == 2,
                        NextState("U2")
                    ).Else(
                        NextState("U3")
                    )
                ).Elif(self.lxu,
                    NextValue(pm_sent_lgo, 0), # rejected: remain in U0
                    NextValue(u3_entry_cnt, 0) # resolved handshake is not a timeout failure
                ).Elif(t_pm_lc.done,
                    # PM_LC_TIMER timeout -> Recovery (7.2.4.2.3). U3 entry (pm_kind==3)
                    # counts consecutive failures: the 3rd strike -> eSS.Inactive
                    # (7.2.4.2.4) instead of another Recovery round trip.
                    NextValue(pm_sent_lgo, 0),
                    If((pm_kind == 3) & (u3_entry_cnt == 2),
                        NextState("eSS.Inactive")
                    ).Else(
                        If(pm_kind == 3,
                            NextValue(u3_entry_cnt, u3_entry_cnt + 1)
                        ),
                        NextValue(self.recovery, 1),
                        NextValue(rx_ts1_seen,     0),
                        NextValue(rx_ts1_inv_seen, 0),
                        NextValue(rx_ts2_seen,     0),
                        NextState("Recovery.Active")
                    )
                )
            ).Elif(pm_acc_kind != 0,
                # Acceptor resolution: enter Ux on PM_ENTRY_TIMER timeout with no LPMA and no TS1
                # (7.2.4.2.3, spec-legal sandbox path: LPMA rx is not in the interface contract).
                If(t_pm_entry.done,
                    NextValue(pm_acc_kind, 0),
                    If(pm_acc_kind == 1,
                        NextState("U1")
                    ).Elif(pm_acc_kind == 2,
                        NextState("U2")
                    ).Else(
                        NextState("U3")
                    )
                ).Elif(self.ts_rx_ts1,
                    # LAU corrupted case: Recovery before PM_ENTRY timeout (7.2.4.2.3).
                    NextValue(pm_acc_kind, 0),
                    NextValue(self.recovery, 1),
                    NextValue(rx_ts1_seen,     0),
                    NextValue(rx_ts1_inv_seen, 0),
                    NextValue(rx_ts2_seen,     0),
                    NextState("Recovery.Active")
                )
            ).Elif(self.lfps_rx_polling, # FIXME: for bringup (legacy bring-up trigger).
                NextState("Polling.Entry")
            )
        )

        # U1 State (7.5.7) ----------------------------------------------------------------------------
        fsm.act("U1", # 7.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            t_ux_exit.wait.eq(ux_armed),
            self.u1.eq(1),
            t_u2_inact.wait.eq(with_timers & self.u2_inactivity_enable),
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            ).Elif(self.lfps_rx_polling,
                # Sandbox escape: partner restarted training (Rx.Detect absence equivalent).
                NextState("Polling.Entry")
            ).Elif(hs_done_ok,
                # U1 exit handshake success -> both partners to Recovery (6.9.2/7.2.4.2.7).
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(hs_done_err,
                # tNoLFPSResponseTimeout (2ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            ).Elif(t_u2_inact.done & self.u2_inactivity_enable,
                # PORT_U2_TIMEOUT: U1 -> U2 direct degradation (7.2.4.2.3).
                NextState("U2")
            ).Elif(self.ds_port & u1_ping_done,
                # tU1PingTimeout (300ms) without Ping -> Rx.Detect (7.5.7.2).
                NextState("Polling.ExitToRxDetect")
            ).Elif(~hs_run & self.rx_burst_ok,
                # Responder: partner's single exit burst qualified (6.9.2). hs_rx is pre-latched:
                # the qualified burst IS the partner request evidence.
                NextValue(hs_mode, 1),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(hs_rx,   1),
                NextValue(ux_armed, 1)
            ).Elif(~hs_run & ((self.exit_u1_req & (self.ds_port | u1_residency_done))
                              | (self.hot_reset_req & self.ds_port)),
                # U1 exit initiator (UFP only after U1_MIN_RESIDENCY). DS-directed
                # PORT_RESET exits through the same handshake (p189-190); the held
                # request completes the chain later in Recovery.Idle -> Hot Reset.
                NextValue(hs_mode, 1),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(ux_armed, 1)
            )
        )

        # U2 State (7.5.8) ----------------------------------------------------------------------------
        fsm.act("U2", # 8.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            t_ux_exit.wait.eq(ux_armed),
            self.u2.eq(1),
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            ).Elif(self.lfps_rx_polling,
                NextState("Polling.Entry") # sandbox escape
            ).Elif(hs_done_ok,
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(hs_done_err,
                NextState("eSS.Inactive")
            ).Elif(~hs_run & self.rx_burst_ok,
                # Responder: partner's single exit burst qualified (6.9.2), hs_rx pre-latched.
                NextValue(hs_mode, 2),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(hs_rx,   1),
                NextValue(ux_armed, 1)
            ).Elif(~hs_run & (self.exit_u2_req | (self.hot_reset_req & self.ds_port)),
                # U2 exit initiator; DS-directed PORT_RESET rides the same
                # handshake (p189-190) and completes in Recovery.Idle -> Hot Reset.
                NextValue(hs_mode, 2),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(ux_armed, 1)
            )
        )

        # U3 State (7.5.9) ----------------------------------------------------------------------------
        fsm.act("U3", # 9.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            self.u3.eq(1),
            t_u3_retry.wait.eq(u3_retry_wait),
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            ).Elif(self.lfps_rx_polling,
                NextState("Polling.Entry") # sandbox escape
            ).Elif((self.hot_reset_req | self.warm_reset_req) & self.ds_port,
                # PORT_RESET/BH_PORT_RESET directed in U3 -> Warm Reset
                # (Inband Reset selection, p189-190).
                NextValue(wr_run,  1),
                NextValue(wr_orig, 1),
                NextState("WarmReset")
            ).Elif(hs_done_ok,
                # U3 wakeup handshake success -> Recovery (7.2.4.2.7).
                NextValue(u3_wakeup_pending, 0),
                NextValue(u3_retry_cnt, 0),
                NextValue(u3_retry_wait, 0),
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(hs_done_err,
                If(self.ds_port,
                    If(u3_retry_cnt == 2,
                        # 3 consecutive failures -> eSS.Inactive (7.2.4.2.4).
                        NextState("eSS.Inactive")
                    ).Else(
                        NextValue(u3_retry_cnt, u3_retry_cnt + 1),
                        NextValue(u3_retry_wait, 1) # tU3WakeupRetryDelay (100ms)
                    )
                ).Else(
                    NextState("eSS.Inactive")
                )
            ).Elif(self.wakeup_u3_req & self.ds_port,
                NextValue(u3_wakeup_pending, 1)
            ).Elif(self.ds_port & u3_wakeup_pending & ~hs_run & ~u3_retry_wait,
                # DS initiates/retries wakeup (Ux_EXIT_TIMER not applied to U3, 7.2.4.2.1).
                NextValue(hs_mode, 3),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1)
            ).Elif(~self.ds_port & self.rx_burst_ok,
                # UFP responds to wakeup (qualified single burst, hs_rx pre-latched).
                NextValue(hs_mode, 3),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(hs_rx,   1)
            ).Elif(t_u3_retry.done,
                # tU3WakeupRetryDelay (100ms) elapsed: clear the wait so the pending wakeup is
                # re-initiated by the DS branch above on the next cycle (7.2.4.2.4). Without
                # this clear, u3_retry_wait stays set forever after the first
                # tNoLFPSResponseTimeout and the DS never retries (stuck in U3).
                NextValue(u3_retry_wait, 0)
            )
        )

        # Compliance State (7.5.4.3 entry / 7.5.5) ----------------------------------------------------
        fsm.act("Compliance", # 10.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.compliance.eq(1),
            self.exit_to_compliance.eq(1),
            If(self.lfps_rx_ping,
                # Ping.LFPS switches test pattern (sandbox: count only, design section 4).
                NextValue(self.compliance_ping_count, self.compliance_ping_count + 1)
            ),
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            ).Elif(self.warm_reset_req & self.ds_port,
                # Only Warm Reset exits Compliance (7.5.5.2).
                NextValue(wr_run,  1),
                NextValue(wr_orig, 1),
                NextState("WarmReset")
            )
        )

        # Loopback State (7.5.11) ---------------------------------------------------------------------
        fsm.act("Loopback", # 11.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            t_ux_exit.wait.eq(ux_armed),
            self.loopback.eq(1),
            If(self.lfps_rx_reset,
                NextValue(wr_run,  1),
                NextValue(wr_orig, 0),
                NextState("WarmReset")
            ).Elif(self.lfps_rx_polling,
                # Sandbox escape: partner restarted training after failed exit.
                NextState("Polling.ExitToRxDetect")
            ).Elif((self.hot_reset_req | self.warm_reset_req) & self.ds_port,
                # PORT_RESET/BH_PORT_RESET directed in Loopback -> Warm Reset
                # (Inband Reset selection, p189-190); outranks the master exit.
                NextValue(wr_run,  1),
                NextValue(wr_orig, 1),
                NextState("WarmReset")
            ).Elif(loopback_master & ~self.directed_loopback & ~hs_run,
                # Master exit directed: Loopback exit LFPS (timing of U2 exit, 7.5.11.2).
                NextValue(hs_mode, 2),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextState("Loopback.Exit")
            ).Elif(~loopback_master & self.rx_burst_ok,
                # Slave: exit on the master's qualified Loopback exit LFPS (7.5.11.2);
                # hs_rx pre-latched from the qualified burst.
                NextValue(hs_mode, 2),
                NextValue(hs_run,  1),
                NextValue(hs_send, 1),
                NextValue(hs_rx,   1),
                NextState("Loopback.Exit")
            )
        )

        # Loopback.Exit State (7.5.11.2) --------------------------------------------------------------
        fsm.act("Loopback.Exit", # 12.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            t_ux_exit.wait.eq(ux_armed),
            self.loopback.eq(1),
            If(hs_done_ok,
                NextState("Polling.ExitToRxDetect")
            ).Elif(hs_done_err,
                # tLoopbackExitTimeout (2ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            )
        )

        # Hot Reset Active State (7.5.12.2) -----------------------------------------------------------
        fsm.act("HotReset.Active", # 13.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            self.hot_reset.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts2.eq(1),
            NextValue(self.tx_ts2_reset, 1),
            NextValue(hr_seen, hr_seen | self.rx_ts2_reset),
            If((hr_cnt + 1) >= hr_delay,
                # Own reset done: send Reset=0 TS2 (sandbox: symmetric auto-clear).
                NextValue(self.tx_ts2_reset, 0)
            ),
            If(hr_seen & ~self.rx_ts2_reset & ~self.tx_ts2_reset,
                # Both sides saw Reset=0 -> Exit (sandbox: single fall detection).
                NextState("HotReset.Exit")
            ).Elif(_12_ms_timer.done,
                # tHotResetActiveTimeout (12ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            )
        )

        # Hot Reset Exit State (7.5.12.3) -------------------------------------------------------------
        fsm.act("HotReset.Exit", # 14.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.hot_reset.eq(1),
            t_hotreset_exit.wait.eq(with_timers),
            If(self.rx_idle8,
                # Idle handshake -> U0 (7.5.12.3).
                NextState("U0")
            ).Elif(t_hotreset_exit.done,
                # tHotResetExitTimeout (2ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            )
        )

        # Warm Reset burst State (6.9, DS originator / UFP responder) ---------------------------------
        fsm.act("WarmReset", # 15.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            NextValue(u3_entry_cnt, 0), # reset side effect: PM state cleared
            If(wr_done,
                # Both sides retrain from Rx.Detect (sandbox: transient) through Polling.
                NextState("Polling.ExitToRxDetect")
            )
        )

        # Recovery Active State (7.5.10.1) --------------------------------------------------------------
        fsm.act("Recovery.Active", # 16.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            NextValue(self.pm_sent_lgo, 0),
            NextValue(self.pm_acc_kind, 0),
            t_ux_exit.wait.eq(ux_armed),
            _12_ms_timer.wait.eq(with_timers),
            self.serdes_rx_align.eq(1), # Re-acquire word alignment on the incoming TS1s: the LFPS
                                        # exit burst shifts the RX word phase (Recovery is the only
                                        # TS-receiving path entered after a LFPS burst; the aligner
                                        # re-latches on the K->D boundary of each TS1 set).
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts1.eq(1),

            NextValue(rx_ts1_seen, rx_ts1_seen | self.ts_rx_ts1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),
            NextValue(rx_lb_seen,  rx_lb_seen  | self.rx_ts2_loopback),

            If(t_ux_exit.done,
                # Ux_EXIT_TIMER still running in Recovery -> eSS.Inactive (7.2.4.2.1).
                NextState("eSS.Inactive")
            ).Elif(_12_ms_timer.done,
                # tRecoveryActiveTimeout (12ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            ),
            If(self.ts_tx_done & (rx_ts1_seen | rx_ts2_seen),
                NextState("Recovery.Configuration")
            ),
        )

        # Recovery Configuration State (7.5.10.2) --------------------------------------------------------
        fsm.act("Recovery.Configuration", # 17.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            t_ux_exit.wait.eq(ux_armed),
            _6_ms_timer.wait.eq(with_timers),
            self.serdes_rx_align.eq(1), # Keep alignment re-acquisition during the TS2 exchange.
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),
            NextValue(rx_lb_seen,  rx_lb_seen  | self.rx_ts2_loopback),
            If(t_ux_exit.done,
                NextState("eSS.Inactive")
            ).Elif(_6_ms_timer.done,
                # tRecoveryConfigurationTimeout (6ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            ),
            If(self.ts_tx_done,
                If(rx_ts2_seen,
                    NextState("Recovery.Idle")
                )
            )
        )

        # Recovery Idle State (7.5.10.3) -----------------------------------------------------------------
        fsm.act("Recovery.Idle", # 18.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            t_ux_exit.wait.eq(ux_armed),
            t_recov_idle.wait.eq(with_timers),
            If(t_ux_exit.done,
                NextState("eSS.Inactive")
            ).Elif(self.directed_loopback & self.loopback_capable,
                # Directed loopback master (checked before idle handshake: the partner is being
                # directed into Loopback and must not bounce through U0).
                NextValue(loopback_master, 1),
                NextState("Loopback")
            ).Elif(rx_lb_seen | self.rx_ts2_loopback,
                # Loopback bit in TS2 -> Loopback (slave, forced).
                NextValue(loopback_master, 0),
                NextState("Loopback")
            ).Elif(self.rx_ts2_reset,
                # Reset bit in TS2 -> Hot Reset.
                NextValue(hr_seen, 0),
                NextState("HotReset.Active")
            ).Elif(self.hot_reset_req & self.ds_port,
                # DS-directed Hot Reset (7.5.12 Recovery.Idle entry): completes the
                # U1/U2 directed chain (LFPS Exit -> Recovery -> Hot Reset, p189-190)
                # and must outrank the idle handshake below.
                NextValue(hr_seen, 0),
                NextState("HotReset.Active")
            ).Elif(self.rx_idle8,
                # Idle handshake -> U0 (7.5.10.3).
                NextState("U0")
            ).Elif(t_recov_idle.done,
                # tRecoveryIdleTimeout (2ms) -> eSS.Inactive (Table 7-12).
                NextState("eSS.Inactive")
            )
        )

        # eSS.Inactive State (Table 7-12) ----------------------------------------------------------------
        fsm.act("eSS.Inactive", # 19.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.ess_inactive.eq(1),
            NextValue(ux_armed, 0),
            NextValue(u3_entry_cnt, 0), # the streak is consumed by going inactive
            t_ss_quiet.wait.eq(with_timers),
            If(t_ss_quiet.done,
                # 12ms quiet -> Rx.Detect (sandbox: transient re-train).
                NextState("Polling.ExitToRxDetect")
            )
        )

        # Exit to Compliance (legacy bring-up state, now unreachable: kept for compatibility) -------
        fsm.act("Polling.ExitToCompliance", # 20.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.exit_to_compliance.eq(1),
            If(self.lfps_rx_polling,
                NextState("Polling.Entry")
            ),
        )

        # Exit to RxDetect (sandbox: transient, instant re-train, design section 4) ----------------------
        fsm.act("Polling.ExitToRxDetect", # 21.
            *timers_off,
            self.lfps_tx_idle.eq(1),
            self.exit_to_rx_detect.eq(1),
            t_rx_detect.wait.eq(1),
            If(t_rx_detect.done,
                NextState("Polling.Entry")
            )
        )

        # # #

        # Global clocks/counters ----------------------------------------------------------------------

        # Outputs -------------------------------------------------------------------------------------

        # PM handshake pulses.
        self.comb += [
            self.tx_lgo_u1.eq(pm_lgo_fire[0] & (pm_kind == 1)),
            self.tx_lgo_u2.eq(pm_lgo_fire[0] & (pm_kind == 2)),
            self.tx_lgo_u3.eq(pm_lgo_fire[0] & (pm_kind == 3)),
            self.tx_lau.eq(pm_lau_fire),
            self.tx_lxu.eq(pm_lxu_fire),
            self.tx_lpma.eq(pm_lpma_fire),
        ]

        # LFPS bursts (driven through the LFPSUnit TX mux).
        self.comb += [
            self.lfps_tx_reset.eq(wr_run),
            self.lfps_tx_exit_u1.eq(hs_run & hs_send & (hs_mode == 1)),
            self.lfps_tx_exit_u2.eq(hs_run & hs_send & (hs_mode == 2)),
            self.lfps_tx_wakeup_u3.eq(hs_run & hs_send & (hs_mode == 3)),
        ]

    def finalize(self):
        # Post-finalization wiring: FSM state/encoding only exist once the FSM is finalized.
        LiteXModule.finalize(self)
        fsm = self.fsm

        # Single-burst qualifier (see __init__): mode-aware (each state listens to its own burst
        # class only), single-shot (a commit returns to idle; the handshake start or leaving the
        # handshake-capable states clears the request -- a latched request must never survive).
        hs_capable = ((fsm.state == fsm.encoding["U1"]) | (fsm.state == fsm.encoding["U2"]) |
                      (fsm.state == fsm.encoding["U3"]) | (fsm.state == fsm.encoding["Loopback"]))
        rx_detect = Signal()
        self.comb += [
            If(fsm.state == fsm.encoding["U1"],
                rx_detect.eq(self.lfps_rx_exit_u1)
            ).Elif(fsm.state == fsm.encoding["U3"],
                rx_detect.eq(self.lfps_rx_wakeup_u3)
            ).Else(
                # U2 responder and Loopback slave both listen to U2/Loopback exit bursts.
                rx_detect.eq(self.lfps_rx_exit_u2)
            )
        ]
        self.sync += [
            If(self.hs_run | ~hs_capable,
                # Consumed by the handshake start, or the FSM left the handshake-capable states:
                # drop the request and re-arm the qualifier from scratch.
                self.rx_burst_ok.eq(0),
                self.rxq_state.eq(0),
                self.rxq_cnt.eq(0)
            ).Elif(self.rxq_state == 0,
                If(rx_detect,
                    self.rxq_state.eq(1)
                )
            ).Elif(self.rxq_state == 1,
                If(~rx_detect,
                    self.rxq_state.eq(2),
                    self.rxq_cnt.eq(0)
                )
            ).Else(
                If(rx_detect,
                    # Polling train: the line did not stay quiet, re-arm on the new burst.
                    self.rxq_state.eq(1),
                    self.rx_burst_ok.eq(0)
                ).Elif((self.rxq_cnt + 1) >= self._rx_quiet_lim,
                    # Single-shot commit; consumed by the handshake start (hs_run) next cycle.
                    self.rx_burst_ok.eq(1)
                ).Else(
                    self.rxq_cnt.eq(self.rxq_cnt + 1)
                )
            )
        ]

        # U1 DS Ping timeout counter (reset on Ping reception).
        self.sync += [
            If((fsm.state == fsm.encoding["U1"]) & self.ds_port,
                If(self.lfps_rx_ping,
                    self.u1_ping_cnt.eq(0)
                ).Else(
                    If(~self.u1_ping_done,
                        self.u1_ping_cnt.eq(self.u1_ping_cnt + 1)
                    )
                )
            ).Else(
                self.u1_ping_cnt.eq(0)
            )
        ]

        # U1 UFP min residency counter.
        self.sync += [
            If((fsm.state == fsm.encoding["U1"]) & ~self.ds_port,
                If(~self.u1_residency_done,
                    self.u1_res_cnt.eq(self.u1_res_cnt + 1)
                )
            ).Else(
                self.u1_res_cnt.eq(0)
            )
        ]

        # Hot Reset Active reset-done delay counter.
        self.sync += [
            If(fsm.state == fsm.encoding["HotReset.Active"],
                self.hr_cnt.eq(self.hr_cnt + 1)
            ).Else(
                self.hr_cnt.eq(0)
            )
        ]

        self.comb += [
            self.lfps_tx_ping.eq((fsm.state == fsm.encoding["U1"]) &
                                 ~self.ds_port & self.u1_residency_done),
            self.tx_ts2_loopback.eq(self.lb_pending &
                ((fsm.state == fsm.encoding["Recovery.Configuration"]) |
                 (fsm.state == fsm.encoding["Recovery.Idle"]))),
        ]

