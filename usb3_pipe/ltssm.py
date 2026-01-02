#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from migen.genlib.misc import WaitTimer

from litex.gen import *

# Link Training and Status State Machine -----------------------------------------------------------

# Note: Currently just FSM skeletons with states/transitions.

@ResetInserter()
class LTSSMFSM(FSM):
    """Link Training and Status State Machine (section 7.5)"""
    def __init__(self):
        # FSM --------------------------------------------------------------------------------------
        FSM.__init__(self, reset_state="SS-INACTIVE")

        # SSInactive State -------------------------------------------------------------------------
        self.act("SS-INACTIVE",
            NextState("RX-DETECT"), # Warm reset, far-end termination absent.
        )

        # SSDisabled State -------------------------------------------------------------------------
        self.act("SS-DISABLED",
            NextState("RX-DETECT"), # Directed, PowerOn reset, USB2 bus reset
        )

        # RXDetect State ---------------------------------------------------------------------------
        self.act("RX-DETECT",
            NextState("SS-DISABLED"), # RXDetect events over limit (Dev) or directed (DS).
        )

        # Polling State ----------------------------------------------------------------------------
        self.act("POLLING",
            NextState("U0"),               # Idle symbol handshake.
            NextState("HOT-RESET"),        # Directed.
            NextState("LOOPBACK"),         # Directed.
            NextState("COMPLIANCE-MODE"),  # First LFPS timeout.
            NextState("SS-DISABLED"),       # Timeout, directed
        )

        # U0 State ---------------------------------------------------------------------------------
        self.act("U0",
            NextState("U1"),       # LGO_U1
            NextState("U2"),       # LGO_U2
            NextState("U3"),       # LGO_U3
            NextState("RECOVERY"), # Error, directed.
        )

        # U1 State ---------------------------------------------------------------------------------
        self.act("U1",
            NextState("U2"),          # Timeout.
            NextState("SS-INACTIVE"), # LFPS timeout.
        )

        # U2 State ---------------------------------------------------------------------------------
        self.act("U2",
            NextState("SS-INACTIVE"), # LFPS timeout.
            NextState("RECOVERY"),    # LTPS handshake.
        )

        # U3 State ---------------------------------------------------------------------------------
        self.act("U3",
            NextState("RECOVERY"), # LTPS handshake.
        )

        # Compliance Mode State --------------------------------------------------------------------
        self.act("COMPLIANCE-MODE",
            NextState("RX-DETECT"),  # Warm reset, PowerOn reset.
        )

        # Loopback State ---------------------------------------------------------------------------
        self.act("LOOPBACK",
            NextState("SS-INACTIVE"), # Timeout.
            NextState("RX-DETECT"),   # LFPS handshake.
        )

        # Hot Reset State --------------------------------------------------------------------------
        self.act("HOT-RESET",
            NextState("SS-INACTIVE"), # Timeout.
            NextState("U0"),          # Idle symbol handshake.
        )

        # Recovery ---------------------------------------------------------------------------------
        self.act("RECOVERY",
            NextState("U0"),          # Training.
            NextState("HOT-RESET"),   # Directed.
            NextState("SS-INACTIVE"), # Timeout.
            NextState("LOOPBACK"),    # Directed.
        )

        # End State --------------------------------------------------------------------------------
        self.act("END")

# SSInactive Finite State Machine  -----------------------------------------------------------------

# Note: Currently just FSM skeletons with states/transitions.

@ResetInserter()
class SSInactiveFSM(FSM):
    """SSInactive Finite State Machine (section 7.5.2)"""
    def __init__(self):
        self.exit_to_ss_disabled = Signal()
        self.exit_to_rx_detect   = Signal()

        # FSM --------------------------------------------------------------------------------------
        FSM.__init__(self, reset_state="QUIET")

        # QUIET State ------------------------------------------------------------------------------
        self.act("QUIET",
            NextState("DETECT"),             # Timeout
            self.exit_to_ss_disabled.eq(1),  # Directed (DS).
            self.exit_to_rx_detect.eq(1),    # Warm reset.
            NextState("END"),                # On any exit case.
        )

        # DETECT State -----------------------------------------------------------------------------
        self.act("DETECT",
            NextState("QUIET"),              # Far-end termination present.
            self.exit_to_rx_detect.eq(1),    # Far-end termination absent.
            NextState("END"),                # On any exit case.
        )

        # End State --------------------------------------------------------------------------------
        self.act("END")

# RXDetect Finite State Machine  -------------------------------------------------------------------

# Note: Currently just FSM skeletons with states/transitions.

@ResetInserter()
class RXDetectFSM(FSM):
    """RxDetect Finite State Machine (section 7.5.3)"""
    def __init__(self):
        self.exit_to_ss_disabled = Signal()
        self.exit_to_polling     = Signal()

        # FSM --------------------------------------------------------------------------------------
        FSM.__init__(self, reset_state="RESET")

        # Reset State ------------------------------------------------------------------------------
        self.act("RESET",
            NextState("ACTIVE"),             # Warm reset de-asserted.
            self.exit_to_ss_disabled.eq(1),  # Directed (DS).
            NextState("END"),                # On any exit case.
        )

        # Detect State -----------------------------------------------------------------------------
        self.act("ACTIVE",
            NextState("QUIET"),              # Far-end termination not detected.
            self.exit_to_polling.eq(1),      # Far-end termination detected.
            self.exit_to_ss_disabled.eq(1),  # RXDetect events over limit (Dev) or directed (DS).
            NextState("END"),                # On any exit case.
        )

        # Quiet State ------------------------------------------------------------------------------
        self.act("QUIET",
            NextState("ACTIVE"),             # Timeout.
            self.exit_to_ss_disabled.eq(1),  # Directed (DS).
            NextState("END"),                # On any exit case.
        )

        # End State --------------------------------------------------------------------------------
        self.act("END")

# Polling Finite State Machine ---------------------------------------------------------------------

@ResetInserter()
class PollingFSM(LiteXModule):
    """ Polling Finite State Machine (section 7.5.4)"""
    def __init__(self, serdes, lfps_unit, ts_unit, sys_clk_freq, with_timers=True):
        self.idle               = Signal()
        self.recovery           = Signal()
        self.rx_ready           = Signal()
        self.tx_ready           = Signal()
        self.exit_to_compliance = Signal()
        self.exit_to_rx_detect  = Signal()

        # # #

        tx_lfps_count   = Signal(16)
        rx_lfps_seen    = Signal()
        rx_ts1_seen     = Signal()
        rx_ts1_inv_seen = Signal()
        rx_ts2_seen     = Signal()

        # 360ms Timer ------------------------------------------------------------------------------
        self._360_ms_timer = _360_ms_timer = WaitTimer(int(360e-3*sys_clk_freq))

        # 12ms Timer -------------------------------------------------------------------------------
        self._12_ms_timer = _12_ms_timer = WaitTimer(int(12e-3*sys_clk_freq))

        # 6ms Timer -------------------------------------------------------------------------------
        self._6_ms_timer = _6_ms_timer = WaitTimer(int(6e-3*sys_clk_freq))

        # FSM --------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="Polling.Entry")

        # Entry State ------------------------------------------------------------------------------
        fsm.act("Polling.Entry", # 0.
            NextValue(tx_lfps_count,  16),
            NextValue(rx_lfps_seen,    0),
            NextValue(rx_ts1_seen,     0),
            NextValue(rx_ts1_inv_seen, 0),
            NextValue(rx_ts2_seen,     0),
            NextState("Polling.LFPS"),
        )

        # LFPS State (7.5.4.3) ---------------------------------------------------------------------
        fsm.act("Polling.LFPS", # 1.
            _360_ms_timer.wait.eq(with_timers),
            lfps_unit.tx_polling.eq(1),
            # Go to ExitToCompliance when:
            # - 360ms timer is expired.
            If(_360_ms_timer.done,
                NextState("Polling.ExitToCompliance")
            # Go to RxEQ when:
            # - at least 16 LFPS Polling Bursts have been generated.
            # - 2 consecutive LFPS Polling Bursts have been received (ensured by ts_unit).
            # - 4 LFPS Polling Bursts have been sent since first LFPS Polling Bursts reception.
            ).Elif(lfps_unit.tx_count >= tx_lfps_count,
                If(lfps_unit.rx_polling & ~rx_lfps_seen,
                    NextValue(rx_lfps_seen, 1),
                    NextValue(tx_lfps_count, lfps_unit.tx_count + 4)
                ),
                If(rx_lfps_seen,
                    NextState("Polling.RxEQ"),
                )
            )
        )

        # RxEQ State (7.5.4.4) ---------------------------------------------------------------------
        fsm.act("Polling.RxEQ", # 2.
            serdes.rx_align.eq(1),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_tseq.eq(1),
            # Go to Active when the 65536 TSEQ ordered sets are sent.
            If(ts_unit.tx_done,
                NextState("Polling.Active")
            ),
        )

        # Active State (7.5.4.5) -------------------------------------------------------------------
        fsm.act("Polling.Active", # 3.
            serdes.rx_align.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts1.eq(1),

            # Latch what we saw from the host (rx_ts1 / rx_ts1_inv are pulses).
            NextValue(rx_ts1_seen,     rx_ts1_seen     | ts_unit.rx_ts1),
            NextValue(rx_ts1_inv_seen, rx_ts1_inv_seen | ts_unit.rx_ts1_inv),

            # Go to RxDetect if no TS1/TS2 seen in the 12ms.
            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),

            # Go to Configuration only after:
            # - we have completed a TS1 transmit burst (tx_done while tx_ts1 selected)
            # - and we have seen TS1 (normal or inverted) from the host.
            If(ts_unit.tx_done & (rx_ts1_seen | rx_ts1_inv_seen),
                If(rx_ts1_seen,
                    If(~self.recovery, NextValue(serdes.rx_polarity, 0)),
                ),
                If(rx_ts1_inv_seen,
                    If(~self.recovery, NextValue(serdes.rx_polarity, 1)),
                ),
                _12_ms_timer.wait.eq(0),
                NextState("Polling.Configuration")
            ),
        )

        # Configuration State (7.5.4.6) ------------------------------------------------------------
        fsm.act("Polling.Configuration", # 4.
            serdes.rx_align.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | ts_unit.rx_ts2),
            # Go to RxDetect if no TS2 seen in the 12ms.
            If(_12_ms_timer.done,
                _12_ms_timer.wait.eq(0),
                NextState("Polling.ExitToRxDetect")
            ),
            # Go to Idle when:
            # - 8 consecutive TS2 ordered sets are received. (8 ensured by ts_unit)
            # - 16 TS2 ordered sets are sent after receiving the first 8 TS2 ordered sets. FIXME
            If(ts_unit.tx_done,
                If(rx_ts2_seen,
                    self.rx_ready.eq(1),
                    NextState("Polling.Idle")
                )
            )
        )

        # Idle State (7.5.4.7) ---------------------------------------------------------------------
        fsm.act("Polling.Idle", # 5.
            self.idle.eq(1),
            self.rx_ready.eq(1),
            self.tx_ready.eq(1),
            NextValue(self.recovery, 0),
            If(ts_unit.rx_ts1, # FIXME: for bringup, should be Recovery.Active
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            )
        )

        # Exit to Compliance -----------------------------------------------------------------------
        fsm.act("Polling.ExitToCompliance", # 6.
            lfps_unit.tx_idle.eq(1), # FIXME: for bringup
            If(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_compliance.eq(1)
        )

        # Exit to RxDetect -------------------------------------------------------------------------
        fsm.act("Polling.ExitToRxDetect", # 7.
            lfps_unit.tx_idle.eq(1), # FIXME: for bringup
            If(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_rx_detect.eq(1)
        )


        # Recovery ---------------------------------------------------------------------------------
        fsm.act("Recovery.Active", # 8.
            serdes.rx_align.eq(1),
            _12_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts1.eq(1),

            # Latch what we saw from the host (rx_ts1 / rx_ts2 are pulses).
            NextValue(rx_ts1_seen, rx_ts1_seen | ts_unit.rx_ts1),
            NextValue(rx_ts2_seen, rx_ts2_seen | ts_unit.rx_ts2),

            # Go to RxDetect if no TS1/TS2 seen in the 12ms.
            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),

            # Go to Configuration only after:
            # - we have completed a TS1 transmit burst (tx_done while tx_ts1 selected)
            # - and we have seen TS1 or TS2 from the host.
            If(ts_unit.tx_done & (rx_ts1_seen | rx_ts2_seen),
                _12_ms_timer.wait.eq(0),
                NextState("Recovery.Configuration")
            ),
        )


        fsm.act("Recovery.Configuration", # 9.
            serdes.rx_align.eq(1),
            _6_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | ts_unit.rx_ts2),
            # Go to RxDetect if no TS2 seen in the 6ms.
            If(_6_ms_timer.done,
                _6_ms_timer.wait.eq(0),
                NextState("Polling.ExitToRxDetect")
            ),
            # Go to Idle when:
            # - 8 consecutive TS2 ordered sets are received. (8 ensured by ts_unit)
            # - 16 TS2 ordered sets are sent after receiving the first 8 TS2 ordered sets. FIXME
            If(ts_unit.tx_done,
                If(rx_ts2_seen,
                    self.rx_ready.eq(1),
                    NextState("Polling.Idle")
                )
            )
        )


# Link Training and Status State Machine -----------------------------------------------------------

class LTSSM(LiteXModule):
    def __init__(self, serdes, lfps_unit, ts_unit, sys_clk_freq):
        # SS Inactive FSM --------------------------------------------------------------------------
        self.ss_inactive = SSInactiveFSM()

        # RX Detect FSM ----------------------------------------------------------------------------
        self.rx_detect   = RXDetectFSM()

        # Polling FSM ------------------------------------------------------------------------------
        self.polling     = PollingFSM(
            serdes       = serdes,
            lfps_unit    = lfps_unit,
            ts_unit      = ts_unit,
            sys_clk_freq = sys_clk_freq)

        # LTSSM FSM --------------------------------------------------------------------------------
        self.ltssm       = LTSSMFSM()
