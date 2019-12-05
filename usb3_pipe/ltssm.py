# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from migen.genlib.misc import WaitTimer

# Note: Currently just FSM skeletons with states/transitions.

# Link Training and Status State Machine -----------------------------------------------------------

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
class PollingFSM(Module):
    """ Polling Finite State Machine (section 7.5.4)"""
    def __init__(self, serdes, lfps_unit, ts_unit, sys_clk_freq, with_timers=True):
        self.idle               = Signal()
        self.rx_ready           = Signal()
        self.tx_ready           = Signal()
        self.exit_to_compliance = Signal()
        self.exit_to_rx_detect  = Signal()

        # # #

        tx_lfps_count = Signal(16)
        rx_lfps_seen  = Signal()
        rx_ts2_seen   = Signal()

        # 360ms Timer ------------------------------------------------------------------------------
        _360_ms_timer = WaitTimer(int(360e-3*sys_clk_freq))
        self.submodules += _360_ms_timer

        # 12ms Timer -------------------------------------------------------------------------------
        _12_ms_timer = WaitTimer(int(12e-3*sys_clk_freq))
        self.submodules += _12_ms_timer

        # FSM --------------------------------------------------------------------------------------
        self.submodules.fsm = fsm = FSM(reset_state="Polling.Entry")

        # Entry State ------------------------------------------------------------------------------
        fsm.act("Polling.Entry",
            NextValue(tx_lfps_count, 16),
            NextValue(rx_lfps_seen, 0),
            NextValue(rx_ts2_seen, 0),
            NextState("Polling.LFPS"),
        )

        # LFPS State (7.5.4.3) ---------------------------------------------------------------------
        fsm.act("Polling.LFPS",
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
        fsm.act("Polling.RxEQ",
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
        fsm.act("Polling.Active",
            _12_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts1.eq(1),
            # Go to RxDetect if no TS1/TS2 seen in the 12ms.
            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),
            # Go to Configuration if at least 8 consecutive TS1 or TS1_INV seen (8 ensured by ts_unit)
            If(ts_unit.rx_ts1,
                NextValue(serdes.rx_polarity, 0),
                _12_ms_timer.wait.eq(0),
                NextValue(rx_ts2_seen, 0),
                NextState("Polling.Configuration")
            ),
            If(ts_unit.rx_ts1_inv,
                NextValue(serdes.rx_polarity, 1),
                _12_ms_timer.wait.eq(0),
                NextValue(rx_ts2_seen, 0),
                NextState("Polling.Configuration")
            ),
        )

        # Configuration State (7.5.4.6) ------------------------------------------------------------
        fsm.act("Polling.Configuration",
            _12_ms_timer.wait.eq(with_timers),
            ts_unit.rx_enable.eq(1),
            ts_unit.tx_enable.eq(1),
            ts_unit.tx_ts2.eq(1),
            self.rx_ready.eq(rx_ts2_seen),
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
                    NextState("Polling.Idle")
                )
            )
        )

        # Idle State (7.5.4.7) ---------------------------------------------------------------------
        fsm.act("Polling.Idle",
            self.idle.eq(1),
            self.rx_ready.eq(1),
            self.tx_ready.eq(1),
            If(ts_unit.rx_ts1, # FIXME: for bringup, should be Recovery.Active
                NextState("Polling.Active")
            ).Elif(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            )
        )

        # Exit to Compliance -----------------------------------------------------------------------
        fsm.act("Polling.ExitToCompliance",
            lfps_unit.tx_idle.eq(1), # FIXME: for bringup
            If(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_compliance.eq(1)
        )

        # Exit to RxDetect -------------------------------------------------------------------------
        fsm.act("Polling.ExitToRxDetect",
            lfps_unit.tx_idle.eq(1), # FIXME: for bringup
            If(lfps_unit.rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_rx_detect.eq(1)
        )

# Link Training and Status State Machine -----------------------------------------------------------

class LTSSM(Module):
    def __init__(self, serdes, lfps_unit, ts_unit, sys_clk_freq):
        # SS Inactive FSM --------------------------------------------------------------------------
        self.submodules.ss_inactive = SSInactiveFSM()

        # RX Detect FSM ----------------------------------------------------------------------------
        self.submodules.rx_detect   = RXDetectFSM()

        # Polling FSM ------------------------------------------------------------------------------
        self.submodules.polling     = PollingFSM(
            serdes       = serdes,
            lfps_unit    = lfps_unit,
            ts_unit      = ts_unit,
            sys_clk_freq = sys_clk_freq)

        # LTSSM FSM --------------------------------------------------------------------------------
        self.submodules.ltssm       = LTSSMFSM()
