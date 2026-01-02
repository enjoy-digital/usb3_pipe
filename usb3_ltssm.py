#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

# Link Training and Status State Machine -----------------------------------------------------------

@ResetInserter()
class USB3LTSSM(LiteXModule):
    """ Link Training and Status State Machine (section 7.5)"""
    def __init__(self, sys_clk_freq, with_timers=True):
        # Status -----------------------------------------------------------------------------------
        self.u0                 = Signal()
        self.recovery           = Signal()
        self.rx_ready           = Signal()
        self.tx_ready           = Signal()
        self.exit_to_compliance = Signal()
        self.exit_to_rx_detect  = Signal()

        # SerDes control ---------------------------------------------------------------------------
        self.serdes_rx_align    = Signal()         # o
        self.serdes_rx_polarity = Signal(reset=0)  # o

        # LFPS control/status ----------------------------------------------------------------------
        self.lfps_tx_polling = Signal()     # o
        self.lfps_tx_idle    = Signal()     # o
        self.lfps_rx_polling = Signal()     # i
        self.lfps_tx_count   = Signal(16)   # i

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

        # 6ms Timer --------------------------------------------------------------------------------
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
            self.lfps_tx_polling.eq(1),
            If(_360_ms_timer.done,
                NextState("Polling.ExitToCompliance")
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
                If(rx_ts1_seen,
                    If(~self.recovery, NextValue(self.serdes_rx_polarity, 0)),
                ),
                If(rx_ts1_inv_seen,
                    If(~self.recovery, NextValue(self.serdes_rx_polarity, 1)),
                ),
                _12_ms_timer.wait.eq(0),
                NextState("Polling.Configuration")
            ),
        )

        # Configuration State (7.5.4.6) ------------------------------------------------------------
        fsm.act("Polling.Configuration", # 4.
            _12_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),
            If(_12_ms_timer.done,
                _12_ms_timer.wait.eq(0),
                NextState("Polling.ExitToRxDetect")
            ),
            If(self.ts_tx_done,
                If(rx_ts2_seen,
                    self.rx_ready.eq(1),
                    NextState("U0")
                )
            )
        )

        # U0 State ---------------------------------------------------------------------------------
        fsm.act("U0", # 5.
            self.u0.eq(1),
            self.rx_ready.eq(1),
            self.tx_ready.eq(1),
            NextValue(self.recovery, 0),
            If(self.ts_rx_ts1, # FIXME: for bringup, should be Recovery.Active
                NextValue(self.recovery, 1),
                NextValue(rx_ts1_seen,     0),
                NextValue(rx_ts1_inv_seen, 0),
                NextValue(rx_ts2_seen,     0),
                NextState("Recovery.Active")
            ).Elif(self.lfps_rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            )
        )

        # Exit to Compliance -----------------------------------------------------------------------
        fsm.act("Polling.ExitToCompliance", # 6.
            self.lfps_tx_idle.eq(1), # FIXME: for bringup
            If(self.lfps_rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_compliance.eq(1)
        )

        # Exit to RxDetect -------------------------------------------------------------------------
        fsm.act("Polling.ExitToRxDetect", # 7.
            self.lfps_tx_idle.eq(1), # FIXME: for bringup
            If(self.lfps_rx_polling, # FIXME: for bringup
                NextState("Polling.Entry")
            ),
            self.exit_to_rx_detect.eq(1)
        )

        # Recovery ---------------------------------------------------------------------------------
        fsm.act("Recovery.Active", # 8.
            _12_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts1.eq(1),

            NextValue(rx_ts1_seen, rx_ts1_seen | self.ts_rx_ts1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),

            If(_12_ms_timer.done,
                NextState("Polling.ExitToRxDetect")
            ),

            If(self.ts_tx_done & (rx_ts1_seen | rx_ts2_seen),
                _12_ms_timer.wait.eq(0),
                NextState("Recovery.Configuration")
            ),
        )

        fsm.act("Recovery.Configuration", # 9.
            _6_ms_timer.wait.eq(with_timers),
            self.ts_rx_enable.eq(1),
            self.ts_tx_enable.eq(1),
            self.ts_tx_ts2.eq(1),
            NextValue(rx_ts2_seen, rx_ts2_seen | self.ts_rx_ts2),
            If(_6_ms_timer.done,
                _6_ms_timer.wait.eq(0),
                NextState("Polling.ExitToRxDetect")
            ),
            If(self.ts_tx_done,
                If(rx_ts2_seen,
                    self.rx_ready.eq(1),
                    NextState("U0")
                )
            )
        )
