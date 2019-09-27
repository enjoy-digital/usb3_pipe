from migen import *

# Note: Currently just FSM skeletons with states/transitions.

# Polling Finite State Machine  --------------------------------------------------------------------

@ResetInserter()
class PollingFSM(FSM):
    """ Polling Finite State Machine (section 7.5.4)"""
    def __init__(self):
        self.exit_to_compliance_mode = Signal()
        self.exit_to_rx_detect       = Signal()
        self.exit_to_ss_disabled     = Signal()
        self.exit_to_loopback        = Signal()
        self.exit_to_hot_reset       = Signal()
        self.exit_to_u0              = Signal()

        # # #

        # FSM --------------------------------------------------------------------------------------
        FSM.__init__(self, reset_state="LFPS")

        # LFPS State -------------------------------------------------------------------------------
        self.act("LFPS",
            NextState("RX-EQ"),                 # LFPS handshake.
            self.exit_to_compliance_mode.eq(1), # First LFPS timeout.
            self.exit_to_ss_disabled.eq(1),     # Subsequent LFPS timeouts (Dev) or directed (DS).
            self.exit_to_rx_detect.eq(1),       # Subsequent LFPS timeouts (DS).
            NextState("END"),                   # On any exit case.
        )

        # RxEQ State -------------------------------------------------------------------------------
        self.act("RX-EQ",
            NextState("ACTIVE"),                # TSEQ transmitted.
            self.exit_to_ss_disabled.eq(1),     # Directed (DS).
            NextState("END"),                   # On any exit case.
        )

        # Active State -----------------------------------------------------------------------------
        self.act("ACTIVE",
            NextState("CONFIGURATION"),         # 8 consecutiive TS1 or TS2 received.
            self.exit_to_ss_disabled.eq(1),     # Timeout (Dev) or directed (DS).
            self.exit_to_rx_detect.eq(1),       # Timeout (DS).
            NextState("END")                    # On any exit case.
        )

        # Configuration State ----------------------------------------------------------------------
        self.act("CONFIGURATION",
            NextState("IDLE"),                  # TS2 handshake.
            self.exit_to_ss_disabled.eq(1),     # Timeout (Dev) or directed (DS).
            self.exit_to_rx_detect.eq(1),       # Timeout (DS).
            NextState("END"),                   # On any exit case.
        )

        # Idle State -------------------------------------------------------------------------------
        self.act("IDLE",
            self.exit_to_ss_disabled.eq(1),     # Timeout (Dev) or directed (DS).
            self.exit_to_rx_detect.eq(1),       # Timeout (DS).
            self.exit_to_loopback.eq(1),        # Directed.
            self.exit_to_hot_reset.eq(1),       # Directed.
            self.exit_to_u0.eq(1),              # Idle symbol handshake.
            NextState("END"),                   # On any exit case.
        )

        # End State -------------------------------------------------------------------------------
        self.act("END")
