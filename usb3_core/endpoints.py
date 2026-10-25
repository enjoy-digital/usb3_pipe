#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Endpoints base (USB 3.2 8.10/8.11/8.12): Bulk (bulk.py) and Isochronous (iso.py) Endpoints.

Endpoints interface with the Protocol Layer (see USB3ProtocolLayer):
- hdr: received Header Packet (broadcast) and hdr_strobe (header processed), with the DPP status
  (dpp_ok/dpp_landed) for DPHs.
- resp_valid/resp_dw1: TP response to the current header (when addressed).
- tx_hp/tx_dpp: Header Packets (DPH/ERDY/NRDY) and Data Packet Payloads to transmit.
- rx_dpp/rx_dpp_done/rx_dpp_ok: received Data Packet Payloads (routed by Endpoint Number).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import *

# Constants ----------------------------------------------------------------------------------------

MAX_PACKET_SIZE  = 1024
MAX_PACKET_WORDS = MAX_PACKET_SIZE//4

# USB3 Endpoint Base -------------------------------------------------------------------------------

class USB3Endpoint(LiteXModule):
    def __init__(self, ept, direction):
        self.ept       = ept
        self.direction = direction # 0: OUT, 1: IN.

        # Protocol Layer interface.
        self.hdr_dw0      = Signal(32) # i
        self.hdr_dw1      = Signal(32) # i
        self.hdr_dw2      = Signal(32) # i
        self.hdr_strobe   = Signal()   # i
        self.dpp_ok       = Signal()   # i
        self.dpp_landed   = Signal()   # i
        self.resp_valid   = Signal()   # o
        self.resp_dw1     = Signal(32) # o
        self.tx_hp        = stream.Endpoint(tx_header_layout()) # o
        self.tx_dpp       = stream.Endpoint(dpp_layout())       # o
        self.rx_dpp       = stream.Endpoint(dpp_layout())       # i
        self.rx_dpp_done  = Signal() # i
        self.rx_dpp_ok    = Signal() # i
        self.rx_dpp_setup = Signal() # i: Received DPP is a SETUP DP (Control Endpoint).
        self.landed       = Signal() # o: rx DPP landed in a slot (with rx_dpp_done).
        self.pm_busy      = Signal() # o: U1/U2 entry rejected (ex: ACK TP expected, 7.2.4.2.2).

        # Control.
        self.reset_seq = Signal() # i: Seq Numbers reset (SET_CONFIGURATION, CLEAR_FEATURE(HALT)).

        self.hdr = HeaderFields(self.hdr_dw0, self.hdr_dw1, self.hdr_dw2)

    def tp_addressed(self, subtype):
        h = self.hdr
        return (h.type == TYPE_TP) & (h.subtype == subtype) & (h.ept == self.ept) & \
            (h.direction == self.direction)

    def dp_addressed(self):
        h = self.hdr
        return (h.type == TYPE_DP) & (h.ept == self.ept) & (h.setup == 0)
