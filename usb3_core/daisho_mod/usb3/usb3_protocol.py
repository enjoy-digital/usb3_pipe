#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2013 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen import *

# USB3 Protocol ------------------------------------------------------------------------------------

class USB3Protocol(LiteXModule):
    def __init__(self, platform=None):
        # -----------------------------------------------------------------------------------------
        # Constants
        # -----------------------------------------------------------------------------------------
        # LTSSM states
        LT_U0            = 16

        # Transaction Packet subtypes
        LP_TP_SUB_ACK    = 0b0001
        LP_TP_SUB_NRDY   = 0b0010
        LP_TP_SUB_ERDY   = 0b0011
        LP_TP_SUB_STATUS = 0b0100
        LP_TP_SUB_PING   = 0b0111

        # Retry flags
        LP_TP_NORETRY    = 0
        LP_TP_RETRY      = 1

        # Directions
        LP_TP_HOSTTODEVICE = 0
        LP_TP_DEVICETOHOST = 1

        # Endpoint selector values
        SEL_ENDP0 = 0
        SEL_ENDP1 = 1
        SEL_ENDP2 = 2

        # Endpoint modes
        EP_MODE_CONTROL   = 0
        EP_MODE_ISOCH     = 1
        EP_MODE_BULK      = 2
        EP_MODE_INTERRUPT = 3

        EP1_MODE = EP_MODE_BULK
        EP2_MODE = EP_MODE_BULK

        # -----------------------------------------------------------------------------------------
        # Ports (match Verilog module interface)
        # -----------------------------------------------------------------------------------------
        self.slow_clk              = Signal()
        self.local_clk             = Signal()
        self.ext_clk               = Signal()
        self.reset_n               = Signal()
        self.ltssm_state           = Signal(5)

        # Link RX
        self.rx_tp                 = Signal()
        self.rx_tp_hosterr         = Signal()
        self.rx_tp_retry           = Signal()
        self.rx_tp_pktpend         = Signal()
        self.rx_tp_subtype         = Signal(4)
        self.rx_tp_endp            = Signal(4)
        self.rx_tp_nump            = Signal(5)
        self.rx_tp_seq             = Signal(5)
        self.rx_tp_stream          = Signal(16)

        self.rx_dph                = Signal()
        self.rx_dph_eob            = Signal()
        self.rx_dph_setup          = Signal()
        self.rx_dph_pktpend        = Signal()
        self.rx_dph_endp           = Signal(4)
        self.rx_dph_seq            = Signal(5)
        self.rx_dph_len            = Signal(16)
        self.rx_dpp_start          = Signal()
        self.rx_dpp_done           = Signal()
        self.rx_dpp_crcgood        = Signal()

        # Link TX
        self.tx_tp_a               = Signal()
        self.tx_tp_a_retry         = Signal()
        self.tx_tp_a_dir           = Signal()
        self.tx_tp_a_subtype       = Signal(4)
        self.tx_tp_a_endp          = Signal(4)
        self.tx_tp_a_nump          = Signal(5)
        self.tx_tp_a_seq           = Signal(5)
        self.tx_tp_a_stream        = Signal(16)
        self.tx_tp_a_ack           = Signal()

        self.tx_tp_b               = Signal()
        self.tx_tp_b_retry         = Signal()
        self.tx_tp_b_dir           = Signal()
        self.tx_tp_b_subtype       = Signal(4)
        self.tx_tp_b_endp          = Signal(4)
        self.tx_tp_b_nump          = Signal(5)
        self.tx_tp_b_seq           = Signal(5)
        self.tx_tp_b_stream        = Signal(16)
        self.tx_tp_b_ack           = Signal()

        self.tx_tp_c               = Signal()
        self.tx_tp_c_retry         = Signal()
        self.tx_tp_c_dir           = Signal()
        self.tx_tp_c_subtype       = Signal(4)
        self.tx_tp_c_endp          = Signal(4)
        self.tx_tp_c_nump          = Signal(5)
        self.tx_tp_c_seq           = Signal(5)
        self.tx_tp_c_stream        = Signal(16)
        self.tx_tp_c_ack           = Signal()

        self.tx_dph                = Signal()
        self.tx_dph_eob            = Signal()
        self.tx_dph_dir            = Signal()
        self.tx_dph_endp           = Signal(4)
        self.tx_dph_seq            = Signal(5)
        self.tx_dph_len            = Signal(16)
        self.tx_dpp_ack            = Signal()
        self.tx_dpp_done           = Signal()

        # Internal muxed BRAM interface (link layer side)
        self.buf_in_addr           = Signal(9)
        self.buf_in_data           = Signal(32)
        self.buf_in_wren           = Signal()
        self.buf_in_ready          = Signal()
        self.buf_in_commit         = Signal()
        self.buf_in_commit_len     = Signal(11)
        self.buf_in_commit_ack     = Signal()

        self.buf_out_addr          = Signal(9)
        self.buf_out_q             = Signal(32)
        self.buf_out_len           = Signal(11)
        self.buf_out_hasdata       = Signal()
        self.buf_out_arm           = Signal()
        self.buf_out_arm_ack       = Signal()

        # External interface
        self.ext_buf_in_addr       = Signal(9)
        self.ext_buf_in_data       = Signal(32)
        self.ext_buf_in_wren       = Signal()
        self.ext_buf_in_request    = Signal()
        self.ext_buf_in_ready      = Signal()
        self.ext_buf_in_commit     = Signal()
        self.ext_buf_in_commit_len = Signal(11)
        self.ext_buf_in_commit_ack = Signal()

        self.ext_buf_out_addr      = Signal(9)
        self.ext_buf_out_q         = Signal(32)
        self.ext_buf_out_len       = Signal(11)
        self.ext_buf_out_hasdata   = Signal()
        self.ext_buf_out_arm       = Signal()
        self.ext_buf_out_arm_ack   = Signal()

        # Endpoint modes
        self.endp_mode_rx          = Signal(2)
        self.endp_mode_tx          = Signal(2)

        # Vendor request
        self.vend_req_act          = Signal()
        self.vend_req_request      = Signal(8)
        self.vend_req_val          = Signal(16)

        # Device state
        self.dev_addr              = Signal(7)
        self.configured            = Signal()

        # Errors
        self.err_miss_rx           = Signal()
        self.err_miss_tx           = Signal()
        self.err_tp_subtype        = Signal()
        self.err_missed_dpp_start  = Signal()
        self.err_missed_dpp_done   = Signal()

        # # #

        # -----------------------------------------------------------------------------------------
        # Internal regs/wires
        # -----------------------------------------------------------------------------------------
        rx_endp      = Signal(4, reset=SEL_ENDP0)
        tx_endp      = Signal(4, reset=SEL_ENDP0)

        in_dpp_seq   = Signal(5)
        out_dpp_seq  = Signal(5)

        out_length   = Signal(16)
        out_nump     = Signal(5)

        do_send_dpp  = Signal()

        recv_count   = Signal(11)
        dc           = Signal(11)

        reset_dp_seq = Signal()  # driven by EP0

        # -----------------------------------------------------------------------------------------
        # Endpoint muxing (same behavior as the Verilog assigns)
        # -----------------------------------------------------------------------------------------
        # EP0 (control)
        ep0_buf_in_addr       = Signal(9)
        ep0_buf_in_data       = Signal(32)
        ep0_buf_in_wren       = Signal()
        ep0_buf_in_ready      = Signal()
        ep0_buf_in_commit     = Signal()
        ep0_buf_in_commit_len = Signal(11)
        ep0_buf_in_commit_ack = Signal()

        ep0_buf_out_addr      = Signal(9)
        ep0_buf_out_q         = Signal(32)
        ep0_buf_out_len       = Signal(11)
        ep0_buf_out_hasdata   = Signal()
        ep0_buf_out_arm       = Signal()
        ep0_buf_out_arm_ack   = Signal()

        # EP1 (IN to host)
        ep1_buf_out_addr      = Signal(9)
        ep1_buf_out_q         = Signal(32)
        ep1_buf_out_len       = Signal(11)
        ep1_buf_out_hasdata   = Signal()
        ep1_buf_out_arm       = Signal()
        ep1_buf_out_arm_ack   = Signal()

        # EP2 (OUT from host)
        ep2_buf_in_addr       = Signal(9)
        ep2_buf_in_data       = Signal(32)
        ep2_buf_in_wren       = Signal()
        ep2_buf_in_ready      = Signal()
        ep2_buf_in_commit     = Signal()
        ep2_buf_in_commit_len = Signal(11)
        ep2_buf_in_commit_ack = Signal()

        # Drive endpoint-side muxed inputs.
        self.comb += [
            # RX-side buffer mux (who receives incoming payload writes)
            ep0_buf_in_addr.eq(Mux(rx_endp == SEL_ENDP0, self.buf_in_addr, 0)),
            ep0_buf_in_data.eq(Mux(rx_endp == SEL_ENDP0, self.buf_in_data, 0)),
            ep0_buf_in_wren.eq(Mux(rx_endp == SEL_ENDP0, self.buf_in_wren, 0)),
            ep0_buf_in_commit.eq(Mux(rx_endp == SEL_ENDP0, self.buf_in_commit, 0)),
            ep0_buf_in_commit_len.eq(Mux(rx_endp == SEL_ENDP0, self.buf_in_commit_len, 0)),

            ep2_buf_in_addr.eq(Mux(rx_endp == SEL_ENDP2, self.buf_in_addr, 0)),
            ep2_buf_in_data.eq(Mux(rx_endp == SEL_ENDP2, self.buf_in_data, 0)),
            ep2_buf_in_wren.eq(Mux(rx_endp == SEL_ENDP2, self.buf_in_wren, 0)),
            ep2_buf_in_commit.eq(Mux(rx_endp == SEL_ENDP2, self.buf_in_commit, 0)),
            ep2_buf_in_commit_len.eq(Mux(rx_endp == SEL_ENDP2, self.buf_in_commit_len, 0)),

            # TX-side buffer mux (who provides outgoing data reads)
            ep0_buf_out_addr.eq(Mux(tx_endp == SEL_ENDP0, self.buf_out_addr, 0)),
            ep0_buf_out_arm.eq(Mux(tx_endp == SEL_ENDP0, self.buf_out_arm, 0)),

            ep1_buf_out_addr.eq(Mux(tx_endp == SEL_ENDP1, self.buf_out_addr, 0)),
            ep1_buf_out_arm.eq(Mux(tx_endp == SEL_ENDP1, self.buf_out_arm, 0)),

            # Mux outputs back to link layer side.
            self.buf_in_ready.eq(
                Mux(rx_endp == SEL_ENDP0, ep0_buf_in_ready,
                Mux(rx_endp == SEL_ENDP2, ep2_buf_in_ready, 0))
            ),
            self.buf_in_commit_ack.eq(
                Mux(rx_endp == SEL_ENDP0, ep0_buf_in_commit_ack,
                Mux(rx_endp == SEL_ENDP2, ep2_buf_in_commit_ack, 0))
            ),

            self.buf_out_q.eq(
                Mux(tx_endp == SEL_ENDP0, ep0_buf_out_q,
                Mux(tx_endp == SEL_ENDP1, ep1_buf_out_q, 0))
            ),
            self.buf_out_len.eq(
                Mux(tx_endp == SEL_ENDP0, ep0_buf_out_len,
                Mux(tx_endp == SEL_ENDP1, ep1_buf_out_len, 0))
            ),
            self.buf_out_hasdata.eq(
                Mux(tx_endp == SEL_ENDP0, ep0_buf_out_hasdata,
                Mux(tx_endp == SEL_ENDP1, ep1_buf_out_hasdata, 0))
            ),
            self.buf_out_arm_ack.eq(
                Mux(tx_endp == SEL_ENDP0, ep0_buf_out_arm_ack,
                Mux(tx_endp == SEL_ENDP1, ep1_buf_out_arm_ack, 0))
            ),

            # Endpoint mode outputs.
            self.endp_mode_tx.eq(Mux(tx_endp == SEL_ENDP1, EP1_MODE,
                                Mux(tx_endp == SEL_ENDP2, EP2_MODE, EP_MODE_CONTROL))),
            self.endp_mode_rx.eq(Mux(rx_endp == SEL_ENDP1, EP1_MODE,
                                Mux(rx_endp == SEL_ENDP2, EP2_MODE, EP_MODE_CONTROL))),
        ]

        # -----------------------------------------------------------------------------------------
        # RX/TX Control FSMs (translated from the Verilog case(rx_state)/case(tx_state))
        # -----------------------------------------------------------------------------------------
        RX_RESET = 0
        RX_IDLE  = 1
        RX_TP_0  = 10
        RX_DPH_0 = 20
        RX_DPH_1 = 21
        RX_DPH_2 = 22

        TX_RESET       = 0
        TX_IDLE        = 1
        TX_DP_WAITDATA = 2
        TX_DP_0        = 3
        TX_DP_1        = 4
        TX_DP_NRDY     = 7
        TX_DP_ERDY     = 8

        rx_state = Signal(5, reset=RX_RESET)
        tx_state = Signal(5, reset=TX_RESET)

        # Default “one-cycle pulse” regs: clear each cycle, then set in states.
        self.sync += [
            self.tx_tp_a.eq(0),
            self.tx_tp_b.eq(0),
            self.tx_tp_c.eq(0),
            self.tx_dph.eq(0),
            do_send_dpp.eq(0),
            self.ext_buf_in_request.eq(0),

            dc.eq(dc + 1),
            recv_count.eq(recv_count + 1),
        ]

        # Convenience: reset datapacket seq counters if EP0 asks.
        self.sync += If(~self.reset_n,
            in_dpp_seq.eq(0),
            out_dpp_seq.eq(0),
        ).Elif(reset_dp_seq,
            in_dpp_seq.eq(0),
            out_dpp_seq.eq(0),
        )

        # RX machine -------------------------------------------------------------------------------
        self.sync += If(~self.reset_n,
            rx_state.eq(RX_RESET),
        ).Else(
            Case(rx_state, {
                RX_RESET: [
                    rx_state.eq(RX_IDLE)
                ],
                RX_IDLE: [
                    If(self.rx_dph,
                        in_dpp_seq.eq(self.rx_dph_seq),
                        rx_endp.eq(self.rx_dph_endp),
                        rx_state.eq(RX_DPH_0),
                        recv_count.eq(0),
                    ).Elif(self.rx_tp,
                        rx_state.eq(RX_TP_0)
                    )
                ],
                RX_DPH_0: [
                    If(self.rx_dpp_start,
                        rx_state.eq(RX_DPH_1)
                    ),
                    If((self.ltssm_state != LT_U0) | (recv_count == 20),
                        self.err_missed_dpp_start.eq(1),
                        rx_state.eq(RX_DPH_2)
                    )
                ],
                RX_DPH_1: [
                    If(self.rx_dpp_done,
                        If(self.rx_dpp_crcgood,
                            in_dpp_seq.eq(in_dpp_seq + 1)
                        ),
                        self.err_missed_dpp_start.eq(0),
                        self.err_missed_dpp_done.eq(0),
                        rx_state.eq(RX_DPH_2)
                    ),
                    If((self.ltssm_state != LT_U0) | (recv_count == 270),
                        self.err_missed_dpp_done.eq(1),
                        rx_state.eq(RX_DPH_2)
                    )
                ],
                RX_DPH_2: [
                    # send ACK (TP A)
                    self.tx_tp_a.eq(1),
                    self.tx_tp_a_retry.eq(Mux(self.rx_dpp_crcgood &
                                              ~self.err_missed_dpp_start &
                                              ~self.err_missed_dpp_done,
                                              LP_TP_NORETRY, LP_TP_RETRY)),
                    self.tx_tp_a_dir.eq(LP_TP_HOSTTODEVICE),
                    self.tx_tp_a_subtype.eq(LP_TP_SUB_ACK),
                    self.tx_tp_a_endp.eq(self.rx_dph_endp),
                    self.tx_tp_a_nump.eq(1),
                    self.tx_tp_a_seq.eq(in_dpp_seq),
                    self.tx_tp_a_stream.eq(0),

                    If(self.tx_tp_a_ack,
                        rx_state.eq(RX_IDLE)
                    )
                ],
                RX_TP_0: [
                    # default: return to idle unless we need to stall (like Verilog)
                    rx_state.eq(RX_IDLE),

                    Case(self.rx_tp_subtype, {
                        LP_TP_SUB_ACK: [
                            If(self.rx_tp_pktpend & (self.rx_tp_nump > 0),
                                tx_endp.eq(self.rx_tp_endp),
                                out_nump.eq(self.rx_tp_nump),
                                out_dpp_seq.eq(self.rx_tp_seq),
                                do_send_dpp.eq(1),
                            )
                        ],
                        LP_TP_SUB_NRDY: [
                            # no-op in original
                        ],
                        LP_TP_SUB_ERDY: [
                            # no-op in original
                        ],
                        LP_TP_SUB_STATUS: [
                            self.tx_tp_b.eq(1),
                            self.tx_tp_b_retry.eq(LP_TP_NORETRY),
                            self.tx_tp_b_dir.eq(LP_TP_HOSTTODEVICE),
                            self.tx_tp_b_subtype.eq(LP_TP_SUB_ACK),
                            self.tx_tp_b_endp.eq(self.rx_tp_endp),
                            self.tx_tp_b_nump.eq(0),
                            self.tx_tp_b_seq.eq(in_dpp_seq),
                            self.tx_tp_b_stream.eq(0),

                            If(~self.tx_tp_b_ack,
                                rx_state.eq(RX_TP_0)  # hold state until ack
                            )
                        ],
                        LP_TP_SUB_PING: [
                            # no-op in original
                        ],
                        "default": [
                            self.err_tp_subtype.eq(1)
                        ]
                    })
                ],
            })
        )

        # TX machine -------------------------------------------------------------------------------
        self.sync += If(~self.reset_n,
            tx_state.eq(TX_RESET),
        ).Else(
            Case(tx_state, {
                TX_RESET: [
                    tx_state.eq(TX_IDLE)
                ],
                TX_IDLE: [
                    If(do_send_dpp,
                        # EP1 special-case: request IN data from external side.
                        If(tx_endp == SEL_ENDP1,
                            self.ext_buf_in_request.eq(1)
                        ),
                        If(self.buf_out_hasdata,
                            out_length.eq(self.buf_out_len),
                            tx_state.eq(TX_DP_0),
                        ).Else(
                            tx_state.eq(TX_DP_NRDY)
                        )
                    )
                ],
                TX_DP_NRDY: [
                    self.tx_tp_c.eq(1),
                    self.tx_tp_c_retry.eq(LP_TP_NORETRY),
                    self.tx_tp_c_dir.eq(LP_TP_HOSTTODEVICE),
                    self.tx_tp_c_subtype.eq(LP_TP_SUB_NRDY),
                    self.tx_tp_c_endp.eq(tx_endp),
                    self.tx_tp_c_nump.eq(0),
                    self.tx_tp_c_seq.eq(0),
                    self.tx_tp_c_stream.eq(0),

                    If(self.tx_tp_c_ack,
                        tx_state.eq(TX_DP_WAITDATA)
                    )
                ],
                TX_DP_WAITDATA: [
                    If(tx_endp == SEL_ENDP1,
                        self.ext_buf_in_request.eq(1)
                    ),
                    If(self.buf_out_hasdata,
                        out_length.eq(self.buf_out_len),
                        tx_state.eq(TX_DP_ERDY)
                    )
                ],
                TX_DP_ERDY: [
                    self.tx_tp_c.eq(1),
                    self.tx_tp_c_retry.eq(LP_TP_NORETRY),
                    self.tx_tp_c_dir.eq(LP_TP_HOSTTODEVICE),
                    self.tx_tp_c_subtype.eq(LP_TP_SUB_ERDY),
                    self.tx_tp_c_endp.eq(tx_endp),
                    self.tx_tp_c_nump.eq(1),
                    self.tx_tp_c_seq.eq(0),
                    self.tx_tp_c_stream.eq(0),

                    If(self.tx_tp_c_ack,
                        tx_state.eq(TX_DP_0)
                    )
                ],
                TX_DP_0: [
                    self.tx_dph.eq(1),
                    self.tx_dph_eob.eq(0),  # TODO like Verilog
                    self.tx_dph_dir.eq(Mux(tx_endp == 0, 0, LP_TP_DEVICETOHOST)),
                    self.tx_dph_endp.eq(tx_endp),
                    self.tx_dph_seq.eq(out_dpp_seq),
                    self.tx_dph_len.eq(out_length),

                    dc.eq(0),
                    If(self.tx_dpp_ack,
                        tx_state.eq(TX_DP_1)
                    )
                ],
                TX_DP_1: [
                    If(self.tx_dpp_done,
                        tx_state.eq(TX_IDLE)
                    )
                ],
            })
        )

        # Missed transaction detection -------------------------------------------------------------
        self.sync += [
            If(rx_state != RX_IDLE,
                If(self.rx_dph | self.rx_tp,
                    self.err_miss_rx.eq(1)
                )
            ),
            If(tx_state != TX_IDLE,
                If(do_send_dpp,
                    self.err_miss_tx.eq(1)
                )
            ),
        ]

        # Error clears on reset (as in Verilog) ---------------------------------------------------
        self.sync += If(~self.reset_n,
            self.err_miss_rx.eq(0),
            self.err_miss_tx.eq(0),
            self.err_tp_subtype.eq(0),
            self.err_missed_dpp_start.eq(0),
            self.err_missed_dpp_done.eq(0),
        )

        # -----------------------------------------------------------------------------------------
        # Endpoint instances (kept as Verilog blackboxes for progressive rewrite)
        # -----------------------------------------------------------------------------------------
        # EP0: control
        self.specials += Instance("usb3_ep0",
            i_slow_clk           = self.slow_clk,
            i_local_clk          = self.local_clk,
            i_reset_n            = self.reset_n,

            i_buf_in_addr        = ep0_buf_in_addr,
            i_buf_in_data        = ep0_buf_in_data,
            i_buf_in_wren        = ep0_buf_in_wren,
            o_buf_in_ready       = ep0_buf_in_ready,
            i_buf_in_commit      = ep0_buf_in_commit,
            i_buf_in_commit_len  = ep0_buf_in_commit_len,
            o_buf_in_commit_ack  = ep0_buf_in_commit_ack,

            i_buf_out_addr       = ep0_buf_out_addr,
            o_buf_out_q          = ep0_buf_out_q,
            o_buf_out_len        = ep0_buf_out_len,
            o_buf_out_hasdata    = ep0_buf_out_hasdata,
            i_buf_out_arm        = ep0_buf_out_arm,
            o_buf_out_arm_ack    = ep0_buf_out_arm_ack,

            o_vend_req_act       = self.vend_req_act,
            o_vend_req_request   = self.vend_req_request,
            o_vend_req_val       = self.vend_req_val,

            o_dev_addr           = self.dev_addr,
            o_configured         = self.configured,
            o_reset_dp_seq       = reset_dp_seq,
        )

        # EP1: IN to host (DATA TO PC)
        self.specials += Instance("usb3_ep",
            i_slow_clk           = self.slow_clk,
            i_local_clk          = self.local_clk,
            i_rd_clk             = self.local_clk,
            i_wr_clk             = self.ext_clk,
            i_reset_n            = self.reset_n,

            i_buf_in_addr        = self.ext_buf_in_addr,
            i_buf_in_data        = self.ext_buf_in_data,
            i_buf_in_wren        = self.ext_buf_in_wren,
            o_buf_in_ready       = self.ext_buf_in_ready,
            i_buf_in_commit      = self.ext_buf_in_commit,
            i_buf_in_commit_len  = self.ext_buf_in_commit_len,
            o_buf_in_commit_ack  = self.ext_buf_in_commit_ack,

            i_buf_out_addr       = ep1_buf_out_addr,
            o_buf_out_q          = ep1_buf_out_q,
            o_buf_out_len        = ep1_buf_out_len,
            o_buf_out_hasdata    = ep1_buf_out_hasdata,
            i_buf_out_arm        = ep1_buf_out_arm,
            o_buf_out_arm_ack    = ep1_buf_out_arm_ack,

            i_mode               = EP1_MODE,
        )

        # EP2: OUT from host (DATA FROM PC)
        self.specials += Instance("usb3_ep",
            i_slow_clk           = self.slow_clk,
            i_local_clk          = self.local_clk,
            i_rd_clk             = self.ext_clk,
            i_wr_clk             = self.local_clk,
            i_reset_n            = self.reset_n,

            i_buf_in_addr        = ep2_buf_in_addr,
            i_buf_in_data        = ep2_buf_in_data,
            i_buf_in_wren        = ep2_buf_in_wren,
            o_buf_in_ready       = ep2_buf_in_ready,
            i_buf_in_commit      = ep2_buf_in_commit,
            i_buf_in_commit_len  = ep2_buf_in_commit_len,
            o_buf_in_commit_ack  = ep2_buf_in_commit_ack,

            i_buf_out_addr       = self.ext_buf_out_addr,
            o_buf_out_q          = self.ext_buf_out_q,
            o_buf_out_len        = self.ext_buf_out_len,
            o_buf_out_hasdata    = self.ext_buf_out_hasdata,
            i_buf_out_arm        = self.ext_buf_out_arm,
            o_buf_out_arm_ack    = self.ext_buf_out_arm_ack,

            i_mode               = EP2_MODE,
        )
