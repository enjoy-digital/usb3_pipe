#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2013 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from litex.gen import *


# USB3 Top (LiteX) ---------------------------------------------------------------------------------

class USB3TopUSB3Pipe(LiteXModule):
    """USB3.0 top-level (LiteX)

    LiteX/Migen equivalent of the Verilog usb3_top_usb3_pipe top-level:
    - Instantiates usb3_link and usb3_protocol.
    - Recreates and wires all intermediate prot_* signals.
    - Exposes the same external interface signals as the original module.
    """
    def __init__(self, platform):
        # Clock/Reset ------------------------------------------------------------------------------
        self.clk               = Signal()
        self.reset             = Signal()

        # Inputs ----------------------------------------------------------------------------------
        self.ltssm_state       = Signal(5)

        self.in_data           = Signal(32)
        self.in_datak          = Signal(4)
        self.in_active         = Signal()

        self.out_stall         = Signal()

        self.buf_in_addr       = Signal(9)
        self.buf_in_data       = Signal(32)
        self.buf_in_wren       = Signal()
        self.buf_in_commit     = Signal()
        self.buf_in_commit_len = Signal(11)

        self.buf_out_addr      = Signal(9)
        self.buf_out_arm       = Signal()

        # Outputs ---------------------------------------------------------------------------------
        self.out_data          = Signal(32)
        self.out_datak         = Signal(4)
        self.out_active        = Signal()

        self.buf_in_request    = Signal()
        self.buf_in_ready      = Signal()
        self.buf_in_commit_ack = Signal()

        self.buf_out_q         = Signal(32)
        self.buf_out_len       = Signal(11)
        self.buf_out_hasdata   = Signal()
        self.buf_out_arm_ack   = Signal()

        self.vend_req_act      = Signal()
        self.vend_req_request  = Signal(8)
        self.vend_req_val      = Signal(16)

        # # #

        reset_n   = Signal()
        self.comb += reset_n.eq(~self.reset)

        # ------------------------------------------------------------
        # USB 3.0 Protocol layer interface nets (prot_*)
        # ------------------------------------------------------------

        prot_endp_mode_rx      = Signal(2)
        prot_endp_mode_tx      = Signal(2)
        prot_dev_addr          = Signal(7)
        prot_configured        = Signal()

        prot_rx_tp             = Signal()
        prot_rx_tp_hosterr     = Signal()
        prot_rx_tp_retry       = Signal()
        prot_rx_tp_pktpend     = Signal()
        prot_rx_tp_subtype     = Signal(4)
        prot_rx_tp_endp        = Signal(4)
        prot_rx_tp_nump        = Signal(5)
        prot_rx_tp_seq         = Signal(5)
        prot_rx_tp_stream      = Signal(16)

        prot_rx_dph            = Signal()
        prot_rx_dph_eob        = Signal()
        prot_rx_dph_setup      = Signal()
        prot_rx_dph_pktpend    = Signal()
        prot_rx_dph_endp       = Signal(4)
        prot_rx_dph_seq        = Signal(5)
        prot_rx_dph_len        = Signal(16)
        prot_rx_dpp_start      = Signal()
        prot_rx_dpp_done       = Signal()
        prot_rx_dpp_crcgood    = Signal()

        prot_tx_tp_a           = Signal()
        prot_tx_tp_a_retry     = Signal()
        prot_tx_tp_a_dir       = Signal()
        prot_tx_tp_a_subtype   = Signal(4)
        prot_tx_tp_a_endp      = Signal(4)
        prot_tx_tp_a_nump      = Signal(5)
        prot_tx_tp_a_seq       = Signal(5)
        prot_tx_tp_a_stream    = Signal(16)
        prot_tx_tp_a_ack       = Signal()

        prot_tx_tp_b           = Signal()
        prot_tx_tp_b_retry     = Signal()
        prot_tx_tp_b_dir       = Signal()
        prot_tx_tp_b_subtype   = Signal(4)
        prot_tx_tp_b_endp      = Signal(4)
        prot_tx_tp_b_nump      = Signal(5)
        prot_tx_tp_b_seq       = Signal(5)
        prot_tx_tp_b_stream    = Signal(16)
        prot_tx_tp_b_ack       = Signal()

        prot_tx_tp_c           = Signal()
        prot_tx_tp_c_retry     = Signal()
        prot_tx_tp_c_dir       = Signal()
        prot_tx_tp_c_subtype   = Signal(4)
        prot_tx_tp_c_endp      = Signal(4)
        prot_tx_tp_c_nump      = Signal(5)
        prot_tx_tp_c_seq       = Signal(5)
        prot_tx_tp_c_stream    = Signal(16)
        prot_tx_tp_c_ack       = Signal()

        prot_tx_dph            = Signal()
        prot_tx_dph_eob        = Signal()
        prot_tx_dph_dir        = Signal()
        prot_tx_dph_endp       = Signal(4)
        prot_tx_dph_seq        = Signal(5)
        prot_tx_dph_len        = Signal(16)
        prot_tx_dpp_ack        = Signal()
        prot_tx_dpp_done       = Signal()

        prot_buf_in_addr       = Signal(9)
        prot_buf_in_data       = Signal(32)
        prot_buf_in_wren       = Signal()
        prot_buf_in_ready      = Signal()
        prot_buf_in_commit     = Signal()
        prot_buf_in_commit_len = Signal(11)
        prot_buf_in_commit_ack = Signal()

        prot_buf_out_addr      = Signal(9)
        prot_buf_out_q         = Signal(32)
        prot_buf_out_len       = Signal(11)
        prot_buf_out_hasdata   = Signal()
        prot_buf_out_arm       = Signal()
        prot_buf_out_arm_ack   = Signal()

        # ------------------------------------------------------------
        # USB 3.0 Link layer interface
        # ------------------------------------------------------------

        # Unused/ignored in original top.
        ltssm_hot_reset   = 0
        ltssm_go_disabled = Signal()
        ltssm_go_recovery = Signal()
        ltssm_go_u        = Signal(2)

        self.specials += Instance("usb3_link",
            i_local_clk            = self.clk,
            i_reset_n              = reset_n,

            i_ltssm_state          = self.ltssm_state,
            i_ltssm_hot_reset      = ltssm_hot_reset,
            o_ltssm_go_disabled    = ltssm_go_disabled,
            o_ltssm_go_recovery    = ltssm_go_recovery,
            o_ltssm_go_u           = ltssm_go_u,

            i_in_data              = self.in_data,
            i_in_datak             = self.in_datak,
            i_in_active            = self.in_active,

            o_outp_data            = self.out_data,
            o_outp_datak           = self.out_datak,
            o_outp_active          = self.out_active,
            i_out_stall            = self.out_stall,

            o_endp_mode_rx         = prot_endp_mode_rx,
            o_endp_mode_tx         = prot_endp_mode_tx,

            o_prot_rx_tp           = prot_rx_tp,
            o_prot_rx_tp_hosterr   = prot_rx_tp_hosterr,
            o_prot_rx_tp_retry     = prot_rx_tp_retry,
            o_prot_rx_tp_pktpend   = prot_rx_tp_pktpend,
            o_prot_rx_tp_subtype   = prot_rx_tp_subtype,
            o_prot_rx_tp_endp      = prot_rx_tp_endp,
            o_prot_rx_tp_nump      = prot_rx_tp_nump,
            o_prot_rx_tp_seq       = prot_rx_tp_seq,
            o_prot_rx_tp_stream    = prot_rx_tp_stream,

            o_prot_rx_dph          = prot_rx_dph,
            o_prot_rx_dph_eob      = prot_rx_dph_eob,
            o_prot_rx_dph_setup    = prot_rx_dph_setup,
            o_prot_rx_dph_pktpend  = prot_rx_dph_pktpend,
            o_prot_rx_dph_endp     = prot_rx_dph_endp,
            o_prot_rx_dph_seq      = prot_rx_dph_seq,
            o_prot_rx_dph_len      = prot_rx_dph_len,
            o_prot_rx_dpp_start    = prot_rx_dpp_start,
            o_prot_rx_dpp_done     = prot_rx_dpp_done,
            o_prot_rx_dpp_crcgood  = prot_rx_dpp_crcgood,

            i_prot_tx_tp_a         = prot_tx_tp_a,
            i_prot_tx_tp_a_retry   = prot_tx_tp_a_retry,
            i_prot_tx_tp_a_dir     = prot_tx_tp_a_dir,
            i_prot_tx_tp_a_subtype = prot_tx_tp_a_subtype,
            i_prot_tx_tp_a_endp    = prot_tx_tp_a_endp,
            i_prot_tx_tp_a_nump    = prot_tx_tp_a_nump,
            i_prot_tx_tp_a_seq     = prot_tx_tp_a_seq,
            i_prot_tx_tp_a_stream  = prot_tx_tp_a_stream,
            i_prot_tx_tp_a_ack     = prot_tx_tp_a_ack,

            i_prot_tx_tp_b         = prot_tx_tp_b,
            i_prot_tx_tp_b_retry   = prot_tx_tp_b_retry,
            i_prot_tx_tp_b_dir     = prot_tx_tp_b_dir,
            i_prot_tx_tp_b_subtype = prot_tx_tp_b_subtype,
            i_prot_tx_tp_b_endp    = prot_tx_tp_b_endp,
            i_prot_tx_tp_b_nump    = prot_tx_tp_b_nump,
            i_prot_tx_tp_b_seq     = prot_tx_tp_b_seq,
            i_prot_tx_tp_b_stream  = prot_tx_tp_b_stream,
            i_prot_tx_tp_b_ack     = prot_tx_tp_b_ack,

            i_prot_tx_tp_c         = prot_tx_tp_c,
            i_prot_tx_tp_c_retry   = prot_tx_tp_c_retry,
            i_prot_tx_tp_c_dir     = prot_tx_tp_c_dir,
            i_prot_tx_tp_c_subtype = prot_tx_tp_c_subtype,
            i_prot_tx_tp_c_endp    = prot_tx_tp_c_endp,
            i_prot_tx_tp_c_nump    = prot_tx_tp_c_nump,
            i_prot_tx_tp_c_seq     = prot_tx_tp_c_seq,
            i_prot_tx_tp_c_stream  = prot_tx_tp_c_stream,
            i_prot_tx_tp_c_ack     = prot_tx_tp_c_ack,

            i_prot_tx_dph          = prot_tx_dph,
            i_prot_tx_dph_eob      = prot_tx_dph_eob,
            i_prot_tx_dph_dir      = prot_tx_dph_dir,
            i_prot_tx_dph_endp     = prot_tx_dph_endp,
            i_prot_tx_dph_seq      = prot_tx_dph_seq,
            i_prot_tx_dph_len      = prot_tx_dph_len,
            i_prot_tx_dpp_ack      = prot_tx_dpp_ack,
            i_prot_tx_dpp_done     = prot_tx_dpp_done,

            i_buf_in_addr          = prot_buf_in_addr,
            i_buf_in_data          = prot_buf_in_data,
            i_buf_in_wren          = prot_buf_in_wren,
            o_buf_in_ready         = prot_buf_in_ready,
            i_buf_in_commit        = prot_buf_in_commit,
            i_buf_in_commit_len    = prot_buf_in_commit_len,
            o_buf_in_commit_ack    = prot_buf_in_commit_ack,

            i_buf_out_addr         = prot_buf_out_addr,
            o_buf_out_q            = prot_buf_out_q,
            o_buf_out_len          = prot_buf_out_len,
            o_buf_out_hasdata      = prot_buf_out_hasdata,
            i_buf_out_arm          = prot_buf_out_arm,
            o_buf_out_arm_ack      = prot_buf_out_arm_ack,

            i_dev_addr             = prot_dev_addr,
        )

        # ------------------------------------------------------------
        # USB 3.0 Protocol layer interface
        # ------------------------------------------------------------

        self.specials += Instance("usb3_protocol",
            i_local_clk             = self.clk,  # FIXME ?
            i_slow_clk              = self.clk,  # FIXME ?
            i_ext_clk               = self.clk,  # FIXME ?

            i_reset_n               = reset_n,
            i_ltssm_state           = self.ltssm_state,

            i_endp_mode_rx          = prot_endp_mode_rx,
            i_endp_mode_tx          = prot_endp_mode_tx,

            i_rx_tp                 = prot_rx_tp,
            i_rx_tp_hosterr         = prot_rx_tp_hosterr,
            i_rx_tp_retry           = prot_rx_tp_retry,
            i_rx_tp_pktpend         = prot_rx_tp_pktpend,
            i_rx_tp_subtype         = prot_rx_tp_subtype,
            i_rx_tp_endp            = prot_rx_tp_endp,
            i_rx_tp_nump            = prot_rx_tp_nump,
            i_rx_tp_seq             = prot_rx_tp_seq,
            i_rx_tp_stream          = prot_rx_tp_stream,

            i_rx_dph                = prot_rx_dph,
            i_rx_dph_eob            = prot_rx_dph_eob,
            i_rx_dph_setup          = prot_rx_dph_setup,
            i_rx_dph_pktpend        = prot_rx_dph_pktpend,
            i_rx_dph_endp           = prot_rx_dph_endp,
            i_rx_dph_seq            = prot_rx_dph_seq,
            i_rx_dph_len            = prot_rx_dph_len,
            i_rx_dpp_start          = prot_rx_dpp_start,
            i_rx_dpp_done           = prot_rx_dpp_done,
            i_rx_dpp_crcgood        = prot_rx_dpp_crcgood,

            o_tx_tp_a               = prot_tx_tp_a,
            o_tx_tp_a_retry         = prot_tx_tp_a_retry,
            o_tx_tp_a_dir           = prot_tx_tp_a_dir,
            o_tx_tp_a_subtype       = prot_tx_tp_a_subtype,
            o_tx_tp_a_endp          = prot_tx_tp_a_endp,
            o_tx_tp_a_nump          = prot_tx_tp_a_nump,
            o_tx_tp_a_seq           = prot_tx_tp_a_seq,
            o_tx_tp_a_stream        = prot_tx_tp_a_stream,
            o_tx_tp_a_ack           = prot_tx_tp_a_ack,

            o_tx_tp_b               = prot_tx_tp_b,
            o_tx_tp_b_retry         = prot_tx_tp_b_retry,
            o_tx_tp_b_dir           = prot_tx_tp_b_dir,
            o_tx_tp_b_subtype       = prot_tx_tp_b_subtype,
            o_tx_tp_b_endp          = prot_tx_tp_b_endp,
            o_tx_tp_b_nump          = prot_tx_tp_b_nump,
            o_tx_tp_b_seq           = prot_tx_tp_b_seq,
            o_tx_tp_b_stream        = prot_tx_tp_b_stream,
            o_tx_tp_b_ack           = prot_tx_tp_b_ack,

            o_tx_tp_c               = prot_tx_tp_c,
            o_tx_tp_c_retry         = prot_tx_tp_c_retry,
            o_tx_tp_c_dir           = prot_tx_tp_c_dir,
            o_tx_tp_c_subtype       = prot_tx_tp_c_subtype,
            o_tx_tp_c_endp          = prot_tx_tp_c_endp,
            o_tx_tp_c_nump          = prot_tx_tp_c_nump,
            o_tx_tp_c_seq           = prot_tx_tp_c_seq,
            o_tx_tp_c_stream        = prot_tx_tp_c_stream,
            o_tx_tp_c_ack           = prot_tx_tp_c_ack,

            o_tx_dph                = prot_tx_dph,
            o_tx_dph_eob            = prot_tx_dph_eob,
            o_tx_dph_dir            = prot_tx_dph_dir,
            o_tx_dph_endp           = prot_tx_dph_endp,
            o_tx_dph_seq            = prot_tx_dph_seq,
            o_tx_dph_len            = prot_tx_dph_len,
            o_tx_dpp_ack            = prot_tx_dpp_ack,
            o_tx_dpp_done           = prot_tx_dpp_done,

            o_buf_in_addr           = prot_buf_in_addr,
            o_buf_in_data           = prot_buf_in_data,
            o_buf_in_wren           = prot_buf_in_wren,
            i_buf_in_ready          = prot_buf_in_ready,
            o_buf_in_commit         = prot_buf_in_commit,
            o_buf_in_commit_len     = prot_buf_in_commit_len,
            i_buf_in_commit_ack     = prot_buf_in_commit_ack,

            o_buf_out_addr          = prot_buf_out_addr,
            i_buf_out_q             = prot_buf_out_q,
            i_buf_out_len           = prot_buf_out_len,
            i_buf_out_hasdata       = prot_buf_out_hasdata,
            o_buf_out_arm           = prot_buf_out_arm,
            i_buf_out_arm_ack       = prot_buf_out_arm_ack,

            # External interface.
            i_ext_buf_in_addr       = self.buf_in_addr,
            i_ext_buf_in_data       = self.buf_in_data,
            i_ext_buf_in_wren       = self.buf_in_wren,
            o_ext_buf_in_request    = self.buf_in_request,
            o_ext_buf_in_ready      = self.buf_in_ready,
            i_ext_buf_in_commit     = self.buf_in_commit,
            i_ext_buf_in_commit_len = self.buf_in_commit_len,
            o_ext_buf_in_commit_ack = self.buf_in_commit_ack,

            i_ext_buf_out_addr      = self.buf_out_addr,
            o_ext_buf_out_q         = self.buf_out_q,
            o_ext_buf_out_len       = self.buf_out_len,
            o_ext_buf_out_hasdata   = self.buf_out_hasdata,
            i_ext_buf_out_arm       = self.buf_out_arm,
            o_ext_buf_out_arm_ack   = self.buf_out_arm_ack,

            o_vend_req_act          = self.vend_req_act,
            o_vend_req_request      = self.vend_req_request,
            o_vend_req_val          = self.vend_req_val,

            o_dev_addr              = prot_dev_addr,
            o_configured            = prot_configured,
        )
