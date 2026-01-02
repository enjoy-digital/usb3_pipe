#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2013 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *
from litex.gen import *

from usb3_core.daisho_mod.usb3.usb3_protocol import USB3Protocol

# USB3 Top -----------------------------------------------------------------------------------------

class USB3Top(LiteXModule):
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
        # USB 3.0 Protocol layer (LiteX)
        # ------------------------------------------------------------

        self.usb3_protocol = usb3_protocol = USB3Protocol()
        self.submodules += usb3_protocol

        # Clocks/resets.
        self.comb += [
            usb3_protocol.local_clk.eq(self.clk),
            usb3_protocol.slow_clk.eq(self.clk),
            usb3_protocol.ext_clk.eq(self.clk),

            usb3_protocol.reset_n.eq(reset_n),
            usb3_protocol.ltssm_state.eq(self.ltssm_state),
        ]

        # Link -> Protocol (RX).
        self.comb += [
            usb3_protocol.rx_tp.eq(prot_rx_tp),
            usb3_protocol.rx_tp_hosterr.eq(prot_rx_tp_hosterr),
            usb3_protocol.rx_tp_retry.eq(prot_rx_tp_retry),
            usb3_protocol.rx_tp_pktpend.eq(prot_rx_tp_pktpend),
            usb3_protocol.rx_tp_subtype.eq(prot_rx_tp_subtype),
            usb3_protocol.rx_tp_endp.eq(prot_rx_tp_endp),
            usb3_protocol.rx_tp_nump.eq(prot_rx_tp_nump),
            usb3_protocol.rx_tp_seq.eq(prot_rx_tp_seq),
            usb3_protocol.rx_tp_stream.eq(prot_rx_tp_stream),

            usb3_protocol.rx_dph.eq(prot_rx_dph),
            usb3_protocol.rx_dph_eob.eq(prot_rx_dph_eob),
            usb3_protocol.rx_dph_setup.eq(prot_rx_dph_setup),
            usb3_protocol.rx_dph_pktpend.eq(prot_rx_dph_pktpend),
            usb3_protocol.rx_dph_endp.eq(prot_rx_dph_endp),
            usb3_protocol.rx_dph_seq.eq(prot_rx_dph_seq),
            usb3_protocol.rx_dph_len.eq(prot_rx_dph_len),
            usb3_protocol.rx_dpp_start.eq(prot_rx_dpp_start),
            usb3_protocol.rx_dpp_done.eq(prot_rx_dpp_done),
            usb3_protocol.rx_dpp_crcgood.eq(prot_rx_dpp_crcgood),
        ]

        # Protocol -> Link (TX).
        self.comb += [
            prot_tx_tp_a.eq(usb3_protocol.tx_tp_a),
            prot_tx_tp_a_retry.eq(usb3_protocol.tx_tp_a_retry),
            prot_tx_tp_a_dir.eq(usb3_protocol.tx_tp_a_dir),
            prot_tx_tp_a_subtype.eq(usb3_protocol.tx_tp_a_subtype),
            prot_tx_tp_a_endp.eq(usb3_protocol.tx_tp_a_endp),
            prot_tx_tp_a_nump.eq(usb3_protocol.tx_tp_a_nump),
            prot_tx_tp_a_seq.eq(usb3_protocol.tx_tp_a_seq),
            prot_tx_tp_a_stream.eq(usb3_protocol.tx_tp_a_stream),
            usb3_protocol.tx_tp_a_ack.eq(prot_tx_tp_a_ack),

            prot_tx_tp_b.eq(usb3_protocol.tx_tp_b),
            prot_tx_tp_b_retry.eq(usb3_protocol.tx_tp_b_retry),
            prot_tx_tp_b_dir.eq(usb3_protocol.tx_tp_b_dir),
            prot_tx_tp_b_subtype.eq(usb3_protocol.tx_tp_b_subtype),
            prot_tx_tp_b_endp.eq(usb3_protocol.tx_tp_b_endp),
            prot_tx_tp_b_nump.eq(usb3_protocol.tx_tp_b_nump),
            prot_tx_tp_b_seq.eq(usb3_protocol.tx_tp_b_seq),
            prot_tx_tp_b_stream.eq(usb3_protocol.tx_tp_b_stream),
            usb3_protocol.tx_tp_b_ack.eq(prot_tx_tp_b_ack),

            prot_tx_tp_c.eq(usb3_protocol.tx_tp_c),
            prot_tx_tp_c_retry.eq(usb3_protocol.tx_tp_c_retry),
            prot_tx_tp_c_dir.eq(usb3_protocol.tx_tp_c_dir),
            prot_tx_tp_c_subtype.eq(usb3_protocol.tx_tp_c_subtype),
            prot_tx_tp_c_endp.eq(usb3_protocol.tx_tp_c_endp),
            prot_tx_tp_c_nump.eq(usb3_protocol.tx_tp_c_nump),
            prot_tx_tp_c_seq.eq(usb3_protocol.tx_tp_c_seq),
            prot_tx_tp_c_stream.eq(usb3_protocol.tx_tp_c_stream),
            usb3_protocol.tx_tp_c_ack.eq(prot_tx_tp_c_ack),

            prot_tx_dph.eq(usb3_protocol.tx_dph),
            prot_tx_dph_eob.eq(usb3_protocol.tx_dph_eob),
            prot_tx_dph_dir.eq(usb3_protocol.tx_dph_dir),
            prot_tx_dph_endp.eq(usb3_protocol.tx_dph_endp),
            prot_tx_dph_seq.eq(usb3_protocol.tx_dph_seq),
            prot_tx_dph_len.eq(usb3_protocol.tx_dph_len),
            usb3_protocol.tx_dpp_ack.eq(prot_tx_dpp_ack),
            usb3_protocol.tx_dpp_done.eq(prot_tx_dpp_done),
        ]

        # Link <-> Protocol buffer interface (prot_buf_*).
        self.comb += [
            # Protocol outputs driving link inputs.
            prot_buf_in_addr.eq(usb3_protocol.buf_in_addr),
            prot_buf_in_data.eq(usb3_protocol.buf_in_data),
            prot_buf_in_wren.eq(usb3_protocol.buf_in_wren),
            prot_buf_in_commit.eq(usb3_protocol.buf_in_commit),
            prot_buf_in_commit_len.eq(usb3_protocol.buf_in_commit_len),

            prot_buf_out_addr.eq(usb3_protocol.buf_out_addr),
            prot_buf_out_arm.eq(usb3_protocol.buf_out_arm),

            # Link outputs driving protocol inputs.
            usb3_protocol.buf_in_ready.eq(prot_buf_in_ready),
            usb3_protocol.buf_in_commit_ack.eq(prot_buf_in_commit_ack),

            usb3_protocol.buf_out_q.eq(prot_buf_out_q),
            usb3_protocol.buf_out_len.eq(prot_buf_out_len),
            usb3_protocol.buf_out_hasdata.eq(prot_buf_out_hasdata),
            usb3_protocol.buf_out_arm_ack.eq(prot_buf_out_arm_ack),
        ]

        # External interface (Top-level ports) -> protocol.
        self.comb += [
            usb3_protocol.ext_buf_in_addr.eq(self.buf_in_addr),
            usb3_protocol.ext_buf_in_data.eq(self.buf_in_data),
            usb3_protocol.ext_buf_in_wren.eq(self.buf_in_wren),
            usb3_protocol.ext_buf_in_commit.eq(self.buf_in_commit),
            usb3_protocol.ext_buf_in_commit_len.eq(self.buf_in_commit_len),

            usb3_protocol.ext_buf_out_addr.eq(self.buf_out_addr),
            usb3_protocol.ext_buf_out_arm.eq(self.buf_out_arm),

            # Protocol -> top-level.
            self.buf_in_request.eq(usb3_protocol.ext_buf_in_request),
            self.buf_in_ready.eq(usb3_protocol.ext_buf_in_ready),
            self.buf_in_commit_ack.eq(usb3_protocol.ext_buf_in_commit_ack),

            self.buf_out_q.eq(usb3_protocol.ext_buf_out_q),
            self.buf_out_len.eq(usb3_protocol.ext_buf_out_len),
            self.buf_out_hasdata.eq(usb3_protocol.ext_buf_out_hasdata),
            self.buf_out_arm_ack.eq(usb3_protocol.ext_buf_out_arm_ack),

            self.vend_req_act.eq(usb3_protocol.vend_req_act),
            self.vend_req_request.eq(usb3_protocol.vend_req_request),
            self.vend_req_val.eq(usb3_protocol.vend_req_val),

            prot_endp_mode_rx.eq(usb3_protocol.endp_mode_rx),
            prot_endp_mode_tx.eq(usb3_protocol.endp_mode_tx),
            prot_dev_addr.eq(usb3_protocol.dev_addr),
            prot_configured.eq(usb3_protocol.configured),
        ]

        # ------------------------------------------------------------
        # USB 3.0 Link layer interface (Verilog)
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
