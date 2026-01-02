#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2013 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen import *

# -------------------------------------------------------------------------------------------------
# usb3_link (LiteX/Migen translation of Daisho usb3_link)
# -------------------------------------------------------------------------------------------------
# Notes:
# - Keeps Verilog-style, single-clock sequential behavior in cd_usb3 (driven by local_clk/reset_n).
# - Keeps CRC blocks as Verilog Instances (drop-in compatible with existing RTL).
# - Bit ordering carefully mirrors the Verilog concatenations/slices used by Daisho.
# - outp_* are registered from internal out_* each cycle (same 1-cycle behavior as Verilog).
# -------------------------------------------------------------------------------------------------

# LTSSM states.
LT_SS_DISABLED         = 1
LT_POLLING_IDLE        = 15
LT_U0                  = 16
LT_U1                  = 17
LT_U2                  = 18
LT_U3                  = 19
LT_HOTRESET_EXIT       = 24
LT_RECOVERY_IDLE       = 29

# Timing (62.5MHz local clock).
T_U0_RECOVERY          = 125000
T_U0L_TIMEOUT          = 1250
T_PENDING_HP           = 376
T_CREDIT_HP            = 625000
T_PM_ENTRY             = 752
T_PORT_CONFIG          = 2550

# POWER, etc. (not all used here).

# Link commands (11-bit).
LCMD_LGOOD_0           = 0b00_00_000_0_000
LCMD_LGOOD_1           = 0b00_00_000_0_001
LCMD_LGOOD_2           = 0b00_00_000_0_010
LCMD_LGOOD_3           = 0b00_00_000_0_011
LCMD_LGOOD_4           = 0b00_00_000_0_100
LCMD_LGOOD_5           = 0b00_00_000_0_101
LCMD_LGOOD_6           = 0b00_00_000_0_110
LCMD_LGOOD_7           = 0b00_00_000_0_111

LCMD_LCRD_A            = 0b00_01_000_00_00
LCMD_LCRD_B            = 0b00_01_000_00_01
LCMD_LCRD_C            = 0b00_01_000_00_10
LCMD_LCRD_D            = 0b00_01_000_00_11

LCMD_LRTY              = 0b00_10_000_0000
LCMD_LBAD              = 0b00_11_000_0000

LCMD_LGO_U1            = 0b01_00_000_0001
LCMD_LGO_U2            = 0b01_00_000_0010
LCMD_LGO_U3            = 0b01_00_000_0011

LCMD_LAU               = 0b01_01_000_0000
LCMD_LXU               = 0b01_10_000_0000
LCMD_LPMA              = 0b01_11_000_0000

LCMD_LUP               = 0b10_00_000_0000
LCMD_LDN               = 0b10_11_000_0000

# Header packet types (5-bit).
LP_TYPE_LMP             = 0b00000
LP_TYPE_TP              = 0b00100
LP_TYPE_DP              = 0b01000
LP_TYPE_ITP             = 0b01100

# LMP subtypes (4-bit).
LP_LMP_SUB_SETLINK       = 0b0001
LP_LMP_SUB_U2INACT       = 0b0010
LP_LMP_SUB_VENDTEST      = 0b0011
LP_LMP_SUB_PORTCAP       = 0b0100
LP_LMP_SUB_PORTCFG       = 0b0101
LP_LMP_SUB_PORTCFGRSP    = 0b0110

# LMP misc.
LP_LMP_SPEED_5GBPS       = 0b0000001
LP_LMP_SPEED_ACCEPT      = 0b0000001
LP_LMP_NUM_HP_4          = 4
LP_LMP_DIR_UP            = 0b10
LP_LMP_OTG_INCAPABLE     = 0
LP_LMP_TIEBREAK          = 0

# TP misc.
LP_TP_ROUTE0             = 0
LP_TP_NORETRY            = 0
LP_TP_RETRY              = 1
LP_TP_HOSTTODEVICE       = 0
LP_TP_DEVICETOHOST       = 1
LP_TP_STREAMID           = 0
LP_TP_SSI_NO             = 0
LP_TP_WPA_NO             = 0
LP_TP_DBI_NO             = 0
LP_TP_PPEND_NO           = 0
LP_TP_NBI_0              = 0

# Ordered-sets / markers.
OS_CMDW  = 0xFEFEFEF7  # link command word marker
OS_HPST  = 0xFBFBFBF7  # HPSTART
OS_DPP   = 0x5C5C5CF7  # DPP ordered set
OS_EOP   = 0xFDFDFDF7  # EOP (pattern used by RTL)


def _swap16_from32(w32):
    # Verilog: {in_data[23:16], in_data[31:24], in_data[7:0], in_data[15:8]}
    return Cat(w32[8:16], w32[0:8], w32[24:32], w32[16:24])

def _swap32(w32):
    # Verilog swap32: {i[7:0], i[15:8], i[23:16], i[31:24]}
    return Cat(w32[24:32], w32[16:24], w32[8:16], w32[0:8])


class USB3Link(LiteXModule):
    def __init__(self):
        # -----------------------------------------------------------------------------------------
        # Ports (match Verilog)
        # -----------------------------------------------------------------------------------------
        self.slow_clk  = Signal()
        self.local_clk = Signal()
        self.reset_n   = Signal()

        self.ltssm_state       = Signal(5)
        self.ltssm_hot_reset   = Signal()
        self.ltssm_go_disabled = Signal()
        self.ltssm_go_u        = Signal(3)
        self.ltssm_go_recovery = Signal()

        # PIPE in/out
        self.in_data   = Signal(32)
        self.in_datak  = Signal(4)
        self.in_active = Signal()

        self.outp_data   = Signal(32)
        self.outp_datak  = Signal(4)
        self.outp_active = Signal()
        self.out_stall   = Signal()

        # Protocol interface
        self.endp_mode_rx = Signal(2)
        self.endp_mode_tx = Signal(2)
        self.dev_addr     = Signal(7)

        self.prot_rx_tp         = Signal()
        self.prot_rx_tp_hosterr = Signal()
        self.prot_rx_tp_retry   = Signal()
        self.prot_rx_tp_pktpend = Signal()
        self.prot_rx_tp_subtype = Signal(4)
        self.prot_rx_tp_endp    = Signal(4)
        self.prot_rx_tp_nump    = Signal(5)
        self.prot_rx_tp_seq     = Signal(5)
        self.prot_rx_tp_stream  = Signal(16)

        self.prot_rx_dph         = Signal()
        self.prot_rx_dph_eob     = Signal()
        self.prot_rx_dph_setup   = Signal()
        self.prot_rx_dph_pktpend = Signal()
        self.prot_rx_dph_endp    = Signal(4)
        self.prot_rx_dph_seq     = Signal(5)
        self.prot_rx_dph_len     = Signal(16)
        self.prot_rx_dpp_start   = Signal()
        self.prot_rx_dpp_done    = Signal()
        self.prot_rx_dpp_crcgood = Signal()

        self.prot_tx_tp_a         = Signal()
        self.prot_tx_tp_a_retry   = Signal()
        self.prot_tx_tp_a_dir     = Signal()
        self.prot_tx_tp_a_subtype = Signal(4)
        self.prot_tx_tp_a_endp    = Signal(4)
        self.prot_tx_tp_a_nump    = Signal(5)
        self.prot_tx_tp_a_seq     = Signal(5)
        self.prot_tx_tp_a_stream  = Signal(16)
        self.prot_tx_tp_a_ack     = Signal()

        self.prot_tx_tp_b         = Signal()
        self.prot_tx_tp_b_retry   = Signal()
        self.prot_tx_tp_b_dir     = Signal()
        self.prot_tx_tp_b_subtype = Signal(4)
        self.prot_tx_tp_b_endp    = Signal(4)
        self.prot_tx_tp_b_nump    = Signal(5)
        self.prot_tx_tp_b_seq     = Signal(5)
        self.prot_tx_tp_b_stream  = Signal(16)
        self.prot_tx_tp_b_ack     = Signal()

        self.prot_tx_tp_c         = Signal()
        self.prot_tx_tp_c_retry   = Signal()
        self.prot_tx_tp_c_dir     = Signal()
        self.prot_tx_tp_c_subtype = Signal(4)
        self.prot_tx_tp_c_endp    = Signal(4)
        self.prot_tx_tp_c_nump    = Signal(5)
        self.prot_tx_tp_c_seq     = Signal(5)
        self.prot_tx_tp_c_stream  = Signal(16)
        self.prot_tx_tp_c_ack     = Signal()

        self.prot_tx_dph      = Signal()
        self.prot_tx_dph_eob  = Signal()
        self.prot_tx_dph_dir  = Signal()
        self.prot_tx_dph_endp = Signal(4)
        self.prot_tx_dph_seq  = Signal(5)
        self.prot_tx_dph_len  = Signal(16)
        self.prot_tx_dpp_ack  = Signal()
        self.prot_tx_dpp_done = Signal()

        # Buffers
        self.buf_in_addr       = Signal(9)
        self.buf_in_data       = Signal(32)
        self.buf_in_wren       = Signal()
        self.buf_in_ready      = Signal()
        self.buf_in_commit     = Signal()
        self.buf_in_commit_len = Signal(11)
        self.buf_in_commit_ack = Signal()

        self.buf_out_addr      = Signal(10)
        self.buf_out_q         = Signal(32)
        self.buf_out_len       = Signal(11)
        self.buf_out_hasdata   = Signal()
        self.buf_out_arm       = Signal()
        self.buf_out_arm_ack   = Signal()

        # Errors
        self.err_lbad             = Signal()
        self.err_lbad_recv        = Signal()
        self.err_stuck_hpseq      = Signal()
        self.err_lcmd_undefined   = Signal()
        self.err_lcrd_mismatch    = Signal()
        self.err_lgood_order      = Signal()
        self.err_lgood_missed     = Signal()
        self.err_pending_hp       = Signal()
        self.err_credit_hp        = Signal()
        self.err_hp_crc           = Signal()
        self.err_hp_seq           = Signal()
        self.err_hp_type          = Signal()
        self.err_dpp_len_mismatch = Signal()

        # -----------------------------------------------------------------------------------------
        # Clock domain (cd_usb3 := local_clk)
        # -----------------------------------------------------------------------------------------
        self.clock_domains.cd_usb3 = ClockDomain()
        self.comb += [
            self.cd_usb3.clk.eq(self.local_clk),
            self.cd_usb3.rst.eq(~self.reset_n),
        ]

        # -----------------------------------------------------------------------------------------
        # Internal regs/wires
        # -----------------------------------------------------------------------------------------

        # Input pipelines (1..6).
        in_data_1   = Signal(32)
        in_data_2   = Signal(32)
        in_data_3   = Signal(32)
        in_data_4   = Signal(32)
        in_data_5   = Signal(32)
        in_data_6   = Signal(32)

        in_datak_1  = Signal(4)
        in_datak_2  = Signal(4)
        in_datak_3  = Signal(4)
        in_datak_4  = Signal(4)
        in_datak_5  = Signal(4)
        in_datak_6  = Signal(4)

        in_active_1 = Signal()
        in_active_2 = Signal()
        in_active_3 = Signal()
        in_active_4 = Signal()
        in_active_5 = Signal()
        in_active_6 = Signal()

        in_data_swap16 = Signal(32)
        in_data_swap32 = Signal(32)
        self.comb += [
            in_data_swap16.eq(_swap16_from32(self.in_data)),
            in_data_swap32.eq(_swap32(self.in_data)),
        ]

        # Outputs internal.
        out_data     = Signal(32)
        out_datak    = Signal(4)
        out_active   = Signal()

        out_data_1   = Signal(32)
        out_data_2   = Signal(32)
        out_datak_1  = Signal(4)
        out_datak_2  = Signal(4)
        out_active_1 = Signal()
        out_active_2 = Signal()

        # Link command / HP.
        rx_lcmd      = Signal(11)
        rx_lcmd_act  = Signal()
        tx_lcmd      = Signal(11)
        tx_lcmd_act  = Signal()
        tx_lcmd_act_1= Signal()
        tx_lcmd_queue= Signal()
        tx_lcmd_latch= Signal(11)
        tx_lcmd_out  = Signal(11)
        tx_lcmd_done = Signal()

        tx_hp_act    = Signal()
        tx_hp_act_1  = Signal()
        tx_hp_queue  = Signal()
        tx_hp_done   = Signal()
        tx_hp_dph    = Signal()
        tx_hp_retry  = Signal()
        queue_hp_retry = Signal()

        # Credits/seq.
        tx_hdr_seq_num     = Signal(3)
        last_hdr_seq_num   = Signal(3, reset=7)
        ack_tx_hdr_seq_num = Signal(3)
        rty_tx_hdr_seq_num = Signal(3)
        rx_hdr_seq_num     = Signal(3)
        rx_hdr_seq_num_dec = Signal(3)
        rx_hdr_seq_ignore  = Signal()

        local_rx_cred_count = Signal(3, reset=4)
        remote_rx_cred_count= Signal(3)
        remote_rx_cred_count_inc = Signal()
        remote_rx_cred_count_dec = Signal()

        tx_cred_idx = Signal(2)
        rx_cred_idx = Signal(2)
        rx_cred_idx_cache = Signal(2)

        # HP queues/buffers (note: receive stores as {w2,w1,w0} like Verilog).
        in_header_pkt_queued = Signal(4)
        in_header_pkt_pick   = Signal(2)
        in_header_pkt_a      = Signal(96)
        in_header_pkt_b      = Signal(96)
        in_header_pkt_c      = Signal(96)
        in_header_pkt_d      = Signal(96)

        out_header_pkt_pick  = Signal(2)
        out_header_pkt_a     = Signal(96)
        out_header_pkt_b     = Signal(96)
        out_header_pkt_c     = Signal(96)
        out_header_pkt_d     = Signal(96)

        in_header_pkt_mux  = Signal(96)
        out_header_pkt_mux = Signal(96)

        self.comb += [
            # in_header_pkt_mux: pick order mirrors Verilog.
            in_header_pkt_mux.eq(
                Mux(in_header_pkt_pick == 3, in_header_pkt_a,
                Mux(in_header_pkt_pick == 2, in_header_pkt_b,
                Mux(in_header_pkt_pick == 1, in_header_pkt_c, in_header_pkt_d)))
            ),
            out_header_pkt_mux.eq(
                Mux(out_header_pkt_pick == 0, out_header_pkt_a,
                Mux(out_header_pkt_pick == 1, out_header_pkt_b,
                Mux(out_header_pkt_pick == 2, out_header_pkt_c, out_header_pkt_d)))
            ),
        ]

        # HP control/CRC.
        in_header_cw      = Signal(16)
        in_header_crc     = Signal(16)
        in_header_seq     = Signal(3)
        self.comb += in_header_seq.eq(in_header_cw[0:3])
        in_header_err_count = Signal(2)

        out_header_cw = Signal(11)  # stored as 11-bit (cw[10:0]) in RTL
        out_header_cw_full = Signal(16)

        # Link command word capture.
        in_link_command     = Signal(32)
        in_link_command_1   = Signal(32)
        in_link_command_act = Signal()

        # Timers / LTSSM tracking.
        ltssm_last     = Signal(5)
        ltssm_stored   = Signal(5)
        ltssm_changed  = Signal()
        ltssm_go_recovery_1 = Signal()
        ltssm_go_u_1         = Signal(3)

        dc = Signal(25)
        u0l_timeout         = Signal(25)
        u0_recovery_timeout = Signal(25)
        u2_timeout          = Signal(25)
        pending_hp_timer    = Signal(25)
        credit_hp_timer     = Signal(25)
        pm_lc_timer         = Signal(25)   # kept for parity; unused
        pm_entry_timer      = Signal(25)
        ux_exit_timer       = Signal(25)
        port_config_timeout = Signal(25)
        T_PORT_U2_TIMEOUT   = Signal(25)

        link_error_count = Signal(8)

        force_linkpm_accept = Signal()
        pm_waiting_for_ack  = Signal()
        send_port_cfg_resp  = Signal()
        recv_port_cmdcfg    = Signal(2)

        # Queueing flags.
        tx_queue_open  = Signal()
        tx_queue_lup   = Signal()
        tx_queue_lcred = Signal(3)  # {strobe, idx[1:0]}
        tx_queue_lgood = Signal(4)  # {strobe, seq[2:0]}

        queue_send_u0_adv     = Signal()
        sent_u0_adv           = Signal()
        queue_send_u0_portcap = Signal()
        sent_u0_portcap       = Signal()
        recv_u0_adv           = Signal()
        out_header_first_since_entry = Signal()

        # TP/DP TX words.
        local_dev_addr = Signal(7)
        tx_hp_word_0   = Signal(32)
        tx_hp_word_1   = Signal(32)
        tx_hp_word_2   = Signal(32)

        # DPP RX/TX bookkeeping.
        in_dpp_length         = Signal(16)
        in_dpp_length_expect  = Signal(16)
        in_dpp_wasready       = Signal()
        in_dpp_crc32          = Signal(32)

        out_dpp_length          = Signal(16)
        out_dpp_length_remain   = Signal(16)
        out_dpp_length_remain_1 = Signal(16)

        # ITP value.
        itp_value = Signal(27)

        # Counters.
        qc = Signal(10)
        rc = Signal(10)
        sc = Signal(10)

        # CRC signals (instances below).
        crc_dpprx_q     = Signal(32)
        crc_dpprx_rst   = Signal()
        crc_dpprx_in    = Signal(32)
        crc_dpprx32_out = Signal(32)
        crc_dpprx24_out = Signal(32)
        crc_dpprx16_out = Signal(32)
        crc_dpprx8_out  = Signal(32)

        crc_dpptx_q     = Signal(32)
        crc_dpptx_rst   = Signal()
        crc_dpptx_in    = Signal(32)
        crc_dpptx32_out = Signal(32)
        crc_dpptx24_out = Signal(32)
        crc_dpptx16_out = Signal(32)
        crc_dpptx8_out  = Signal(32)
        crc_dpptx_out_1 = Signal(32)

        crc_hprx_rst = Signal()
        crc_hprx_in  = Signal(32)
        crc_hprx_out = Signal(16)

        crc_hptx_rst = Signal()
        crc_hptx_out = Signal(16)

        crc_cw1_out  = Signal(5)
        crc_cw2_out  = Signal(5)
        crc_cw3_out  = Signal(5)
        crc_cw4_out  = Signal(5)

        crc_lcmd_in  = Signal(11)
        crc_lcmd_out = Signal(5)

        # TX CRC selection for DPP (matches RTL).
        crc_dpptx_out = Signal(32)
        self.comb += crc_dpptx_out.eq(
            Mux(out_dpp_length_remain_1 == 4, _swap32(crc_dpptx32_out),
            Mux(out_dpp_length_remain_1 == 3, _swap32(crc_dpptx24_out),
            Mux(out_dpp_length_remain_1 == 2, _swap32(crc_dpptx16_out),
            Mux(out_dpp_length_remain_1 == 1, _swap32(crc_dpptx8_out),
                                         _swap32(crc_dpptx32_out)))))
        )

        # -----------------------------------------------------------------------------
        # CRC Instances (verbatim module names from Daisho RTL)
        # -----------------------------------------------------------------------------
        self.specials += Instance("usb3_crc_dpp32",
            i_clk     = self.local_clk,
            i_rst     = crc_dpprx_rst,
            i_crc_en  = in_active_6,
            i_di      = crc_dpprx_in,
            o_lfsr_q  = crc_dpprx_q,
            o_crc_out = crc_dpprx32_out,
        )
        self.specials += Instance("usb3_crc_dpp24",
            i_di      = crc_dpprx_in[8:32],
            i_q       = crc_dpprx_q,
            o_crc_out = crc_dpprx24_out,
        )
        self.specials += Instance("usb3_crc_dpp16",
            i_di      = crc_dpprx_in[16:32],
            i_q       = crc_dpprx_q,
            o_crc_out = crc_dpprx16_out,
        )
        self.specials += Instance("usb3_crc_dpp8",
            i_di      = crc_dpprx_in[24:32],
            i_q       = crc_dpprx_q,
            o_crc_out = crc_dpprx8_out,
        )

        self.specials += Instance("usb3_crc_dpp32",
            i_clk     = self.local_clk,
            i_rst     = crc_dpptx_rst,
            i_crc_en  = 1,
            i_di      = crc_dpptx_in,
            o_lfsr_q  = crc_dpptx_q,
            o_crc_out = crc_dpptx32_out,
        )
        self.specials += Instance("usb3_crc_dpp24",
            i_di      = crc_dpptx_in[0:24],
            i_q       = crc_dpptx_q,
            o_crc_out = crc_dpptx24_out,
        )
        self.specials += Instance("usb3_crc_dpp16",
            i_di      = crc_dpptx_in[0:16],
            i_q       = crc_dpptx_q,
            o_crc_out = crc_dpptx16_out,
        )
        self.specials += Instance("usb3_crc_dpp8",
            i_di      = crc_dpptx_in[0:8],
            i_q       = crc_dpptx_q,
            o_crc_out = crc_dpptx8_out,
        )

        self.specials += Instance("usb3_crc_hp",
            i_clk     = self.local_clk,
            i_rst     = crc_hprx_rst,
            i_crc_en  = self.in_active,
            i_di      = crc_hprx_in,
            o_crc_out = crc_hprx_out,
        )
        self.specials += Instance("usb3_crc_hp",
            i_clk     = self.local_clk,
            i_rst     = crc_hptx_rst,
            i_crc_en  = 1,
            i_di      = _swap32(out_data_2),
            o_crc_out = crc_hptx_out,
        )

        # CRC-5 (command words)
        self.specials += Instance("usb3_crc_cw",
            i_di      = in_link_command[16:27],
            o_crc_out = crc_cw1_out,
        )
        self.specials += Instance("usb3_crc_cw",
            i_di      = in_link_command[0:11],
            o_crc_out = crc_cw2_out,
        )
        self.specials += Instance("usb3_crc_cw",
            i_di      = in_header_cw[0:11],
            o_crc_out = crc_cw3_out,
        )
        self.specials += Instance("usb3_crc_cw",
            i_di      = out_header_cw[0:11],
            o_crc_out = crc_cw4_out,
        )
        self.specials += Instance("usb3_crc_cw",
            i_di      = crc_lcmd_in,
            o_crc_out = crc_lcmd_out,
        )

        # -----------------------------------------------------------------------------------------
        # State encodings (verbatim numbers from RTL)
        # -----------------------------------------------------------------------------------------

        send_state = Signal(6)
        LINK_SEND_RESET   = 0
        LINK_SEND_IDLE    = 2
        LINK_SEND_0       = 4
        LINK_SEND_1       = 6
        LINK_SEND_2       = 8
        LINK_SEND_3       = 10
        LINK_SEND_4       = 12
        LINK_SEND_CMDW_0  = 14
        LINK_SEND_CMDW_1  = 16
        LINK_SEND_HP_RTY  = 17
        LINK_SEND_HP_0    = 18
        LINK_SEND_HP_1    = 20
        LINK_SEND_HP_2    = 22

        recv_state = Signal(6)
        LINK_RECV_RESET  = 0
        LINK_RECV_IDLE   = 2
        LINK_RECV_HP_0   = 4
        LINK_RECV_HP_1   = 5
        LINK_RECV_DP_0   = 7
        LINK_RECV_DP_1   = 8
        LINK_RECV_DP_2   = 9
        LINK_RECV_DP_3   = 10
        LINK_RECV_CMDW_0 = 15
        LINK_RECV_CMDW_1 = 16
        LINK_RECV_CMDW_2 = 17
        LINK_RECV_CMDW_3 = 18
        LINK_RECV_CMDW_4 = 19

        read_dpp_state = Signal(6)
        READ_DPP_RESET = 0
        READ_DPP_IDLE  = 1
        READ_DPP_0     = 2
        READ_DPP_1     = 3
        READ_DPP_2     = 4
        READ_DPP_3     = 5
        READ_DPP_4     = 6
        READ_DPP_5     = 7
        READ_DPP_6     = 8

        write_dpp_state = Signal(6)
        WRITE_DPP_RESET = 0
        WRITE_DPP_IDLE  = 1
        WRITE_DPP_0     = 2
        WRITE_DPP_1     = 3
        WRITE_DPP_2     = 4
        WRITE_DPP_3     = 5
        WRITE_DPP_4     = 6
        WRITE_DPP_5     = 7
        WRITE_DPP_6     = 8
        WRITE_DPP_7     = 9
        WRITE_DPP_8     = 10
        WRITE_DPP_9     = 11
        WRITE_DPP_10    = 12
        WRITE_DPP_11    = 13
        WRITE_DPP_12    = 14
        WRITE_DPP_13    = 15

        check_hp_state = Signal(6)
        CHECK_HP_RESET = 0
        CHECK_HP_IDLE  = 1
        CHECK_HP_0     = 2
        CHECK_HP_1     = 3
        CHECK_HP_2     = 4
        CHECK_HP_3     = 5

        expect_state = Signal(6)
        LINK_EXPECT_RESET       = 0
        LINK_EXPECT_IDLE        = 2
        LINK_EXPECT_HDR_SEQ_AD  = 4
        LINK_EXPECT_HP_1        = 6
        LINK_EXPECT_HP_2        = 8
        LINK_EXPECT_HP_3        = 10

        queue_state = Signal(6)
        LINK_QUEUE_RESET         = 0
        LINK_QUEUE_IDLE          = 2
        LINK_QUEUE_HDR_SEQ_AD    = 4
        LINK_QUEUE_PORTCAP       = 6
        LINK_QUEUE_PORTCFGRSP    = 8
        LINK_QUEUE_RTY_HP        = 9
        LINK_QUEUE_TP_A          = 10
        LINK_QUEUE_TP_B          = 11
        LINK_QUEUE_DP            = 12
        LINK_QUEUE_TP_C          = 13

        rd_hp_state = Signal(5)
        RD_HP_RESET = 0
        RD_HP_IDLE  = 1
        RD_HP_LMP_0 = 4
        RD_HP_LMP_1 = 5
        RD_HP_TP_0  = 10
        RD_HP_TP_1  = 11
        RD_HP_TP_2  = 12
        RD_HP_DP_0  = 16
        RD_HP_DP_1  = 17
        RD_HP_DP_2  = 18
        RD_HP_0     = 20
        RD_HP_1     = 21
        RD_HP_2     = 22
        RD_HP_3     = 22  # note: RTL typo; keep as-is.

        rd_lcmd_state = Signal(4)
        RD_LCMD_RESET = 0
        RD_LCMD_IDLE  = 1
        RD_LCMD_0     = 2

        wr_lcmd_state = Signal(4)
        WR_LCMD_RESET = 0
        WR_LCMD_IDLE  = 1
        WR_LCMD_0     = 2
        WR_LCMD_1     = 3

        wr_hp_state = Signal(4)
        WR_HP_RESET = 0
        WR_HP_IDLE  = 1
        WR_HP_0     = 2
        WR_HP_1     = 3
        WR_HP_2     = 4
        WR_HP_3     = 5
        WR_HP_4     = 6
        WR_HP_5     = 7
        WR_HP_6     = 8
        WR_HP_7     = 9

        # -----------------------------------------------------------------------------------------
        # Main sequential logic (single always @(posedge local_clk) equivalent)
        # -----------------------------------------------------------------------------------------
        self.sync.usb3 += [

            # Track ltssm last + edges.
            ltssm_last.eq(self.ltssm_state),
            ltssm_changed.eq(0),
            ltssm_go_recovery_1.eq(self.ltssm_go_recovery),
            ltssm_go_u_1.eq(self.ltssm_go_u),

            If(ltssm_last != self.ltssm_state,
                ltssm_changed.eq(1),
                ltssm_stored.eq(ltssm_last),
            ),

            # Rising edge of go_recovery increments error count.
            If(self.ltssm_go_recovery & ~ltssm_go_recovery_1,
                link_error_count.eq(link_error_count + 1)
            ),

            # outp_* are registered from internal out_* (same as Verilog).
            self.outp_data.eq(out_data),
            self.outp_datak.eq(out_datak),
            self.outp_active.eq(out_active),

            # Clear internal out_* each cycle (writers overwrite).
            out_data.eq(0),
            out_datak.eq(0),
            out_active.eq(0),
            out_data_1.eq(0),
            out_datak_1.eq(0),
            out_active_1.eq(0),
            out_data_2.eq(0),
            out_datak_2.eq(0),
            out_active_2.eq(0),

            # Input pipelines shift (exact 6-stage).
            in_data_6.eq(in_data_5),
            in_data_5.eq(in_data_4),
            in_data_4.eq(in_data_3),
            in_data_3.eq(in_data_2),
            in_data_2.eq(in_data_1),
            in_data_1.eq(self.in_data),

            in_datak_6.eq(in_datak_5),
            in_datak_5.eq(in_datak_4),
            in_datak_4.eq(in_datak_3),
            in_datak_3.eq(in_datak_2),
            in_datak_2.eq(in_datak_1),
            in_datak_1.eq(self.in_datak),

            in_active_6.eq(in_active_5),
            in_active_5.eq(in_active_4),
            in_active_4.eq(in_active_3),
            in_active_3.eq(in_active_2),
            in_active_2.eq(in_active_1),
            in_active_1.eq(self.in_active),

            # Defaults for pulses/acts each cycle (matches " <= 0" style).
            tx_lcmd_act.eq(0),
            tx_lcmd_done.eq(0),
            rx_lcmd_act.eq(0),

            tx_hp_act.eq(0),
            tx_hp_done.eq(0),

            in_link_command_act.eq(0),

            crc_hprx_rst.eq(0),
            crc_hptx_rst.eq(0),
            crc_dpprx_rst.eq(0),
            crc_dpptx_rst.eq(0),

            self.buf_in_wren.eq(0),
            self.buf_in_commit.eq(0),
            self.buf_out_arm.eq(0),

            self.prot_rx_tp.eq(0),
            self.prot_rx_dph.eq(0),
            self.prot_tx_tp_a_ack.eq(0),
            self.prot_tx_tp_b_ack.eq(0),
            self.prot_tx_tp_c_ack.eq(0),
            self.prot_tx_dpp_ack.eq(0),

            # DPP pulses are regs in RTL; clear default here.
            self.prot_rx_dpp_start.eq(0),
            self.prot_rx_dpp_done.eq(0),
            self.prot_rx_dpp_crcgood.eq(0),

            # Track tx_lcmd/tx_hp "queue" latching (simplified but faithful):
            # In RTL, tx_lcmd_queue / tx_hp_queue are set when a strobe is seen.
            # Here, any assertion of tx_lcmd_act/tx_hp_act in this cycle causes queue to latch.
            If(tx_lcmd_act,
                tx_lcmd_queue.eq(1)
            ),
            If(tx_hp_act,
                tx_hp_queue.eq(1)
            ),

            # Preserve previous act for parity (not strictly needed with queue-latch).
            tx_lcmd_act_1.eq(tx_lcmd_act),
            tx_hp_act_1.eq(tx_hp_act),

            # Counters.
            If(self.in_data == 0,
                dc.eq(dc + 1)
            ),
            rc.eq(rc + 1),
            sc.eq(sc + 1),

            # Atomic remote cred count (+/-).
            remote_rx_cred_count_inc.eq(0),
            remote_rx_cred_count_dec.eq(0),
            remote_rx_cred_count.eq(remote_rx_cred_count + remote_rx_cred_count_inc - remote_rx_cred_count_dec),

            # rx_hdr_seq_num_dec wire.
            rx_hdr_seq_num_dec.eq(rx_hdr_seq_num - 1),

            # -------------------------------------------------------------------------------------
            # LTSSM timer behavior (verbatim structure)
            # -------------------------------------------------------------------------------------
            Case(self.ltssm_state, {
                LT_U0: [
                    pending_hp_timer.eq(pending_hp_timer + 1),
                    credit_hp_timer.eq(credit_hp_timer + 1),
                    pm_entry_timer.eq(pm_entry_timer + 1),
                    ux_exit_timer.eq(ux_exit_timer + 1),
                    port_config_timeout.eq(port_config_timeout + 1),

                    If(u0l_timeout < T_U0L_TIMEOUT,
                        u0l_timeout.eq(u0l_timeout + 1)
                    ),
                    If(u0_recovery_timeout < T_U0_RECOVERY,
                        u0_recovery_timeout.eq(u0_recovery_timeout + 1)
                    ),

                    # U0LTimeout -> send LUP keepalive.
                    If(u0l_timeout == T_U0L_TIMEOUT,
                        tx_queue_lup.eq(1),
                        u0l_timeout.eq(0)
                    ),

                    # Absence of heartbeat -> Recovery.
                    If(u0_recovery_timeout == T_U0_RECOVERY,
                        self.ltssm_go_recovery.eq(1)
                    ),

                    u2_timeout.eq(0),

                    # CREDIT_HP_TIMER logic.
                    If(remote_rx_cred_count == 4,
                        credit_hp_timer.eq(0)
                    ),
                    If(credit_hp_timer == T_CREDIT_HP,
                        credit_hp_timer.eq(T_CREDIT_HP),
                        If(wr_hp_state == WR_HP_IDLE,
                            self.err_credit_hp.eq(1),
                            self.ltssm_go_recovery.eq(1),
                        )
                    ),

                    # PENDING_HP_TIMER logic.
                    If((remote_rx_cred_count == 4) & recv_u0_adv,
                        pending_hp_timer.eq(0)
                    ),
                    If(pending_hp_timer == T_PENDING_HP,
                        pending_hp_timer.eq(T_PENDING_HP),
                        If(wr_hp_state == WR_HP_IDLE,
                            self.err_pending_hp.eq(1),
                            self.ltssm_go_recovery.eq(1),
                        )
                    ),

                    # PM_ENTRY_TIMER (LPMA timeout)
                    If(~pm_waiting_for_ack,
                        pm_entry_timer.eq(0)
                    ),
                    If(pm_entry_timer == T_PM_ENTRY,
                        pm_entry_timer.eq(T_PM_ENTRY),
                        If(pm_waiting_for_ack,
                            # strobe + target in ltssm_go_u[1:0]
                            self.ltssm_go_u.eq(Cat(self.ltssm_go_u[0:2], C(1, 1))),
                            pm_waiting_for_ack.eq(0),
                        )
                    ),

                    # tPortConfiguration.
                    If(recv_port_cmdcfg == 0b11,
                        port_config_timeout.eq(0)
                    ),
                    If(port_config_timeout == T_PORT_CONFIG,
                        port_config_timeout.eq(T_PORT_CONFIG),
                        If(recv_port_cmdcfg != 0b11,
                            self.ltssm_go_disabled.eq(1)
                        )
                    ),
                ],
                LT_U1: [
                    If(u2_timeout < T_PORT_U2_TIMEOUT,
                        u2_timeout.eq(u2_timeout + 1)
                    ),
                    If(u2_timeout == T_PORT_U2_TIMEOUT,
                        self.ltssm_go_u.eq(Cat(C(2, 2), C(1, 1)))  # {U2, strobe} packed later below
                    ),
                ],
                "default": [
                    u0l_timeout.eq(0),
                    u0_recovery_timeout.eq(0),
                    pending_hp_timer.eq(0),
                    credit_hp_timer.eq(0),
                    pm_lc_timer.eq(0),
                    pm_entry_timer.eq(0),
                    ux_exit_timer.eq(0),
                    port_config_timeout.eq(0),
                    self.ltssm_go_disabled.eq(0),
                ],
            }),

            # If not U0: clear adv/portcap flags
            If(self.ltssm_state != LT_U0,
                recv_u0_adv.eq(0),
                sent_u0_adv.eq(0),
                sent_u0_portcap.eq(0),
            ),

            # Handle LTSSM change.
            If(ltssm_changed,
                self.ltssm_go_u.eq(0),
                Case(self.ltssm_state, {
                    LT_U0: [
                        If((ltssm_stored == LT_POLLING_IDLE) | (ltssm_stored == LT_HOTRESET_EXIT),
                            tx_hdr_seq_num.eq(0),
                            rx_hdr_seq_num.eq(0),
                            rx_hdr_seq_ignore.eq(0),
                            queue_hp_retry.eq(0),
                            rty_tx_hdr_seq_num.eq(0),
                            last_hdr_seq_num.eq(7),
                            local_rx_cred_count.eq(4),

                            in_header_pkt_queued.eq(0),
                            in_header_pkt_a.eq(0),
                            in_header_pkt_b.eq(0),
                            in_header_pkt_c.eq(0),
                            in_header_pkt_d.eq(0),

                            link_error_count.eq(0),
                        ),

                        If((ltssm_stored == LT_POLLING_IDLE) | (ltssm_stored == LT_HOTRESET_EXIT),
                            queue_send_u0_portcap.eq(1),
                            recv_port_cmdcfg.eq(0),
                        ),
                        queue_send_u0_adv.eq(1),

                        tx_cred_idx.eq(0),
                        rx_cred_idx.eq(0),
                        remote_rx_cred_count.eq(0),

                        dc.eq(0),
                        force_linkpm_accept.eq(0),
                        pm_waiting_for_ack.eq(0),
                        tx_queue_lup.eq(0),
                        tx_queue_lcred.eq(0),
                        tx_queue_lgood.eq(0),
                        tx_queue_open.eq(1),
                        tx_lcmd_queue.eq(0),
                        send_port_cfg_resp.eq(0),

                        out_header_first_since_entry.eq(1),
                        in_header_err_count.eq(0),

                        queue_state.eq(LINK_QUEUE_RESET),
                        read_dpp_state.eq(READ_DPP_RESET),
                        rd_hp_state.eq(RD_HP_RESET),
                        check_hp_state.eq(CHECK_HP_RESET),
                    ],
                    LT_RECOVERY_IDLE: [
                        self.ltssm_go_recovery.eq(0),
                        rx_hdr_seq_ignore.eq(0),
                    ],
                    "default": [
                        self.ltssm_go_recovery.eq(0),
                    ],
                })
            ),

            # -------------------------------------------------------------------------------------
            # RX FSM (LINK_RECV)
            # -------------------------------------------------------------------------------------
            in_link_command_1.eq(in_link_command),

            Case(recv_state, {
                LINK_RECV_RESET: [
                    recv_state.eq(LINK_RECV_IDLE)
                ],
                LINK_RECV_IDLE: [
                    If((self.ltssm_state == LT_U0) & self.in_active,
                        If((self.in_data == OS_CMDW) & (self.in_datak == 0b1111),
                            u0_recovery_timeout.eq(0),
                            recv_state.eq(LINK_RECV_CMDW_0),
                        ),
                        If((self.in_data == OS_HPST) & (self.in_datak == 0b1111),
                            rc.eq(0),
                            crc_hprx_rst.eq(1),
                            recv_state.eq(LINK_RECV_HP_0),
                        ),
                    )
                ],
                LINK_RECV_HP_0: [
                    If(self.in_active,
                        Case(rx_cred_idx, {
                            0: in_header_pkt_a.eq(Cat(in_header_pkt_a[32:96], _swap32(self.in_data))),
                            1: in_header_pkt_b.eq(Cat(in_header_pkt_b[32:96], _swap32(self.in_data))),
                            2: in_header_pkt_c.eq(Cat(in_header_pkt_c[32:96], _swap32(self.in_data))),
                            3: in_header_pkt_d.eq(Cat(in_header_pkt_d[32:96], _swap32(self.in_data))),
                        }),
                        crc_hprx_in.eq(_swap32(self.in_data)),
                        If(rc == 2,
                            recv_state.eq(LINK_RECV_HP_1)
                        )
                    ).Else(
                        rc.eq(rc)
                    ),
                    If((~self.in_active) & (rc == 0),
                        crc_hprx_rst.eq(1)
                    )
                ],
                LINK_RECV_HP_1: [
                    If(self.in_active,
                        in_header_crc.eq(in_data_swap16[16:32]),
                        in_header_cw.eq(in_data_swap16[0:16]),
                        recv_state.eq(LINK_RECV_IDLE),
                        If(~rx_hdr_seq_ignore,
                            check_hp_state.eq(CHECK_HP_0)
                        )
                    )
                ],
                LINK_RECV_CMDW_0: [
                    If(self.in_active,
                        in_link_command.eq(in_data_swap16),
                        in_link_command_act.eq(1),
                        recv_state.eq(LINK_RECV_IDLE),
                    )
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Expect FSM
            # -------------------------------------------------------------------------------------
            Case(expect_state, {
                LINK_EXPECT_RESET: [
                    expect_state.eq(LINK_EXPECT_IDLE)
                ],
                LINK_EXPECT_IDLE: [
                    If((self.ltssm_state == LT_POLLING_IDLE) | (self.ltssm_state == LT_RECOVERY_IDLE),
                        expect_state.eq(LINK_EXPECT_HDR_SEQ_AD)
                    )
                ],
                LINK_EXPECT_HDR_SEQ_AD: [
                    If(self.ltssm_state == LT_U0,
                        Case(Cat(rx_lcmd, rx_lcmd_act), {
                            (LCMD_LGOOD_0 << 1) | 1: ack_tx_hdr_seq_num.eq(1),
                            (LCMD_LGOOD_1 << 1) | 1: ack_tx_hdr_seq_num.eq(2),
                            (LCMD_LGOOD_2 << 1) | 1: ack_tx_hdr_seq_num.eq(3),
                            (LCMD_LGOOD_3 << 1) | 1: ack_tx_hdr_seq_num.eq(4),
                            (LCMD_LGOOD_4 << 1) | 1: ack_tx_hdr_seq_num.eq(5),
                            (LCMD_LGOOD_5 << 1) | 1: ack_tx_hdr_seq_num.eq(6),
                            (LCMD_LGOOD_6 << 1) | 1: ack_tx_hdr_seq_num.eq(7),
                            (LCMD_LGOOD_7 << 1) | 1: ack_tx_hdr_seq_num.eq(0),

                            (LCMD_LCRD_A << 1) | 1: [
                                remote_rx_cred_count_inc.eq(1),
                                rx_cred_idx.eq(rx_cred_idx + 1),
                            ],
                            (LCMD_LCRD_B << 1) | 1: [
                                remote_rx_cred_count_inc.eq(1),
                                rx_cred_idx.eq(rx_cred_idx + 1),
                            ],
                            (LCMD_LCRD_C << 1) | 1: [
                                remote_rx_cred_count_inc.eq(1),
                                rx_cred_idx.eq(rx_cred_idx + 1),
                            ],
                            (LCMD_LCRD_D << 1) | 1: [
                                remote_rx_cred_count_inc.eq(1),
                                rx_cred_idx.eq(rx_cred_idx + 1),
                                pending_hp_timer.eq(0),
                                credit_hp_timer.eq(0),
                                recv_u0_adv.eq(1),
                                expect_state.eq(LINK_EXPECT_IDLE),
                            ],
                        })
                    )
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Read DPP FSM (RX)
            # -------------------------------------------------------------------------------------
            Case(read_dpp_state, {
                READ_DPP_RESET: [
                    read_dpp_state.eq(READ_DPP_IDLE)
                ],
                READ_DPP_IDLE: [
                    If((self.ltssm_state == LT_U0) & in_active_6,
                        read_dpp_state.eq(READ_DPP_0),
                        self.buf_in_wren.eq(0),
                    )
                ],
                READ_DPP_0: [
                    self.buf_in_addr.eq((2**9)-1),  # -1 in 9-bit
                    self.buf_in_wren.eq(0),
                    in_dpp_length.eq(0),
                    in_dpp_wasready.eq(0),
                    self.buf_in_commit_len.eq(in_dpp_length_expect[0:11]),
                    crc_dpprx_rst.eq(1),

                    If((in_data_6 == OS_DPP) & (in_datak_6 == 0b1111) & in_active_6,
                        read_dpp_state.eq(READ_DPP_1),
                        self.prot_rx_dpp_start.eq(1),
                        in_dpp_wasready.eq(self.buf_in_ready),
                    ),
                    If(self.ltssm_state != LT_U0,
                        read_dpp_state.eq(READ_DPP_RESET)
                    ),
                ],
                READ_DPP_1: [
                    If(in_active_6,
                        If((in_data_6 == OS_EOP) & (in_datak_6 == 0b1111),
                            read_dpp_state.eq(READ_DPP_IDLE),
                            self.err_dpp_len_mismatch.eq(1),
                        ).Else(
                            in_dpp_length.eq(in_dpp_length + 4),
                            self.buf_in_addr.eq(self.buf_in_addr + 1),
                            self.buf_in_data.eq(in_data_6),
                            crc_dpprx_in.eq(_swap32(in_data_6)),
                            self.buf_in_wren.eq(in_dpp_wasready),

                            If((in_dpp_length + 4) >= in_dpp_length_expect,
                                read_dpp_state.eq(READ_DPP_2)
                            )
                        )
                    ).Else(
                        If(in_dpp_length == 0,
                            crc_dpprx_rst.eq(1)
                        )
                    )
                ],
                READ_DPP_2: [
                    If(in_active_6,
                        in_dpp_crc32.eq(_swap32(in_data_6)),
                        read_dpp_state.eq(READ_DPP_3)
                    )
                ],
                READ_DPP_3: [
                    self.prot_rx_dpp_done.eq(1),
                    If(in_dpp_crc32 == crc_dpprx32_out,
                        self.prot_rx_dpp_crcgood.eq(1),
                        self.buf_in_commit.eq(1),
                    ).Else(
                        # RTL sets crcgood=1 even on fail (kept as-is)
                        self.prot_rx_dpp_crcgood.eq(1),
                        self.buf_in_commit.eq(1),
                    ),
                    read_dpp_state.eq(READ_DPP_IDLE)
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Check HP FSM
            # -------------------------------------------------------------------------------------
            Case(check_hp_state, {
                CHECK_HP_RESET: [
                    check_hp_state.eq(CHECK_HP_IDLE)
                ],
                CHECK_HP_IDLE: [
                    # triggered externally
                ],
                CHECK_HP_0: [
                    If((crc_hprx_out == in_header_crc) & (crc_cw3_out == in_header_cw[11:16]),
                        If((in_header_seq == rx_hdr_seq_num) & (local_rx_cred_count > 0),
                            check_hp_state.eq(CHECK_HP_1)
                        ).Else(
                            self.err_hp_seq.eq(1),
                            check_hp_state.eq(CHECK_HP_IDLE),
                            self.ltssm_go_recovery.eq(1),
                        )
                    ).Else(
                        self.err_lbad.eq(1),
                        tx_lcmd.eq(LCMD_LBAD),
                        tx_lcmd_act.eq(1),
                        self.err_hp_crc.eq(1),
                        in_header_err_count.eq(in_header_err_count + 1),
                        rx_hdr_seq_ignore.eq(1),
                        check_hp_state.eq(CHECK_HP_IDLE),

                        If(in_header_err_count >= 2,
                            tx_lcmd_act.eq(0),
                            self.ltssm_go_recovery.eq(1),
                        )
                    )
                ],
                CHECK_HP_1: [
                    in_header_err_count.eq(0),
                    local_rx_cred_count.eq(local_rx_cred_count - 1),
                    rx_hdr_seq_num.eq(rx_hdr_seq_num + 1),

                    # set dirty bit in queue: in_header_pkt_queued[3-rx_cred_idx] <= 1;
                    Case(rx_cred_idx, {
                        0: in_header_pkt_queued.eq(in_header_pkt_queued | 0b1000),
                        1: in_header_pkt_queued.eq(in_header_pkt_queued | 0b0100),
                        2: in_header_pkt_queued.eq(in_header_pkt_queued | 0b0010),
                        3: in_header_pkt_queued.eq(in_header_pkt_queued | 0b0001),
                    }),
                    rx_cred_idx_cache.eq(rx_cred_idx),
                    rx_cred_idx.eq(rx_cred_idx + 1),

                    # send LGOOD
                    tx_queue_lgood.eq(Cat(rx_hdr_seq_num, C(1, 1))),  # strobe+seq
                    check_hp_state.eq(CHECK_HP_IDLE),
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Parse Header Packet FSM (rd_hp_state)
            # -------------------------------------------------------------------------------------
            Case(rd_hp_state, {
                RD_HP_RESET: [
                    rd_hp_state.eq(RD_HP_IDLE)
                ],
                RD_HP_IDLE: [
                    # pick based on rx_cred_idx_cache rotation exactly like RTL
                    Case(rx_cred_idx_cache, {
                        0: [
                            If(in_header_pkt_queued[3], in_header_pkt_pick.eq(3)).Elif(
                               in_header_pkt_queued[2], in_header_pkt_pick.eq(2)).Elif(
                               in_header_pkt_queued[1], in_header_pkt_pick.eq(1)).Elif(
                               in_header_pkt_queued[0], in_header_pkt_pick.eq(0))
                        ],
                        1: [
                            If(in_header_pkt_queued[2], in_header_pkt_pick.eq(2)).Elif(
                               in_header_pkt_queued[1], in_header_pkt_pick.eq(1)).Elif(
                               in_header_pkt_queued[0], in_header_pkt_pick.eq(0)).Elif(
                               in_header_pkt_queued[3], in_header_pkt_pick.eq(3))
                        ],
                        2: [
                            If(in_header_pkt_queued[1], in_header_pkt_pick.eq(1)).Elif(
                               in_header_pkt_queued[0], in_header_pkt_pick.eq(0)).Elif(
                               in_header_pkt_queued[3], in_header_pkt_pick.eq(3)).Elif(
                               in_header_pkt_queued[2], in_header_pkt_pick.eq(2))
                        ],
                        3: [
                            If(in_header_pkt_queued[0], in_header_pkt_pick.eq(0)).Elif(
                               in_header_pkt_queued[3], in_header_pkt_pick.eq(3)).Elif(
                               in_header_pkt_queued[2], in_header_pkt_pick.eq(2)).Elif(
                               in_header_pkt_queued[1], in_header_pkt_pick.eq(1))
                        ],
                    }),
                    If(in_header_pkt_queued != 0,
                        rd_hp_state.eq(RD_HP_0)
                    )
                ],
                RD_HP_0: [
                    # Type is bits [4:0] of word0 (stored in low bits of in_header_pkt_mux)
                    Case(in_header_pkt_mux[0:5], {
                        LP_TYPE_LMP: [
                            Case(in_header_pkt_mux[5:9], {
                                LP_LMP_SUB_SETLINK: [
                                    force_linkpm_accept.eq(in_header_pkt_mux[10])
                                ],
                                LP_LMP_SUB_U2INACT: [
                                    # *32000 (256us units); keep math in 25-bit.
                                    T_PORT_U2_TIMEOUT.eq(in_header_pkt_mux[9:17] * 32000)
                                ],
                                LP_LMP_SUB_VENDTEST: [
                                ],
                                LP_LMP_SUB_PORTCAP: [
                                    recv_port_cmdcfg.eq(recv_port_cmdcfg | 0b10)
                                ],
                                LP_LMP_SUB_PORTCFG: [
                                    send_port_cfg_resp.eq(1),
                                    recv_port_cmdcfg.eq(recv_port_cmdcfg | 0b01)
                                ],
                                LP_LMP_SUB_PORTCFGRSP: [
                                ],
                            })
                        ],
                        LP_TYPE_TP: [
                            self.prot_rx_tp.eq(1),
                            self.prot_rx_tp_hosterr.eq(in_header_pkt_mux[32+15]),
                            self.prot_rx_tp_retry.eq(in_header_pkt_mux[32+6]),
                            self.prot_rx_tp_pktpend.eq(in_header_pkt_mux[64+27]),
                            self.prot_rx_tp_subtype.eq(in_header_pkt_mux[32+0:32+4]),
                            self.prot_rx_tp_endp.eq(in_header_pkt_mux[32+8:32+12]),
                            self.prot_rx_tp_nump.eq(in_header_pkt_mux[32+16:32+21]),
                            self.prot_rx_tp_seq.eq(in_header_pkt_mux[32+21:32+26]),
                            self.prot_rx_tp_stream.eq(in_header_pkt_mux[64:80]),
                        ],
                        LP_TYPE_DP: [
                            in_dpp_length_expect.eq(in_header_pkt_mux[32+16:32+32]),
                            self.prot_rx_dph.eq(1),
                            self.prot_rx_dph_eob.eq(in_header_pkt_mux[32+6]),
                            self.prot_rx_dph_setup.eq(in_header_pkt_mux[32+15]),
                            self.prot_rx_dph_pktpend.eq(in_header_pkt_mux[64+27]),
                            self.prot_rx_dph_endp.eq(in_header_pkt_mux[32+8:32+12]),
                            self.prot_rx_dph_seq.eq(in_header_pkt_mux[32+0:32+5]),
                            self.prot_rx_dph_len.eq(in_header_pkt_mux[32+16:32+32]),
                            self.prot_rx_dpp_start.eq(0),
                            self.prot_rx_dpp_done.eq(0),
                        ],
                        LP_TYPE_ITP: [
                            itp_value.eq(in_header_pkt_mux[5:32])
                        ],
                        "default": [
                            self.err_hp_type.eq(1)
                        ],
                    }),

                    # in_header_pkt_queued[in_header_pkt_pick] <= 0;
                    Case(in_header_pkt_pick, {
                        0: in_header_pkt_queued.eq(in_header_pkt_queued & ~0b0001),
                        1: in_header_pkt_queued.eq(in_header_pkt_queued & ~0b0010),
                        2: in_header_pkt_queued.eq(in_header_pkt_queued & ~0b0100),
                        3: in_header_pkt_queued.eq(in_header_pkt_queued & ~0b1000),
                    }),

                    # tx_queue_lcred <= {1'b1, rx_cred_idx_cache[1:0]}
                    tx_queue_lcred.eq(Cat(rx_cred_idx_cache, C(1, 1))),

                    local_rx_cred_count.eq(local_rx_cred_count + 1),
                    rd_hp_state.eq(RD_HP_IDLE),
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Link Command READ FSM (rd_lcmd_state)
            # -------------------------------------------------------------------------------------
            Case(rd_lcmd_state, {
                RD_LCMD_RESET: [
                    rd_lcmd_state.eq(RD_LCMD_IDLE)
                ],
                RD_LCMD_IDLE: [
                    If(in_link_command_act,
                        If((in_link_command[0:11] == in_link_command[16:27]) &
                           (crc_cw1_out == in_link_command[27:32]) &
                           (crc_cw2_out == in_link_command[11:16]),
                            rd_lcmd_state.eq(RD_LCMD_0)
                        )
                    )
                ],
                RD_LCMD_0: [
                    rd_lcmd_state.eq(RD_LCMD_IDLE),
                    rx_lcmd.eq(in_link_command_1[0:11]),
                    rx_lcmd_act.eq(1),
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Parse link commands (runs on rx_lcmd_act asserted in prior cycle)
            # -------------------------------------------------------------------------------------
            If(rx_lcmd_act,
                If((rx_lcmd[3:11] == (LCMD_LGOOD_0 >> 3)) & sent_u0_adv & recv_u0_adv,
                    If(ack_tx_hdr_seq_num != rx_lcmd[0:3],
                        self.err_lgood_order.eq(1),
                        self.ltssm_go_recovery.eq(1),
                    ).Else(
                        ack_tx_hdr_seq_num.eq(ack_tx_hdr_seq_num + 1),
                        pending_hp_timer.eq(0),
                    )
                ).Elif((rx_lcmd[2:11] == (LCMD_LCRD_A >> 2)) & sent_u0_adv & recv_u0_adv,
                    If(remote_rx_cred_count > 3,
                        self.err_lcrd_mismatch.eq(1),
                        self.ltssm_go_recovery.eq(1),
                    ).Elif(tx_cred_idx != rx_lcmd[0:2],
                        self.err_lcrd_mismatch.eq(1),
                        self.ltssm_go_recovery.eq(1),
                    ).Else(
                        tx_cred_idx.eq(tx_cred_idx + 1),
                        remote_rx_cred_count_inc.eq(1),
                        credit_hp_timer.eq(0),
                    )
                ).Elif(rx_lcmd[2:11] == (LCMD_LGO_U1 >> 2),
                    # LGO_Ux
                    self.ltssm_go_u.eq(Cat(rx_lcmd[0:2], C(0, 1))),  # latch target in [1:0]
                    If((local_rx_cred_count < 4) & ~force_linkpm_accept,
                        tx_lcmd.eq(LCMD_LXU),
                    ).Else(
                        tx_lcmd.eq(LCMD_LAU),
                        pm_waiting_for_ack.eq(1),
                    ),
                    tx_lcmd_act.eq(1),
                ).Elif(rx_lcmd == LCMD_LPMA,
                    self.ltssm_go_u.eq(Cat(self.ltssm_go_u[0:2], C(1, 1))),
                    pm_waiting_for_ack.eq(0),
                ).Elif(rx_lcmd == LCMD_LRTY,
                    rx_hdr_seq_ignore.eq(0)
                ).Elif(rx_lcmd == LCMD_LBAD,
                    queue_hp_retry.eq(1),
                    rty_tx_hdr_seq_num.eq(ack_tx_hdr_seq_num),
                    self.err_lbad_recv.eq(1),
                )
            ),

            # -------------------------------------------------------------------------------------
            # Queue FSM (queue_state)
            # -------------------------------------------------------------------------------------
            Case(queue_state, {
                LINK_QUEUE_RESET: [
                    queue_state.eq(LINK_QUEUE_IDLE)
                ],
                LINK_QUEUE_IDLE: [
                    qc.eq(0),

                    If(queue_send_u0_adv & (dc > 1),
                        queue_state.eq(LINK_QUEUE_HDR_SEQ_AD),
                        queue_send_u0_adv.eq(0)
                    ).Elif((send_state == LINK_SEND_IDLE) & ~tx_hp_queue,
                        If(queue_send_u0_portcap & sent_u0_adv,
                            queue_state.eq(LINK_QUEUE_PORTCAP),
                            queue_send_u0_portcap.eq(0),
                        ).Elif((~sent_u0_adv) | (~recv_u0_adv),
                            # do nothing
                        ).Elif(send_port_cfg_resp,
                            queue_state.eq(LINK_QUEUE_PORTCFGRSP),
                            send_port_cfg_resp.eq(0),
                        ).Elif(queue_hp_retry,
                            queue_state.eq(LINK_QUEUE_RTY_HP),
                        ).Elif(self.prot_tx_tp_c,
                            queue_state.eq(LINK_QUEUE_TP_C),
                        ).Elif(self.prot_tx_tp_b,
                            queue_state.eq(LINK_QUEUE_TP_B),
                        ).Elif(self.prot_tx_tp_a,
                            queue_state.eq(LINK_QUEUE_TP_A),
                        ).Elif(self.prot_tx_dph,
                            queue_state.eq(LINK_QUEUE_DP),
                        )
                    )
                ],
                LINK_QUEUE_HDR_SEQ_AD: [
                    If(tx_lcmd_done,
                        qc.eq(qc + 1)
                    ).Else(
                        Case(qc, {
                            0: [
                                tx_lcmd.eq(Cat(rx_hdr_seq_num_dec, C(LCMD_LGOOD_0 >> 3, 8))),  # build {LCMD_LGOOD_0[10:3], seq}
                                tx_lcmd_act.eq(1),
                            ],
                            1: [
                                tx_lcmd.eq(LCMD_LCRD_A),
                                tx_lcmd_act.eq(1),
                                If(local_rx_cred_count == 1, qc.eq(5))
                            ],
                            2: [
                                tx_lcmd.eq(LCMD_LCRD_B),
                                tx_lcmd_act.eq(1),
                                If(local_rx_cred_count == 2, qc.eq(5))
                            ],
                            3: [
                                tx_lcmd.eq(LCMD_LCRD_C),
                                tx_lcmd_act.eq(1),
                                If(local_rx_cred_count == 3, qc.eq(5))
                            ],
                            4: [
                                tx_lcmd.eq(LCMD_LCRD_D),
                                tx_lcmd_act.eq(1),
                            ],
                            5: [
                                sent_u0_adv.eq(1),
                                queue_state.eq(LINK_QUEUE_IDLE),
                            ],
                        })
                    )
                ],
                LINK_QUEUE_PORTCAP: [
                    Case(qc, {
                        0: [
                            tx_hp_word_0.eq(Cat(C(LP_TYPE_LMP, 5),
                                                C(LP_LMP_SUB_PORTCAP, 4),
                                                C(LP_LMP_SPEED_5GBPS, 7),
                                                C(0, 16))),
                            tx_hp_word_1.eq(Cat(C(LP_LMP_NUM_HP_4, 8),
                                                C(0, 8),
                                                C(LP_LMP_DIR_UP, 2),
                                                C(LP_LMP_OTG_INCAPABLE, 1),
                                                C(0, 1),
                                                C(LP_LMP_TIEBREAK, 4),
                                                C(0, 8))),
                            tx_hp_word_2.eq(0),
                            tx_hp_act.eq(1),
                            qc.eq(1),
                        ],
                        1: [
                            sent_u0_portcap.eq(1),
                            queue_state.eq(LINK_QUEUE_IDLE),
                        ],
                    })
                ],
                LINK_QUEUE_PORTCFGRSP: [
                    Case(qc, {
                        0: [
                            tx_hp_word_0.eq(Cat(C(LP_TYPE_LMP, 5),
                                                C(LP_LMP_SUB_PORTCFGRSP, 4),
                                                C(LP_LMP_SPEED_ACCEPT, 7),
                                                C(0, 16))),
                            tx_hp_word_1.eq(0),
                            tx_hp_word_2.eq(0),
                            tx_hp_act.eq(1),
                            qc.eq(1),
                        ],
                        1: [
                            queue_state.eq(LINK_QUEUE_IDLE),
                        ],
                    })
                ],
                LINK_QUEUE_RTY_HP: [
                    tx_hp_retry.eq(1),
                    If(tx_lcmd_done,
                        qc.eq(qc + 1)
                    ).Else(
                        Case(qc, {
                            0: [
                                tx_lcmd.eq(LCMD_LRTY),
                                tx_lcmd_act.eq(1),
                            ],
                            1: [
                                If(tx_hp_done,
                                    queue_state.eq(LINK_QUEUE_IDLE),
                                    queue_hp_retry.eq(0),
                                )
                            ],
                        })
                    )
                ],
                LINK_QUEUE_TP_A: [
                    tx_hp_word_0.eq(Cat(C(LP_TYPE_TP, 5),
                                        C(LP_TP_ROUTE0, 20),
                                        local_dev_addr)),
                    tx_hp_word_1.eq(Cat(self.prot_tx_tp_a_subtype,
                                        C(0, 2),
                                        self.prot_tx_tp_a_retry,
                                        self.prot_tx_tp_a_dir,
                                        self.prot_tx_tp_a_endp,
                                        C(0, 3),
                                        C(0, 1),
                                        self.prot_tx_tp_a_nump,
                                        self.prot_tx_tp_a_seq,
                                        C(0, 6))),
                    tx_hp_word_2.eq(Cat(self.prot_tx_tp_a_stream,
                                        C(0, 8),
                                        C(LP_TP_SSI_NO, 1),
                                        C(LP_TP_WPA_NO, 1),
                                        C(LP_TP_DBI_NO, 1),
                                        C(LP_TP_PPEND_NO, 1),
                                        C(LP_TP_NBI_0, 4))),
                    tx_hp_act.eq(1),
                    self.prot_tx_tp_a_ack.eq(1),
                    queue_state.eq(LINK_QUEUE_IDLE),
                ],
                LINK_QUEUE_TP_B: [
                    tx_hp_word_0.eq(Cat(C(LP_TYPE_TP, 5),
                                        C(LP_TP_ROUTE0, 20),
                                        local_dev_addr)),
                    tx_hp_word_1.eq(Cat(self.prot_tx_tp_b_subtype,
                                        C(0, 2),
                                        self.prot_tx_tp_b_retry,
                                        self.prot_tx_tp_b_dir,
                                        self.prot_tx_tp_b_endp,
                                        C(0, 3),
                                        C(0, 1),
                                        self.prot_tx_tp_b_nump,
                                        self.prot_tx_tp_b_seq,
                                        C(0, 6))),
                    tx_hp_word_2.eq(Cat(self.prot_tx_tp_b_stream,
                                        C(0, 8),
                                        C(LP_TP_SSI_NO, 1),
                                        C(LP_TP_WPA_NO, 1),
                                        C(LP_TP_DBI_NO, 1),
                                        C(LP_TP_PPEND_NO, 1),
                                        C(LP_TP_NBI_0, 4))),
                    tx_hp_act.eq(1),
                    local_dev_addr.eq(self.dev_addr),
                    self.prot_tx_tp_b_ack.eq(1),
                    queue_state.eq(LINK_QUEUE_IDLE),
                ],
                LINK_QUEUE_TP_C: [
                    tx_hp_word_0.eq(Cat(C(LP_TYPE_TP, 5),
                                        C(LP_TP_ROUTE0, 20),
                                        local_dev_addr)),
                    tx_hp_word_1.eq(Cat(self.prot_tx_tp_c_subtype,
                                        C(0, 2),
                                        self.prot_tx_tp_c_retry,
                                        self.prot_tx_tp_c_dir,
                                        self.prot_tx_tp_c_endp,
                                        C(0, 3),
                                        C(0, 1),
                                        self.prot_tx_tp_c_nump,
                                        self.prot_tx_tp_c_seq,
                                        C(0, 6))),
                    tx_hp_word_2.eq(Cat(self.prot_tx_tp_c_stream,
                                        C(0, 8),
                                        C(LP_TP_SSI_NO, 1),
                                        C(LP_TP_WPA_NO, 1),
                                        C(LP_TP_DBI_NO, 1),
                                        C(LP_TP_PPEND_NO, 1),
                                        C(LP_TP_NBI_0, 4))),
                    tx_hp_act.eq(1),
                    local_dev_addr.eq(self.dev_addr),
                    self.prot_tx_tp_c_ack.eq(1),
                    queue_state.eq(LINK_QUEUE_IDLE),
                ],
                LINK_QUEUE_DP: [
                    Case(qc, {
                        0: [
                            If(send_state == LINK_SEND_IDLE,
                                tx_hp_word_0.eq(Cat(C(LP_TYPE_DP, 5),
                                                    C(LP_TP_ROUTE0, 20),
                                                    local_dev_addr)),
                                tx_hp_word_1.eq(Cat(self.prot_tx_dph_seq,
                                                    C(0, 1),
                                                    self.prot_tx_dph_eob,
                                                    self.prot_tx_dph_dir,
                                                    self.prot_tx_dph_endp,
                                                    C(0, 3),
                                                    C(0, 1),
                                                    self.prot_tx_dph_len)),
                                tx_hp_word_2.eq(0),
                                tx_hp_act.eq(1),
                                tx_hp_dph.eq(1),
                                out_dpp_length.eq(self.prot_tx_dph_len),
                                self.prot_tx_dpp_ack.eq(1),
                                self.prot_tx_dpp_done.eq(0),
                                qc.eq(1),
                            )
                        ],
                        1: [
                            queue_state.eq(LINK_QUEUE_IDLE)
                        ],
                    })
                ],
            }),

            # -------------------------------------------------------------------------------------
            # SEND FSM (send_state)
            # -------------------------------------------------------------------------------------
            Case(send_state, {
                LINK_SEND_RESET: [
                    send_state.eq(LINK_SEND_IDLE)
                ],
                LINK_SEND_IDLE: [
                    tx_queue_open.eq(1),

                    If(sent_u0_adv == 0,
                        tx_queue_open.eq(0)
                    ),

                    If((self.ltssm_state != LT_U0) | (pm_waiting_for_ack & (tx_lcmd != LCMD_LAU)),
                        tx_queue_open.eq(0)
                    ).Elif(tx_lcmd_queue,
                        tx_lcmd_queue.eq(0),
                        tx_queue_open.eq(0),
                        send_state.eq(LINK_SEND_CMDW_0)
                    ).Elif(tx_queue_lgood[3],
                        tx_queue_lgood.eq(0),
                        tx_queue_open.eq(0),
                        tx_lcmd.eq(Cat(tx_queue_lgood[0:3], C(LCMD_LGOOD_0 >> 3, 8))),
                        send_state.eq(LINK_SEND_CMDW_0)
                    ).Elif(tx_queue_lcred[2],
                        tx_queue_lcred.eq(0),
                        tx_queue_open.eq(0),
                        tx_lcmd.eq(Cat(tx_queue_lcred[0:2], C(LCMD_LCRD_A >> 2, 9))),
                        send_state.eq(LINK_SEND_CMDW_0)
                    ).Elif(tx_hp_queue & (remote_rx_cred_count > 0),
                        tx_queue_open.eq(0),
                        tx_hp_queue.eq(0),
                        send_state.eq(LINK_SEND_HP_0)
                    ).Elif(tx_hp_retry & (remote_rx_cred_count > 0) & (qc > 0),
                        tx_queue_open.eq(0),
                        send_state.eq(LINK_SEND_HP_RTY)
                    ).Elif(tx_queue_lup,
                        tx_queue_lup.eq(0),
                        tx_queue_open.eq(0),
                        tx_lcmd.eq(LCMD_LUP),
                        send_state.eq(LINK_SEND_CMDW_0)
                    )
                ],
                LINK_SEND_CMDW_0: [
                    If(wr_lcmd_state != WR_LCMD_0,
                        tx_lcmd_latch.eq(tx_lcmd),
                        wr_lcmd_state.eq(WR_LCMD_0),
                        u0l_timeout.eq(0),
                        send_state.eq(LINK_SEND_IDLE),
                    )
                ],
                LINK_SEND_HP_RTY: [
                    out_header_pkt_pick.eq(tx_cred_idx),
                    send_state.eq(LINK_SEND_HP_1),
                ],
                LINK_SEND_HP_0: [
                    Case(tx_cred_idx, {
                        0: out_header_pkt_a.eq(Cat(tx_hp_word_2, tx_hp_word_1, tx_hp_word_0)),
                        1: out_header_pkt_b.eq(Cat(tx_hp_word_2, tx_hp_word_1, tx_hp_word_0)),
                        2: out_header_pkt_c.eq(Cat(tx_hp_word_2, tx_hp_word_1, tx_hp_word_0)),
                        3: out_header_pkt_d.eq(Cat(tx_hp_word_2, tx_hp_word_1, tx_hp_word_0)),
                    }),
                    out_header_pkt_pick.eq(tx_cred_idx),
                    send_state.eq(LINK_SEND_HP_1),
                ],
                LINK_SEND_HP_1: [
                    If(wr_hp_state == WR_HP_IDLE,
                        wr_hp_state.eq(WR_HP_0),
                        send_state.eq(LINK_SEND_HP_2),
                    )
                ],
                LINK_SEND_HP_2: [
                    If(wr_hp_state == WR_HP_IDLE,
                        tx_hp_retry.eq(0),
                        send_state.eq(LINK_SEND_IDLE)
                    )
                ],
            }),
            If(self.ltssm_state != LT_U0,
                send_state.eq(LINK_SEND_RESET)
            ),

            # -------------------------------------------------------------------------------------
            # WR_LCMD FSM
            # -------------------------------------------------------------------------------------
            Case(wr_lcmd_state, {
                WR_LCMD_RESET: [
                    wr_lcmd_state.eq(WR_LCMD_IDLE)
                ],
                WR_LCMD_IDLE: [
                    # dummy
                ],
                WR_LCMD_0: [
                    out_data.eq(OS_CMDW),
                    out_datak.eq(0b1111),
                    out_active.eq(1),
                    crc_lcmd_in.eq(tx_lcmd_latch),
                    tx_lcmd_out.eq(tx_lcmd_latch),
                    wr_lcmd_state.eq(WR_LCMD_1),
                ],
                WR_LCMD_1: [
                    # {{2{tx_lcmd_out[7:0], crc_lcmd_out[4:0], tx_lcmd_out[10:8]}}, 4'b00}
                    out_data.eq(Cat(
                        C(0, 2),
                        tx_lcmd_out[8:11],
                        crc_lcmd_out,
                        tx_lcmd_out[0:8],
                        tx_lcmd_out[8:11],
                        crc_lcmd_out,
                        tx_lcmd_out[0:8],
                    )),
                    out_datak.eq(0),
                    out_active.eq(1),

                    If(send_state == LINK_SEND_CMDW_0,
                        wr_lcmd_state.eq(WR_LCMD_0)
                    ).Else(
                        wr_lcmd_state.eq(WR_LCMD_IDLE)
                    ),
                    tx_lcmd_done.eq(1),
                ],
            }),

            # -------------------------------------------------------------------------------------
            # WR_HP FSM (includes "trickery" output shift)
            # -------------------------------------------------------------------------------------
            If(wr_hp_state != WR_HP_IDLE,
                out_data.eq(out_data_1),
                out_data_1.eq(out_data_2),
                out_datak.eq(out_datak_1),
                out_datak_1.eq(out_datak_2),
                out_active.eq(out_active_1),
                out_active_1.eq(out_active_2),
            ),

            Case(wr_hp_state, {
                WR_HP_RESET: [
                    wr_hp_state.eq(WR_HP_IDLE)
                ],
                WR_HP_IDLE: [
                    # dummy
                ],
                WR_HP_0: [
                    crc_hptx_rst.eq(1),
                    If(~tx_hp_retry,
                        remote_rx_cred_count_dec.eq(1)
                    ),
                    out_data_2.eq(OS_HPST),
                    out_datak_2.eq(0b1111),
                    out_active_2.eq(1),
                    sc.eq(0),
                    wr_hp_state.eq(WR_HP_1),
                ],
                WR_HP_1: [
                    Case(sc, {
                        0: out_data_2.eq(_swap32(out_header_pkt_mux[64:96])),
                        1: out_data_2.eq(_swap32(out_header_pkt_mux[32:64])),
                        2: out_data_2.eq(_swap32(out_header_pkt_mux[0:32])),
                    }),
                    If(sc == 0,
                        last_hdr_seq_num.eq(tx_hdr_seq_num),
                        If((tx_hdr_seq_num == last_hdr_seq_num) & ~tx_hp_retry,
                            self.err_stuck_hpseq.eq(1)
                        )
                    ),
                    out_header_cw.eq(Cat(tx_hdr_seq_num, C(0, 6), tx_hp_retry, C(0, 1))),
                    out_active_2.eq(1),
                    If(sc == 2,
                        wr_hp_state.eq(WR_HP_2)
                    ),
                ],
                WR_HP_2: [
                    wr_hp_state.eq(WR_HP_3),
                    If(tx_hp_dph,
                        write_dpp_state.eq(WRITE_DPP_0)
                    ),
                ],
                WR_HP_3: [
                    # swap32({crc_cw4_out, out_header_cw, crc_hptx_out})
                    out_data_1.eq(_swap32(Cat(crc_hptx_out, out_header_cw, crc_cw4_out))),
                    out_active_1.eq(1),
                    wr_hp_state.eq(WR_HP_4),
                ],
                WR_HP_4: [
                    If(~tx_hp_retry,
                        tx_hdr_seq_num.eq(tx_hdr_seq_num + 1)
                    ),
                    If(out_header_first_since_entry,
                        out_header_first_since_entry.eq(0)
                    ),
                    If(tx_hp_dph,
                        wr_hp_state.eq(WR_HP_5)
                    ).Else(
                        tx_hp_done.eq(1),
                        wr_hp_state.eq(WR_HP_IDLE)
                    )
                ],
                WR_HP_5: [
                    If(write_dpp_state == WRITE_DPP_IDLE,
                        self.buf_out_arm.eq(1),
                        tx_hp_done.eq(1),
                        self.prot_tx_dpp_done.eq(1),
                        wr_hp_state.eq(WR_HP_IDLE),
                    )
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Write DPP FSM (TX)
            # -------------------------------------------------------------------------------------
            Case(write_dpp_state, {
                WRITE_DPP_RESET: [
                    write_dpp_state.eq(WRITE_DPP_IDLE)
                ],
                WRITE_DPP_IDLE: [
                    self.buf_out_addr.eq(0)
                ],
                WRITE_DPP_0: [
                    self.buf_out_addr.eq(self.buf_out_addr + 1),
                    write_dpp_state.eq(WRITE_DPP_1),
                ],
                WRITE_DPP_1: [
                    self.buf_out_addr.eq(self.buf_out_addr + 1),
                    out_data_1.eq(OS_DPP),
                    out_datak_1.eq(0b1111),
                    out_active_1.eq(1),
                    out_dpp_length_remain.eq(out_dpp_length),
                    out_dpp_length_remain_1.eq(out_dpp_length),
                    crc_dpptx_rst.eq(1),
                    write_dpp_state.eq(WRITE_DPP_2),
                ],
                WRITE_DPP_2: [
                    Case(out_dpp_length_remain_1, {
                        3: [
                            out_data.eq(Cat(crc_dpptx_out[24:32], out_data_1[8:32])),
                            out_datak.eq(0),
                            write_dpp_state.eq(WRITE_DPP_6),
                        ],
                        2: [
                            out_data.eq(Cat(crc_dpptx_out[16:32], out_data_1[16:32])),
                            out_datak.eq(0),
                            write_dpp_state.eq(WRITE_DPP_7),
                        ],
                        1: [
                            out_data.eq(Cat(crc_dpptx_out[8:32], out_data_1[24:32])),
                            out_datak.eq(0),
                            write_dpp_state.eq(WRITE_DPP_8),
                        ],
                        0: [
                            out_data.eq(out_data_1),
                            out_datak.eq(0),
                            write_dpp_state.eq(WRITE_DPP_5),
                        ],
                        "default": [
                            out_data_1.eq(self.buf_out_q),
                            out_datak_1.eq(0),
                        ]
                    }),

                    If((out_dpp_length_remain_1 == 4),
                        If(out_dpp_length == 4,
                            write_dpp_state.eq(WRITE_DPP_4)
                        ).Else(
                            out_data_1.eq(self.buf_out_q),
                            out_datak_1.eq(0),
                            write_dpp_state.eq(WRITE_DPP_9),
                        )
                    ),

                    If(out_dpp_length_remain > 4,
                        out_dpp_length_remain.eq(out_dpp_length_remain - 4)
                    ).Else(
                        out_dpp_length_remain.eq(0)
                    ),
                    out_dpp_length_remain_1.eq(out_dpp_length_remain),

                    self.buf_out_addr.eq(self.buf_out_addr + 1),
                    crc_dpptx_out_1.eq(crc_dpptx_out),
                    crc_dpptx_in.eq(_swap32(self.buf_out_q)),
                    out_active_1.eq(1),
                ],
                WRITE_DPP_3: [
                    tx_hp_dph.eq(0),
                    write_dpp_state.eq(WRITE_DPP_IDLE)
                ],
                WRITE_DPP_4: [
                    write_dpp_state.eq(WRITE_DPP_9)
                ],
                WRITE_DPP_5: [
                    out_data.eq(crc_dpptx_out_1),
                    out_datak.eq(0),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_10)
                ],
                WRITE_DPP_6: [
                    out_data.eq(Cat(C(0xFD, 8), crc_dpptx_out_1[0:24])),
                    out_datak.eq(0b0001),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_11)
                ],
                WRITE_DPP_7: [
                    out_data.eq(Cat(C(0xFDFD, 16), crc_dpptx_out_1[0:16])),
                    out_datak.eq(0b0011),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_12)
                ],
                WRITE_DPP_8: [
                    out_data.eq(Cat(C(0xFDFDFD, 24), crc_dpptx_out_1[0:8])),
                    out_datak.eq(0b0111),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_13)
                ],
                WRITE_DPP_9: [
                    out_data.eq(crc_dpptx_out),
                    out_datak.eq(0),
                    out_active.eq(1),

                    out_data_1.eq(OS_EOP),
                    out_datak_1.eq(0b1111),
                    out_active_1.eq(1),
                    write_dpp_state.eq(WRITE_DPP_3)
                ],
                WRITE_DPP_10: [
                    out_data.eq(OS_EOP),
                    out_datak.eq(0b1111),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_3)
                ],
                WRITE_DPP_11: [
                    out_data.eq(0xFDFDF700),
                    out_datak.eq(0b1110),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_3)
                ],
                WRITE_DPP_12: [
                    out_data.eq(0xFDF70000),
                    out_datak.eq(0b1100),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_3)
                ],
                WRITE_DPP_13: [
                    out_data.eq(0xF7000000),
                    out_datak.eq(0b1000),
                    out_active.eq(1),
                    write_dpp_state.eq(WRITE_DPP_3)
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Reset block (~reset_n)
            # -------------------------------------------------------------------------------------
            If(~self.reset_n,
                send_state.eq(LINK_SEND_RESET),
                recv_state.eq(LINK_RECV_RESET),
                expect_state.eq(LINK_EXPECT_RESET),
                queue_state.eq(LINK_QUEUE_RESET),
                rd_lcmd_state.eq(RD_LCMD_RESET),
                rd_hp_state.eq(RD_HP_RESET),
                wr_lcmd_state.eq(WR_LCMD_RESET),
                wr_hp_state.eq(WR_HP_RESET),
                check_hp_state.eq(CHECK_HP_RESET),

                self.err_lcmd_undefined.eq(0),
                self.err_lcrd_mismatch.eq(0),
                self.err_lgood_order.eq(0),
                self.err_lgood_missed.eq(0),
                self.err_pending_hp.eq(0),
                self.err_credit_hp.eq(0),
                self.err_hp_crc.eq(0),
                self.err_hp_seq.eq(0),
                self.err_hp_type.eq(0),
                self.err_dpp_len_mismatch.eq(0),
                self.err_lbad.eq(0),
                self.err_lbad_recv.eq(0),
                self.err_stuck_hpseq.eq(0),

                link_error_count.eq(0),
                self.ltssm_go_recovery.eq(0),
                self.ltssm_go_u.eq(0),
                queue_send_u0_adv.eq(0),

                local_dev_addr.eq(0),
                tx_hp_dph.eq(0),
                tx_hp_retry.eq(0),

                self.prot_rx_dpp_done.eq(0),
                self.prot_tx_dpp_done.eq(0),
            ),
        ]
