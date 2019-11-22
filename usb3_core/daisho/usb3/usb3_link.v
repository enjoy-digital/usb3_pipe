
//
// usb 3.0 link level
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb3_link (

input	wire			slow_clk,
input	wire			local_clk,
input	wire			reset_n,

input	wire	[4:0]	ltssm_state,
input	wire			ltssm_hot_reset,
output	reg				ltssm_go_disabled,
output	reg		[2:0]	ltssm_go_u,
output	reg				ltssm_go_recovery,

// pipe interface
input	wire	[31:0]	in_data,
input	wire	[3:0]	in_datak,
input	wire			in_active,

output	reg		[31:0]	outp_data,
output	reg		[3:0]	outp_datak,
output	reg				outp_active,
input	wire			out_stall,

// protocol interface
input	wire	[1:0]	endp_mode_rx,
input	wire	[1:0]	endp_mode_tx,
input	wire	[6:0]	dev_addr,

output	reg				prot_rx_tp,
output	reg				prot_rx_tp_hosterr,
output	reg				prot_rx_tp_retry,
output	reg				prot_rx_tp_pktpend,
output	reg		[3:0]	prot_rx_tp_subtype,
output	reg		[3:0]	prot_rx_tp_endp,
output	reg		[4:0]	prot_rx_tp_nump,
output	reg		[4:0]	prot_rx_tp_seq,
output	reg		[15:0]	prot_rx_tp_stream,

output	reg				prot_rx_dph,
output	reg				prot_rx_dph_eob,
output	reg				prot_rx_dph_setup,
output	reg				prot_rx_dph_pktpend,
output	reg		[3:0]	prot_rx_dph_endp,
output	reg		[4:0]	prot_rx_dph_seq,
output	reg		[15:0]	prot_rx_dph_len,
output	reg				prot_rx_dpp_start,
output	reg				prot_rx_dpp_done,
output	reg				prot_rx_dpp_crcgood,

input	wire			prot_tx_tp_a,
input	wire			prot_tx_tp_a_retry,
input	wire			prot_tx_tp_a_dir,
input	wire	[3:0]	prot_tx_tp_a_subtype,
input	wire	[3:0]	prot_tx_tp_a_endp,
input	wire	[4:0]	prot_tx_tp_a_nump,
input	wire	[4:0]	prot_tx_tp_a_seq,
input	wire	[15:0]	prot_tx_tp_a_stream,
output	reg				prot_tx_tp_a_ack,

input	wire			prot_tx_tp_b,
input	wire			prot_tx_tp_b_retry,
input	wire			prot_tx_tp_b_dir,
input	wire	[3:0]	prot_tx_tp_b_subtype,
input	wire	[3:0]	prot_tx_tp_b_endp,
input	wire	[4:0]	prot_tx_tp_b_nump,
input	wire	[4:0]	prot_tx_tp_b_seq,
input	wire	[15:0]	prot_tx_tp_b_stream,
output	reg				prot_tx_tp_b_ack,

input	wire			prot_tx_tp_c,
input	wire			prot_tx_tp_c_retry,
input	wire			prot_tx_tp_c_dir,
input	wire	[3:0]	prot_tx_tp_c_subtype,
input	wire	[3:0]	prot_tx_tp_c_endp,
input	wire	[4:0]	prot_tx_tp_c_nump,
input	wire	[4:0]	prot_tx_tp_c_seq,
input	wire	[15:0]	prot_tx_tp_c_stream,
output	reg				prot_tx_tp_c_ack,

input	wire			prot_tx_dph,
input	wire			prot_tx_dph_eob,
input	wire			prot_tx_dph_dir,
input	wire	[3:0]	prot_tx_dph_endp,
input	wire	[4:0]	prot_tx_dph_seq,
input	wire	[15:0]	prot_tx_dph_len,
output	reg				prot_tx_dpp_ack,
output	reg				prot_tx_dpp_done,



output	reg		[8:0]	buf_in_addr,
output	reg		[31:0]	buf_in_data,
output	reg				buf_in_wren,
input	wire			buf_in_ready,
output	reg				buf_in_commit,
output	reg		[10:0]	buf_in_commit_len,
input	wire			buf_in_commit_ack,

output	reg		[9:0]	buf_out_addr,
input	wire	[31:0]	buf_out_q,
input	wire	[10:0]	buf_out_len,
input	wire			buf_out_hasdata,
output	reg				buf_out_arm,
input	wire			buf_out_arm_ack,

// error outputs
output	reg				err_lbad,
output	reg				err_lbad_recv,
output	reg				err_stuck_hpseq,
output	reg				err_lcmd_undefined,
output	reg				err_lcrd_mismatch,
output	reg				err_lgood_order,
output	reg				err_lgood_missed,
output	reg				err_pending_hp,
output	reg				err_credit_hp,
output	reg				err_hp_crc,
output	reg				err_hp_seq,
output	reg				err_hp_type,
output	reg				err_dpp_len_mismatch

);

`include "usb3_const.vh"	
	
	reg		[5:0]	send_state;
parameter	[5:0]	LINK_SEND_RESET		= 'd0,
					LINK_SEND_IDLE		= 'd2,
					LINK_SEND_0			= 'd4,
					LINK_SEND_1			= 'd6,
					LINK_SEND_2			= 'd8,
					LINK_SEND_3			= 'd10,
					LINK_SEND_4			= 'd12,
					LINK_SEND_CMDW_0	= 'd14,
					LINK_SEND_CMDW_1	= 'd16,
					LINK_SEND_HP_RTY	= 'd17,
					LINK_SEND_HP_0		= 'd18,
					LINK_SEND_HP_1		= 'd20,
					LINK_SEND_HP_2		= 'd22;
					

	reg		[5:0]	recv_state;
parameter	[5:0]	LINK_RECV_RESET		= 'd0,
					LINK_RECV_IDLE		= 'd2,
					LINK_RECV_HP_0		= 'd4,
					LINK_RECV_HP_1		= 'd5,
					LINK_RECV_DP_0		= 'd7,
					LINK_RECV_DP_1		= 'd8,
					LINK_RECV_DP_2		= 'd9,
					LINK_RECV_DP_3		= 'd10,
					LINK_RECV_CMDW_0		= 'd15,
					LINK_RECV_CMDW_1		= 'd16,
					LINK_RECV_CMDW_2		= 'd17,
					LINK_RECV_CMDW_3		= 'd18,
					LINK_RECV_CMDW_4		= 'd19;

	reg		[5:0]	read_dpp_state;
parameter	[5:0]	READ_DPP_RESET		= 'd0,
					READ_DPP_IDLE		= 'd1,
					READ_DPP_0			= 'd2,
					READ_DPP_1			= 'd3,
					READ_DPP_2			= 'd4,
					READ_DPP_3			= 'd5,
					READ_DPP_4			= 'd6,
					READ_DPP_5			= 'd7,
					READ_DPP_6			= 'd8;
					
	reg		[5:0]	write_dpp_state;
parameter	[5:0]	WRITE_DPP_RESET		= 'd0,
					WRITE_DPP_IDLE		= 'd1,
					WRITE_DPP_0			= 'd2,
					WRITE_DPP_1			= 'd3,
					WRITE_DPP_2			= 'd4,
					WRITE_DPP_3			= 'd5,
					WRITE_DPP_4			= 'd6,
					WRITE_DPP_5			= 'd7,
					WRITE_DPP_6			= 'd8,
					WRITE_DPP_7			= 'd9,
					WRITE_DPP_8			= 'd10,
					WRITE_DPP_9			= 'd11,
					WRITE_DPP_10		= 'd12,
					WRITE_DPP_11		= 'd13,
					WRITE_DPP_12		= 'd14,
					WRITE_DPP_13		= 'd15;
					
	reg		[5:0]	check_hp_state;
parameter	[5:0]	CHECK_HP_RESET		= 'd0,
					CHECK_HP_IDLE		= 'd1,
					CHECK_HP_0			= 'd2,
					CHECK_HP_1			= 'd3,
					CHECK_HP_2			= 'd4,
					CHECK_HP_3			= 'd5;

	reg		[5:0]	expect_state;
parameter	[5:0]	LINK_EXPECT_RESET		= 'd0,
					LINK_EXPECT_IDLE		= 'd2,
					LINK_EXPECT_HDR_SEQ_AD	= 'd4,
					LINK_EXPECT_HP_1		= 'd6,
					LINK_EXPECT_HP_2		= 'd8,
					LINK_EXPECT_HP_3		= 'd10;
					
	reg		[5:0]	queue_state;
parameter	[5:0]	LINK_QUEUE_RESET		= 'd0,
					LINK_QUEUE_IDLE			= 'd2,
					LINK_QUEUE_HDR_SEQ_AD	= 'd4,
					LINK_QUEUE_PORTCAP		= 'd6,
					LINK_QUEUE_PORTCFGRSP	= 'd8,
					LINK_QUEUE_RTY_HP		= 'd9,
					LINK_QUEUE_TP_A			= 'd10,
					LINK_QUEUE_TP_B			= 'd11,
					LINK_QUEUE_TP_C			= 'd13,
					LINK_QUEUE_DP			= 'd12;
						
	reg		[4:0]	rd_hp_state;
parameter	[4:0]	RD_HP_RESET			= 'd0,
					RD_HP_IDLE			= 'd1,
					RD_HP_LMP_0			= 'd4,
					RD_HP_LMP_1			= 'd5,
					RD_HP_TP_0			= 'd10,
					RD_HP_TP_1			= 'd11,
					RD_HP_TP_2			= 'd12,
					RD_HP_DP_0			= 'd16,
					RD_HP_DP_1			= 'd17,
					RD_HP_DP_2			= 'd18,
					RD_HP_0				= 'd20,
					RD_HP_1				= 'd21,
					RD_HP_2				= 'd22,
					RD_HP_3				= 'd22;
					
	reg		[3:0]	rd_lcmd_state;
parameter	[3:0]	RD_LCMD_RESET		= 'd0,
					RD_LCMD_IDLE		= 'd1,
					RD_LCMD_0			= 'd2;
					
	reg		[3:0]	wr_lcmd_state;
parameter	[3:0]	WR_LCMD_RESET		= 'd0,
					WR_LCMD_IDLE		= 'd1,
					WR_LCMD_0			= 'd2,
					WR_LCMD_1			= 'd3;
					
	reg		[3:0]	wr_hp_state;
parameter	[3:0]	WR_HP_RESET		= 'd0,
					WR_HP_IDLE		= 'd1,
					WR_HP_0			= 'd2,
					WR_HP_1			= 'd3,
					WR_HP_2			= 'd4,
					WR_HP_3			= 'd5,
					WR_HP_4			= 'd6,
					WR_HP_5			= 'd7,
					WR_HP_6			= 'd8,
					WR_HP_7			= 'd9;
	
	reg		[4:0]	ltssm_last;
	reg		[4:0]	ltssm_stored /* synthesis noprune */;
	reg				ltssm_changed /* synthesis noprune */;
	reg				ltssm_go_recovery_1;
	reg		[2:0]	ltssm_go_u_1;
	
	reg		[24:0]	dc;
	
	reg		[24:0]	u0l_timeout;
	reg		[24:0]	u0_recovery_timeout;
	reg		[24:0]	u2_timeout;
	reg		[24:0]	pending_hp_timer;
	reg		[24:0]	credit_hp_timer;
	reg		[7:0]	link_error_count;
	
	reg				force_linkpm_accept;
	reg		[24:0]	pm_lc_timer;									// not used unless this port initiates power modes (LGO_Ux)
	reg		[24:0]	pm_entry_timer;		// new
	reg		[24:0]	ux_exit_timer;		// new
	reg				pm_waiting_for_ack;
	
	reg		[24:0]	port_config_timeout;
	reg		[24:0]	T_PORT_U2_TIMEOUT;
	
	reg		[2:0]	tx_hdr_seq_num /* synthesis noprune */;			// Header Sequence Number (0-7)
	reg		[2:0]	last_hdr_seq_num /* synthesis noprune */;
	reg		[2:0]	ack_tx_hdr_seq_num /* synthesis noprune */;		// ACK Header Seq Num
	reg		[2:0]	rty_tx_hdr_seq_num /* synthesis noprune */;		// Retry Header Seq Num
	reg		[2:0]	rx_hdr_seq_num /* synthesis noprune */;			// RX Header Seq Num
	wire	[2:0]	rx_hdr_seq_num_dec = rx_hdr_seq_num - 3'h1;
	reg				rx_hdr_seq_ignore;								// Ignoring HP between LBAD/LRTY
	reg		[2:0]	local_rx_cred_count /* synthesis noprune */;	// Local RX Header Credit Count (0-4)
	reg		[2:0]	remote_rx_cred_count /* synthesis noprune */;	// Remote RX Header Credit
	reg				remote_rx_cred_count_inc;
	reg				remote_rx_cred_count_dec;
	reg		[1:0]	tx_cred_idx /* synthesis noprune */;	
	reg		[1:0]	rx_cred_idx /* synthesis preserve */;
	reg		[1:0]	rx_cred_idx_cache;
	reg		[26:0]	itp_value;
	wire	[13:0]	bus_interval_count = itp_value[13:0];
	
	reg		[3:0]	in_header_pkt_queued;
	reg		[1:0]	in_header_pkt_pick;
	wire	[95:0]	in_header_pkt_mux = (	in_header_pkt_pick == 3 ? in_header_pkt_a :
											in_header_pkt_pick == 2 ? in_header_pkt_b :
											in_header_pkt_pick == 1 ? in_header_pkt_c : 
											in_header_pkt_d );
	reg		[95:0]	in_header_pkt_a;
	reg		[95:0]	in_header_pkt_b;
	reg		[95:0]	in_header_pkt_c;
	reg		[95:0]	in_header_pkt_d;
	reg		[1:0]	out_header_pkt_pick;
	wire	[95:0]	out_header_pkt_mux = (	out_header_pkt_pick == 0 ? out_header_pkt_a :
											out_header_pkt_pick == 1 ? out_header_pkt_b :
											out_header_pkt_pick == 2 ? out_header_pkt_c : 
											out_header_pkt_d );
	reg		[95:0]	out_header_pkt_a;
	reg		[95:0]	out_header_pkt_b;
	reg		[95:0]	out_header_pkt_c;
	reg		[95:0]	out_header_pkt_d;
	
	/*
	output	reg		[31:0]	out_data,
	output	reg		[3:0]	out_datak,
	output	reg				out_active,
	input	wire			out_stall,
	*/
	reg		[31:0]	out_data;
	reg		[3:0]	out_datak;
	reg				out_active;
	
	reg		[31:0]	out_data_1, out_data_2;
	reg		[3:0]	out_datak_1, out_datak_2;
	reg				out_active_1, out_active_2;
	reg		[10:0]	out_header_cw;
	
	reg		[15:0]	in_header_cw_a;
	reg		[15:0]	in_header_cw_b;
	reg		[15:0]	in_header_cw_c;
	reg		[15:0]	in_header_cw_d;
	reg		[15:0]	in_header_cw;
	wire	[2:0]	in_header_seq = in_header_cw[2:0];
	reg		[15:0]	in_header_crc;
	reg		[1:0]	in_header_err_count;
	
	reg				out_header_first_since_entry;
	
	reg		[31:0]	in_link_command /* synthesis noprune */;
	reg		[31:0]	in_link_command_1 /* synthesis noprune */;
	reg				in_link_command_act /* synthesis noprune */;

	wire	[31:0]	in_data_swap16 = {in_data[23:16], in_data[31:24], in_data[7:0], in_data[15:8]};
	wire	[31:0]	in_data_swap32 = {in_data[7:0], in_data[15:8], in_data[23:16], in_data[31:24]};
	reg		[31:0]	in_data_1, in_data_2, in_data_3;
	reg		[31:0]	in_data_4, in_data_5, in_data_6;
	reg		[3:0]	in_datak_1, in_datak_2, in_datak_3;
	reg		[3:0]	in_datak_4, in_datak_5, in_datak_6;
	reg				in_active_1, in_active_2, in_active_3;
	reg				in_active_4, in_active_5, in_active_6;
	
	reg		[15:0]	in_dpp_length /* synthesis noprune */;
	reg		[15:0]	in_dpp_length_expect /* synthesis noprune */;
	reg				in_dpp_wasready;
	reg		[31:0]	in_dpp_crc32 /* synthesis noprune */;
	
	reg		[15:0]	out_dpp_length /* synthesis noprune */;
	reg		[15:0]	out_dpp_length_remain /* synthesis noprune */;
	reg		[15:0]	out_dpp_length_remain_1 /* synthesis noprune */;

	reg		[95:0]	out_dpp_end;
	reg		[11:0]	out_dpp_endk;
	
	reg				buf_in_ready_1;
	
	reg		[10:0]	rx_lcmd  /* synthesis noprune */;
	reg				rx_lcmd_act  /* synthesis noprune */;
	
	reg		[10:0]	tx_lcmd  /* synthesis noprune */;
	reg				tx_lcmd_act  /* synthesis noprune */;
	reg				tx_lcmd_act_1;
	reg				tx_lcmd_queue;
	wire			tx_lcmd_do = (tx_lcmd_act & ~tx_lcmd_act_1) | tx_lcmd_queue;
	reg		[10:0]	tx_lcmd_latch;
	reg		[10:0]	tx_lcmd_out;
	reg				tx_lcmd_done  /* synthesis noprune */;
	
	reg		[6:0]	local_dev_addr;
	
	reg		[31:0]	tx_hp_word_0;
	reg		[31:0]	tx_hp_word_1;
	reg		[31:0]	tx_hp_word_2;	

	reg				tx_hp_act  /* synthesis noprune */;
	reg				tx_hp_act_1;
	reg				tx_hp_queue;
	wire			tx_hp_do = (tx_hp_act & ~tx_hp_act_1) | tx_hp_queue;
	reg		[10:0]	tx_hp_latch;
	reg				tx_hp_done  /* synthesis noprune */;
	reg				tx_hp_dph;
	reg				tx_hp_retry;
	reg				queue_hp_retry;
	
	reg				tx_queue_open;
	reg				tx_queue_lup;
	reg		[2:0]	tx_queue_lcred;	// {strobe, CRD_A-CRD_D[1:0]}
	reg		[3:0]	tx_queue_lgood;	// {strobe, GOOD_0-GOOD_3[2:0]}
	
	reg				send_port_cfg_resp;
	reg		[1:0]	recv_port_cmdcfg;
	
	reg		[9:0]	qc;		// queue counter
	reg				queue_send_u0_adv;
	reg				sent_u0_adv;
	reg				queue_send_u0_portcap;
	reg				sent_u0_portcap;
	reg				recv_u0_adv;
	
	reg		[9:0]	rc;		// receive counter
	reg		[9:0]	sc;		// send counter
	

always @(posedge local_clk) begin

	ltssm_last <= ltssm_state;
	ltssm_changed <= 0;
	ltssm_go_recovery_1 <= ltssm_go_recovery;
	ltssm_go_u_1 <= ltssm_go_u_1;
	
	outp_data <= out_data;
	outp_datak <= out_datak;
	outp_active <= out_active;
	
	out_data <= 32'h0;
	out_data_1 <= 32'h0;
	out_data_2 <= 32'h0;
	out_datak <= 4'b0000;
	out_datak_1 <= 4'b0000;
	out_datak_2 <= 4'b0000;
	out_active <= 1'b0;
	out_active_1 <= 1'b0;
	out_active_2 <= 1'b0;
	
	{in_data_6, in_data_5,  in_data_4,  in_data_3,  in_data_2,  in_data_1}  <= 
		{in_data_5, in_data_4,  in_data_3,  in_data_2,  in_data_1,  in_data};
	{in_datak_6, in_datak_5, in_datak_4, in_datak_3, in_datak_2, in_datak_1} <= 
		{in_datak_5, in_datak_4, in_datak_3, in_datak_2, in_datak_1, in_datak};
	{in_active_6, in_active_5,  in_active_4,  in_active_3,  in_active_2,  in_active_1}  <= 
		{in_active_5, in_active_4,  in_active_3,  in_active_2,  in_active_1,  in_active};
		
	tx_lcmd_act <= 0;
	tx_lcmd_act_1 <= tx_lcmd_act;
	tx_lcmd_done <= 0;
	if(tx_lcmd_do) tx_lcmd_queue <= 1;
	
	tx_hp_act <= 0;
	tx_hp_act_1 <= tx_hp_act;
	tx_hp_done <= 0;
	if(tx_hp_do) tx_hp_queue <= 1;
	
	rx_lcmd_act <= 0;

	crc_hprx_rst <= 0;
	crc_hptx_rst <= 0;
	crc_dpprx_rst <= 0;
	crc_dpptx_rst <= 0;
	
	buf_in_wren <= 0;
	buf_in_commit <= 0;
	buf_out_arm <= 0;
	buf_in_ready_1 <= buf_in_ready;
	
	prot_rx_tp <= 0;
	prot_rx_dph <= 0;
	prot_tx_tp_a_ack <= 0;
	prot_tx_tp_b_ack <= 0;
	prot_tx_tp_c_ack <= 0;
	prot_tx_dpp_ack <= 0;
	
	if(in_data == 32'h0) `INC(dc);
	`INC(rc);
	`INC(sc);
		
	// atomic increment/decrement
	remote_rx_cred_count_inc <= 0;
	remote_rx_cred_count_dec <= 0;
	remote_rx_cred_count <= remote_rx_cred_count + 
							(remote_rx_cred_count_inc ? 1 : 0) - 
							(remote_rx_cred_count_dec ? 1 : 0);
							
	// detect LTSSM change
	if(ltssm_last != ltssm_state) begin
		ltssm_changed <= 1;
		ltssm_stored <= ltssm_last;
	end	

	if(ltssm_go_recovery & ~ltssm_go_recovery_1) begin
		// rising edge, increment error count
		`INC(link_error_count);
	end
	
	// 
	case(ltssm_state)
	LT_U0: begin
		`INC(pending_hp_timer);
		`INC(credit_hp_timer);
		`INC(pm_entry_timer);
		`INC(ux_exit_timer);
		`INC(port_config_timeout);
		
		// increment U0LTimeout up to its triggering amount (10uS)
		// shall be reset when sending any data, paused then restarted upon idle re-entry
		if(u0l_timeout < T_U0L_TIMEOUT) 
			`INC(u0l_timeout);
			
		// increment U0RecoveryTimeout up to 1ms
		// shall be reset every time link command is received
		if(u0_recovery_timeout < T_U0_RECOVERY)
			`INC(u0_recovery_timeout);
		
		// U0LTimeout generation (LUP/LDN) keepalive
		if(u0l_timeout == T_U0L_TIMEOUT) begin
			// we need to send LUP
			tx_queue_lup <= 1;
			u0l_timeout <= 0;
		end

		// detect absence of far-end heartbeat
		if(u0_recovery_timeout == T_U0_RECOVERY) begin
			// tell LTSSM to transition to Recovery
			ltssm_go_recovery <= 1;
		end
		
		u2_timeout <= 0;
		
		
		// run CREDIT_HP_TIMER if any header packets are pending re-credit
		if(remote_rx_cred_count == 4) credit_hp_timer <= 0;
		if(credit_hp_timer == T_CREDIT_HP) begin
			// stall until any header packet finished sending
			credit_hp_timer <= T_CREDIT_HP;
			if(wr_hp_state == WR_HP_IDLE) begin
				// tell LTSSM to transition to Recovery
				err_credit_hp <= 1;
				ltssm_go_recovery <= 1;
			end
		end
		
		// run PENDING_HP_TIMER if any sent HP were lost, and U0 Adv. is done
		if(remote_rx_cred_count == 4 && recv_u0_adv) pending_hp_timer <= 0;
		if(pending_hp_timer == T_PENDING_HP) begin
			// stall until any header packet finished sending
			pending_hp_timer <= T_PENDING_HP;
			if(wr_hp_state == WR_HP_IDLE) begin
				// tell LTSSM to transition to Recovery
				err_pending_hp <= 1;
				ltssm_go_recovery <= 1;
			end
		end
		
		// run PM_LC_TIMER
		// TODO only needed if device requests PM changes (unlikely)
		
		// run PM_ENTRY_TIMER
		if(!pm_waiting_for_ack) pm_entry_timer <= 0;
		if(pm_entry_timer == T_PM_ENTRY) begin
			pm_entry_timer <= T_PM_ENTRY;
			// we never received any LPMA, but did accept a link PM change, so
			// just assume LPMA was lost and proceed to the new state anyway
			if(pm_waiting_for_ack) begin
				// desired PM state is stored in ltssm_go_u[1:0]
				ltssm_go_u[2] <= 1'b1;
				pm_waiting_for_ack <= 0;
			end
		end
		
		// run Ux_EXIT_TIMER
		// TODO
		
		// run tPortConfiguration timeout
		if(recv_port_cmdcfg == 2'b11) port_config_timeout <= 0;
		if(port_config_timeout == T_PORT_CONFIG) begin
			port_config_timeout <= T_PORT_CONFIG;
			if(recv_port_cmdcfg != 2'b11) begin
				// timer has run out and Port Capability LMP/Port Config LMP were not
				// exchanged properly. As we are an upstream port (device) then goto
				// SS.Disabled.
				ltssm_go_disabled <= 1;
			end
		end
	end
	LT_U1: begin
		// U2 entry timer, if we stay here in U1 too long this boots us to U2
		if(u2_timeout < T_PORT_U2_TIMEOUT)
			`INC(u2_timeout);
			
		// tell LTSSM we want to go to U2
		if(u2_timeout == T_PORT_U2_TIMEOUT)
			ltssm_go_u[2:0] <= {1'b1, 2'd2};
	end
	default: begin  
		// reset timeouts
		u0l_timeout <= 0;
		u0_recovery_timeout <= 0;
		pending_hp_timer <= 0;
		credit_hp_timer <= 0;
		pm_lc_timer <= 0;
		pm_entry_timer <= 0;
		ux_exit_timer <= 0;
		port_config_timeout <= 0;
		
		ltssm_go_disabled <= 0;
	end
	endcase
	
	if(ltssm_state != LT_U0) begin
		// reset advertisement/portcap sent flags
		recv_u0_adv <= 0;
		sent_u0_adv <= 0;
		sent_u0_portcap <= 0;
	end
	
	// handle LTSSM change
	if(ltssm_changed) begin
		ltssm_go_u <= 0;
		
		case(ltssm_state) 
		LT_U0: begin
			// upon entry to U0, we must send : (Page 194)
			// 1. Header Sequence Advertisement [LGOOD_(rx_hdr_seq_num-1)]
			// 2. Advertisement of Link Credits (ABCD)
			// flush all header packets in Tx Header Buffers =< advertised number
			// initialize ACK Tx HdrSeqNum to value_received+1
			// start PENDING_HP_TIMER and CREDIT_HP_TIMER in expectation of HdrSeqNum and RxHdrBufCred
			// Note that the state should be preserved upon U0 entry because Recovery and thereby U0
			// are entered quite frequently, and both ends need to keep track of what packets had been
			// sent prior to Recovery, whether by a random phy error or power state change (ex. U1->U0)
			
			if(ltssm_stored == LT_POLLING_IDLE || ltssm_stored == LT_HOTRESET_EXIT) begin
				// only reset these in inital U0 entry or Hot Reset, lose state
				tx_hdr_seq_num <= 0;
				rx_hdr_seq_num <= 0;
				rx_hdr_seq_ignore <= 0;
				queue_hp_retry <= 0;
				rty_tx_hdr_seq_num <= 0;
				last_hdr_seq_num <= 7;
				local_rx_cred_count <= 4;
				
				// clear HP parse queue
				in_header_pkt_queued <= 4'b0000;
				// flush all header packet buffers
				in_header_pkt_a <= 'h0;
				in_header_pkt_b <= 'h0;
				in_header_pkt_c <= 'h0;
				in_header_pkt_d <= 'h0;
				
				link_error_count <= 0;
			end
			
			if(ltssm_stored == LT_POLLING_IDLE || ltssm_stored == LT_HOTRESET_EXIT) begin
				// initiate port capabilities
				queue_send_u0_portcap <= 1;
				// expect to receive portcap/portcfg
				recv_port_cmdcfg <= 2'h0;
			end else begin
				//recv_port_cmdcfg == 2'b11;
			end
			queue_send_u0_adv <= 1;
			
			// note that the local_rx_cred is not updated here.
			// it should be continually processed by internal logic and updated based on
			// the buffer contents.
			tx_cred_idx <= 0; // A
			rx_cred_idx <= 0; // A
			remote_rx_cred_count <= 0;
			// reset timers
			dc <= 0;
			// reset flags
			force_linkpm_accept <= 0;
			pm_waiting_for_ack <= 0;
			tx_queue_lup <= 0;
			tx_queue_lcred <= 0;
			tx_queue_lgood <= 0;
			tx_queue_open <= 1;
			tx_lcmd_act <= 0;
			tx_lcmd_queue <= 0; ///////
			send_port_cfg_resp <= 0;
			
			// upon sending of first header packet, the remote rx hdr cred count
			// should be decremented
			out_header_first_since_entry <= 1;
			in_header_err_count <= 0;
			
			queue_state <= LINK_QUEUE_RESET;
			read_dpp_state <= READ_DPP_RESET;
			rd_hp_state <= RD_HP_RESET;
			check_hp_state <= CHECK_HP_RESET;
		end
		LT_RECOVERY_IDLE: begin
			// we can de-assert the recovery signal now
			ltssm_go_recovery <= 0;
			rx_hdr_seq_ignore <= 0;
		end
		default: begin
			// we can de-assert the recovery signal now
			ltssm_go_recovery <= 0;
		end
		endcase	
	end
	

	
	in_link_command_act <= 0;
	in_link_command_1 <= in_link_command;
	
	///////////////////////////////////////
	// LINK RECEIVE (RX) FSM
	///////////////////////////////////////
	
	case(recv_state)
	LINK_RECV_RESET: begin
		recv_state <= LINK_RECV_IDLE;
	end
	LINK_RECV_IDLE: begin
		if(ltssm_state == LT_U0 && in_active) begin
			if({in_data, in_datak} == {32'hFEFEFEF7, 4'b1111}) begin
				// Link Command Word
				u0_recovery_timeout <= 0;
				recv_state <= LINK_RECV_CMDW_0;
			end
			
			if({in_data, in_datak} == {32'hFBFBFBF7, 4'b1111}) begin
				// HPSTART ordered set
				rc <= 0;
				crc_hprx_rst <= 1;
				recv_state <= LINK_RECV_HP_0;
			end
		end
	end
	LINK_RECV_HP_0: begin
		// shift in header packet (12 bytes)
		// account for possible active de-assert on last word
		if(in_active) begin
			case(rx_cred_idx)
			0: in_header_pkt_a <= {swap32(in_data), in_header_pkt_a[95:32]};
			1: in_header_pkt_b <= {swap32(in_data), in_header_pkt_b[95:32]};
			2: in_header_pkt_c <= {swap32(in_data), in_header_pkt_c[95:32]};
			3: in_header_pkt_d <= {swap32(in_data), in_header_pkt_d[95:32]};
			endcase
			crc_hprx_in <= swap32(in_data);
			if(rc == 2) recv_state <= LINK_RECV_HP_1;
		end else rc <= rc;
		
		if(~in_active && rc == 0) crc_hprx_rst <= 1;
	end
	LINK_RECV_HP_1: begin
		// load crc16 of HP + control word (4 bytes)
		if(in_active) begin
			in_header_crc <= in_data_swap16[31:16];
			in_header_cw <= in_data_swap16[15:0];
			// delegate crc and checking of header packet to another fsm
			// so that we can start parsing on the very next cycle
			recv_state <= LINK_RECV_IDLE;
			
			// only bother parsing if no header packet error was seen earlier
			// note: cycle delays to HP parse must be consistent
			if(!rx_hdr_seq_ignore) begin
				check_hp_state <= CHECK_HP_0;
			end
		end
	end
	LINK_RECV_CMDW_0: begin
		// byteswap word
		if(in_active) begin
			in_link_command <= in_data_swap16;
			in_link_command_act <= 1;
			recv_state <= LINK_RECV_IDLE;
		end
	end
	endcase
	
	//
	// Link State Expect FSM
	//
	case(expect_state)
	LINK_EXPECT_RESET: expect_state <= LINK_EXPECT_IDLE;
	LINK_EXPECT_IDLE: begin
		if(ltssm_state == LT_POLLING_IDLE || ltssm_state == LT_RECOVERY_IDLE) begin
			// finish link training, we will want to expect
			// Header Sequence Advertisement next
			expect_state <= LINK_EXPECT_HDR_SEQ_AD;
		end
	end
	LINK_EXPECT_HDR_SEQ_AD: begin
		if(ltssm_state == LT_U0) begin
			// now it should happen
			case({rx_lcmd, rx_lcmd_act})
			{LCMD_LGOOD_0, 1'b1}: ack_tx_hdr_seq_num <= 1;
			{LCMD_LGOOD_1, 1'b1}: ack_tx_hdr_seq_num <= 2;
			{LCMD_LGOOD_2, 1'b1}: ack_tx_hdr_seq_num <= 3;
			{LCMD_LGOOD_3, 1'b1}: ack_tx_hdr_seq_num <= 4;
			{LCMD_LGOOD_4, 1'b1}: ack_tx_hdr_seq_num <= 5;
			{LCMD_LGOOD_5, 1'b1}: ack_tx_hdr_seq_num <= 6;
			{LCMD_LGOOD_6, 1'b1}: ack_tx_hdr_seq_num <= 7;
			{LCMD_LGOOD_7, 1'b1}: ack_tx_hdr_seq_num <= 0;
			{LCMD_LCRD_A, 1'b1}: begin 
				//`INC(remote_rx_cred_count); 
				remote_rx_cred_count_inc <= 1;
				`INC(rx_cred_idx);
			end
			{LCMD_LCRD_B, 1'b1}: begin 
				//`INC(remote_rx_cred_count); 
				remote_rx_cred_count_inc <= 1;
				`INC(rx_cred_idx);
			end
			{LCMD_LCRD_C, 1'b1}: begin 
				//`INC(remote_rx_cred_count); 
				remote_rx_cred_count_inc <= 1;
				`INC(rx_cred_idx);
			end
			{LCMD_LCRD_D, 1'b1}: begin 
				//`INC(remote_rx_cred_count); 
				remote_rx_cred_count_inc <= 1;
				`INC(rx_cred_idx);
				pending_hp_timer <= 0;
				credit_hp_timer <= 0;
				recv_u0_adv <= 1;
				expect_state <= LINK_EXPECT_IDLE;
			end
			endcase 
		end
	end
	endcase
	
	//
	// Read Data Packet Payload FSM
	//
	case(read_dpp_state)
	READ_DPP_RESET: read_dpp_state <= READ_DPP_IDLE;
	READ_DPP_IDLE: begin
		if(ltssm_state == LT_U0 && in_active_6) begin
			read_dpp_state <= READ_DPP_0;
			buf_in_wren <= 0;
		end
	end
	READ_DPP_0: begin
		// next cycle, DPP will appear on pipelined in_data
		buf_in_addr <= -1;
		buf_in_wren <= 0;
		in_dpp_length <= 0;
		in_dpp_wasready <= 0;
		buf_in_commit_len <= in_dpp_length_expect;
		crc_dpprx_rst <= 1;
		if({in_data_6, in_datak_6, in_active_6} == {32'h5C5C5CF7, 4'b1111, 1'b1}) begin
			read_dpp_state <= READ_DPP_1;
			prot_rx_dpp_start <= 1;
			in_dpp_wasready <= buf_in_ready;
		end
		if(ltssm_state != LT_U0) read_dpp_state <= READ_DPP_RESET;
	end
	READ_DPP_1: begin
		if(in_active_6) begin
			if(	{in_data_6, in_datak_6} == {32'hFDFDFDF7, 4'b1111}) begin
				// EOP, too early
				read_dpp_state <= READ_DPP_IDLE;
				err_dpp_len_mismatch <= 1;
				
			end else begin
				in_dpp_length <= in_dpp_length + 3'h4;
				`INC(buf_in_addr);
				buf_in_data <= in_data_6;
				crc_dpprx_in <= swap32(in_data_6);
				buf_in_wren <= in_dpp_wasready;
				//if(in_dpp_length == in_dpp_length_expect)
				//	err_dpp_len_mismatch <= 1;
					
				if((in_dpp_length + 3'h4) >= in_dpp_length_expect) begin
					// EOP
					read_dpp_state <= READ_DPP_2;
				end
			end
		end else begin
			if(in_dpp_length == 0) crc_dpprx_rst <= 1;
		end
	end
	READ_DPP_2: begin
		if(in_active_6) begin
			// latch CRC
			in_dpp_crc32 <= swap32(in_data_6);
			read_dpp_state <= READ_DPP_3;
		end
	end
	READ_DPP_3: begin
		// verify CRC
		prot_rx_dpp_done <= 1;
		if(in_dpp_crc32 == crc_dpprx32_out) begin
			// match
			prot_rx_dpp_crcgood <= 1;
			buf_in_commit <= 1; // TODO move commits to protocol layer
		end else begin
			// fail
			prot_rx_dpp_crcgood <= 1;
			buf_in_commit <= 1;
		end
		read_dpp_state <= READ_DPP_IDLE;
	end
	endcase
	
	
	//
	// Check Header Packet FSM
	// 
	case(check_hp_state)
	CHECK_HP_RESET: check_hp_state <= CHECK_HP_IDLE;
	CHECK_HP_IDLE: begin
		// do nothing, triggered by LINK_RECV_FSM	
	end
	CHECK_HP_0: begin
		// check CRC-5, CRC-16 against calculated values
		if(crc_hprx_out == in_header_crc && crc_cw3_out == in_header_cw[15:11]) begin
			// matched
			if(in_header_seq == rx_hdr_seq_num && local_rx_cred_count > 0) begin
				// header seq num matches what we expect
				check_hp_state <= CHECK_HP_1;
			end else begin
				err_hp_seq <= 1;
				check_hp_state <= CHECK_HP_IDLE;
				// transition to Recovery
				ltssm_go_recovery <= 1;
			end
			
		end else begin
			// didn't match
			// issue LBAD and do nothing until either 
			// 1. Recovery entered
			// 2. LRTY received
			err_lbad <= 1;
			tx_lcmd <= LCMD_LBAD;
			tx_lcmd_act <= 1;
			err_hp_crc <= 1;
			`INC(in_header_err_count);
			rx_hdr_seq_ignore <= 1;
			check_hp_state <= CHECK_HP_IDLE;
			//`INC(rx_hdr_seq_num); /// NOPE
			//`INC(rx_cred_idx); // NOPE AS WELL
			if(in_header_err_count >= 2) begin
				// give up, go to Recovery
				// do not transmit LBAD on the third consecutive try
				tx_lcmd_act <= 0;
				ltssm_go_recovery <= 1;
			end
		end
	end
	CHECK_HP_1: begin
		// received properly
		// reset HP error count, decrement local RX cred count
		in_header_err_count <= 0;
		`DEC(local_rx_cred_count);
		`INC(rx_hdr_seq_num);
		// set dirty bit in queue, cause HP to be parsed by next FSM
		in_header_pkt_queued[3-rx_cred_idx] <= 1;
		rx_cred_idx_cache <= rx_cred_idx;
		`INC(rx_cred_idx); // TOOD ORKRDSR?
		// send LGOOD
		tx_queue_lgood <= {1'b1, rx_hdr_seq_num[2:0]};
		check_hp_state <= CHECK_HP_IDLE;
	end
	endcase
	
	//
	// Parse Header Packet FSM
	//
	case(rd_hp_state)
	RD_HP_RESET: rd_hp_state <= RD_HP_IDLE;
	RD_HP_IDLE: begin
		case(rx_cred_idx_cache)
		0: begin
			if(in_header_pkt_queued[3]) in_header_pkt_pick <= 3; else
			if(in_header_pkt_queued[2]) in_header_pkt_pick <= 2; else
			if(in_header_pkt_queued[1]) in_header_pkt_pick <= 1; else
			if(in_header_pkt_queued[0]) in_header_pkt_pick <= 0; end
		1: begin
			if(in_header_pkt_queued[2]) in_header_pkt_pick <= 2; else
			if(in_header_pkt_queued[1]) in_header_pkt_pick <= 1; else
			if(in_header_pkt_queued[0]) in_header_pkt_pick <= 0; else
			if(in_header_pkt_queued[3]) in_header_pkt_pick <= 3; end
		2: begin
			if(in_header_pkt_queued[1]) in_header_pkt_pick <= 1; else
			if(in_header_pkt_queued[0]) in_header_pkt_pick <= 0; else
			if(in_header_pkt_queued[3]) in_header_pkt_pick <= 3; else
			if(in_header_pkt_queued[2]) in_header_pkt_pick <= 2; end
		3: begin
			if(in_header_pkt_queued[0]) in_header_pkt_pick <= 0; else
			if(in_header_pkt_queued[3]) in_header_pkt_pick <= 3; else
			if(in_header_pkt_queued[2]) in_header_pkt_pick <= 2; else
			if(in_header_pkt_queued[1]) in_header_pkt_pick <= 1; end
		endcase
		if(|in_header_pkt_queued) rd_hp_state <= RD_HP_0;
	end
	RD_HP_0: begin
	
		case(in_header_pkt_mux[4:0])
		LP_TYPE_LMP: begin
			case(in_header_pkt_mux[8:5])
			LP_LMP_SUB_SETLINK: force_linkpm_accept <= in_header_pkt_mux[10];
			LP_LMP_SUB_U2INACT: T_PORT_U2_TIMEOUT <= in_header_pkt_mux[16:9] * 32000; // 256 uS units
			LP_LMP_SUB_VENDTEST: begin end
			LP_LMP_SUB_PORTCAP: begin 
				recv_port_cmdcfg[1] <= 1'b1;
			end
			LP_LMP_SUB_PORTCFG: begin
				send_port_cfg_resp <= 1;
				recv_port_cmdcfg[0] <= 1'b1;
			end
			LP_LMP_SUB_PORTCFGRSP: begin 
				
			end
			endcase		
		end
		LP_TYPE_TP: begin
			// pass the Transaction Packet info onto protocol layer
			prot_rx_tp <= 1;
			prot_rx_tp_hosterr	<= in_header_pkt_mux[(15+32)];
			prot_rx_tp_retry	<= in_header_pkt_mux[( 6+32)];
			prot_rx_tp_pktpend	<= in_header_pkt_mux[(27+64)];
			prot_rx_tp_subtype	<= in_header_pkt_mux[( 3+32):( 0+32)];
			prot_rx_tp_endp		<= in_header_pkt_mux[(11+32):( 8+32)];
			prot_rx_tp_nump		<= in_header_pkt_mux[(20+32):(16+32)];
			prot_rx_tp_seq		<= in_header_pkt_mux[(25+32):(21+32)];
			prot_rx_tp_stream	<= in_header_pkt_mux[79:64];
		end		
		LP_TYPE_DP: begin
			// parse data packet preamble
			// read_dpp FSM will trap payload ordered set itself
			in_dpp_length_expect	<= in_header_pkt_mux[(31+32):(16+32)];

			// pass the Data Packet Header info onto protocol layer
			prot_rx_dph <= 1;
			prot_rx_dph_eob		<= in_header_pkt_mux[( 6+32)];
			prot_rx_dph_setup	<= in_header_pkt_mux[(15+32)];
			prot_rx_dph_pktpend	<= in_header_pkt_mux[(27+64)];
			prot_rx_dph_endp	<= in_header_pkt_mux[(11+32):( 8+32)];
			prot_rx_dph_seq		<= in_header_pkt_mux[( 4+32):( 0+32)];
			prot_rx_dph_len		<= in_header_pkt_mux[(31+32):(16+32)];
			prot_rx_dpp_start	<= 0;
			prot_rx_dpp_done	<= 0;
		end
		LP_TYPE_ITP: begin
			itp_value <= in_header_pkt_mux[31:5];
		end
		default: err_hp_type <= 1;
		endcase
		
		in_header_pkt_queued[in_header_pkt_pick] <= 0;
		tx_queue_lcred[2:0] <= {1'b1, rx_cred_idx_cache[1:0]};
		`INC(local_rx_cred_count);
		//`INC(rx_cred_idx);
		rd_hp_state <= RD_HP_IDLE;
	end
	endcase
	
	   
	//
	// Link Command READ FSM
	//
	case(rd_lcmd_state) 
	RD_LCMD_RESET: rd_lcmd_state <= RD_LCMD_IDLE;
	RD_LCMD_IDLE: begin
		if(in_link_command_act) begin
			// make sure command words match
			if(	in_link_command[10:0] == in_link_command[26:16] &&
				crc_cw1_out == in_link_command[31:27] &&
				crc_cw2_out == in_link_command[15:11] ) begin
				
				rd_lcmd_state <= RD_LCMD_0;
			end
		end
	end 
	RD_LCMD_0: begin
		rd_lcmd_state <= RD_LCMD_IDLE;
		rx_lcmd <= in_link_command_1[10:0];
		rx_lcmd_act <= 1;
	end
	endcase
	
	
	// parse link commands after initial u0 advertisement
	// and link management header packets
	//
	if(rx_lcmd_act) begin
		if(rx_lcmd[10:3] == LCMD_LGOOD_0[10:3] && sent_u0_adv && recv_u0_adv) begin
			// LGOOD, after U0 advertisement is done
			if(ack_tx_hdr_seq_num != rx_lcmd[2:0]) begin
				// out of order
				err_lgood_order <= 1;
				ltssm_go_recovery <= 1;
			end else begin
				`INC(ack_tx_hdr_seq_num);
				//`INC(tx_hdr_seq_num);
				pending_hp_timer <= 0;
			end
			
		end else
		if(rx_lcmd[10:2] == LCMD_LCRD_A[10:2] && sent_u0_adv && recv_u0_adv) begin
			// LCRD, after U0 advertisement
			if(remote_rx_cred_count > 3) begin
				// mismatch, remote had too many credits
				err_lcrd_mismatch <= 1;
				ltssm_go_recovery <= 1;
			end else if(tx_cred_idx != rx_lcmd[1:0]) begin
				// out of sequence
				err_lcrd_mismatch <= 1;
				ltssm_go_recovery <= 1;
			end else if (0) begin // TODO DETECT LCRD WTHIOUT LGOOD
				err_lgood_missed <= 1;
				ltssm_go_recovery <= 1;
			end else begin
				`INC(tx_cred_idx);
				//`INC(remote_rx_cred_count); 
				remote_rx_cred_count_inc <= 1;
				credit_hp_timer <= 0;
			end
		end else
		if(rx_lcmd[10:2] == LCMD_LGO_U1[10:2]) begin
			// LGO_x, latch U1,U2,U3
			ltssm_go_u[1:0] <= rx_lcmd[1:0];
			if(local_rx_cred_count < 4 && !force_linkpm_accept) begin // rx_lcmd[1:0] == 2'b11 || 
				// reject if not enough credits
				tx_lcmd <= LCMD_LXU;
			end else begin
				tx_lcmd <= LCMD_LAU;	// TODO stall until packet's done if necessary
				pm_waiting_for_ack <= 1;
			end
			tx_lcmd_act <= 1;
		end else
		if(rx_lcmd == LCMD_LPMA) begin
			// Host acknowledged our acceptance of new power state
			ltssm_go_u[2] <= 1'b1;
			pm_waiting_for_ack <= 0;
		end else
		if(rx_lcmd == LCMD_LRTY) begin
			// re-receive header packet
			// we've been ignoring all header packets since the error, but now start listening
			rx_hdr_seq_ignore <= 0;
		end else
		if(rx_lcmd == LCMD_LBAD) begin
			// re-send all un-acknowledged header packets
			queue_hp_retry <= 1;
			rty_tx_hdr_seq_num <= ack_tx_hdr_seq_num;
			err_lbad_recv <= 1;
		end
	
	end
	

	
	//
	// Link State Queue FSM
	//
	case(queue_state)
	LINK_QUEUE_RESET: queue_state <= LINK_QUEUE_IDLE;
	LINK_QUEUE_IDLE: begin
		qc <= 0;
		// check for queued advertisement from U0 entry, and wait a bit
		if(queue_send_u0_adv && (recv_u0_adv||1) && dc > 1) begin
			queue_state <= LINK_QUEUE_HDR_SEQ_AD;
			queue_send_u0_adv <= 0;
		end else if( send_state == LINK_SEND_IDLE && !tx_hp_do) begin // 
			// send Port Capability HP once advertisement is finished
			if(queue_send_u0_portcap && sent_u0_adv) begin
				queue_state <= LINK_QUEUE_PORTCAP;
				queue_send_u0_portcap <= 0;
			end else
			if(!sent_u0_adv || !recv_u0_adv) begin
				// do nothing, prevent below events from firing
			end else
			// send Port Configuration Response HP
			if(send_port_cfg_resp) begin
				queue_state <= LINK_QUEUE_PORTCFGRSP;
				send_port_cfg_resp <= 0;
			end else
			// retry HP send
			if(queue_hp_retry) begin
				queue_state <= LINK_QUEUE_RTY_HP;
			end else
			// send TP from Protocol Layer C
			if(prot_tx_tp_c) begin
				queue_state <= LINK_QUEUE_TP_C;
			end else
			// send TP from Protocol Layer B
			if(prot_tx_tp_b) begin
				queue_state <= LINK_QUEUE_TP_B;
			end else
			// send TP from Protocol Layer A
			if(prot_tx_tp_a) begin
				queue_state <= LINK_QUEUE_TP_A;
			end else
			// send DPH, DPP from Protocol Layer
			if(prot_tx_dph) begin
				queue_state <= LINK_QUEUE_DP;
			end
		end
	end
	LINK_QUEUE_HDR_SEQ_AD: begin
		if(tx_lcmd_done) `INC(qc);
		else
		case(qc)
		0: begin
			tx_lcmd <= {LCMD_LGOOD_0[10:3], rx_hdr_seq_num_dec[2:0]};
			tx_lcmd_act <= 1;
		end
		1: begin
			{tx_lcmd, tx_lcmd_act} <= {LCMD_LCRD_A, 1'b1};
			if(local_rx_cred_count == 1) qc <= 5; 	// only advertise up to LOCAL_RX_CRED_COUNT
		end
		2: begin
			{tx_lcmd, tx_lcmd_act} <= {LCMD_LCRD_B, 1'b1};
			if(local_rx_cred_count == 2) qc <= 5;	// only advertise up to LOCAL_RX_CRED_COUNT
		end
		3: begin
			{tx_lcmd, tx_lcmd_act} <= {LCMD_LCRD_C, 1'b1};
			if(local_rx_cred_count == 3) qc <= 5;	// only advertise up to LOCAL_RX_CRED_COUNT
		end
		4: begin
			{tx_lcmd, tx_lcmd_act} <= {LCMD_LCRD_D, 1'b1};
		end
		5: begin 
			sent_u0_adv <= 1; 
			queue_state <= LINK_QUEUE_IDLE; 
		end
		default: queue_state <= LINK_QUEUE_IDLE;
		endcase
	end
	LINK_QUEUE_PORTCAP: begin
		case(qc)
		0: begin
			tx_hp_word_0 <= {16'h0, LP_LMP_SPEED_5GBPS, LP_LMP_SUB_PORTCAP, LP_TYPE_LMP};
			tx_hp_word_1 <= {8'h0, LP_LMP_TIEBREAK, 1'b0, LP_LMP_OTG_INCAPABLE, LP_LMP_DIR_UP, 8'b0, LP_LMP_NUM_HP_4};
			tx_hp_word_2 <= {32'h0};
			tx_hp_act <= 1;
			qc <= 1;
		end
		1: begin
			sent_u0_portcap <= 1;
			queue_state <= LINK_QUEUE_IDLE;
		end		
		endcase
	end
	LINK_QUEUE_PORTCFGRSP: begin
		case(qc)
		0: begin
			tx_hp_word_0 <= {16'h0, LP_LMP_SPEED_ACCEPT, LP_LMP_SUB_PORTCFGRSP, LP_TYPE_LMP};
			tx_hp_word_1 <= {32'h0};
			tx_hp_word_2 <= {32'h0};
			tx_hp_act <= 1;
			qc <= 1;
		end
		1: queue_state <= LINK_QUEUE_IDLE;
		endcase
	end
	LINK_QUEUE_RTY_HP: begin
		tx_hp_retry <= 1;
		if(tx_lcmd_done) `INC(qc);
		else
		case(qc)
		0: begin
			tx_lcmd <= LCMD_LRTY;
			tx_lcmd_act <= 1;
		end
		1: begin
			if(tx_hp_done) begin
				queue_state <= LINK_QUEUE_IDLE;
				queue_hp_retry <= 0;
			end
		end		
		default: queue_state <= LINK_QUEUE_IDLE;
		endcase
	end
	LINK_QUEUE_TP_A: begin
		tx_hp_word_0 <= {	local_dev_addr, LP_TP_ROUTE0, LP_TYPE_TP};
		tx_hp_word_1 <= {	6'h0, prot_tx_tp_a_seq, prot_tx_tp_a_nump, 
							1'b0, 3'h0, prot_tx_tp_a_endp, prot_tx_tp_a_dir, prot_tx_tp_a_retry, 2'h0, prot_tx_tp_a_subtype};
		tx_hp_word_2 <= {	LP_TP_NBI_0, LP_TP_PPEND_NO, LP_TP_DBI_NO, LP_TP_WPA_NO, LP_TP_SSI_NO, 8'h0, prot_tx_tp_a_stream};
		tx_hp_act <= 1;
		prot_tx_tp_a_ack <= 1;
		queue_state <= LINK_QUEUE_IDLE;
	end
	LINK_QUEUE_TP_B: begin
		tx_hp_word_0 <= {	local_dev_addr, LP_TP_ROUTE0, LP_TYPE_TP};
		tx_hp_word_1 <= {	6'h0, prot_tx_tp_b_seq, prot_tx_tp_b_nump, 
							1'b0, 3'h0, prot_tx_tp_b_endp, prot_tx_tp_b_dir, prot_tx_tp_b_retry, 2'h0, prot_tx_tp_b_subtype};
		tx_hp_word_2 <= {	LP_TP_NBI_0, LP_TP_PPEND_NO, LP_TP_DBI_NO, LP_TP_WPA_NO, LP_TP_SSI_NO, 8'h0, prot_tx_tp_b_stream};
		tx_hp_act <= 1;
		local_dev_addr <= dev_addr;
		prot_tx_tp_b_ack <= 1;
		queue_state <= LINK_QUEUE_IDLE;
	end
	LINK_QUEUE_TP_C: begin
		tx_hp_word_0 <= {	local_dev_addr, LP_TP_ROUTE0, LP_TYPE_TP};
		tx_hp_word_1 <= {	6'h0, prot_tx_tp_c_seq, prot_tx_tp_c_nump, 
							1'b0, 3'h0, prot_tx_tp_c_endp, prot_tx_tp_c_dir, prot_tx_tp_c_retry, 2'h0, prot_tx_tp_c_subtype};
		tx_hp_word_2 <= {	LP_TP_NBI_0, LP_TP_PPEND_NO, LP_TP_DBI_NO, LP_TP_WPA_NO, LP_TP_SSI_NO, 8'h0, prot_tx_tp_c_stream};
		tx_hp_act <= 1;
		local_dev_addr <= dev_addr;
		prot_tx_tp_c_ack <= 1;
		queue_state <= LINK_QUEUE_IDLE;
	end
	LINK_QUEUE_DP: begin
		case(qc)
		0: begin
			if(send_state == LINK_SEND_IDLE) begin
			tx_hp_word_0 <= {	local_dev_addr, LP_TP_ROUTE0, LP_TYPE_DP};
			tx_hp_word_1 <= {	prot_tx_dph_len, 1'b0, 3'b0, prot_tx_dph_endp, prot_tx_dph_dir, prot_tx_dph_eob, 1'b0, prot_tx_dph_seq};
			tx_hp_word_2 <= {	32'h0};
			tx_hp_act <= 1;
			tx_hp_dph <= 1;
			out_dpp_length <= prot_tx_dph_len;
			prot_tx_dpp_ack <= 1;
			prot_tx_dpp_done <= 0;
			qc <= 1;
			end
		end
		1: begin
			queue_state <= LINK_QUEUE_IDLE;
		end
		endcase
	end
	endcase

	
	
	///////////////////////////////////////
	// LINK SEND (TX) FSM
	///////////////////////////////////////
	
	case(send_state)
	LINK_SEND_RESET: begin
		send_state <= LINK_SEND_IDLE;
	end
	LINK_SEND_IDLE: begin
		// N.B. potential race condition with loading queued CMD
		// upon exact cycle where queue was closed with another
		// TODO figure out if this ever happens
		tx_queue_open <= 1;
		
		if(sent_u0_adv == 0) begin
			tx_queue_open <= 0;
		end
		
		if(ltssm_state != LT_U0 || (pm_waiting_for_ack && tx_lcmd != LCMD_LAU)) begin
			// freeze until U0
			// don't send any packets or commands while waiting on LPMA
			// but do allow LAU to be sent
			tx_queue_open <= 0;
		end else
		if(tx_lcmd_do) begin
			//tx_lcmd_act <= 0;
			tx_lcmd_queue <= 0;
			tx_queue_open <= 0;
			send_state <= LINK_SEND_CMDW_0;
		end else 
		if(|tx_queue_lgood) begin
			tx_queue_lgood <= 0;
			tx_queue_open <= 0;
			tx_lcmd <= {LCMD_LGOOD_0[10:3], tx_queue_lgood[2:0]};
			send_state <= LINK_SEND_CMDW_0;
		end else
		if(|tx_queue_lcred) begin
			tx_queue_lcred <= 0;
			tx_queue_open <= 0;
			tx_lcmd <= {LCMD_LCRD_A[10:2], tx_queue_lcred[1:0]};
			send_state <= LINK_SEND_CMDW_0;
		end else 
		if(tx_hp_do && remote_rx_cred_count > 0) begin
			tx_queue_open <= 0;
			tx_hp_queue <= 0;
			send_state <= LINK_SEND_HP_0;
		end else
		if(tx_hp_retry && remote_rx_cred_count > 0 && qc > 0) begin
			tx_queue_open <= 0;
			send_state <= LINK_SEND_HP_RTY;
		end else
		if(tx_queue_lup) begin
			tx_queue_lup <= 0;
			tx_queue_open <= 0;
			tx_lcmd <= LCMD_LUP;
			send_state <= LINK_SEND_CMDW_0;
		end
	end
	LINK_SEND_CMDW_0: begin
		if(wr_lcmd_state != WR_LCMD_0) begin
			tx_lcmd_latch <= tx_lcmd;
			wr_lcmd_state <= WR_LCMD_0;
			// reset U0 LUP timeout
			u0l_timeout <= 0;
			send_state <= LINK_SEND_IDLE;
		end
	end
	LINK_SEND_HP_RTY: begin
		// set to relevant one
		out_header_pkt_pick <= tx_cred_idx;
		send_state <= LINK_SEND_HP_1;
	end
	LINK_SEND_HP_0: begin
		// commit built packet to queue
		case(tx_cred_idx)
		0: out_header_pkt_a <= {tx_hp_word_0, tx_hp_word_1, tx_hp_word_2};
		1: out_header_pkt_b <= {tx_hp_word_0, tx_hp_word_1, tx_hp_word_2};
		2: out_header_pkt_c <= {tx_hp_word_0, tx_hp_word_1, tx_hp_word_2};
		3: out_header_pkt_d <= {tx_hp_word_0, tx_hp_word_1, tx_hp_word_2};
		endcase	
		// set to relevant one
		out_header_pkt_pick <= tx_cred_idx;
		
		send_state <= LINK_SEND_HP_1;
	end
	LINK_SEND_HP_1: begin
		if(wr_hp_state == WR_HP_IDLE) begin
			wr_hp_state <= WR_HP_0;
			send_state <= LINK_SEND_HP_2;
		end
	end
	LINK_SEND_HP_2: begin
		if(wr_hp_state == WR_HP_IDLE) begin
			tx_hp_retry <= 0;
			send_state <= LINK_SEND_IDLE;
		end
	end
	endcase	
	if(ltssm_state != LT_U0) send_state <= LINK_SEND_RESET;
	
	
	//
	// Link Command WRITE FSM
	//
	case(wr_lcmd_state) 
	WR_LCMD_RESET: wr_lcmd_state <= WR_LCMD_IDLE;
	WR_LCMD_IDLE: begin
		// dummy state
		// triggered by above FSM
	end
	WR_LCMD_0: begin
		{out_data, out_datak} <= {32'hFEFEFEF7, 4'b1111};
		out_active <= 1;
		crc_lcmd_in <= tx_lcmd_latch;
		tx_lcmd_out <= tx_lcmd_latch;	// retain this for next cycle so it won't get clobbered
		wr_lcmd_state <= WR_LCMD_1;
	end
	WR_LCMD_1: begin
		{out_data, out_datak} <= {{2{tx_lcmd_out[7:0], crc_lcmd_out[4:0], tx_lcmd_out[10:8]}}, 4'b00};
		out_active <= 1;
		
		if(send_state == LINK_SEND_CMDW_0) 	// look at SEND fsm, is it trying to send command?
			wr_lcmd_state <= WR_LCMD_0;		// immediately write another command
		else	
			wr_lcmd_state <= WR_LCMD_IDLE;	// idle
			
		tx_lcmd_done <= 1;
	end
	endcase
	
	
	//
	// Header Packet WRITE FSM
	//
	if(wr_hp_state != WR_HP_IDLE) begin
		// trickery to get around CRC cycle latency
		{out_data, out_data_1} <= {out_data_1, out_data_2} ; 
		{out_datak, out_datak_1} <= {out_datak_1, out_datak_2} ; 
		{out_active, out_active_1} <= {out_active_1, out_active_2} ; 
	end	
	case(wr_hp_state) 
	WR_HP_RESET: wr_hp_state <= WR_HP_IDLE;
	WR_HP_IDLE: begin
		// dummy state
		// triggered by above FSM
	end
	WR_HP_0: begin
		crc_hptx_rst <= 1;
		if(!tx_hp_retry) remote_rx_cred_count_dec <= 1;
			//`DEC(remote_rx_cred_count);

		{out_data_2, out_datak_2} <= {32'hFBFBFBF7, 4'b1111};	// HPSTART ordered set
		out_active_2 <= 1;
		sc <= 0;
		wr_hp_state <= WR_HP_1;
	end
	WR_HP_1: begin
		case(sc)
		0: out_data_2 <= swap32(out_header_pkt_mux[95:64]);
		1: out_data_2 <= swap32(out_header_pkt_mux[63:32]);
		2: out_data_2 <= swap32(out_header_pkt_mux[31:0]);
		endcase		
		if(sc == 0) begin
			last_hdr_seq_num <= tx_hdr_seq_num;
			if(tx_hdr_seq_num == last_hdr_seq_num && !tx_hp_retry) err_stuck_hpseq <= 1;
		end
		out_header_cw <= {1'b0, tx_hp_retry, 6'b0, tx_hdr_seq_num};
		out_active_2 <= 1;
		if(sc == 2) wr_hp_state <= WR_HP_2;
	end
	WR_HP_2: begin
		wr_hp_state <= WR_HP_3;
		if(tx_hp_dph) begin
			// fire up DPP writer
			write_dpp_state <= WRITE_DPP_0;
		end
	end
	WR_HP_3: begin
		out_data_1 <= swap32({crc_cw4_out, out_header_cw, crc_hptx_out});
		out_active_1 <= 1;

		wr_hp_state <= WR_HP_4;
	end
	WR_HP_4: begin
		// TODO only increment if this wasn't a RETRY
		if(!tx_hp_retry) `INC(tx_hdr_seq_num);
		if(out_header_first_since_entry) begin
			//remote_rx_cred_count_dec <= 1;
			out_header_first_since_entry <= 0;
		end
		if(tx_hp_dph) begin
			wr_hp_state <= WR_HP_5;
		end else begin
			tx_hp_done <= 1;
			wr_hp_state <= WR_HP_IDLE;
		end
	end
	WR_HP_5: begin
		// wait for DPP write to complete
		if(write_dpp_state == WRITE_DPP_IDLE) begin
			buf_out_arm <= 1;
			tx_hp_done <= 1;
			prot_tx_dpp_done <= 1;
			wr_hp_state <= WR_HP_IDLE;
		end
	end
	endcase
	
	//
	// Write Data Packet Payload FSM
	//
	// TODO refactor into fewer states
	//
	case(write_dpp_state)
	WRITE_DPP_RESET: write_dpp_state <= WRITE_DPP_IDLE;
	WRITE_DPP_IDLE: begin
		//write_dpp_state <= WRITE_DPP_0;
		buf_out_addr <= 0;
	end
	WRITE_DPP_0: begin
		`INC(buf_out_addr);
		
		write_dpp_state <= WRITE_DPP_1;
	end
	WRITE_DPP_1: begin
		`INC(buf_out_addr);
		
		{out_data_1, out_datak_1} <= {32'h5C5C5CF7, 4'b1111};	// DPP ordered set
		out_active_1 <= 1;
		out_dpp_length_remain <= out_dpp_length;
		out_dpp_length_remain_1 <= out_dpp_length;
		crc_dpptx_rst <= 1;
		
		write_dpp_state <= WRITE_DPP_2;
	end
	WRITE_DPP_2: begin
		case(out_dpp_length_remain_1)
		3: begin
			{out_data, out_datak} <= {out_data_1[31:8], crc_dpptx_out[31:24], 4'b0000};
			write_dpp_state <= WRITE_DPP_6;
		end
		2: begin
			{out_data, out_datak} <= {out_data_1[31:16], crc_dpptx_out[31:16], 4'b0000};
			write_dpp_state <= WRITE_DPP_7;
		end
		1: begin
			{out_data, out_datak} <= {out_data_1[31:24], crc_dpptx_out[31:8], 4'b0000};
			write_dpp_state <= WRITE_DPP_8;
		end
		0: begin
			{out_data, out_datak} <= {out_data_1, 4'b0000};
			write_dpp_state <= WRITE_DPP_5;
		end
		//4: begin
		//	{out_data_1, out_datak_1} <= {buf_out_q, 4'b0000};
		//	if(out_dpp_length == 4)write_dpp_state <= WRITE_DPP_4; else write_dpp_state <= WRITE_DPP_5;
		//end
		
		default: begin
			{out_data_1, out_datak_1} <= {buf_out_q, 4'b0000};
		end
		endcase
		
		if(out_dpp_length_remain_1 == 4 )
		if(out_dpp_length == 4) write_dpp_state <= WRITE_DPP_4; else begin
			{out_data_1, out_datak_1} <= {buf_out_q, 4'b0000};
			write_dpp_state <= WRITE_DPP_9;
		end
		
		// decrement length remaining
		if(out_dpp_length_remain > 4)
			out_dpp_length_remain <= out_dpp_length_remain - 16'h4;
		else
			out_dpp_length_remain <= 0;
		out_dpp_length_remain_1 <= out_dpp_length_remain;
		
		`INC(buf_out_addr);
		crc_dpptx_out_1 <= crc_dpptx_out;
		
		// feed data to CRC
		crc_dpptx_in <= swap32(buf_out_q);
		out_active_1 <= 1;
	end
	WRITE_DPP_3: begin
		// done
		tx_hp_dph <= 0;
		write_dpp_state <= WRITE_DPP_IDLE;
	end
	
	WRITE_DPP_4: begin
		 write_dpp_state <= WRITE_DPP_9;
	end
	WRITE_DPP_5: begin
		{out_data, out_datak, out_active} <= {crc_dpptx_out_1, 4'b0000, 1'b1}; write_dpp_state <= WRITE_DPP_10;
	end
	WRITE_DPP_6: begin
		{out_data, out_datak, out_active} <= {crc_dpptx_out_1[23:0], 8'hFD, 4'b0001, 1'b1}; write_dpp_state <= WRITE_DPP_11;
	end
	WRITE_DPP_7: begin 
		{out_data, out_datak, out_active} <= {crc_dpptx_out_1[15:0], 16'hFDFD, 4'b0011, 1'b1}; write_dpp_state <= WRITE_DPP_12;
	end
	WRITE_DPP_8: begin
		{out_data, out_datak, out_active} <= {crc_dpptx_out_1[7:0], 24'hFDFDFD, 4'b0111, 1'b1}; write_dpp_state <= WRITE_DPP_13;
	end
	
	WRITE_DPP_9: begin
		{out_data, out_datak, out_active} <= {crc_dpptx_out, 4'b0000, 1'b1};
		{out_data_1, out_datak_1, out_active_1} <= {32'hFDFDFDF7, 4'b1111, 1'b1}; write_dpp_state <= WRITE_DPP_3;
	end
	WRITE_DPP_10: begin
		{out_data, out_datak, out_active} <= {32'hFDFDFDF7, 4'b1111, 1'b1}; write_dpp_state <= WRITE_DPP_3;
	end
	WRITE_DPP_11: begin
		{out_data, out_datak, out_active} <= {32'hFDFDF700, 4'b1110, 1'b1}; write_dpp_state <= WRITE_DPP_3;
	end
	WRITE_DPP_12: begin 
		{out_data, out_datak, out_active} <= {32'hFDF70000, 4'b1100, 1'b1}; write_dpp_state <= WRITE_DPP_3;
	end
	WRITE_DPP_13: begin
		{out_data, out_datak, out_active} <= {32'hF7000000, 4'b1000, 1'b1}; write_dpp_state <= WRITE_DPP_3;
	end
	endcase

	
	
	if(~reset_n) begin
		send_state <= LINK_SEND_RESET;
		recv_state <= LINK_RECV_RESET;
		expect_state <= LINK_EXPECT_RESET;
		queue_state <= LINK_QUEUE_RESET;
		rd_lcmd_state <= RD_LCMD_RESET;
		rd_hp_state <= RD_HP_RESET;
		wr_lcmd_state <= WR_LCMD_RESET;
		wr_hp_state <= WR_HP_RESET;
		check_hp_state <= CHECK_HP_RESET;
		
		err_lcmd_undefined <= 0;
		err_lcrd_mismatch <= 0;
		err_lgood_order <= 0;
		err_lgood_missed <= 0;
		err_pending_hp <= 0;
		err_credit_hp <= 0;
		err_hp_crc <= 0;
		err_hp_seq <= 0;
		err_hp_type <= 0;
		err_dpp_len_mismatch <= 0;
		err_lbad <= 0;
		err_lbad_recv <= 0;
		err_stuck_hpseq <= 0;
		
		link_error_count <= 0;
		ltssm_go_recovery <= 0;
		ltssm_go_u <= 0;
		queue_send_u0_adv <= 0;
		
		local_dev_addr <= 0;
		tx_hp_dph <= 0;
		tx_hp_retry <= 0;
		
		prot_rx_dpp_done <= 0;
		prot_tx_dpp_done <= 0;

	end
end




//
// CRC-32 of Data Packet Payloads
//
	//wire	[31:0]	crc_dpprx_out = (in_dpp_length_remain_1 == 4) ? (swap32(crc_dpprx32_out)) :
	//								(in_dpp_length_remain_1 == 3) ? (swap32(crc_dpprx24_out)) :
	//								(in_dpp_length_remain_1 == 2) ? (swap32(crc_dpprx16_out)) :
	//								(in_dpp_length_remain_1 == 1) ? (swap32(crc_dpprx8_out)) : (swap32(crc_dpprx32_out));
									// TODO FIXUP FOR RX
	wire	[31:0]	crc_dpprx_q;
	reg				crc_dpprx_rst;
	reg		[31:0]	crc_dpprx_in;
	wire	[31:0]	crc_dpprx32_out;
	wire	[31:0]	crc_dpprx24_out;
	wire	[31:0]	crc_dpprx16_out;
	wire	[31:0]	crc_dpprx8_out;
usb3_crc_dpp32 iu3cdprx32 (
	.clk		( local_clk ),
	.rst		( crc_dpprx_rst ),
	.crc_en		( in_active_6 ),
	.di			( crc_dpprx_in ),
	.lfsr_q		( crc_dpprx_q ),
	.crc_out	( crc_dpprx32_out )
);
usb3_crc_dpp24 iu3cdprx24 (
	.di			( crc_dpprx_in[31:8] ),
	.q			( crc_dpprx_q ),
	.crc_out	( crc_dpprx24_out )
);
usb3_crc_dpp16 iu3cdprx16 (
	.di			( crc_dpprx_in[31:16] ),
	.q			( crc_dpprx_q ),
	.crc_out	( crc_dpprx16_out )
);
usb3_crc_dpp8 iu3cdprx8 (
	.di			( crc_dpprx_in[31:24] ),
	.q			( crc_dpprx_q ),
	.crc_out	( crc_dpprx8_out )
);

	wire	[31:0]	crc_dpptx_out = (out_dpp_length_remain_1 == 4) ? (swap32(crc_dpptx32_out)) :
									(out_dpp_length_remain_1 == 3) ? (swap32(crc_dpptx24_out)) :
									(out_dpp_length_remain_1 == 2) ? (swap32(crc_dpptx16_out)) :
									(out_dpp_length_remain_1 == 1) ? (swap32(crc_dpptx8_out)) : (swap32(crc_dpptx32_out));
	reg		[31:0]	crc_dpptx_out_1;
	wire	[31:0]	crc_dpptx_q;
	reg				crc_dpptx_rst;
	reg		[31:0]	crc_dpptx_in;
	wire	[31:0]	crc_dpptx32_out;
	wire	[31:0]	crc_dpptx24_out;
	wire	[31:0]	crc_dpptx16_out;
	wire	[31:0]	crc_dpptx8_out;
usb3_crc_dpp32 iu3cdptx32 (
	.clk		( local_clk ),
	.rst		( crc_dpptx_rst ),
	.crc_en		( 1'b1 ),
	.di			( crc_dpptx_in ),
	.lfsr_q		( crc_dpptx_q ),
	.crc_out	( crc_dpptx32_out )
);
usb3_crc_dpp24 iu3cdptx24 (
	.di			( crc_dpptx_in[23:0] ),
	.q			( crc_dpptx_q ),
	.crc_out	( crc_dpptx24_out )
);
usb3_crc_dpp16 iu3cdptx16 (
	.di			( crc_dpptx_in[15:0] ),
	.q			( crc_dpptx_q ),
	.crc_out	( crc_dpptx16_out )
);
usb3_crc_dpp8 iu3cdptx8 (
	.di			( crc_dpptx_in[7:0] ),
	.q			( crc_dpptx_q ),
	.crc_out	( crc_dpptx8_out )
);


//
// CRC-16 of Header Packet Headers
//
	reg				crc_hprx_rst;
	reg		[31:0]	crc_hprx_in;
	wire	[15:0]	crc_hprx_out;
usb3_crc_hp iu3chprx (
	.clk		( local_clk ),
	.rst		( crc_hprx_rst ),
	.crc_en		( in_active ),
	.di			( crc_hprx_in ),
	.crc_out	( crc_hprx_out )
);

	reg				crc_hptx_rst;
	wire	[31:0]	crc_hptx_in = swap32(out_data_2);
	wire	[15:0]	crc_hptx_out;
usb3_crc_hp iu3chptx (
	.clk		( local_clk ),
	.rst		( crc_hptx_rst ),
	.crc_en		( 1'b1 ),
	.di			( crc_hptx_in ),
	.crc_out	( crc_hptx_out )
);


//
// CRC-5 of Command Words
//

// RX
//
	wire	[10:0]	crc_cw1_in = in_link_command[26:16];
	wire	[4:0]	crc_cw1_out;
usb3_crc_cw iu3ccw1 (
	.di			( crc_cw1_in ),
	.crc_out	( crc_cw1_out )
);

	wire	[10:0]	crc_cw2_in = in_link_command[10:0];
	wire	[4:0]	crc_cw2_out;
usb3_crc_cw iu3ccw2 (
	.di			( crc_cw2_in ),
	.crc_out	( crc_cw2_out )
);

	wire	[10:0]	crc_cw3_in = in_header_cw[10:0];
	wire	[4:0]	crc_cw3_out;
usb3_crc_cw iu3ccw3 (
	.di			( crc_cw3_in ),
	.crc_out	( crc_cw3_out )
);

	wire	[10:0]	crc_cw4_in = out_header_cw[10:0];
	wire	[4:0]	crc_cw4_out;
usb3_crc_cw iu3ccw5 (
	.di			( crc_cw4_in ),
	.crc_out	( crc_cw4_out )
);

// TX
//
	reg		[10:0]	crc_lcmd_in;
	wire	[4:0]	crc_lcmd_out;
usb3_crc_cw iu3ccw4 (
	.di			( crc_lcmd_in ),
	.crc_out	( crc_lcmd_out )
);

endmodule
