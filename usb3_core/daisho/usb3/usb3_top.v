
//
// usb 3.0 top-level
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license.
// See LICENSE.TXT for details.
//

module usb3_top (

input	wire			ext_clk,
input	wire			reset_n,

input	wire			phy_pipe_half_clk,        // 125MHz:  1/2 PCLK
input	wire			phy_pipe_half_clk_phase,  // 125MHz:  1/2 PCLK, phase shift 90
input	wire			phy_pipe_quarter_clk,     // 62.5MHz: 1/4 PCLK
input	wire	[31:0]	phy_pipe_rx_data,
input	wire	[3:0]	phy_pipe_rx_datak,
input	wire	[1:0]	phy_pipe_rx_valid,
output	wire	[31:0]	phy_pipe_tx_data,
output	wire	[3:0]	phy_pipe_tx_datak,

output	wire			phy_reset_n,
output	wire			phy_out_enable,
output	wire			phy_phy_reset_n,
output	wire			phy_tx_detrx_lpbk,
output	wire			phy_tx_elecidle,
inout	wire			phy_rx_elecidle,
input	wire	[5:0]	phy_rx_status,
output	wire	[1:0]	phy_power_down,
input   wire	[1:0]	phy_phy_status_i,
output	wire	        phy_phy_status_o,
input	wire			phy_pwrpresent,

output	wire			phy_tx_oneszeros,
output	wire	[1:0]	phy_tx_deemph,
output	wire	[2:0]	phy_tx_margin,
output	wire			phy_tx_swing,
output	wire			phy_rx_polarity,
output	wire			phy_rx_termination,
output	wire			phy_rate,
output	wire			phy_elas_buf_mode,

input	wire	[8:0]	buf_in_addr,
input	wire	[31:0]	buf_in_data,
input	wire			buf_in_wren,
output	wire			buf_in_request,
output	wire			buf_in_ready,
input	wire			buf_in_commit,
input	wire	[10:0]	buf_in_commit_len,
output	wire			buf_in_commit_ack,

input	wire	[8:0]	buf_out_addr,
output	wire	[31:0]	buf_out_q,
output	wire	[10:0]	buf_out_len,
output	wire			buf_out_hasdata,
input	wire			buf_out_arm,
output	wire			buf_out_arm_ack,

output	wire			vend_req_act,
output	wire	[7:0]	vend_req_request,
output	wire	[15:0]	vend_req_val,
output  wire    [5:0] dbg_pipe_state,
output  wire    [4:0] dbg_ltssm_state
);

	reg 			reset_1, reset_2;				// local reset
	wire			local_reset		= reset_n & phy_pwrpresent;

always @(posedge phy_pipe_half_clk ) begin

	// synchronize external reset to local domain
	{reset_2, reset_1} <= {reset_1, local_reset};

end

	assign			phy_reset_n = reset_n;			// TUSB1310A has minimum 1uS pulse width for RESET
													// responsibility of the toplevel module to supply this reset
													// NOTE: reset entire phy will cause loss of PLL lock
	assign			phy_phy_reset_n = reset_n & phy_pwrpresent;
													// reset the PHY along with all our core code if cable unplugged
	assign			phy_out_enable = 1'b1;

	wire	[1:0]	mux_tx_margin;

	parameter		XTAL_SEL			= 1'b0; 	// crystal input
	parameter		OSC_SEL				= 1'b1; 	// clock input
	parameter [2:0]	SSC_DIS				= 2'b11;	// spread spectrum clock disable
	parameter [2:0]	SSC_EN				= 2'b00;	// spread spectrum clock enable
	parameter		PIPE_16BIT			= 1'b0;		// sdr 16bit pipe interface
	// strap pins
	assign			phy_rx_elecidle 	= reset_2 ? 1'bZ : XTAL_SEL;
	assign			phy_tx_margin	  	= reset_2 ? mux_tx_margin  : SSC_DIS;
	assign			phy_phy_status_o 	= reset_2 ? 1'bZ : PIPE_16BIT;

////////////////////////////////////////////////////////////
//
// USB 3.0 PIPE3 interface
//
////////////////////////////////////////////////////////////

	wire		ltssm_reset_n;

usb3_pipe	iu3p (

	.slow_clk				( phy_pipe_quarter_clk ),
	.local_clk				( phy_pipe_half_clk ),
	.local_clk_capture		( phy_pipe_half_clk_phase ),
	.reset_n				( reset_2 ),
	.ltssm_reset_n			( ltssm_reset_n ),

	.phy_pipe_rx_data		( phy_pipe_rx_data ),
	.phy_pipe_rx_datak		( phy_pipe_rx_datak	 ),
	.phy_pipe_rx_valid		( phy_pipe_rx_valid ),
	.phy_pipe_tx_data		( phy_pipe_tx_data ),
	.phy_pipe_tx_datak		( phy_pipe_tx_datak ),

	.phy_tx_detrx_lpbk		( phy_tx_detrx_lpbk ),
	.phy_tx_elecidle		( phy_tx_elecidle ),
	.phy_rx_elecidle		( phy_rx_elecidle ),
	.phy_rx_status			( phy_rx_status ),
	.phy_power_down			( phy_power_down ),
	.phy_phy_status			( phy_phy_status_i ),
	.phy_pwrpresent			( phy_pwrpresent ),

	.phy_tx_oneszeros		( phy_tx_oneszeros ),
	.phy_tx_deemph			( phy_tx_deemph ),
	.phy_tx_margin			( mux_tx_margin ),
	.phy_tx_swing			( phy_tx_swing ),
	.phy_rx_polarity		( phy_rx_polarity ),
	.phy_rx_termination		( phy_rx_termination ),
	.phy_rate				( phy_rate ),
	.phy_elas_buf_mode		( phy_elas_buf_mode ),

	.link_in_data			( link_in_data ),
	.link_in_datak			( link_in_datak ),
	.link_in_active			( link_in_active ),
	.link_out_data			( link_out_data ),
	.link_out_datak			( link_out_datak ),
	.link_out_active		( link_out_active ),
	.link_out_stall			( link_out_stall ),

	.partner_detect			( partner_detect ),
	.partner_looking		( partner_looking ),
	.partner_detected		( partner_detected ),

	.ltssm_tx_detrx_lpbk	( port_tx_detrx_lpbk ),
	.ltssm_tx_elecidle		( port_tx_elecidle ),
	.ltssm_power_down		( port_power_down ),
	.ltssm_power_go			( port_power_go ),
	.ltssm_power_ack		( port_power_ack ),
	.ltssm_hot_reset		( ltssm_hot_reset ),

	.ltssm_state				( ltssm_state ),
	.ltssm_training				( ltssm_training ),
	.ltssm_train_rxeq			( ltssm_train_rxeq ),
	.ltssm_train_rxeq_pass		( ltssm_train_rxeq_pass ),
	.ltssm_train_active			( ltssm_train_active ),
	.ltssm_train_ts1			( ltssm_train_ts1 ),
	.ltssm_train_ts2			( ltssm_train_ts2 ),
	.ltssm_train_config			( ltssm_train_config ),
	.ltssm_train_idle			( ltssm_train_idle ),
	.ltssm_train_idle_pass		( ltssm_train_idle_pass ),

	.lfps_recv_active		( lfps_recv_active ),
	.lfps_recv_poll_u1		( lfps_recv_poll_u1 ),
	.lfps_recv_ping			( lfps_recv_ping ),
	.lfps_recv_reset		( lfps_recv_reset ),
	.lfps_recv_u2lb			( lfps_recv_u2lb ),
	.lfps_recv_u3			( lfps_recv_u3 ),
	.dbg_state              ( dbg_pipe_state )

);

////////////////////////////////////////////////////////////
//
// USB 3.0 LTSSM, LFPS
//
////////////////////////////////////////////////////////////

	wire	[4:0]	ltssm_state;
	wire			port_rx_term;
	wire			port_tx_detrx_lpbk;
	wire			port_tx_elecidle;

	wire	[1:0]	port_power_down;
	wire			port_power_go;
	wire			port_power_ack;
	wire			port_power_err;
	wire			ltssm_hot_reset;

	wire			ltssm_training;
	wire			ltssm_train_rxeq;
	wire			ltssm_train_rxeq_pass;
	wire			ltssm_train_active;
	wire			ltssm_train_ts1;
	wire			ltssm_train_ts2;
	wire			ltssm_train_config;
	wire			ltssm_train_idle;
	wire			ltssm_train_idle_pass;

	wire			partner_detect;
	wire			partner_looking;
	wire			partner_detected;

	wire			lfps_recv_active;
	wire			lfps_recv_poll_u1;
	wire			lfps_recv_ping;
	wire			lfps_recv_reset;
	wire			lfps_recv_u2lb;
	wire			lfps_recv_u3;

	wire			ltssm_warm_reset;

usb3_ltssm	iu3lt (

	.slow_clk				( phy_pipe_quarter_clk ),
	.local_clk				( phy_pipe_half_clk ),
	.reset_n				( ltssm_reset_n ),

	// inputs
	.vbus_present			( phy_pwrpresent ),
	.port_rx_valid			( phy_pipe_rx_valid ),	// these signals are in the 250mhz source
	.port_rx_elecidle		( phy_rx_elecidle ),	// domain, but no problem for lfps in 62.5mhz
	.partner_looking		( partner_looking ),
	.partner_detected		( partner_detected ),
	.port_power_state		( phy_power_down ),		// reflect actual value driven by PIPE pd_fsm
	.port_power_ack			( port_power_ack ),
	.port_power_err			( port_power_err ),

	.train_rxeq_pass		( ltssm_train_rxeq_pass ),
	.train_idle_pass		( ltssm_train_idle_pass ),
	.train_ts1				( ltssm_train_ts1 ),
	.train_ts2				( ltssm_train_ts2 ),
	.go_disabled			( ltssm_go_disabled ),
	.go_recovery			( ltssm_go_recovery ),
	.go_u					( ltssm_go_u ),
	.hot_reset				( ltssm_hot_reset ),

	// outputs
	.ltssm_state			( ltssm_state ),
	.port_rx_term			( port_rx_term ),
	.port_tx_detrx_lpbk		( port_tx_detrx_lpbk ),
	.port_tx_elecidle		( port_tx_elecidle ),
	.port_power_down		( port_power_down ),
	.port_power_go			( port_power_go ),
	.partner_detect			( partner_detect ),

	.training				( ltssm_training ),
	.train_rxeq				( ltssm_train_rxeq ),
	.train_active			( ltssm_train_active ),
	.train_config			( ltssm_train_config ),
	.train_idle				( ltssm_train_idle ),

	.lfps_recv_active		( lfps_recv_active ),
	.lfps_recv_poll_u1		( lfps_recv_poll_u1 ),
	.lfps_recv_ping			( lfps_recv_ping ),
	.lfps_recv_reset		( lfps_recv_reset ),
	.lfps_recv_u2lb			( lfps_recv_u2lb ),
	.lfps_recv_u3			( lfps_recv_u3 ),

	.warm_reset				( ltssm_warm_reset ),
	.dbg_state              ( dbg_ltssm_state )
);


////////////////////////////////////////////////////////////
//
// USB 3.0 Link layer interface
//
////////////////////////////////////////////////////////////

	wire		[31:0]	link_in_data;
	wire		[3:0]	link_in_datak;
	wire				link_in_active;
	wire		[31:0]	link_out_data;
	wire		[3:0]	link_out_datak;
	wire				link_out_active;
	wire				link_out_stall;
	wire				ltssm_go_disabled;
	wire				ltssm_go_recovery;
	wire		[2:0]	ltssm_go_u;

usb3_link iu3l (

	.local_clk				( phy_pipe_half_clk ),
	.reset_n				( reset_2 ),

	.ltssm_state			( ltssm_state ),
	.ltssm_hot_reset		( ltssm_hot_reset ),
	.ltssm_go_disabled		( ltssm_go_disabled ),
	.ltssm_go_recovery		( ltssm_go_recovery ),
	.ltssm_go_u				( ltssm_go_u),
	.in_data				( link_in_data ),
	.in_datak				( link_in_datak ),
	.in_active				( link_in_active ),

	.outp_data				( link_out_data ),
	.outp_datak				( link_out_datak ),
	.outp_active			( link_out_active ),
	.out_stall				( link_out_stall ),

	.endp_mode_rx			( prot_endp_mode_rx ),
	.endp_mode_tx			( prot_endp_mode_tx ),

	.prot_rx_tp				( prot_rx_tp ),
	.prot_rx_tp_hosterr		( prot_rx_tp_hosterr ),
	.prot_rx_tp_retry		( prot_rx_tp_retry ),
	.prot_rx_tp_pktpend		( prot_rx_tp_pktpend ),
	.prot_rx_tp_subtype		( prot_rx_tp_subtype ),
	.prot_rx_tp_endp		( prot_rx_tp_endp ),
	.prot_rx_tp_nump		( prot_rx_tp_nump ),
	.prot_rx_tp_seq			( prot_rx_tp_seq ),
	.prot_rx_tp_stream		( prot_rx_tp_stream ),

	.prot_rx_dph			( prot_rx_dph ),
	.prot_rx_dph_eob		( prot_rx_dph_eob ),
	.prot_rx_dph_setup		( prot_rx_dph_setup ),
	.prot_rx_dph_pktpend	( prot_rx_dph_pktpend ),
	.prot_rx_dph_endp		( prot_rx_dph_endp ),
	.prot_rx_dph_seq		( prot_rx_dph_seq ),
	.prot_rx_dph_len		( prot_rx_dph_len ),
	.prot_rx_dpp_start		( prot_rx_dpp_start ),
	.prot_rx_dpp_done		( prot_rx_dpp_done ),
	.prot_rx_dpp_crcgood	( prot_rx_dpp_crcgood ),

	.prot_tx_tp_a			( prot_tx_tp_a ),
	.prot_tx_tp_a_retry		( prot_tx_tp_a_retry ),
	.prot_tx_tp_a_dir		( prot_tx_tp_a_dir ),
	.prot_tx_tp_a_subtype	( prot_tx_tp_a_subtype ),
	.prot_tx_tp_a_endp		( prot_tx_tp_a_endp ),
	.prot_tx_tp_a_nump		( prot_tx_tp_a_nump ),
	.prot_tx_tp_a_seq		( prot_tx_tp_a_seq ),
	.prot_tx_tp_a_stream	( prot_tx_tp_a_stream ),
	.prot_tx_tp_a_ack		( prot_tx_tp_a_ack ),

	.prot_tx_tp_b			( prot_tx_tp_b ),
	.prot_tx_tp_b_retry		( prot_tx_tp_b_retry ),
	.prot_tx_tp_b_dir		( prot_tx_tp_b_dir ),
	.prot_tx_tp_b_subtype	( prot_tx_tp_b_subtype ),
	.prot_tx_tp_b_endp		( prot_tx_tp_b_endp ),
	.prot_tx_tp_b_nump		( prot_tx_tp_b_nump ),
	.prot_tx_tp_b_seq		( prot_tx_tp_b_seq ),
	.prot_tx_tp_b_stream	( prot_tx_tp_b_stream ),
	.prot_tx_tp_b_ack		( prot_tx_tp_b_ack ),

	.prot_tx_tp_c			( prot_tx_tp_c ),
	.prot_tx_tp_c_retry		( prot_tx_tp_c_retry ),
	.prot_tx_tp_c_dir		( prot_tx_tp_c_dir ),
	.prot_tx_tp_c_subtype	( prot_tx_tp_c_subtype ),
	.prot_tx_tp_c_endp		( prot_tx_tp_c_endp ),
	.prot_tx_tp_c_nump		( prot_tx_tp_c_nump ),
	.prot_tx_tp_c_seq		( prot_tx_tp_c_seq ),
	.prot_tx_tp_c_stream	( prot_tx_tp_c_stream ),
	.prot_tx_tp_c_ack		( prot_tx_tp_c_ack ),

	.prot_tx_dph			( prot_tx_dph ),
	.prot_tx_dph_eob		( prot_tx_dph_eob ),
	.prot_tx_dph_dir		( prot_tx_dph_dir ),
	.prot_tx_dph_endp		( prot_tx_dph_endp ),
	.prot_tx_dph_seq		( prot_tx_dph_seq ),
	.prot_tx_dph_len		( prot_tx_dph_len ),
	.prot_tx_dpp_ack		( prot_tx_dpp_ack ),
	.prot_tx_dpp_done		( prot_tx_dpp_done ),

	.buf_in_addr			( prot_buf_in_addr ),
	.buf_in_data			( prot_buf_in_data ),
	.buf_in_wren			( prot_buf_in_wren ),
	.buf_in_ready			( prot_buf_in_ready ),
	.buf_in_commit			( prot_buf_in_commit ),
	.buf_in_commit_len		( prot_buf_in_commit_len ),
	.buf_in_commit_ack		( prot_buf_in_commit_ack ),

	.buf_out_addr			( prot_buf_out_addr ),
	.buf_out_q				( prot_buf_out_q ),
	.buf_out_len			( prot_buf_out_len ),
	.buf_out_hasdata		( prot_buf_out_hasdata ),
	.buf_out_arm			( prot_buf_out_arm ),
	.buf_out_arm_ack		( prot_buf_out_arm_ack ),

	// current device address, driven by endpoint 0
	.dev_addr				( prot_dev_addr )
);



////////////////////////////////////////////////////////////
//
// USB 3.0 Protocol layer interface
//
////////////////////////////////////////////////////////////

	//wire	[31:0]	prot_in_data;
	//wire	[3:0]	prot_in_datak;
	//wire			prot_in_active;

	wire	[1:0]	prot_endp_mode_rx;
	wire	[1:0]	prot_endp_mode_tx;
	wire	[6:0]	prot_dev_addr;
	wire			prot_configured;

	wire			prot_rx_tp;
	wire			prot_rx_tp_hosterr;
	wire			prot_rx_tp_retry;
	wire			prot_rx_tp_pktpend;
	wire	[3:0]	prot_rx_tp_subtype;
	wire	[3:0]	prot_rx_tp_endp;
	wire	[4:0]	prot_rx_tp_nump;
	wire	[4:0]	prot_rx_tp_seq;
	wire	[15:0]	prot_rx_tp_stream;

	wire			prot_rx_dph;
	wire			prot_rx_dph_eob;
	wire			prot_rx_dph_setup;
	wire			prot_rx_dph_pktpend;
	wire	[3:0]	prot_rx_dph_endp;
	wire	[4:0]	prot_rx_dph_seq;
	wire	[15:0]	prot_rx_dph_len;
	wire			prot_rx_dpp_start;
	wire			prot_rx_dpp_done;
	wire			prot_rx_dpp_crcgood;

	wire			prot_tx_tp_a;
	wire			prot_tx_tp_a_retry;
	wire			prot_tx_tp_a_dir;
	wire	[3:0]	prot_tx_tp_a_subtype;
	wire	[3:0]	prot_tx_tp_a_endp;
	wire	[4:0]	prot_tx_tp_a_nump;
	wire	[4:0]	prot_tx_tp_a_seq;
	wire	[15:0]	prot_tx_tp_a_stream;
	wire			prot_tx_tp_a_ack;

	wire			prot_tx_tp_b;
	wire			prot_tx_tp_b_retry;
	wire			prot_tx_tp_b_dir;
	wire	[3:0]	prot_tx_tp_b_subtype;
	wire	[3:0]	prot_tx_tp_b_endp;
	wire	[4:0]	prot_tx_tp_b_nump;
	wire	[4:0]	prot_tx_tp_b_seq;
	wire	[15:0]	prot_tx_tp_b_stream;
	wire			prot_tx_tp_b_ack;

	wire			prot_tx_tp_c;
	wire			prot_tx_tp_c_retry;
	wire			prot_tx_tp_c_dir;
	wire	[3:0]	prot_tx_tp_c_subtype;
	wire	[3:0]	prot_tx_tp_c_endp;
	wire	[4:0]	prot_tx_tp_c_nump;
	wire	[4:0]	prot_tx_tp_c_seq;
	wire	[15:0]	prot_tx_tp_c_stream;
	wire			prot_tx_tp_c_ack;

	wire			prot_tx_dph;
	wire			prot_tx_dph_eob;
	wire			prot_tx_dph_dir;
	wire	[3:0]	prot_tx_dph_endp;
	wire	[4:0]	prot_tx_dph_seq;
	wire	[15:0]	prot_tx_dph_len;
	wire			prot_tx_dpp_ack;
	wire			prot_tx_dpp_done;


	wire	[8:0]	prot_buf_in_addr;
	wire	[31:0]	prot_buf_in_data;
	wire			prot_buf_in_wren;
	wire			prot_buf_in_ready;
	wire			prot_buf_in_commit;
	wire	[10:0]	prot_buf_in_commit_len;
	wire			prot_buf_in_commit_ack;

	wire	[8:0]	prot_buf_out_addr;
	wire	[31:0]	prot_buf_out_q;
	wire	[10:0]	prot_buf_out_len;
	wire			prot_buf_out_hasdata;
	wire			prot_buf_out_arm;
	wire			prot_buf_out_arm_ack;



usb3_protocol iu3r (

	.local_clk				( phy_pipe_half_clk ),
	.slow_clk				( phy_pipe_quarter_clk ),
	.ext_clk				( ext_clk ),

	.reset_n				( reset_2 | ~ltssm_warm_reset),
	.ltssm_state			( ltssm_state ),

	// muxed endpoint signals
	.endp_mode_rx			( prot_endp_mode_rx ),
	.endp_mode_tx			( prot_endp_mode_tx ),

	.rx_tp					( prot_rx_tp ),
	.rx_tp_hosterr			( prot_rx_tp_hosterr ),
	.rx_tp_retry			( prot_rx_tp_retry ),
	.rx_tp_pktpend			( prot_rx_tp_pktpend ),
	.rx_tp_subtype			( prot_rx_tp_subtype ),
	.rx_tp_endp				( prot_rx_tp_endp ),
	.rx_tp_nump				( prot_rx_tp_nump ),
	.rx_tp_seq				( prot_rx_tp_seq ),
	.rx_tp_stream			( prot_rx_tp_stream ),

	.rx_dph					( prot_rx_dph ),
	.rx_dph_eob				( prot_rx_dph_eob ),
	.rx_dph_setup			( prot_rx_dph_setup ),
	.rx_dph_pktpend			( prot_rx_dph_pktpend ),
	.rx_dph_endp			( prot_rx_dph_endp ),
	.rx_dph_seq				( prot_rx_dph_seq ),
	.rx_dph_len				( prot_rx_dph_len ),
	.rx_dpp_start			( prot_rx_dpp_start ),
	.rx_dpp_done			( prot_rx_dpp_done ),
	.rx_dpp_crcgood			( prot_rx_dpp_crcgood ),

	.tx_tp_a				( prot_tx_tp_a ),
	.tx_tp_a_retry			( prot_tx_tp_a_retry ),
	.tx_tp_a_dir			( prot_tx_tp_a_dir ),
	.tx_tp_a_subtype		( prot_tx_tp_a_subtype ),
	.tx_tp_a_endp			( prot_tx_tp_a_endp ),
	.tx_tp_a_nump			( prot_tx_tp_a_nump ),
	.tx_tp_a_seq			( prot_tx_tp_a_seq ),
	.tx_tp_a_stream			( prot_tx_tp_a_stream ),
	.tx_tp_a_ack			( prot_tx_tp_a_ack ),

	.tx_tp_b				( prot_tx_tp_b ),
	.tx_tp_b_retry			( prot_tx_tp_b_retry ),
	.tx_tp_b_dir			( prot_tx_tp_b_dir ),
	.tx_tp_b_subtype		( prot_tx_tp_b_subtype ),
	.tx_tp_b_endp			( prot_tx_tp_b_endp ),
	.tx_tp_b_nump			( prot_tx_tp_b_nump ),
	.tx_tp_b_seq			( prot_tx_tp_b_seq ),
	.tx_tp_b_stream			( prot_tx_tp_b_stream ),
	.tx_tp_b_ack			( prot_tx_tp_b_ack ),

	.tx_tp_c				( prot_tx_tp_c ),
	.tx_tp_c_retry			( prot_tx_tp_c_retry ),
	.tx_tp_c_dir			( prot_tx_tp_c_dir ),
	.tx_tp_c_subtype		( prot_tx_tp_c_subtype ),
	.tx_tp_c_endp			( prot_tx_tp_c_endp ),
	.tx_tp_c_nump			( prot_tx_tp_c_nump ),
	.tx_tp_c_seq			( prot_tx_tp_c_seq ),
	.tx_tp_c_stream			( prot_tx_tp_c_stream ),
	.tx_tp_c_ack			( prot_tx_tp_c_ack ),

	.tx_dph					( prot_tx_dph ),
	.tx_dph_eob				( prot_tx_dph_eob ),
	.tx_dph_dir				( prot_tx_dph_dir ),
	.tx_dph_endp			( prot_tx_dph_endp ),
	.tx_dph_seq				( prot_tx_dph_seq ),
	.tx_dph_len				( prot_tx_dph_len ),
	.tx_dpp_ack				( prot_tx_dpp_ack ),
	.tx_dpp_done			( prot_tx_dpp_done ),

	.buf_in_addr			( prot_buf_in_addr ),
	.buf_in_data			( prot_buf_in_data ),
	.buf_in_wren			( prot_buf_in_wren ),
	.buf_in_ready			( prot_buf_in_ready ),
	.buf_in_commit			( prot_buf_in_commit ),
	.buf_in_commit_len		( prot_buf_in_commit_len ),
	.buf_in_commit_ack		( prot_buf_in_commit_ack ),

	.buf_out_addr			( prot_buf_out_addr ),
	.buf_out_q				( prot_buf_out_q ),
	.buf_out_len			( prot_buf_out_len ),
	.buf_out_hasdata		( prot_buf_out_hasdata ),
	.buf_out_arm			( prot_buf_out_arm ),
	.buf_out_arm_ack		( prot_buf_out_arm_ack ),

	// external interface

	.ext_buf_in_addr		( buf_in_addr ),
	.ext_buf_in_data		( buf_in_data ),
	.ext_buf_in_wren		( buf_in_wren ),
	.ext_buf_in_request		( buf_in_request ),
	.ext_buf_in_ready		( buf_in_ready ),
	.ext_buf_in_commit		( buf_in_commit ),
	.ext_buf_in_commit_len	( buf_in_commit_len ),
	.ext_buf_in_commit_ack	( buf_in_commit_ack ),

	.ext_buf_out_addr		( buf_out_addr ),
	.ext_buf_out_q			( buf_out_q ),
	.ext_buf_out_len		( buf_out_len ),
	.ext_buf_out_hasdata	( buf_out_hasdata ),
	.ext_buf_out_arm		( buf_out_arm ),
	.ext_buf_out_arm_ack	( buf_out_arm_ack ),

	.vend_req_act			( vend_req_act ),
	.vend_req_request		( vend_req_request ),
	.vend_req_val			( vend_req_val ),

	// tell the rest of the USB controller about what
	// our current device address is, assigned by host
	.dev_addr				( prot_dev_addr ),
	.configured				( prot_configured )
);

endmodule
