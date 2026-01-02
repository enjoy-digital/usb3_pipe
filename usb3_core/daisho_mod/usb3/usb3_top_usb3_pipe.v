
//
// usb 3.0 top-level
//
// Copyright (c) 2013 Marshall H.
// Copyright (c) 2019 Florent Kermarrec
// All rights reserved.
// This code is released under the terms of the simplified BSD license.
// See LICENSE.TXT for details.
//

module usb3_top_usb3_pipe (

input	wire			clk,
input	wire			reset_n,

input	wire	[4:0]	ltssm_state,

input wire		[31:0]	in_data,
input wire		[3:0]	in_datak,
input wire				in_active,

output wire		[31:0]	out_data,
output wire		[3:0]	out_datak,
output wire				out_active,
input  wire				out_stall,

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
output	wire	[15:0]	vend_req_val
);

`include "usb3_const.vh"

////////////////////////////////////////////////////////////
//
// USB 3.0 Link layer interface
//
////////////////////////////////////////////////////////////

usb3_link iu3l (

	.local_clk				( clk ),
	.reset_n				( reset_n ),

	.ltssm_state			( ltssm_state ),
	.ltssm_hot_reset		( 1'b0 ),
	.ltssm_go_disabled		( ),
	.ltssm_go_recovery		( ),
	.ltssm_go_u				( ),
	.in_data				( in_data ),
	.in_datak				( in_datak ),
	.in_active				( in_active ),

	.outp_data				( out_data ),
	.outp_datak				( out_datak ),
	.outp_active			( out_active ),
	.out_stall				( out_stall ),

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

	.local_clk				( clk ), // FIXME ?
	.slow_clk				( clk ), // FIXME ?
	.ext_clk				( clk ), // FIXME ?

	.reset_n				( reset_n),
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
