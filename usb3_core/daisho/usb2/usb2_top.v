
//
// usb 2.0 top-level
//
// Copyright (c) 2012-2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb2_top (

input	wire			ext_clk,
input	wire			reset_n,
output	wire			reset_n_out,

input	wire			phy_ulpi_clk,
inout	wire	[7:0]	phy_ulpi_d,
input	wire			phy_ulpi_dir,
output	wire			phy_ulpi_stp,
input	wire			phy_ulpi_nxt,

input	wire			opt_disable_all,
input	wire			opt_enable_hs,
input	wire			opt_ignore_vbus,
output	wire			stat_connected,
output	wire			stat_fs,
output	wire			stat_hs,
output	wire			stat_configured,

input	wire	[8:0]	buf_in_addr,
input	wire	[7:0]	buf_in_data,
input	wire			buf_in_wren,
output	wire			buf_in_ready,
input	wire			buf_in_commit,
input	wire	[9:0]	buf_in_commit_len,
output	wire			buf_in_commit_ack,
input	wire	[8:0]	buf_out_addr,
output	wire	[7:0]	buf_out_q,
output	wire	[9:0]	buf_out_len,
output	wire			buf_out_hasdata,
input	wire			buf_out_arm,
output	wire			buf_out_arm_ack,

output	wire			vend_req_act,
output	wire	[7:0]	vend_req_request,
output	wire	[15:0]	vend_req_val,

output	wire			err_crc_pid,
output	wire			err_crc_tok,
output	wire			err_crc_pkt,
output	wire			err_pid_out_of_seq,
output	wire			err_setup_pkt,

output	wire	[10:0]	dbg_frame_num,
output	wire	[1:0]	dbg_linestate

);

	reg 			reset_1, reset_2;				// local reset
	
	// allow reset-time pin strapping for the usb 3.0 phy. 
	// this should not affect regular usb 2.0 PHYs
	//
	assign			phy_ulpi_d = (reset_2 ? (phy_ulpi_d_oe ? phy_ulpi_d_out : 8'bZZZZZZZZ) : 
					{
						1'b0,	// ISO_START	PIPE Isolate Mode
						1'b0,	// ULPI_8BIT	Bus Width ULPI
						2'b11,	// REFCLKSEL	Reference clock freq
						4'b0
					});
	
	wire	[7:0]	phy_ulpi_d_in = phy_ulpi_d;
	wire	[7:0]	phy_ulpi_d_out;
	wire			phy_ulpi_d_oe;
									
always @(posedge ext_clk) begin
	// synchronize external reset to local domain
	{reset_2, reset_1} <= {reset_1, reset_n};
end
									
////////////////////////////////////////////////////////////
//
// USB 2.0 ULPI interface
//
////////////////////////////////////////////////////////////

	wire			ulpi_out_act;
	wire	[7:0]	ulpi_out_byte;
	wire 			ulpi_out_latch;
	wire			ulpi_in_nxt;
	wire			ulpi_in_cts;
	wire	[7:0]	ulpi_in_byte;
	wire 			ulpi_in_latch;
	wire			ulpi_in_stp;

usb2_ulpi 	ia (
	// reset signal frome external clock domain, must be synchronized
	.reset_n		( reset_n ),
	
	// locally generated reset, triggers local USB reset when cable is 
	// unplugged, OR'd with external reset above
	.reset_local	( reset_n_out ),
	
	// easy flag that will cause all modules to stay in reset and be optimized out
	// useful for testing 3.0 only
	.opt_disable_all ( opt_disable_all ),
	
	// high-speed enable, full speed enumeration works but data transmission
	// is not guaranteed
	.opt_enable_hs	( opt_enable_hs ),
	
	// normally a change in Vbus signals that the device has been disconnected.
	// on some PHYs this may be unreliable (such as TUSB1310A)
	.opt_ignore_vbus (opt_ignore_vbus),
	
	// status signals
	.stat_connected	( stat_connected ),
	.stat_fs		( stat_fs ),
	.stat_hs		( stat_hs ),

	// external PHY interface
	.phy_clk		( phy_ulpi_clk ),
	.phy_d_in		( phy_ulpi_d_in ),
	.phy_d_out_mux	( phy_ulpi_d_out ),
	.phy_d_oe		( phy_ulpi_d_oe ),
	.phy_dir		( phy_ulpi_dir ),
	.phy_stp		( phy_ulpi_stp ),
	.phy_nxt		( phy_ulpi_nxt ),
	
	// packet layer interface
	.pkt_out_act	( ulpi_out_act ),
	.pkt_out_byte	( ulpi_out_byte ),
	.pkt_out_latch	( ulpi_out_latch ),
	.pkt_in_cts		( ulpi_in_cts ),
	.pkt_in_nxt		( ulpi_in_nxt ),
	.pkt_in_byte	( ulpi_in_byte ),
	.pkt_in_latch	( ulpi_in_latch ),
	.pkt_in_stp		( ulpi_in_stp ),
	
	.dbg_linestate 	( dbg_linestate )
);


////////////////////////////////////////////////////////////
//
// USB 2.0 Packet layer
//
////////////////////////////////////////////////////////////

	wire			packet_xfer_in;
	wire			packet_xfer_in_ok;
	wire			packet_xfer_out;
	wire			packet_xfer_out_ok;
	wire			packet_xfer_query;
	wire	[3:0]	packet_xfer_endp;
	wire	[3:0]	packet_xfer_pid;
	
usb2_packet ip (
	// note this is the locally generated reset driven by the ULPI module
	.reset_n			( reset_n_out ),
	
	// clock (60mhz) driven by external PHY
	.phy_clk			( phy_ulpi_clk ),
	
	// connections to ULPI module
	.in_act				( ulpi_out_act ),
	.in_byte			( ulpi_out_byte ),
	.in_latch			( ulpi_out_latch ),
	.out_cts			( ulpi_in_cts ),
	.out_nxt			( ulpi_in_nxt ),
	.out_byte			( ulpi_in_byte ),
	.out_latch			( ulpi_in_latch ),
	.out_stp			( ulpi_in_stp ),
	
	// currently selected endpoint buffer
	.sel_endp			( prot_sel_endp ),
	
	.buf_in_addr		( prot_buf_in_addr ),
	.buf_in_data		( prot_buf_in_data ),
	.buf_in_wren		( prot_buf_in_wren ),
	.buf_in_ready		( prot_buf_in_ready ),
	.buf_in_commit		( prot_buf_in_commit ),
	.buf_in_commit_len	( prot_buf_in_commit_len ),
	.buf_in_commit_ack	( prot_buf_in_commit_ack ),
	
	.buf_out_addr		( prot_buf_out_addr ),
	.buf_out_q			( prot_buf_out_q ),
	.buf_out_len		( prot_buf_out_len ),
	.buf_out_hasdata	( prot_buf_out_hasdata ),
	.buf_out_arm		( prot_buf_out_arm ),
	.buf_out_arm_ack	( prot_buf_out_arm_ack ),
	
	.endp_mode			( prot_endp_mode ),
	
	.data_toggle_act	( prot_data_toggle_act ),
	.data_toggle		( prot_data_toggle ),
	
	// current device address, driven by endpoint 0
	.dev_addr			( prot_dev_addr ),
	
	// error signals, put your LA on these
	.err_crc_pid		( err_crc_pid ),
	.err_crc_tok		( err_crc_tok ),
	.err_crc_pkt		( err_crc_pkt ),
	.err_pid_out_of_seq ( err_pid_out_of_seq ),
	
	.dbg_frame_num	( dbg_frame_num )
);


////////////////////////////////////////////////////////////
//
// USB 2.0 Protocol layer
//
////////////////////////////////////////////////////////////

	wire	[3:0]	prot_sel_endp;
	wire	[8:0]	prot_buf_in_addr;
	wire	[7:0]	prot_buf_in_data;
	wire			prot_buf_in_wren;
	wire			prot_buf_in_ready;
	wire			prot_buf_in_commit;
	wire	[9:0]	prot_buf_in_commit_len;
	wire			prot_buf_in_commit_ack;

	wire	[8:0]	prot_buf_out_addr;
	wire	[7:0]	prot_buf_out_q;
	wire	[9:0]	prot_buf_out_len;
	wire			prot_buf_out_hasdata;
	wire			prot_buf_out_arm;
	wire			prot_buf_out_arm_ack;
	wire	[6:0]	prot_dev_addr;
	
	wire	[1:0]	prot_endp_mode;
	wire			prot_data_toggle_act;
	wire	[1:0]	prot_data_toggle;
	
usb2_protocol ipr (
	.reset_n			( reset_n_out ),
	
	// external interface clock. the dual-port endpoint block rams
	// have one port clocked with the ULPI, and the other with this
	// externally provided clock.
	.ext_clk			( ext_clk ),
	.phy_clk			( phy_ulpi_clk ),
	
	// muxed endpoint signals
	.sel_endp			( prot_sel_endp ),
	
	.buf_in_addr		( prot_buf_in_addr ),
	.buf_in_data		( prot_buf_in_data ),
	.buf_in_wren		( prot_buf_in_wren ),
	.buf_in_ready		( prot_buf_in_ready ),
	.buf_in_commit		( prot_buf_in_commit ),
	.buf_in_commit_len	( prot_buf_in_commit_len ),
	.buf_in_commit_ack	( prot_buf_in_commit_ack ),
	
	.buf_out_addr		( prot_buf_out_addr ),
	.buf_out_q			( prot_buf_out_q ),
	.buf_out_len		( prot_buf_out_len ),
	.buf_out_hasdata	( prot_buf_out_hasdata ),
	.buf_out_arm		( prot_buf_out_arm ),
	.buf_out_arm_ack	( prot_buf_out_arm_ack ),
	
	// external interface
	.ext_buf_in_addr		( buf_in_addr ),
	.ext_buf_in_data		( buf_in_data ),
	.ext_buf_in_wren		( buf_in_wren ),
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

	.vend_req_act		( vend_req_act ),
	.vend_req_request	( vend_req_request ),
	.vend_req_val		( vend_req_val ),
	
	.endp_mode			( prot_endp_mode ),
	
	.data_toggle_act	( prot_data_toggle_act ),
	.data_toggle		( prot_data_toggle ),
	
	.err_setup_pkt		( err_setup_pkt ),
	
	// tell the rest of the USB controller about what
	// our current device address is, assigned by host
	.dev_addr			( prot_dev_addr ),
	.configured			( stat_configured )
);


endmodule
