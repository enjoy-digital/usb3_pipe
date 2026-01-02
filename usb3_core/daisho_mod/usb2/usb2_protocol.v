
//
// usb 2.0 protocol layer
//
// Copyright (c) 2012-2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb2_protocol (

// top-level interface
input	wire			phy_clk,
input	wire			ext_clk,
input	wire			reset_n,

// PACKET
input	wire	[3:0]	sel_endp,

input	wire	[8:0]	buf_in_addr,
input	wire	[7:0]	buf_in_data,
input	wire			buf_in_wren,
output	wire			buf_in_ready,
input	wire			buf_in_commit,
input	wire	[9:0]	buf_in_commit_len,
output  wire            buf_in_commit_ack,    

input	wire	[8:0]	buf_out_addr,
output  wire    [7:0]   buf_out_q, 
output	wire	[9:0]	buf_out_len,
output	wire			buf_out_hasdata,
input	wire			buf_out_arm,
output	wire			buf_out_arm_ack,
	
// EXTERNAL
input	wire	[8:0]	ext_buf_in_addr,
input	wire	[7:0]	ext_buf_in_data,
input	wire			ext_buf_in_wren,
output	wire			ext_buf_in_ready,
input	wire			ext_buf_in_commit,
input	wire	[9:0]	ext_buf_in_commit_len,
output	wire			ext_buf_in_commit_ack,
input	wire	[8:0]	ext_buf_out_addr,
output	wire	[7:0]	ext_buf_out_q,
output	wire	[9:0]	ext_buf_out_len,
output	wire			ext_buf_out_hasdata,
input	wire			ext_buf_out_arm,
output	wire			ext_buf_out_arm_ack,

output	wire	[1:0]	endp_mode,

input	wire			data_toggle_act,
output	wire	[1:0]	data_toggle,

output	wire			vend_req_act,
output	wire	[7:0]	vend_req_request,
output	wire	[15:0]	vend_req_val,

output	wire	[6:0]	dev_addr,
output	wire			configured,

output	wire			err_setup_pkt

);

	reg 			reset_1, reset_2;
	
	// mux bram signals
	wire	[5:0]	ep0_buf_in_addr		= 	sel_endp == SEL_ENDP0 ? buf_in_addr[5:0] : 6'h0;
	wire	[7:0]	ep0_buf_in_data		= 	sel_endp == SEL_ENDP0 ? buf_in_data : 8'h0;
	wire			ep0_buf_in_wren		= 	sel_endp == SEL_ENDP0 ? buf_in_wren : 1'h0;
	wire			ep0_buf_in_ready;
	wire			ep0_buf_in_commit	= 	sel_endp == SEL_ENDP0 ? buf_in_commit : 1'h0;
	wire	[9:0]	ep0_buf_in_commit_len = sel_endp == SEL_ENDP0 ? buf_in_commit_len : 10'h0;
	wire			ep0_buf_in_commit_ack;
	wire			ep0_data_toggle_act	= 	sel_endp == SEL_ENDP0 ? data_toggle_act : 1'h0;
	wire	[1:0]	ep0_data_toggle;
	
	wire	[7:0]	ep0_buf_out_addr	= 	sel_endp == SEL_ENDP0 ? buf_out_addr[7:0] : 7'h0;
	wire	[7:0]	ep0_buf_out_q;
	wire	[9:0]	ep0_buf_out_len;
	wire			ep0_buf_out_hasdata;
	wire			ep0_buf_out_arm		= 	sel_endp == SEL_ENDP0 ? buf_out_arm : 1'h0;
	wire			ep0_buf_out_arm_ack;
	
	wire	[8:0]	ep1_buf_out_addr	= 	sel_endp == SEL_ENDP1 ? buf_out_addr : 9'h0;
	wire	[7:0]	ep1_buf_out_q;
	wire	[9:0]	ep1_buf_out_len;	
	wire			ep1_buf_out_hasdata;
	wire			ep1_buf_out_arm		= 	sel_endp == SEL_ENDP1 ? buf_out_arm : 1'h0;
	wire			ep1_buf_out_arm_ack;
	wire			ep1_data_toggle_act	= 	sel_endp == SEL_ENDP1 ? data_toggle_act : 1'h0;
	wire	[1:0]	ep1_data_toggle;
	
	wire	[8:0]	ep2_buf_in_addr		= 	sel_endp == SEL_ENDP2 ? buf_in_addr : 9'h0;
	wire	[7:0]	ep2_buf_in_data		= 	sel_endp == SEL_ENDP2 ? buf_in_data : 8'h0;
	wire			ep2_buf_in_wren		= 	sel_endp == SEL_ENDP2 ? buf_in_wren : 1'h0;
	wire			ep2_buf_in_ready;
	wire			ep2_buf_in_commit 	= 	sel_endp == SEL_ENDP2 ? buf_in_commit : 1'h0;
	wire	[9:0]	ep2_buf_in_commit_len = sel_endp == SEL_ENDP2 ? buf_in_commit_len : 10'h0;
	wire			ep2_buf_in_commit_ack;
	wire			ep2_data_toggle_act	= 	sel_endp == SEL_ENDP2 ? data_toggle_act : 1'h0;
	wire	[1:0]	ep2_data_toggle;

	
	assign			buf_in_ready		= 	sel_endp == SEL_ENDP0 ? ep0_buf_in_ready : 
											sel_endp == SEL_ENDP2 ? ep2_buf_in_ready : 1'h0;
											
	assign			buf_in_commit_ack	= 	sel_endp == SEL_ENDP0 ? ep0_buf_in_commit_ack : 
											sel_endp == SEL_ENDP2 ? ep2_buf_in_commit_ack : 1'h0;

	assign			buf_out_q			= 	sel_endp == SEL_ENDP0 ? ep0_buf_out_q :
											sel_endp == SEL_ENDP1 ? ep1_buf_out_q : 8'h0;
											
	assign			buf_out_len			= 	sel_endp == SEL_ENDP0 ? ep0_buf_out_len : 
											sel_endp == SEL_ENDP1 ? ep1_buf_out_len : 10'h0;
											
	assign			buf_out_hasdata		= 	sel_endp == SEL_ENDP0 ? ep0_buf_out_hasdata : 
											sel_endp == SEL_ENDP1 ? ep1_buf_out_hasdata : 1'h0;
											
	assign			buf_out_arm_ack		= 	sel_endp == SEL_ENDP0 ? ep0_buf_out_arm_ack : 
											sel_endp == SEL_ENDP1 ? ep1_buf_out_arm_ack : 1'h0;
											
	assign			endp_mode			=	sel_endp == SEL_ENDP1 ? EP1_MODE : 
											sel_endp == SEL_ENDP2 ? EP2_MODE : EP_MODE_CONTROL;
											
	assign			data_toggle			= 	sel_endp == SEL_ENDP0 ? ep0_data_toggle : 
											sel_endp == SEL_ENDP1 ? ep1_data_toggle : 
											sel_endp == SEL_ENDP2 ? ep2_data_toggle : 2'h0;
											
	parameter [3:0]	SEL_ENDP0 			= 4'd0,
					SEL_ENDP1 			= 4'd1,
					SEL_ENDP2 			= 4'd2;
					
	parameter [1:0]	EP_MODE_CONTROL		= 2'd0,
					EP_MODE_ISOCH		= 2'd1,
					EP_MODE_BULK		= 2'd2,
					EP_MODE_INTERRUPT	= 2'd3;
					
	// assign endpoint modes here and also 
	// in the descriptor strings
	wire	[1:0]	EP1_MODE			= EP_MODE_BULK;
	wire	[1:0]	EP2_MODE			= EP_MODE_BULK;
					
	reg		[5:0]	dc;
	
	reg		[5:0]	state;
	parameter [5:0]	ST_RST_0			= 6'd0,
					ST_RST_1			= 6'd1,
					ST_IDLE				= 6'd10;

always @(posedge phy_clk) begin

	{reset_2, reset_1} <= {reset_1, reset_n};
	
	dc <= dc + 1'b1;

	case(state)
	ST_RST_0: begin
		// reset state
		state <= ST_RST_1;
	end
	ST_RST_1: begin
		state <= ST_IDLE;
	end
	ST_IDLE: begin
		
	end
	default: state <= ST_RST_0;
	endcase
	
	if(~reset_2) begin
		// reset
		state <= ST_RST_0;
	end
	
end



////////////////////////////////////////////////////////////
//
// ENDPOINT 0 IN/OUT
//
////////////////////////////////////////////////////////////

usb2_ep0 iep0 (
	.phy_clk		( phy_clk ),
	.reset_n		( reset_n ),

	.buf_in_addr		( ep0_buf_in_addr ),
	.buf_in_data		( ep0_buf_in_data ),
	.buf_in_wren		( ep0_buf_in_wren ),
	.buf_in_ready		( ep0_buf_in_ready ),
	.buf_in_commit		( ep0_buf_in_commit ),
	.buf_in_commit_len 	( ep0_buf_in_commit_len ),
	.buf_in_commit_ack 	( ep0_buf_in_commit_ack ),
	
	.buf_out_addr		( ep0_buf_out_addr ),
	.buf_out_q			( ep0_buf_out_q ),
	.buf_out_len		( ep0_buf_out_len ),
	.buf_out_hasdata	( ep0_buf_out_hasdata ),
	.buf_out_arm		( ep0_buf_out_arm ),
	.buf_out_arm_ack	( ep0_buf_out_arm_ack ),
	
	.vend_req_act		( vend_req_act ),
	.vend_req_request	( vend_req_request ),
	.vend_req_val		( vend_req_val ),

	.data_toggle_act	( ep0_data_toggle_act ),
	.data_toggle		( ep0_data_toggle ),
	
	.dev_addr		( dev_addr ),
	.configured		( configured ),
	
	.err_setup_pkt	( err_setup_pkt )
);


////////////////////////////////////////////////////////////
//
// ENDPOINT 1 IN
//
////////////////////////////////////////////////////////////

usb2_ep iep1 (
	.phy_clk		( phy_clk ),
	.rd_clk			( phy_clk ),
	.wr_clk			( ext_clk ),
	
	.reset_n		( reset_n ),

	.buf_in_addr		( ext_buf_in_addr ),
	.buf_in_data		( ext_buf_in_data ),
	.buf_in_wren		( ext_buf_in_wren ),
	.buf_in_ready		( ext_buf_in_ready ),
	.buf_in_commit		( ext_buf_in_commit ),
	.buf_in_commit_len 	( ext_buf_in_commit_len ),
	.buf_in_commit_ack 	( ext_buf_in_commit_ack ),
	
	.buf_out_addr		( ep1_buf_out_addr ),
	.buf_out_q			( ep1_buf_out_q ),
	.buf_out_len		( ep1_buf_out_len ),
	.buf_out_hasdata	( ep1_buf_out_hasdata ),
	.buf_out_arm		( ep1_buf_out_arm ),
	.buf_out_arm_ack	( ep1_buf_out_arm_ack ),
	
	.mode				( EP1_MODE ),
	
	.data_toggle_act	( ep1_data_toggle_act ),
	.data_toggle		( ep1_data_toggle )
);

////////////////////////////////////////////////////////////
//
// ENDPOINT 2 OUT
//
////////////////////////////////////////////////////////////

usb2_ep iep2 (
	.phy_clk		( phy_clk ),
	.rd_clk			( ext_clk ),
	.wr_clk			( phy_clk ),
	
	.reset_n		( reset_n ),

	.buf_in_addr		( ep2_buf_in_addr ),
	.buf_in_data		( ep2_buf_in_data ),
	.buf_in_wren		( ep2_buf_in_wren ),
	.buf_in_ready		( ep2_buf_in_ready ),
	.buf_in_commit		( ep2_buf_in_commit ),
	.buf_in_commit_len 	( ep2_buf_in_commit_len ),
	.buf_in_commit_ack 	( ep2_buf_in_commit_ack ),
	
	.buf_out_addr		( ext_buf_out_addr ),
	.buf_out_q			( ext_buf_out_q ),
	.buf_out_len		( ext_buf_out_len ),
	.buf_out_hasdata	( ext_buf_out_hasdata ),
	.buf_out_arm		( ext_buf_out_arm ),
	.buf_out_arm_ack	( ext_buf_out_arm_ack ),
	
	.mode				( EP2_MODE ),
	
	.data_toggle_act	( ep2_data_toggle_act ),
	.data_toggle		( ep2_data_toggle )
);



endmodule

