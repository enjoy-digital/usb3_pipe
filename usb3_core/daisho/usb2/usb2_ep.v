
//
// usb 2.0 endpoint abstract
//
// Copyright (c) 2012-2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb2_ep (

// top-level interface
input	wire			phy_clk,
input	wire			rd_clk,
input	wire			wr_clk,
input	wire			reset_n,

// PROTOCOL
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

input	wire	[1:0]	mode,

input	wire			data_toggle_act,
output	reg		[1:0]	data_toggle

);

	// synchronizers
	reg 			reset_1, reset_2;
	reg				buf_in_commit_1, buf_in_commit_2, buf_in_commit_3;
	reg				buf_out_arm_1, buf_out_arm_2, buf_out_arm_3;
	
	parameter [1:0]	DATA_TOGGLE_0	= 2'b00;
	parameter [1:0]	DATA_TOGGLE_1	= 2'b01;
	parameter [1:0]	DATA_TOGGLE_2	= 2'b10;
	parameter [1:0]	DATA_TOGGLE_M	= 2'b11;
	
	// for keeping track of the endpoint double buffering
	reg				ptr_in;
	reg				ptr_out;
	
	reg		[9:0]	len_in;
	reg				ready_in_a;
	reg				ready_in_b;
	assign			buf_in_ready 		= 	ptr_in ? ready_in_b : ready_in_a;
	assign			buf_in_commit_ack	= 	(state_in == ST_IN_COMMIT || state_in == ST_IN_SWAP);
	
	reg		[9:0]	len_out_a;
	reg		[9:0]	len_out_b;
	reg				hasdata_out_a;
	reg				hasdata_out_b;
	assign			buf_out_len			=	ptr_out ? len_out_b : len_out_a;
	assign			buf_out_hasdata 	= 	ptr_out ? hasdata_out_b : hasdata_out_a;
	assign			buf_out_arm_ack 	= 	(state_out == ST_OUT_ARM || state_out == ST_OUT_SWAP);
	
	parameter [1:0]	EP_MODE_CONTROL		= 2'd0,
					EP_MODE_ISOCH		= 2'd1,
					EP_MODE_BULK		= 2'd2,
					EP_MODE_INTERRUPT	= 2'd3;
					
	reg		[3:0]	dc;
	
	reg		[5:0]	state_in;
	parameter [5:0]	ST_RST_0			= 6'd0,
					ST_RST_1			= 6'd1,
					ST_IDLE				= 6'd10,
					ST_IN_COMMIT		= 6'd11,
					ST_IN_SWAP			= 6'd12;
					
	reg		[5:0]	state_out;	
	parameter [5:0]	ST_OUT_ARM			= 6'd11,
					ST_OUT_SWAP			= 6'd12;	
	
always @(posedge phy_clk) begin

	// synchronizers
	{reset_2, reset_1} <= {reset_1, reset_n};
	{buf_in_commit_3, buf_in_commit_2, buf_in_commit_1} <= 
		{buf_in_commit_2, buf_in_commit_1, buf_in_commit};
	{buf_out_arm_3, buf_out_arm_2, buf_out_arm_1} <= 
		{buf_out_arm_2, buf_out_arm_1, buf_out_arm};
	
	dc <= dc + 1'b1;
		
	if(data_toggle_act) begin
		data_toggle <= data_toggle + 1'b1;
		// modify for non-bulk endpoints
		// bulk transfers only utilize DATA0 and DATA1
		if(data_toggle == DATA_TOGGLE_1) data_toggle <= DATA_TOGGLE_0;
	end
	
	// input FSM
	//
	case(state_in) 
	ST_RST_0: begin
		// reset buffer index
		ptr_in <= 0;
		data_toggle <= DATA_TOGGLE_0;
		
		ready_in_a <= 1;
		ready_in_b <= 1;
				
		// configure default state		
		state_in <= ST_RST_1;
	end
	ST_RST_1: begin
		state_in <= ST_IDLE;
	end
	
	ST_IDLE: begin
		// idle state
		if(buf_in_commit_2 & ~buf_in_commit_3) begin
			// external device has written to this endpoint
			len_in <= buf_in_commit_len;
			dc <= 0;
			state_in <= ST_IN_COMMIT;
		end
	end
	ST_IN_COMMIT: begin
		// generate ACK pulse, 4 cycles long for slower clock domains
		if(dc == 3) begin
			state_in <= ST_IN_SWAP;
		end
	end
	ST_IN_SWAP: begin
		// swap the current buffer
		ptr_in <= ~ptr_in;
		
		// current buffer is now not ready anymore
		case(ptr_in)
		0: ready_in_a <= 0;
		1: ready_in_b <= 0;
		endcase
		
		// tell output FSM this has data
		case(ptr_in)
		0: hasdata_out_a <= 1;
		1: hasdata_out_b <= 1;
		endcase
		
		// copy over the amount of data intended to be sent
		case(ptr_in)
		0: len_out_a <= len_in;
		1: len_out_b <= len_in;
		endcase
		
		state_in <= ST_IDLE;
	end
	default: state_in <= ST_RST_0;
	endcase
	
	//////////////////////////////////////////////////////
	
	// output FSM
	//
	case(state_out) 
	ST_RST_0: begin
		// reset buffer index
		ptr_out <= 0;
		
		hasdata_out_a <= 0;
		hasdata_out_b <= 0;
		
		// configure default state		
		state_out <= ST_RST_1;
	end
	ST_RST_1: begin
		state_out <= ST_IDLE;
	end
	ST_IDLE: begin
		// idle state
		if(buf_out_arm_2 & ~buf_out_arm_3) begin
			// free up this endpoint
			dc <= 0;
			state_out <= ST_OUT_ARM;
		end
	end
	ST_OUT_ARM: begin
		// generate ARM_ACK pulse, several cycles for compat with slower FSMs
		if(dc == 3) begin
			state_out <= ST_OUT_SWAP;
		end
	end
	ST_OUT_SWAP: begin
		// swap the current buffer
		ptr_out <= ~ptr_out;
		
		// current buffer is now ready for data
		case(ptr_out)
		0: ready_in_a <= 1;
		1: ready_in_b <= 1;
		endcase
		
		// update hasdata status
		case(ptr_out)
		0: hasdata_out_a <= 0;
		1: hasdata_out_b <= 0;
		endcase
		
		state_out <= ST_IDLE;
	end
	default: state_out <= ST_RST_0;
	endcase
		
	if(~reset_2) begin
		// reset
		state_in <= ST_RST_0;
		state_out <= ST_RST_0;
	end
	
end

// endpoint bram
//

	// segment the space into two 512 byte buffers
	//
	wire	[9:0]	rd_addr = buf_out_addr + (ptr_out ? 10'd512 : 10'h0);
	wire	[9:0]	wr_addr = buf_in_addr + (ptr_in ? 10'd512 : 10'h0);
	
usb2_ep_ram	iu2ep (
	.rd_clk		( rd_clk ),
	.rd_adr 	( rd_addr ),
	.rd_dat_r 	( buf_out_q ),
	
	.wr_clk		( wr_clk ),
	.wr_adr 	( wr_addr ),
	.wr_dat_w 	( buf_in_data ),
	.wr_we 		( buf_in_wren )
);

	
endmodule
