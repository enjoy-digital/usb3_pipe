
//
// usb 3.0 endpoint 0
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb3_ep0 (

input	wire			slow_clk,
input	wire			local_clk,
input	wire			reset_n,

input	wire	[8:0]	buf_in_addr,
input	wire	[31:0]	buf_in_data,
input	wire			buf_in_wren,
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

output	reg				vend_req_act,
output	reg		[7:0]	vend_req_request,
output	reg		[15:0]	vend_req_val,
output	reg		[15:0]	vend_req_index,
//output	reg		[15:0]	vend_req_len,

output	reg		[6:0]	dev_addr,
output	reg				configured,
output	reg				reset_dp_seq,

output	reg				err_setup_pkt

);

`include "usb3_const.vh"
`include "usb_descrip.vh"

	reg				buf_in_commit_1, buf_in_commit_2;
	reg				buf_out_arm_1, buf_out_arm_2;

	reg		[63:0]	packet_setup;
	wire	[7:0]	packet_setup_reqtype = packet_setup[63:56];	
	wire			packet_setup_dir	= packet_setup_reqtype[7];
	parameter		SETUP_DIR_HOSTTODEV	= 1'b0,
					SETUP_DIR_DEVTOHOST	= 1'b1;
	wire	[1:0]	packet_setup_type	= packet_setup_reqtype[6:5];
	parameter [1:0]	SETUP_TYPE_STANDARD	= 2'h0,
					SETUP_TYPE_CLASS	= 2'h1,
					SETUP_TYPE_VENDOR	= 2'h2,
					SETUP_TYPE_RESVD	= 2'h3;
	wire	[4:0]	packet_setup_recpt	= packet_setup_reqtype[4:0];
	parameter [4:0]	SETUP_RECPT_DEVICE	= 5'h0,
					SETUP_RECPT_IFACE	= 5'h1,
					SETUP_RECPT_ENDP	= 5'h2,
					SETUP_RECPT_OTHER	= 5'h3;
	wire	[7:0]	packet_setup_req	= packet_setup[55:48];
	parameter [7:0]	REQ_GET_STATUS		= 8'h0,
					REQ_CLEAR_FEAT		= 8'h1,
					REQ_SET_FEAT		= 8'h3,
					REQ_SET_ADDR		= 8'h5,
					REQ_GET_DESCR		= 8'h6,
					REQ_SET_DESCR		= 8'h7,
					REQ_GET_CONFIG		= 8'h8,
					REQ_SET_CONFIG		= 8'h9,
					REQ_SET_INTERFACE	= 8'hB,
					REQ_SYNCH_FRAME		= 8'h12,
					REQ_SET_SEL			= 8'h30;
	wire	[15:0]	packet_setup_wval	= {packet_setup[39:32], packet_setup[47:40]};
	wire	[15:0]	packet_setup_widx	= {packet_setup[23:16], packet_setup[31:24]};
	wire	[15:0]	packet_setup_wlen	= {packet_setup[7:0], packet_setup[15:8]};
	
	reg		[10:0]	desired_out_len;
	reg		[10:0]	packet_out_len;
	reg		[3:0]	dev_config;
	
	reg				ptr_in;
	reg				ptr_out;
	
	reg		[10:0]	len_in;
	reg				ready_in;
	assign			buf_in_ready 		= 	ready_in;
	assign			buf_in_commit_ack	= 	(state_in == ST_IN_COMMIT);
	
	reg		[10:0]	len_out;
	reg				hasdata_out;
	assign			buf_out_len			=	len_out;
	assign			buf_out_hasdata 	= 	hasdata_out;
	assign			buf_out_arm_ack 	= 	(state_out == ST_OUT_ARM);
	
	reg		[6:0]	dc;
	
	reg		[5:0]	state_in;
	parameter [5:0]	ST_RST_0			= 6'd0,
					ST_RST_1			= 6'd1,
					ST_IDLE				= 6'd10,
					ST_IN_COMMIT		= 6'd11,
					ST_IN_SWAP			= 6'd20,
					ST_IN_PARSE_0		= 6'd21,
					ST_IN_PARSE_1		= 6'd22,
					ST_REQ_DESCR		= 6'd30,
					ST_RDLEN_0			= 6'd31,
					ST_RDLEN_1			= 6'd32,
					ST_RDLEN_2			= 6'd33,
					ST_RDLEN_3			= 6'd34,
					ST_REQ_GETCONFIG	= 6'd35,
					ST_REQ_SETCONFIG	= 6'd36,
					ST_REQ_SETINTERFACE	= 6'd37,
					ST_REQ_SETADDR		= 6'd38,
					ST_REQ_VENDOR		= 6'd39,
					ST_REQ_SETSEL		= 6'd40,
					ST_REQ_SETFEAT		= 6'd41,
					ST_REQ_CLRFEAT		= 6'd42;
					
	reg		[5:0]	state_out;
	parameter [5:0]	ST_OUT_ARM			= 6'd11,
					ST_OUT_SWAP			= 6'd20;
				
always @(posedge local_clk) begin

	{buf_in_commit_2, buf_in_commit_1} <= {buf_in_commit_1, buf_in_commit};
	{buf_out_arm_2, buf_out_arm_1} <= {buf_out_arm_1, buf_out_arm};

	configured <= dev_config ? 1'b1 : 1'b0;
		
	reset_dp_seq <= 0;
	
	`INC(dc);
	
	// clear act strobe after 4 cycles
	if(dc == 3) vend_req_act <= 1'b0;
	
	// main fsm
	case(state_in) 
	ST_RST_0: begin
		len_out <= 0;
		
		desired_out_len <= 0;
		dev_addr <= 0;
		dev_config <= 0;
		err_setup_pkt <= 0;
		
		ready_in <= 1;
		
		state_in <= ST_RST_1;
	end
	ST_RST_1: begin
		state_in <= ST_IDLE;
	end
	ST_IDLE: begin
		// idle state
		if(buf_in_commit_1 & ~buf_in_commit_2) begin
			// link/protocol layer has written to this endpoint
			len_in <= buf_in_commit_len;

			ready_in <= 0;

			dc <= 0;
			state_in <= ST_IN_COMMIT;
		end
	end
	ST_IN_COMMIT: begin
		// generate ACK pulse
		if(dc == 3) begin
			dc <= 0;
			buf_in_rdaddr <= 0;
			state_in <= ST_IN_PARSE_0;
		end
	end
	
	ST_IN_PARSE_0: begin
		// parse setup packet
		`INC(buf_in_rdaddr);
		
		packet_setup <= {packet_setup[31:0], buf_in_q[31:0]};
		if(dc == (2+2-1)) state_in <= ST_IN_PARSE_1;		
	end
	ST_IN_PARSE_1: begin
		// parse setup packet
		packet_out_len <= packet_setup_wlen;
		
		// confirm this is going in the right direction
		//if(packet_setup_dir != SETUP_DIR_DEVTOHOST) begin
		//	err_setup_pkt <= 1;
		//	state <= 10;
		//end else begin
		
		if(packet_setup_type == SETUP_TYPE_VENDOR) begin
			// parse vendor request
			state_in <= ST_REQ_VENDOR;
		end else begin
			// proceed with parsing
			
			case(packet_setup_req)
			REQ_GET_DESCR: begin
				state_in <= ST_REQ_DESCR;
			end
			REQ_GET_CONFIG: begin
				state_in <= ST_REQ_GETCONFIG;
			end
			REQ_SET_CONFIG: begin
				state_in <= ST_REQ_SETCONFIG;
			end
			REQ_SET_INTERFACE: begin
				state_in <= ST_REQ_SETINTERFACE;
			end
			REQ_SET_ADDR: begin
				state_in <= ST_REQ_SETADDR;
			end
			REQ_SET_FEAT: begin
				state_in <= ST_REQ_SETFEAT;
			end
			REQ_CLEAR_FEAT: begin
				state_in <= ST_REQ_CLRFEAT;
			end
			REQ_SET_SEL: begin
				state_in <= ST_REQ_SETSEL;
			end
			default: begin
				ready_in <= 1;
				state_in <= ST_IDLE;
			end
			endcase
		end
	end
	
	ST_REQ_DESCR: begin
		state_in <= ST_RDLEN_0;
		
		// GET_DESCRIPTOR
		case(packet_setup_wval)
		16'h0100: begin
			// device descriptor
			descrip_addr_offset <= DESCR_USB3_DEVICE;
		end
		16'h0200: begin
			// config descriptor
			descrip_addr_offset <= DESCR_USB3_CONFIG;
			desired_out_len <= DESCR_USB3_CONFIG_LEN;
			state_in <= ST_RDLEN_3;
		end
		16'h0300: begin
			// string: languages
			descrip_addr_offset <= DESCR_USB3_STRING0;
		end
		16'h0301: begin
			// string: manufacturer
			descrip_addr_offset <= DESCR_USB3_STRING1;
		end
		16'h0302: begin
			// string: product name
			descrip_addr_offset <= DESCR_USB3_STRING2;
		end
		16'h0303: begin
			// string: serial number
			descrip_addr_offset <= DESCR_USB3_STRING3;
		end
		//16'h0600: begin
			// device qualifier descriptor
			//descrip_addr_offset <= DESCR_OFF_DEVQUAL;
		//end
		16'h0f00: begin
			// BOS #0
			descrip_addr_offset <= DESCR_USB3_BOS;
			desired_out_len <= DESCR_USB3_BOS_LEN;
			state_in <= ST_RDLEN_3;
		end
		default: begin
			packet_out_len <= 0;
		end
		endcase
	end
	ST_RDLEN_0: begin
		// wait cycle if descriptor BRAM has a buffered output
		state_in <= ST_RDLEN_1;
	end
	ST_RDLEN_1: begin
		// wait cycle if descriptor BRAM has a buffered output
		state_in <= ST_RDLEN_2;
	end
	ST_RDLEN_2: begin
		// pick off the first byte at the pointer
		desired_out_len <= buf_out_q[31:24];
		state_in <= ST_RDLEN_3;
	end
	ST_RDLEN_3: begin
		// pick smaller of the setup packet's wanted length and the stored length
		len_out <= packet_out_len < desired_out_len ? packet_out_len : desired_out_len;
		// send response 
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE; 
	end
	ST_REQ_GETCONFIG: begin
		// GET DEVICE CONFIGURATION
		
		// send 1byte response 
		len_out <= 1;
		ready_in <= 1;
		hasdata_out <= 1;
		descrip_addr_offset <= dev_config ? DESCR_USB3_CONFSET : DESCR_USB3_CONFUNSET;
		state_in <= ST_IDLE;
	end
	ST_REQ_SETCONFIG: begin
		// SET DEVICE CONFIGURATION
		dev_config <= packet_setup_wval[6:0];
		reset_dp_seq <= 1;
		
		// send 0byte response 
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_SETINTERFACE: begin
		// SET INTERFACE
		//dev_config <= packet_setup_wval[6:0];
		reset_dp_seq <= 1;
	
		// send 0byte response
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_SETADDR: begin
		// SET DEVICE ADDRESS
		dev_addr <= packet_setup_wval[6:0];
	
		// send 0byte response
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_VENDOR: begin
		// VENDOR REQUEST
		vend_req_request <= packet_setup_req;
		vend_req_val <= packet_setup_wval;
		vend_req_index <= packet_setup_widx;
		// optional data stage for bidir control transfers
		// would require additional unsupported code in this endpoint
		//vend_req_len <= packet_setup_wlen;
		// signal to external interface there was a vend_req
		vend_req_act <= 1'b1;
		dc <= 0;
		// send 0byte response
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_SETSEL: begin
		// send 0byte response
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_SETFEAT: begin
		// U1/U2 Enable; parse wValue TODO
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	ST_REQ_CLRFEAT: begin
		reset_dp_seq <= 1;
		
		len_out <= 0;
		ready_in <= 1;
		hasdata_out <= 1;
		state_in <= ST_IDLE;
	end
	
	
	
	
	default: state_in <= ST_RST_0;
	endcase
	

	
	// output FSM
	//
	case(state_out) 
	ST_RST_0: begin
		hasdata_out <= 0;
		
		// configure default state		
		state_out <= ST_RST_1;
	end
	ST_RST_1: begin
		state_out <= ST_IDLE;
	end
	ST_IDLE: begin
		// idle state
		if(buf_out_arm_1 & ~buf_out_arm_2) begin
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
		// this endpoint is not double buffered!
		// current buffer is now ready for data
		ready_in <= 1;
		// update hasdata status
		hasdata_out <= 0;
		
		state_out <= ST_IDLE;
	end
	default: state_out <= ST_RST_0;
	endcase
	
	if(~reset_n) begin
		// reset
		state_in <= ST_RST_0;
		state_out <= ST_RST_0;
		
	end
	
end


	reg		[3:0]	buf_in_rdaddr;
	wire	[31:0]	buf_in_q;
	
usb3_ep0in_ram	iu3ep0i (
	.clk 		( local_clk ),
	.wr_dat_w 	( buf_in_data ),
	.rd_adr 	( buf_in_rdaddr ),
	.wr_adr 	( buf_in_addr ),
	.wr_we 		( buf_in_wren ),
	.rd_dat_r 	( buf_in_q )
);

	reg		[7:0]	descrip_addr_offset;
	
usb3_descrip_rom	iu3d (
	.clk 		( local_clk ),
	.adr 		( buf_out_addr + descrip_addr_offset),
	.dat_r 		( buf_out_q )
);


endmodule
