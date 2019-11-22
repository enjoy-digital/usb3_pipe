
//
// usb 3.0 rx descrambling and alignment
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb3_descramble (

input	wire			clock,
input	wire			local_clk,
input	wire			reset_n,

input	wire			enable,

input	wire	[1:0]	raw_valid,
input	wire	[5:0]	raw_status,
input	wire	[1:0]	raw_phy_status,
input	wire	[3:0]	raw_datak,
input	wire	[31:0]	raw_data,

//output	reg		[1:0]	proc_valid,
//output	reg		[5:0]	proc_status,
//output	reg		[1:0]	proc_phy_status,
output	reg		[3:0]	proc_datak,
output	reg		[31:0]	proc_data,
output	reg				proc_active,

output	reg				err_skp_unexpected
);
	
`include "usb3_const.vh"

// what this module does is filtering of the input symbol stream 
// to remove everything that >PHY layer is not interested in.
//
// example: scrambled stream with skip padding
// BE40A73C3C3CE62CD3E2B20702772A (D)
// 000000111111000000000000000000 (K)
// >
// 000000000000000000000000       (D)
// 000000000000000000000000       (K)
//
// this would be fairly trivial if we could clock this module at 500mhz
// and only process 1 symbol at a time. however, due to timing constraints
// it's working at 125mhz instead, and on 4 symbols at once. this is where
// symbol alignment gets sticky, and why there are so many cases.
//
// 10/08/13 - edge case discovered where SKP immediately following the end of a
// packet on the next cycle will cause a 1 cycle deassertion of ACTIVE, and
// push the last word of the packet onto the following cycle where ACTIVE
// is asserted again.


	// indicates presence of SKP at any symbol position
	wire	[3:0]	skip	= {	(raw_data[31:24] == 8'h3C) & raw_datak[3], (raw_data[23:16] == 8'h3C) & raw_datak[2], 
								(raw_data[15:8] == 8'h3C) & raw_datak[1],  (raw_data[7:0] == 8'h3C) & raw_datak[0] };
	// indicates presence of COM at any symbol position (K28.5)
	wire	[3:0]	comma	= {	(coll_data[31:24] == 8'hBC) & coll_datak[3], (coll_data[23:16] == 8'hBC) & coll_datak[2], 
								(coll_data[15:8] == 8'hBC) & coll_datak[1],  (coll_data[7:0] == 8'hBC) & coll_datak[0] };
	
// step 1.
// collapse incoming stream to remove all SKP symbols.
// these may be sent as 0x3C, 0x3C3C, 0x3C3C3C and so on.
	
	//reg		[5:0]	skr_status;
	reg		[31:0]	skr_data;
	reg		[3:0]	skr_datak; 
	reg		[2:0]	skr_num; 
	reg		[1:0]	skr_valid;

always @(posedge local_clk) begin
	
	case(skip)
	4'b0000: begin
		skr_data 	<= raw_data;
		skr_datak	<= raw_datak; end
	4'b0001: begin
		skr_data 	<= raw_data[31:8];
		skr_datak	<= raw_datak[3:1]; end
	4'b0010: begin
		skr_data 	<= {raw_data[31:16], raw_data[7:0]};
		skr_datak	<= {raw_datak[3:2], raw_datak[0]}; end
	4'b0011: begin
		skr_data 	<= raw_data[31:16];
		skr_datak	<= raw_datak[3:2]; end
	4'b0100: begin
		skr_data 	<= {raw_data[31:24], raw_data[15:0]};
		skr_datak	<= {raw_datak[3], raw_datak[1:0]}; end
	4'b0110: begin
		skr_data 	<= {raw_data[31:24], raw_data[7:0]};
		skr_datak	<= {raw_datak[3], raw_datak[0]}; end
	4'b0111: begin
		skr_data 	<= raw_data[31:24];
		skr_datak	<= raw_datak[3]; end
	4'b1110: begin
		skr_data 	<= raw_data[7:0];
		skr_datak	<= raw_datak[0]; end
	4'b1100: begin
		skr_data 	<= raw_data[15:0];
		skr_datak	<= raw_datak[1:0]; end
	4'b1000: begin
		skr_data 	<= raw_data[23:0];
		skr_datak	<= raw_datak[2:0]; end
	4'b1111: begin
		skr_data 	<= 0;
		skr_datak	<= 0; end
	default: begin
		//{skr_status, skr_data, skr_datak} <= 0;
		{skr_data, skr_datak} <= 0;
		err_skp_unexpected <= 1;
	end
	endcase

	// count valid symbols
	skr_num <= 3'h4 - (skip[3] + skip[2] + skip[1] + skip[0]);
	//skr_status <= raw_status;
	skr_valid <= raw_valid;
	
	if(~reset_n) begin
		err_skp_unexpected <= 0;
	end
end



// step 2.
// accumulate these fragments and then squeeze them out
// 32bits at a time.

	reg		[5:0]	acc_status;
	reg		[63:0]	acc_data;
	reg		[7:0]	acc_datak; 
	
	reg		[2:0]	acc_depth; 
	
always @(posedge local_clk) begin

	// take in either 8, 16, 24, or 32 bits of data from the prior stage
	case(skr_num)
	0: begin end
	1: begin
		acc_data  <= {acc_data[55:0], skr_data[7:0]};
		acc_datak <= {acc_datak[6:0], skr_datak[0:0]};
		acc_depth <= acc_depth + 3'd1;
	end
	2: begin
		acc_data  <= {acc_data[47:0], skr_data[15:0]};
		acc_datak <= {acc_datak[5:0], skr_datak[1:0]};
		acc_depth <= acc_depth + 3'd2;
	end
	3: begin
		acc_data  <= {acc_data[39:0], skr_data[23:0]};
		acc_datak <= {acc_datak[4:0], skr_datak[2:0]};
		acc_depth <= acc_depth + 3'd3;
	end
	4: begin
		acc_data  <= {acc_data[31:0], skr_data[31:0]};
		acc_datak <= {acc_datak[3:0], skr_datak[3:0]};
		acc_depth <= acc_depth + 3'd4;
	end
	endcase
	
	// pick off 32bits and decrement the accumulator
	coll_valid <= skr_valid;
	coll_active <= (acc_depth > 3);
	case(acc_depth)
	4: begin
		{coll_data, coll_datak} <= {acc_data[31:0], acc_datak[3:0]};
		acc_depth <= 3'd0 + skr_num;
	end
	5: begin
		{coll_data, coll_datak} <= {acc_data[39:8], acc_datak[4:1]};
		acc_depth <= 3'd1 + skr_num;
	end
	6: begin
		{coll_data, coll_datak} <= {acc_data[47:16], acc_datak[5:2]};
		acc_depth <= 3'd2 + skr_num;
	end
	7: begin
		{coll_data, coll_datak} <= {acc_data[55:24], acc_datak[6:3]};
		acc_depth <= 3'd3 + skr_num;
	end
	endcase
	
	if(~reset_n) begin
		acc_depth <= 0;
	end
end


// step 3.
// handle descrambling LFSR, resetting upon COM with
// proper symbol alignment.

	reg		[31:0]	coll_data ;
	reg		[3:0]	coll_datak;
	reg				coll_active;
	reg		[1:0]	coll_valid;
	
	reg		[2:0]	ds_align;
	reg		[2:0]	scr_defer;
	
always @(posedge local_clk) begin
	
	if(scr_defer < 3) scr_defer <= scr_defer + 1'b1;
	if(|comma) scr_defer <= 0;

	case(comma)
	4'b1111: begin
		ds_align <= 0;
	end
	4'b1110: begin
		ds_align <= 1;
	end
	4'b1100: begin
		ds_align <= 2;
	end
	4'b1000: begin
		ds_align <= 3;
	end
	endcase
	
	if(coll_active) begin
		// only apply descrambling to data, not K-symbols
		next_data[31:24] <= coll_data[31:24] ^ (coll_datak[3] ? 8'h0 : ds_delay[31:24]);
		next_data[23:16] <= coll_data[23:16] ^ (coll_datak[2] ? 8'h0 : ds_delay[23:16]);
		next_data[15:8] <= coll_data[15:8] ^ (coll_datak[1] ? 8'h0 : ds_delay[15:8]);
		next_data[7:0] <= coll_data[7:0] ^ (coll_datak[0] ? 8'h0 : ds_delay[7:0]);
		next_datak <= coll_datak;
		
		// match incoming alignment
		case(ds_align)
		0: ds_delay <= {ds_last};
		1: ds_delay <= {ds_last[23:0], ds_out[31:24]};
		2: ds_delay <= {ds_last[15:0], ds_out[31:16]};
		3: ds_delay <= {ds_last[7:0], ds_out[31:8]};
		endcase
		ds_last <= ds_out;
	end
	
	next_active <= coll_active;
	
	// squelch invalids
	if(~coll_valid || ~coll_active) begin
		next_data <= 32'h0;
		next_datak <= 4'b0;
		next_active <= 0;
	end
	
end

// step 4.
// pipeline data to relax timing

	reg		[31:0]	next_data;
	reg		[3:0]	next_datak;
	reg				next_active;

always @(posedge local_clk) begin
	proc_data <= next_data;
	proc_datak <= next_datak;
	proc_active <= next_active;
end

//
// data de-scrambling for RX
//
	reg		[31:0]	ds_delay;
	reg		[31:0]	ds_last;
	wire			ds_suppress = |comma || (scr_defer < 3);
	wire			ds_enable = enable && !ds_suppress;
	wire	[31:0]	ds_out_swap;
	wire	[31:0]	ds_out = ds_enable ? 
							{ds_out_swap[7:0], ds_out_swap[15:8], ds_out_swap[23:16], ds_out_swap[31:24]} 
							: 0;
usb3_lfsr iu3srx(

	.clock		( local_clk ),
	.reset_n	( reset_n ),
	
	.data_in	( 32'h0 ),
	.scram_en	( coll_active ),
	.scram_rst	( |comma ),
	.scram_init ( 16'h7DBD ),	// reset to FFFF + 3 cycles
	.data_out_reg	( ds_out_swap )
	
);

endmodule
