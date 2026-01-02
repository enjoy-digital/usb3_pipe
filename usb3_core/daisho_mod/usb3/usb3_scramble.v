
//
// usb 3.0 tx scrambling and elastic buffer padding
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//

module usb3_scramble (

input	wire			clock,
input	wire			local_clk,
input	wire			reset_n,

input	wire			enable,

input	wire			skp_inhibit,
input	wire			skp_defer,

input	wire	[3:0]	raw_datak,
input	wire	[31:0]	raw_data,
input	wire			raw_active,
output	reg				raw_stall,

output	reg		[3:0]	proc_datak,
output	reg		[31:0]	proc_data,

output	reg				err_empty_accum
);

`include "usb3_const.vh"
	
	reg				reset_n_1;
	reg				enable_1;
	
	// indicates presence of COM at last symbol position (K28.5)
	wire			comma	= {	(pl_data[7:0] == 8'hBC) & pl_datak[0] };
	reg				comma_1;
		
	reg		[31:0]	pl_data;
	reg		[3:0]	pl_datak;
	reg				pl_active;

// step 1.
// accept incoming data, but inject SKP sets to allow remote elastic buffer
// to catch up and compensate for spread spectrum clocking.

	wire			insert_skp	=  enable && symbols_since_skp > 78;
	// nominal SKP insertion rate is every 354 symbols. 
	// however due to SSC profiles varying across the industry (i.e. +/- 2500ppm vs. -5000 to 0 ppm)
	// err on the safe side and send more SKP.
	
	reg		[15:0]	symbols_since_skp;
	reg		[2:0]	num_queued_skp;
	
	reg		[31:0]	ac_data;
	reg		[3:0]	ac_datak;
		
always @(posedge local_clk) begin
		
	pl_data <= raw_data;
	pl_datak <= raw_datak;
	pl_active <= raw_active;
	
	reset_n_1 <= reset_n;
	comma_1 <= comma;
	enable_1 <= enable;
	
	raw_stall <= 0;
	
	// increment symbol sent counter
	if(enable_1) `INC(symbols_since_skp);
	if(insert_skp) begin
		symbols_since_skp <= 0;
		// increment number of queued sets up to 4
		if(num_queued_skp < 4)
			`INC(num_queued_skp);
	end

	if(enable_1) begin
		if(num_queued_skp == 0 || pl_active) begin
			// don't inject SKP
			ac_data[31:24] <= pl_datak[3] ? pl_data[31:24] : pl_data[31:24] ^ sp_pick32[31:24];
			ac_data[23:16] <= pl_datak[2] ? pl_data[23:16] : pl_data[23:16] ^ sp_pick32[23:16];
			ac_data[15:8 ] <= pl_datak[1] ? pl_data[15:8 ] : pl_data[15:8 ] ^ sp_pick32[15:8 ];
			ac_data[ 7:0 ] <= pl_datak[0] ? pl_data[ 7:0 ] : pl_data[ 7:0 ] ^ sp_pick32[ 7:0 ];
			ac_datak <= pl_datak;
		end else if(num_queued_skp == 1) begin
			// only 1 ordered set needed
			ac_data <= {16'h3C3C, pl_data[15:0] ^ sp_pick16[15:0]};
			ac_datak <= {2'b11, pl_datak[1:0]};
			raw_stall <= 1;
			`DEC(num_queued_skp);
		end else begin
			// 2 or more sets needed
			ac_data <= {32'h3C3C3C3C};
			ac_datak <= {4'b1111};
			raw_stall <= 1;
			num_queued_skp <= num_queued_skp - 2'h2;
		end
	end else begin
		ac_data <= pl_data;
		ac_datak <= pl_datak;
	end

	proc_data <= ac_data;
	proc_datak <= ac_datak;
	
	err_empty_accum <= 0;

	if(~enable_1) begin
		symbols_since_skp <= 0;
		num_queued_skp <= 0;
	end
	
	if(~reset_n) begin
		symbols_since_skp <= 0;
	end
end

always @(*) begin
	//
	// combinational to give us 1 cycle foresight so that
	// the lfsr pool can have next word prepared in time
	//
	if(enable_1) begin
		if(num_queued_skp == 0 || pl_active) begin
			// don't inject SKP
			sp_read_num = 4;
		end else if(num_queued_skp == 1) begin
			// only 1 ordered set needed
			sp_read_num = 2;
		end else begin
			// 2 or more sets needed
			sp_read_num = 0;
		end
	end else begin
		sp_read_num = 0;
	end
end

//
// scrambling LFSR pool
//

	reg		[63:0]	sp_data;
	reg		[3:0]	sp_depth;
	reg		[3:0]	sp_read_num;
	
	wire			sp_dofill = ((sp_depth+8 - sp_read_num) <= 12);
	
	wire	[15:0]	sp_pick16 = 	sp_pick32[31:16];
	wire	[31:0]	sp_pick32 = 	(sp_depth == 4) ? sp_data[31:0] :
									(sp_depth == 6) ? sp_data[47:16] : 
									(sp_depth == 8) ? sp_data[63:32] : 32'hFFFFFFFF;
	
always @(posedge local_clk) begin
	
	sp_depth <= sp_depth - sp_read_num + (sp_dofill ? 4'h4 : 4'h0);

	if(sp_dofill) sp_data <= {sp_data[31:0], ds_out};

	if(~reset_n_1 | comma_1) begin
		sp_depth <= 0;
		//sp_data <= 32'hEEEEEEEE;
	end
end


//
// data scrambling for TX
//
	wire	[31:0]	ds_out = {ds_out_swap[7:0], ds_out_swap[15:8], 
							ds_out_swap[23:16], ds_out_swap[31:24]};
	wire	[31:0]	ds_out_swap;
	
usb3_lfsr iu3srx(

	.clock		( local_clk ),
	.reset_n	( reset_n ),
	
	.data_in	( 32'h0 ),
	.scram_en	( comma | sp_dofill),
	.scram_rst	( comma | ~reset_n),
	.scram_init ( 16'h7DBD ),
	.data_out	( ds_out_swap )
	
);

endmodule
