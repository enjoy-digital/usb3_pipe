
//
// usb 3.0 pipe3
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license.
// See LICENSE.TXT for details.
//

module usb3_pipe (

input	wire			slow_clk,
input	wire			local_clk,
input	wire			local_clk_capture,
input	wire			reset_n,
output	reg				ltssm_reset_n,

input	wire	[31:0]	phy_pipe_rx_data,
input	wire	[3:0]	phy_pipe_rx_datak,
input	wire	[1:0]	phy_pipe_rx_valid,
output	wire	[31:0]	phy_pipe_tx_data,
output	wire	[3:0]	phy_pipe_tx_datak,

output	wire			phy_tx_detrx_lpbk,
output	wire			phy_tx_elecidle,
input	wire			phy_rx_elecidle,
input	wire	[5:0]	phy_rx_status,
output	reg		[1:0]	phy_power_down,
input	wire	[1:0]	phy_phy_status,
input	wire			phy_pwrpresent,

output	reg				phy_tx_oneszeros,
output	reg		[1:0]	phy_tx_deemph,
output	reg		[2:0]	phy_tx_margin,
output	reg				phy_tx_swing,
output	reg				phy_rx_polarity,
output	reg				phy_rx_termination,
output	reg				phy_rate,
output	reg				phy_elas_buf_mode,

output	wire	[31:0]	link_in_data,
output	wire	[3:0]	link_in_datak,
output	wire			link_in_active,

input	wire	[31:0]	link_out_data,
input	wire	[3:0]	link_out_datak,
input	wire			link_out_active,
input	wire			link_out_skp_inhibit,
input	wire			link_out_skp_defer,
output	wire			link_out_stall,

input	wire	[4:0]	ltssm_state,
input	wire			ltssm_tx_detrx_lpbk,
input	wire			ltssm_tx_elecidle,
input	wire	[1:0]	ltssm_power_down,
input	wire			ltssm_power_go,
output	reg				ltssm_power_ack,
output	reg				ltssm_power_err,
output	reg				ltssm_hot_reset,
input	wire			ltssm_training,
input	wire			ltssm_train_rxeq,
output	reg				ltssm_train_rxeq_pass,
input	wire			ltssm_train_active,
input	wire			ltssm_train_config,
input	wire			ltssm_train_idle,
output	reg				ltssm_train_idle_pass,
output	reg				ltssm_train_ts1,
output	reg				ltssm_train_ts2,

input	wire			lfps_recv_active,
input	wire			lfps_recv_poll_u1,
input	wire			lfps_recv_ping,
input	wire			lfps_recv_reset,
input	wire			lfps_recv_u2lb,
input	wire			lfps_recv_u3,

input	wire			partner_detect,
output	reg				partner_looking,
output	reg				partner_detected,
output  wire [5:0]      dbg_state
);

`include "usb3_const.vh"

	// mux these phy signals with both local PIPE and external LTSSM control
	//
	assign phy_tx_elecidle = phy_tx_elecidle_local & ltssm_tx_elecidle;
	assign phy_tx_detrx_lpbk = phy_tx_detrx_lpbk_local | ltssm_tx_detrx_lpbk;

	reg				lfps_recv_active_1;
	reg				partner_detect_1;
	reg				ltssm_power_go_1;

	reg		[23:0]	dc;								// delay count
	reg		[7:0]	swc;							// small word count
	reg		[4:0]	rdc;							// rxdet delay count
	reg		[7:0]	sc;								//

	reg		[5:0]	state;
	assign dbg_state = state;
parameter	[5:0]	ST_RST_0			= 6'd0,
					ST_RST_1			= 6'd1,
					ST_RST_2			= 6'd2,
					ST_RST_3			= 6'd3,
					ST_RST_4			= 6'd4,
					ST_IDLE				= 6'd10,
					ST_TRAIN_RXEQ_0		= 6'd40,
					ST_TRAIN_RXEQ_1		= 6'd41,
					ST_TRAIN_ACTIVECONFIG_0	= 6'd42,
					ST_TRAIN_ACTIVECONFIG_1	= 6'd43,
					ST_TRAIN_IDLE_0		= 6'd44,
					ST_TRAIN_IDLE_1		= 6'd49,
					ST_U0				= 6'd45,
					ST_U1				= 6'd46,
					ST_U2				= 6'd47,
					ST_U3				= 6'd48;

	reg		[5:0]	align_state;
parameter	[5:0]	ALIGN_RESET			= 6'd0,
					ALIGN_IDLE			= 6'd1,
					ALIGN_0				= 6'd2,
					ALIGN_1				= 6'd3,
					ALIGN_2				= 6'd4,
					ALIGN_3				= 6'd5,
					ALIGN_4				= 6'd6;

	reg		[5:0]	tsdet_state;
parameter	[5:0]	TSDET_RESET			= 6'd0,
					TSDET_IDLE			= 6'd1,
					TSDET_0				= 6'd2,
					TSDET_1				= 6'd3,
					TSDET_2				= 6'd4;

	reg		[5:0]	rxdet_state;
parameter	[5:0]	RXDET_RESET			= 6'd0,
					RXDET_IDLE			= 6'd1,
					RXDET_0		 		= 6'd2,
					RXDET_1				= 6'd3,
					RXDET_2				= 6'd4,
					RXDET_3				= 6'd5;

	reg		[5:0]	pd_state;
parameter	[5:0]	PD_RESET			= 6'd0,
					PD_IDLE				= 6'd1,
					PD_0		 		= 6'd2,
					PD_1				= 6'd3,
					PD_2				= 6'd4,
					PD_3				= 6'd5;

	reg				phy_tx_elecidle_local;
	reg				phy_tx_detrx_lpbk_local;

	reg		[20:0]	rxdet_timeout;
	reg		[15:0]	train_count;

	reg		[12:0]	train_sym_skp;

	reg		[1:0]	word_rx_align;
	reg				set_ts;

	reg				s_enable;
	reg				ds_enable;

	reg				ts_disable_scrambling;
	reg				ts_disable_scrambling_latch;
	reg				ts_hot_reset_local;
	reg				ts_hot_reset_1;
	reg				ts_hot_reset_latch;
	reg		[4:0]	hot_reset_count;

	reg		[3:0]	idle_symbol_send;
	reg		[3:0]	idle_symbol_recv;

	reg				set_ts1_found, set_ts1_found_1;
	reg				set_ts2_found, set_ts2_found_1;

	reg		[31:0]	sync_a;
	reg		[3:0]	sync_ak;
	reg				sync_a_active;
	reg		[31:0]	sync_b;
	reg		[3:0]	sync_bk;
	reg				sync_b_active;
	reg		[31:0]	sync_out /* synthesis noprune */;
	reg		[3:0]	sync_outk /* synthesis noprune */;
	reg				sync_out_active;

	// combinational detection of valid packet framing
	// k-symbols within a received word.
	wire			sync_byte_3 = proc_datak[3] && (	proc_data[31:24] == 8'h5C || proc_data[31:24] == 8'hBC ||
														proc_data[31:24] == 8'hFB || proc_data[31:24] == 8'hFE );
	wire			sync_byte_2 = proc_datak[2] && (	proc_data[23:16] == 8'h5C || proc_data[23:16] == 8'hBC ||
														proc_data[23:16] == 8'hFB || proc_data[23:16] == 8'hFE );
	wire			sync_byte_1 = proc_datak[1] && (	proc_data[15:8] == 8'h5C || proc_data[15:8] == 8'hBC ||
														proc_data[15:8] == 8'hFB || proc_data[15:8] == 8'hFE );
	wire			sync_byte_0 = proc_datak[0] && (	proc_data[7:0] == 8'h5C || proc_data[7:0] == 8'hBC ||
														proc_data[7:0] == 8'hFB || proc_data[7:0] == 8'hFE );
	wire	[3:0]	sync_start 	= {sync_byte_3, sync_byte_2, sync_byte_1, sync_byte_0};

	wire			sync_end_3 = proc_datak[3] && (		proc_data[31:24] == 8'hF7 || proc_data[31:24] == 8'hBC );
	wire			sync_end_2 = proc_datak[2] && (		proc_data[23:16] == 8'hF7 || proc_data[23:16] == 8'hBC );
	wire			sync_end_1 = proc_datak[1] && (		proc_data[15:8] == 8'hF7 || proc_data[15:8] == 8'hBC );
	wire			sync_end_0 = proc_datak[0] && (		proc_data[7:0] == 8'hF7 || proc_data[7:0] == 8'hBC );
	wire	[3:0]	sync_end 	= {sync_end_3, sync_end_2, sync_end_1, sync_end_0};

	assign			link_in_data	= sync_out;
	assign			link_in_datak	= sync_outk;
	assign			link_in_active	= sync_out_active;

always @(posedge local_clk) begin

	// synchronizers
	lfps_recv_active_1 <= lfps_recv_active; // (just rising edge detection)
	partner_detect_1 <= partner_detect;
	ltssm_power_go_1 <= ltssm_power_go;

	// counters
	`INC(dc);
	`INC(rdc);
	`INC(swc);
	`INC(rxdet_timeout);

	phy_tx_elecidle_local <= 1'b1;
	phy_tx_detrx_lpbk_local <= 1'b0;

	ltssm_train_rxeq_pass <= 0;
	ltssm_train_idle_pass <= 0;

	local_tx_data <= 32'h0;
	local_tx_datak <= 4'b0000;
	local_tx_active <= 1'b0;
	local_tx_skp_inhibit <= 0;
	local_tx_skp_defer <= 0;

	set_ts1_found <= 0;
	set_ts2_found <= 0;
	ts_disable_scrambling <= 0;
	ts_hot_reset_1 <= ts_hot_reset_local;
	//ltssm_hot_reset <= ts_hot_reset_local | ts_hot_reset_1;

	ltssm_train_ts1 <= set_ts1_found_1 | set_ts1_found;
	ltssm_train_ts2 <= set_ts2_found_1 | set_ts2_found;

	set_ts1_found_1 <= set_ts1_found;
	set_ts2_found_1 <= set_ts2_found;

	// on first detection of TS2 Reset, send at least 16 TS2 with Reset
	if(ts_hot_reset_local & ~ts_hot_reset_1) hot_reset_count <= 16;

	///////////////////////////////////////
	// PIPE FSM
	///////////////////////////////////////

	case(state)
	ST_RST_0: begin
		// reset state

		// start-up parameters
		ltssm_reset_n <= 0;
		phy_tx_oneszeros = 		1'b0;
		phy_rx_polarity <= 		1'b0;			// no lane polarity inversion
		phy_power_down <= 		POWERDOWN_2;	// P2 state
		phy_tx_margin <= 		MARGIN_A;		// normal operating margin
		phy_tx_deemph <= 		DEEMPH_3_5_DB;	// -3.5dB
		phy_rate <= 			1'b1;			// 5.0 Gb/s fixed
		phy_tx_swing <= 		SWING_FULL;		// full swing
		phy_rx_termination <= 	1'b1;			// enable rx termination
		phy_elas_buf_mode <= 	ELASBUF_HALF;	// elastic buffer nominally half full

		ds_enable <= 0;							// disable descrambling
		s_enable <= 0;							// disable scrambling
		scr_mux <= 0;							// switch TX mux to local PIPE layer

		ts_disable_scrambling_latch <= 0;
		hot_reset_count <= 0;
		ltssm_hot_reset <= 0;

		state <= ST_RST_1;
		dc <= 0;
	end
	ST_RST_1: begin
		// wait for PHY's initial poweron reset notification
		if(pipe_phy_status[1] | pipe_phy_status[0]) state <= ST_RST_2;
	end
	ST_RST_2: begin
		// once PHY deasserts status, it's ready
		if(~pipe_phy_status[1] | ~pipe_phy_status[0]) state <= ST_RST_3;
	end
	ST_RST_3: begin
		// phy is ready, bring LTSSM out of reset
		ltssm_reset_n <= 1;
		state <= ST_IDLE;
	end

	ST_IDLE: begin
		// disable scrambling
		s_enable <= 0;
		ds_enable <= 0;
		// squash idle in P0
		if(phy_power_down == POWERDOWN_0) phy_tx_elecidle_local <= 1'b0;

		// LTSSM wants to initiate link training!
		if(ltssm_training) begin
			// grab tx mux from link layer
			scr_mux <= 0;

			phy_tx_elecidle_local <= 1'b0;

			// Polling.RxEq
			if(ltssm_train_rxeq) begin
				swc <= 0;
				train_count <= 0;
				state <= ST_TRAIN_RXEQ_0;
			end
			// Polling.Active, Polling.Config
			if(ltssm_train_active | ltssm_train_config) begin
				swc <= 0;
				set_ts <= 0;
				train_count <= 0;
				// reset SKP elastic buffer compensation
				train_sym_skp <= 0;
				state <= ST_TRAIN_ACTIVECONFIG_0;
			end
			// Polling.Idle
			if(ltssm_train_idle) begin
				swc <= 0;
				// enable de/scrambling
				ds_enable <= 1;
				s_enable <= 1;
				if(ts_disable_scrambling) ds_enable <= 0;
				if(ts_disable_scrambling) s_enable <= 0;
				idle_symbol_send <= 0;
				idle_symbol_recv <= 0;

				local_tx_active <= 1;
				local_tx_data <= 32'h0;
				local_tx_datak <= 4'b0;
				state <= ST_TRAIN_IDLE_0;
			end
		end
	end

	ST_TRAIN_RXEQ_0: begin
		// transmitting TSEQ
		// N.B. this is just COM + scrambled 00.
		phy_tx_elecidle_local <= 1'b0;
		local_tx_active <= 1;
		case(swc)
		0: {local_tx_data, local_tx_datak} <= {32'hBCFF17C0, 4'b1000};
		1: {local_tx_data, local_tx_datak} <= {32'h14B2E702, 4'b0000};
		2: {local_tx_data, local_tx_datak} <= {32'h82726E28, 4'b0000};
		3: {local_tx_data, local_tx_datak} <= {32'hA6BE6DBF, 4'b0000};
		4: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
		5: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
		6: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
		7: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
		endcase

		if(swc == 7) begin
			// increment send count and repeat
			swc <= 0;
			`INC(train_count);
			if(train_count == 65535) begin
				// proceed
				state <= ST_TRAIN_RXEQ_1;
			end
		end
		// allow for swapped lane polarity on receiver
		// if RX polarity was swapped, then D10.2 (0x4A) in TSEQ would appear as D21.5 (0xB5).
		if(pipe_rx_data == 32'hB5B5B5B5) phy_rx_polarity <= 1;
	end
	ST_TRAIN_RXEQ_1: begin
		// allow several cycles for slower FSM in other domain to deassert training signal,
		// and inhibit repeated training
		phy_tx_elecidle_local <= 1'b0;

		ltssm_train_rxeq_pass <= 1;
		if(swc == 7) state <= ST_IDLE;
	end

	ST_TRAIN_ACTIVECONFIG_0: begin
		// transmitting TS1
		phy_tx_elecidle_local <= 1'b0;
		local_tx_active <= 1;
		ds_enable <= 0;

		if(~set_ts) begin
			case(swc)
			0: {local_tx_data, local_tx_datak} <= {32'hBCBCBCBC, 4'b1111};
			1: {local_tx_data, local_tx_datak} <= {32'h00004A4A, 4'b0000};
			2: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
			3: {local_tx_data, local_tx_datak} <= {32'h4A4A4A4A, 4'b0000};
			endcase
		end else begin
			case(swc)
			0: {local_tx_data, local_tx_datak} <= {32'hBCBCBCBC, 4'b1111};
			1: {local_tx_data, local_tx_datak} <= {8'h00, {4'h0, 3'h0, hot_reset_count > 0}, 16'h4545, 4'b0000};
			2: {local_tx_data, local_tx_datak} <= {32'h45454545, 4'b0000};
			3: {local_tx_data, local_tx_datak} <= {32'h45454545, 4'b0000};
			endcase
		end

		if(swc == 3) begin
			// end of ordered set, repeat
			swc <= 0;
			if(hot_reset_count > 0) `DEC(hot_reset_count);
			// increment symbols sent for SKP compensation
			train_sym_skp <= train_sym_skp + 13'd16;
			if(train_sym_skp >= (354*2-16)) begin
				// time to insert SKP ordered set
				state <= ST_TRAIN_ACTIVECONFIG_1;
			end
			// change sending set from TS1 to TS2
			set_ts <= ltssm_train_config;

			if(!(ltssm_train_active || ltssm_train_config)) begin
				// LTSSM has aborted or timed out
				state <= ST_IDLE;
			end
		end
	end
	ST_TRAIN_ACTIVECONFIG_1: begin
		// transmitting SKP ordered set
		// transmit two ordered sets so that we stay 32bit aligned.
		// this will throw off the remote elastic buffer normally but isn't a
		// problem during training
		phy_tx_elecidle_local <= 1'b0;
		local_tx_active <= 1;
		{local_tx_data, local_tx_datak} <= {32'h3C3C3C3C, 4'b1111};
		// decrement overflow counter
		train_sym_skp <= train_sym_skp - 13'd354*2+32;
		// reset sequence index
		swc <= 0;
		state <= ST_TRAIN_ACTIVECONFIG_0;
	end

	ST_TRAIN_IDLE_0: begin
		phy_tx_elecidle_local <= 1'b0;
		local_tx_active <= 1;
		{local_tx_data, local_tx_datak} <= {32'h00000000, 4'b0000};

		if(idle_symbol_send < 4) `INC(idle_symbol_send);
		// our descrambler takes a cycle or two to switch on.
		// some host controllers send link commands immediately after the last TS2 symbol
		// and the requisite 8 idle symbols.
		// solution is just to enter U0 immediately and put the smarts into the link layer
		/*
		if(sync_out != 32'h00000000) begin
			// non-IDLE symbol received
			idle_symbol_send <= 0;
		end else begin
			if(idle_symbol_recv < 2) `INC(idle_symbol_recv);
		end
		if(idle_symbol_send == 4 && idle_symbol_recv == 2) begin
			// exit conditions matching those of LTSSM
			state <= ST_TRAIN_IDLE_1;
		end
		*/
		// Once other port sends Idle then it has exit training. Wait for this or we'll just go
		// right back into Recovery
		// N.B. this requires a few cycles and may cause the miss of the first Link Command
		// on some very fast links. More testing is needed TODO
		if(idle_symbol_send == 4 && {sync_out, sync_outk} == {32'h00000000, 4'b0000}) begin
			state <= ST_TRAIN_IDLE_1;
		end
		if(!ltssm_training) begin
			// LTSSM has aborted or timed out
			state <= ST_IDLE;
		end
	end
	ST_TRAIN_IDLE_1: begin
		phy_tx_elecidle_local <= 1'b0;
		ltssm_train_idle_pass <= 1;
		if(!ltssm_train_idle) state <= ST_U0;
	end

	ST_U0: begin
		phy_tx_elecidle_local <= 1'b0;

		// pass tx mux to link layer
		scr_mux <= 1;

		ltssm_hot_reset <= 0;

		case(ltssm_state)
		LT_U0: state <= ST_U0;
		LT_U1: state <= ST_U1;
		LT_U2: state <= ST_U2;
		LT_U3: state <= ST_U3;
		default: state <= ST_IDLE;
		endcase
	end
	ST_U1: begin
		case(ltssm_state)
		default: state <= ST_IDLE;
		endcase
	end
	ST_U2: begin
		case(ltssm_state)
		default: state <= ST_IDLE;
		endcase
	end
	ST_U3: begin
		case(ltssm_state)
		default: state <= ST_IDLE;
		endcase
	end

	default: state <= ST_RST_0;
	endcase



	///////////////////////////////////////
	// ORDERED SET ALIGNMENT FSM
	///////////////////////////////////////

	if(proc_active) begin
		case(word_rx_align)
		0: begin
			sync_a <= {proc_data[31:0]};
			sync_ak <= proc_datak;
			sync_b <= sync_a;
			sync_bk <= sync_ak;
		end
		1: begin
			sync_a <= {proc_data[23:0], 8'h0};
			sync_ak <= {proc_datak[2:0], 1'h0};
			sync_b <= {sync_a[31:8], proc_data[31:24]};
			sync_bk <= {sync_ak[3:1], proc_datak[3]};
		end
		2: begin
			sync_a <= {proc_data[15:0], 16'h0};
			sync_ak <= {proc_datak[1:0], 2'h0};
			sync_b <= {sync_a[31:16], proc_data[31:16]};
			sync_bk <= {sync_ak[3:2], proc_datak[3:2]};
		end
		3: begin
			sync_a <= {proc_data[7:0], 24'h0};
			sync_ak <= {proc_datak[0], 3'h0};
			sync_b <= {sync_a[31:24], proc_data[31:8]};
			sync_bk <= {sync_ak[3:3], proc_datak[3:1]};
		end
		endcase
	end

	case(align_state)
	ALIGN_RESET: begin
		align_state <= ALIGN_IDLE;
	end
	ALIGN_IDLE: begin

		// packet framing detected
		// determine bit alignment
		if( sync_start[0]) begin
			//if( &sync_start[3:1] ) begin				// 1111
			//	word_rx_align <= 0;
			//	sync_a <= proc_data;
			//	sync_ak <= proc_datak;
			//end else
			if( &sync_start[2:1] ) begin				 // 0111
				word_rx_align <= 1;
				sync_a <= {proc_data[23:0], 8'h0};
				sync_ak <= {proc_datak[2:0], 1'h0};
			end else if( &sync_start[1] ) begin			// 0011
				word_rx_align <= 2;
				sync_a <= {proc_data[15:0], 16'h0};
				sync_ak <= {proc_datak[1:0], 2'h0};
			end else begin								// 0001
				word_rx_align <= 3;
				sync_a <= {proc_data[7:0], 24'h0};
				sync_ak <= {proc_datak[0], 3'h0};
			end
		end

		if( &sync_start[3:1] & sync_end[0]) begin		// 1111
			word_rx_align <= 0;
			sync_a <= proc_data;
			sync_ak <= proc_datak;
		end
	end
	default: align_state <= ALIGN_RESET;
	endcase

	sync_a_active <= proc_active;
	sync_b_active <= sync_a_active;
	sync_out_active <= sync_a_active; // yes, intentional ??!!
	sync_out <= sync_b;
	sync_outk <= sync_bk;



	///////////////////////////////////////
	// TRAINING SEQUENCE DETECTION
	///////////////////////////////////////

	case(tsdet_state)
	TSDET_RESET: begin
		ts_hot_reset_latch <= 0;
		ts_hot_reset_local <= 0;
		tsdet_state <= TSDET_IDLE;
	end
	TSDET_IDLE: begin
		if(sync_out_active) begin
			tsdet_state <=
				{sync_out, sync_outk} == {32'hBCBCBCBC, 4'b1111} ? TSDET_0 : TSDET_IDLE;
		end
		ts_disable_scrambling <= ts_disable_scrambling_latch && set_ts2_found;
		//ts_hot_reset_local <= ts_hot_reset_latch && set_ts2_found;
		if(set_ts2_found) begin
			// update whether or not the Reset bit was set
			// this is used by the LTSSM to decide whether to proceed to U0 or HotReset
			ts_hot_reset_local <= ts_hot_reset_latch;

			if(ts_hot_reset_latch) ltssm_hot_reset <= 1;
		end
	end
	TSDET_0: begin
		if(sync_out_active) begin
			tsdet_state <=
				{sync_out[31:20], 4'b0, sync_out[15:0], sync_outk} == {32'h00004A4A, 4'b0000} ? TSDET_1 :
				{sync_out[31:20], 4'b0, sync_out[15:0], sync_outk} == {32'h00004545, 4'b0000} ? TSDET_1 : TSDET_IDLE;
			if(ltssm_state == LT_RECOVERY_IDLE || ltssm_state == LT_POLLING_IDLE) begin
				// only allow latching of Disable Scrambling in these two LTSSM states
				ts_disable_scrambling_latch <= sync_out[19];
			end
			ts_hot_reset_latch <= sync_out[16];
		end
	end
	TSDET_1: begin
		if(sync_out_active) begin
			tsdet_state <=
				{sync_out, sync_outk} == {32'h4A4A4A4A, 4'b0000} ? TSDET_2 :
				{sync_out, sync_outk} == {32'h45454545, 4'b0000} ? TSDET_2 : TSDET_IDLE;
		end
	end
	TSDET_2: begin
		if(sync_out_active) begin
			set_ts1_found <= {sync_out, sync_outk} == {32'h4A4A4A4A, 4'b0000};
			set_ts2_found <= {sync_out, sync_outk} == {32'h45454545, 4'b0000};
			tsdet_state <= TSDET_IDLE;
		end
	end
	endcase
	// if PHY is not in P0 then reset TSDET FSM
	if(phy_power_down != POWERDOWN_0) tsdet_state <= TSDET_RESET;


	///////////////////////////////////////
	// RX DETECTION HANDLING FOR LTSSM
	///////////////////////////////////////

	partner_looking <= 0;
	partner_detected <= 0;

	case(rxdet_state)
	RXDET_RESET: begin
		rxdet_state <= RXDET_IDLE;
	end
	RXDET_IDLE: begin

		if(partner_detect & ~partner_detect_1) begin
			// new rising edge from LTSSM
			// it wants us to perform a receiver check
			rdc <= 0;
			rxdet_timeout <= 0;
			rxdet_state <= RXDET_0;
			partner_looking <= 1;

			if(phy_power_down == POWERDOWN_0 || phy_power_down == POWERDOWN_1) begin
				// if in P0, P1 assume we're already connected anyway
				rxdet_state <= RXDET_3;
			end
		end
	end
	RXDET_0: begin
		// NOTE: TX_ELECIDLE must be asserted while this is run,
		// or loopback mode is entered
		// Also, PHY must already be in P2 or P3 mode

		partner_looking <= 1;
		// delay the detection attempt
		if(rdc == 31) begin
			rdc <= 31;
			phy_tx_detrx_lpbk_local <= 1'b1;

			// check RX_STATUS corresponding to this cycle
			if(pipe_phy_status[1]) begin
				if(pipe_rx_status[5:3] == 3'b011)
					partner_detected <= 1;
				else
					partner_detected <= 0;
				rxdet_state <= RXDET_1;

			end else if( pipe_phy_status[0]) begin
				if(pipe_rx_status[2:0] == 3'b011)
					partner_detected <= 1;
				else
					partner_detected <= 0;
				rxdet_state <= RXDET_1;
			end

			if(rxdet_timeout[20]) begin
				// it's been 8ms and no response! bail
				rxdet_state <= RXDET_1;
			end
		end
	end
	RXDET_1: begin
		partner_looking <= 0;
		// maintain value
		partner_detected <= partner_detected;
		rxdet_state <= RXDET_2;
	end
	RXDET_2: begin
		partner_detected <= partner_detected;
		rxdet_state <= RXDET_IDLE;
	end
	RXDET_3: begin
		// fake a receiver detect while running
		if(rdc < 2)
			partner_looking <= 1;
		else
			partner_detected <= 1;
		if(rdc == 10) rxdet_state <= RXDET_IDLE;
	end
	default: rxdet_state <= RXDET_RESET;
	endcase



	///////////////////////////////////////
	// PORT POWERDOWN (PHY) MGMT FSM
	///////////////////////////////////////

	ltssm_power_ack <= 0;
	ltssm_power_err <= 0;

	case(pd_state)
	PD_RESET: begin
		pd_state <= PD_IDLE;
	end
	PD_IDLE: begin
		if(ltssm_power_go & ~ltssm_power_go_1) begin
			// new rising edge from LTSSM
			// it wants to change the POWERDOWN state of the PHY
			pd_state <= PD_0;

			if(ltssm_power_down == phy_power_down) begin
				// already in the requested state
				pd_state <= PD_2;
			end
		end
	end
	PD_0: begin
		phy_power_down <= ltssm_power_down;

		// check RX_STATUS corresponding to this cycle
		if(pipe_phy_status[1] | pipe_phy_status[0]) begin
			// PHY asserted status acknowledging the powerstate change
			ltssm_power_ack <= 1;
			pd_state <= PD_1;
		end
	end
	PD_1: begin
		ltssm_power_ack <= 1;
		pd_state <= PD_IDLE;
	end
	PD_2: begin
		ltssm_power_ack <= 1;
		pd_state <= PD_3;
	end
	PD_3: begin
		ltssm_power_ack <= 1;
		pd_state <= PD_IDLE;
	end
	default: pd_state <= PD_RESET;
	endcase



	if(~reset_n) begin
		// reset
		state <= ST_RST_0;
		align_state <= ALIGN_RESET;
		tsdet_state <= TSDET_RESET;
		rxdet_state <= RXDET_RESET;
		pd_state <= PD_RESET;
	end

end


//
// RX descramble and filtering
//
	wire	[3:0]	proc_datak  /* synthesis keep */;
	wire	[31:0]	proc_data /* synthesis keep */;
	wire			proc_active /* synthesis keep */;
usb3_descramble iu3rds (
	.local_clk		( local_clk ),
	.reset_n		( reset_n ),
	.enable			( ds_enable ),

	.raw_valid		( pipe_rx_valid ),
	.raw_status		( pipe_rx_status ),
	.raw_phy_status	( pipe_phy_status ),
	.raw_datak		( pipe_rx_datak ),
	.raw_data		( pipe_rx_data ),

	.proc_data		( proc_data ),
	.proc_datak		( proc_datak ),
	.proc_active	( proc_active )
);


//
// TX scramble and padding
//
	wire	[3:0]	send_datak  /* synthesis keep */;
	wire	[31:0]	send_data /* synthesis keep */;

	reg		[3:0]	local_tx_datak;
	reg		[31:0]	local_tx_data;
	reg				local_tx_active;
	reg				local_tx_skp_inhibit;
	reg				local_tx_skp_defer;

	reg				scr_mux;

	wire	[3:0]	scr_mux_datak 	= scr_mux ? link_out_datak : local_tx_datak;
	wire	[31:0]	scr_mux_data 	= scr_mux ? link_out_data : local_tx_data;
	wire			scr_mux_active 	= scr_mux ? link_out_active : local_tx_active;
	wire			scr_mux_inhibit = scr_mux ? link_out_skp_inhibit : local_tx_skp_inhibit;
	wire			scr_mux_defer 	= scr_mux ? link_out_skp_defer : local_tx_skp_defer;

usb3_scramble iu3ss (
	.local_clk		( local_clk ),
	.reset_n		( reset_n ),
	.enable			( s_enable ),

	.raw_datak		( scr_mux_datak  ),
	.raw_data		( scr_mux_data ),
	.raw_active		( scr_mux_active ),
	.raw_stall		( link_out_stall ),

	.skp_inhibit	( scr_mux_inhibit ),
	.skp_defer		( scr_mux_defer ),

	.proc_data		( send_data ),
	.proc_datak		( send_datak )
);


//
// ALT_DDIO megafunction instantiations
// these are used to clock the PIPE bus and related 250mhz signals
// at the more leisurely 125mhz domain from the parent's PLL
//

//
// TRANSMIT
//
	// byteswap the outgoing data to the proper order
	wire	[31:0]	pipe_tx_data_swap	= {	pipe_tx_data[7:0], pipe_tx_data[15:8],
											pipe_tx_data[23:16], pipe_tx_data[31:24]};
	wire	[3:0]	pipe_tx_datak_swap	= { pipe_tx_datak[0], pipe_tx_datak[1],
											pipe_tx_datak[2], pipe_tx_datak[3] };
	// send processed data from tx scrambler module
	wire	[31:0]	pipe_tx_data	= send_data;
	wire	[3:0]	pipe_tx_datak	= send_datak;

	assign			phy_pipe_tx_data	= pipe_tx_data_swap;
	assign			phy_pipe_tx_datak	= pipe_tx_datak_swap;

//
// RECEIVE
//
	wire	[1:0]	pipe_rx_valid		= phy_pipe_rx_valid;
	wire	[5:0]	pipe_rx_status		= phy_rx_status;
	wire	[1:0]	pipe_phy_status		= phy_phy_status;
	wire	[3:0]	pipe_rx_datak_swap	= phy_pipe_rx_datak;
	wire	[31:0]	pipe_rx_data_swap	= phy_pipe_rx_data;

	wire	[3:0]	pipe_rx_datak = {	pipe_rx_datak_swap[0], pipe_rx_datak_swap[1],
										pipe_rx_datak_swap[2], pipe_rx_datak_swap[3] };
	wire	[31:0]	pipe_rx_data = {	pipe_rx_data_swap[7:0], pipe_rx_data_swap[15:8],
										pipe_rx_data_swap[23:16], pipe_rx_data_swap[31:24] };

endmodule
