
//
// usb 3.0 constants include file
//
// Copyright (c) 2013 Marshall H.
// All rights reserved.
// This code is released under the terms of the simplified BSD license. 
// See LICENSE.TXT for details.
//


//
// LTSSM state definitions
//
parameter	[4:0]	LT_SS_DISABLED			= 'd01,
					LT_SS_INACTIVE			= 'd02,
					LT_SS_INACTIVE_QUIET 	= 'd03,
					LT_SS_INACTIVE_DETECT_0	= 'd04,
					LT_SS_INACTIVE_DETECT_1	= 'd05,
					LT_RX_DETECT_RESET		= 'd06,
					LT_RX_DETECT_ACTIVE_0	= 'd07,
					LT_RX_DETECT_ACTIVE_1	= 'd08,
					LT_RX_DETECT_QUIET		= 'd09,
					LT_POLLING_LFPS			= 'd10,
					LT_POLLING_RXEQ_0		= 'd11,
					LT_POLLING_RXEQ_1		= 'd12,
					LT_POLLING_ACTIVE		= 'd13,
					LT_POLLING_CONFIG		= 'd14,
					LT_POLLING_IDLE			= 'd15,
					LT_U0					= 'd16,
					LT_U1					= 'd17,
					LT_U2					= 'd18,
					LT_U3					= 'd19,
					LT_COMPLIANCE			= 'd20,
					LT_LOOPBACK				= 'd21,
					LT_HOTRESET				= 'd22,
					LT_HOTRESET_ACTIVE		= 'd23,
					LT_HOTRESET_EXIT		= 'd24,
					LT_RECOVERY				= 'd25,
					LT_RECOVERY_WAIT		= 'd26,
					LT_RECOVERY_ACTIVE		= 'd27,
					LT_RECOVERY_CONFIG		= 'd28,
					LT_RECOVERY_IDLE		= 'd29,
					LT_RESET				= 'd30,
					LT_LAST					= 'b11111;

parameter	[4:0]	LFPS_RESET			= 'h00,
					LFPS_IDLE			= 'h01,
					LFPS_RECV_1			= 'h02,
					LFPS_RECV_2			= 'h03,
					LFPS_RECV_3			= 'h04,
					LFPS_SEND_1			= 'h05,
					LFPS_SEND_2			= 'h06,
					LFPS_SEND_3			= 'h07,
					LFPS_0				= 'h08,
					LFPS_LAST			= 'b11111;
					
// timing parameters
// all are calculated for local clock of 62.5 MHz (1/4 PIPE CLK)
// except those denoted by *2 which compensates for 125 MHz Link clock domain.
//
parameter	[23:0]	LFPS_POLLING_MIN	= 'd37;			// 0.6 uS (nom 1.0 us)
parameter	[23:0]	LFPS_POLLING_NOM	= 'd62;			// 1.0 uS
parameter	[23:0]	LFPS_POLLING_MAX	= 'd87;			// 1.4 uS
parameter	[23:0]	LFPS_PING_MIN		= 'd2;			// 40 ns
parameter	[23:0]	LFPS_PING_NOM		= 'd6;			// 96 ns
parameter	[23:0]	LFPS_PING_MAX		= 'd12;			// 200 ns
parameter	[23:0]	LFPS_RESET_MIN		= 'd5000000;	// 80 ms (nom 100 ms)
parameter	[23:0]	LFPS_RESET_DELAY	= 'd1875000;	// 30 ms
parameter	[23:0]	LFPS_RESET_MAX		= 'd7500000;	// 120 ms
parameter	[23:0]	LFPS_U1EXIT_MIN		= 'd36;			// 656 ns // now 616ns due to lecroy
parameter	[23:0]	LFPS_U1EXIT_NOM		= 'd42;			// 656 ns // was 625*4 in testing
parameter	[23:0]	LFPS_U1EXIT_MAX		= 'd125000;		// 2 ms (1.5, fudged)
parameter	[23:0]	LFPS_U2LBEXIT_MIN	= 'd5100;		// 80 us
parameter	[23:0]	LFPS_U2LBEXIT_NOM	= 'd6250;		// 100 us // 5362
parameter	[23:0]	LFPS_U2LBEXIT_MAX	= 'd125000+62500;		// 2 ms
parameter	[23:0]	LFPS_U3WAKEUP_MIN	= 'd5000;		// 80 us
parameter	[23:0]	LFPS_U3WAKEUP_NOM	= 'd62500;		// 1 ms
parameter	[23:0]	LFPS_U3WAKEUP_MAX	= 'd625000;		// 10 ms

parameter	[23:0]	LFPS_BURST_POLL_MIN	= 'd375;		// 6 us (nom 10 uS)
parameter	[23:0]	LFPS_BURST_POLL_NOM	= 'd625;		// 10 uS
parameter	[23:0]	LFPS_BURST_POLL_MAX	= 'd875;		// 14 us
parameter	[23:0]	LFPS_BURST_PING_MIN	= 'd10000000;	// 160 ms (nom 200 ms)
parameter	[23:0]	LFPS_BURST_PING_NOM	= 'd12500000;	// 200 ms
parameter	[23:0]	LFPS_BURST_PING_MAX	= 'd15000000;	// 240 ms

parameter	[24:0]	T_SS_INACTIVE_QUIET	= 'd750000;		// 12 ms
parameter	[24:0]	T_RX_DETECT_QUIET	= 'd7500000;	// 120 ms
parameter	[24:0]	T_POLLING_LFPS		= 'd22500000;	// 360 ms
parameter	[24:0]	T_POLLING_ACTIVE	= 'd750000;		// 12 ms
parameter	[24:0]	T_POLLING_CONFIG	= 'd750000;		// 12 ms
parameter	[24:0]	T_POLLING_IDLE		= 'd125000;		// 2 ms
parameter	[24:0]	T_U0_RECOVERY		= 'd125000;		// 1 ms			used in Link Layer, double cycles
parameter	[24:0]	T_U0L_TIMEOUT		= 'd1250;		// 10 us		used in Link Layer, double cycles
parameter	[24:0]	T_PENDING_HP		= 'd376;		// 3 us			used in Link Layer, double cycles
parameter	[24:0]	T_CREDIT_HP			= 'd625000;		// 5000 us		used in Link Layer, double cycles
parameter	[24:0]	T_NOLFPS_U1			= 'd125000;		// 2 ms
parameter	[24:0]	T_U1_PING			= 'd18750000;	// 300 ms
parameter	[24:0]	T_NOLFPS_U2			= 'd125000;		// 2 ms
parameter	[24:0]	T_NOLFPS_U3			= 'd625000;		// 10 ms
parameter	[24:0]	T_RECOV_ACTIVE		= 'd750000;		// 12 ms
parameter	[24:0]	T_RECOV_CONFIG		= 'd375000;		// 6 ms
parameter	[24:0]	T_RECOV_IDLE		= 'd125000;		// 2 ms
parameter	[24:0]	T_LOOPBACK_EXIT		= 'd125000;		// 2 ms

parameter	[24:0]	T_PM_LC				= 'd376;		// 3 us			used in Link Layer, double cycles
parameter	[24:0]	T_PM_ENTRY			= 'd752;		// 6 us			used in Link Layer, double cycles
parameter	[24:0]	T_UX_EXIT			= 'd750000;		// 6 ms			used in Link Layer, double cycles

parameter	[24:0]	T_PORT_CONFIG		= 'd2550;		// 20 us		used in Link Layer, double cycles

parameter	[1:0]	POWERDOWN_0			= 2'd0,		// active transmitting
					POWERDOWN_1			= 2'd1,		// slight powerdown	
					POWERDOWN_2			= 2'd2,		// slowest
					POWERDOWN_3			= 2'd3;		// deep sleep, clock stopped
parameter			SWING_FULL			= 1'b0,		// transmitter voltage swing
					SWING_HALF			= 1'b1;
parameter			ELASBUF_HALF		= 1'b0,		// elastic buffer mode
					ELASBUF_EMPTY		= 1'b1;
parameter	[2:0]	MARGIN_A			= 3'b000,	// Normal range
					MARGIN_B			= 3'b001,	// Decreasing voltage levels
					MARGIN_C			= 3'b010,	// See PHY datasheet
					MARGIN_D			= 3'b011,
					MARGIN_E			= 3'b100;			
parameter	[1:0]	DEEMPH_6_0_DB		= 2'b00,	// -6.0dB TX de-emphasis
					DEEMPH_3_5_DB		= 2'b01,	// -3.5dB TX de-emphasis
					DEEMPH_NONE			= 2'b10,	// no TX de-emphasis
					DEEMPH_RESVD		= 2'b11;	
					
//
// Link constants
//

parameter	[10:0]	LCMD_LGOOD			= 'b00_00_000_0_xxx,
					LCMD_LGOOD_0		= 'b00_00_000_0_000,
					LCMD_LGOOD_1		= 'b00_00_000_0_001,
					LCMD_LGOOD_2		= 'b00_00_000_0_010,
					LCMD_LGOOD_3		= 'b00_00_000_0_011,
					LCMD_LGOOD_4		= 'b00_00_000_0_100,
					LCMD_LGOOD_5		= 'b00_00_000_0_101,
					LCMD_LGOOD_6		= 'b00_00_000_0_110,
					LCMD_LGOOD_7		= 'b00_00_000_0_111;
					
parameter	[10:0]	LCMD_LCRD			= 'b00_01_000_00_xx,
					LCMD_LCRD_A			= 'b00_01_000_00_00,
					LCMD_LCRD_B			= 'b00_01_000_00_01,
					LCMD_LCRD_C			= 'b00_01_000_00_10,
					LCMD_LCRD_D			= 'b00_01_000_00_11;
					
parameter	[10:0]	LCMD_LRTY			= 'b00_10_000_0000,
					LCMD_LBAD			= 'b00_11_000_0000;
					
parameter	[10:0]	LCMD_LGO			= 'b01_00_000_00xx,
					LCMD_LGO_U1			= 'b01_00_000_0001,
					LCMD_LGO_U2			= 'b01_00_000_0010,
					LCMD_LGO_U3			= 'b01_00_000_0011;
					
parameter	[10:0]	LCMD_LAU			= 'b01_01_000_0000,
					LCMD_LXU			= 'b01_10_000_0000,
					LCMD_LPMA			= 'b01_11_000_0000;
					
parameter	[10:0]	LCMD_LDN			= 'b10_11_000_0000,
					LCMD_LUP			= 'b10_00_000_0000;
					
parameter	[4:0]	LP_TYPE_LMP			= 'b00000,				
					LP_TYPE_TP			= 'b00100,
					LP_TYPE_DP			= 'b01000,
					LP_TYPE_ITP			= 'b01100;

parameter	[3:0]	LP_LMP_SUB_RSVD			= 'b0000,
					LP_LMP_SUB_SETLINK		= 'b0001,
					LP_LMP_SUB_U2INACT		= 'b0010,
					LP_LMP_SUB_VENDTEST		= 'b0011,
					LP_LMP_SUB_PORTCAP		= 'b0100,
					LP_LMP_SUB_PORTCFG		= 'b0101,
					LP_LMP_SUB_PORTCFGRSP	= 'b0110;
					
parameter	[6:0]	LP_LMP_SPEED_5GBPS		= 'b000000_1;
parameter	[6:0]	LP_LMP_SPEED_DECLINE	= 'b000000_0;
parameter	[6:0]	LP_LMP_SPEED_ACCEPT		= 'b000000_1;
parameter	[7:0]	LP_LMP_NUM_HP_4			= 'd4;
parameter	[1:0]	LP_LMP_DIR_DOWN			= 'b01;
parameter	[1:0]	LP_LMP_DIR_UP			= 'b10;
parameter	[0:0]	LP_LMP_OTG_INCAPABLE	= 'b0;
parameter	[0:0]	LP_LMP_OTG_CAPABLE		= 'b1;
parameter	[3:0]	LP_LMP_TIEBREAK			= 'b0000;

parameter	[3:0]	LP_TP_SUB_RSVD			= 'b0000,
					LP_TP_SUB_ACK			= 'b0001,
					LP_TP_SUB_NRDY			= 'b0010,
					LP_TP_SUB_ERDY			= 'b0011,
					LP_TP_SUB_STATUS		= 'b0100,
					LP_TP_SUB_STALL			= 'b0101,
					LP_TP_SUB_DEVNOTIFY		= 'b0110,
					LP_TP_SUB_PING			= 'b0111,
					LP_TP_SUB_PINGRSP		= 'b1000;
					
parameter	[19:0]	LP_TP_ROUTE0			= 'b0;
parameter	[0:0]	LP_TP_NORETRY			= 'b0,
					LP_TP_RETRY				= 'b1;
parameter	[0:0]	LP_TP_HOSTTODEVICE		= 'b0,
					LP_TP_DEVICETOHOST		= 'b1;
parameter	[15:0]	LP_TP_STREAMID			= 'h0;
//
// the following are for isochronous endpoints only (Table 8-12)
//
parameter	[0:0]	LP_TP_SSI_NO			= 'b0,	// Support Smart Isochronous
					LP_TP_SSI_YES			= 'b1;
parameter	[0:0]	LP_TP_WPA_NO			= 'b0,	// Will Ping Again
					LP_TP_WPA_YES			= 'b1;
parameter	[0:0]	LP_TP_DBI_NO			= 'b0,	// Data in Bus Interval Done
					LP_TP_DBI_YES			= 'b1;
//
parameter	[0:0]	LP_TP_PPEND_NO			= 'b0,	// Packets Pending from Host
					LP_TP_PPEND_YES			= 'b1;
parameter	[3:0]	LP_TP_NBI_0				= 'b0;	// Number of Bus Intervals

parameter	[3:0]	LP_TP_DN_FUNC_AWAKE		= 'b0001,
					LP_TP_DN_LAT_TOL_MSG	= 'b0010,
					LP_TP_DN_BUS_INTER_ADJ	= 'b0011,
					LP_TP_DN_HOST_ROLE_REQ	= 'b0100;
					
parameter	[0:0]	LP_DP_EOB_LPF_NO		= 1'b0,
					LP_DP_EOB_LPF_YES		= 1'b1;
		
function	[15:0]	swap16;
	input	[15:0]	i;
	swap16 = {i[7:0], i[15:8]};
endfunction

function	[31:0]	swap32;
	input	[31:0]	i;
	swap32 = {i[7:0], i[15:8], i[23:16], i[31:24]};
endfunction

`define INC(x) x<=x+1'b1
`define DEC(x) x<=x-1'b1

//
// end usb3_const.vh
//
