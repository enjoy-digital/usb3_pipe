#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2013 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen import *

# -------------------------------------------------------------------------------------------------
# usb3_ep0 (LiteX/Migen translation of Daisho usb3_ep0)
# -------------------------------------------------------------------------------------------------
# Notes:
# - Synchronous process in cd_usb3 driven by local_clk / reset_n (as in original Verilog).
# - RAM/ROM are instantiated as Verilog blocks (usb3_ep0in_ram, usb3_descrip_rom).
# - Handshake outputs match Verilog: buf_in_ready, buf_in_commit_ack, buf_out_hasdata/len, arm_ack.
# - Setup packet field extraction matches Verilog byte order.
# -------------------------------------------------------------------------------------------------


# ---- usb3_const.vh subset -----------------------------------------------------------------------

SETUP_DIR_HOSTTODEV = 0
SETUP_DIR_DEVTOHOST = 1

SETUP_TYPE_STANDARD = 0
SETUP_TYPE_CLASS    = 1
SETUP_TYPE_VENDOR   = 2
SETUP_TYPE_RESVD    = 3

# Standard requests (subset used by ep0).
REQ_CLEAR_FEAT     = 0x01
REQ_SET_FEAT       = 0x03
REQ_SET_ADDR       = 0x05
REQ_GET_DESCR      = 0x06
REQ_GET_CONFIG     = 0x08
REQ_SET_CONFIG     = 0x09
REQ_SET_INTERFACE  = 0x0B
REQ_SET_SEL        = 0x30


# ---- usb_descrip.vh (USB3 constants you provided) -----------------------------------------------

DESCR_USB3_DEVICE      = 0
DESCR_USB3_CONFIG      = 5
DESCR_USB3_CONFIG_LEN  = 53
DESCR_USB3_BOS         = 19
DESCR_USB3_BOS_LEN     = 22
DESCR_USB3_STRING0     = 25
DESCR_USB3_STRING1     = 26
DESCR_USB3_STRING2     = 36
DESCR_USB3_STRING3     = 44
DESCR_USB3_CONFUNSET   = 51
DESCR_USB3_CONFSET     = 52
DESCR_USB3_EOF         = 53


class USB3EP0(LiteXModule):
    def __init__(self):
        # -----------------------------------------------------------------------------------------
        # Ports (match Verilog)
        # -----------------------------------------------------------------------------------------
        self.slow_clk  = Signal()
        self.local_clk = Signal()
        self.reset_n   = Signal()

        self.buf_in_addr       = Signal(9)
        self.buf_in_data       = Signal(32)
        self.buf_in_wren       = Signal()
        self.buf_in_ready      = Signal()
        self.buf_in_commit     = Signal()
        self.buf_in_commit_len = Signal(11)
        self.buf_in_commit_ack = Signal()

        self.buf_out_addr      = Signal(9)
        self.buf_out_q         = Signal(32)
        self.buf_out_len       = Signal(11)
        self.buf_out_hasdata   = Signal()
        self.buf_out_arm       = Signal()
        self.buf_out_arm_ack   = Signal()

        self.vend_req_act      = Signal()
        self.vend_req_request  = Signal(8)
        self.vend_req_val      = Signal(16)
        self.vend_req_index    = Signal(16)

        self.dev_addr          = Signal(7)
        self.configured        = Signal()
        self.reset_dp_seq      = Signal()

        self.err_setup_pkt     = Signal()

        # -----------------------------------------------------------------------------------------
        # Clock domain (cd_usb3 := local_clk)
        # -----------------------------------------------------------------------------------------
        self.clock_domains.cd_usb3 = ClockDomain()
        self.comb += [
            self.cd_usb3.clk.eq(self.local_clk),
            self.cd_usb3.rst.eq(~self.reset_n),
        ]

        # -----------------------------------------------------------------------------------------
        # Internal regs/wires
        # -----------------------------------------------------------------------------------------
        buf_in_commit_1 = Signal()
        buf_in_commit_2 = Signal()
        buf_out_arm_1   = Signal()
        buf_out_arm_2   = Signal()

        packet_setup    = Signal(64)

        # Derived fields (match Verilog bit layout).
        packet_setup_reqtype = Signal(8)
        packet_setup_dir     = Signal()
        packet_setup_type    = Signal(2)
        packet_setup_req     = Signal(8)
        packet_setup_wval    = Signal(16)
        packet_setup_widx    = Signal(16)
        packet_setup_wlen    = Signal(16)

        self.comb += [
            packet_setup_reqtype.eq(packet_setup[56:64]),
            packet_setup_dir.eq(packet_setup_reqtype[7]),
            packet_setup_type.eq(packet_setup_reqtype[5:7]),
            packet_setup_req.eq(packet_setup[48:56]),

            # Verilog:
            # wval = {packet_setup[39:32], packet_setup[47:40]}
            # widx = {packet_setup[23:16], packet_setup[31:24]}
            # wlen = {packet_setup[7:0],   packet_setup[15:8]}
            packet_setup_wval.eq(Cat(packet_setup[40:48], packet_setup[32:40])),
            packet_setup_widx.eq(Cat(packet_setup[24:32], packet_setup[16:24])),
            packet_setup_wlen.eq(Cat(packet_setup[8:16],  packet_setup[0:8])),
        ]

        desired_out_len = Signal(11)
        packet_out_len  = Signal(11)
        dev_config      = Signal(4)

        ptr_in  = Signal()
        ptr_out = Signal()

        len_in   = Signal(11)
        ready_in = Signal()
        self.comb += self.buf_in_ready.eq(ready_in)

        len_out     = Signal(11)
        hasdata_out = Signal()
        self.comb += [
            self.buf_out_len.eq(len_out),
            self.buf_out_hasdata.eq(hasdata_out),
        ]

        dc = Signal(7)

        # IN FSM states.
        state_in = Signal(6)
        ST_RST_0            = 0
        ST_RST_1            = 1
        ST_IDLE             = 10
        ST_IN_COMMIT        = 11
        ST_IN_PARSE_0       = 21
        ST_IN_PARSE_1       = 22
        ST_REQ_DESCR        = 30
        ST_RDLEN_0          = 31
        ST_RDLEN_1          = 32
        ST_RDLEN_2          = 33
        ST_RDLEN_3          = 34
        ST_REQ_GETCONFIG    = 35
        ST_REQ_SETCONFIG    = 36
        ST_REQ_SETINTERFACE = 37
        ST_REQ_SETADDR      = 38
        ST_REQ_VENDOR       = 39
        ST_REQ_SETSEL       = 40
        ST_REQ_SETFEAT      = 41
        ST_REQ_CLRFEAT      = 42

        # OUT FSM states (reuse reset/idle numbers like the Verilog).
        state_out = Signal(6)
        ST_OUT_ARM  = 11
        ST_OUT_SWAP = 20

        # Read-port of EP0 IN RAM.
        buf_in_rdaddr = Signal(4)
        buf_in_q      = Signal(32)

        # Descriptor address offset.
        descrip_addr_offset = Signal(8)

        # Acks per original combinational assigns.
        self.comb += [
            self.buf_in_commit_ack.eq(state_in == ST_IN_COMMIT),
            self.buf_out_arm_ack.eq(state_out == ST_OUT_ARM),
        ]

        # -----------------------------------------------------------------------------------------
        # External RAM/ROM instances (verbatim module names)
        # -----------------------------------------------------------------------------------------
        self.specials += Instance("usb3_ep0in_ram",
            i_clk      = self.local_clk,
            i_wr_dat_w  = self.buf_in_data,
            i_rd_adr    = buf_in_rdaddr,
            i_wr_adr    = self.buf_in_addr,
            i_wr_we     = self.buf_in_wren,
            o_rd_dat_r  = buf_in_q,
        )

        # adr = buf_out_addr + descrip_addr_offset
        rom_adr = Signal(17)
        self.comb += rom_adr.eq(self.buf_out_addr + descrip_addr_offset)

        self.specials += Instance("usb3_descrip_rom",
            i_clk   = self.local_clk,
            i_adr   = rom_adr,
            o_dat_r = self.buf_out_q,
        )

        # -----------------------------------------------------------------------------------------
        # Main sequential logic (always @(posedge local_clk))
        # -----------------------------------------------------------------------------------------
        self.sync.usb3 += [
            # Edge detectors for commit/arm strobes.
            buf_in_commit_2.eq(buf_in_commit_1),
            buf_in_commit_1.eq(self.buf_in_commit),
            buf_out_arm_2.eq(buf_out_arm_1),
            buf_out_arm_1.eq(self.buf_out_arm),

            # configured flag from dev_config.
            self.configured.eq(dev_config != 0),

            # Default pulses.
            self.reset_dp_seq.eq(0),

            # Increment dc (macro `INC(dc)`).
            dc.eq(dc + 1),

            # Clear vend_req_act after 4 cycles.
            If(dc == 3,
                self.vend_req_act.eq(0)
            ),

            # -------------------------------------------------------------------------------------
            # IN FSM
            # -------------------------------------------------------------------------------------
            Case(state_in, {
                ST_RST_0: [
                    len_out.eq(0),

                    desired_out_len.eq(0),
                    self.dev_addr.eq(0),
                    dev_config.eq(0),
                    self.err_setup_pkt.eq(0),

                    ready_in.eq(1),

                    state_in.eq(ST_RST_1),
                ],
                ST_RST_1: [
                    state_in.eq(ST_IDLE),
                ],
                ST_IDLE: [
                    If(buf_in_commit_1 & ~buf_in_commit_2,
                        len_in.eq(self.buf_in_commit_len),
                        ready_in.eq(0),
                        dc.eq(0),
                        state_in.eq(ST_IN_COMMIT),
                    )
                ],
                ST_IN_COMMIT: [
                    If(dc == 3,
                        dc.eq(0),
                        buf_in_rdaddr.eq(0),
                        state_in.eq(ST_IN_PARSE_0),
                    )
                ],
                ST_IN_PARSE_0: [
                    buf_in_rdaddr.eq(buf_in_rdaddr + 1),
                    # Verilog: packet_setup <= {packet_setup[31:0], buf_in_q[31:0]};
                    packet_setup.eq(Cat(buf_in_q, packet_setup[0:32])),
                    If(dc == 3,  # (2+2-1)
                        state_in.eq(ST_IN_PARSE_1)
                    )
                ],
                ST_IN_PARSE_1: [
                    packet_out_len.eq(packet_setup_wlen),

                    If(packet_setup_type == SETUP_TYPE_VENDOR,
                        state_in.eq(ST_REQ_VENDOR)
                    ).Else(
                        Case(packet_setup_req, {
                            REQ_GET_DESCR:      state_in.eq(ST_REQ_DESCR),
                            REQ_GET_CONFIG:     state_in.eq(ST_REQ_GETCONFIG),
                            REQ_SET_CONFIG:     state_in.eq(ST_REQ_SETCONFIG),
                            REQ_SET_INTERFACE:  state_in.eq(ST_REQ_SETINTERFACE),
                            REQ_SET_ADDR:       state_in.eq(ST_REQ_SETADDR),
                            REQ_SET_FEAT:       state_in.eq(ST_REQ_SETFEAT),
                            REQ_CLEAR_FEAT:     state_in.eq(ST_REQ_CLRFEAT),
                            REQ_SET_SEL:        state_in.eq(ST_REQ_SETSEL),
                            "default": [
                                ready_in.eq(1),
                                state_in.eq(ST_IDLE),
                            ]
                        })
                    )
                ],

                ST_REQ_DESCR: [
                    state_in.eq(ST_RDLEN_0),
                    Case(packet_setup_wval, {
                        0x0100: [  # device descriptor
                            descrip_addr_offset.eq(DESCR_USB3_DEVICE),
                        ],
                        0x0200: [  # config descriptor
                            descrip_addr_offset.eq(DESCR_USB3_CONFIG),
                            desired_out_len.eq(DESCR_USB3_CONFIG_LEN),
                            state_in.eq(ST_RDLEN_3),
                        ],
                        0x0300: [  # string: languages
                            descrip_addr_offset.eq(DESCR_USB3_STRING0),
                        ],
                        0x0301: [  # string: manufacturer
                            descrip_addr_offset.eq(DESCR_USB3_STRING1),
                        ],
                        0x0302: [  # string: product name
                            descrip_addr_offset.eq(DESCR_USB3_STRING2),
                        ],
                        0x0303: [  # string: serial number
                            descrip_addr_offset.eq(DESCR_USB3_STRING3),
                        ],
                        0x0F00: [  # BOS
                            descrip_addr_offset.eq(DESCR_USB3_BOS),
                            desired_out_len.eq(DESCR_USB3_BOS_LEN),
                            state_in.eq(ST_RDLEN_3),
                        ],
                        "default": [
                            packet_out_len.eq(0),
                        ],
                    })
                ],

                ST_RDLEN_0: [ state_in.eq(ST_RDLEN_1) ],
                ST_RDLEN_1: [ state_in.eq(ST_RDLEN_2) ],
                ST_RDLEN_2: [
                    # first byte at pointer is total descriptor length
                    desired_out_len.eq(self.buf_out_q[24:32]),
                    state_in.eq(ST_RDLEN_3),
                ],
                ST_RDLEN_3: [
                    If(packet_out_len < desired_out_len,
                        len_out.eq(packet_out_len)
                    ).Else(
                        len_out.eq(desired_out_len)
                    ),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_GETCONFIG: [
                    len_out.eq(1),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    If(dev_config != 0,
                        descrip_addr_offset.eq(DESCR_USB3_CONFSET)
                    ).Else(
                        descrip_addr_offset.eq(DESCR_USB3_CONFUNSET)
                    ),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_SETCONFIG: [
                    # Verilog stores [6:0] into a 4-bit reg => truncation also happens there.
                    dev_config.eq(packet_setup_wval[0:4]),
                    self.reset_dp_seq.eq(1),

                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_SETINTERFACE: [
                    self.reset_dp_seq.eq(1),
                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_SETADDR: [
                    self.dev_addr.eq(packet_setup_wval[0:7]),
                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_VENDOR: [
                    self.vend_req_request.eq(packet_setup_req),
                    self.vend_req_val.eq(packet_setup_wval),
                    self.vend_req_index.eq(packet_setup_widx),

                    self.vend_req_act.eq(1),
                    dc.eq(0),

                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_SETSEL: [
                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_SETFEAT: [
                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                ST_REQ_CLRFEAT: [
                    self.reset_dp_seq.eq(1),
                    len_out.eq(0),
                    ready_in.eq(1),
                    hasdata_out.eq(1),
                    state_in.eq(ST_IDLE),
                ],

                "default": [
                    state_in.eq(ST_RST_0)
                ],
            }),

            # -------------------------------------------------------------------------------------
            # OUT FSM
            # -------------------------------------------------------------------------------------
            Case(state_out, {
                ST_RST_0: [
                    hasdata_out.eq(0),
                    state_out.eq(ST_RST_1),
                ],
                ST_RST_1: [
                    state_out.eq(ST_IDLE),
                ],
                ST_IDLE: [
                    If(buf_out_arm_1 & ~buf_out_arm_2,
                        dc.eq(0),
                        state_out.eq(ST_OUT_ARM),
                    )
                ],
                ST_OUT_ARM: [
                    If(dc == 3,
                        state_out.eq(ST_OUT_SWAP)
                    )
                ],
                ST_OUT_SWAP: [
                    ready_in.eq(1),
                    hasdata_out.eq(0),
                    state_out.eq(ST_IDLE),
                ],
                "default": [
                    state_out.eq(ST_RST_0)
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Async reset (active low) mirrors Verilog "if(~reset_n) ..."
            # -------------------------------------------------------------------------------------
            If(~self.reset_n,
                state_in.eq(ST_RST_0),
                state_out.eq(ST_RST_0),
            ),
        ]
