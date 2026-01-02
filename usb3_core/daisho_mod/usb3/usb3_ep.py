#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2014 Marshall H.
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen import *

# -------------------------------------------------------------------------------------------------
# usb3_ep (LiteX/Migen translation of Daisho usb3_ep)
# -------------------------------------------------------------------------------------------------
# Notes:
# - Logic is synchronous to local_clk, with reset synchronized (reset_1/reset_2) exactly like Verilog.
# - Dual-port RAM is still an external Verilog instance (usb3_ep_ram) with independent rd_clk/wr_clk.
# - Double-buffering is preserved: 2x 256x32 (1024 bytes) windows selected by ptr_in/ptr_out.
# - buf_in_ready/buf_out_len/buf_out_hasdata muxed by ptr_in/ptr_out exactly like Verilog assigns.
# - buf_in_commit_ack/buf_out_arm_ack are asserted during *_COMMIT/*_SWAP and *_ARM/*_SWAP.
# -------------------------------------------------------------------------------------------------


class USB3EP(LiteXModule):
    def __init__(self):
        # -----------------------------------------------------------------------------------------
        # Ports (match Verilog)
        # -----------------------------------------------------------------------------------------
        self.slow_clk  = Signal()
        self.local_clk = Signal()
        self.rd_clk    = Signal()
        self.wr_clk    = Signal()
        self.reset_n   = Signal()

        self.buf_in_addr       = Signal(9)
        self.buf_in_data       = Signal(32)
        self.buf_in_wren       = Signal()
        self.buf_in_request    = Signal()   # Present in Verilog, not driven there either.
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

        self.mode              = Signal(2)

        # -----------------------------------------------------------------------------------------
        # Clock domain (cd_usb3 := local_clk)
        # -----------------------------------------------------------------------------------------
        self.clock_domains.cd_usb3 = ClockDomain()
        self.comb += [
            self.cd_usb3.clk.eq(self.local_clk),
            # Use the synchronized reset like the RTL does (reset_2), but we still need a CD reset.
            # We'll drive cd reset from ~reset_n; the internal logic uses reset_2 for behavior.
            self.cd_usb3.rst.eq(~self.reset_n),
        ]

        # -----------------------------------------------------------------------------------------
        # Internal regs/wires
        # -----------------------------------------------------------------------------------------
        reset_1 = Signal()
        reset_2 = Signal()

        buf_in_commit_1 = Signal()
        buf_in_commit_2 = Signal()
        buf_in_commit_3 = Signal()

        buf_out_arm_1 = Signal()
        buf_out_arm_2 = Signal()
        buf_out_arm_3 = Signal()

        # Double-buffer pointers.
        ptr_in  = Signal()
        ptr_out = Signal()

        len_in = Signal(11)

        ready_in_a = Signal()
        ready_in_b = Signal()

        len_out_a = Signal(11)
        len_out_b = Signal(11)

        hasdata_out_a = Signal()
        hasdata_out_b = Signal()

        # Muxed outputs (match assign statements).
        self.comb += [
            self.buf_in_ready.eq(Mux(ptr_in,  ready_in_b,  ready_in_a)),
            self.buf_out_len.eq(Mux(ptr_out, len_out_b,   len_out_a)),
            self.buf_out_hasdata.eq(Mux(ptr_out, hasdata_out_b, hasdata_out_a)),
        ]

        # FSM states.
        dc = Signal(4)

        state_in  = Signal(6)
        state_out = Signal(6)

        ST_RST_0     = 0
        ST_RST_1     = 1
        ST_IDLE      = 10
        ST_IN_COMMIT = 11
        ST_IN_SWAP   = 12
        ST_OUT_ARM   = 11
        ST_OUT_SWAP  = 12

        # Acks (match RTL).
        self.comb += [
            self.buf_in_commit_ack.eq((state_in == ST_IN_COMMIT) | (state_in == ST_IN_SWAP)),
            self.buf_out_arm_ack.eq((state_out == ST_OUT_ARM) | (state_out == ST_OUT_SWAP)),
        ]

        # buf_in_request is not implemented in this RTL; keep it tied low for now.
        self.comb += self.buf_in_request.eq(0)

        # -----------------------------------------------------------------------------------------
        # Endpoint BRAM instance (verbatim module name)
        # -----------------------------------------------------------------------------------------
        # Segment the space into two 1024 byte (256 word) buffers.
        rd_addr = Signal(10)
        wr_addr = Signal(10)
        self.comb += [
            rd_addr.eq(self.buf_out_addr + Mux(ptr_out, 10*256, 0)),
            wr_addr.eq(self.buf_in_addr  + Mux(ptr_in,  10*256, 0)),
        ]

        self.specials += Instance("usb3_ep_ram",
            i_rd_clk   = self.rd_clk,
            i_rd_adr   = rd_addr,
            o_rd_dat_r = self.buf_out_q,

            i_wr_clk   = self.wr_clk,
            i_wr_adr   = wr_addr,
            i_wr_dat_w = self.buf_in_data,
            i_wr_we    = self.buf_in_wren,
        )

        # -----------------------------------------------------------------------------------------
        # Sequential logic (always @(posedge local_clk))
        # -----------------------------------------------------------------------------------------
        self.sync.usb3 += [
            # Synchronizers (exact shift-register behavior).
            reset_1.eq(self.reset_n),
            reset_2.eq(reset_1),

            buf_in_commit_3.eq(buf_in_commit_2),
            buf_in_commit_2.eq(buf_in_commit_1),
            buf_in_commit_1.eq(self.buf_in_commit),

            buf_out_arm_3.eq(buf_out_arm_2),
            buf_out_arm_2.eq(buf_out_arm_1),
            buf_out_arm_1.eq(self.buf_out_arm),

            dc.eq(dc + 1),

            # -------------------------------------------------------------------------------------
            # Input FSM
            # -------------------------------------------------------------------------------------
            Case(state_in, {
                ST_RST_0: [
                    ptr_in.eq(0),
                    ready_in_a.eq(1),
                    ready_in_b.eq(1),
                    state_in.eq(ST_RST_1),
                ],
                ST_RST_1: [
                    state_in.eq(ST_IDLE),
                ],
                ST_IDLE: [
                    If(buf_in_commit_2 & ~buf_in_commit_3,
                        len_in.eq(self.buf_in_commit_len),
                        dc.eq(0),
                        state_in.eq(ST_IN_COMMIT),
                    )
                ],
                ST_IN_COMMIT: [
                    If(dc == 3,
                        state_in.eq(ST_IN_SWAP)
                    )
                ],
                ST_IN_SWAP: [
                    # swap the current buffer
                    ptr_in.eq(~ptr_in),

                    # current buffer is now not ready anymore (based on *old* ptr_in, as in RTL case)
                    If(ptr_in == 0,
                        ready_in_a.eq(0)
                    ).Else(
                        ready_in_b.eq(0)
                    ),

                    # tell output FSM this has data (old ptr_in)
                    If(ptr_in == 0,
                        hasdata_out_a.eq(1)
                    ).Else(
                        hasdata_out_b.eq(1)
                    ),

                    # copy length (old ptr_in)
                    If(ptr_in == 0,
                        len_out_a.eq(len_in)
                    ).Else(
                        len_out_b.eq(len_in)
                    ),

                    state_in.eq(ST_IDLE),
                ],
                "default": [
                    state_in.eq(ST_RST_0)
                ],
            }),

            # -------------------------------------------------------------------------------------
            # Output FSM
            # -------------------------------------------------------------------------------------
            Case(state_out, {
                ST_RST_0: [
                    ptr_out.eq(0),
                    hasdata_out_a.eq(0),
                    hasdata_out_b.eq(0),
                    state_out.eq(ST_RST_1),
                ],
                ST_RST_1: [
                    state_out.eq(ST_IDLE),
                ],
                ST_IDLE: [
                    If(buf_out_arm_2 & ~buf_out_arm_3,
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
                    # swap current buffer
                    ptr_out.eq(~ptr_out),

                    # current buffer is now ready for data (based on *old* ptr_out)
                    If(ptr_out == 0,
                        ready_in_a.eq(1)
                    ).Else(
                        ready_in_b.eq(1)
                    ),

                    # update hasdata status (based on *old* ptr_out)
                    If(ptr_out == 0,
                        hasdata_out_a.eq(0)
                    ).Else(
                        hasdata_out_b.eq(0)
                    ),

                    state_out.eq(ST_IDLE),
                ],
                "default": [
                    state_out.eq(ST_RST_0)
                ],
            }),

            # Reset (as in RTL: if(~reset_2) ...).
            If(~reset_2,
                state_in.eq(ST_RST_0),
                state_out.eq(ST_RST_0),
            ),
        ]
