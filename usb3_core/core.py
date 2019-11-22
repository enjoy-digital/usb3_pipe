# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import os

from migen import *
from migen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

# USB3 Core Control --------------------------------------------------------------------------------

class USB3CoreControl(Module, AutoCSR):
    def __init__(self):
        # Not functional but prevents synthesis optimizations
        self._buf_in_addr       = CSRStorage(9)
        self._buf_in_data       = CSRStorage(32)
        self._buf_in_wren       = CSR()
        self._buf_in_request    = CSRStatus()
        self._buf_in_ready      = CSRStatus()
        self._buf_in_commit     = CSR()
        self._buf_in_commit_len = CSRStorage(11)
        self._buf_in_commit_ack = CSRStatus()
        self._buf_out_addr      = CSRStorage(9)
        self._buf_out_q         = CSRStatus(32)
        self._buf_out_len       = CSRStatus(11)
        self._buf_out_hasdata   = CSRStatus()
        self._buf_out_arm       = CSR()
        self._buf_out_arm_ack   = CSRStatus()

        # # #

        self.buf_in_addr       = self._buf_in_addr.storage
        self.buf_in_data       = self._buf_in_data.storage
        self.buf_in_wren       = self._buf_in_wren.re & self._buf_in_wren.r
        self.buf_in_request    = self._buf_in_request.status
        self.buf_in_ready      = self._buf_in_ready.status
        self.buf_in_commit     = self._buf_in_commit.re & self._buf_in_commit.r
        self.buf_in_commit_len = self._buf_in_commit_len.storage
        self.buf_in_commit_ack = self._buf_in_commit_ack.status
        self.buf_out_addr      = self._buf_out_addr.storage
        self.buf_out_q         = self._buf_out_q.status
        self.buf_out_len       = self._buf_out_len.status
        self.buf_out_hasdata   = self._buf_out_hasdata.status
        self.buf_out_arm       = self._buf_out_arm.re & self._buf_out_arm.r
        self.buf_out_arm_ack   = self._buf_out_arm_ack.status

# USB3 Core ----------------------------------------------------------------------------------------

class USB3Core(Module, AutoCSR):
    def __init__(self, platform):
        self.reset  = Signal()
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        self.submodules.usb3_control = usb3_control = USB3CoreControl()

        u0_timer = WaitTimer(32)
        u0_timer = ResetInserter()(u0_timer)
        self.submodules += u0_timer
        self.comb += u0_timer.wait.eq(1)

        LT_POLLING_IDLE = 15
        LT_POLLING_U0   = 16
        ltssm_state     = Signal(5)
        self.comb += [
            If(~u0_timer.done,
                ltssm_state.eq(LT_POLLING_IDLE)
            ).Else(
                ltssm_state.eq(LT_POLLING_U0)
            )
        ]


        self.comb += sink.ready.eq(1)
        self.comb += source.valid.eq(1)
        out_stall = Signal()
        self.comb += out_stall.eq(~source.ready)
        self.specials += Instance("usb3_top_usb3_pipe",
            i_clk               = ClockSignal(),
            i_reset_n           = ~self.reset,

            i_ltssm_state       = ltssm_state,

            i_in_data           = sink.data,
            i_in_datak          = sink.ctrl,
            i_in_active         = sink.valid,

            o_out_data          = source.data,
            o_out_datak         = source.ctrl,
            #o_out_active        = ,
            i_out_stall         = out_stall,

            i_buf_in_addr       = usb3_control.buf_in_addr,
            i_buf_in_data       = usb3_control.buf_in_data,
            i_buf_in_wren       = usb3_control.buf_in_wren,
            o_buf_in_request    = usb3_control.buf_in_request,
            o_buf_in_ready      = usb3_control.buf_in_ready,
            i_buf_in_commit     = usb3_control.buf_in_commit,
            i_buf_in_commit_len = usb3_control.buf_in_commit_len,
            o_buf_in_commit_ack = usb3_control.buf_in_commit_ack,

            i_buf_out_addr      = usb3_control.buf_out_addr,
            o_buf_out_q         = usb3_control.buf_out_q,
            o_buf_out_len       = usb3_control.buf_out_len,
            o_buf_out_hasdata   = usb3_control.buf_out_hasdata,
            i_buf_out_arm       = usb3_control.buf_out_arm,
            o_buf_out_arm_ack   = usb3_control.buf_out_arm_ack,

            #o_vend_req_act     =,
            #o_vend_req_request =,
            #o_vend_req_val     =
        )

        daisho_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "daisho")
        platform.add_verilog_include_path(os.path.join(daisho_path))
        platform.add_verilog_include_path(os.path.join(daisho_path, "usb3"))
        platform.add_source_dir(os.path.join(daisho_path, "usb3"))
