# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect.csr import *

# USB3 Core ----------------------------------------------------------------------------------------

class USB3Core(Module, AutoCSR):
    def __init__(self, platform):
        self.reset  = Signal()
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        class USB3Control(Module, AutoCSR):
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

                self.phy_enable        = self._phy_enable.storage
                self.core_enable       = self._core_enable.storage
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


        self.submodules.usb3_control = USB3Control()

        self.specials += Instance("usb3_top",
            i_clk               = ClockSignal(),
            i_reset_n           = ~self.reset,

            i_in_data           = sink.data,
            i_in_datak          = sink.ctrl,
            i_in_active         = sink.valid,

            o_out_data          = source.data,
            o_out_datak         = source.datak,
            o_out_active        = source.valid,
            i_out_stall         = ~source.ready,

            i_buf_in_addr       = self.usb3_control.buf_in_addr,
            i_buf_in_data       = self.usb3_control.buf_in_data,
            i_buf_in_wren       = self.usb3_control.buf_in_wren,
            o_buf_in_request    = self.usb3_control.buf_in_request,
            o_buf_in_ready      = self.usb3_control.buf_in_ready,
            i_buf_in_commit     = self.usb3_control.buf_in_commit,
            i_buf_in_commit_len = self.usb3_control.buf_in_commit_len,
            o_buf_in_commit_ack = self.usb3_control.buf_in_commit_ack,

            i_buf_out_addr      = self.usb3_control.buf_out_addr,
            o_buf_out_q         = self.usb3_control.buf_out_q,
            o_buf_out_len       = self.usb3_control.buf_out_len,
            o_buf_out_hasdata   = self.usb3_control.buf_out_hasdata,
            i_buf_out_arm       = self.usb3_control.buf_out_arm,
            o_buf_out_arm_ack   = self.usb3_control.buf_out_arm_ack,

            #o_vend_req_act     =,
            #o_vend_req_request =,
            #o_vend_req_val     =
        )
        daisho_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "daisho")
        platform.add_verilog_include_path(os.path.join(daisho_path, "core"))
        platform.add_verilog_include_path(os.path.join(daisho_path, "core", "usb3"))
        platform.add_source_dir(os.path.join(daisho_path, "core", "usb3"))
