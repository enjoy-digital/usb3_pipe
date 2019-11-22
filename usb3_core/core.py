# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect.csr import *

# USB3 Core ----------------------------------------------------------------------------------------

class USB3Core(Module, AutoCSR):
    def __init__(self, platform):
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

        phy_rx_status     = Signal(6)
        phy_phy_status    = Signal(2)

        dbg_pipe_state    = Signal(6)
        dbg_ltssm_state   = Signal(5)

        usb_pipe_status_phy_status = Signal()
        self.specials += Tristate(usb_pipe_status.phy_status, 0, ~usb3_reset_n, usb_pipe_status_phy_status)

        self.comb += usb3_reset_n.eq(self.usb3_control.phy_enable)
        self.specials += Instance("usb3_top",
            i_ext_clk = ClockSignal(),
            i_reset_n = self.usb3_control.core_enable,

            i_phy_pipe_half_clk       = 0, # FIXME
            i_phy_pipe_half_clk_phase = 0, # FIXME,
            i_phy_pipe_quarter_clk    = 0, # FIXME,

            i_phy_pipe_rx_data  = sink.data,
            i_phy_pipe_rx_datak = sink.ctrl,
            i_phy_pipe_rx_valid = sink.valid,
            o_phy_pipe_tx_data  = source.data, # FIXME use source.ready
            o_phy_pipe_tx_datak = source.ctrl, # FIXME use source.ready

            # FIXME: remove since USB3PIPE is handling this internally
            #o_phy_reset_n       = ,
            #o_phy_out_enable    = ,
            #o_phy_phy_reset_n   = ,
            #o_phy_tx_detrx_lpbk = ,
            #o_phy_tx_elecidle   = ,
            #io_phy_rx_elecidle  = ,
            #i_phy_rx_status     = ,
            #o_phy_power_down    = ,
            #i_phy_phy_status_i  = ,
            #o_phy_phy_status_o  = ,
            #i_phy_pwrpresent    = ,

            #o_phy_tx_oneszeros   = ,
            #o_phy_tx_deemph      = ,
            #o_phy_tx_margin      = ,
            #o_phy_tx_swing       = ,
            #o_phy_rx_polarity    = ,
            #o_phy_rx_termination = ,
            #o_phy_rate           = ,
            #o_phy_elas_buf_mode  = ,

            i_buf_in_addr       = self.usb3_control.buf_in_addr,
            i_buf_in_data       = self.usb3_control.buf_in_data,
            i_buf_in_wren       = self.usb3_control.buf_in_wren,
            o_buf_in_request    = self.usb3_control.buf_in_request,
            o_buf_in_ready      = self.usb3_control.buf_in_ready,
            i_buf_in_commit     = self.usb3_control.buf_in_commit,
            i_buf_in_commit_len = self.usb3_control.buf_in_commit_len,
            o_buf_in_commit_ack = self.usb3_control.buf_in_commit_ack,

            i_buf_out_addr    = self.usb3_control.buf_out_addr,
            o_buf_out_q       = self.usb3_control.buf_out_q,
            o_buf_out_len     = self.usb3_control.buf_out_len,
            o_buf_out_hasdata = self.usb3_control.buf_out_hasdata,
            i_buf_out_arm     = self.usb3_control.buf_out_arm,
            o_buf_out_arm_ack = self.usb3_control.buf_out_arm_ack,

            #o_vend_req_act     =,
            #o_vend_req_request =,
            #o_vend_req_val     =,

            #o_dbg_pipe_state  = ,
            #o_dbg_ltssm_state = ,
        )
        daisho_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "daisho")
        platform.add_verilog_include_path(os.path.join(daisho_path, "core"))
        platform.add_verilog_include_path(os.path.join(daisho_path, "core", "usb3"))
        platform.add_source_dir(os.path.join(daisho_path, "core", "usb3"))
