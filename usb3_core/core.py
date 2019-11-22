# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect.csr import *

# USB3 Core ----------------------------------------------------------------------------------------

class USB3Core(Module):
    def __init__(self, platform):

        # usb ios
        usb_reset_n = platform.request("usb_reset_n", usb_connector)
        if with_usb3:
            usb_pipe_ctrl   = platform.request("usb_pipe_ctrl", usb_connector)
            usb_pipe_status = platform.request("usb_pipe_status", usb_connector)
            usb_pipe_data   = platform.request("usb_pipe_data", usb_connector)

        usb3_reset_n = Signal(reset=1)
        self.comb += usb_reset_n.eq(usb3_reset_n)

        # usb3 core
        if with_usb3:
            class USB3Control(Module, AutoCSR):
                def __init__(self):
                    self._phy_enable        = CSRStorage()
                    self._core_enable       = CSRStorage()

                    # probably not working but prevent synthesis optimizations
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

                    self.phy_enable  = self._phy_enable.storage
                    self.core_enable = self._core_enable.storage

                    self.buf_in_addr       = self._buf_in_addr.storage
                    self.buf_in_data       = self._buf_in_data.storage
                    self.buf_in_wren       = self._buf_in_wren.re & self._buf_in_wren.r
                    self.buf_in_request    = self._buf_in_request.status
                    self.buf_in_ready      = self._buf_in_ready.status
                    self.buf_in_commit     = self._buf_in_commit.re & self._buf_in_commit.r
                    self.buf_in_commit_len = self._buf_in_commit_len.storage
                    self.buf_in_commit_ack = self._buf_in_commit_ack.status

                    self.buf_out_addr    = self._buf_out_addr.storage
                    self.buf_out_q       = self._buf_out_q.status
                    self.buf_out_len     = self._buf_out_len.status
                    self.buf_out_hasdata = self._buf_out_hasdata.status
                    self.buf_out_arm     = self._buf_out_arm.re & self._buf_out_arm.r
                    self.buf_out_arm_ack = self._buf_out_arm_ack.status


            self.submodules.usb3_control = USB3Control()
            self.add_csr("usb3_control")

            phy_pipe_pll_locked = Signal()
            phy_pipe_pll_fb     = Signal()

            phy_pipe_half_clk_pll       = Signal()
            phy_pipe_half_clk_phase_pll = Signal()
            phy_pipe_quarter_clk_pll    = Signal()
            phy_pipe_tx_clk_phase_pll   = Signal()

            phy_pipe_half_clk       = Signal()
            phy_pipe_half_clk_phase = Signal()
            phy_pipe_quarter_clk    = Signal()
            phy_pipe_tx_clk_phase   = Signal()

            self.specials += [
                Instance("PLLE2_BASE",
                    p_STARTUP_WAIT="FALSE", o_LOCKED=phy_pipe_pll_locked,

                    # VCO @ 1GHz
                    p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=4.0,
                    p_CLKFBOUT_MULT=4, p_DIVCLK_DIVIDE=1,
                    i_CLKIN1=usb_pipe_data.rx_clk, i_CLKFBIN=phy_pipe_pll_fb,
                    o_CLKFBOUT=phy_pipe_pll_fb,

                    # 125MHz: 1/2 PCLK
                    p_CLKOUT0_DIVIDE=8, p_CLKOUT0_PHASE=0.0,
                    o_CLKOUT0=phy_pipe_half_clk_pll,

                    # 125MHz: 1/2 PCLK, phase shift 90
                    p_CLKOUT1_DIVIDE=8, p_CLKOUT1_PHASE=90.0,
                    o_CLKOUT1=phy_pipe_half_clk_phase_pll,

                    # 62.5MHz: 1/4 PCLK
                    p_CLKOUT2_DIVIDE=16, p_CLKOUT2_PHASE=0.0,
                    o_CLKOUT2=phy_pipe_quarter_clk_pll,

                    # 250Mhz: TX CLK, phase shift 90
                    p_CLKOUT3_DIVIDE=4, p_CLKOUT3_PHASE=90.0,
                    o_CLKOUT3=phy_pipe_tx_clk_phase_pll
                ),
                Instance("BUFG", i_I=phy_pipe_half_clk_pll, o_O=phy_pipe_half_clk),
                Instance("BUFG", i_I=phy_pipe_half_clk_phase_pll, o_O=phy_pipe_half_clk_phase),
                Instance("BUFG", i_I=phy_pipe_quarter_clk_pll, o_O=phy_pipe_quarter_clk),
                Instance("BUFG", i_I=phy_pipe_tx_clk_phase_pll, o_O=phy_pipe_tx_clk_phase),
            ]

            self.clock_domains.cd_phy_pipe_half       = ClockDomain()
            self.clock_domains.cd_phy_pipe_half_phase = ClockDomain()
            self.clock_domains.cd_phy_pipe_quarter    = ClockDomain()
            self.clock_domains.cd_phy_pipe_tx_phase   = ClockDomain()
            self.comb += [
                self.cd_phy_pipe_half.clk.eq(phy_pipe_half_clk),
                self.cd_phy_pipe_half_phase.clk.eq(phy_pipe_half_clk_phase),
                self.cd_phy_pipe_quarter.clk.eq(phy_pipe_quarter_clk),
                self.cd_phy_pipe_tx_phase.clk.eq(phy_pipe_tx_clk_phase)
            ]
            self.specials += [
                AsyncResetSynchronizer(self.cd_phy_pipe_half,       ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_half_phase, ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_quarter,    ~phy_pipe_pll_locked),
                AsyncResetSynchronizer(self.cd_phy_pipe_tx_phase,   ~phy_pipe_pll_locked)
            ]
            self.cd_phy_pipe_half.clk.attr.add("keep")
            self.cd_phy_pipe_half_phase.clk.attr.add("keep")
            self.cd_phy_pipe_quarter.clk.attr.add("keep")
            self.cd_phy_pipe_tx_phase.clk.attr.add("keep")
            self.platform.add_period_constraint(self.cd_phy_pipe_half.clk, 8.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_half_phase.clk, 8.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_quarter.clk, 16.0)
            self.platform.add_period_constraint(self.cd_phy_pipe_tx_phase.clk, 4.0)
            self.platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.cd_phy_pipe_half.clk,
                self.cd_phy_pipe_half_phase.clk,
                self.cd_phy_pipe_quarter.clk,
                self.cd_phy_pipe_tx_phase.clk)


            phy_pipe_rx_data  = Signal(32)
            phy_pipe_rx_datak = Signal(4)
            phy_pipe_rx_valid = Signal(2)

            phy_pipe_tx_data  = Signal(32)
            phy_pipe_tx_datak = Signal(4)

            phy_rx_status  = Signal(6)
            phy_phy_status = Signal(2)

            dbg_pipe_state  = Signal(6)
            dbg_ltssm_state = Signal(5)

            usb_pipe_status_phy_status = Signal()
            self.specials += Tristate(usb_pipe_status.phy_status, 0, ~usb3_reset_n, usb_pipe_status_phy_status)

            self.comb += usb3_reset_n.eq(self.usb3_control.phy_enable)
            self.specials += Instance("usb3_top",
                i_ext_clk = ClockSignal(),
                i_reset_n = self.usb3_control.core_enable,

                i_phy_pipe_half_clk       = ClockSignal("phy_pipe_half"),
                i_phy_pipe_half_clk_phase = ClockSignal("phy_pipe_half_phase"),
                i_phy_pipe_quarter_clk    = ClockSignal("phy_pipe_quarter"),

                i_phy_pipe_rx_data  = phy_pipe_rx_data,
                i_phy_pipe_rx_datak = phy_pipe_rx_datak,
                i_phy_pipe_rx_valid = phy_pipe_rx_valid,
                o_phy_pipe_tx_data  = phy_pipe_tx_data,
                o_phy_pipe_tx_datak = phy_pipe_tx_datak,

                #o_phy_reset_n      = ,
                #o_phy_out_enable   = ,
                o_phy_phy_reset_n   = usb_pipe_ctrl.phy_reset_n,
                o_phy_tx_detrx_lpbk = usb_pipe_ctrl.tx_detrx_lpbk,
                o_phy_tx_elecidle   = usb_pipe_ctrl.tx_elecidle,
                io_phy_rx_elecidle  = usb_pipe_status.rx_elecidle,
                i_phy_rx_status     = phy_rx_status,
                o_phy_power_down    = usb_pipe_ctrl.power_down,
                i_phy_phy_status_i  = phy_phy_status,
                #o_phy_phy_status_o = ,
                i_phy_pwrpresent    = usb_pipe_status.pwr_present,

                o_phy_tx_oneszeros   = usb_pipe_ctrl.tx_oneszeros,
                o_phy_tx_deemph      = usb_pipe_ctrl.tx_deemph,
                o_phy_tx_margin      = usb_pipe_ctrl.tx_margin,
                o_phy_tx_swing       = usb_pipe_ctrl.tx_swing,
                o_phy_rx_polarity    = usb_pipe_ctrl.rx_polarity,
                o_phy_rx_termination = usb_pipe_ctrl.rx_termination,
                o_phy_rate           = usb_pipe_ctrl.rate,
                o_phy_elas_buf_mode  = usb_pipe_ctrl.elas_buf_mode,

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

                o_dbg_pipe_state  = dbg_pipe_state,
                o_dbg_ltssm_state = dbg_ltssm_state
            )
            platform.add_verilog_include_path(os.path.join("core"))
            platform.add_verilog_include_path(os.path.join("core", "usb3"))
            platform.add_source_dir(os.path.join("core", "usb3"))

            # ddr inputs
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                i_D=usb_pipe_data.rx_valid, o_Q1=phy_pipe_rx_valid[0], o_Q2=phy_pipe_rx_valid[1],
            )
            for i in range(16):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_data.rx_data[i], o_Q1=phy_pipe_rx_data[i], o_Q2=phy_pipe_rx_data[16+i],
                )
            for i in range(2):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_data.rx_datak[i], o_Q1=phy_pipe_rx_datak[i], o_Q2=phy_pipe_rx_datak[2+i],
                )
            for i in range(3):
                self.specials += Instance("IDDR",
                    p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                    i_D=usb_pipe_status.rx_status[i], o_Q1=phy_rx_status[i], o_Q2=phy_rx_status[3+i],
                )
            self.specials += Instance("IDDR",
                p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                i_C=ClockSignal("phy_pipe_half"), i_CE=1, i_S=0, i_R=0,
                i_D=usb_pipe_status_phy_status, o_Q1=phy_phy_status[0], o_Q2=phy_phy_status[1],
            )

            # ddr outputs
            self.specials += Instance("ODDR",
                p_DDR_CLK_EDGE="SAME_EDGE",
                i_C=ClockSignal("phy_pipe_tx_phase"), i_CE=1, i_S=0, i_R=0,
                i_D1=1, i_D2=0, o_Q=usb_pipe_data.tx_clk,
            )
            for i in range(16):
                self.specials += Instance("ODDR",
                    p_DDR_CLK_EDGE="SAME_EDGE",
                    i_C=ClockSignal("phy_pipe_half_phase"), i_CE=1, i_S=0, i_R=0,
                    i_D1=phy_pipe_tx_data[i], i_D2=phy_pipe_tx_data[16+i], o_Q=usb_pipe_data.tx_data[i],
                )
            for i in range(2):
                self.specials += Instance("ODDR",
                    p_DDR_CLK_EDGE="SAME_EDGE",
                    i_C=ClockSignal("phy_pipe_half_phase"), i_CE=1, i_S=0, i_R=0,
                    i_D1=phy_pipe_tx_datak[i], i_D2=phy_pipe_tx_datak[2+i], o_Q=usb_pipe_data.tx_datak[i],
                )
