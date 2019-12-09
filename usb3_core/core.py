# This file is Copyright (c) 2017-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import os

from migen import *
from migen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

# USB3 Core Endpoint --------------------------------------------------------------------------------

class USB3CoreEndpoint(Module, AutoCSR):
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
    def __init__(self, platform, with_endpoint=False):
        self.reset  = Signal()
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        # Artificial after reset delay from LT_POLLING_IDLE to LT_POLLING_U0
        u0_timer = WaitTimer(32)
        u0_timer = ResetInserter()(u0_timer)
        self.submodules += u0_timer
        self.comb += u0_timer.wait.eq(1)
        self.comb += u0_timer.reset.eq(self.reset)

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

        # RX (Sink) --------------------------------------------------------------------------------
        in_data   = Signal(32)
        in_datak  = Signal(4)
        in_active = Signal()
        self.comb += [
            sink.ready.eq(1), # Always ready
            in_data.eq(sink.data),
            in_datak.eq(sink.ctrl),
            in_active.eq(sink.valid),
        ]

        # TX (Source) ------------------------------------------------------------------------------
        # Daisho core does not support back-pressure (ready signal of LiteX's streams). To accomodate
        # that, we use a FIFO that absorbs the data bursts from the core and re-transmits datas to
        # the USB3 Pipe at the maximum allowed rate with back-pressure. This is a hack for our tests
        # and should be fixed correctly in the core.

        # FIFO
        out_fifo = stream.SyncFIFO([("data", 32), ("ctrl", 4)], 128)
        self.submodules += out_fifo

        # Map core signals to stream, re-generate first/last delimiters from active signal.
        out_data     = Signal(32)
        out_datak    = Signal(4)
        out_stall    = Signal()
        out_active   = Signal()
        out_active_d = Signal()
        self.comb += out_fifo.sink.valid.eq(out_active_d)
        self.sync += [
            out_fifo.sink.data.eq(out_data),
            out_fifo.sink.ctrl.eq(out_datak),
            out_active_d.eq(out_active),
            out_fifo.sink.first.eq(out_active & ~ out_active_d),
        ]
        self.comb += out_fifo.sink.last.eq(~out_active & out_active_d)

        # Connect FIFO to source.
        self.comb += [
            If(out_fifo.source.valid,
                out_fifo.source.connect(source)
            ).Else(
                source.valid.eq(1),
                source.first.eq(0),
                source.last.eq(0),
                source.data.eq(0),
                source.ctrl.eq(0),
            )
        ]

        # Daisho USB3 core -------------------------------------------------------------------------
        usb3_top_params = dict(
            i_clk               = ClockSignal(),
            i_reset_n           = ~self.reset,

            i_ltssm_state       = ltssm_state,

            i_in_data           = sink.data,
            i_in_datak          = sink.ctrl,
            i_in_active         = sink.valid,

            o_out_data          = out_data,
            o_out_datak         = out_datak,
            o_out_active        = out_active,
            i_out_stall         = 0, # FIXME
        )

        # Daisho USB3 core endpoinst ---------------------------------------------------------------
        if with_endpoint:
            self.submodules.usb3_control = usb3_control = USB3CoreControl()
            usb3_top_params.update(
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
            )

        # Daisho USB3 instance ---------------------------------------------------------------------
        self.specials += Instance("usb3_top_usb3_pipe", **usb3_top_params)

        daisho_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "daisho")
        platform.add_verilog_include_path(os.path.join(daisho_path))
        platform.add_verilog_include_path(os.path.join(daisho_path, "usb3"))
        platform.add_source_dir(os.path.join(daisho_path, "usb3"))
