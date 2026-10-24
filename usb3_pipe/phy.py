#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY: USB3PIPE + USB3LTSSM with the Link interface used by a USB3 Link Layer (doc/usb3-pipe.md).
"""

from migen import *

from litex.gen import *

from usb3_pipe.core  import USB3PIPE
from usb3_pipe.ltssm import USB3LTSSM

# LTSSM <-> PIPE -----------------------------------------------------------------------------------

def connect_ltssm_pipe(ltssm, pipe):
    """LTSSM <-> PIPE wiring (LFPS/TS face)."""
    return [
        # LTSSM -> PIPE
        pipe.lfps_tx_polling.eq(ltssm.lfps_tx_polling),
        pipe.lfps_tx_idle.eq(ltssm.lfps_tx_idle),
        pipe.lfps_tx_ping.eq(ltssm.lfps_tx_ping),
        pipe.lfps_tx_exit_u1.eq(ltssm.lfps_tx_exit_u1),
        pipe.lfps_tx_exit_u2.eq(ltssm.lfps_tx_exit_u2),
        pipe.lfps_tx_wakeup_u3.eq(ltssm.lfps_tx_wakeup_u3),
        pipe.lfps_tx_reset.eq(ltssm.lfps_tx_reset),
        pipe.ts_rx_enable.eq(ltssm.ts_rx_enable),
        pipe.ts_tx_enable.eq(ltssm.ts_tx_enable),
        pipe.ts_tx_tseq.eq(ltssm.ts_tx_tseq),
        pipe.ts_tx_ts1.eq(ltssm.ts_tx_ts1),
        pipe.ts_tx_ts2.eq(ltssm.ts_tx_ts2),
        pipe.ts_tx_ts2_reset.eq(ltssm.tx_ts2_reset),
        pipe.ts_tx_ts2_loopback.eq(ltssm.tx_ts2_loopback),
        pipe.ts2_decode.eq(ltssm.ts2_decode),

        pipe.serdes_rx_align.eq(ltssm.serdes_rx_align),
        pipe.serdes_rx_polarity.eq(ltssm.serdes_rx_polarity),

        pipe.rx_ready.eq(ltssm.rx_ready),
        pipe.tx_ready.eq(ltssm.tx_ready),
        pipe.idle_handshake.eq(ltssm.idle_handshake),

        # PIPE -> LTSSM
        ltssm.lfps_rx_polling.eq(pipe.lfps_rx_polling),
        ltssm.lfps_tx_count.eq(pipe.lfps_tx_count),
        ltssm.lfps_rx_ping.eq(pipe.lfps_rx_ping),
        ltssm.lfps_rx_exit_u1.eq(pipe.lfps_rx_exit_u1),
        ltssm.lfps_rx_exit_u2.eq(pipe.lfps_rx_exit_u2),
        ltssm.lfps_rx_wakeup_u3.eq(pipe.lfps_rx_wakeup_u3),
        ltssm.lfps_rx_reset.eq(pipe.lfps_rx_reset),

        ltssm.ts_rx_ts1.eq(pipe.ts_rx_ts1),
        ltssm.ts_rx_ts1_inv.eq(pipe.ts_rx_ts1_inv),
        ltssm.ts_rx_ts2.eq(pipe.ts_rx_ts2),
        ltssm.rx_ts2_reset.eq(pipe.ts_rx_ts2_reset),
        ltssm.rx_ts2_loopback.eq(pipe.ts_rx_ts2_loopback),
        ltssm.ts_tx_done.eq(pipe.ts_tx_done),
        ltssm.rx_idle8.eq(pipe.idle_done),
    ]

# USB3 PHY -----------------------------------------------------------------------------------------

# LTSSM status outputs exposed by USB3PHY.
USB3PHY_STATUS = [
    "u0", "recovery", "rx_ready", "tx_ready", "u1", "u2", "u3", "compliance", "loopback",
    "hot_reset", "ess_inactive", "polling_idle",
]

# LTSSM Link-Layer control interface (PM handshake, directed requests): (name, direction).
USB3PHY_LINK_CTRL = [
    ("lgo_u1",            "i"), ("lgo_u2",         "i"), ("lgo_u3",        "i"),
    ("lau",               "i"), ("lxu",            "i"),
    ("req_u1",            "i"), ("req_u2",         "i"), ("req_u3",        "i"),
    ("tx_lgo_u1",         "o"), ("tx_lgo_u2",      "o"), ("tx_lgo_u3",     "o"),
    ("tx_lau",            "o"), ("tx_lxu",         "o"), ("tx_lpma",       "o"),
    ("link_activity",     "i"),
    ("directed_recovery", "i"), ("hot_reset_req",  "i"), ("warm_reset_req", "i"),
    ("exit_u1_req",       "i"), ("exit_u2_req",    "i"), ("wakeup_u3_req",  "i"),
]

@ResetInserter()
class USB3PHY(LiteXModule):
    """USB3.0 PHY: USB3PIPE + USB3LTSSM.

    Combines the PIPE (LFPS/TS/scrambling/datapath) with the LTSSM to provide a link-trained
    32-bit data/4-bit ctrl (K-char) symbol stream in the sys clock domain, ready for a USB3.0 Link
    Layer. The SerDes is not a submodule and must be added by the caller.

    Link interface (what a USB3.0 Link Layer needs from the PHY, the Link Layer does not have to know
    the PHY internals):
    - `sink`/`source`: data/ctrl symbol streams (Logical Idle sent when `sink` has no data).
    - Status (see USB3PHY_STATUS): mirrored from the LTSSM.
    - Link-Layer control (see USB3PHY_LINK_CTRL): PM handshake (LGO_Ux/LAU/LXU/LPMA decoded or to
      send by the Link Layer), directed Recovery/Hot Reset/Warm Reset and Ux exit requests. The
      signals are the LTSSM ones (same Signal objects); undriven inputs keep their reset values.
    """
    def __init__(self, serdes, sys_clk_freq, role="device", with_endianness_swap=True, timer_scale=1):
        assert role in ["device", "host"]

        # PIPE / LTSSM -----------------------------------------------------------------------------
        self.pipe  = pipe  = USB3PIPE(serdes,
            sys_clk_freq         = sys_clk_freq,
            with_endianness_swap = with_endianness_swap,
            timer_scale          = timer_scale)
        self.ltssm = ltssm = USB3LTSSM(sys_clk_freq=sys_clk_freq, timer_scale=timer_scale)
        self.comb += connect_ltssm_pipe(ltssm, pipe)
        self.comb += ltssm.ds_port.eq(role == "host")

        # SerDes RX Re-initialization --------------------------------------------------------------
        # The transceiver RX initialization (PMA/CDR/offset calibration) does not always end in a
        # good state: on hardware (Artix7 GTP, xHCI host with SSC) ~1/12 RX resets ended with a
        # receiver unable to train or with a high error rate, recovered by another RX reset. Re-run
        # it on each training failure (and Warm Reset), when the LTSSM exits to Rx.Detect, or when
        # requested (serdes_rx_reset, ex: debug). SerDes without RX reset control: no-op.
        self.serdes_rx_reset = Signal() # i
        if hasattr(serdes, "rx_reset"):
            self.comb += serdes.rx_reset.eq(ltssm.exit_to_rx_detect | self.serdes_rx_reset)

        # Endpoints --------------------------------------------------------------------------------
        self.sink   = pipe.sink
        self.source = pipe.source

        # Status -----------------------------------------------------------------------------------
        self.ready = Signal() # SerDes ready (PLL locked, transceiver reset done).
        self.comb += self.ready.eq(serdes.ready)
        for name in USB3PHY_STATUS:
            setattr(self, name, Signal(name=name))
            self.comb += getattr(self, name).eq(getattr(ltssm, name))

        # Link-Layer Control -----------------------------------------------------------------------
        for name, direction in USB3PHY_LINK_CTRL:
            setattr(self, name, getattr(ltssm, name))

        # LTSSM State ------------------------------------------------------------------------------
        ltssm.finalize() # Elaborate the FSM to expose its state/encoding.
        self.ltssm_states = ltssm.fsm.encoding # {state name: value}.
        self.ltssm_state  = Signal(len(ltssm.fsm.state))
        self.comb += self.ltssm_state.eq(ltssm.fsm.state)
