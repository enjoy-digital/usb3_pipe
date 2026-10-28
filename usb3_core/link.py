#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Link Layer (USB 3.2 chapter 7, SuperSpeed Gen1, Upstream Port).

- RX: framing parser (Header Packets, Link Commands, Data Packet Payloads), CRCs checks.
- Header Packets flow control (7.2.4.1): Header Sequence Numbers, Rx/Tx Header Buffers and credits
  (LGOOD_n/LCRD_x), LBAD/LRTY retries, Header Sequence Number/Rx Header Buffer Credit
  Advertisements on each U0 entry (link state kept across Recovery).
- SuperSpeedPlus operation (phy.gen2_active, Gen 2x1, 7.2.4.1): 4-bit Header Sequence Numbers,
  Type 1/Type 2 Rx Header Buffer Credits (LCRD1_x/LCRD2_x, 4 each), Type 2: asynchronous DPs (TT:
  Control/Bulk), Transfer Type (TT) set in the transmitted ACK TPs/DPHs (ssp_tt: {ept: TT}).
- Timers: PENDING_HP_TIMER (10us), CREDIT_HP_TIMER (Recovery on timeout), U0 LUP keep-alive.
- LMPs (8.4): Port Capability/Configuration (Response), Set Link Function (Force_LinkPM_Accept),
  U2 Inactivity Timeout.
- Link Power Management (7.2.4.2): partner LGO_Ux forwarded to the LTSSM (U3 always, U1/U2 when
  forced or when enabled and idle: no pending packets/Link Commands, all received packets
  acknowledged and credited, all transmitted packets acknowledged and credited, upper layers not
  busy), rejected with LXU otherwise. LAU/LXU sent when requested by the LTSSM, partner LPMA
  forwarded for accepted U1/U2 entries. U1/U2 exit requested when packets are pending (exit through
  Recovery, Advertisements on U0 re-entry).
- TX: framer (Link Commands, Header Packets, Data Packet Payloads with CRC-32/END framing).

The PHY is only accessed through its Link interface (see usb3_pipe.phy.USB3PHY): sink/source data/
ctrl words, u0/u1/u2/polling_idle/hot_reset status, lgo_u1/lgo_u2/lgo_u3/tx_lau/tx_lxu/lpma PM
handshake, exit_u1_req/exit_u2_req, u2_timeout, directed_recovery and link_activity.
"""

from functools import reduce
from operator import add

from migen import *
from migen.genlib.misc import WaitTimer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_core.common import *
from usb3_core.crc    import CRC5, CRC16, CRC16Words, CRC32
from usb3_core.dpp    import DPPSplitter

# Constants ----------------------------------------------------------------------------------------

LINK_HP_BUFFERS = 4 # Rx/Tx Header Buffers (SuperSpeed).

# Port Capability LMP (8.4.5): 5Gbps, 4 Rx Header Buffers, Upstream Port.
PORT_CAP_DW0      = TYPE_LMP | (LMP_SUB_PORT_CAP      << 5) | (1 << 9)
PORT_CAP_DW1      = LINK_HP_BUFFERS | (0b10 << 16)
# Port Configuration Response LMP (8.4.7): 5Gbps accepted.
PORT_CFG_RESP_DW0 = TYPE_LMP | (LMP_SUB_PORT_CFG_RESP << 5) | (1 << 9)

# USB3 Link Layer ----------------------------------------------------------------------------------

class USB3LinkLayer(LiteXModule):
    def __init__(self, phy, sys_clk_freq, timer_scale=1, ssp_tt={}):
        # PHY Link Layer words (32-bit) or words pairs (64-bit, USB3PHY link_words=2): DPP words of
        # 32 or 64 bits (8 bytes per cycle in the DPPs).
        lw = 2 if len(phy.sink.data) == 64 else 1
        dw = 32*lw
        self.data_width = dw

        # Protocol Layer interface.
        self.rx_hp       = stream.Endpoint(header_layout())                                # o
        self.rx_dpp      = stream.Endpoint(dpp_layout(dw) + [("ept", 4), ("setup", 1)])    # o
        self.rx_dpp_done = Signal() # o: DPP received (pulse), one per accepted DPH, in order.
        self.rx_dpp_ok   = Signal() # o: DPP received without error (with rx_dpp_done).
        self.tx_hp       = stream.Endpoint(tx_header_layout())                             # i
        self.tx_dpp      = stream.Endpoint(dpp_layout(dw))                                 # i

        # Control/Status.
        self.lpm_enable   = Signal() # i: Accept the partner's LGO_U1/LGO_U2 when idle (else LXU).
        self.pm_busy      = Signal() # i: Upper layers busy (ex: ACK TP expected): LGO_Ux rejected.
        self.ready        = Signal() # o: U0 and link initialized (Advertisement received).
        self.reset        = Signal() # o: Link reset (Polling/Hot Reset): Protocol Layer reset.

        # Statistics.
        self.rx_lbad      = Signal(16) # o: LBAD sent (received Header Packets errors).
        self.tx_retry     = Signal(16) # o: LBAD received (Header Packets retransmissions).
        self.recovery_req = Signal(16) # o: Recovery requests (timeouts, sequence/credit errors).

        # # #

        def scaled(t):
            return max(int(t*sys_clk_freq/timer_scale), 2)

        # SuperSpeed/SuperSpeedPlus operation ------------------------------------------------------
        # SuperSpeedPlus (Gen 2x1) when the PHY operates at Gen 2: 4-bit Header Sequence Numbers
        # (masked to 3 bits in SuperSpeed operation), Type 1/Type 2 credits.
        ssp_capable = hasattr(phy, "gen2_active")
        sw          = 4 if ssp_capable else 3 # Header Sequence Numbers width.
        ssp         = Signal()
        seq_mask    = Signal(sw)
        self.comb += [
            ssp.eq(phy.gen2_active if ssp_capable else 0),
            seq_mask.eq(Mux(ssp, 0xf, 0x7)),
        ]
        def is_type2(dw0, dw1):
            """Type 2 packet (SuperSpeedPlus): asynchronous DP (TT: Control/Bulk)."""
            return ssp & (dw0[0:5] == TYPE_DP) & ((dw1[12:15] == TT_CONTROL) | (dw1[12:15] == TT_BULK))

        # Link State -------------------------------------------------------------------------------
        u0      = Signal()
        u0_d    = Signal()
        u0_rise = Signal()
        self.comb += [
            u0.eq(phy.u0),
            u0_rise.eq(u0 & ~u0_d),
            self.reset.eq(phy.polling_idle | phy.hot_reset),
        ]
        self.sync += u0_d.eq(u0)

        recovery = Signal() # Recovery request (pulse).
        self.sync += phy.directed_recovery.eq(recovery)
        self.sync += If(recovery & u0 & (self.recovery_req != (2**16 - 1)),
            self.recovery_req.eq(self.recovery_req + 1)
        )

        # RX Words ---------------------------------------------------------------------------------
        # Registered (timings: the PHY RX datapath output is combinatorial).
        rx_valid = Signal()
        rx_data  = Signal(32, reset_less=True)
        rx_ctrl  = Signal(4, reset_less=True)
        rx_data1 = Signal(32) # Next word (words pairs: DPP data 8 bytes per cycle).
        rx_two   = Signal()   # Words pairs: current and next words consumed (DPP data).
        if lw == 1:
            self.comb += phy.source.ready.eq(1)
            self.sync += [
                rx_valid.eq(phy.source.valid & u0),
                rx_data.eq(swap_data(phy.source.data)),
                rx_ctrl.eq(swap_ctrl(phy.source.ctrl)),
            ]
        else:
            # Words window (registered): one word per cycle, two in the DPPs, words dropped out of U0.
            self.rx_window = rx_window = WordsWindow()
            self.comb += [
                phy.source.connect(rx_window.sink),
                rx_valid.eq((rx_window.n != 0) & u0),
                rx_data.eq(swap_data(rx_window.w0)),
                rx_ctrl.eq(swap_ctrl(rx_window.c0)),
                rx_data1.eq(swap_data(rx_window.w1)),
                rx_window.consume.eq(Mux(u0, Mux(rx_two, 2, rx_valid), rx_window.n)),
            ]
        rx_shp = Signal()
        rx_slc = Signal()
        rx_sdp = Signal()
        self.comb += [
            rx_shp.eq((rx_ctrl == 0b1111) & (rx_data == FRAMING_SHP)),
            rx_slc.eq((rx_ctrl == 0b1111) & (rx_data == FRAMING_SLC)),
            rx_sdp.eq((rx_ctrl == 0b1111) & (rx_data == FRAMING_SDP)),
        ]

        # RX Header Packets checks -----------------------------------------------------------------
        # Header Packet words: DW0, DW1, DW2, CRC-16/Link Control Word.
        hp_w      = [Signal(32, reset_less=True) for _ in range(4)]
        hp_last   = Signal() # Last Header Packet word received.
        hp_check  = Signal() # Header Packet received, checked (pulse).
        hp_ok     = Signal(reset_less=True)
        hp_seq    = Signal(sw)
        hp_dl     = Signal()
        hp_is_dp  = Signal()
        # CRC-16 computed on the words as received, checks registered with the last word (timings).
        self.crc16_rx = crc16_rx = CRC16Words()
        self.crc5_rx  = crc5_rx  = CRC5()
        self.comb += [
            crc16_rx.data.eq(rx_data),
            crc5_rx.data.eq(rx_data[16:27]),
        ]
        self.sync += If(hp_last,
            hp_ok.eq((crc16_rx.value == rx_data[0:16]) & (crc5_rx.value == rx_data[27:32]))
        )
        self.comb += [
            hp_seq.eq(hp_w[3][16:16+sw] & seq_mask),
            hp_dl.eq(hp_w[3][16 + 9]),
            hp_is_dp.eq(hp_w[0][0:5]  == TYPE_DP),
        ]
        self.sync += hp_check.eq(hp_last)

        # RX Link Commands checks ------------------------------------------------------------------
        # From the RX words or, when the PHY provides it (Gen 2), from the PHY LGOOD_n/LCRD_x Link
        # Commands words (source_lc, received without waiting behind the RX words), when no Link
        # Command is received from the RX words.
        lc_main  = Signal()   # Link Command word in the RX words.
        lc_fast  = Signal()   # Link Command word from the PHY source_lc.
        lc_data  = Signal(32)
        lc_valid = Signal()   # Link Command received (pulse, before registration).
        lc_check = Signal()   # Link Command received (pulse, registered: timings).
        lc_info  = Signal(11, reset_less=True)
        self.crc5_lc0 = crc5_lc0 = CRC5()
        self.crc5_lc1 = crc5_lc1 = CRC5()
        lc_ok0 = Signal()
        lc_ok1 = Signal()
        if hasattr(phy, "source_lc"):
            self.comb += [
                lc_fast.eq(phy.source_lc.valid & ~lc_main),
                phy.source_lc.ready.eq(lc_fast | ~u0),
                phy.source_lc_enable.eq(1),
                lc_data.eq(Mux(lc_fast, swap_data(phy.source_lc.data), rx_data)),
            ]
        else:
            self.comb += lc_data.eq(rx_data)
        self.comb += [
            crc5_lc0.data.eq(lc_data[0:11]),
            crc5_lc1.data.eq(lc_data[16:27]),
            lc_ok0.eq(crc5_lc0.value == lc_data[11:16]),
            lc_ok1.eq(crc5_lc1.value == lc_data[27:32]),
            lc_valid.eq((lc_main | (lc_fast & u0)) & (lc_ok0 | lc_ok1)),
        ]
        self.sync += [
            lc_check.eq(lc_valid),
            lc_info.eq(Mux(lc_ok0, lc_data[0:11], lc_data[16:27])),
        ]

        # RX DPP expectation (from the last accepted DPH) ------------------------------------------
        hp_accept    = Signal() # Header Packet accepted (with hp_check).
        dpp_expect   = Signal()
        dpp_length   = Signal(16, reset_less=True)
        dpp_ept      = Signal(4, reset_less=True)
        dpp_setup    = Signal(reset_less=True)
        dpp_new      = Signal() # Accepted DPH (with DPP) this cycle.
        expect_now   = Signal()
        length_now   = Signal(16)
        dpp_missing  = Signal()
        self.comb += [
            dpp_new.eq(hp_check & hp_accept & hp_is_dp & ~hp_dl),
            expect_now.eq(dpp_expect | dpp_new),
            length_now.eq(Mux(dpp_new, hp_w[1][16:32], dpp_length)),
        ]
        self.sync += [
            If(dpp_new,
                dpp_expect.eq(1),
                dpp_length.eq(hp_w[1][16:32]),
                dpp_ept.eq(hp_w[1][8:12]),
                dpp_setup.eq(hp_w[1][15]),
            ),
            If(~u0, dpp_expect.eq(0)),
        ]

        # RX Parser --------------------------------------------------------------------------------
        hp_count  = Signal(2, reset_less=True)
        dpp_rem   = Signal(17, reset_less=True)
        dpp_off   = Signal(2, reset_less=True)
        dpp_first = Signal(reset_less=True)
        dpp_prev  = Signal(32, reset_less=True)
        dpp_pb    = Signal(3)
        dpp_crc   = Signal(32)
        dpp_end   = Signal() # DPP CRC word received (pulse).
        self.crc32_rx = crc32_rx = CRC32(data_width=dw)
        # Words pairs: DPP words of 8 bytes (two words: dpp_two, or a held word and the next one),
        # the last one 1-8 bytes; CRC-32 on the words as consumed.
        dpp_two    = Signal() # Two DPP data words consumed (8 bytes, not the last ones).
        dpp_hold   = Signal() # DPP data word held (no DPP word output).
        dpp_held   = Signal(32, reset_less=True)
        dpp_held_v = Signal()
        rx_fsm_dpp_data = Signal()
        if lw == 2:
            self.comb += [
                dpp_two.eq(rx_fsm_dpp_data & ~dpp_held_v & (rx_window.n == 2) & (dpp_rem >= 8)),
                dpp_hold.eq(~dpp_two & ~dpp_held_v & (dpp_rem > 4)),
            ]
        self.comb += [
            dpp_pb.eq(Mux(dpp_rem >= 4, 4, dpp_rem)),
            Case(dpp_off, {
                0: dpp_crc.eq(rx_data),
                1: dpp_crc.eq(Cat(dpp_prev[ 8:32], rx_data[0: 8])),
                2: dpp_crc.eq(Cat(dpp_prev[16:32], rx_data[0:16])),
                3: dpp_crc.eq(Cat(dpp_prev[24:32], rx_data[0:24])),
            }),
            crc32_rx.data.eq(Cat(rx_data, Mux(dpp_two, rx_data1, 0))[:dw]),
            crc32_rx.bytes.eq(Mux(dpp_two, 8, dpp_pb)),
            self.rx_dpp.data.eq(Mux(dpp_held_v,
                Cat(dpp_held, rx_data), Cat(rx_data, Mux(dpp_two, rx_data1, 0)))[:dw]),
            self.rx_dpp.bytes.eq(Mux(dpp_two, 8, Mux(dpp_held_v, 4 + dpp_pb, dpp_pb))),
            # DPH with DL bit (no DPP): routed with the DPH fields (rx_dpp_done on hp_check).
            If(hp_check,
                self.rx_dpp.ept.eq(hp_w[1][8:12]),
                self.rx_dpp.setup.eq(hp_w[1][15]),
            ).Else(
                self.rx_dpp.ept.eq(dpp_ept),
                self.rx_dpp.setup.eq(dpp_setup),
            ),
        ]

        self.rx_fsm = rx_fsm = FSM(reset_state="IDLE")
        self.comb += rx_fsm_dpp_data.eq(rx_fsm.ongoing("DPP") & u0)
        rx_fsm.act("IDLE",
            If(rx_valid,
                If(rx_shp,
                    dpp_missing.eq(expect_now),
                    crc16_rx.clear.eq(1),
                    NextValue(hp_count, 0),
                    NextState("HP")
                ).Elif(rx_slc,
                    dpp_missing.eq(expect_now),
                    NextState("LC")
                ).Elif(rx_sdp & expect_now,
                    crc32_rx.clear.eq(1),
                    NextValue(dpp_rem,   length_now),
                    NextValue(dpp_off,   length_now[0:2]),
                    NextValue(dpp_first, 1),
                    NextValue(dpp_held_v, 0),
                    NextState("DPP")
                )
            )
        )
        rx_fsm.act("HP",
            If(rx_valid,
                Case(hp_count, {i: NextValue(hp_w[i], rx_data) for i in range(4)}),
                NextValue(hp_count, hp_count + 1),
                crc16_rx.ce.eq(hp_count != 3),
                If(hp_count == 3,
                    hp_last.eq(1),
                    NextState("IDLE")
                )
            )
        )
        rx_fsm.act("LC",
            If(rx_valid,
                lc_main.eq(1),
                NextState("IDLE")
            )
        )
        rx_fsm.act("DPP",
            If(~u0,
                NextState("IDLE")
            ).Elif(rx_valid,
                If(dpp_rem != 0,
                    self.rx_dpp.valid.eq(~dpp_hold),
                    self.rx_dpp.first.eq(dpp_first),
                    self.rx_dpp.last.eq(Mux(dpp_two, dpp_rem == 8, dpp_rem <= 4)),
                    crc32_rx.ce.eq(1),
                    rx_two.eq(dpp_two),
                    If(~dpp_hold, NextValue(dpp_first, 0)),
                    NextValue(dpp_held,   rx_data),
                    NextValue(dpp_held_v, dpp_hold),
                    NextValue(dpp_rem,  dpp_rem - Mux(dpp_two, 8, dpp_pb)),
                    NextValue(dpp_prev, Mux(dpp_two, rx_data1, rx_data)),
                ).Else(
                    dpp_end.eq(1),
                    NextState("IDLE")
                )
            )
        )
        self.comb += [
            self.rx_dpp_done.eq(dpp_end | dpp_missing | (hp_check & hp_accept & hp_is_dp & hp_dl)),
            self.rx_dpp_ok.eq(dpp_end & (dpp_crc == crc32_rx.value)),
        ]
        self.sync += If(dpp_end | dpp_missing, dpp_expect.eq(0))

        # Link activity (U0 Recovery timeout of the LTSSM): any received packet/Link Command.
        self.sync += phy.link_activity.eq(hp_check | lc_check | dpp_end)

        # RX Header Buffers ------------------------------------------------------------------------
        rx_buffers = 2*LINK_HP_BUFFERS if ssp_capable else LINK_HP_BUFFERS # Type 1 + Type 2.
        rx_buffer = ResetInserter()(stream.SyncFIFO(header_layout(), rx_buffers))
        self.rx_buffer = rx_buffer
        self.comb += [
            rx_buffer.reset.eq(self.reset),
            rx_buffer.sink.dw0.eq(hp_w[0]),
            rx_buffer.sink.dw1.eq(hp_w[1]),
            rx_buffer.sink.dw2.eq(hp_w[2]),
        ]

        # Header Packets flow control state.
        rx_seq          = Signal(sw) # Rx Header Sequence Number (next expected).
        lbad_wait       = Signal()   # LBAD sent, waiting for LRTY.
        lgood_seq       = Signal(sw) # Next LGOOD_n to send (pending while != rx_seq).
        lcrd_pending    = Signal(3)  # LCRD_x/LCRD1_x to send.
        lcrd_idx        = Signal(2)  # Next LCRD_x/LCRD1_x index.
        lcrd2_pending   = Signal(3)  # LCRD2_x to send (SuperSpeedPlus).
        lcrd2_idx       = Signal(2)  # Next LCRD2_x index.
        rx_t2_level     = Signal(4)  # Type 2 packets in the Rx Header Buffers.
        lbad_pending    = Signal()
        lrty_pending    = Signal()
        tx_seq           = Signal(sw) # Tx Header Sequence Number (next new Header Packet).
        ack_seq          = Signal(sw) # ACK Tx Header Sequence Number (next expected LGOOD_n).
        send_seq         = Signal(sw) # Next Header Packet to (re)send.
        remote_credits   = Signal(3)  # Remote (Type 1) Rx Header Buffer Credit Count.
        remote_lcrd_idx  = Signal(2)  # Next expected LCRD_x/LCRD1_x.
        remote_credits2  = Signal(3)  # Remote Type 2 Rx Buffer Credit Count (SuperSpeedPlus).
        remote_lcrd2_idx = Signal(2)  # Next expected LCRD2_x.
        adv_received    = Signal()  # Header Sequence Number Advertisement received.
        lau_pending     = Signal()
        lxu_pending     = Signal()
        lup_pending     = Signal()
        pm_block        = Signal()  # LAU sent: no more transmission until the next U0 entry.
        portcap_pending = Signal(reset=1)
        portcfg_pending = Signal()
        force_pm_accept = Signal()
        pm_accepted     = Signal()  # LGO_U1/LGO_U2 accepted: no new Header Packets until next U0.
        u2_timeout      = Signal(8) # U2 Inactivity Timeout (LMP).

        self.comb += self.ready.eq(u0 & adv_received & ~pm_block)

        # RX Header Packets accept (7.2.4.1.6/7.2.4.1.8).
        rx_hp_bad   = Signal()
        rx_hp_seqer = Signal()
        self.comb += [
            rx_hp_bad.eq(hp_check & ~hp_ok & ~lbad_wait),
            rx_hp_seqer.eq(hp_check & hp_ok & ~lbad_wait &
                ((hp_seq != rx_seq) | ~rx_buffer.sink.ready)),
            hp_accept.eq(hp_check & hp_ok & ~lbad_wait & (hp_seq == rx_seq) & rx_buffer.sink.ready),
            rx_buffer.sink.valid.eq(hp_accept),
        ]

        # RX Header Buffers: LMPs consumed by the Link Layer, others to the Protocol Layer.
        lmp_pop   = Signal()
        lmp_sub   = Signal(4)
        rx_pop    = Signal()
        self.comb += [
            lmp_sub.eq(rx_buffer.source.dw0[5:9]),
            If(rx_buffer.source.dw0[0:5] == TYPE_LMP,
                rx_buffer.source.ready.eq(1),
                lmp_pop.eq(rx_buffer.source.valid),
            ).Else(
                rx_buffer.source.connect(self.rx_hp),
            ),
            rx_pop.eq(rx_buffer.source.valid & rx_buffer.source.ready),
        ]
        rx_pop_t2    = Signal()
        rx_accept_t2 = Signal()
        self.comb += [
            rx_pop_t2.eq(rx_pop & is_type2(rx_buffer.source.dw0, rx_buffer.source.dw1)),
            rx_accept_t2.eq(hp_accept & is_type2(hp_w[0], hp_w[1])),
        ]

        # RX Link Commands decoding.
        lc_lgood = Signal()
        lc_lcrd  = Signal()
        lc_lrty  = Signal()
        lc_lbad  = Signal()
        lc_lgo   = Signal()
        lc_lpma  = Signal()
        self.comb += [
            lc_lgood.eq(lc_check & ((lc_info & Mux(ssp, 0x7f0, 0x7f8)) == LCMD_LGOOD)),
            lc_lcrd.eq( lc_check & ((lc_info & Mux(ssp, 0x7f8, 0x7fc)) == LCMD_LCRD)),
            lc_lrty.eq( lc_check & ( lc_info          == LCMD_LRTY)),
            lc_lbad.eq( lc_check & ( lc_info          == LCMD_LBAD)),
            lc_lgo.eq(  lc_check & ((lc_info & 0x7fc) == LCMD_LGO_U) & (lc_info[0:2] != 0)),
            lc_lpma.eq( lc_check & ( lc_info          == LCMD_LPMA)),
        ]

        # Link Power Management (7.2.4.2) ----------------------------------------------------------
        # LGO_U1/LGO_U2 accepted (7.2.4.2.2) when forced (Force_LinkPM_Accept) or when enabled and:
        # a. LGOOD_n and LCRD_x sent for all the received Header Packets (Rx Header Buffers empty).
        # b. LGOOD_n and LCRD_x received for all the transmitted Header Packets.
        # c. No pending packets for transmission (Link Layer LMPs, Protocol Layer Header Packets).
        # d. Not rejected by the upper layers (pm_busy: ex ACK TP expected for a sent DP).
        # LGO_U3 always accepted (7.2.4.2.4). Forwarded to the LTSSM (it requests LAU), else LXU.
        lpm_idle = Signal()
        self.comb += lpm_idle.eq(adv_received &
            (lgood_seq == rx_seq) & (lcrd_pending == 0) & (lcrd2_pending == 0) & ~rx_buffer.source.valid &
            ~lbad_wait & ~lbad_pending &
            (ack_seq == tx_seq) & (send_seq == tx_seq) & (remote_credits == LINK_HP_BUFFERS) &
            (~ssp | (remote_credits2 == LINK_HP_BUFFERS)) &
            ~lrty_pending &
            ~portcap_pending & ~portcfg_pending & ~self.tx_hp.valid &
            ~lau_pending & ~lxu_pending & ~pm_accepted & ~self.pm_busy)
        lgo_u12 = Signal()
        lgo_fwd = Signal()
        self.comb += [
            lgo_u12.eq(lc_lgo & (lc_info[0:2] != 3)),
            lgo_fwd.eq(lc_lgo &
                ((lc_info[0:2] == 3) | force_pm_accept | (self.lpm_enable & lpm_idle))),
        ]
        self.sync += [
            phy.lgo_u1.eq(lgo_fwd & (lc_info[0:2] == 1)),
            phy.lgo_u2.eq(lgo_fwd & (lc_info[0:2] == 2)),
            phy.lgo_u3.eq(lgo_fwd & (lc_info[0:2] == 3)),
            # LPMA of an accepted U1/U2 entry: Ux entered without waiting for PM_ENTRY_TIMER.
            phy.lpma.eq(lc_lpma & pm_accepted),
            If(lgo_fwd & lgo_u12,
                pm_accepted.eq(1)
            ).Elif(~u0 | phy.tx_lxu,
                pm_accepted.eq(0)
            ),
        ]

        # U1/U2 exit when packets are to be transmitted (ex: ERDY, 7.2.4.2.7). The LTSSM waits for
        # U1_MIN_RESIDENCY before initiating the U1 exit.
        tx_pending = Signal()
        self.comb += [
            tx_pending.eq(self.tx_hp.valid | (send_seq != tx_seq) | portcap_pending |
                portcfg_pending),
            phy.exit_u1_req.eq(phy.u1 & tx_pending),
            phy.exit_u2_req.eq(phy.u2 & tx_pending),
            phy.u2_timeout.eq(u2_timeout),
        ]

        # TX Header Buffers ------------------------------------------------------------------------
        # SuperSpeed: 4 unacknowledged Header Packets. SuperSpeedPlus: 4 Type 1 and 4 Type 2
        # (7.2.4.1, Gen 1x2/Gen 2x1): 8 buffers (indexed by the Header Sequence Number LSBs).
        tx_bufs = 2*LINK_HP_BUFFERS if ssp_capable else LINK_HP_BUFFERS
        ib      = log2_int(tx_bufs)
        tx_dws  = Array(Signal(96, reset_less=True) for _ in range(tx_bufs))
        tx_dpps = Array(Signal(reset_less=True)     for _ in range(tx_bufs))
        tx_sent = Array(Signal()                     for _ in range(tx_bufs))
        tx_t2   = Array(Signal(reset_less=True)     for _ in range(tx_bufs)) # Type 2.
        tx_count  = Signal(4)
        tx_count2 = Signal(4) # Type 2 Header Packets in the buffers.
        tx_full   = Signal()  # SuperSpeed: buffers full.
        tx_full1  = Signal()  # SuperSpeedPlus: Type 1 buffers full.
        tx_full2  = Signal()  # SuperSpeedPlus: Type 2 buffers full.
        tx_in_buf = [Signal(name=f"tx_in_buf{i}") for i in range(tx_bufs)]
        self.comb += [
            tx_count.eq((tx_seq - ack_seq) & seq_mask),
            [tx_in_buf[i].eq(((i - ack_seq[0:ib]) & (tx_bufs - 1)) < tx_count) for i in range(tx_bufs)],
            tx_count2.eq(reduce(add, [Cat(tx_in_buf[i] & tx_t2[i], C(0, 3)) for i in range(tx_bufs)])),
            tx_full.eq(tx_count >= LINK_HP_BUFFERS),
            tx_full1.eq((tx_count - tx_count2) >= LINK_HP_BUFFERS),
            tx_full2.eq(tx_count2 >= LINK_HP_BUFFERS),
        ]

        # Header Packets from the Link Layer (LMPs) or the Protocol Layer.
        lmp_push     = Signal()
        lmp_push_dw0 = Signal(32)
        lmp_push_dw1 = Signal(32)
        tx_push      = Signal()
        tx_push_dws  = Signal(96)
        tx_push_dpp  = Signal()
        tx_dw1       = Signal(32)
        tx_hp_full   = Signal() # No buffer for the Protocol Layer Header Packet.
        self.comb += [
            tx_hp_full.eq(Mux(ssp,
                Mux(is_type2(self.tx_hp.dw0, tx_dw1), tx_full2, tx_full1),
                tx_full)),
            lmp_push.eq(adv_received & Mux(ssp, ~tx_full1, ~tx_full) &
                (portcap_pending | portcfg_pending)),
            # SuperSpeedPlus operation: Link Speed and Num HP Buffers fields reserved (zero, 8.4.5).
            If(portcap_pending,
                lmp_push_dw0.eq(Mux(ssp, PORT_CAP_DW0 & ~(0x7f << 9), PORT_CAP_DW0)),
                lmp_push_dw1.eq(Mux(ssp, PORT_CAP_DW1 & ~0xff,        PORT_CAP_DW1)),
            ).Else(
                lmp_push_dw0.eq(Mux(ssp, PORT_CFG_RESP_DW0 & ~(0x7f << 9), PORT_CFG_RESP_DW0)),
            ),
            self.tx_hp.ready.eq(self.ready & ~tx_hp_full & ~lmp_push & ~pm_accepted),
            If(lmp_push,
                tx_push.eq(1),
                tx_push_dws.eq(Cat(lmp_push_dw0, lmp_push_dw1, C(0, 32))),
            ).Else(
                tx_push.eq(self.tx_hp.valid & self.tx_hp.ready),
                tx_push_dws.eq(Cat(self.tx_hp.dw0, tx_dw1, self.tx_hp.dw2)),
                tx_push_dpp.eq(self.tx_hp.dpp),
            ),
        ]
        # Transfer Type (TT) of the ACK TPs/DPHs in SuperSpeedPlus operation (8.2, 8.10.4).
        tx_tt = Signal(3)
        self.comb += Case(self.tx_hp.dw1[8:12], {**{ept: tx_tt.eq(tt) for ept, tt in ssp_tt.items()},
            "default": tx_tt.eq(TT_BULK)})
        self.comb += tx_dw1.eq(self.tx_hp.dw1)
        self.comb += If(ssp & ((self.tx_hp.dw0[0:5] == TYPE_DP) |
            ((self.tx_hp.dw0[0:5] == TYPE_TP) & (self.tx_hp.dw1[0:4] == TP_SUB_ACK))),
            tx_dw1[12:15].eq(tx_tt)
        )

        # TX Framer --------------------------------------------------------------------------------
        tx_word  = Signal(32)
        tx_word2 = Signal(32) # Words pairs: second word (DPP data, tx_two).
        tx_two   = Signal()
        tx_ctrl  = Signal(4)
        tx_valid = Signal()
        self.tx_pipe = tx_pipe = stream.PipeValid(phy_layout() if lw == 1 else link_pairs_layout()) # Registered (timings).
        self.comb += [
            tx_pipe.sink.valid.eq(tx_valid),
            tx_pipe.sink.data.eq(Cat(swap_data(tx_word), swap_data(tx_word2))),
            tx_pipe.sink.ctrl.eq(swap_ctrl(tx_ctrl)),
            tx_pipe.sink.words.eq(Mux(tx_two, 2, 1)) if lw == 2 else [],
            tx_pipe.source.connect(phy.sink),
        ]
        tx_ready = tx_pipe.sink.ready

        # Selected Link Command/Header Packet.
        lc_word   = Signal(32, reset_less=True)
        hp_dws    = Signal(96, reset_less=True)
        hp_dpp    = Signal(reset_less=True)
        hp_null   = Signal(reset_less=True) # Retransmitted DPH: nullified DPP (SuperSpeedPlus).
        hp_lcw    = Signal(16)
        hp_info   = Signal(11, reset_less=True)
        self.crc16_tx  = crc16_tx  = CRC16()
        self.crc5_tx   = crc5_tx   = CRC5()
        self.crc5_lctx = crc5_lctx = CRC5()
        self.comb += [
            crc16_tx.data.eq(hp_dws),
            crc5_tx.data.eq(hp_info),
            hp_lcw.eq(Cat(hp_info, crc5_tx.value)),
        ]
        lc_sel = Signal(11)
        self.comb += crc5_lctx.data.eq(lc_sel)

        # Transmission requests (by priority).
        can_send_hp = Signal()
        send_t2 = Signal()
        self.comb += [
            send_t2.eq(tx_t2[send_seq[0:ib]]),
            can_send_hp.eq(adv_received & (send_seq != tx_seq) &
                Mux(send_t2, remote_credits2 != 0, remote_credits != 0) & ~lrty_pending),
        ]
        start_hp = Signal()
        sel_lrty = Signal()
        sel_lbad = Signal()
        sel_lgood = Signal()
        sel_lcrd = Signal()
        sel_lcrd2 = Signal()
        sel_lau  = Signal()
        sel_lxu  = Signal()
        sel_lup  = Signal()
        self.comb += [
            If(~pm_block,
                If(lrty_pending,
                    sel_lrty.eq(1), lc_sel.eq(LCMD_LRTY)
                ).Elif(lbad_pending,
                    sel_lbad.eq(1), lc_sel.eq(LCMD_LBAD)
                ).Elif(lgood_seq != rx_seq,
                    sel_lgood.eq(1), lc_sel.eq(LCMD_LGOOD | lgood_seq)
                ).Elif(lcrd_pending != 0,
                    sel_lcrd.eq(1), lc_sel.eq(LCMD_LCRD | lcrd_idx)
                ).Elif(lcrd2_pending != 0,
                    sel_lcrd2.eq(1), lc_sel.eq(LCMD_LCRD | 0b100 | lcrd2_idx)
                ).Elif(lau_pending,
                    sel_lau.eq(1), lc_sel.eq(LCMD_LAU)
                ).Elif(lxu_pending,
                    sel_lxu.eq(1), lc_sel.eq(LCMD_LXU)
                ).Elif(can_send_hp,
                    start_hp.eq(1)
                ).Elif(lup_pending,
                    sel_lup.eq(1), lc_sel.eq(LCMD_LUP)
                )
            ),
        ]

        dpp_b     = Signal(3, reset_less=True)
        dpp_tail1 = Signal(32, reset_less=True)
        dpp_tail2 = Signal(32, reset_less=True)
        dpp_ctrl1 = Signal(4, reset_less=True)
        dpp_ctrl2 = Signal(4, reset_less=True)
        # DPP words: 32-bit, or 64-bit split in 8-byte words and 32-bit words for the last bytes.
        if lw == 1:
            tx_dpp = self.tx_dpp
        else:
            self.tx_dpp_splitter = tx_dpp_splitter = DPPSplitter()
            self.comb += self.tx_dpp.connect(tx_dpp_splitter.sink)
            tx_dpp = tx_dpp_splitter.source
            self.comb += tx_word2.eq(tx_dpp.data[32:64])
        self.crc32_tx = crc32_tx = CRC32(data_width=dw)
        self.comb += [
            crc32_tx.data.eq(tx_dpp.data),
            crc32_tx.bytes.eq(tx_dpp.bytes),
        ]
        crc_next = crc32_tx.next_value
        end_bytes = [(FRAMING_END >> 8*i) & 0xff for i in range(4)]
        def end_slice(i, j):
            return Cat(*[C(end_bytes[k], 8) for k in range(i, j)])
        tail_cases = {
            0: [
                NextValue(dpp_tail1, FRAMING_END), NextValue(dpp_ctrl1, 0b1111),
            ],
            4: [
                NextValue(dpp_tail1, crc_next),    NextValue(dpp_ctrl1, 0b0000),
                NextValue(dpp_tail2, FRAMING_END), NextValue(dpp_ctrl2, 0b1111),
            ],
        }
        for b in range(1, 4):
            tail_cases[b] = [
                NextValue(dpp_tail1, Cat(crc_next[32-8*b:32], end_slice(0, 4-b))),
                NextValue(dpp_ctrl1, ((1 << (4-b)) - 1) << b),
                NextValue(dpp_tail2, Cat(end_slice(4-b, 4), C(0, 8*(4-b)))),
                NextValue(dpp_ctrl2, (1 << b) - 1),
            ]
        dpp_word = Signal(32)
        self.comb += If(tx_dpp.last,
            Case(tx_dpp.bytes, {b: dpp_word.eq(
                Cat(tx_dpp.data[0:8*b], crc_next[0:32-8*b]) if b not in [0, 4] else
                (crc_next if b == 0 else tx_dpp.data)) for b in range(5)})
        ).Else(
            dpp_word.eq(tx_dpp.data)
        )

        tx_sent_lc  = Signal() # Link Command/Header Packet transmission started (pulse).
        tx_sent_hp  = Signal()
        self.tx_fsm = tx_fsm = FSM(reset_state="IDLE")
        tx_fsm.act("IDLE",
            If(u0,
                If(start_hp,
                    tx_sent_hp.eq(1),
                    NextValue(hp_dws,  tx_dws[send_seq[0:ib]]),
                    NextValue(hp_dpp,  tx_dpps[send_seq[0:ib]] & ~tx_sent[send_seq[0:ib]]),
                    NextValue(hp_null, tx_dpps[send_seq[0:ib]] &  tx_sent[send_seq[0:ib]] & ssp),
                    NextValue(hp_info, Cat(send_seq, C(0, 9 - sw), tx_sent[send_seq[0:ib]], C(0, 1))),
                    NextState("HP0")
                ).Elif(sel_lrty | sel_lbad | sel_lgood | sel_lcrd | sel_lcrd2 | sel_lau | sel_lxu | sel_lup,
                    tx_sent_lc.eq(1),
                    NextValue(lc_word, Cat(lc_sel, crc5_lctx.value, lc_sel, crc5_lctx.value)),
                    NextState("LC0")
                )
            )
        )
        tx_fsm.act("LC0",
            tx_valid.eq(1),
            tx_word.eq(FRAMING_SLC),
            tx_ctrl.eq(0b1111),
            If(tx_ready, NextState("LC1"))
        )
        tx_fsm.act("LC1",
            tx_valid.eq(1),
            tx_word.eq(lc_word),
            If(tx_ready, NextState("IDLE"))
        )
        for i in range(5):
            tx_fsm.act(f"HP{i}",
                tx_valid.eq(1),
                tx_word.eq([FRAMING_SHP, hp_dws[0:32], hp_dws[32:64], hp_dws[64:96],
                    Cat(crc16_tx.value, hp_lcw)][i]),
                tx_ctrl.eq(0b1111 if i == 0 else 0b0000),
                If(tx_ready,
                    NextState(f"HP{i+1}") if i < 4 else
                    If(hp_dpp,
                        NextState("DPP-SOF")
                    ).Elif(hp_null,
                        NextState("DPP-ABORT")
                    ).Else(
                        NextState("IDLE")
                    )
                )
            )
        # Retransmitted DPH (DL bit set) in SuperSpeedPlus operation: DPP replaced by a nullified DPP
        # (DPPABORT, 7.2.4.1.13/7.2.1.2.2), the Gen 2 framing has a DPP after each DPH.
        tx_fsm.act("DPP-ABORT",
            tx_valid.eq(1),
            tx_word.eq(FRAMING_EDB),
            tx_ctrl.eq(0b1111),
            If(tx_ready, NextState("IDLE"))
        )
        tx_fsm.act("DPP-SOF",
            tx_valid.eq(1),
            tx_word.eq(FRAMING_SDP),
            tx_ctrl.eq(0b1111),
            crc32_tx.clear.eq(1),
            If(tx_ready, NextState("DPP-DATA"))
        )
        tx_fsm.act("DPP-DATA",
            tx_valid.eq(tx_dpp.valid),
            tx_word.eq(dpp_word),
            tx_two.eq(tx_dpp.two) if lw == 2 else [],
            tx_dpp.ready.eq(tx_ready),
            If(tx_dpp.valid & tx_ready,
                crc32_tx.ce.eq(1),
                If(tx_dpp.last,
                    NextValue(dpp_b, tx_dpp.bytes),
                    Case(tx_dpp.bytes, tail_cases),
                    NextState("DPP-TAIL1")
                )
            )
        )
        tx_fsm.act("DPP-TAIL1",
            tx_valid.eq(1),
            tx_word.eq(dpp_tail1),
            tx_ctrl.eq(dpp_ctrl1),
            If(tx_ready,
                If(dpp_b == 0, NextState("IDLE")).Else(NextState("DPP-TAIL2"))
            )
        )
        tx_fsm.act("DPP-TAIL2",
            tx_valid.eq(1),
            tx_word.eq(dpp_tail2),
            tx_ctrl.eq(dpp_ctrl2),
            If(tx_ready, NextState("IDLE"))
        )

        # Timers -----------------------------------------------------------------------------------
        # U0 keep-alive: LUP when nothing sent for 10us (7.5.6.1).
        self.t_lup = t_lup = WaitTimer(scaled(10e-6))
        self.comb += t_lup.wait.eq(u0 & ~pm_block & ~tx_sent_lc & ~tx_sent_hp & ~lup_pending)
        # PENDING_HP_TIMER (10us, Table 7-7): Header Packets sent and not acknowledged, Advertisement
        # expected (the partner returns LGOOD_n within 3us/1.5us in SuperSpeed/SuperSpeedPlus operation).
        # SuperSpeedPlus at Gen 2: the Header Packets are transmitted on the line after their DPP is
        # stored (Gen 2 TX framing, DPH and DPP without spacing): the timer is extended by the
        # maximum DPP buffering time (1024 bytes on the 32-bit Link Layer words).
        pending_hp_ssp = 10e-6 + (1024 + 64)/4/sys_clk_freq
        self.t_pending_hp = t_pending_hp = WaitTimer(scaled(pending_hp_ssp if ssp_capable else 10e-6))
        t_pending_hp_ss = WaitTimer(scaled(10e-6))
        self.submodules += t_pending_hp_ss
        pending_hp_done = Signal()
        self.comb += [
            t_pending_hp.wait.eq(u0 & ~pm_block & ~lc_lgood & (~adv_received | (ack_seq != send_seq))),
            t_pending_hp_ss.wait.eq(t_pending_hp.wait),
            pending_hp_done.eq(Mux(ssp, t_pending_hp.done, t_pending_hp_ss.done)),
        ]
        # CREDIT_HP_TIMER (5ms): Remote Rx Header Buffer Credits missing.
        self.t_credit_hp = t_credit_hp = WaitTimer(scaled(5e-3))
        self.comb += t_credit_hp.wait.eq(u0 & ~pm_block & ~lc_lcrd &
            ((remote_credits != LINK_HP_BUFFERS) | (ssp & (remote_credits2 != LINK_HP_BUFFERS))))

        # Flow Control -----------------------------------------------------------------------------
        # Credits of the Header Packets to retransmit on LBAD (sent, not acknowledged) per type.
        lbad_t1 = Signal(3)
        lbad_t2 = Signal(3)
        lbad_n  = Signal(sw)
        self.comb += lbad_n.eq((send_seq - ack_seq) & seq_mask)
        in_range = [Signal(name=f"lbad_in_range{i}") for i in range(tx_bufs)]
        for i in range(tx_bufs):
            self.comb += in_range[i].eq(((i - ack_seq[0:ib]) & (tx_bufs - 1)) < lbad_n)
        self.comb += [
            lbad_t2.eq(reduce(add, [Cat(in_range[i] & tx_t2[i], C(0, 2)) for i in range(tx_bufs)])),
            lbad_t1.eq(lbad_n - lbad_t2),
        ]
        lgood_ok  = Signal()
        lcrd_ok   = Signal()
        self.comb += [
            lgood_ok.eq((lc_info[0:sw] & seq_mask) == ack_seq),
            lcrd_ok.eq(lc_info[0:2] == Mux(lc_info[2], remote_lcrd2_idx, remote_lcrd_idx)),
            recovery.eq(u0 & ~pm_block & (
                rx_hp_seqer |
                (lc_lgood & adv_received & ~lgood_ok) |
                (lc_lcrd & ~lcrd_ok) |
                pending_hp_done |
                t_credit_hp.done)),
        ]
        # Debug (USB3LinkTrace): Link Commands/Header Packets events, Recovery causes.
        self.dbg_rx_lc     = lc_check
        self.dbg_rx_lc_info = lc_info
        self.dbg_tx_lc     = tx_sent_lc
        self.dbg_tx_lc_sel = lc_sel
        self.dbg_rx_hp     = hp_check
        self.dbg_rx_hp_info = Cat(hp_w[0][0:5], hp_w[0][5:9], hp_seq[0:4] if sw == 4 else Cat(hp_seq, C(0, 1)),
            hp_ok, hp_accept, rx_hp_seqer)
        self.dbg_tx_hp     = tx_sent_hp
        self.dbg_tx_hp_info = Cat(send_seq[0:4] if sw == 4 else Cat(send_seq, C(0, 1)),
            tx_dws[send_seq[0:ib]][0:5], tx_sent[send_seq[0:ib]])
        self.dbg_u0        = u0
        self.dbg_lgood_pending = u0 & (lgood_seq != rx_seq)
        self.dbg_lcrd_pending  = u0 & ((lcrd_pending != 0) | (lcrd2_pending != 0))
        self.dbg_dpp_stall     = tx_fsm.ongoing("DPP-DATA") & ~tx_dpp.valid
        self.dbg_tx_full       = u0 & self.tx_hp.valid & tx_hp_full           # Tx Header Buffers full (LGOOD_n wait).
        self.dbg_tx_credit     = u0 & (send_seq != tx_seq) & ~can_send_hp     # Header Packet waiting for a credit.
        self.dbg_tx_idle       = u0 & tx_fsm.ongoing("IDLE")
        self.dbg_tx_count      = tx_count                                     # Header Packets in the Tx Header Buffers.
        self.dbg_tx_push       = tx_push
        self.dbg_lgood_rx      = lc_lgood & adv_received & lgood_ok           # Expected LGOOD_n received...
        self.dbg_lgood_seq     = lc_info[0:4]                                 # ...for this Header Sequence Number.
        self.dbg_lbad_rx       = lc_lbad                                      # LBAD received.
        self.dbg_state     = Cat(tx_seq[0:4] if sw == 4 else Cat(tx_seq, C(0, 1)),
            ack_seq[0:4] if sw == 4 else Cat(ack_seq, C(0, 1)),
            send_seq[0:4] if sw == 4 else Cat(send_seq, C(0, 1)),
            remote_credits, remote_credits2, adv_received, pm_block, pm_accepted,
            self.tx_hp.valid, self.tx_hp.ready, lrty_pending, lbad_wait, ssp, tx_full,
            rx_seq[0:4] if sw == 4 else Cat(rx_seq, C(0, 1)))
        self.dbg_recovery  = recovery
        self.dbg_recovery_cause = Cat(rx_hp_seqer, lc_lgood & adv_received & ~lgood_ok,
            lc_lcrd & ~lcrd_ok, pending_hp_done, t_credit_hp.done)

        self.sync += [
            # Header Packets received.
            If(hp_accept, rx_seq.eq((rx_seq + 1) & seq_mask)),
            rx_t2_level.eq(rx_t2_level + rx_accept_t2 - rx_pop_t2),
            If(rx_hp_bad,
                lbad_wait.eq(1),
                lbad_pending.eq(1),
                If(self.rx_lbad != (2**16 - 1), self.rx_lbad.eq(self.rx_lbad + 1)),
            ),
            If(lc_lrty, lbad_wait.eq(0)),

            # Link Commands sent.
            If(tx_sent_lc,
                If(sel_lrty,  lrty_pending.eq(0)),
                If(sel_lbad,  lbad_pending.eq(0)),
                If(sel_lgood, lgood_seq.eq((lgood_seq + 1) & seq_mask)),
                If(sel_lcrd,  lcrd_idx.eq(lcrd_idx + 1)),
                If(sel_lcrd2, lcrd2_idx.eq(lcrd2_idx + 1)),
                If(sel_lau,   lau_pending.eq(0), pm_block.eq(1)),
                If(sel_lxu,   lxu_pending.eq(0)),
                If(sel_lup,   lup_pending.eq(0)),
            ),
            If(t_lup.done & ~lup_pending, lup_pending.eq(1)), # done stays set 1 cycle after.


            # Local Rx Header Buffer credits (LCRD_x/LCRD1_x/LCRD2_x on each freed Rx Header Buffer).
            lcrd_pending.eq(lcrd_pending + (rx_pop & ~rx_pop_t2) - (tx_sent_lc & sel_lcrd)),
            lcrd2_pending.eq(lcrd2_pending + rx_pop_t2 - (tx_sent_lc & sel_lcrd2)),

            # LMPs received.
            If(lmp_pop,
                If(lmp_sub == LMP_SUB_PORT_CFG,      portcfg_pending.eq(1)),
                If(lmp_sub == LMP_SUB_SET_LINK_FUNC, force_pm_accept.eq(rx_buffer.source.dw0[10])),
                If(lmp_sub == LMP_SUB_U2_INACT,      u2_timeout.eq(rx_buffer.source.dw0[9:17])),
            ),

            # Link Power Management.
            If(phy.tx_lau, lau_pending.eq(1)),
            If(phy.tx_lxu | (lc_lgo & ~lgo_fwd), lxu_pending.eq(1)),

            # Header Packets to send.
            If(tx_push,
                tx_dws[tx_seq[0:ib]].eq(tx_push_dws),
                tx_dpps[tx_seq[0:ib]].eq(tx_push_dpp),
                tx_sent[tx_seq[0:ib]].eq(0),
                tx_t2[tx_seq[0:ib]].eq(is_type2(tx_push_dws[0:32], tx_push_dws[32:64])),
                tx_seq.eq((tx_seq + 1) & seq_mask),
                If(lmp_push,
                    If(portcap_pending,
                        portcap_pending.eq(0)
                    ).Else(
                        portcfg_pending.eq(0)
                    )
                )
            ),
            If(tx_sent_hp,
                tx_sent[send_seq[0:ib]].eq(1),
                send_seq.eq((send_seq + 1) & seq_mask),
            ),

            # Remote Rx Header Buffer credits (Type 1/Type 2).
            remote_credits.eq(remote_credits +
                (lc_lcrd & lcrd_ok & ~lc_info[2] & (remote_credits != LINK_HP_BUFFERS)) -
                (tx_sent_hp & ~send_t2)),
            remote_credits2.eq(remote_credits2 +
                (lc_lcrd & lcrd_ok & lc_info[2] & (remote_credits2 != LINK_HP_BUFFERS)) -
                (tx_sent_hp & send_t2)),
            If(lc_lcrd & lcrd_ok & ~lc_info[2], remote_lcrd_idx.eq(remote_lcrd_idx + 1)),
            If(lc_lcrd & lcrd_ok &  lc_info[2], remote_lcrd2_idx.eq(remote_lcrd2_idx + 1)),

            # LGOOD_n received: Advertisement or acknowledge.
            If(lc_lgood,
                If(~adv_received,
                    adv_received.eq(1),
                    ack_seq.eq((lc_info[0:sw] + 1) & seq_mask),
                    send_seq.eq((lc_info[0:sw] + 1) & seq_mask),
                ).Elif(lgood_ok,
                    ack_seq.eq((ack_seq + 1) & seq_mask),
                )
            ),
            # LBAD received: LRTY then retransmission of the unacknowledged Header Packets.
            If(lc_lbad,
                lrty_pending.eq(1),
                send_seq.eq(ack_seq),
                remote_credits.eq(remote_credits + lbad_t1 - (tx_sent_hp & ~send_t2)),
                remote_credits2.eq(remote_credits2 + lbad_t2 - (tx_sent_hp & send_t2)),
                If(self.tx_retry != (2**16 - 1), self.tx_retry.eq(self.tx_retry + 1)),
            ),

            # U0 entry: Header Sequence Number/Rx Header Buffer Credit Advertisements (7.2.4.1.1).
            If(u0_rise,
                # Advertisement: last received Header Sequence Number (reset value when the link
                # reset coincides with the U0 entry, ex: short Polling.Idle at Gen 2).
                lgood_seq.eq(Mux(self.reset, seq_mask, (rx_seq - 1) & seq_mask)),
                lcrd_pending.eq(Mux(self.reset, LINK_HP_BUFFERS,
                    LINK_HP_BUFFERS - (rx_buffer.level - rx_t2_level))),
                lcrd_idx.eq(0),
                lcrd2_pending.eq(Mux(ssp, Mux(self.reset, LINK_HP_BUFFERS,
                    LINK_HP_BUFFERS - rx_t2_level), 0)),
                lcrd2_idx.eq(0),
                remote_credits.eq(0),
                remote_lcrd_idx.eq(0),
                remote_credits2.eq(0),
                remote_lcrd2_idx.eq(0),
                adv_received.eq(0),
                lbad_wait.eq(0),
                lbad_pending.eq(0),
                lrty_pending.eq(0),
                lau_pending.eq(0),
                lxu_pending.eq(0),
                lup_pending.eq(0),
                pm_block.eq(0),
            ),

            # Link reset (Polling/Hot Reset): Header Sequence Numbers to zero, Tx Header Buffers
            # flushed.
            If(self.reset,
                rx_seq.eq(0),
                rx_t2_level.eq(0),
                tx_seq.eq(0),
                ack_seq.eq(0),
                send_seq.eq(0),
                portcap_pending.eq(1),
                portcfg_pending.eq(0),
                force_pm_accept.eq(0),
                u2_timeout.eq(0),
            ),
        ]
