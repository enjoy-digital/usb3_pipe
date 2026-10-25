#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""SuperSpeedPlus Polling (USB 3.2 6.9.4/6.9.5, 7.5.4.3-7.5.4.6), see doc/gen2-x2-roadmap.md.

- SCDGenerator/SCDChecker   : Polling.LFPS binary representation (tRepeat 6-9us: 0, 11-14us: 1,
                              Table 6-32), SCD1 (0010b) / SCD2 (1101b) transmitted LSb first.
- LBPMGenerator/LBPMChecker : LFPS Based PWM Messages (tPWM 2.2us, 0: 1/3 LFPS, 1: 2/3 LFPS, delimiter:
                              tPWM LFPS + tPWM EI, LSb first, Table 6-33).
- SSPPolling                : Polling.LFPS (SCD1 handshake) -> Polling.LFPSPlus (SCD2 handshake) ->
                              Polling.PortMatch (PHY Capability LBPM, Table 7-13/7-14) ->
                              Polling.PortConfig (PHY Ready LBPM handshake) -> done (Polling.RxEQ),
                              negotiating Gen 1x1/Gen 1x2. A partner without SCD (SuperSpeed) is
                              reported (ss_fallback) for the LTSSM to complete the SuperSpeed
                              Polling.LFPS handshake.

All LFPS signaling is on the Configuration Lane (6.13.12). Timings are divided by timer_scale (sim
acceleration, both partners must use the same scale).
"""

from migen import *

from litex.gen import *

from usb3_pipe.lfps import lfps_clk_half_period_cycles, LFPSIdleDeglitcher

# Constants ----------------------------------------------------------------------------------------

SCD1 = 0b0010 # Transmitted LSb first: 0, 1, 0, 0.
SCD2 = 0b1101 # Transmitted LSb first: 1, 0, 1, 1.

def scd_bits(scd):
    """Transmission order (LSb first) of a SCD."""
    return [(scd >> i) & 0x1 for i in range(4)]

# PHY LBPM (Table 7-13).
LBPM_PHY_CAPABILITY = 0b00
LBPM_PHY_READY      = 0b01

def lbpm_phy_capability(gen2=False, x2=False):
    return LBPM_PHY_CAPABILITY | ((1 if gen2 else 0) << 2) | ((1 if x2 else 0) << 5)

def lbpm_phy_ready(dfp=False, x2=False, retimers=0, rt_config=False):
    return (LBPM_PHY_READY | (retimers << 2) |
        ((1 if (x2 and dfp) else 0) << 6) | ((1 if rt_config else 0) << 7))

def cycles(clk_freq, t, timer_scale=1, floor=2):
    return max(floor, int(round(t*clk_freq/timer_scale)))

# LFPS RX Front-End --------------------------------------------------------------------------------

class LFPSRXFrontEnd(LiteXModule):
    """Electrical idle resynchronization/deglitch (as the LFPS checkers) and burst/gap measurement:
    burst_end pulses with burst (length), burst_start pulses with period (cycles since the previous
    burst start), gap is the current electrical idle length."""
    def __init__(self):
        self.idle        = Signal()   # i
        self.burst_start = Signal()   # o
        self.burst_end   = Signal()   # o
        self.burst       = Signal(32) # o
        self.period      = Signal(32) # o
        self.gap         = Signal(32) # o
        self.active      = Signal()   # o: Burst ongoing.

        # # #

        self.idle_deglitcher = idle_deglitcher = LFPSIdleDeglitcher(self.idle)
        idle = idle_deglitcher.o
        idle_d       = Signal(reset=1)
        burst_count  = Signal(32)
        period_count = Signal(32)
        self.sync += [
            idle_d.eq(idle),
            If(period_count != 2**32 - 1, period_count.eq(period_count + 1)),
            If(idle, If(self.gap != 2**32 - 1, self.gap.eq(self.gap + 1))).Else(self.gap.eq(0)),
            If(~idle, burst_count.eq(burst_count + 1)),
            If(idle_d & ~idle,
                self.period.eq(period_count),
                period_count.eq(1),
                burst_count.eq(1),
            ),
            If(~idle_d & idle,
                self.burst.eq(burst_count),
            ),
        ]
        self.comb += [
            self.active.eq(~idle),
            self.burst_start.eq(idle_d & ~idle),
            self.burst_end.eq(~idle_d & idle),
        ]

# SCD Generator ------------------------------------------------------------------------------------

class SCDGenerator(LiteXModule):
    """Polling.LFPS bursts (tBurst 1us) with tRepeat encoding the bits of SCD1/SCD2 (7.5us: 0, 12.5us:
    1) or non varying tRepeat (10us, mode 0). count increments after each complete SCD (4 bursts,
    mode 1/2) or each burst (mode 0)."""
    def __init__(self, sys_clk_freq, timer_scale=1):
        self.mode    = Signal(2)  # i: 0: Polling.LFPS, 1: SCD1, 2: SCD2.
        self.enable  = Signal()   # i
        self.tx_lfps = Signal()   # o: LFPS burst (else electrical idle).
        self.count   = Signal(16) # o

        # # #

        burst    = cycles(sys_clk_freq,  1.0e-6, timer_scale)
        repeat_0 = cycles(sys_clk_freq,  7.5e-6, timer_scale)
        repeat_1 = cycles(sys_clk_freq, 12.5e-6, timer_scale)
        repeat_n = cycles(sys_clk_freq, 10.0e-6, timer_scale)

        bit_index = Signal(2)
        timer     = Signal(32)
        bit       = Signal()
        repeat    = Signal(32)
        self.comb += [
            Case(self.mode, {
                1: bit.eq(Array(scd_bits(SCD1))[bit_index]),
                2: bit.eq(Array(scd_bits(SCD2))[bit_index]),
                "default": bit.eq(0),
            }),
            If(self.mode == 0,
                repeat.eq(repeat_n)
            ).Elif(bit,
                repeat.eq(repeat_1)
            ).Else(
                repeat.eq(repeat_0)
            ),
            self.tx_lfps.eq(self.enable & (timer < burst)),
        ]
        self.sync += [
            If(~self.enable,
                timer.eq(0),
                bit_index.eq(0),
                self.count.eq(0),
            ).Elif(timer == (repeat - 1),
                timer.eq(0),
                bit_index.eq(bit_index + 1),
                If((self.mode == 0) | (bit_index == 3),
                    self.count.eq(self.count + 1)
                )
            ).Else(
                timer.eq(timer + 1)
            )
        ]

# SCD Checker --------------------------------------------------------------------------------------

class SCDChecker(LiteXModule):
    """Polling.LFPS binary representation decoding (Table 6-32, burst start to burst start): scd1/scd2
    pulse when the last 4 received bits are a SCD1/SCD2, polling pulses on each Polling.LFPS burst
    (period 6-14us, valid burst 0.6-1.4us), plain pulses on bursts with a non-SCD period (guard band)."""
    def __init__(self, frontend, sys_clk_freq, timer_scale=1):
        self.scd1    = Signal() # o
        self.scd2    = Signal() # o
        self.polling = Signal() # o
        self.plain   = Signal() # o

        # # #

        c = lambda t: cycles(sys_clk_freq, t, timer_scale)
        bits  = Signal(4)
        valid = Signal(3)
        b0    = Signal()
        b1    = Signal()
        self.comb += [
            b0.eq((frontend.period >= c(6e-6))  & (frontend.period <= c(9e-6))),
            b1.eq((frontend.period >= c(11e-6)) & (frontend.period <= c(14e-6))),
        ]
        start_d = Signal()
        self.sync += [
            start_d.eq(frontend.burst_start),
            self.scd1.eq(0),
            self.scd2.eq(0),
            self.polling.eq(0),
            self.plain.eq(0),
            If(start_d,
                self.polling.eq((frontend.period >= c(6e-6)) & (frontend.period <= c(14e-6))),
                If(b0 | b1,
                    bits.eq(Cat(bits[1:], b1)),
                    If(valid != 4, valid.eq(valid + 1)),
                    If(valid >= 3,
                        # First received bit in bits[0] (LSb first).
                        self.scd1.eq(Cat(bits[1:], b1) == SCD1),
                        self.scd2.eq(Cat(bits[1:], b1) == SCD2),
                    )
                ).Else(
                    valid.eq(0),
                    self.plain.eq((frontend.period >= c(6e-6)) & (frontend.period <= c(14e-6))),
                )
            )
        ]

# LBPM Generator -----------------------------------------------------------------------------------

class LBPMGenerator(LiteXModule):
    """Continuous LBPM transmission of data (sampled at each LBPM start): delimiter, 8 bits (LSb
    first), delimiter between consecutive LBPMs (6.9.5.2). sent pulses at the end of each LBPM."""
    def __init__(self, sys_clk_freq, timer_scale=1):
        self.enable  = Signal()  # i
        self.data    = Signal(8) # i
        self.tx_lfps = Signal()  # o
        self.sent    = Signal()  # o
        self.current = Signal(8) # o: LBPM being sent.

        # # #

        t_pwm = cycles(sys_clk_freq, 2.2e-6, timer_scale, floor=6)
        t_0   = t_pwm//3
        t_1   = (2*t_pwm)//3

        timer = Signal(32)
        index = Signal(4) # 0: delimiter, 1-8: bits.
        phase = Signal()  # Delimiter: 0 LFPS tPWM, 1 EI tPWM.
        bit   = Signal()
        self.comb += bit.eq(Array([self.current[i] for i in range(8)] + [0]*8)[index - 1])
        self.comb += [
            If(~self.enable,
                self.tx_lfps.eq(0)
            ).Elif(index == 0,
                self.tx_lfps.eq(~phase)
            ).Else(
                self.tx_lfps.eq(timer < Mux(bit, t_1, t_0))
            )
        ]
        self.sync += [
            self.sent.eq(0),
            If(~self.enable,
                timer.eq(0),
                index.eq(0),
                phase.eq(0),
                self.current.eq(self.data),
            ).Elif(timer == (t_pwm - 1),
                timer.eq(0),
                If(index == 0,
                    phase.eq(~phase),
                    If(phase,
                        index.eq(1)
                    )
                ).Elif(index == 8,
                    index.eq(0),
                    self.sent.eq(1),
                    self.current.eq(self.data),
                ).Else(
                    index.eq(index + 1)
                )
            ).Else(
                timer.eq(timer + 1)
            )
        ]

# LBPM Checker -------------------------------------------------------------------------------------

class LBPMChecker(LiteXModule):
    """LBPM reception (receive windows of Table 6-33): bursts of 0.45-0.85us: 0, 1.28-1.85us: 1, a
    delimiter (>= 1.9us burst) starts/ends the LBPMs. received pulses with data after 8 bits followed
    by a delimiter."""
    def __init__(self, frontend, sys_clk_freq, timer_scale=1):
        self.received = Signal()  # o
        self.data     = Signal(8) # o

        # # #

        t_pwm = cycles(sys_clk_freq, 2.2e-6, timer_scale, floor=6)
        lo0   = max(1, int(t_pwm*0.45/2.2))
        hi0   = int(t_pwm*0.85/2.2) + 1
        lo1   = int(t_pwm*1.28/2.2)
        hi1   = int(t_pwm*1.85/2.2) + 1
        delim = int(t_pwm*1.9/2.2)

        shift   = Signal(8)
        count   = Signal(4)
        started = Signal()
        is0     = Signal()
        is1     = Signal()
        isd     = Signal()
        self.comb += [
            is0.eq((frontend.burst >= lo0) & (frontend.burst <= hi0)),
            is1.eq((frontend.burst >= lo1) & (frontend.burst <= hi1)),
            isd.eq(frontend.burst >= delim),
        ]
        end_d = Signal()
        self.sync += [
            end_d.eq(frontend.burst_end),
            self.received.eq(0),
            If(frontend.gap > 3*t_pwm,
                started.eq(0)
            ),
            If(end_d,
                If(isd,
                    If(started & (count == 8),
                        self.received.eq(1),
                        self.data.eq(shift),
                    ),
                    started.eq(1),
                    count.eq(0),
                ).Elif(started & (is0 | is1) & (count != 8),
                    shift.eq(Cat(shift[1:], is1)),
                    count.eq(count + 1),
                ).Else(
                    started.eq(0)
                )
            )
        ]

# SSP Polling --------------------------------------------------------------------------------------

class SSPPolling(LiteXModule):
    """SuperSpeedPlus Polling substates (7.5.4.3-7.5.4.6) on the Configuration Lane, Gen 1 speeds.

    start (level, LTSSM in Polling.LFPS) runs: Polling.LFPS (SCD1 transmitted, 2 SCD1 sent after a SCD1/
    SCD2 is received) -> Polling.LFPSPlus (SCD2, 2 SCD2 sent after a SCD2 is received, end of SCD) ->
    Polling.PortMatch (PHY Capability LBPMs, the port with the higher capability adjusts to the
    partner's, 4 matched LBPMs sent after 2 consecutive matched received) -> Polling.PortConfig (PHY
    Ready LBPMs, 4 sent after 2 consecutive identical received, the DFP PHY Ready bit 7 de-asserted)
    -> done (level until start is released), x2 = negotiated Gen 1x2 (Table 7-14).

    16 consecutive Polling.LFPS bursts received without SCD (SuperSpeed partner, 7.5.4.3.1) after
    4 SCD1 sent: ss_fallback (level until start is released), the LTSSM continues with the SuperSpeed
    Polling.LFPS handshake. tPollingLBPMLFPSTimeout (12ms) in PortMatch/PortConfig: fail.

    Not implemented: tPollingSCDLFPSTimeout, re-timers presence announcement/RT Config, Gen 2,
    PortMatch re-entry/capability fallback after a training failure (Table 7-15).
    """
    def __init__(self, sys_clk_freq, timer_scale=1, x2_capable=True):
        self.start       = Signal() # i
        self.ds_port     = Signal() # i
        self.x2_capable  = Signal(reset=int(x2_capable)) # i
        self.idle        = Signal() # i: RX electrical idle (Configuration Lane).
        self.tx_lfps     = Signal() # o: LFPS burst (else electrical idle).
        self.tx_active   = Signal() # o: SSP LFPS transmission (tx_lfps drives the transmitter).
        self.done        = Signal() # o
        self.x2          = Signal() # o
        self.ss_fallback = Signal() # o
        self.fail        = Signal() # o
        self.state       = Signal(4) # o: (debug).

        # # #

        self.frontend = frontend = LFPSRXFrontEnd()
        self.scd_gen  = scd_gen  = SCDGenerator(sys_clk_freq, timer_scale)
        self.scd_chk  = scd_chk  = SCDChecker(frontend, sys_clk_freq, timer_scale)
        self.lbpm_gen = lbpm_gen = LBPMGenerator(sys_clk_freq, timer_scale)
        self.lbpm_chk = lbpm_chk = LBPMChecker(frontend, sys_clk_freq, timer_scale)
        self.comb += frontend.idle.eq(self.idle)

        # Timeouts.
        lbpm_timeout = cycles(sys_clk_freq, 12e-3, timer_scale)
        scd_end      = cycles(sys_clk_freq, 1e-6, timer_scale) + 2*cycles(sys_clk_freq, 14e-6, timer_scale)
        timer = Signal(32)

        # Received LBPM tracking (consecutive identical/matched).
        rx_last      = Signal(8)
        rx_count     = Signal(2)
        tx_count     = Signal(3)
        scd_rx       = Signal()
        scd_sent_ref = Signal(16)
        plain_count  = Signal(5)
        own_cap      = Signal(8)
        tx_cap       = Signal(8)
        partner_x2   = Signal()
        self.comb += own_cap.eq(lbpm_phy_capability(x2=False) | (self.x2_capable << 5))

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(tx_cap,      own_cap),
            NextValue(scd_rx,      0),
            NextValue(plain_count, 0),
            NextValue(timer,       0),
            NextValue(self.x2,     0),
            If(self.start,
                NextState("POLLING-LFPS")
            )
        )
        fsm.act("POLLING-LFPS",
            self.tx_active.eq(1),
            scd_gen.enable.eq(1),
            scd_gen.mode.eq(1),
            self.tx_lfps.eq(scd_gen.tx_lfps),
            If((scd_chk.scd1 | scd_chk.scd2) & ~scd_rx,
                NextValue(scd_rx, 1),
                NextValue(scd_sent_ref, scd_gen.count),
            ),
            If(scd_chk.plain,
                If(plain_count != 16, NextValue(plain_count, plain_count + 1))
            ),
            If(scd_rx & (scd_gen.count >= (scd_sent_ref + 2)),
                NextValue(scd_rx, 0),
                NextState("POLLING-LFPSPLUS")
            ).Elif(~scd_rx & (plain_count >= 16) & (scd_gen.count >= 4),
                NextState("SS-FALLBACK")
            ),
            If(~self.start, NextState("IDLE"))
        )
        fsm.act("POLLING-LFPSPLUS",
            self.tx_active.eq(1),
            scd_gen.enable.eq(1),
            scd_gen.mode.eq(2),
            self.tx_lfps.eq(scd_gen.tx_lfps),
            If(scd_chk.scd2 & ~scd_rx,
                NextValue(scd_rx, 1),
                NextValue(scd_sent_ref, scd_gen.count),
            ),
            If(scd_rx & (scd_gen.count >= (scd_sent_ref + 2)),
                NextValue(timer, 0),
                NextState("SCD-END")
            ),
            If(~self.start, NextState("IDLE"))
        )
        fsm.act("SCD-END",
            # End of SCD (Figure 6-36 (c)/(d)): an extra tBurst followed by EI of 2x max tRepeat.
            self.tx_active.eq(1),
            self.tx_lfps.eq(timer < cycles(sys_clk_freq, 1e-6, timer_scale)),
            NextValue(timer, timer + 1),
            If(timer == scd_end,
                NextValue(timer, 0),
                NextValue(rx_count, 0),
                NextValue(tx_count, 0),
                NextState("POLLING-PORTMATCH")
            ),
            If(~self.start, NextState("IDLE"))
        )
        rx_matched = Signal()
        self.comb += rx_matched.eq(
            ((lbpm_chk.data[0:2] == LBPM_PHY_CAPABILITY) & (lbpm_chk.data == tx_cap)) |
            (lbpm_chk.data[0:2] == LBPM_PHY_READY))
        fsm.act("POLLING-PORTMATCH",
            self.tx_active.eq(1),
            lbpm_gen.enable.eq(1),
            self.tx_lfps.eq(lbpm_gen.tx_lfps),
            NextValue(timer, timer + 1),
            If(lbpm_chk.received,
                If(lbpm_chk.data[0:2] == LBPM_PHY_CAPABILITY,
                    # Highest common capability (Gen 1: x2 when both are x2 capable, Table 7-14).
                    NextValue(tx_cap, lbpm_chk.data & tx_cap),
                    NextValue(partner_x2, lbpm_chk.data[5]),
                ),
                If(rx_matched,
                    If(rx_count != 2, NextValue(rx_count, rx_count + 1))
                ).Else(
                    NextValue(rx_count, 0),
                    NextValue(tx_count, 0),
                )
            ),
            If(lbpm_gen.sent,
                If((lbpm_gen.current == tx_cap) & (rx_count == 2),
                    NextValue(tx_count, tx_count + 1)
                ).Else(
                    NextValue(tx_count, 0)
                )
            ),
            If(tx_count == 4,
                NextValue(self.x2, tx_cap[5]),
                NextValue(rx_count, 0),
                NextValue(tx_count, 0),
                NextState("POLLING-PORTCONFIG")
            ),
            If(timer == lbpm_timeout, NextState("FAIL")),
            If(~self.start, NextState("IDLE"))
        )
        tx_ready = Signal(8)
        self.comb += tx_ready.eq(LBPM_PHY_READY | ((self.x2 & self.ds_port) << 6))
        rx_ready_ok = Signal()
        self.comb += rx_ready_ok.eq(
            (lbpm_chk.data[0:2] == LBPM_PHY_READY) &
            (lbpm_chk.data == rx_last) &
            ~lbpm_chk.data[7]) # DFP Config Done (bit 7 de-asserted, x2) / reserved (UFP).
        fsm.act("POLLING-PORTCONFIG",
            self.tx_active.eq(1),
            lbpm_gen.enable.eq(1),
            self.tx_lfps.eq(lbpm_gen.tx_lfps),
            NextValue(timer, timer + 1),
            If(lbpm_chk.received,
                NextValue(rx_last, lbpm_chk.data),
                If(rx_ready_ok,
                    If(rx_count != 2, NextValue(rx_count, rx_count + 1))
                ).Elif(lbpm_chk.data[0:2] == LBPM_PHY_READY,
                    NextValue(rx_count, 1),
                    NextValue(tx_count, 0),
                ).Else(
                    NextValue(rx_count, 0),
                    NextValue(tx_count, 0),
                )
            ),
            If(lbpm_gen.sent & (lbpm_gen.current == tx_ready),
                If(rx_count == 2,
                    NextValue(tx_count, tx_count + 1)
                )
            ),
            If(tx_count == 4,
                NextState("DONE")
            ),
            If(timer == lbpm_timeout, NextState("FAIL")),
            If(~self.start, NextState("IDLE"))
        )
        self.comb += lbpm_gen.data.eq(Mux(fsm.ongoing("POLLING-PORTCONFIG"), tx_ready, tx_cap))
        fsm.act("DONE",
            self.done.eq(1),
            If(~self.start, NextState("IDLE"))
        )
        fsm.act("SS-FALLBACK",
            self.ss_fallback.eq(1),
            If(~self.start, NextState("IDLE"))
        )
        fsm.act("FAIL",
            self.fail.eq(1),
            If(~self.start, NextState("IDLE"))
        )

        # LFPS clock pattern (for transceivers sending the LFPS as a data pattern).
        self.tx_pattern = Signal(20) # o
        lfps_clk = Signal()
        lfps_div = Signal(8)
        half     = lfps_clk_half_period_cycles(sys_clk_freq, 25e6)
        self.sync += [
            lfps_div.eq(lfps_div + 1),
            If(lfps_div == (half - 1),
                lfps_div.eq(0),
                lfps_clk.eq(~lfps_clk)
            )
        ]
        self.comb += self.tx_pattern.eq(Replicate(lfps_clk, 20))
