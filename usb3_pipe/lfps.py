#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Low Frequency Periodic Signaling (USB 3.2 6.9): Polling, Ping, U1/U2 exit, U3 wakeup and
Warm Reset LFPS generation/detection and SuperSpeedPlus LFPS signaling (6.9.4/6.9.5):

- SCDGenerator/SCDChecker   : Polling.LFPS binary representation (tRepeat 6-9us: 0, 11-14us: 1,
                              Table 6-32), SCD1 (0010b) / SCD2 (1101b) transmitted LSb first.
- LBPMGenerator/LBPMChecker : LFPS Based PWM Messages (tPWM 2.2us, 0: 1/3 LFPS, 1: 2/3 LFPS,
                              delimiter: tPWM LFPS + tPWM EI, LSb first, Table 6-33).
"""

# LFPS is the first signaling to happen during the initialization of the USB3.0 link.

# LFPS allows partners to exchange Out Of Band (OOB) controls/commands and consists of bursts where
# a "slow" clock is generated (between 10M-50MHz) for a specific duration and with a specific repeat
# period. After the burst, the transceiver is put in electrical idle mode (same electrical level on
# P/N pairs while in nominal mode P/N pairs always have an opposite level):
#
# Transceiver level/mode: _=0, -=1 x=electrical idle
# |-_-_-_-xxxxxxxxxxxxxxxxxxxx|-_-_-_-xxxxxxxxxxxxxxxxxxxx|...
# |<burst>                    |<burst>                    |...
# |<-----repeat period------->|<-----repeat period------->|...
#
# A LFPS pattern is identified by a burst duration and repeat period.
#
# To be able generate and receive LFPS, a transceiver needs to be able put its TX in electrical idle
# and to detect RX electrical idle.

from math import ceil

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.misc import WaitTimer

from litex.gen import *

# Constants/Helpers --------------------------------------------------------------------------------

lfps_clk_freq_min = 1/100e-9
lfps_clk_freq_max = 1/20e-9

def lfps_clk_half_period_cycles(sys_clk_freq, lfps_clk_freq):
    """Half period (in sys_clk cycles) of the generated LFPS clock. The generated period is rounded
    to an even number of sys_clk cycles: check it (not the requested frequency) against tPeriod
    (20-100ns, Table 6-29)."""
    half_period_cycles = ceil(sys_clk_freq/(2*lfps_clk_freq))
    period = 2*half_period_cycles/sys_clk_freq
    assert 1/lfps_clk_freq_max <= period <= 1/lfps_clk_freq_min, \
        "LFPS period {:.1f}ns (lfps_clk_freq={:.3f}MHz at sys_clk_freq={:.3f}MHz) outside of " \
        "20-100ns (Table 6-29).".format(period*1e9, lfps_clk_freq/1e6, sys_clk_freq/1e6)
    return half_period_cycles

class LFPSTiming:
    """LFPS timings with typical, minimum and maximum timing values."""
    def __init__(self, t_typ=None, t_min=None, t_max=None):
        assert t_min is not None
        assert t_max is not None
        self.t_typ = t_typ
        self.t_min = t_min
        self.t_max = t_max

class LFPS:
    """LFPS patterns with burst and repeat timings."""
    def __init__(self, burst, repeat=None, cycles=None):
        self.burst  = burst
        self.repeat = repeat
        self.cycles = cycles

def time_to_cycles(clk_freq, t):
    # Round before ceil to absorb floating point errors (ex: 80e-6*150e6 = 12000.000000000002).
    return ceil(round(t*clk_freq, 6))

def scaled_cycles(clk_freq, t, timer_scale=1, floor=1):
    """Convert a time to clock cycles, dividing by timer_scale (with a minimal floor).

    timer_scale uniformly scales all LFPS burst/repeat cycle counts (sim acceleration):
    relative order between timeouts is preserved; timer_scale=1 gives specification exact
    values. The floor (default 1, use 2 for bursts) keeps scaled bursts long enough to be
    seen through the 2-cycles idle deglitcher of the checkers."""
    return max(floor, time_to_cycles(clk_freq, t)//timer_scale)

# Single-burst handshake timing contract (USB 3.2 Rev 1.1 Table 6-31, section 6.9) -----------------
# Shared by the LFPS checkers below (detection min_cycles) and the LTSSM exit-handshake
# generator (burst length, ltssm.py) so both derive from the same source and scale the
# same way: at any timer_scale the initiator's burst must outlast the partner's detection
# latency (2-cycle MultiReg resync + 2-cycle idle deglitcher + min_cycles count) plus a
# sampling margin, so the responder still sees the initiator burst in flight when its own
# handshake starts (floor convention below, Table 6-31 order preserved).
RX_DETECT_LATENCY = 8 # sys cycles

def ping_burst_cycles(clk_freq, timer_scale=1):
    """Ping.LFPS burst length (typ, floor 2): also the Ping classification max_cycles."""
    return scaled_cycles(clk_freq, PingLFPSBurst.t_typ, timer_scale, floor=2)

def exit_u1_min_cycles(clk_freq, timer_scale=1):
    """U1 Exit detection window: above Ping max (length classification) and above the 300ns
    guard band of Table 6-30 note 7."""
    return max(ping_burst_cycles(clk_freq, timer_scale) + 1,
               scaled_cycles(clk_freq, U1ExitLFPSBurst.t_min, timer_scale))

def exit_u2_min_cycles(clk_freq, timer_scale=1):
    """U2/Loopback Exit detection window (80us min, Table 6-30)."""
    return scaled_cycles(clk_freq, U2ExitLFPSBurst.t_min, timer_scale)

def exit_u3_min_cycles(clk_freq, timer_scale=1):
    """U3 Wakeup detection window (80us min, Table 6-30)."""
    return scaled_cycles(clk_freq, U3WakeupLFPSBurst.t_min, timer_scale)

def exit_burst_cycles(clk_freq, timer_scale, burst_timing, detect_min_cycles):
    """Single-burst handshake burst length (Table 6-31): scaled typ burst, but never shorter
    than the partner detection window plus the RX detection latency, keeping the
    responder-sees-initiator relation true at any timer_scale."""
    return max(scaled_cycles(clk_freq, burst_timing.t_typ, timer_scale),
               detect_min_cycles + RX_DETECT_LATENCY)

def polling_scale_of(timer_scale):
    """Polling pattern softening rule shared with LFPSUnit: the Polling burst/repeat use a 10x
    softer scale so they stay above the idle deglitcher floor at high timer_scale."""
    return max(1, timer_scale//10)

# LFPS Patterns ------------------------------------------------------------------------------------
# All timings from USB 3.2 Rev 1.1, Table 6-30 (LFPS Transmitter Timing for SuperSpeed Designs)
# and Table 6-31 (LFPS Handshake Timing), section 6.9 (PDF pages 130-134).

PollingLFPSBurst  = LFPSTiming(t_typ=1.0e-6,  t_min=0.6e-6, t_max=1.4e-6)
PollingLFPSRepeat = LFPSTiming(t_typ=10.0e-6, t_min=6.0e-6, t_max=14.0e-6)
PollingLFPS       = LFPS(burst=PollingLFPSBurst, repeat=PollingLFPSRepeat)

# Ping.LFPS: burst 40ns min / 200ns max (Gen 1x1), min 2 LFPS cycles, repeat 160/200/240ms.
# t_typ is slightly below t_max so that ceil() quantization at sys_clk_freq stays within range.
PingLFPSBurst     = LFPSTiming(t_typ=190e-9, t_min=40e-9,   t_max=200e-9)
PingLFPSRepeat    = LFPSTiming(t_typ=200e-3, t_min=160e-3,  t_max=240e-3)
PingLFPS          = LFPS(burst=PingLFPSBurst, repeat=PingLFPSRepeat)

# Warm Reset: single burst 80/100/120ms (tRepeat not applicable, Table 6-30 note 4).
ResetLFPSBurst    = LFPSTiming(t_typ=100.0e-3, t_min=80.0e-3,  t_max=120.0e-3)
ResetLFPS         = LFPS(burst=ResetLFPSBurst)

# Warm Reset detection delay (tResetDelay, 6.9.3): an upstream port shall declare the detection of
# Warm Reset within 18-50ms. Also above the longest exit/wakeup LFPS bursts (U3 Wakeup 10ms max,
# Table 6-30/6-31) so that they are not detected as Warm Reset.
ResetDetectDelay  = 25.0e-3

# U1 Exit: single burst, 900ns min (2ms max). Detection minimum is 300ns (Table 6-30 note 7:
# guard band for the handshake). Enforced by the LTSSM handshake, not by the checker alone.
U1ExitLFPSBurst   = LFPSTiming(t_typ=900e-9, t_min=300e-9,  t_max=2e-3)

# U2/Loopback Exit: single burst, 80us min (2ms max). Burst is held for ~100us (typ, Daisho
# LFPS_U2LBEXIT_NOM) so that the partner detecting at 80us still sees a valid burst.
U2ExitLFPSBurst   = LFPSTiming(t_typ=100e-6, t_min=80e-6,   t_max=2e-3)

# U3 Wakeup: single burst, 80us min (10ms max). Held ~1ms (typ).
U3WakeupLFPSBurst = LFPSTiming(t_typ=1e-3,   t_min=80e-6,   t_max=10e-3)

# LFPS Idle Deglitcher -----------------------------------------------------------------------------

class LFPSIdleDeglitcher(LiteXModule):
    """LFPS Idle Deglitcher

    Resynchronizes the RX electrical idle signal of the transceiver to sys_clk and deglitches it:
    the output only changes after 2 consecutive identical samples.
    """
    def __init__(self, idle):
        self.o = Signal(reset=1) # o

        # # #

        # Resynchronization.
        idle_resync = Signal()
        self.specials += MultiReg(idle, idle_resync)

        # Deglitch.
        idle_reg = Signal(2, reset=0b11)
        self.sync += [
            idle_reg.eq(Cat(idle_reg[1], idle_resync)),
            If(idle_reg == 0b00,
                self.o.eq(0)
            ).Elif(idle_reg == 0b11,
                self.o.eq(1)
            )
        ]

# LFPS Checker -------------------------------------------------------------------------------------

class LFPSChecker(LiteXModule):
    """LFPS Checker

    Generic LFPS checker.

    This module is able to detect a specific LFPS pattern by analyzing the RX electrical idle signal
    of the transceiver. Detection measures the burst and repeat (gap) durations of the RX bursts and
    reports a detection when they both match the pattern timings.

    The bursts are measured through the SerDes electrical idle filter, which stretches them (and
    shortens the gaps) by stretch_cycles: the windows are widened accordingly.
    """
    def __init__(self, lfps_pattern, sys_clk_freq, timer_scale=1, stretch_cycles=0):
        self.idle   = Signal() # i
        self.detect = Signal() # o

        # # #

        # Idle Resynchronization/Deglitch ----------------------------------------------------------
        self.idle_deglitcher = idle_deglitcher = LFPSIdleDeglitcher(self.idle)
        idle = idle_deglitcher.o

        # Polling LFPS Detection -------------------------------------------------------------------
        # Deterministic burst+gap cadence measurement (USB 3.2 Table 6-30): a Polling.LFPS pattern
        # is one burst within [t_min, t_max] followed by a gap within [repeat_min - burst_max,
        # repeat_max - burst_min]. Detection asserts on the first cycle of the second qualifying
        # burst (<= 2 periods, versus the original phase-drift lock that depended on reset
        # alignment and could miss the 1-cycle lock window forever). Single stray bursts (Ping,
        # Exit, Warm Reset) or continuous traffic never qualify (burst or gap out of window).
        burst  = lfps_pattern.burst
        repeat = lfps_pattern.repeat
        bmin = scaled_cycles(sys_clk_freq, burst.t_min, timer_scale, floor=2)
        bmax = scaled_cycles(sys_clk_freq, burst.t_max, timer_scale) + stretch_cycles
        gmin = scaled_cycles(sys_clk_freq, repeat.t_min - burst.t_max, timer_scale, floor=2)
        gmin = max(2, gmin - stretch_cycles)
        gmax = scaled_cycles(sys_clk_freq, repeat.t_max - burst.t_min, timer_scale)

        count    = Signal(32)
        burst_ok = Signal()
        st_wait  = 0 # waiting for a burst
        st_burst = 1 # measuring burst length
        st_gap   = 2 # measuring gap length
        state    = Signal(2)

        self.sync += [
            self.detect.eq(0),
            If(state == st_wait,
                If(~idle,
                    state.eq(st_burst),
                    count.eq(1),
                    burst_ok.eq(0),
                )
            ).Elif(state == st_burst,
                count.eq(count + 1),
                If(idle,
                    burst_ok.eq((count >= bmin) & (count <= bmax)),
                    state.eq(st_gap),
                    count.eq(1),
                )
            ).Else(
                count.eq(count + 1),
                If(~idle,
                    self.detect.eq(burst_ok & (count >= gmin) & (count <= gmax)),
                    state.eq(st_burst),
                    count.eq(1),
                )
            )
        ]

# LFPS Burst Checker -------------------------------------------------------------------------------

class LFPSBurstChecker(LiteXModule):
    """LFPS Burst Checker

    Single-burst LFPS pattern detection (USB 3.2 Table 6-30 note 4: Warm Reset, U1/U2/Loopback
    Exit and U3 Wakeup are all single burst LFPS signals; only Ping.LFPS has a burst+repeat
    structure and it is classified by burst length here).

    The checker measures the duration of the ongoing RX burst (electrical idle deasserted):
    - detect: level, asserted while a burst has reached min_cycles and is still ongoing
      (Warm Reset / U1-U2-Loopback Exit / U3 Wakeup detection).
    - ping:   1 cycle pulse when a burst ends with a duration in [ping_min_cycles, max_cycles]
      (Ping.LFPS classification: burst shorter than the U1 exit 300ns detection window).
    """
    def __init__(self, min_cycles=0, max_cycles=0, ping_min_cycles=1):
        self.idle   = Signal() # i (1 = electrical idle, no burst)
        self.detect = Signal() # o (level)
        self.ping   = Signal() # o (pulse)

        # # #

        # Idle Resynchronization/Deglitch ----------------------------------------------------------
        self.idle_deglitcher = idle_deglitcher = LFPSIdleDeglitcher(self.idle)
        idle = idle_deglitcher.o

        # Burst Duration Measurement ---------------------------------------------------------------
        count = Signal(32)
        self.sync += [
            self.ping.eq(0),
            If(~idle,
                count.eq(count + 1),
                If((min_cycles > 0) & ((count + 1) >= min_cycles),
                    self.detect.eq(1)
                )
            ).Else(
                If((max_cycles > 0) & (count >= ping_min_cycles) & (count <= max_cycles),
                    self.ping.eq(1)
                ),
                count.eq(0),
                self.detect.eq(0),
            )
        ]

# LFPS Generator -----------------------------------------------------------------------------------

class LFPSBurstGenerator(LiteXModule):
    """LFPS Burst Generator

    Generate a LFPS burst of configurable length on the TX lane. The LFPS clock is generated by
    sending an alternating ones/zeroes data pattern on the parallel interface of the transceiver.
    """
    def __init__(self, sys_clk_freq, lfps_clk_freq):
        # Control
        self.start  = Signal()   # i
        self.done   = Signal()   # o
        self.length = Signal(32) # i

        # Transceiver
        self.tx_idle    = Signal(reset=1) # o
        self.tx_pattern = Signal(20)      # o

        # # #

        # LFPS Burst Clock generation (period checked against tPeriod, Table 6-29) -----------------
        clk = Signal()
        clk_timer = WaitTimer(lfps_clk_half_period_cycles(sys_clk_freq, lfps_clk_freq) - 1)
        clk_timer = ResetInserter()(clk_timer)
        self.submodules += clk_timer
        self.comb += clk_timer.wait.eq(~clk_timer.done)
        self.sync += If(clk_timer.done, clk.eq(~clk))

        # LFPS Burst generation --------------------------------------------------------------------
        count = Signal.like(self.length)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            clk_timer.reset.eq(1),
            NextValue(count, self.length - 1),
            If(self.start,
                NextState("BURST")
            )
        )
        fsm.act("BURST",
            self.tx_idle.eq(0),
            self.tx_pattern.eq(Replicate(clk, 20)),
            NextValue(count, count - 1),
            If(count == 0,
                NextState("IDLE")
            )
        )

# LFPS Generator -----------------------------------------------------------------------------------

class LFPSGenerator(LiteXModule):
    """LFPS Generator

    Generate a specific LFPS pattern on the TX lane. This module handles LFPS clock generation, LFPS
    burst generation and repetition.
    """
    def __init__(self, lfps_pattern, sys_clk_freq, lfps_clk_freq, timer_scale=1):
        # Control
        self.generate = Signal()      # i
        self.count    = Signal(16)    # o

        # Transceiver
        self.tx_idle    = Signal()   # o
        self.tx_pattern = Signal(20) # o

        # # #

        # Build-time Quantization/Range Checks -----------------------------------------------------
        # (only meaningful for timer_scale=1, i.e. specification exact cycle counts)
        burst         = lfps_pattern.burst
        repeat        = lfps_pattern.repeat
        repeat_cycles = scaled_cycles(sys_clk_freq, repeat.t_typ, timer_scale)
        burst_cycles  = scaled_cycles(sys_clk_freq, burst.t_typ,  timer_scale, floor=2)
        if timer_scale == 1:
            assert (burst.t_min  <= (burst_cycles/sys_clk_freq)  <= burst.t_max)
            assert (repeat.t_min <= (repeat_cycles/sys_clk_freq) <= repeat.t_max)

        # Burst Generator --------------------------------------------------------------------------
        burst_generator = LFPSBurstGenerator(sys_clk_freq=sys_clk_freq, lfps_clk_freq=lfps_clk_freq)
        burst_generator = ResetInserter()(burst_generator)
        self.submodules += burst_generator

        # Burst Generation -------------------------------------------------------------------------
        burst_repeat_count = Signal(max=repeat_cycles + 1)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.tx_idle.eq(0),
            If(self.generate,
                self.tx_idle.eq(1),
                NextValue(burst_generator.start, 1),
                NextValue(burst_generator.length, burst_cycles),
                NextValue(burst_repeat_count,     repeat_cycles),
                NextState("RUN")
            ).Else(
                # Not requested: abort any ongoing burst (a new request starts a full burst).
                burst_generator.reset.eq(1),
                NextValue(burst_generator.start, 0),
                NextValue(self.count, 0)
            )
        )
        fsm.act("RUN",
            NextValue(burst_generator.start, 0),
            self.tx_idle.eq(burst_generator.tx_idle),
            self.tx_pattern.eq(burst_generator.tx_pattern),
            NextValue(burst_repeat_count, burst_repeat_count - 1),
            If(~self.generate,
                # Request released: abort the repetition (no runt burst on a quick re-request,
                # tBurst min, Table 6-30).
                NextValue(self.count, 0),
                NextState("IDLE")
            ).Elif(burst_repeat_count == 0,
                NextState("IDLE"),
                NextValue(self.count, self.count + 1)
            ),
        )

# LFPS Unit ----------------------------------------------------------------------------------------

class LFPSUnit(LiteXModule):
    """LFPS Unit

    Detect/generate the LFPS patterns required for a USB3 link with simple control/status signals.

    Patterns (USB 3.2 Table 6-30/6-31):
    - Polling.LFPS      : burst + repeat (training).
    - Ping.LFPS         : burst + repeat (U1 keep-alive), classified by burst length.
    - Warm Reset        : single burst 80-120ms (UFP must detect it in any state).
    - U1 Exit           : single burst (>=300ns detection window, 900ns min handshake burst).
    - U2/Loopback Exit  : single burst (80us min detection).
    - U3 Wakeup         : single burst (80us min detection).

    timer_scale uniformly divides all burst/repeat cycle counts (sim acceleration).
    The Polling pattern uses a 10x softer scale so its burst stays above the idle
    deglitcher floor at high scales; both link partners use the same scale so detection
    relations are preserved.
    """
    def __init__(self, serdes, sys_clk_freq, lfps_clk_freq=25e6, timer_scale=1):
        # Control ----------------------------------------------------------------------------------
        self.tx_idle      = Signal()   # i
        self.tx_polling   = Signal()   # i
        self.tx_ping      = Signal()   # i
        self.tx_exit_u1   = Signal()   # i
        self.tx_exit_u2   = Signal()   # i
        self.tx_wakeup_u3 = Signal()   # i
        self.tx_reset     = Signal()   # i

        # Status -----------------------------------------------------------------------------------
        self.rx_polling   = Signal()   # o
        self.rx_ping      = Signal()   # o (pulse)
        self.rx_exit_u1   = Signal()   # o (level, burst ongoing)
        self.rx_exit_u2   = Signal()   # o (level)
        self.rx_wakeup_u3 = Signal()   # o (level)
        self.rx_reset     = Signal()   # o (level)
        self.tx_count     = Signal(16) # o

        # # #

        # LFPS Clock (free running, unscaled: LFPS bitrate is not affected by timer_scale) ---------
        lfps_clk       = Signal()
        lfps_clk_timer = WaitTimer(lfps_clk_half_period_cycles(sys_clk_freq, lfps_clk_freq) - 1)
        self.submodules += lfps_clk_timer
        self.comb += lfps_clk_timer.wait.eq(~lfps_clk_timer.done) # Free-running.
        self.sync += If(lfps_clk_timer.done, lfps_clk.eq(~lfps_clk))
        lfps_pattern = Replicate(lfps_clk, 20)

        # Polling (burst + repeat, softened scale to survive the idle deglitcher) ------------------
        # Bursts measured through the SerDes electrical idle filter (stretched by rx_idle_stretch).
        rx_idle_stretch = getattr(serdes, "rx_idle_stretch", 0)
        polling_scale   = polling_scale_of(timer_scale)
        self.polling_checker = polling_checker = LFPSChecker(PollingLFPS,
            sys_clk_freq   = sys_clk_freq,
            timer_scale    = polling_scale,
            stretch_cycles = rx_idle_stretch)
        self.comb += polling_checker.idle.eq(serdes.rx_idle)
        self.comb += self.rx_polling.eq(polling_checker.detect)

        self.polling_generator = polling_generator = LFPSGenerator(PollingLFPS,
            sys_clk_freq  = sys_clk_freq,
            lfps_clk_freq = lfps_clk_freq,
            timer_scale   = polling_scale)
        self.comb += self.tx_count.eq(polling_generator.count)

        # RX burst classification (Table 6-30, 6.9.2 implementation note) --------------------------
        # Bursts are measured through the SerDes electrical idle filter, which stretches them by
        # rx_idle_stretch cycles. A burst ending before the 300ns U1 exit detection is a Ping.LFPS
        # (min tBurst 40ns, shorter glitches are rejected); a burst reaching 300ns is a U1 exit.
        u1_exit_min_cycles = exit_u1_min_cycles(sys_clk_freq, timer_scale) + rx_idle_stretch
        ping_min_cycles    = scaled_cycles(sys_clk_freq, PingLFPSBurst.t_min, timer_scale)
        ping_min_cycles    = max(1, ping_min_cycles + rx_idle_stretch)
        ping_max_cycles    = u1_exit_min_cycles - 1

        # Ping (burst + repeat generator, burst-length classified checker) -------------------------
        self.ping_checker = ping_checker = LFPSBurstChecker(
            max_cycles      = ping_max_cycles,
            ping_min_cycles = ping_min_cycles)
        self.comb += [
            ping_checker.idle.eq(serdes.rx_idle),
            self.rx_ping.eq(ping_checker.ping),
        ]
        self.ping_generator = ping_generator = LFPSGenerator(PingLFPS,
            sys_clk_freq  = sys_clk_freq,
            lfps_clk_freq = lfps_clk_freq,
            timer_scale   = timer_scale)

        # U1 Exit (detection window: above Ping max, 300ns min per Table 6-30 note 7) --------------
        self.exit_u1_checker = exit_u1_checker = LFPSBurstChecker(min_cycles=u1_exit_min_cycles)
        self.comb += [
            exit_u1_checker.idle.eq(serdes.rx_idle),
            self.rx_exit_u1.eq(exit_u1_checker.detect),
        ]

        # U2/Loopback Exit -------------------------------------------------------------------------
        self.exit_u2_checker = exit_u2_checker = LFPSBurstChecker(
            min_cycles=exit_u2_min_cycles(sys_clk_freq, timer_scale))
        self.comb += [
            exit_u2_checker.idle.eq(serdes.rx_idle),
            self.rx_exit_u2.eq(exit_u2_checker.detect),
        ]

        # U3 Wakeup --------------------------------------------------------------------------------
        self.wakeup_u3_checker = wakeup_u3_checker = LFPSBurstChecker(
            min_cycles=exit_u3_min_cycles(sys_clk_freq, timer_scale))
        self.comb += [
            wakeup_u3_checker.idle.eq(serdes.rx_idle),
            self.rx_wakeup_u3.eq(wakeup_u3_checker.detect),
        ]

        # Warm Reset -------------------------------------------------------------------------------
        # A non electrical idle line is also seen with SuperSpeed data (U0, Recovery...): the LFPS
        # is differentiated from data by the RX decode errors it produces (see RXErrorMonitor).
        # SerDes without rx_errors fall back to the electrical idle detection only.
        rx_errors = getattr(serdes, "rx_errors", None)
        self.reset_checker = reset_checker = LFPSBurstChecker(
            min_cycles=scaled_cycles(sys_clk_freq, ResetDetectDelay, timer_scale))
        self.comb += [
            reset_checker.idle.eq(serdes.rx_idle | ((~rx_errors) if rx_errors is not None else 0)),
            self.rx_reset.eq(reset_checker.detect),
        ]

        # Burst+repeat generators arming (single-burst patterns are driven raw by the LTSSM) -------
        self.comb += [
            polling_generator.generate.eq(self.tx_polling),
            ping_generator.generate.eq(self.tx_ping),
        ]

        # TX mux (priority: Warm Reset/U3 Wakeup/U2-Loopback Exit/U1 Exit > Ping > Polling) --------
        # LFPS is a square wave (6.9.1, Table 6-29): during a burst, the raw tx_pattern (all ones /
        # all zeros 20-bit words) is transmitted instead of the 8b/10b encoded data (tx_lfps).
        # Single-burst patterns (driven raw by the LTSSM) all send the free running LFPS clock.
        tx_lfps = Signal()
        self.comb += [
            If(self.tx_reset | self.tx_wakeup_u3 | self.tx_exit_u2 | self.tx_exit_u1,
                serdes.tx_idle.eq(0),
                tx_lfps.eq(1),
                serdes.tx_pattern.eq(lfps_pattern),
            ).Elif(self.tx_ping,
                serdes.tx_idle.eq(ping_generator.tx_idle),
                tx_lfps.eq(~ping_generator.tx_idle),
                serdes.tx_pattern.eq(ping_generator.tx_pattern),
            ).Elif(self.tx_polling,
                serdes.tx_idle.eq(polling_generator.tx_idle),
                tx_lfps.eq(~polling_generator.tx_idle),
                serdes.tx_pattern.eq(polling_generator.tx_pattern),
            ).Else(
                serdes.tx_idle.eq(self.tx_idle)
            )
        ]
        # SerDes without tx_lfps: raw pattern only sent when non-zero (high half of the period).
        if hasattr(serdes, "tx_lfps"):
            self.comb += serdes.tx_lfps.eq(tx_lfps)

# SuperSpeedPlus Constants/Helpers (6.9.4/6.9.5, Table 7-13) ---------------------------------------

# SuperSpeedPlus LFPS signaling is on the Configuration Lane (6.13.12). Timings are divided by
# timer_scale (sim acceleration, both partners must use the same scale).

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

def rounded_cycles(clk_freq, t, timer_scale=1, floor=2):
    """Convert a time to clock cycles divided by timer_scale, rounded (with a minimal floor): used
    by the SuperSpeedPlus LFPS signaling (scaled_cycles rounds up before the timer_scale division).
    """
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

        burst    = rounded_cycles(sys_clk_freq,  1.0e-6, timer_scale)
        repeat_0 = rounded_cycles(sys_clk_freq,  7.5e-6, timer_scale)
        repeat_1 = rounded_cycles(sys_clk_freq, 12.5e-6, timer_scale)
        repeat_n = rounded_cycles(sys_clk_freq, 10.0e-6, timer_scale)

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

        c = lambda t: rounded_cycles(sys_clk_freq, t, timer_scale)
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

        t_pwm = rounded_cycles(sys_clk_freq, 2.2e-6, timer_scale, floor=6)
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

        t_pwm = rounded_cycles(sys_clk_freq, 2.2e-6, timer_scale, floor=6)
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
