#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from migen.genlib.cdc import BusSynchronizer

from litex.soc.interconnect.csr import *
from litex.soc.cores.freqmeter import FreqMeter

from usb3_pipe.common import K

# USB3 PHY Debug -----------------------------------------------------------------------------------

USB3PHY_DEBUG_HISTORY_DEPTH = 32

class USB3PHYDebug(LiteXModule):
    """USB3 PHY Debug (CSRs, hardware bring-up).

    Lightweight always-on observability of a USB3PHY, to be polled from the host (UARTBone/JTAGBone):
    - LTSSM state, SerDes ready.
    - LTSSM state history (last `USB3PHY_DEBUG_HISTORY_DEPTH` transitions, state + us timestamp).
    - Event counters (TS/LFPS detections, RX errors/idle, K28.4 substitutions, U0/Recovery entries).
    - PHY reset (re-runs link training from Polling) and counters/history clear.
    """
    def __init__(self, phy, serdes, sys_clk_freq):
        ltssm = phy.ltssm
        pipe  = phy.pipe

        self._phy_reset = CSRStorage(description="PHY (PIPE + LTSSM) reset (level).")
        self._clear     = CSR() # Clear counters and history (write).
        self._status    = CSRStatus(11, description="Status: LTSSM state [7:0], SerDes ready [8], "
                                                    "RX idle [9], RX polarity [10].")
        self._timestamp = CSRStatus(32, description="Timestamp (us).")

        # # #

        clear = Signal()
        self.comb += [
            clear.eq(self._clear.re),
            phy.reset.eq(self._phy_reset.storage),
            self._status.status.eq(Cat(phy.ltssm_state, Constant(0, 8 - len(phy.ltssm_state)),
                serdes.ready, serdes.rx_idle, serdes.rx_polarity)),
        ]

        # Timestamp (us).
        timestamp = self._timestamp.status
        prescaler = Signal(max=int(sys_clk_freq/1e6))
        self.sync += [
            prescaler.eq(prescaler + 1),
            If(prescaler == (int(sys_clk_freq/1e6) - 1),
                prescaler.eq(0),
                timestamp.eq(timestamp + 1),
            )
        ]

        # LTSSM State History.
        depth = USB3PHY_DEBUG_HISTORY_DEPTH
        self._history_count = CSRStatus(32, description="Number of LTSSM state transitions.")
        self._history_index = CSRStorage(log2_int(depth), description="History entry to read (0 = oldest).")
        self._history_state = CSRStatus(8,  description="History entry state.")
        self._history_time  = CSRStatus(32, description="History entry timestamp (us).")
        history_count = self._history_count.status
        history       = Memory(8 + 32, depth)
        history_wr    = history.get_port(write_capable=True)
        history_rd    = history.get_port(async_read=True)
        self.specials += history, history_wr, history_rd
        state   = Signal(8)
        state_d = Signal(8)
        self.comb += state.eq(phy.ltssm_state)
        self.sync += state_d.eq(state)
        self.comb += [
            history_wr.adr.eq(history_count),
            history_wr.dat_w.eq(Cat(state, timestamp)),
            history_wr.we.eq(state != state_d),
            # Read relative to the oldest entry still in memory.
            If(history_count > depth,
                history_rd.adr.eq(history_count + self._history_index.storage)
            ).Else(
                history_rd.adr.eq(self._history_index.storage)
            ),
            self._history_state.status.eq(history_rd.dat_r[:8]),
            self._history_time.status.eq(history_rd.dat_r[8:]),
        ]
        self.sync += If(clear,
            history_count.eq(0)
        ).Elif(history_wr.we,
            history_count.eq(history_count + 1)
        )

        # Event Counters.
        k28_4 = Signal()
        self.comb += k28_4.eq(serdes.source.valid & serdes.source.ready & Reduce("OR", [
            serdes.source.ctrl[i] & (serdes.source.data[8*i:8*(i+1)] == K(28, 4)) for i in range(4)]))
        events = [
            # Name,               Signal,                     Edge (1) or Level (0).
            ("ts1",               pipe.ts_rx_ts1,             1),
            ("ts1_inv",           pipe.ts_rx_ts1_inv,         1),
            ("ts2",               pipe.ts_rx_ts2,             1),
            ("lfps_polling",      pipe.lfps_rx_polling,       1),
            ("lfps_ping",         pipe.lfps_rx_ping,          1),
            ("lfps_exit_u1",      pipe.lfps_rx_exit_u1,       1),
            ("lfps_exit_u2",      pipe.lfps_rx_exit_u2,       1),
            ("lfps_wakeup_u3",    pipe.lfps_rx_wakeup_u3,     1),
            ("lfps_reset",        pipe.lfps_rx_reset,         1),
            ("rx_idle",           serdes.rx_idle,             1),
            ("rx_errors_cycles",  serdes.rx_errors,           0),
            ("k28_4",             k28_4,                      0),
            ("u0",                ltssm.u0,                   1),
            ("recovery",          ltssm.recovery,             1),
        ]
        for name, signal, edge in events:
            csr = CSRStatus(32, name=f"{name}_count")
            setattr(self, f"_{name}_count", csr)
            signal_d = Signal()
            self.sync += signal_d.eq(signal)
            event = (signal & ~signal_d) if edge else signal
            self.sync += If(clear,
                csr.status.eq(0)
            ).Elif(event & (csr.status != (2**32 - 1)),
                csr.status.eq(csr.status + 1)
            )

# USB3 RX Errors Debug -----------------------------------------------------------------------------

class USB3RXErrorsDebug(LiteXModule):
    """USB3 RX Errors Debug (CSRs, hardware bring-up).

    Counts, in the RX clock domain, the received symbols and the 8b/10b errors per type:
    - decoder_invalid: LiteX Decoder invalid flags.
    - code_invalid:    RX8b10bChecker invalid codes.
    - disparity:       RX8b10bChecker disparity errors.
    - bursts:          Error bursts (error words following an error-free word).
    - max_good_run:    Longest error-free run (words).
    Counters are synchronized to the sys clock domain (BusSynchronizer) and cleared with `clear`.
    """
    def __init__(self, decoders, checker, clock_domain="rx"):
        self._clear = CSR()

        # # #

        n = len(decoders)

        # Clear (sys -> rx).
        clear = Signal()
        clear_toggle   = Signal()
        clear_toggle_r = Signal(3)
        self.sync += If(self._clear.re, clear_toggle.eq(~clear_toggle))
        sync_rx = getattr(self.sync, clock_domain)
        sync_rx += clear_toggle_r.eq(Cat(clear_toggle, clear_toggle_r[:2]))
        self.comb += clear.eq(clear_toggle_r[1] ^ clear_toggle_r[2])

        # Counters (rx).
        events = {
            "symbols"         : [Constant(1, 1)]*n,
            "decoder_invalid" : [decoder.invalid for decoder in decoders],
            "code_invalid"    : [checker.invalid[i] for i in range(n)],
            "disparity"       : [checker.disparity_error[i] for i in range(n)],
        }
        # Error bursts / longest error-free run.
        error   = Signal()
        error_d = Signal()
        run     = Signal(32)
        bursts  = Signal(32)
        max_run = Signal(32)
        sync_rx += [
            error.eq(Reduce("OR", events["decoder_invalid"] + events["code_invalid"] + events["disparity"])),
            error_d.eq(error),
            If(clear,
                run.eq(0),
                bursts.eq(0),
                max_run.eq(0),
            ).Elif(error,
                run.eq(0),
                If(~error_d, bursts.eq(bursts + 1)),
            ).Else(
                run.eq(run + 1),
                If(run > max_run, max_run.eq(run)),
            )
        ]

        for name, flags in events.items():
            count     = Signal(32)
            increment = Signal(max=n + 1)
            sync_rx += increment.eq(Reduce("ADD", flags))
            sync_rx += If(clear,
                count.eq(0)
            ).Else(
                count.eq(count + increment)
            )
            csr  = CSRStatus(32, name=f"{name}_count")
            sync = BusSynchronizer(32, clock_domain, "sys")
            self.submodules += sync
            setattr(self, f"_{name}_count", csr)
            self.comb += [
                sync.i.eq(count),
                csr.status.eq(sync.o),
            ]
        for name, value in [("bursts", bursts), ("max_good_run", max_run)]:
            csr  = CSRStatus(32, name=f"{name}_count" if name == "bursts" else name)
            sync = BusSynchronizer(32, clock_domain, "sys")
            self.submodules += sync
            setattr(self, f"_{csr.name}", csr)
            self.comb += [
                sync.i.eq(value),
                csr.status.eq(sync.o),
            ]

# USB3 SerDes Debug --------------------------------------------------------------------------------

class USB3SerDesDebug(LiteXModule):
    """USB3 SerDes Debug (CSRs, hardware bring-up of A7USB3SerDes/K7USB3SerDes).

    - TX polarity / TX inhibit controls.
    - Transceiver loopback control (0b010: Near-End PMA).
    - TX/RX clocks frequency meters (100ms period): the RX clock is the recovered clock, its offset
      vs the TX clock shows the CDR frequency tracking of the partner (SSC: 0/-5000ppm).
    - RX 8b/10b errors statistics (USB3RXErrorsDebug).
    """
    def __init__(self, serdes, sys_clk_freq):
        self._tx_polarity = CSRStorage(description="TX polarity.")
        self._tx_inhibit  = CSRStorage(description="TX inhibit.")
        self._loopback    = CSRStorage(3, description="Transceiver loopback (0b010: Near-End PMA).")

        # # #

        gt = getattr(serdes, "gtp", None) or getattr(serdes, "gtx")
        self.comb += [
            serdes.tx_polarity.eq(self._tx_polarity.storage),
            gt.tx_inhibit.eq(self._tx_inhibit.storage),
            gt.loopback.eq(self._loopback.storage),
        ]

        # Clocks frequency meters.
        self.tx_freq = FreqMeter(period=int(sys_clk_freq/10), clk=ClockSignal("tx"))
        self.rx_freq = FreqMeter(period=int(sys_clk_freq/10), clk=ClockSignal("rx"))

        # RX errors.
        self.rx_errors = USB3RXErrorsDebug(gt.decoders, serdes.rx_substitution.checker)
