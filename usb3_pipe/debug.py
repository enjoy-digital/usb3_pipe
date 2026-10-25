#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 PHY debug (doc/debugging-guide.md): LTSSM state history and counters (USB3PHYDebug), RX
errors (USB3RXErrorsDebug), SerDes controls and measurements (USB3SerDesDebug), LiteScope probes
groups (usb3_analyzer_groups).
"""

from functools import reduce
from operator import xor

from migen import *
from migen.genlib.cdc import BusSynchronizer, GrayCounter, MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.cores.freqmeter import FreqMeter

from liteiclink.serdes.common import DRPControl

from usb3_pipe.common import K

# Helpers ------------------------------------------------------------------------------------------

def gray_decode(gray):
    """Gray to binary decoding (combinatorial)."""
    n = len(gray)
    return Cat(*[reduce(xor, [gray[j] for j in range(i, n)]) for i in range(n)])

# USB3 PHY Debug -----------------------------------------------------------------------------------

USB3PHY_DEBUG_HISTORY_DEPTH = 32

class USB3PHYDebug(LiteXModule):
    """USB3 PHY Debug (CSRs, hardware bring-up).

    Lightweight always-on observability of a USB3PHY, polled from the host (UARTBone/JTAGBone):
    - LTSSM state, SerDes ready.
    - LTSSM state history (last `USB3PHY_DEBUG_HISTORY_DEPTH` transitions, or the first ones after a
      clear with history_freeze, state + us timestamp).
    - Event counters (TS/LFPS detections, RX errors/idle, K28.4 substitutions, U0/Recovery/U1/U2/U3
      entries).
    - PHY reset (re-runs link training from Polling) and counters/history clear.
    - SerDes RX re-initialization (transceiver RX reset sequence).
    """
    def __init__(self, phy, serdes, sys_clk_freq):
        ltssm = phy.ltssm
        pipe  = phy.pipe

        self._phy_reset       = CSRStorage(description="PHY (PIPE + LTSSM) reset (level).")
        self._serdes_rx_reset = CSRStorage(description="SerDes RX re-initialization (level).")
        self._clear           = CSR() # Clear counters and history (write).
        self._status          = CSRStatus(11, description="Status: LTSSM state [7:0], "
            "SerDes ready [8], RX idle [9], RX polarity [10].")
        self._timestamp       = CSRStatus(32, description="Timestamp (us).")

        # # #

        self.comb += phy.serdes_rx_reset.eq(self._serdes_rx_reset.storage)

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
        self._history_count  = CSRStatus(32, description="Number of LTSSM state transitions.")
        self._history_index  = CSRStorage(log2_int(depth),
            description="History entry to read (0 = oldest).")
        self._history_state  = CSRStatus(8,  description="History entry state.")
        self._history_time   = CSRStatus(32, description="History entry timestamp (us).")
        self._history_freeze = CSRStorage(
            description="Keep the first transitions after a clear (else the last ones).")
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
            history_wr.we.eq((state != state_d) &
                ~(self._history_freeze.storage & (history_count >= depth))),
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
            serdes.source.ctrl[i] & (serdes.source.data[8*i:8*(i+1)] == K(28, 4))
            for i in range(4)]))
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
            ("u1",                ltssm.u1,                   1),
            ("u2",                ltssm.u2,                   1),
            ("u3",                ltssm.u3,                   1),
        ]
        for name, signal, edge in events:
            csr = CSRStatus(32, name=f"{name}_count",
                description=f"{name} {'events' if edge else 'cycles'} count (saturating).")
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
        self.clear_ps = clear_ps = PulseSynchronizer("sys", clock_domain)
        self.comb += clear_ps.i.eq(self._clear.re)
        clear   = clear_ps.o
        sync_rx = getattr(self.sync, clock_domain)

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
            error.eq(Reduce("OR",
                events["decoder_invalid"] + events["code_invalid"] + events["disparity"])),
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
            csr  = CSRStatus(32, name=f"{name}_count", description=f"RX {name} count.")
            sync = BusSynchronizer(32, clock_domain, "sys")
            self.submodules += sync
            setattr(self, f"_{name}_count", csr)
            self.comb += [
                sync.i.eq(count),
                csr.status.eq(sync.o),
            ]
        for name, value, description in [
            ("bursts_count", bursts,  "RX error bursts count."),
            ("max_good_run", max_run, "RX longest error-free run (words)."),
            ]:
            csr  = CSRStatus(32, name=name, description=description)
            sync = BusSynchronizer(32, clock_domain, "sys")
            self.submodules += sync
            setattr(self, f"_{csr.name}", csr)
            self.comb += [
                sync.i.eq(value),
                csr.status.eq(sync.o),
            ]

# USB3 SerDes Debug --------------------------------------------------------------------------------

class USB3SerDesDebug(LiteXModule):
    """USB3 SerDes Debug (CSRs, hardware bring-up of the USB3 SerDes: A7/K7/USP).

    - TX polarity / TX inhibit controls (TX inhibit can be restricted to an LTSSM state, ex to
      isolate the partner's signal from crosstalk during training).
    - Transceiver loopback control (0b010: Near-End PMA).
    - TX/RX clocks frequency meters (100ms period): the RX clock is the recovered clock, its offset
      vs the TX clock shows the CDR frequency tracking of the partner (SSC: 0/-5000ppm).
    - RX vs TX clocks phase (rx_tx_phase, sys clock domain, for LiteScope): its slope is the
      instantaneous frequency offset of the recovered clock (SSC profile, CDR tracking limits).
    - Transceiver DRP access (attributes changes without rebuild, ex: RXCDR_CFG/LPM sweeps; to be
      followed by a SerDes RX re-initialization).
    - RX 8b/10b errors statistics (USB3RXErrorsDebug).
    """
    def __init__(self, serdes, sys_clk_freq, ltssm_state=None):
        self._tx_polarity      = CSRStorage(description="TX polarity.")
        self._tx_inhibit       = CSRStorage(description="TX inhibit.")
        self._tx_inhibit_state = CSRStorage(9, description="TX inhibit while in LTSSM state [7:0] "
                                                            "when enabled [8].")
        self._loopback         = CSRStorage(3,
            description="Transceiver loopback (0b010: Near-End PMA).")
        self.rx_tx_phase       = Signal(16) # o

        # # #

        gt = getattr(serdes, serdes.xcvr_name)

        # TX/Loopback controls.
        tx_inhibit_state = Signal()
        if ltssm_state is not None:
            self.comb += tx_inhibit_state.eq(self._tx_inhibit_state.storage[8] &
                (ltssm_state == self._tx_inhibit_state.storage[:8]))
        self.comb += [
            serdes.tx_polarity.eq(self._tx_polarity.storage),
            gt.tx_inhibit.eq(self._tx_inhibit.storage | tx_inhibit_state),
            gt.loopback.eq(self._loopback.storage),
        ]

        # DRP access (selected on the transceiver DRP mux during the accesses, the mux default is
        # the transceiver RX initialization).
        self._drp_sel = CSRStorage(description="Select DRP access on the transceiver DRP mux.")
        self.drp      = DRPControl(gt.drp)
        self.comb += gt.drp_mux.sel.eq(self._drp_sel.storage)

        # RX vs TX clocks phase (gray counters).
        rx_count = ClockDomainsRenamer("rx")(GrayCounter(16))
        tx_count = ClockDomainsRenamer("tx")(GrayCounter(16))
        self.submodules += rx_count, tx_count
        self.comb += rx_count.ce.eq(1), tx_count.ce.eq(1)
        rx_gray = Signal(16)
        tx_gray = Signal(16)
        self.specials += [
            MultiReg(rx_count.q, rx_gray),
            MultiReg(tx_count.q, tx_gray),
        ]
        self.sync += self.rx_tx_phase.eq(gray_decode(rx_gray) - gray_decode(tx_gray))

        # Clocks frequency meters.
        self.tx_freq = FreqMeter(period=int(sys_clk_freq/10), clk=ClockSignal("tx"))
        self.rx_freq = FreqMeter(period=int(sys_clk_freq/10), clk=ClockSignal("rx"))

        # RX errors.
        self.rx_errors = USB3RXErrorsDebug(gt.decoders, serdes.rx_substitution.checker)

# USB3 Analyzer Groups -----------------------------------------------------------------------------

USB3_ANALYZER_GROUPS = ["link", "lfps", "ts", "datapath", "phase"]

def usb3_analyzer_groups(phy, serdes, serdes_debug=None):
    """LiteScope probes groups of a USB3PHY/USB3SerDes (bench/usb3_analyzer.py), returns a dict
    {name: signals}:
    - link     : LTSSM + TS/LFPS Polling detections/generation, RX idle/errors/polarity.
    - lfps     : LTSSM + LFPS TX/RX (Polling, Ping, U1/U2/U3 exit, Warm Reset).
    - ts       : LTSSM + TS detections/generation, SerDes RX/TX streams.
    - datapath : LTSSM + SerDes and PHY RX/TX streams, SKP removal.
    - phase    : LTSSM + RX vs TX clocks phase (requires serdes_debug: USB3SerDesDebug).
    """
    pipe  = phy.pipe
    ltssm = phy.ltssm
    link_signals = [
        phy.ltssm_state,
        ltssm.u0,
        ltssm.recovery,
        ltssm.rx_ready,
        ltssm.tx_ready,
        ltssm.idle_handshake,
        ltssm.idle_handshake_done,
        ltssm.ts2_decode,
    ]
    groups = {
        "link" : link_signals + [
            pipe.ts_rx_ts1, pipe.ts_rx_ts1_inv, pipe.ts_rx_ts2, pipe.ts_tx_done,
            pipe.ts_tx_tseq, pipe.ts_tx_ts1, pipe.ts_tx_ts2,
            pipe.lfps_rx_polling, pipe.lfps_tx_polling, pipe.lfps_tx_count,
            serdes.rx_idle, serdes.rx_errors, serdes.rx_polarity,
        ],
        "lfps" : link_signals + [
            serdes.tx_idle, serdes.tx_lfps, serdes.tx_pattern,
            serdes.rx_idle, serdes.rx_errors,
            pipe.lfps_rx_polling, pipe.lfps_rx_ping, pipe.lfps_rx_exit_u1,
            pipe.lfps_rx_exit_u2, pipe.lfps_rx_wakeup_u3, pipe.lfps_rx_reset,
            pipe.lfps_tx_polling, pipe.lfps_tx_count,
        ],
        "ts" : link_signals + [
            pipe.ts_rx_ts1, pipe.ts_rx_ts1_inv, pipe.ts_rx_ts2, pipe.ts_tx_done,
            pipe.ts_tx_tseq, pipe.ts_tx_ts1, pipe.ts_tx_ts2,
            serdes.source, serdes.sink,
        ],
        "datapath" : link_signals + [
            serdes.rx_errors,
            serdes.rx_datapath.skip_remover.skip,
            serdes.source, serdes.sink,
            phy.source, phy.sink,
        ],
    }
    if serdes_debug is not None:
        groups["phase"] = link_signals + [
            serdes_debug.rx_tx_phase,
            serdes.rx_idle, serdes.rx_errors,
        ]
    return groups
