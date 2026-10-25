#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Gen 1x2 Host <-> Device Verilator simulation: two USB3X2PHY (4 SerDes models) train to U0 through
the SuperSpeedPlus Polling (SCD1/SCD2, LBPM) and exchange 64-bit PRBS packets (Link-Layer stand-ins),
with lane-to-lane skew, per lane polarity and SKP Ordered Sets differences (see doc/gen2-x2-roadmap.md)."""

import argparse

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.build.io import CRG
from litex.build.sim.config import SimConfig

from litex.soc.interconnect import stream
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe.x2    import USB3X2PHY
from usb3_pipe.lanes import link_layout

from sim import Platform, USB3SerDesModel, NCOClockGenerator

# PRBS 64-bit Link-Layer Stand-Ins -----------------------------------------------------------------

PRBS_START = (0xfbfbfbfbfbfbfbfb, 0xff) # K27.7 x8.
PRBS_END   = (0xfdfdfdfdfdfdfdfd, 0xff) # K29.7 x8.

def xorshift64(x):
    x1 = x ^ (x << 13)[:64]
    x2 = x1 ^ (x1 >> 7)
    return x2 ^ (x2 << 17)[:64]

class PRBS64Generator(LiteXModule):
    """Packets: START, 16 PRBS words (sequence continued from packet to packet), END, gaps of 0-3 cycles
    (Logical Idle)."""
    def __init__(self, length=16):
        self.enable = Signal()
        self.source = source = stream.Endpoint(link_layout())

        # # #

        prbs  = Signal(64, reset=0x0123456789abcdef)
        count = Signal(8)
        gap   = Signal(2)
        self.fsm = fsm = FSM(reset_state="GAP")
        fsm.act("GAP",
            NextValue(gap, gap - 1),
            If(self.enable & (gap == 0),
                NextState("START")
            )
        )
        fsm.act("START",
            source.valid.eq(1),
            source.data.eq(PRBS_START[0]),
            source.ctrl.eq(PRBS_START[1]),
            If(source.ready,
                NextValue(count, 0),
                NextState("DATA")
            )
        )
        fsm.act("DATA",
            source.valid.eq(1),
            source.data.eq(prbs),
            If(source.ready,
                NextValue(prbs, xorshift64(prbs)),
                NextValue(count, count + 1),
                If(count == (length - 1),
                    NextState("END")
                )
            )
        )
        fsm.act("END",
            source.valid.eq(1),
            source.data.eq(PRBS_END[0]),
            source.ctrl.eq(PRBS_END[1]),
            If(source.ready,
                NextValue(gap, prbs[0:2]),
                NextState("GAP")
            )
        )

class PRBS64Checker(LiteXModule):
    def __init__(self):
        self.enable  = Signal()
        self.sink    = sink = stream.Endpoint(link_layout())
        self.words   = Signal(32)
        self.packets = Signal(32)
        self.errors  = Signal(32)

        # # #

        prbs    = Signal(64, reset=0x0123456789abcdef)
        in_pkt  = Signal()
        self.comb += sink.ready.eq(1)
        is_start = (sink.data == PRBS_START[0]) & (sink.ctrl == PRBS_START[1])
        is_end   = (sink.data == PRBS_END[0])   & (sink.ctrl == PRBS_END[1])
        self.sync += If(self.enable & sink.valid,
            If(is_start,
                If(in_pkt, self.errors.eq(self.errors + 1)),
                in_pkt.eq(1)
            ).Elif(is_end,
                If(in_pkt, self.packets.eq(self.packets + 1)),
                in_pkt.eq(0)
            ).Elif(in_pkt,
                self.words.eq(self.words + 1),
                prbs.eq(xorshift64(prbs)),
                If((sink.data != prbs) | (sink.ctrl != 0),
                    self.errors.eq(self.errors + 1),
                    # Resynchronize on the received word.
                    prbs.eq(xorshift64(sink.data)),
                )
            ).Elif((sink.data != 0) | (sink.ctrl != 0),
                self.errors.eq(self.errors + 1)
            )
        )

# Simulation SoC -----------------------------------------------------------------------------------

class USB3X2Sim(SoCMini):
    """Host <-> Device Gen 1x2 simulation.

    - lane0/1_delay:    Lane 0/1 delay (words of 4 symbols) from Host to Device (lane-to-lane skew).
    - lane1_word_shift: Lane 1 RX word shift (symbols) of the Device SerDes model.
    - lane_skp:         Extra SKP Ordered Sets (2 Symbols) on the Host Lane 1 then Lane 0 every lane_skp
                        cycles in U0 (per lane SKP differences, 0 = disabled, >= 4096 for the model).
    - ppm:              Device clock offset (ppm).
    """
    def __init__(self, timer_scale=1, prbs_words=2**14, max_cycles=2**24, lane0_delay=0, lane1_delay=1,
        lane1_word_shift=2, lane_skp=4096, ppm=0):
        sys_clk_freq = int(133e6)
        platform     = Platform()
        self.comb += platform.trace.eq(1)
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)
        self.crg = CRG(platform.request("sys_clk"))
        if ppm != 0:
            self.cd_host  = ClockDomain()
            self.cd_dev   = ClockDomain()
            self.host_crg = NCOClockGenerator(self.cd_host)
            self.dev_crg  = NCOClockGenerator(self.cd_dev, ppm=ppm)
            cds = {"host": "host", "dev": "dev"}
        else:
            cds = {"host": "sys", "dev": "sys"}

        phys     = {}
        serdeses = {}
        checkers = {}
        for name, role, partner in [("host", "host", "dev"), ("dev", "device", "host")]:
            renamer = ClockDomainsRenamer({"sys": cds[name], "rx": cds[partner]})
            lanes   = []
            for lane in range(2):
                serdes = renamer(USB3SerDesModel(
                    phy_dw          = 40,
                    rx_word_shift   = lane1_word_shift if (name == "dev" and lane == 1) else 0,
                    rx_clock_domain = "rx"))
                self.add_module(name=f"{name}_serdes{lane}", module=serdes)
                lanes.append(serdes)
            # Polarity inversion on Host Lane 1 TX only (per lane polarity, 6.13.7).
            self.comb += lanes[1].tx_polarity.eq(name == "host")
            phy = renamer(USB3X2PHY(lanes,
                sys_clk_freq = sys_clk_freq,
                role         = role,
                timer_scale  = timer_scale))
            self.add_module(name=f"{name}_phy", module=phy)
            generator = renamer(PRBS64Generator())
            checker   = renamer(PRBS64Checker())
            self.add_module(name=f"{name}_prbs_generator", module=generator)
            self.add_module(name=f"{name}_prbs_checker",   module=checker)
            self.comb += [
                generator.enable.eq(phy.u0),
                generator.source.connect(phy.sink),
                checker.enable.eq(phy.u0),
                phy.source.connect(checker.sink),
                phy.ltssm.link_activity.eq(1),
            ]
            phys[name]     = phy
            serdeses[name] = lanes
            checkers[name] = checker

        # Lanes connection (Host -> Device lanes delayed) ------------------------------------------
        for lane in range(2):
            h = serdeses["host"][lane]
            d = serdeses["dev"][lane]
            self.comb += d.tx.connect(h.rx)
            n = {0: lane0_delay, 1: lane1_delay}[lane]
            if n > 0:
                delay = [Signal(40) for _ in range(n)]
                sync  = getattr(self.sync, cds["host"])
                sync += delay[0].eq(h.tx.data)
                for i in range(1, n):
                    sync += delay[i].eq(delay[i-1])
                self.comb += d.rx.data.eq(delay[-1])
            else:
                self.comb += h.tx.connect(d.rx)
            self.specials += MultiReg(h.tx_idle, d.rx_idle)
            self.specials += MultiReg(d.tx_idle, h.rx_idle)

        # Extra SKP Ordered Sets on a single Host lane at a time -----------------------------------
        # Each lane receives SKP Ordered Sets not present on the other lane (lane-to-lane skew varying
        # by 2 Symbols, per lane SKP removal). The SerDes model holds its TX datapath for the inserted
        # SKP Ordered Sets (1 word every 2 insertions, absorbed by the TX CDC FIFO): they are inserted
        # alternately on Lane 1 and Lane 0 and rarely enough for the TX CDC FIFOs not to fill (the
        # lanes TX must stay in lockstep).
        if lane_skp:
            skp_timer = Signal(32)
            sync = getattr(self.sync, cds["host"])
            sync += [
                serdeses["host"][0].tx_skp.eq(0),
                serdeses["host"][1].tx_skp.eq(0),
                If(phys["host"].u0,
                    skp_timer.eq(skp_timer + 1),
                    If(skp_timer == (lane_skp//2 - 1),
                        serdeses["host"][1].tx_skp.eq(1),
                    ),
                    If(skp_timer == (lane_skp - 1),
                        skp_timer.eq(0),
                        serdeses["host"][0].tx_skp.eq(1),
                    )
                )
            ]

        # Simulation Timer / Status ----------------------------------------------------------------
        timer = Signal(32)
        self.sync += timer.eq(timer + 1)
        for name, phy in phys.items():
            phy.ltssm.finalize() # Elaborate the FSMs to expose their encodings.
            phy.ssp.finalize()
            fsm  = phy.ltssm.fsm
            sync = getattr(self.sync, cds[name])
            label = {"host": "HOST", "dev": "DEV "}[name]
            for state, value in fsm.encoding.items():
                sync += If((fsm.next_state != fsm.state) & (fsm.next_state == value),
                    Display("[%08d] {} entering {} state".format(label, state), timer)
                )
            ssp = phy.ssp.fsm
            for state, value in ssp.encoding.items():
                sync += If((ssp.next_state != ssp.state) & (ssp.next_state == value),
                    Display("[%08d] {} SSP entering {} state (x2: %d)".format(label, state),
                        timer, phy.ssp.x2)
                )
            aligned_d = Signal()
            sync += aligned_d.eq(phy.deskew_aligned)
            sync += If(phy.deskew_aligned & ~aligned_d,
                Display("[%08d] {} lanes deskewed".format(label), timer)
            )

        # Simulation End ---------------------------------------------------------------------------
        done   = Signal()
        report = Signal()
        self.comb += done.eq(
            (checkers["host"].words >= prbs_words) &
            (checkers["dev"].words  >= prbs_words))
        for name, checker in checkers.items():
            self.sync += If(done & ~report,
                Display("{} PRBS: %d words, %d packets, %d errors, x2: %d".format(name.upper()),
                    checker.words, checker.packets, checker.errors, phys[name].x2)
            )
        self.sync += [
            If(done | (timer >= max_cycles),
                report.eq(1),
                If(~done & ~report, Display("[%08d] Simulation timeout", timer))
            ),
            If(report, Finish())
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 Gen 1x2 PHY Simulation")
    parser.add_argument("--trace",            action="store_true", help="Enable VCD tracing.")
    parser.add_argument("--timer-scale",      default=1,           help="LTSSM/LFPS timers scale factor (1 = spec exact).")
    parser.add_argument("--prbs-words",       default=2**14,       help="64-bit words to check on both sides.")
    parser.add_argument("--max-cycles",       default=2**24,       help="Simulation timeout (sys clock cycles).")
    parser.add_argument("--lane0-delay",      default=0,           help="Host to Device Lane 0 delay (words).")
    parser.add_argument("--lane1-delay",      default=1,           help="Host to Device Lane 1 delay (words).")
    parser.add_argument("--lane1-word-shift", default=2,           help="Device Lane 1 RX word shift (symbols).")
    parser.add_argument("--lane-skp",         default=4096,        help="Extra Host Lane 1/Lane 0 SKP Ordered Sets period (cycles, 0: disabled).")
    parser.add_argument("--ppm",              default=0,           help="Device clock offset vs Host clock (ppm).")
    parser.add_argument("--output-dir",       default="build/sim_x2", help="Build directory.")
    args = parser.parse_args()

    soc = USB3X2Sim(
        timer_scale      = int(args.timer_scale),
        prbs_words       = int(args.prbs_words),
        max_cycles       = int(args.max_cycles),
        lane0_delay      = int(args.lane0_delay),
        lane1_delay      = int(args.lane1_delay),
        lane1_word_shift = int(args.lane1_word_shift),
        lane_skp         = int(args.lane_skp),
        ppm              = float(args.ppm))
    builder = Builder(soc, output_dir=args.output_dir)
    builder.build(sim_config=SimConfig(default_clk="sys_clk"),
        opt_level   = "O0",
        interactive = False,
        trace       = args.trace)

if __name__ == "__main__":
    main()
