#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# USB3-PIPE on the LambdaConcept ECPIX-5: progressive bring-up designs (small bitstreams, see
# doc/ecpix5.md), on the lambdaconcept_ecpix5.py base (CRG, USB-C control, LEDs):
# - serdes: LiteICLink SerDesECP5 (20-bit/250MHz) on DCU0 with PRBS/loopback, SCI access, TX/RX
#           clocks frequency meters, raw RX words observer (no USB3 logic). PLL/reference clock
#           experiments: --refclk extref (EXTREF0 100MHz), --dcu-mult/--dcu-div/--refck-mode.
# - dcu:    LiteICLink SerDesECP5 40-bit/125MHz words (gearboxes, fabric comma aligner), USB3 RX
#           settings, loopback checker (incrementing bytes: decode errors/commas/sequence breaks
#           counters, words capture).
# - lfps:   ECP5USB3SerDes with the LFPS Unit, no LTSSM/Link Layer (host LFPS/TSEQ/TS1 only, no
#           link training): LFPS bursts measurements, LOS/LDR captures, RX words monitor.

import argparse

import common # Repository root on the path.

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex_boards.platforms import lambdaconcept_ecpix5

from litex.soc.interconnect.csr import *
from litex.soc.cores.freqmeter  import FreqMeter
from litex.soc.cores.prbs       import PRBSRX

from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

from usb3_pipe      import ECP5USB3SerDes
from usb3_pipe.lfps import LFPSUnit

from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

from lambdaconcept_ecpix5 import ECPIX5BaseSoC, ecpix5_args

# SerDes PLL ---------------------------------------------------------------------------------------

class _SerDesECP5PLL:
    """DCU PLL configuration (refclk x m / d) for the bring-up, constrained for 5Gbps (250MHz TX/RX
    clocks): the actual line rate is measured with the TX/RX clocks frequency meters."""
    def __init__(self, refclk, refclk_freq, m, d):
        self.refclk = refclk
        self.config = {"clkin": refclk_freq, "m": m, "d": d, "vco_freq": refclk_freq*m, "linerate": 5e9}

# DCU Probes ---------------------------------------------------------------------------------------

def dcu_rx_los(serdes):
    """DCU RX loss of signal output (async)."""
    return serdes.serdes_params["o_CHX_FFS_RLOS"]

def dcu_add_rx_ldr(serdes):
    """DCU RX low data rate input (raw RX input value to the fabric, async)."""
    rx_ldr = Signal()
    serdes.serdes_params.update(p_CHX_LDR_RX2CORE_SEL="0b1", o_CHX_LDR_RX2CORE=rx_ldr)
    return rx_ldr

def dcu_add_rx_los_mask(serdes):
    """Mask of the DCU RX loss of signal to the initialization: RX up without RX signal (ex:
    near-end serial loopback with the RX inputs unconnected)."""
    mask     = Signal()
    rx_los   = Signal()
    init_los = serdes.serdes_params["o_CHX_FFS_RLOS"]
    serdes.serdes_params["o_CHX_FFS_RLOS"] = rx_los
    serdes.comb += init_los.eq(rx_los & ~mask)
    return mask, rx_los

# SerDes Status ------------------------------------------------------------------------------------

class SerDesStatus(LiteXModule):
    def __init__(self, serdes):
        self.status = CSRStatus(fields=[
            CSRField("tx_ready", size=1),
            CSRField("rx_ready", size=1),
            CSRField("rx_idle",  size=1, description="RX loss of signal."),
            CSRField("tx_lol",   size=1, description="TX PLL loss of lock."),
            CSRField("rx_lol",   size=1, description="RX CDR loss of lock."),
            CSRField("rx_los",   size=1, description="RX loss of signal (init)."),
        ])
        self.decode_errors = CSRStatus(32, description="RX 8b/10b decode errors (rx_ready).")
        self.commas        = CSRStatus(32, description="RX K28.5 received (rx_ready).")

        # # #

        tx_ready = Signal()
        rx_ready = Signal()
        self.specials += [
            MultiReg(serdes.tx_ready, tx_ready),
            MultiReg(serdes.rx_ready, rx_ready),
        ]
        self.comb += [
            self.status.fields.tx_ready.eq(tx_ready),
            self.status.fields.rx_ready.eq(rx_ready),
            self.status.fields.rx_idle.eq(serdes.rx_idle),
            self.status.fields.tx_lol.eq(serdes.init.tx_lol),
            self.status.fields.rx_lol.eq(serdes.init.rx_lol),
            self.status.fields.rx_los.eq(serdes.init.rx_los),
        ]

        # Decode errors/commas counters (rx clock domain, gray-free: read while stable, debug only).
        errors = Signal(32)
        commas = Signal(32)
        error  = Signal()
        comma  = Signal()
        self.comb += [
            error.eq(Reduce("OR", [d.invalid for d in serdes.decoders])),
            comma.eq(Reduce("OR", [d.k & (d.d == 0xbc) for d in serdes.decoders])),
        ]
        self.sync.rx += [
            If(error, errors.eq(errors + 1)),
            If(comma, commas.eq(commas + 1)),
        ]
        self.specials += [
            MultiReg(errors, self.decode_errors.status),
            MultiReg(commas, self.commas.status),
        ]

# SerDes RX Gearbox --------------------------------------------------------------------------------

class SerDesRXGearbox(LiteXModule):
    """DCU RX raw words 20-bit at 250MHz (recovered clock) to 40-bit at 125MHz ("rx_half" clock domain,
    CLKDIVF /2 of the recovered clock: frequency/phase related, timed by nextpnr). No reset."""
    def __init__(self, rxoutclk, rx_data):
        self.source = Signal(40)

        # # #

        self.cd_rx_full = ClockDomain(reset_less=True)
        self.cd_rx_half = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_full.clk.eq(rxoutclk)
        self.specials += Instance("CLKDIVF",
            p_DIV     = "2.0",
            i_CLKI    = rxoutclk,
            i_RST     = 0,
            i_ALIGNWD = 0,
            o_CDIVX   = self.cd_rx_half.clk,
        )

        toggle = Signal()
        lsb    = Signal(20)
        word   = Signal(40)
        self.sync.rx_full += [
            toggle.eq(~toggle),
            If(toggle,
                word.eq(Cat(lsb, rx_data))
            ).Else(
                lsb.eq(rx_data)
            )
        ]
        self.sync.rx_half += self.source.eq(word)

# SerDes RX Observer -------------------------------------------------------------------------------

class SerDesRXObserver(LiteXModule):
    """RX raw words PRBS checkers/capture at 125MHz (40-bit words from the SerDesRXGearbox), without
    reset (independent of the DCU init/CDR lock status, bring-up only)."""
    def __init__(self, serdes, depth=4):
        self.prbs_config   = CSRStorage(2, description="RX PRBS checker (0: off, 1: PRBS7, 2: PRBS15, 3: PRBS31).")
        self.prbs_errors   = CSRStatus(32, description="RX PRBS errors (reversed checker).")
        self.prbs_errors_n = CSRStatus(32, description="RX PRBS errors (non-reversed checker).")
        self.capture       = CSRStorage(description="Capture RX raw words (write 1).")
        for i in range(depth):
            setattr(self, f"word{i}", CSRStatus(40, name=f"word{i}"))

        # # #

        # DCU RX bus (2x 10-bit raw symbols at bits 0-9/12-21), before the LiteICLink aligner/resets.
        rx_bus  = Cat(*[serdes.serdes_params[f"o_CHX_FF_RX_D_{n}"] for n in range(24)])
        rx_raw  = Cat(rx_bus[0:10], rx_bus[12:22])
        self.gearbox = gearbox = SerDesRXGearbox(serdes.rxoutclk, rx_raw)
        rx_data = Signal(40)
        self.sync.rx_half += rx_data.eq(gearbox.source)

        prbs_config = Signal(2)
        self.specials += MultiReg(self.prbs_config.storage, prbs_config, "rx_half")
        for name, reverse, csr in [("prbs", True, self.prbs_errors), ("prbs_n", False, self.prbs_errors_n)]:
            prbs = ClockDomainsRenamer("rx_half")(PRBSRX(40, reverse=reverse))
            self.add_module(name=name, module=prbs)
            self.comb += [
                prbs.config.eq(prbs_config),
                prbs.i.eq(rx_data),
            ]
            self.specials += MultiReg(prbs.errors, csr.status)

        capture = PulseSynchronizer("sys", "rx_half")
        self.submodules += capture
        self.comb += capture.i.eq(self.capture.re)
        count = Signal(max=depth + 1, reset=depth)
        words = [Signal(40) for _ in range(depth)]
        self.sync.rx_half += [
            If(capture.o,
                count.eq(0)
            ).Elif(count != depth,
                count.eq(count + 1),
                Case(count, {i: words[i].eq(rx_data) for i in range(depth)}),
            )
        ]
        for i, word in enumerate(words):
            self.specials += MultiReg(word, getattr(self, f"word{i}").status)

# DCU 40-bit Checker -------------------------------------------------------------------------------

class DCUChecker(LiteXModule):
    """SerDesECP5 40-bit path check (dcu stage): TX incrementing bytes with a COM word every 16
    words, RX decode errors/commas/sequence breaks counters (symbols rotation agnostic), aligner
    status. rx_los: DCU RX loss of signal, rx_los_mask: its mask to the initialization."""
    def __init__(self, dcu, sys_clk_freq, rx_los, rx_los_mask):
        self.status = CSRStatus(fields=[
            CSRField("tx_ready",   size=1),
            CSRField("rx_ready",   size=1),
            CSRField("tx_lol",     size=1),
            CSRField("rx_lol",     size=1),
            CSRField("rx_los",     size=1),
            CSRField("rx_aligned", size=1),
            CSRField("offset",     size=4, offset=8),
        ])
        self.control = CSRStorage(fields=[
            CSRField("loopback",   size=1, description="Near-end serial loopback (SCI lb_ctl)."),
            CSRField("rx_release", size=1, description="Mask the RX loss of signal to the init (RX up without RX signal)."),
            CSRField("rx_align",   size=1, reset=1, description="RX comma alignment enable."),
        ])
        self.errors = CSRStatus(32, description="RX decode errors.")
        self.commas = CSRStatus(32, description="RX COMs.")
        self.breaks = CSRStatus(32, description="RX data sequence breaks.")
        self.words  = CSRStatus(32, description="RX words.")
        self.capture = CSRStorage(description="Capture 8 RX words (k[35:32], d[31:0]).")
        for i in range(8):
            setattr(self, f"word{i}", CSRStatus(36, name=f"word{i}"))
        self.tx_freq = FreqMeter(period=int(sys_clk_freq), clk=ClockSignal("tx"))
        self.rx_freq = FreqMeter(period=int(sys_clk_freq), clk=ClockSignal("rx"))

        # # #

        offset   = Signal(4)
        aligned  = Signal()
        rx_los_s = Signal()
        self.specials += [
            MultiReg(dcu.rx_aligner.offset, offset),
            MultiReg(dcu.rx_aligner.aligned, aligned),
            MultiReg(rx_los, rx_los_s),
        ]
        self.comb += [
            self.status.fields.tx_ready.eq(dcu.tx_ready),
            self.status.fields.rx_ready.eq(dcu.rx_ready),
            self.status.fields.tx_lol.eq(dcu.init.tx_lol),
            self.status.fields.rx_lol.eq(dcu.init.rx_lol),
            self.status.fields.rx_los.eq(rx_los_s),
            self.status.fields.rx_aligned.eq(aligned),
            self.status.fields.offset.eq(offset),
            dcu.loopback.eq(self.control.fields.loopback),
            rx_los_mask.eq(self.control.fields.rx_release),
            dcu.rx_align.eq(self.control.fields.rx_align),
        ]

        # TX: incrementing bytes, COM word every 16 words.
        count = Signal(4)
        data  = Signal(8)
        self.sync.tx += [
            count.eq(count + 1),
            If(count != 0, data.eq(data + 4)),
        ]
        self.comb += [
            If(count == 0,
                dcu.sink.ctrl.eq(0b1111),
                dcu.sink.data.eq(Replicate(Constant(0xbc, 8), 4)),
            ).Else(
                dcu.sink.ctrl.eq(0),
                dcu.sink.data.eq(Cat(data, (data + 1)[:8], (data + 2)[:8], (data + 3)[:8])),
            )
        ]

        # RX: counters (symbols in order: lsb first, any rotation), with a no-reset clock domain.
        self.cd_rx_chk = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_chk.clk.eq(ClockSignal("rx"))
        errors = Signal(32)
        commas = Signal(32)
        breaks = Signal(32)
        words  = Signal(32)
        last   = Signal(8)
        last_k = Signal()
        sym_d  = [dcu.decoders[i].d for i in range(4)]
        sym_k  = [dcu.decoders[i].k for i in range(4)]
        sym_e  = [dcu.decoders[i].invalid for i in range(4)]
        brk    = Signal(4)
        prev_d, prev_k = last, last_k
        for i in range(4):
            self.comb += brk[i].eq(~sym_k[i] & ~prev_k & (sym_d[i] != (prev_d + 1)[:8]))
            prev_d, prev_k = sym_d[i], sym_k[i]
        self.sync.rx_chk += [
            words.eq(words + 1),
            last.eq(sym_d[3]),
            last_k.eq(sym_k[3]),
            If(Reduce("OR", sym_e), errors.eq(errors + 1)),
            If(Reduce("OR", [sym_k[i] & (sym_d[i] == 0xbc) for i in range(4)]), commas.eq(commas + 1)),
            If(brk != 0, breaks.eq(breaks + 1)),
        ]
        capture = PulseSynchronizer("sys", "rx_chk")
        self.submodules += capture
        self.comb += capture.i.eq(self.capture.re)
        cap_count = Signal(4, reset=8)
        cap_words = [Signal(36) for _ in range(8)]
        rx_word   = Cat(*sym_d, *sym_k)
        self.sync.rx_chk += [
            If(capture.o,
                cap_count.eq(0)
            ).Elif(cap_count != 8,
                cap_count.eq(cap_count + 1),
                Case(cap_count, {i: cap_words[i].eq(rx_word) for i in range(8)}),
            )
        ]
        for i, w in enumerate(cap_words):
            self.specials += MultiReg(w, getattr(self, f"word{i}").status)
        self.specials += [
            MultiReg(errors, self.errors.status),
            MultiReg(commas, self.commas.status),
            MultiReg(breaks, self.breaks.status),
            MultiReg(words,  self.words.status),
        ]

# LFPS Monitor -------------------------------------------------------------------------------------

class LFPSMonitor(LiteXModule):
    """LFPS bursts measurements on the SerDes filtered RX Electrical Idle (sys clock cycles): bursts
    count, last/min/max burst width, last period (burst starts), Polling LFPS detections (lfps
    stage, S4). TX: Polling LFPS generation on request (else Electrical Idle)."""
    def __init__(self, serdes, lfps):
        self.control = CSRStorage(fields=[
            CSRField("tx_polling", size=1, description="Transmit Polling LFPS."),
            CSRField("clear",      size=1, pulse=True, description="Clear the measurements."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("ready",      size=1, description="SerDes ready."),
            CSRField("rx_idle",    size=1, description="RX Electrical Idle (filtered)."),
            CSRField("rx_polling", size=1, description="Polling LFPS detected."),
        ])
        self.bursts    = CSRStatus(32, description="RX bursts.")
        self.width     = CSRStatus(32, description="RX last burst width.")
        self.width_min = CSRStatus(32, description="RX min burst width.")
        self.width_max = CSRStatus(32, description="RX max burst width.")
        self.period    = CSRStatus(32, description="RX last burst period.")
        self.polling   = CSRStatus(32, description="RX Polling LFPS detections.")
        self.tx_count  = CSRStatus(16, description="TX Polling LFPS bursts.")

        # # #

        self.comb += [
            lfps.tx_idle.eq(1),
            lfps.tx_polling.eq(self.control.fields.tx_polling),
            self.status.fields.ready.eq(serdes.ready),
            self.status.fields.rx_idle.eq(serdes.rx_idle),
            self.status.fields.rx_polling.eq(lfps.rx_polling),
            self.tx_count.status.eq(lfps.tx_count),
        ]

        idle      = serdes.rx_idle
        idle_d    = Signal(reset=1)
        polling_d = Signal()
        width     = Signal(32)
        period    = Signal(32)
        clear     = self.control.fields.clear
        self.sync += [
            idle_d.eq(idle),
            polling_d.eq(lfps.rx_polling),
            period.eq(period + 1),
            If(~idle, width.eq(width + 1)),
            # Burst start.
            If(idle_d & ~idle,
                self.bursts.status.eq(self.bursts.status + 1),
                self.period.status.eq(period),
                period.eq(1),
                width.eq(1),
            ),
            # Burst end.
            If(~idle_d & idle,
                self.width.status.eq(width),
                If(width < self.width_min.status, self.width_min.status.eq(width)),
                If(width > self.width_max.status, self.width_max.status.eq(width)),
            ),
            If(lfps.rx_polling & ~polling_d, self.polling.status.eq(self.polling.status + 1)),
            If(clear,
                self.bursts.status.eq(0),
                self.polling.status.eq(0),
                self.width_min.status.eq(2**32 - 1),
                self.width_max.status.eq(0),
            ),
        ]

# LOS Capture --------------------------------------------------------------------------------------

class LOSCapture(LiteXModule):
    """RX signal capture (DCU loss of signal or LDR input): 32*depth samples (1 sample every divider
    + 1 sys cycles) in a block RAM, armed by a write to arm, triggered on the signal low (trigger
    "low": LOS) or on its first edge (trigger "edge": LDR), read with addr/data (32 samples per
    word, first sample in bit 0)."""
    def __init__(self, rx_los, depth=64, trigger="low"):
        self.arm     = CSR()
        self.divider = CSRStorage(16, reset=124, description="Sample period - 1 (sys cycles).")
        self.addr    = CSRStorage(log2_int(depth), description="Read address.")
        self.data    = CSRStatus(32, description="Read data (32 samples).")
        self.status  = CSRStatus(fields=[
            CSRField("armed", size=1),
            CSRField("done",  size=1),
        ])

        # # #

        los   = Signal(reset=1)
        los_d = Signal(reset=1)
        self.specials += MultiReg(rx_los, los, reset=1)
        self.sync += los_d.eq(los)
        triggered = ~los if trigger == "low" else (los != los_d)

        mem = Memory(32, depth)
        wr  = mem.get_port(write_capable=True)
        rd  = mem.get_port()
        self.specials += mem, wr, rd
        self.comb += [
            rd.adr.eq(self.addr.storage),
            self.data.status.eq(rd.dat_r),
        ]

        armed   = Signal()
        running = Signal()
        done    = Signal()
        count   = Signal(16)
        bit     = Signal(5)
        word    = Signal(32)
        self.comb += [
            self.status.fields.armed.eq(armed),
            self.status.fields.done.eq(done),
            wr.dat_w.eq(Cat(word[1:], los)),
        ]
        self.sync += [
            wr.we.eq(0),
            If(self.arm.re,
                armed.eq(1),
                running.eq(0),
                done.eq(0),
                wr.adr.eq(0),
                bit.eq(0),
            ).Elif(armed & triggered,
                armed.eq(0),
                running.eq(1),
                count.eq(0),
            ).Elif(running,
                count.eq(count + 1),
                If(count == self.divider.storage,
                    count.eq(0),
                    word.eq(Cat(word[1:], los)),
                    bit.eq(bit + 1),
                    If(bit == 31,
                        wr.we.eq(1),
                    ),
                ),
                If(wr.we,
                    wr.adr.eq(wr.adr + 1),
                    If(wr.adr == (depth - 1),
                        running.eq(0),
                        done.eq(1),
                    )
                )
            )
        ]

# RX Monitor ---------------------------------------------------------------------------------------

class RXMonitor(LiteXModule):
    """SerDesECP5 40-bit RX words monitor (S5, host TSEQ/TS1 after the LFPS handshake), on a no-reset clock
    domain on the recovered clock: words/ready/aligned words, decode errors (rx ready), COMs/SKPs
    counters, decoded words capture (block RAM, 36-bit: k[35:32]/d[31:0], triggered on a COM),
    recovered clock frequency."""
    def __init__(self, dcu, sys_clk_freq, depth=256):
        self.clear   = CSR()
        self.arm     = CSR()
        self.status  = CSRStatus(fields=[
            CSRField("rx_ready", size=1),
            CSRField("aligned",  size=1),
            CSRField("offset",   size=4, offset=4),
            CSRField("done",     size=1, offset=8),
        ])
        self.addr    = CSRStorage(log2_int(depth))
        self.data    = CSRStatus(36)
        self.rx_freq = FreqMeter(period=int(sys_clk_freq), clk=ClockSignal("rx"))
        names = ["words", "ready", "aligned", "errors", "coms", "skps", "data_words", "data_errors",
            "com_good", "com_bad"]
        for name in names:
            setattr(self, name, CSRStatus(32, name=name))

        # # #

        self.cd_rx_mon = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_mon.clk.eq(ClockSignal("rx"))

        rx_ready = Signal()
        self.specials += MultiReg(dcu.rx_ready, rx_ready, "rx_mon")
        k = [d.k for d in dcu.decoders]
        d = [d.d for d in dcu.decoders]
        e = [d.invalid for d in dcu.decoders]
        com = Reduce("OR", [k[i] & (d[i] == 0xbc) for i in range(4)])
        skp = Reduce("OR", [k[i] & (d[i] == 0x3c) for i in range(4)])

        clear = PulseSynchronizer("sys", "rx_mon")
        arm   = PulseSynchronizer("sys", "rx_mon")
        self.submodules += clear, arm
        self.comb += clear.i.eq(self.clear.re), arm.i.eq(self.arm.re)

        # Data window: words within 64 words of a COM (Ordered Sets streams, excludes LFPS/idle).
        window = Signal(7)
        self.sync.rx_mon += If(rx_ready & com, window.eq(64)).Elif(window != 0, window.eq(window - 1))
        in_data = rx_ready & (window != 0)

        # COM spacing (symbols, first COM of a word vs last COM before): 1/13 (TS1/TS2: 4 COMs + 12
        # symbols), 32 (TSEQ), else bad (slips). Registered inputs.
        coms_r   = Signal(4)
        position = Signal(16)
        last_com = Signal(16)
        com_good = Signal()
        com_bad  = Signal()
        self.sync.rx_mon += [
            coms_r.eq(Cat(*[rx_ready & k[i] & (d[i] == 0xbc) for i in range(4)])),
            position.eq(position + 4),
        ]
        first = Signal(2)
        last  = Signal(2)
        for i in reversed(range(4)):
            self.comb += If(coms_r[i], first.eq(i))
        for i in range(4):
            self.comb += If(coms_r[i], last.eq(i))
        dist = Signal(16)
        self.comb += dist.eq(position + first - last_com)
        self.sync.rx_mon += [
            com_good.eq(0),
            com_bad.eq(0),
            If(coms_r != 0,
                last_com.eq(position + last),
                If((dist == 1) | (dist == 13) | (dist == 32),
                    com_good.eq(1)
                ).Else(
                    com_bad.eq(1)
                )
            )
        ]

        counters = {n: Signal(32) for n in names}
        increments = {}
        events   = {
            "data_words"  : in_data,
            "data_errors" : in_data & Reduce("OR", e),
            "com_good"    : com_good != 0,
            "com_bad"     : com_bad != 0,
            "words"   : 1,
            "ready"   : rx_ready,
            "aligned" : rx_ready & dcu.rx_aligner.aligned,
            "errors"  : rx_ready & Reduce("OR", e),
            "coms"    : rx_ready & com,
            "skps"    : rx_ready & skp,
        }
        for n in names:
            inc = increments.get(n, 1)
            self.sync.rx_mon += If(clear.o, counters[n].eq(0)).Elif(events[n], counters[n].eq(counters[n] + inc))
            self.specials += MultiReg(counters[n], getattr(self, n).status)

        offset  = Signal(4)
        aligned = Signal()
        self.specials += [
            MultiReg(dcu.rx_aligner.offset, offset),
            MultiReg(dcu.rx_aligner.aligned, aligned),
        ]

        # Capture.
        mem = Memory(36, depth)
        wr  = mem.get_port(write_capable=True, clock_domain="rx_mon")
        rd  = mem.get_port()
        self.specials += mem, wr, rd
        armed   = Signal()
        running = Signal()
        done    = Signal()
        self.comb += [
            wr.dat_w.eq(Cat(*d, *k)),
            wr.we.eq(running | (armed & rx_ready & com)),
            rd.adr.eq(self.addr.storage),
            self.data.status.eq(rd.dat_r),
        ]
        self.sync.rx_mon += [
            If(arm.o,
                armed.eq(1), running.eq(0), done.eq(0), wr.adr.eq(0),
            ).Elif(armed & rx_ready & com,
                armed.eq(0), running.eq(1), wr.adr.eq(1),
            ).Elif(running,
                wr.adr.eq(wr.adr + 1),
                If(wr.adr == (depth - 1), running.eq(0), done.eq(1)),
            )
        ]
        done_sys = Signal()
        self.specials += MultiReg(done, done_sys)
        self.comb += [
            self.status.fields.rx_ready.eq(dcu.rx_ready),
            self.status.fields.aligned.eq(aligned),
            self.status.fields.offset.eq(offset),
            self.status.fields.done.eq(done_sys),
        ]


# BringupSoC ---------------------------------------------------------------------------------------

class BringupSoC(ECPIX5BaseSoC):
    def __init__(self, platform, sys_clk_freq=125e6, stage="serdes", channel=0, refclk="fabric",
        refclk_freq=250e6, dcu_mult=20, dcu_div=1, refck_mode=None, serdes_params={}, **kwargs):
        ECPIX5BaseSoC.__init__(self, platform, sys_clk_freq, refclk_freq=refclk_freq, **kwargs)

        # SerDes Reference Clock: fabric (CRG) or EXTREF0 (serdes stage experiments).
        if refclk == "extref":
            assert stage == "serdes"
            serdes_refclk = Signal()
            refclk_pads   = platform.request("serdes_clk100")
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP     = refclk_pads.p,
                i_REFCLKN     = refclk_pads.n,
                o_REFCLKO     = serdes_refclk,
                p_REFCK_PWDNB = "0b1",
                p_REFCK_RTERM = "0b1", # 100 Ohm.
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))
        else:
            serdes_refclk = ClockSignal("refclk")

        # Stage.
        if stage == "serdes":
            self.add_serdes_stage(platform, serdes_refclk, refclk_freq, channel, dcu_mult, dcu_div,
                refck_mode, serdes_params)
        elif stage == "dcu":
            self.add_dcu_stage(platform, serdes_refclk, refclk_freq, channel)
        elif stage == "lfps":
            self.add_lfps_stage(platform, serdes_refclk, refclk_freq, channel)
            self.usb3_serdes.dcu.serdes_params.update(serdes_params) # DCU overrides.

    def add_serdes_stage(self, platform, serdes_refclk, refclk_freq, channel, dcu_mult, dcu_div,
        refck_mode, serdes_params):
        """LiteICLink SerDesECP5 (20-bit/250MHz) with PRBS/loopback/RX observer (S1/S2)."""
        sys_clk_freq = self.sys_clk_freq

        # DCU PLL: refclk x dcu_mult (D_REFCK_MODE), TX/RX clock divider dcu_div (D_TX_VCO_CK_DIV/
        # CHX_RX_DCO_CK_DIV).
        serdes_pll = _SerDesECP5PLL(serdes_refclk, refclk_freq=refclk_freq, m=dcu_mult, d=dcu_div)
        self.serdes = serdes = SerDesECP5(serdes_pll,
            tx_pads    = platform.request("usb3_tx", channel),
            rx_pads    = platform.request("usb3_rx", channel),
            dual       = 0,
            channel    = channel,
            data_width = 20,
            with_oob   = True) # Separate TX/RX init, raw 10BSER words aligned in the fabric.
        serdes.serdes_params.update(serdes_params) # DCU parameters overrides (bring-up).
        if refck_mode is not None:
            serdes.serdes_params.update(p_D_REFCK_MODE=refck_mode) # PLL multiplier code override.
        self.fb_loopback = CSRStorage(description="DCU FFC_FB_LOOPBACK (near-end serial loopback).")
        serdes.serdes_params.update(i_CHX_FFC_FB_LOOPBACK=self.fb_loopback.storage)
        self.rx_rst_force = CSRStorage(description="Release the RX PCS reset (bypass the DCU init, bring-up).")
        rx_pcs_rst = serdes.serdes_params["i_CHX_FFC_LANE_RX_RST"]
        serdes.serdes_params.update(i_CHX_FFC_LANE_RX_RST=rx_pcs_rst & ~self.rx_rst_force.storage)
        serdes.add_stream_endpoints()
        serdes.add_controls()
        self.comb += [
            serdes.sink.valid.eq(1),
            serdes.sink.ctrl.eq(0b01),
            serdes.sink.data.eq(0x00bc), # K28.5 + D0.0 (comma alignment checks in loopback).
        ]
        platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)
        platform.add_false_path_constraints(self.crg.cd_sys.clk, serdes.cd_tx.clk, serdes.cd_rx.clk)

        self.serdes_status = SerDesStatus(serdes)
        self.rx_obs        = SerDesRXObserver(serdes)
        self.tx_freq = FreqMeter(period=int(sys_clk_freq), clk=serdes.cd_tx.clk)
        self.rx_freq = FreqMeter(period=int(sys_clk_freq), clk=serdes.cd_rx.clk)

        # Leds: TX clock heartbeat, RX ready.
        beat    = Signal(27)
        tx_beat = Signal()
        self.sync.tx += beat.eq(beat + 1)
        self.specials += MultiReg(beat[-1], tx_beat)
        self.led(self.leds[0].g, tx_beat)
        self.led(self.leds[1].g, self.serdes_status.status.fields.rx_ready)

    def add_dcu_stage(self, platform, serdes_refclk, refclk_freq, channel):
        """SerDesECP5 (40-bit/125MHz words, gearboxes, fabric comma aligner, USB3 RX settings) with
        the DCUChecker (S3)."""
        pll = SerDesECP5PLL(serdes_refclk, refclk_freq, linerate=5e9)
        self.dcu = dcu = SerDesECP5(pll,
            tx_pads    = platform.request("usb3_tx", channel),
            rx_pads    = platform.request("usb3_rx", channel),
            dual       = 0,
            channel    = channel,
            data_width = 40,
            with_oob   = True)
        dcu.serdes_params.update(ECP5USB3SerDes.rx_settings)
        rx_los_mask, rx_los = dcu_add_rx_los_mask(dcu)
        dcu.add_stream_endpoints()
        platform.add_period_constraint(dcu.txoutclk, 1e9/(2*dcu.tx_clk_freq))
        platform.add_period_constraint(dcu.rxoutclk, 1e9/(2*dcu.rx_clk_freq))
        platform.add_false_path_constraints(self.crg.cd_sys.clk, dcu.txoutclk, dcu.rxoutclk)
        self.dcu_checker = DCUChecker(dcu, self.sys_clk_freq, rx_los, rx_los_mask)

        # Leds: TX ready, RX aligned.
        self.led(self.leds[0].g, dcu.tx_ready)
        self.led(self.leds[1].g, self.dcu_checker.status.fields.rx_aligned)

    def add_lfps_stage(self, platform, serdes_refclk, refclk_freq, channel):
        """ECP5USB3SerDes with the LFPS Unit and LFPSMonitor, no LTSSM/Link (S4: host LFPS only, no
        link training)."""
        self.usb3_serdes = serdes = ECP5USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = self.sys_clk_freq,
            refclk       = serdes_refclk,
            refclk_freq  = refclk_freq,
            tx_pads      = platform.request("usb3_tx", channel),
            rx_pads      = platform.request("usb3_rx", channel),
            dual         = 0,
            channel      = channel)
        self.lfps = lfps = LFPSUnit(serdes, self.sys_clk_freq)
        self.lfps_monitor = LFPSMonitor(serdes, lfps)
        self.los_capture  = LOSCapture(dcu_rx_los(serdes.dcu))
        self.ldr_capture  = LOSCapture(dcu_add_rx_ldr(serdes.dcu), trigger="edge")
        self.rx_monitor   = RXMonitor(serdes.dcu, self.sys_clk_freq)
        self.rx_align     = CSRStorage(reset=1, description="RX comma alignment enable.")
        self.comb += serdes.rx_align.eq(self.rx_align.storage)

        # Leds: SerDes ready, RX activity (green), Polling LFPS detected (red).
        self.led(self.leds[0].g, serdes.ready)
        self.led(self.leds[1].g, ~serdes.rx_idle)
        self.led(self.leds[2].r, lfps.rx_polling)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the LambdaConcept ECPIX-5: bring-up designs.")
    parser.add_argument("--stage",       default="serdes", help="Bring-up stage.", choices=["serdes", "dcu", "lfps"])
    parser.add_argument("--refclk",      default="fabric", help="SerDes reference clock (serdes stage, EXTREF0: 100MHz, 2.5Gbps max).", choices=["extref", "fabric"])
    parser.add_argument("--refclk-freq", default=250e6, type=float, help="SerDes reference clock frequency (fabric).")
    parser.add_argument("--dcu-mult",    default=20, type=int, help="DCU PLL multiplier (serdes stage).", choices=[8, 10, 16, 20, 25])
    parser.add_argument("--dcu-div",     default=1,  type=int, help="DCU TX/RX clock divider (serdes stage).", choices=[1, 2, 4])
    parser.add_argument("--refck-mode",  default=None, help="DCU D_REFCK_MODE override (serdes stage, ex: 0b101).")
    parser.add_argument("--i2c-bitbang", action="store_true", help="USB-C CC controller I2C bitbang CSRs (fixed polarity: --usbc-pol 0/1).")
    ecpix5_args(parser)
    parser.set_defaults(sys_clk_freq=125e6) # SerDes datapath in sys (Gen 1 32-bit words at 125MHz).
    usb3_soc_args(parser, output_dir="build/lambdaconcept_ecpix5_bringup")
    args = parser.parse_args()

    platform = lambdaconcept_ecpix5.Platform(toolchain="trellis")
    soc_args = usb3_soc_argdict(args)
    soc = BringupSoC(platform,
        sys_clk_freq     = int(args.sys_clk_freq),
        stage            = args.stage,
        channel          = args.channel,
        refclk           = args.refclk,
        refclk_freq      = args.refclk_freq,
        dcu_mult         = args.dcu_mult,
        dcu_div          = args.dcu_div,
        refck_mode       = args.refck_mode,
        serdes_params    = dict(p.split("=", 1) for p in args.serdes_param),
        usbc_pol         = args.usbc_pol,
        with_i2c_bitbang = args.i2c_bitbang,
        with_uartbone    = soc_args["with_uartbone"],
        with_jtagbone    = soc_args["with_jtagbone"],
        cpu_type         = soc_args["cpu_type"],
    )
    usb3_soc_build(parser, args, soc, seed=args.seed)

if __name__ == "__main__":
    main()
