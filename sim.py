#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc  import MultiReg

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.io import CRG
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.interconnect import stream
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.code_8b10b import Encoder, Decoder

from usb3_pipe.common   import SKP
from usb3_pipe.datapath import TXDatapath, RXDatapath, RXErrorMonitor
from usb3_pipe.phy      import USB3PHY
from usb3_core.host_model import USB3HostModel, enumeration_script
from usb3_core.common      import endpoint_layout
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1))
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    default_clk_name = "sys_clk"

    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Simulation Serializer/Deserializer Model ---------------------------------------------------------

class USB3SerDesModel(LiteXModule):
    """SerDes model: 8b/10b, TX pattern/LFPS, polarity and RX word shift emulation.

    The RX side (decoders, word aligner, SKP remover) runs in `rx_clock_domain`: the partner's TX
    clock, as recovered by the CDR of a real transceiver. The RX datapath's CDC then moves the data
    to the local sys clock domain.
    """
    def __init__(self, phy_dw=20, rx_word_shift=0, rx_clock_domain="sys"):
        assert phy_dw in [20, 40]
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.tx     = stream.Endpoint([("data", phy_dw)])
        self.rx     = stream.Endpoint([("data", phy_dw)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()       # i
        self.tx_idle     = Signal()       # i
        self.tx_pattern  = Signal(phy_dw) # i
        self.tx_lfps     = Signal()       # i

        self.rx_polarity = Signal() # i
        self.rx_idle     = Signal() # o
        self.rx_errors   = Signal() # o

        self.tx_skp_enable = Signal(reset=1) # i
        self.rx_align      = Signal()        # i
        self.rx_reset      = Signal()        # i: (Not modelled).
        self.tx_skp        = Signal()        # i: Insert a SKP Ordered Set (2 symbols) on TX.

        self.rx_idle_stretch = 0
        self.xcvr_name       = None

        # # #

        nwords = phy_dw//10
        assert 0 <= rx_word_shift <= nwords

        # Control.
        self.comb += self.ready.eq(self.enable) # Ready when enabled

        # Datapath.
        tx_datapath = TXDatapath(phy_dw=nwords*8)
        rx_datapath = RXDatapath(phy_dw=nwords*8, clock_domain=rx_clock_domain)
        self.submodules += tx_datapath, rx_datapath
        self.comb += [
            self.sink.connect(tx_datapath.sink),
            tx_datapath.skp_enable.eq(self.tx_skp_enable),
            rx_datapath.source.connect(self.source)
        ]
        self.specials += MultiReg(self.rx_align, rx_datapath.word_aligner.enable, rx_clock_domain)

        # 8b10b Encoders/Decoders.
        encoder  = Encoder(nwords, True)
        decoders = [ClockDomainsRenamer(rx_clock_domain)(Decoder(True)) for _ in range(nwords)]
        self.submodules += encoder, decoders
        self.comb += rx_datapath.sink.valid.eq(1)
        for i in range(nwords):
            self.comb += [
                rx_datapath.sink.ctrl[i].eq(decoders[i].k),
                rx_datapath.sink.data[8*i:8*(i+1)].eq(decoders[i].d),
            ]

        # TX symbols with SKP Ordered Set insertion (as transmitted by a host: a single SKP Ordered
        # Set (2 symbols) shifts the partner's received words by 2 symbols). With 4 symbols per
        # word, the stream is then shifted by 2 symbols (2 symbols carried to the next word) until
        # the next insertion, the TX datapath being held for one word every 2 insertions.
        skp_insert = Signal()
        shifted    = Signal()
        carry_d    = Signal(16)
        carry_k    = Signal(2)
        tx_d       = Signal(8*nwords)
        tx_k       = Signal(nwords)
        skp_d      = Replicate(Constant(SKP.value, 8), 2)
        src        = tx_datapath.source
        self.sync += If(self.tx_skp, skp_insert.eq(1)).Elif(skp_insert, skp_insert.eq(0))
        if nwords == 2:
            self.comb += [
                src.ready.eq(~skp_insert),
                If(skp_insert,
                    tx_d.eq(skp_d), tx_k.eq(0b11)
                ).Else(
                    tx_d.eq(src.data), tx_k.eq(src.ctrl)
                )
            ]
        else:
            self.comb += [
                If(~shifted,
                    src.ready.eq(1),
                    If(skp_insert,
                        tx_d.eq(Cat(skp_d, src.data[:16])), tx_k.eq(Cat(Constant(0b11, 2), src.ctrl[:2]))
                    ).Else(
                        tx_d.eq(src.data), tx_k.eq(src.ctrl)
                    )
                ).Else(
                    src.ready.eq(~skp_insert),
                    If(skp_insert,
                        tx_d.eq(Cat(carry_d, skp_d)), tx_k.eq(Cat(carry_k, Constant(0b11, 2)))
                    ).Else(
                        tx_d.eq(Cat(carry_d, src.data[:16])), tx_k.eq(Cat(carry_k, src.ctrl[:2]))
                    )
                )
            ]
            self.sync += [
                If(src.ready,
                    carry_d.eq(src.data[16:]),
                    carry_k.eq(src.ctrl[2:]),
                ),
                If(skp_insert, shifted.eq(~shifted)),
            ]
        for i in range(nwords):
            self.comb += [
                encoder.k[i].eq(tx_k[i]),
                encoder.d[i].eq(tx_d[8*i:8*(i+1)]),
            ]

        # Pattern/Polarity/Shift emulation.
        tx_data    = Signal(phy_dw)
        rx_data     = Signal(phy_dw)
        rx_data_sr  = Signal(2*phy_dw)
        rx_polarity = Signal()
        self.specials += MultiReg(self.rx_polarity, rx_polarity, rx_clock_domain)
        self.comb += [
            If(self.tx_lfps | (self.tx_pattern != 0),
                tx_data.eq(self.tx_pattern)
            ).Else(
                tx_data.eq(Cat(*[encoder.output[i] for i in range(nwords)])),
            ),
            If(self.tx_polarity,
                self.tx.data.eq(~tx_data)
            ).Else(
                self.tx.data.eq(tx_data)
            )
        ]
        self.comb += [
            If(rx_polarity,
                rx_data.eq(~self.rx.data)
            ).Else(
                rx_data.eq(self.rx.data)
            )
        ]
        sync_rx = getattr(self.sync, rx_clock_domain)
        sync_rx += rx_data_sr.eq(Cat(rx_data_sr[phy_dw:], rx_data)) # Previous word in LSBs.
        for i in range(nwords):
            self.comb += decoders[i].input.eq(rx_data_sr[10*(rx_word_shift+i):10*(rx_word_shift+i+1)])

        # RX decode errors.
        self.rx_error_monitor = RXErrorMonitor(decoders, clock_domain=rx_clock_domain)
        self.comb += self.rx_errors.eq(self.rx_error_monitor.errors)

    def connect(self, serdes):
        self.comb += [
            self.tx.connect(serdes.rx),
            serdes.tx.connect(self.rx),
        ]
        self.specials   += MultiReg(serdes.tx_idle, self.rx_idle)
        serdes.specials += MultiReg(self.tx_idle, serdes.rx_idle)

# Link-Layer Stand-In (PRBS Packets) ---------------------------------------------------------------

PRBS_START = 0xfb # K27.7 on all the Symbols of the word: Packet start marker.
PRBS_END   = 0xfd # K29.7 on all the Symbols of the word: Packet end marker.

def prbs_marker(symbol, data_width):
    """Word with symbol (K-code) on all its Symbols, as (data, ctrl)."""
    n = data_width//8
    return (int.from_bytes(bytes([symbol])*n, "little"), 2**n - 1)

def xorshift(x, name):
    """Next state of a 32/64-bit xorshift PRBS (combinatorial), as (statements, signal)."""
    n = len(x)
    s = {32: (13, 17, 5), 64: (13, 7, 17)}[n]
    a = Signal(n, name=name + "_a")
    b = Signal(n, name=name + "_b")
    c = Signal(n, name=name + "_c")
    return [a.eq(x ^ (x << s[0])), b.eq(a ^ (a >> s[1])), c.eq(b ^ (b << s[2]))], c

class PRBSPacketGenerator(LiteXModule):
    """Link-Layer stand-in (TX): random length packets of PRBS data words, framed by K-code start/end
    markers (first/last) and separated by random gaps of 0 to 2**gap_bits - 1 cycles (Logical Idle
    sent by the PIPE).
    """
    def __init__(self, data_width=32, gap_bits=4):
        self.enable = Signal()   # i
        self.source = source = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.words  = Signal(32) # o: Data words sent.

        # # #

        start = prbs_marker(PRBS_START, data_width)
        end   = prbs_marker(PRBS_END,   data_width)

        prbs = Signal(data_width, reset=1)
        prbs_next_stmts, prbs_next = xorshift(prbs, "prbs_gen")
        self.comb += prbs_next_stmts

        count = Signal(5)
        self.fsm = fsm = FSM(reset_state="GAP")
        fsm.act("GAP",
            If(count != 0,
                NextValue(count, count - 1)
            ).Elif(self.enable,
                NextValue(count, prbs[0:5]), # 1-32 data words.
                NextState("START")
            )
        )
        fsm.act("START",
            source.valid.eq(1),
            source.first.eq(1),
            source.data.eq(start[0]),
            source.ctrl.eq(start[1]),
            If(source.ready,
                NextState("DATA")
            )
        )
        fsm.act("DATA",
            source.valid.eq(1),
            source.data.eq(prbs),
            source.ctrl.eq(0),
            If(source.ready,
                NextValue(prbs, prbs_next),
                NextValue(self.words, self.words + 1),
                NextValue(count, count - 1),
                If(count == 0,
                    NextState("END")
                )
            )
        )
        fsm.act("END",
            source.valid.eq(1),
            source.last.eq(1),
            source.data.eq(end[0]),
            source.ctrl.eq(end[1]),
            If(source.ready,
                NextValue(count, prbs[8:8 + gap_bits]), # Gap.
                NextState("GAP")
            )
        )

class PRBSPacketChecker(LiteXModule):
    """Link-Layer stand-in (RX): checks the packets of PRBSPacketGenerator.

    Inside packets, each data word must be the PRBS successor of the previous one (self-synchronizing:
    a corrupted/lost word only gives a bounded number of errors). Outside packets, only Logical Idle
    (descrambled 00h data symbols) is expected.
    """
    def __init__(self, data_width=32):
        self.enable  = Signal()   # i
        self.sink    = sink = stream.Endpoint([("data", data_width), ("ctrl", data_width//8)])
        self.words   = Signal(32) # o: Data words checked.
        self.packets = Signal(32) # o: Packets received.
        self.errors  = Signal(32) # o: Errors.

        # # #

        start = prbs_marker(PRBS_START, data_width)
        end   = prbs_marker(PRBS_END,   data_width)

        prev      = Signal(data_width)
        prev_ok   = Signal()
        in_packet = Signal()
        prev_next_stmts, prev_next = xorshift(prev, "prbs_chk")
        self.comb += prev_next_stmts
        self.comb += sink.ready.eq(1)

        is_start = (sink.data == start[0]) & (sink.ctrl == start[1])
        is_end   = (sink.data == end[0])   & (sink.ctrl == end[1])
        self.sync += If(self.enable & sink.valid,
            If(is_start,
                If(in_packet, self.errors.eq(self.errors + 1)),
                in_packet.eq(1)
            ).Elif(is_end,
                If(~in_packet, self.errors.eq(self.errors + 1)),
                in_packet.eq(0),
                self.packets.eq(self.packets + 1)
            ).Elif(in_packet,
                self.words.eq(self.words + 1),
                prev.eq(sink.data),
                prev_ok.eq(1),
                If((sink.ctrl != 0) | (prev_ok & (sink.data != prev_next)),
                    self.errors.eq(self.errors + 1)
                )
            ).Elif((sink.data != 0) | (sink.ctrl != 0),
                self.errors.eq(self.errors + 1)
            )
        )

# Clock Generator (NCO) ----------------------------------------------------------------------------

class NCOClockGenerator(LiteXModule):
    """Clock domain generated from the sys clock by a 32-bit phase accumulator (sim only).

    The generated clock runs at ratio x the sys clock frequency (with 1 sys clock cycle of jitter).
    `ppm` offsets its frequency and `ssc` adds a 0/-5000ppm triangular down-spread at 31.5kHz (USB 3.2
    6.5.4) relative to `nominal_freq`.
    """
    def __init__(self, cd, ratio=0.25, ppm=0, ssc=False, nominal_freq=133e6):
        phase  = Signal(32)
        inc    = int(2**32*ratio*(1 + ppm*1e-6))
        offset = Signal(32)

        if ssc:
            ssc_max    = int(2**32*ratio*5000e-6)
            half_ticks = int(nominal_freq/31.5e3/2/ratio)
            ssc_step   = max(1, ssc_max//half_ticks)
            ssc_up     = Signal(reset=1)
            self.sync += [
                If(ssc_up,
                    offset.eq(offset + ssc_step),
                    If(offset >= ssc_max - ssc_step, ssc_up.eq(0))
                ).Else(
                    offset.eq(offset - ssc_step),
                    If(offset <= ssc_step, ssc_up.eq(1))
                )
            ]

        self.sync += phase.eq(phase + inc - offset)
        self.comb += [
            cd.clk.eq(phase[31]),
            cd.rst.eq(ResetSignal("sys")),
        ]

# USB3PIPESim --------------------------------------------------------------------------------------

class USB3PIPESim(SoCMini):
    """Host <-> Device USB3 PHY simulation.

    - with_core:     USB3 Device Core on the Device and scripted Host Link/Protocol Layer model
                     (link initialization, enumeration control transfers, received packets displayed
                     as "HOST_RX"), else PRBS Link-Layer stand-ins with the payload checked on both
                     sides in U0.
    - host_skp:      SKP Ordered Sets inserted by the Host between packets (with_core).
    - host_latency:  Host latency (cycles) between the SETUP ACK and the Data stage request.
    - host_status_gap: Host gap (cycles) between the Data stage ACK TP and the STATUS TP.
    - host_pm:       Host LGO_U1/LGO_U2 requests after the enumeration (with_core).
    - host_lpm:      Device Core with LPM, Host U1/U2 entries/exits after the enumeration (with_core).
    - rx_word_shift: RX shift (in 10-bit symbols) of the SerDes models.
    - ppm/ssc:       Host/Device in separate clock domains, the Device clock offset by ppm and/or with
                     SSC. Each SerDes model receives in the partner's (recovered) clock domain.
    - prbs_words:    Words to check on both sides before ending the simulation (without core).

    Gen 1x2 (lanes=2, PRBS stand-ins on 64-bit link words, see doc/gen2-x2-roadmap.md): the PHYs
    train to U0 through the SuperSpeedPlus Polling (SCD1/SCD2, LBPM), with lane-to-lane skew, per
    lane polarity (Host Lane 1 TX inverted) and SKP Ordered Sets differences:
    - lane_delays:      Lane 0/1 delays (words of 4 symbols) from Host to Device (lanes skew).
    - lane1_word_shift: Lane 1 RX word shift (symbols) of the Device SerDes model.
    - lane_skp:         Extra SKP Ordered Sets (2 Symbols) on the Host Lane 1 then Lane 0 every
                        lane_skp cycles in U0 (per lane SKP differences, 0 = disabled, >= 4096 for
                        the model).

    gen2_capable (lanes=1): Host and Device advertise Gen 2x1 in the SuperSpeedPlus Polling
    (SCD1/SCD2, LBPM), Gen 2 negotiated (no Gen 2 speed switch yet: training continues at Gen 1
    speed).
    """
    def __init__(self, phy_dw=None, timer_scale=1, with_core=True, rx_word_shift=0, ppm=0, ssc=False,
        prbs_words=2**16, max_cycles=2**24, host_skp=True, host_latency=256, host_status_gap=64, host_pm=False,
        host_lpm=False, lanes=1, lane_delays=(0, 1), lane1_word_shift=2, lane_skp=4096,
        gen2_capable=False):
        assert lanes in [1, 2]
        assert (lanes == 1) or (not with_core) # Gen 1x2: PRBS Link-Layer stand-ins only.
        sys_clk_freq = int(133e6)
        # SerDes models data width: 4 symbols per clock with the core (full PIPE rate, as on
        # hardware) and with 2 lanes, 2 with the PRBS stand-ins.
        if phy_dw is None:
            phy_dw = 40 if (with_core or lanes > 1) else 20

        # Platform.
        platform = Platform()
        self.comb += platform.trace.eq(1)

        # SoC Mini.
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq)

        # Clock Domains.
        self.crg = CRG(platform.request("sys_clk"))
        if (ppm != 0) or ssc:
            self.cd_host  = ClockDomain()
            self.cd_dev   = ClockDomain()
            self.host_crg = NCOClockGenerator(self.cd_host)
            self.dev_crg  = NCOClockGenerator(self.cd_dev, ppm=ppm, ssc=ssc)
            cds = {"host": "host", "dev": "dev"}
        else:
            cds = {"host": "sys", "dev": "sys"}

        # USB3 Host (Host = Downstream Port) / Device (Device = Upstream Port).
        phys     = {}
        ltssms   = {}
        serdeses = {}
        checkers = {}
        for name, role, partner in [("host", "host", "dev"), ("dev", "device", "host")]:
            renamer = ClockDomainsRenamer({"sys": cds[name], "rx": cds[partner]})
            serdes  = [renamer(USB3SerDesModel(
                phy_dw          = phy_dw,
                rx_word_shift   = lane1_word_shift if (name, lane) == ("dev", 1) else rx_word_shift,
                rx_clock_domain = "rx")) for lane in range(lanes)]
            phy     = renamer(USB3PHY(
                serdes               = serdes[0] if lanes == 1 else serdes,
                sys_clk_freq         = sys_clk_freq,
                role                 = role,
                timer_scale          = timer_scale,
                with_endianness_swap = (lanes == 1),
                lanes                = lanes,
                gen2_capable         = gen2_capable))
            if lanes == 1:
                self.add_module(name=f"{name}_usb3_serdes", module=serdes[0])
            else:
                for lane in range(lanes):
                    self.add_module(name=f"{name}_usb3_serdes{lane}", module=serdes[lane])
            self.add_module(name=f"{name}_usb3_phy",    module=phy)
            ltssm = phy.ltssm
            if lanes == 1:
                # Inverse TX polarity to test RX auto-polarity.
                self.comb += serdes[0].tx_polarity.eq(1)
            else:
                self.comb += serdes[1].tx_polarity.eq(name == "host") # Per lane polarity (6.13.7).

            if with_core and (name == "host"):
                # Scripted host Link/Protocol Layer (link initialization, enumeration).
                host_model = renamer(USB3HostModel(enumeration_script(skp=host_skp, latency=host_latency,
                    status_gap=host_status_gap, pm=host_pm, lpm=host_lpm).steps))
                self.add_module(name="host_model", module=host_model)
                # Script running in U0, kept running through U1/U2 and the Recovery to U0 that follows.
                host_ux = Signal()
                sync    = getattr(self.sync, cds[name])
                sync   += If(ltssm.u1 | ltssm.u2, host_ux.eq(1)).Elif(ltssm.u0, host_ux.eq(0))
                self.comb += [
                    phy.source.connect(host_model.sink),
                    host_model.source.connect(phy.sink),
                    host_model.enable.eq(ltssm.u0 | ltssm.u1 | ltssm.u2 | host_ux),
                    host_model.u0.eq(ltssm.u0),
                    ltssm.req_u1.eq(host_model.pm[0]),
                    ltssm.req_u2.eq(host_model.pm[1]),
                    ltssm.lau.eq(host_model.pm[2]),
                    ltssm.exit_u1_req.eq(host_model.pm[3]),
                    ltssm.exit_u2_req.eq(host_model.pm[4]),
                    serdes[0].tx_skp.eq(host_model.skp),
                ]
            elif with_core:
                # USB3 Device Core, Bulk loopback: EP2 OUT -> EP1 IN.
                core = renamer(USB3DeviceCore(phy, sys_clk_freq,
                    descriptors = bulk_descriptors(max_burst=8),
                    endpoints   = [USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)],
                    with_lpm    = host_lpm))
                self.add_module(name=f"{name}_usb3_core", module=core)
                loopback = renamer(stream.SyncFIFO(endpoint_layout(), 2*256, buffered=True))
                self.add_module(name=f"{name}_loopback", module=loopback)
                self.comb += [
                    core.endpoints[1].source.connect(loopback.sink),
                    loopback.source.connect(core.endpoints[0].sink),
                ]
            else:
                # Gen 1x2: gaps of 0-3 cycles (Logical Idle gaps of 4+ cycles with per lane extra
                # SKP Ordered Sets give Device PRBS errors, see doc/gen2-x2-roadmap.md).
                generator = renamer(PRBSPacketGenerator(
                    data_width = 32*lanes,
                    gap_bits   = {1: 4, 2: 2}[lanes]))
                checker   = renamer(PRBSPacketChecker(data_width=32*lanes))
                self.add_module(name=f"{name}_prbs_generator", module=generator)
                self.add_module(name=f"{name}_prbs_checker",   module=checker)
                self.comb += [
                    generator.enable.eq(ltssm.u0),
                    generator.source.connect(phy.sink),
                    checker.enable.eq(ltssm.u0),
                    phy.source.connect(checker.sink),
                ]
                checkers[name] = checker

            # LTSSM lower-layer inputs: no PM activity (link activity from the native core).
            if not (with_core and (name == "dev")):
                self.comb += ltssm.link_activity.eq(1)

            phys[name]     = phy
            ltssms[name]   = ltssm
            serdeses[name] = serdes

        # Connect Host <--> Device.
        if lanes == 1:
            serdeses["host"][0].connect(serdeses["dev"][0])
        else:
            self.add_lanes_connection(serdeses, phys, cds, lane_delays, lane_skp)

        # Simulation Timer.
        timer = Signal(32)
        self.sync += timer.eq(timer + 1)

        # Simulation Status.
        for name, ltssm in ltssms.items():
            fsm  = ltssm.fsm
            sync = getattr(self.sync, cds[name])
            for state, value in fsm.encoding.items():
                sync += [
                    If(fsm.next_state != fsm.state,
                        If(fsm.next_state == value,
                            Display("[%08d] {} entering {} state".format(
                                {"host": "HOST", "dev": "DEV "}[name], state), timer)
                        )
                    )
                ]
            if (lanes > 1) or gen2_capable:
                self.add_ssp_status(phys[name], name, sync, timer)
            if lanes > 1:
                self.add_lanes_status(phys[name], name, sync, timer)

        # Simulation End.
        done   = Signal()
        report = Signal()
        if with_core:
            # Host script done (or error), then 2^10 cycles.
            end_timer = WaitTimer(2**10)
            self.submodules += end_timer
            self.comb += [
                end_timer.wait.eq(host_model.done | host_model.error),
                done.eq(end_timer.done),
            ]
            self.sync += If(done & ~report,
                If(host_model.error,
                    Display("HOST script error")
                ).Else(
                    Display("HOST script done")
                )
            )
        else:
            # prbs_words checked on both sides.
            self.comb += done.eq(
                (checkers["host"].words >= prbs_words) &
                (checkers["dev"].words  >= prbs_words))
            for name, checker in checkers.items():
                fmt  = "{} PRBS: %d words, %d packets, %d errors".format(name.upper())
                args = [checker.words, checker.packets, checker.errors]
                if lanes > 1:
                    fmt  += ", x2: %d"
                    args += [phys[name].x2]
                self.sync += If(done & ~report,
                    Display(fmt, *args)
                )
        self.sync += [
            If(done | (timer >= max_cycles),
                report.eq(1),
                If(~done & ~report, Display("[%08d] Simulation timeout", timer))
            ),
            If(report, Finish())
        ]

    def add_lanes_connection(self, serdeses, phys, cds, lane_delays, lane_skp):
        """Gen 1x2 Host <--> Device lanes connection (Host -> Device lanes delayed)."""
        for lane in range(2):
            h = serdeses["host"][lane]
            d = serdeses["dev"][lane]
            self.comb += d.tx.connect(h.rx)
            n = lane_delays[lane]
            if n > 0:
                delay = [Signal(len(h.tx.data)) for _ in range(n)]
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

    def add_ssp_status(self, phy, name, sync, timer):
        """SuperSpeedPlus Polling states (with negotiated x2/gen2)."""
        label = {"host": "HOST", "dev": "DEV "}[name]
        phy.ssp.finalize() # Elaborate the FSM to expose its encoding.
        ssp = phy.ssp.fsm
        for state, value in ssp.encoding.items():
            sync += If((ssp.next_state != ssp.state) & (ssp.next_state == value),
                Display("[%08d] {} SSP entering {} state (x2: %d, gen2: %d)".format(label, state),
                    timer, phy.ssp.x2, phy.ssp.gen2)
            )

    def add_lanes_status(self, phy, name, sync, timer):
        """Gen 1x2 status: lanes deskew."""
        label = {"host": "HOST", "dev": "DEV "}[name]
        aligned_d = Signal()
        sync += aligned_d.eq(phy.deskew_aligned)
        sync += If(phy.deskew_aligned & ~aligned_d,
            Display("[%08d] {} lanes deskewed".format(label), timer)
        )

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3 PIPE Simulation")
    parser.add_argument("--trace",         action="store_true", help="Enable VCD tracing.")
    parser.add_argument("--trace-start",   default=0,           help="Cycle to start VCD tracing.")
    parser.add_argument("--trace-end",     default=-1,          help="Cycle to end VCD tracing.")
    parser.add_argument("--timer-scale",   default=1,           help="LTSSM/LFPS timers scale factor (1 = spec exact).")
    parser.add_argument("--no-core",       action="store_true", help="Replace the USB3 Core with PRBS Link-Layer stand-ins.")
    parser.add_argument("--prbs-words",    default=None,        help="Words to check on both sides (with --no-core, default: 2^16, 2^14 with --lanes 2).")
    parser.add_argument("--rx-word-shift", default=0,           help="SerDes models RX shift (0-3 symbols).")
    parser.add_argument("--ppm",           default=0,           help="Device clock offset vs Host clock (ppm).")
    parser.add_argument("--ssc",           action="store_true", help="Enable SSC (0/-5000ppm down-spread) on the Device clock.")
    parser.add_argument("--max-cycles",    default=2**24,       help="Simulation timeout (sys clock cycles).")
    parser.add_argument("--no-host-skp",   action="store_true", help="No SKP Ordered Sets inserted by the Host model.")
    parser.add_argument("--host-latency",  default=256,         help="Host model latency before the Data stage requests (cycles).")
    parser.add_argument("--host-status-gap", default=64,        help="Host model gap between the Data stage ACK TP and the STATUS TP (cycles).")
    parser.add_argument("--host-pm",       action="store_true", help="Host model U1/U2 requests (LGO_U1/LGO_U2) after the enumeration.")
    parser.add_argument("--host-lpm",      action="store_true", help="Device Core with LPM, Host model U1/U2 entries/exits after the enumeration.")
    parser.add_argument("--lanes",         default=1,           help="Lanes (1: Gen 1x1, 2: Gen 1x2 with PRBS Link-Layer stand-ins).", choices=["1", "2"])
    parser.add_argument("--lane0-delay",   default=0,           help="Host to Device Lane 0 delay (words, --lanes 2).")
    parser.add_argument("--lane1-delay",   default=1,           help="Host to Device Lane 1 delay (words, --lanes 2).")
    parser.add_argument("--lane1-word-shift", default=2,        help="Device Lane 1 RX word shift (symbols, --lanes 2).")
    parser.add_argument("--lane-skp",      default=4096,        help="Extra Host Lane 1/Lane 0 SKP Ordered Sets period (cycles, 0: disabled, --lanes 2).")
    parser.add_argument("--gen2-capable",  action="store_true", help="Gen 2x1 capability advertised/negotiated in the SuperSpeedPlus Polling (--lanes 1).")
    parser.add_argument("--output-dir",    default="build/sim", help="Build directory.")
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")

    lanes      = int(args.lanes)
    with_core  = (not args.no_core) and (lanes == 1)
    prbs_words = args.prbs_words or {1: 2**16, 2: 2**14}[lanes]

    soc = USB3PIPESim(
        timer_scale   = int(args.timer_scale),
        with_core     = with_core,
        rx_word_shift = int(args.rx_word_shift),
        ppm           = float(args.ppm),
        ssc           = args.ssc,
        prbs_words    = int(prbs_words),
        max_cycles    = int(args.max_cycles),
        host_skp      = not args.no_host_skp,
        host_latency  = int(args.host_latency),
        host_status_gap = int(args.host_status_gap),
        host_pm       = args.host_pm,
        host_lpm      = args.host_lpm,
        lanes            = lanes,
        lane_delays      = (int(args.lane0_delay), int(args.lane1_delay)),
        lane1_word_shift = int(args.lane1_word_shift),
        lane_skp         = int(args.lane_skp),
        gen2_capable     = args.gen2_capable)
    builder = Builder(soc, output_dir=args.output_dir)
    builder.build(sim_config=sim_config,
        opt_level   = "O0",
        interactive = False,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))

if __name__ == "__main__":
    main()
