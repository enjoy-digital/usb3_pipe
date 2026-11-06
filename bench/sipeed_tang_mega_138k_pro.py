#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3-PIPE on the Sipeed Tang Mega 138K Pro (Gowin GW5AST-138B GTR12).

SFP2USB adapter in SFP0 (GTR12 Q1 lane 0; --sfp 1: lane 1), GTR12 reference clock from the dock
(--refclk: 100MHz REFCLK1 or 125MHz REFCLK0; --pll: QPLL1 or CPLL). Gen 1x1 (5Gbps) validated,
Gen 2x1 (10Gbps) in bring-up: --gen2-capable --pll CPLL --refclk 125M (doc/targets.md).

Clocking (slow fabric): the PIPE and the SerDes datapath run on the GTR12 TX clock at 125MHz, the
LTSSM/Link Layer/Core/SoC in sys at 62.5MHz (--sys-clk-freq, USB3PHY pipe_cd) on 64-bit Link Layer
words (--link-words 4: 128-bit).

Bridge: UARTBone by default, JTAGBone with --with-jtagbone. The bitstream (.fs) is loaded with
openFPGALoader (--load is not usable: it expects a .bit).

Build: ./bench/sipeed_tang_mega_138k_pro.py --build [--example <name>]. Shared options:
bench/usb3_soc.py; bring-up options: --help.
"""

import argparse

import common # Repository root on the path.

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litex.soc.interconnect           import stream
from litex.soc.interconnect.csr       import *
from litex.soc.cores.clock.gowin_gw5a import GW5APLL

from usb3_pipe          import GW5USB3SerDes
from usb3_pipe.common   import COM, K, BLOCK_HEADER_DATA
from usb3_pipe.datapath import Gen2RX, Gen2RXDatapath, GEN2_IDLE_BLOCK, GEN2_RX_COUNTERS
from usb3_pipe.framing  import Gen2LinkRX, SHP, SLC
from usb3_pipe.debug    import Gen2RXDebug
from usb3_pipe.crc      import CRC16Words, CRC5

from usb3_soc import USB3BaseSoC, add_usb3_device, usb3_analyzer, CountSync
from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain()

        # # #

        # Clk50 / Power on reset (the GW5A PLLs need a long reset after power-up).
        clk50 = platform.request("clk50")
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += [
            self.cd_por.clk.eq(clk50),
            por_done.eq(por_count == 0),
        ]
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL.
        self.pll = pll = GW5APLL(devicename=platform.devicename, device=platform.device, name="pll")
        pll.vco_freq_range = (650e6, 1300e6) # GW5AST-138 PLL limits (Gowin UG306, section 2.3).
        self.comb += pll.reset.eq(~por_done | self.rst)
        pll.register_clkin(clk50, 50e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.add_generated_clock_constraints(platform) # Named PLL clocks (false paths).

class USB3SoC(USB3BaseSoC):
    def __init__(self, platform, sys_clk_freq=62.5e6, sfp=0, refclk="100M", pll=None,
        serdes_csr=None, rx_idle_source="~astat5", with_upar=True, with_gtr12_debug=False,
        with_gtr12_analyzer=False, with_analyzer=False, analyzer_group="link", gen2_capable=False,
        with_gen2_debug=False, with_trace=False, with_tx_check=False, with_tx_raw_check=False,
        with_ltssm_history=False, analyzer_csv="analyzer.csv", usb3_kwargs={}, **kwargs):
        # USB3 PIPE/SerDes at 125MHz (Gen 1 32-bit words); with a lower sys_clk_freq, the PIPE runs
        # on the GTR12 TX fabric clock ("tx" clock domain: frequency-locked to the transmitted
        # words, no TX CDC FIFO underflow) and the LTSSM/Core/SoC in sys with 64-bit Link Layer
        # words (the Core doesn't reach 125MHz on the GW5A fabric).
        usb3_clk_freq = 125e6
        phy_cd        = None if sys_clk_freq == usb3_clk_freq else "tx"

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # USB3BaseSoC ------------------------------------------------------------------------------
        USB3BaseSoC.__init__(self, platform, sys_clk_freq, **kwargs)

        # SFP --------------------------------------------------------------------------------------
        sfp_pads = platform.request("sfp", sfp)
        self.comb += sfp_pads.tx_disable.eq(0)

        # USB3 SerDes ------------------------------------------------------------------------------
        usb3_serdes = GW5USB3SerDes(platform,
            sys_clk        = self.crg.cd_sys.clk,
            sys_clk_freq   = usb3_clk_freq,
            lane           = sfp,
            rx_idle_source = rx_idle_source,
            with_upar      = with_upar,
            upar_clk       = self.crg.cd_sys.clk, # Quad AHB/UPAR clock running from the start.
            xcvr_cd        = "sys" if phy_cd is None else "gtr12",
            tx_cdc         = phy_cd != "tx", # SerDes logic on the GTR12 TX clock: no TX CDC.
            refclk         = refclk,
            pll            = pll,
            xcvr_clk_freq  = sys_clk_freq if phy_cd is not None else None, # gtr12 = sys.
            csr_config     = None if serdes_csr is None else open(serdes_csr).read(),
            gen2_capable   = gen2_capable)
        if phy_cd is not None:
            # GTR12 control logic on sys (free-running), SerDes datapath on the GTR12 TX clock.
            self.cd_gtr12 = ClockDomain()
            self.comb += [
                self.cd_gtr12.clk.eq(self.crg.cd_sys.clk),
                self.cd_gtr12.rst.eq(self.crg.cd_sys.rst),
            ]
            usb3_serdes = ClockDomainsRenamer({"sys": phy_cd})(usb3_serdes)
            # Link Layer words: pairs (64-bit, default) or quads (128-bit, --link-words 4: Gen 2 RX
            # at 16 bytes per cycle).
            link_words  = max(usb3_kwargs.get("link_words", 1), 2)
            usb3_kwargs = dict(usb3_kwargs,
                phy_cd       = phy_cd,
                phy_clk_freq = usb3_clk_freq,
                link_words   = link_words,
                with_profile = False,
                with_trace   = with_gtr12_debug or with_trace)

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        add_usb3_device(self, usb3_serdes,
            leds          = [platform.request("led_n", i) for i in range(3)],
            leds_inverted = True,
            **usb3_kwargs)

        # GTR12 bring-up: lane status, SerDes registers access (UPAR).
        gtr12 = usb3_serdes.gtr12
        gtr12.add_status_csr()
        # UPAR on the sys clock.
        if with_upar and ((phy_cd is None) or with_gtr12_debug or gen2_capable):
            gtr12.upar.add_csr()

        # Bring-up debug: GTR12 status transitions, RX snapshots (the LTSSM history, the PIPE
        # counters and the PHY reset are in the standard USB3PHYDebug: usb3_debug_*).
        if with_gtr12_debug:
            self.add_gtr12_transitions(gtr12)
        if with_gtr12_debug and phy_cd is not None and hasattr(self, "usb3_phy"):
            self.add_rx_snapshots(gtr12, usb3_serdes, phy_cd, self.usb3_phy.pipe)
        elif with_ltssm_history:
            self.add_lgood_latency()
            self.add_gen2_rx_probes(self.usb3_phy)
            self.add_gtr12_rx_valid_probe(gtr12)
        if with_gtr12_debug and phy_cd is not None and hasattr(self, "usb3_phy"):
            if hasattr(self.usb3_phy, "ssp"):
                self.add_ssp_status(self.usb3_phy.ssp, usb3_serdes, phy_cd)

        # Gen 2 RX flow control (runtime-adjustable): Type 2 credits held by the host (4: no limit),
        # RX words buffer throttle level (entries of 32 bytes).
        phy = getattr(self, "usb3_phy", None)
        if phy is not None and isinstance(phy.rx_t2_credits, Signal):
            self.usb3_rx_flow = CSRStorage(fields=[
                CSRField("t2_credits",     size=3, offset=0, reset=4,  description="Type 2 credits held by the host (4: no limit)."),
                CSRField("throttle_level", size=8, offset=8, reset=16, description="RX throttle level (32-byte entries)."),
            ])
            self.comb += [
                phy.rx_t2_credits.eq(self.usb3_rx_flow.fields.t2_credits),
                phy.gen2_rx_throttle_level.eq(self.usb3_rx_flow.fields.throttle_level),
            ]

        # Gen 2 TX FFE (runtime, applied at the next 10Gbps rate change), RX CTLE and TX
        # re-initialization (the PHY reset is usb3_debug_phy_reset).
        if with_ltssm_history and hasattr(usb3_serdes, "tx_ffe_manual"):
            self.usb3_tx_ffe = CSRStorage(fields=[
                CSRField("txlev",  size=4, offset=0,  reset=14, description="TX amplitude."),
                CSRField("c1",     size=4, offset=4,  reset=11, description="TX post-cursor."),
                CSRField("cm",     size=4, offset=8,  reset=0,  description="TX pre-cursor."),
                CSRField("manual", size=1, offset=12, reset=0,  description="Manual TX FFE (else defaults)."),
            ])
            self.usb3_tx_reinit = CSRStorage(
                description="SerDes TX re-initialization (level: TX PCS reset, no retraining).")
            self.usb3_rx_eq = CSRStorage(fields=[
                CSRField("att",    size=4, offset=0,  reset=8,  description="RX CTLE attenuation (10Gbps)."),
                CSRField("boost",  size=4, offset=4,  reset=12, description="RX CTLE boost (10Gbps)."),
                CSRField("manual", size=1, offset=8,  reset=0,  description="Manual RX CTLE (else the configuration values)."),
            ])
            self.comb += [
                usb3_serdes.rx_eq_att.eq(self.usb3_rx_eq.fields.att),
                usb3_serdes.rx_eq_boost.eq(self.usb3_rx_eq.fields.boost),
                usb3_serdes.rx_eq_manual.eq(self.usb3_rx_eq.fields.manual),
            ]
            self.comb += [
                usb3_serdes.tx_ffe_txlev.eq(self.usb3_tx_ffe.fields.txlev),
                usb3_serdes.tx_ffe_c1.eq(self.usb3_tx_ffe.fields.c1),
                usb3_serdes.tx_ffe_cm.eq(self.usb3_tx_ffe.fields.cm),
                usb3_serdes.tx_ffe_manual.eq(self.usb3_tx_ffe.fields.manual),
                usb3_serdes.tx_reinit_req.eq(self.usb3_tx_reinit.storage),
            ]

        # Gen 2 TX self-checks: transmitted Header Packets/Link Commands CRCs (Data Blocks payloads,
        # raw TX words).
        if with_tx_check:
            self.add_gen2_tx_check(self.usb3_phy)
        if with_tx_raw_check and gen2_capable:
            self.add_gen2_tx_raw_check(self.usb3_phy)

        # Gen 2 bring-up debug: RX monitor counters/status.
        if with_gen2_debug:
            self.add_gen2_rx_monitor(usb3_serdes)

        # GTR12 RX analyzer (raw RX words, rx clock domain).
        if with_gtr12_analyzer:
            from litescope import LiteScopeAnalyzer
            self.gtr12_analyzer = LiteScopeAnalyzer([
                    gtr12.rx_raw,
                    gtr12.rx_valid,
                    gtr12.aligned,
                    gtr12.cdr_lock,
                    gtr12.rx_elecidle,
                    gtr12.astat,
                ] + [d.k for d in gtr12.decoders] + [d.d for d in gtr12.decoders],
                depth        = 1024,
                clock_domain = "rx",
                register     = True,
                csr_csv      = analyzer_csv)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            self.analyzer = usb3_analyzer(self, analyzer_group, analyzer_csv)

    def add_indexed_status(self, name, value):
        """Wide (debug) value read 32-bit word by word: <name>_sel (word index), <name>_data (word,
        registered): small CSR bus read mux (timings)."""
        words = (len(value) + 31)//32
        sel   = CSRStorage(bits_for(max(words - 1, 1)), name=f"{name}_sel",
            description=f"{name}: 32-bit word index (0-{words - 1}).")
        data  = CSRStatus(32, name=f"{name}_data",
            description=f"{name}: selected 32-bit word (layout: the add_* docstrings of "
            "bench/sipeed_tang_mega_138k_pro.py).")
        setattr(self, f"_{name}_sel",  sel)
        setattr(self, f"_{name}_data", data)
        cases = {i: data.status.eq(value[32*i:min(32*(i+1), len(value))]) for i in range(words)}
        self.sync += Case(sel.storage, cases)

    def add_gen2_tx_check(self, phy):
        """Gen 2 TX self-check: the transmitted Data Blocks payloads (Gen2TX input, tx clock domain,
        before the SKP Ordered Sets/scrambling) decoded by a second Gen 2 RX framing, the Header
        Packets CRC-16/CRC-5 and Link Commands CRC-5 checked (bad packets from the TX logic vs
        corrupted on the line: host LBADs)."""
        gp        = phy.gen2_pipe
        blocks    = gp.tx_blocks
        stream_on = Signal()
        self.sync.tx += stream_on.eq(gp.tx_ready & phy.gen2_active_pipe)
        self.add_gen2_words_check("usb3_tx_check", blocks.valid & blocks.ready, blocks.data,
            stream_on)

    def add_gen2_tx_raw_check(self, phy):
        """Gen 2 TX blocks self-check: the transmitted blocks (Gen2TX output after the SKP Ordered
        Sets insertion, the DC Balance and the scrambling: gearbox input, tx clock domain) decoded
        by a second Gen 2 RX (descrambling, Ordered Sets) and RX framing, the Header Packets
        CRC-16/CRC-5 and Link Commands CRC-5 checked (bad packets from the TX logic/timings vs
        corrupted on the line or at the host: host LBADs). tx_raw_counts: Data Blocks, SKP Ordered
        Sets, control blocks, SYNC/SDS/unknown control blocks (32-bit, quasi-static reads).
        The raw words (gearbox output) are not checked: a second block aligner does not fit the tx
        clock domain timings."""
        blocks = phy.gen2_pipe.tx.gearbox.sink
        # Tap registered (timings: fanout of the gearbox input registers).
        accept = Signal()
        header = Signal(4)
        data   = Signal(128)
        skp    = Signal()
        self.sync.tx += [
            accept.eq(blocks.valid & blocks.ready),
            header.eq(blocks.header),
            data.eq(blocks.data[:128]),
            skp.eq(blocks.skp),
        ]
        rx = ClockDomainsRenamer("tx")(Gen2RX())
        self.usb3_tx_raw_rx = rx
        self.comb += [
            rx.sink.valid.eq(accept & ~skp),
            rx.sink.header.eq(header),
            rx.sink.data.eq(data),
            rx.source.ready.eq(1),
        ]
        # Data Blocks to the sys clock domain (RX framing at the Link Layer rate: OUT traffic), the
        # Logical Idle blocks dropped except the one following a non idle block (Idle Symbols inside
        # a packet still cut it).
        idle   = Signal()
        idle_d = Signal(reset=1)
        self.comb += idle.eq(rx.source.data == GEN2_IDLE_BLOCK)
        self.sync.tx += If(rx.source.valid, idle_d.eq(idle))
        fifo = ClockDomainsRenamer({"write": "tx", "read": "sys"})(
            stream.AsyncFIFO([("data", 128)], 512, buffered=True))
        self.usb3_tx_raw_fifo = fifo
        overflow    = Signal()
        data_stream = Signal()
        self.comb += [
            fifo.sink.valid.eq(rx.source.valid & ~(idle & idle_d)),
            fifo.sink.data.eq(rx.source.data),
            fifo.source.ready.eq(1),
        ]
        self.sync.tx += If(fifo.sink.valid & ~fifo.sink.ready, overflow.eq(1))
        self.specials += MultiReg(rx.data_stream, data_stream)
        self.add_gen2_words_check("usb3_tx_raw_check", fifo.source.valid, fifo.source.data,
            data_stream, clock_domain="sys", overflow=overflow)
        events = [
            accept & ~skp & (header == BLOCK_HEADER_DATA),
            accept &  skp,
            accept & ~skp & (header != BLOCK_HEADER_DATA),
            rx.rx_sync,
            rx.rx_sds,
            rx.unknown,
        ]
        counts = []
        for e in events:
            c  = Signal(32)
            cs = Signal(32)
            self.sync.tx += If(e, c.eq(c + 1))
            self.submodules += CountSync(c, cs, "tx")
            counts.append(cs)
        self.add_indexed_status("tx_raw_counts", Cat(*counts))

    def add_gen2_words_check(self, name, valid, data, data_stream, clock_domain="tx",
        overflow=None):
        """Gen 2 Data Blocks payloads (valid/data/data_stream, clock_domain) decoded by a Gen 2 RX
        framing (32-bit words), the Header Packets CRC-16/CRC-5 and Link Commands CRC-5 checked
        (<name>_hp/_lc/_status CSRs, overflow: input buffer overflow (clock_domain, sticky), with
        the blocks buffer one). Words consumed at the sys rate: OUT traffic (the blocks buffer
        overflows at the IN rate)."""
        checker = ClockDomainsRenamer({"rx": clock_domain})(Gen2LinkRX(
            buffer_depth = 1024,
            link_words   = 1,
            nbytes       = 32))
        self.add_module(f"{name}er", checker)
        self.comb += [
            checker.sink.valid.eq(valid),
            checker.sink.data.eq(data),
            checker.data_stream.eq(data_stream),
            checker.rx_ready.eq(1),
            checker.lc_enable.eq(0),
            checker.source.ready.eq(1),
            checker.source_lc.ready.eq(1),
        ]
        words = checker.source
        crc16 = CRC16Words()
        crc5a = CRC5()
        crc5b = CRC5()
        self.submodules += crc16, crc5a, crc5b
        hp, hp_bad, lc, lc_bad, cut = [Signal(16) for _ in range(5)]
        n          = Signal(2)
        ok16       = Signal()
        hp_bad_now = Signal()
        self.comb += [
            crc16.data.eq(words.data),
            crc5a.data.eq(words.data[16:27]),
            crc5b.data.eq(words.data[0:11]),
            ok16.eq((crc16.value == words.data[0:16]) & (crc5a.value == words.data[27:32])),
        ]
        start_hp = words.valid & (words.ctrl == 0b1111) & (words.data[0:8] == SHP)
        start_lc = words.valid & (words.ctrl == 0b1111) & (words.data[0:8] == SLC)
        fsm = FSM(reset_state="IDLE")
        self.add_module(f"{name}_fsm", fsm)
        fsm.act("IDLE",
            crc16.clear.eq(1),
            If(start_hp, NextValue(n, 0), NextState("HP")).Elif(start_lc, NextState("LC"))
        )
        fsm.act("HP",
            If(words.valid,
                If(words.ctrl != 0,
                    NextValue(cut, cut + 1),
                    NextState("IDLE")
                ).Elif(n != 3,
                    crc16.ce.eq(1),
                    NextValue(n, n + 1)
                ).Else(
                    NextValue(hp, hp + 1),
                    If(~ok16, NextValue(hp_bad, hp_bad + 1), hp_bad_now.eq(1)),
                    NextState("IDLE")
                )
            )
        )
        # Snapshot of the first bad Header Packet since clear (corruption pattern): its 4 words and
        # the 4 words before (newest first: data words then their ctrl nibbles), <name>_snap.
        hist       = [Signal(36) for _ in range(8)]
        snap       = [Signal(36) for _ in range(8)]
        taken      = Signal()
        snap_clear = CSR(name=f"{name}_snap_clear") # Write: capture the next bad Header Packet.
        setattr(self, f"{name}_snap_clear", snap_clear)
        self.sync += [
            If(words.valid,
                hist[0].eq(Cat(words.data, words.ctrl)),
                *[hist[i].eq(hist[i-1]) for i in range(1, 8)],
            ),
            If(hp_bad_now & ~taken,
                taken.eq(1),
                snap[0].eq(Cat(words.data, words.ctrl)),
                *[snap[i].eq(hist[i-1]) for i in range(1, 8)],
            ),
            If(snap_clear.re, taken.eq(0)),
        ]
        self.add_indexed_status(f"{name}_snap",
            Cat(*[s[0:32] for s in snap], *[s[32:36] for s in snap]))
        fsm.act("LC",
            If(words.valid,
                If(words.ctrl != 0,
                    NextValue(cut, cut + 1),
                ).Else(
                    NextValue(lc, lc + 1),
                    If((crc5b.value != words.data[11:16]) | (crc5a.value != words.data[27:32]),
                        NextValue(lc_bad, lc_bad + 1)),
                ),
                NextState("IDLE")
            )
        )
        overflows = Signal(2)
        self.specials += MultiReg(Cat(checker.overflow, 0 if overflow is None else overflow),
            overflows)
        overflow = overflows != 0
        csr_hp     = CSRStatus(32, name=f"{name}_hp",     description="TX check: Header Packets [15:0], CRC errors [31:16].")
        csr_lc     = CSRStatus(32, name=f"{name}_lc",     description="TX check: Link Commands [15:0], CRC errors [31:16].")
        csr_status = CSRStatus(32, name=f"{name}_status", description="TX check: packets cut by framing [15:0], blocks buffer overflow [16].")
        for csr in [csr_hp, csr_lc, csr_status]:
            setattr(self, csr.name, csr)
        self.comb += [
            csr_hp.status.eq(Cat(hp, hp_bad)),
            csr_lc.status.eq(Cat(lc, lc_bad)),
            csr_status.status.eq(Cat(cut, overflow)),
        ]

    def add_gen2_rx_probes(self, phy):
        """Gen 2 RX backlog probes (bring-up): words buffer max level (32-byte entries), cycles with
        the RX framing output stalled (words buffer full), blocks pairs buffer overflow (sticky),
        cleared by gen2_rx_probes_clear."""
        lrx = phy.gen2_pipe.link_rx
        self._gen2_rx_probes_clear = clear = CSR() # Write: clear the max level/stalls.
        max_level = Signal(16)
        stalls    = Signal(32)
        overflow  = Signal()
        out = lrx.framing.source
        self.specials += MultiReg(lrx.overflow, overflow)
        self.sync += [
            If(lrx.level > max_level, max_level.eq(lrx.level)),
            If(out.valid & ~out.ready & (stalls != 2**32 - 1), stalls.eq(stalls + 1)),
            If(clear.re, max_level.eq(0), stalls.eq(0)),
        ]
        self.add_indexed_status("gen2_rx_probes", Cat(max_level, C(0, 15), overflow, stalls))
        # PHY Gen 2 RX datapath counters (GEN2_RX_COUNTERS order: blocks, header errors (corrected
        # single bit errors: line BER), alignment errors, SYNC/TSEQ/TS1/TS2/SDS/SKP Ordered Sets,
        # unknown control blocks), rx clock domain, quasi-static reads.
        counts = []
        for name, _ in GEN2_RX_COUNTERS:
            c = Signal(32)
            self.submodules += CountSync(getattr(phy.gen2_pipe.rx, f"{name}_count"), c, "rx")
            counts.append(c)
        self.add_indexed_status("gen2_rx_counts", Cat(*counts))
        # PHY Gen 2 RX framing (sys): errors (unknown packet starts), corrected starts (RESYNC), and
        # the bytes of the first unknown packet start since gen2_rx_error_clear (the skipped byte
        # first).
        fr = phy.gen2_pipe.rx_framing
        self.add_indexed_status("gen2_rx_framing", Cat(fr.errors, fr.corrected))
        self._gen2_rx_error_clear = CSR() # Write: capture the next unknown packet start.
        error_window = Signal(len(fr.dbg_error_window))
        error_taken  = Signal()
        self.sync += [
            If(fr.dbg_error & ~error_taken,
                error_taken.eq(1),
                error_window.eq(fr.dbg_error_window),
            ),
            If(self._gen2_rx_error_clear.re, error_taken.eq(0)),
        ]
        self.add_indexed_status("gen2_rx_error", Cat(error_window, error_taken))

    def add_gtr12_rx_valid_probe(self, gtr12):
        """GTR12 RX interface probe (bring-up): rx clock cycles with RX_VLD_OUT low (the raw RX
        words are consumed every cycle: an invalid word would be a duplicated/stale word), 32-bit,
        quasi-static reads."""
        count = Signal(32)
        self.sync.rx += If(~gtr12.rx_valid, count.eq(count + 1))
        self.usb3_gtr12_rx_invalid = CSRStatus(32,
            description="GTR12 RX_VLD_OUT low cycles (rx clock).")
        self.submodules += CountSync(count, self.usb3_gtr12_rx_invalid.status, "rx")

    def add_lgood_latency(self):
        """Header Packets acknowledgement latency (bring-up): durations of the PENDING_HP_TIMER runs
        (Header Packets sent, no LGOOD_n received), max and histogram (16 bins of 64 cycles, last:
        960+ cycles), 16-bit saturating, cleared by lgood_latency_clear."""
        link = self.usb3_core.link
        self._lgood_latency_clear = clear = CSR() # Write: clear the max and the histogram.
        wait = link.t_pending_hp.wait
        cnt  = Signal(16)
        maxv = Signal(16)
        hist = [Signal(16) for _ in range(16)]
        bin_ = Signal(4)
        self.comb += bin_.eq(Mux(cnt >= 960, 15, cnt[6:10]))
        self.sync += [
            If(wait,
                If(cnt != 2**16 - 1, cnt.eq(cnt + 1))
            ).Elif(cnt != 0,
                cnt.eq(0),
                If(cnt > maxv, maxv.eq(cnt)),
                Case(bin_, {i: If(hist[i] != 2**16 - 1, hist[i].eq(hist[i] + 1))
                    for i in range(16)}),
            ),
            If(clear.re, maxv.eq(0), *[h.eq(0) for h in hist]),
        ]
        self.add_indexed_status("lgood_latency", Cat(maxv, C(0, 16), *hist))

    def add_rx_snapshots(self, gtr12, serdes, cd, pipe=None):
        """RX snapshots (bring-up, captured registers static when read): 8 cycles of GTR12 raw words
        and decoders outputs from a K28.5 in the raw words (rx clock domain), 16 SerDes RX words
        (cd clock domain). Re-armed on each rx_snapshot_arm write."""
        self._rx_snapshot_arm = CSR() # Write: re-arm the snapshots (and the TX ring).
        def snapshot(domain, n, data, trigger):
            arm = PulseSynchronizer("sys", domain)
            self.submodules += arm
            self.comb += arm.i.eq(self._rx_snapshot_arm.re)
            run   = Signal()
            count = Signal(max=n + 1)
            regs  = [Signal(len(data)) for _ in range(n)]
            sync  = getattr(self.sync, domain)
            # Registered data/trigger (timings).
            data_r    = Signal(len(data))
            trigger_r = Signal()
            sync += [data_r.eq(data), trigger_r.eq(trigger)]
            data, trigger = data_r, trigger_r
            sync += [
                If(arm.o,
                    run.eq(0), count.eq(0),
                ).Elif(~run & (count == 0) & trigger,
                    run.eq(1),
                ),
                If((run | ((count == 0) & trigger)) & (count != n),
                    Case(count, {i: regs[i].eq(data) for i in range(n)}),
                    count.eq(count + 1),
                ),
            ]
            return Cat(*regs)
        raw = gtr12.rx_raw
        k285 = Reduce("OR", [(raw[10*i:10*(i+1)] == 0x17c) | (raw[10*i:10*(i+1)] == 0x283)
            for i in range(4)])
        dec = Cat(raw, *[Cat(d.d, d.k, d.invalid) for d in gtr12.decoders])
        rx_regs = snapshot("rx", 8, dec, k285)
        src = serdes.source
        sw_regs = snapshot(cd, 16, Cat(src.data, src.ctrl, src.valid), 1)
        # 8x (raw[40], 4x (d[8], k, invalid)) / 16x (data[32], ctrl[4], valid).
        self.add_indexed_status("rx_snapshot_raw",   rx_regs)
        self.add_indexed_status("rx_snapshot_words", sw_regs)

        # TX snapshot (cd = PIPE clock domain = GTR12 TX): 32 cycles of SerDes TX words and 8b/10b
        # encoders outputs from the end of the Training Sequences transmission.
        if pipe is not None:
            tx_active_d = Signal()
            sync = getattr(self.sync, cd)
            sync += tx_active_d.eq(pipe.ts.tx_active)
            ts_end = tx_active_d & ~pipe.ts.tx_active
            snk = serdes.sink
            tx_regs = snapshot(cd, 32, Cat(snk.data, snk.ctrl, snk.valid & snk.ready,
                *gtr12.encoder.output), ts_end)
            # 32x (data[32], ctrl[4], accepted, 4x encoder output[10]).
            self.add_indexed_status("tx_snapshot", tx_regs)

            # TX ring: last 64 PIPE TX words (Link Layer words accepted by the PIPE in U0), frozen
            # on the PIPE U0 exit (tx_ready falling), re-armed with rx_snapshot_arm.
            arm = PulseSynchronizer("sys", cd)
            self.submodules += arm
            self.comb += arm.i.eq(self._rx_snapshot_arm.re)
            psnk     = pipe.sink
            ring     = [Signal(36) for _ in range(64)]
            frozen   = Signal()
            ready_d  = Signal()
            sync += [
                ready_d.eq(pipe.tx_ready),
                If(arm.o, frozen.eq(0)).Elif(ready_d & ~pipe.tx_ready, frozen.eq(1)),
                If(~frozen & psnk.valid & psnk.ready,
                    ring[0].eq(Cat(psnk.data, psnk.ctrl)),
                    *[ring[i].eq(ring[i-1]) for i in range(1, 64)]
                ),
            ]
            # 64x (data[32], ctrl[4]), newest first.
            self.add_indexed_status("tx_ring", Cat(*ring))

    def add_ssp_status(self, ssp, serdes, cd):
        """SuperSpeedPlus Polling status (bring-up): SSPPolling state, last received LFPS
        burst/period (sys cycles), received bursts/SCD1/SCD2/plain Polling LFPS counters,
        transmitted bursts counters (SSPPolling, SerDes in cd)."""
        fe      = ssp.frontend
        burst   = Signal(16)
        period  = Signal(16)
        def edge(s):
            s_d = Signal()
            self.sync += s_d.eq(s)
            return s & ~s_d
        events  = [edge(fe.burst_end), edge(ssp.scd_chk.scd1), edge(ssp.scd_chk.scd2),
            edge(ssp.scd_chk.plain), edge(ssp.tx_lfps)]
        counts  = []
        for e in events:
            c = Signal(16)
            self.sync += If(e, c.eq(c + 1))
            counts.append(c)
        # Transmitted bursts at the SerDes (cd).
        tx_lfps_d    = Signal()
        tx_count     = Signal(16)
        tx_count_sys = Signal(16)
        sync = getattr(self.sync, cd)
        sync += [
            tx_lfps_d.eq(serdes.tx_lfps),
            If(serdes.tx_lfps & ~tx_lfps_d, tx_count.eq(tx_count + 1)),
        ]
        self.specials += MultiReg(tx_count, tx_count_sys)
        counts.append(tx_count_sys)
        self.sync += [
            If(fe.burst_end,   burst.eq(fe.burst)),
            If(fe.burst_start, period.eq(fe.period)),
        ]
        # state[16], burst[16], period[16], bursts/SCD1/SCD2/plain/SSP TX/SerDes TX counts (16-bit).
        self.add_indexed_status("ssp_status",
            Cat(ssp.state, Constant(0, 12), burst, period, *counts))

    def add_gen2_rx_monitor(self, serdes):
        """Gen 2 RX monitor (bring-up): Gen2RXDatapath on the raw RX words in a reset-less rx clock
        domain (kept across the rate changes: counters accumulated over the 10Gbps periods), with
        its debug CSRs (usb3_gen2_rx_*: alignment status, blocks/Ordered Sets counters)."""
        self.cd_rx_nr = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_nr.clk.eq(ClockSignal("rx"))
        gen2_rx = ClockDomainsRenamer("rx_nr")(Gen2RXDatapath(data_width=len(serdes.gen2_rx_data)))
        self.usb3_gen2_rx_datapath = gen2_rx
        self.comb += [
            gen2_rx.sink.valid.eq(1),
            gen2_rx.sink.data.eq(serdes.gen2_rx_data),
            gen2_rx.source.ready.eq(1),
        ]
        self.usb3_gen2_rx = Gen2RXDebug(gen2_rx, clock_domain="rx_nr")

    def add_gtr12_transitions(self, gtr12):
        """Transitions counters of the GTR12 lane status bits (rx_elecidle, astat[0:6], stat[0:13]),
        sampled in the reset-less RX clock domain: LFPS/Electrical Idle indicators
        identification."""
        self.cd_rx_nr = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_nr.clk.eq(ClockSignal("rx"))
        bits = ([gtr12.rx_elecidle] + [gtr12.astat[i] for i in range(6)] +
            [gtr12.stat[i] for i in range(13)])
        counts = []
        for b in bits:
            b_r = Signal()
            b_d = Signal()
            c   = Signal(16)
            self.sync.rx_nr += [
                b_r.eq(b),
                b_d.eq(b_r),
                If(b_r != b_d, c.eq(c + 1)),
            ]
            cs = Signal(16)
            self.specials += MultiReg(c, cs)
            counts.append(cs)
        # Transitions counts: rx_elecidle, astat[0:6], stat[0:13] (16-bit each, wrapping).
        self.add_indexed_status("gtr12_transitions", Cat(*counts))

def main():
    parser = argparse.ArgumentParser(
        description="USB3-PIPE on the Sipeed Tang Mega 138K Pro (Gowin GW5AST-138B GTR12).")
    parser.add_argument("--sys-clk-freq",   default=62.5e6, type=float, help="System clock frequency (125MHz: PHY/Core/SoC in sys).")
    parser.add_argument("--sfp",            default=0, type=int, help="SFP cage (SFP2USB adapter).", choices=[0, 1])
    parser.add_argument("--refclk",         default="100M", help="GTR12 reference clock: 100MHz REFCLK1 or 125MHz REFCLK0 (dock PLLs).", choices=["100M", "125M"])
    parser.add_argument("--pll",            default=None, help="GTR12 lane PLL (default: QPLL1, CPLL: 125MHz REFCLK0, required for 10G).", choices=["QPLL1", "CPLL"])
    parser.add_argument("--serdes-csr",     default=None, help="GTR12 register configuration file override (bring-up: serdes_toml_to_csr output).")
    parser.add_argument("--rx-idle-source", default="~astat5", help="RX Electrical Idle indicator (~astat5, elecidle, [~]astat<n>).")
    parser.add_argument("--gen2-capable",        action="store_true", help="Gen 2 (10Gbps) capable SerDes/PHY (GTR12 rate change through UPAR, 80-bit raw words).")
    parser.add_argument("--with-ltssm-history",  action="store_true", help="Gen 2 bring-up probes (TX FFE/RX CTLE/TX re-initialization CSRs, LGOOD latency, Gen 2 RX and GTR12 RX valid probes).")
    parser.add_argument("--with-tx-check",       action="store_true", help="Gen 2 TX self-check (transmitted Header Packets/Link Commands CRCs).")
    parser.add_argument("--with-tx-raw-check",   action="store_true", help="Gen 2 TX blocks self-check (scrambled blocks with SKPs decoded: Header Packets/Link Commands CRCs).")
    parser.add_argument("--with-trace",          action="store_true", help="USB3 Device Core Header Packets trace (bench/usb3_trace.py).")
    parser.add_argument("--with-gen2-debug",     action="store_true", help="Gen 2 bring-up debug (RX monitor on the raw RX words: alignment, blocks/Ordered Sets counters).")
    parser.add_argument("--with-gtr12-debug",    action="store_true", help="GTR12/PIPE bring-up debug (transitions, RX snapshots, SSP status).")
    parser.add_argument("--with-gtr12-analyzer", action="store_true", help="LiteScope Analyzer on the GTR12 raw RX words.")
    parser.add_argument("--place-option",   default=2, type=int, help="Gowin place algorithm.", choices=[0, 1, 2])
    parser.add_argument("--route-option",   default=1, type=int, help="Gowin route algorithm.", choices=[0, 1, 2])
    usb3_soc_args(parser, output_dir="build/sipeed_tang_mega_138k_pro")
    args = parser.parse_args()

    platform = sipeed_tang_mega_138k_pro.Platform(toolchain="gowin")
    # Place & Route options for timings (Gowin SUG100: timing driven by default).
    platform.toolchain.options["place_option"]        = args.place_option
    platform.toolchain.options["route_option"]        = args.route_option
    platform.toolchain.options["replicate_resources"] = 1
    soc = USB3SoC(platform,
        sys_clk_freq        = args.sys_clk_freq,
        sfp                 = args.sfp,
        refclk              = args.refclk,
        pll                 = args.pll,
        serdes_csr          = args.serdes_csr,
        rx_idle_source      = args.rx_idle_source,
        with_gtr12_debug    = args.with_gtr12_debug,
        gen2_capable        = args.gen2_capable,
        with_gen2_debug     = args.with_gen2_debug,
        with_trace          = args.with_trace,
        with_tx_check       = args.with_tx_check,
        with_tx_raw_check   = args.with_tx_raw_check,
        with_ltssm_history  = args.with_ltssm_history,
        with_gtr12_analyzer = args.with_gtr12_analyzer,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc)

if __name__ == "__main__":
    main()
