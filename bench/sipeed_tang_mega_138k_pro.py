#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# USB3-PIPE on the Sipeed Tang Mega 138K Pro (Gowin GW5AST-138B GTR12) with a SFP2USB adapter in SFP0
# (Q1 lane 0, 100 MHz REFCLK1 from the dock).

import argparse

import common # Repository root on the path.

from migen import *

from litex.gen import *

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litex.soc.cores.clock.gowin_gw5a import GW5APLL

from usb3_pipe import GW5USB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device, usb3_analyzer
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
    def __init__(self, platform, sys_clk_freq=62.5e6, sfp=0, refclk="100M", serdes_csr=None, rx_idle_source="~astat5", with_upar=True,
        with_gtr12_debug=False, with_gtr12_analyzer=False, with_analyzer=False, analyzer_group="link",
        gen2_capable=False,
        analyzer_csv="analyzer.csv", usb3_kwargs={}, **kwargs):
        # USB3 PIPE/SerDes at 125MHz (Gen 1 32-bit words); with a lower sys_clk_freq, the PIPE runs on
        # the GTR12 TX fabric clock ("tx" clock domain: frequency-locked to the transmitted words, no
        # TX CDC FIFO underflow) and the LTSSM/Core/SoC in sys with 64-bit Link Layer words (the Core
        # doesn't reach 125MHz on the GW5A fabric).
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
            usb3_kwargs = dict(usb3_kwargs, phy_cd=phy_cd, phy_clk_freq=usb3_clk_freq, link_words=2,
                with_debug=False, with_profile=False, with_trace=with_gtr12_debug)

        # USB3 (PHY, Debug, Core, Leds) ------------------------------------------------------------
        add_usb3_device(self, usb3_serdes,
            leds          = [platform.request("led_n", i) for i in range(3)],
            leds_inverted = True,
            **usb3_kwargs)

        # GTR12 bring-up: lane status, SerDes registers access (UPAR).
        gtr12 = usb3_serdes.gtr12
        gtr12.add_status_csr()
        if with_upar and ((phy_cd is None) or with_gtr12_debug or gen2_capable): # UPAR on the sys clock.
            gtr12.upar.add_csr()

        # Bring-up debug: GTR12 status transitions, PIPE counters, RX snapshots, LTSSM history.
        if with_gtr12_debug:
            self.add_gtr12_transitions(gtr12)
        if with_gtr12_debug and phy_cd is not None and hasattr(self, "usb3_phy"):
            self.add_pipe_counters(self.usb3_phy.pipe, usb3_serdes, phy_cd)
            self.add_rx_snapshots(gtr12, usb3_serdes, phy_cd, self.usb3_phy.pipe)
            self.add_ltssm_history(self.usb3_phy, sys_clk_freq)

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
        from litex.soc.interconnect.csr import CSRStorage, CSRStatus
        words = (len(value) + 31)//32
        sel   = CSRStorage(bits_for(max(words - 1, 1)), name=f"{name}_sel")
        data  = CSRStatus(32, name=f"{name}_data")
        setattr(self, f"_{name}_sel",  sel)
        setattr(self, f"_{name}_data", data)
        cases = {i: data.status.eq(value[32*i:min(32*(i+1), len(value))]) for i in range(words)}
        self.sync += Case(sel.storage, cases)

    def add_pipe_counters(self, pipe, serdes, cd):
        """PIPE bring-up counters (cd clock domain, 16-bit wrapping, resynchronized): SerDes RX words
        with a COM / K28.4 (decode error substitution) symbol, TS1/TS1 inverted/TS2 detections."""
        from migen.genlib.cdc import MultiReg
        from litex.soc.interconnect.csr import CSRStatus
        from usb3_pipe.common import COM, K
        src = serdes.source
        def has(symbol):
            return src.valid & src.ready & Reduce("OR", [src.ctrl[i] & (src.data[8*i:8*(i+1)] == symbol)
                for i in range(4)])
        events = [has(COM.value), has(K(28, 4)), pipe.ts.rx_ts1, pipe.ts.rx_ts1_inv, pipe.ts.rx_ts2]
        counts = []
        sync = getattr(self.sync, cd)
        for e in events:
            e_r = Signal()
            c   = Signal(16)
            sync += [e_r.eq(e), If(e_r, c.eq(c + 1))]
            cs = Signal(16)
            self.specials += MultiReg(c, cs)
            counts.append(cs)
        # PIPE counters: COM words, K28.4 words, TS1/TS1 inverted/TS2 detections (16-bit each).
        self.add_indexed_status("pipe_counters", Cat(*counts))

    def add_ltssm_history(self, phy, sys_clk_freq, n=32):
        """LTSSM states history (bring-up): last n states (8-bit) with their entry time (24-bit, us),
        frozen while ltssm_history_freeze is set."""
        from litex.soc.interconnect.csr import CSRStorage, CSRStatus
        self._ltssm_history_freeze = CSRStorage()
        us      = Signal(24)
        tick    = Signal(max=int(sys_clk_freq/1e6))
        state_d = Signal(8)
        hist    = [Signal(32) for _ in range(n)]
        self.sync += [
            tick.eq(tick + 1),
            If(tick == int(sys_clk_freq/1e6) - 1, tick.eq(0), us.eq(us + 1)),
            state_d.eq(phy.ltssm_state),
            If((phy.ltssm_state != state_d) & ~self._ltssm_history_freeze.storage,
                hist[0].eq(Cat(phy.ltssm_state, us)),
                *[hist[i].eq(hist[i-1]) for i in range(1, n)]
            ),
        ]
        # n x (state[8], time_us[24]), newest first.
        self.add_indexed_status("ltssm_history", Cat(*hist))

    def add_rx_snapshots(self, gtr12, serdes, cd, pipe=None):
        """RX snapshots (bring-up, captured registers static when read): 8 cycles of GTR12 raw words
        and decoders outputs from a K28.5 in the raw words (rx clock domain), 16 SerDes RX words
        (cd clock domain). Re-armed on each rx_snapshot_arm write."""
        from migen.genlib.cdc import PulseSynchronizer
        from litex.soc.interconnect.csr import CSR, CSRStatus
        self._rx_snapshot_arm = CSR()
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
        k285 = Reduce("OR", [(raw[10*i:10*(i+1)] == 0x17c) | (raw[10*i:10*(i+1)] == 0x283) for i in range(4)])
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

            # TX ring: last 64 PIPE TX words (Link Layer words accepted by the PIPE in U0), frozen on
            # the PIPE U0 exit (tx_ready falling), re-armed with rx_snapshot_arm.
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

    def add_gtr12_transitions(self, gtr12):
        """Transitions counters of the GTR12 lane status bits (rx_elecidle, astat[0:6], stat[0:13]),
        sampled in the reset-less RX clock domain: LFPS/Electrical Idle indicators identification."""
        from migen.genlib.cdc import MultiReg
        from litex.soc.interconnect.csr import CSRStatus
        self.cd_rx_nr = ClockDomain(reset_less=True)
        self.comb += self.cd_rx_nr.clk.eq(ClockSignal("rx"))
        bits = [gtr12.rx_elecidle] + [gtr12.astat[i] for i in range(6)] + [gtr12.stat[i] for i in range(13)]
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
    parser = argparse.ArgumentParser(description="USB3-PIPE on the Sipeed Tang Mega 138K Pro (Gowin GW5AST-138B GTR12).")
    parser.add_argument("--sys-clk-freq",   default=62.5e6, type=float, help="System clock frequency (125MHz: PHY/Core/SoC in sys).")
    parser.add_argument("--sfp",            default=0, type=int, help="SFP cage (SFP2USB adapter).", choices=[0, 1])
    parser.add_argument("--refclk",         default="100M", help="GTR12 reference clock: 100MHz REFCLK1 or 125MHz REFCLK0 (dock PLLs).", choices=["100M", "125M"])
    parser.add_argument("--serdes-csr",     default=None, help="GTR12 register configuration file override (bring-up: serdes_toml_to_csr output).")
    parser.add_argument("--rx-idle-source", default="~astat5", help="RX Electrical Idle indicator (~astat5, elecidle, [~]astat<n>).")
    parser.add_argument("--no-upar",        action="store_true", help="No SerDes registers runtime access (UPAR).")
    parser.add_argument("--gen2-capable",        action="store_true", help="Gen 2 (10Gbps) capable SerDes/PHY (GTR12 rate change through UPAR, 80-bit raw words).")
    parser.add_argument("--with-gtr12-debug",    action="store_true", help="GTR12/PIPE bring-up debug (transitions/PIPE counters, RX snapshots, LTSSM history).")
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
        sys_clk_freq   = args.sys_clk_freq,
        sfp            = args.sfp,
        refclk         = args.refclk,
        serdes_csr     = args.serdes_csr,
        rx_idle_source = args.rx_idle_source,
        with_upar      = not args.no_upar,
        with_gtr12_debug    = args.with_gtr12_debug,
        gen2_capable        = args.gen2_capable,
        with_gtr12_analyzer = args.with_gtr12_analyzer,
        **usb3_soc_argdict(args))
    usb3_soc_build(parser, args, soc)

if __name__ == "__main__":
    main()
