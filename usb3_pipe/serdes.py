#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 SerDes (USB 3.2 chapter 6): LiteICLink transceivers (Artix7 GTP/Kintex7 GTX/Ultrascale+
GTY) at 5Gbps with the TX/RX datapaths (usb3_pipe/datapath.py) and LFPS (TX pattern/electrical
idle, filtered RX electrical idle, RX decode errors).
"""

from migen import *
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.datapath import RXErrorSubstitution, RXErrorMonitor, RXIdleFilter
from usb3_pipe.datapath import TXDatapath, RXDatapath

# USB3 Serializer/Deserializer ---------------------------------------------------------------------

class USB3SerDes(LiteXModule):
    """USB3 SerDes on a LiteICLink transceiver.

    Common part of the transceivers: 5Gbps, 20-bit (or 40-bit) with 8b/10b, TX/RX buffers (no clock aligner, word
    alignment done in the RX datapath), TX/RX datapaths (SKP insertion/removal, CDC, word alignment),
    LFPS (TX pattern/electrical idle, filtered RX electrical idle), RX decode errors (Warm Reset LFPS
    detection) and timing constraints. Transceivers specific parts (PLL, USB3 parameters overrides,
    electrical idle/OOB signals) are done by the subclasses.
    """
    def __init__(self):
        # SerDes interface (used by USB3PIPE/USB3PHY/LFPSUnit/debug, also implemented by the
        # simulation models: test/model/gt.py, sim.py).
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i
        self.tx_lfps     = Signal()   # i: LFPS burst, transmit tx_pattern raw.

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_errors   = Signal()   # o: RX decode errors (Warm Reset LFPS detection).

        self.tx_skp_enable = Signal(reset=1) # i
        self.rx_align      = Signal()        # i
        self.rx_reset      = Signal()        # i: RX re-initialization (PMA/CDR/PCS reset sequence).
        self.rx_reinit     = Signal()        # RX re-initialization request of the SerDes itself (ex: rate change).

        self.rx_idle_stretch = 0    # rx_idle stretch (cycles) of the RX idle filter (add_lfps).
        self.xcvr_name       = None # Transceiver submodule name (add_transceiver).

        # Gen 2 (gen2_capable SerDes): 10Gbps line rate and raw transceiver words (Gen2PIPE).
        self.gen2_capable   = False
        self.rate_10g       = Signal()   # i: 10Gbps line rate.
        self.gen2_tx_enable = Signal()   # i: Transmit gen2_tx_data (raw words, 8b/10b bypassed).
        self.gen2_tx_data   = Signal(40) # i: Raw TX words (tx clock domain, bit 0 first).
        self.gen2_rx_data   = Signal(40) # o: Raw RX words (rx clock domain, bit 0 first).
        self.gen2_rx_errors = Signal()   # i: Gen 2 RX errors (block alignment lost, sys clock domain),
                                         #    rx_errors at 10Gbps (no 8b/10b decode errors).

    def get_refclk(self, refclk_pads, buffer="IBUFDS_GTE2"):
        """Transceiver reference clock: from dedicated pads (buffer) or a fabric clock."""
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            return refclk_pads
        refclk = Signal()
        self.specials += Instance(buffer,
            i_CEB = 0,
            i_I   = refclk_pads.p,
            i_IB  = refclk_pads.n,
            o_O   = refclk
        )
        return refclk

    def add_transceiver(self, name, xcvr_cls, pll, tx_pads, rx_pads, sys_clk_freq, data_width=20,
        **kwargs):
        """Transceiver channel (added as name) with the TX/RX datapaths (data_width: 20 or 40 bits,
        2 or 4 symbols per transceiver clock cycle, kwargs: extra transceiver parameters)."""
        xcvr = xcvr_cls(pll, tx_pads, rx_pads, sys_clk_freq, **kwargs,
            data_width       = data_width,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            tx_polarity      = self.tx_polarity,
            rx_polarity      = self.rx_polarity)
        # Comma alignment on any byte: USB3 Ordered Sets start with consecutive COMs (4 for the
        # TS1/TS2), with 2 symbols per word half of them are on odd bytes. Aligning commas on even
        # bytes only (LiteICLink default) made the aligner slide on every Ordered Set when the first
        # COM was received on an odd byte (ex: after a U3 exit), duplicating/dropping a symbol each
        # time: no TS1/TS2 detected. The bytes rotation is handled by the RXWordAligner.
        getattr(xcvr, name + "_params").update(p_ALIGN_COMMA_WORD=1)
        return self.add_xcvr_datapath(name, xcvr, data_width)

    def add_xcvr_datapath(self, name, xcvr, data_width, rx_disparity_check=True, tx_cdc=True):
        """TX/RX datapaths of a LiteICLink transceiver (added as name): 8b/10b stream endpoints,
        SKP insertion/removal, CDC, word alignment, RX errors substitution."""
        self.add_module(name=name, module=xcvr)
        self.xcvr_name = name
        xcvr.add_stream_endpoints()
        phy_dw = 8*(data_width//10)
        self.tx_datapath     = tx_datapath     = TXDatapath("tx", phy_dw=phy_dw, with_cdc=tx_cdc)
        self.rx_substitution = rx_substitution = RXErrorSubstitution(xcvr, "rx",
            with_disparity=rx_disparity_check)
        self.rx_buffer       = rx_buffer       = ClockDomainsRenamer("rx")(
            stream.Buffer(rx_substitution.source.description, pipe_ready=True)) # Timings.
        self.rx_datapath     = rx_datapath     = RXDatapath("rx", phy_dw=phy_dw)
        self.comb += [
            xcvr.tx_enable.eq(self.enable),
            xcvr.rx_enable.eq(self.enable & ~self.rx_reset & ~self.rx_reinit),
            self.ready.eq(xcvr.tx_ready & xcvr.rx_ready),
            xcvr.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.skp_enable.eq(self.tx_skp_enable),
            tx_datapath.source.connect(xcvr.sink),
            xcvr.source.connect(rx_substitution.sink),
            rx_substitution.source.connect(rx_buffer.sink),
            rx_buffer.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]
        return xcvr

    def add_lfps(self, xcvr, rx_idle):
        """LFPS: TX pattern, filtered RX electrical idle (from the transceiver's OOB detector) and RX
        decode errors (Warm Reset LFPS detection)."""
        self.comb += [
            xcvr.tx_produce_pattern.eq(self.tx_lfps | (self.tx_pattern != 0)),
            xcvr.tx_pattern.eq(Replicate(self.tx_pattern[0], len(xcvr.tx_pattern))), # Constant words.
        ]
        self.rx_idle_filter  = RXIdleFilter(rx_idle)
        self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles
        self.comb += self.rx_idle.eq(self.rx_idle_filter.idle)
        self.rx_error_monitor = RXErrorMonitor(xcvr.decoders, clock_domain="rx")
        # At 10Gbps (Gen 2, raw 128b/132b words), the 8b/10b decode errors are always set: Gen 2 RX
        # errors (block alignment lost) instead (a Warm Reset LFPS on a Gen 2 link would otherwise be
        # detected after tResetDelay in U0).
        self.comb += self.rx_errors.eq(Mux(self.rate_10g, self.gen2_rx_errors,
            self.rx_error_monitor.errors))

    def add_timing_constraints(self, platform, sys_clk, xcvr):
        platform.add_period_constraint(xcvr.cd_tx.clk, 1e9/xcvr.tx_clk_freq)
        platform.add_period_constraint(xcvr.cd_rx.clk, 1e9/xcvr.rx_clk_freq)
        platform.add_false_path_constraints(sys_clk, xcvr.cd_tx.clk, xcvr.cd_rx.clk)

# Xilinx Kintex7 USB3 Serializer/Deserializer ------------------------------------------------------

class K7USB3SerDes(USB3SerDes):
    """Kintex7 GTX USB3 SerDes (Channel PLL)."""
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gtx_7series import GTXChannelPLL, GTX

        # PLL / Transceiver ------------------------------------------------------------------------
        self.pll = pll = GTXChannelPLL(self.get_refclk(refclk_pads), refclk_freq, 5e9)
        gtx = self.add_transceiver("gtx", GTX, pll, tx_pads, rx_pads, sys_clk_freq)

        # Override GTX RX settings for USB3 --------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings from
        # the Xilinx 7-Series Transceivers Wizard (SATA, SSC) and PCIe Gen2 (pcie_7x) GT wrappers:
        # - RXCDR_CFG: SSC tolerant CDR settings (RXOUT_DIV=1).
        # - RXCDR_HOLD_DURING_EIDLE/RX_DFE_LPM_HOLD_DURING_EIDLE: the CDR/LPM keep their state during
        #   Electrical Idle/LFPS.
        # - LPM mode (as PCIe Gen1/Gen2).
        # - RX_CM_SEL/RX_CM_TRIM: Programmable, 800mV.
        assert pll.config['d'] == 1 # 5Gbps.
        gtx.gtx_params.update(
            p_RXCDR_CFG                    = 0x03_8000_8BFF_4010_0008,
            p_RXCDR_HOLD_DURING_EIDLE      = 0b1,
            p_RX_DFE_LPM_HOLD_DURING_EIDLE = 0b1,
            p_RX_DFE_LPM_CFG               = 0x0904,
            i_RXLPMEN                      = 0b1,
            p_RX_CM_SEL                    = 0b11,
            p_RX_CM_TRIM                   = 0b010,
            p_PMA_RSV                      = 0x00018480,
            p_PMA_RSV2                     = 0x2050,
        )

        # Override GTX TX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtx.gtx_params.update(
            i_TXDIFFCTRL = 0b1100,
        )

        # Override GTX parameters/signals for USB3 LFPS --------------------------------------------
        # PCS_RSVD_ATTR: OOB powered up (bit 8), OOB clock from CLKRSVD[0] (bit 3).
        rx_idle = Signal()
        gtx.gtx_params.update(
            p_PCS_RSVD_ATTR    = 0x000000000108,
            p_RXOOB_CFG        = 0b0000110,
            i_CLKRSVD          = ClockSignal("oob"),
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gtx, rx_idle)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtx)

# Xilinx Artix7 USB3 Serializer/Deserializer -------------------------------------------------------

class A7USB3SerDes(USB3SerDes):
    """Artix7 GTP USB3 SerDes (Quad PLL, optional RXCDR_CFG override).

    pll: optional external Quad PLL (shared with other protocols in the quad: PCIe, Ethernet, ...),
    with the GTPQuadPLL interface (channel, clk, refclk, lock, reset, config) and configured for
    5Gbps (2.5GHz VCO, D=1). The PLL is then not reset by the transceiver (owned by the user) and
    refclk_pads/refclk_freq are unused.
    """
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads,
        rxcdr_cfg=None, pll=None):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gtp_7series import GTPQuadPLL, GTP

        # PLL / Transceiver ------------------------------------------------------------------------
        # Fabric reference clock: PLL from GTGREFCLK (dedicated clock network input). Routing it to
        # GTREFCLK0 goes through general interconnect and the RX quality then depends on the build.
        external_pll = pll is not None
        if not external_pll:
            refclk_from_fabric = isinstance(refclk_pads, (Signal, ClockSignal))
            pll = GTPQuadPLL(self.get_refclk(refclk_pads), refclk_freq, 5e9,
                refclk_from_fabric = refclk_from_fabric)
        self.pll = pll
        gtp = self.add_transceiver("gtp", GTP, pll, tx_pads, rx_pads, sys_clk_freq,
            qpll_reset = not external_pll)

        # Override GTP RX settings for USB3 --------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz):
        # - RXCDR_CFG: UG482 Table 4-14 (CDR Recommended Settings for Protocols with SSC, RXOUT_DIV=1).
        #   The 8B/10B (no SSC) settings (Table 4-15, +-1250ppm) saturate the CDR frequency tracking
        #   (recovered clock pinned at ~-1465ppm) and slip a bit every few hundred bits on hardware.
        # - RXCDR_HOLD_DURING_EIDLE/RXCDR_LOCK_CFG: Xilinx 7-Series PCIe Gen2 GTP wrapper settings
        #   (pcie_7x gt_wrapper): the CDR keeps its frequency during Electrical Idle/LFPS.
        assert pll.config['d'] == 1 # 5Gbps.
        gtp.gtp_params.update(
            p_RX_CM_SEL               = 0b11,   # Programmable, 800mV.
            p_RX_CM_TRIM              = 0b1010,
            p_RXCDR_CFG               = 0x0_0000_87FE_2060_2448_1010 if rxcdr_cfg is None else rxcdr_cfg,
            p_RXCDR_HOLD_DURING_EIDLE = 0b1,
            p_RXCDR_LOCK_CFG          = 0b010101,
            p_RXLPM_INCM_CFG          = 0b0,
            p_RXLPM_IPCM_CFG          = 0b1,
        )

        # Override GTP TX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtp.gtp_params.update(
            i_TXDIFFCTRL = 0b1100,
        )

        # Override GTP parameters/signals to allow LFPS --------------------------------------------
        rx_idle = Signal()
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR    = 0x000000000100,
            p_RXOOB_CLK_CFG    = "FABRIC",
            i_SIGVALIDCLK      = ClockSignal("oob"),
            p_RXOOB_CFG        = 0b0000110,
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gtp, rx_idle)

        # RX status (debug: comma realignment, elastic buffer), synchronized to sys ----------------
        rx_realign    = Signal()
        rx_bufstatus  = Signal(3)
        self.rx_realign  = Signal()
        self.rx_buferror = Signal()
        gtp.gtp_params.update(
            o_RXBYTEREALIGN = rx_realign,
            o_RXBUFSTATUS   = rx_bufstatus,
        )
        realign_ps  = PulseSynchronizer("rx", "sys")
        buferror_ps = PulseSynchronizer("rx", "sys")
        self.submodules += realign_ps, buferror_ps
        self.comb += [
            realign_ps.i.eq(rx_realign),
            buferror_ps.i.eq(rx_bufstatus[2]),
            self.rx_realign.eq(realign_ps.o),
            self.rx_buferror.eq(buferror_ps.o),
        ]

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtp)

# Xilinx Ultrascale+ USB3 Serializer/Deserializer --------------------------------------------------

class USPUSB3SerDes(USB3SerDes):
    """Ultrascale+ GTY USB3 SerDes (reference clock from dedicated pads or from the fabric).

    gen2_capable: QPLL at 10Gbps, 40-bit transceiver data (4 symbols per cycle), Gen 1 (5Gbps) with
    the /2 TX/RX rate (125MHz transceiver clocks), 10Gbps selected with `rate_10g` (Gen 2, raw
    128b/132b blocks: transceiver raw symbols). Else CPLL at 5Gbps, 20-bit (250MHz).
    """
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads,
        gen2_capable=False):
        USB3SerDes.__init__(self)
        self.gen2_capable = gen2_capable

        # # #

        from liteiclink.serdes.gty_ultrascale import GTYChannelPLL, GTYQuadPLL, GTY

        # PLL / Transceiver ------------------------------------------------------------------------
        refclk_from_fabric = isinstance(refclk_pads, (Signal, ClockSignal))
        refclk = self.get_refclk(refclk_pads, buffer="IBUFDS_GTE4")
        if gen2_capable:
            self.pll = pll = GTYQuadPLL(refclk, refclk_freq, 10e9, refclk_from_fabric=refclk_from_fabric)
            gty = self.add_transceiver("gty", GTY, pll, tx_pads, rx_pads, sys_clk_freq, data_width=40)
            # TX/RX rate: /2 (5Gbps, Gen 1) or /1 (10Gbps, Gen 2) from the 10Gbps QPLL.
            self.comb += [
                gty.tx_rate.eq(Mux(self.rate_10g, 0b001, 0b010)),
                gty.rx_rate.eq(Mux(self.rate_10g, 0b001, 0b010)),
            ]
            # RX re-initialization (~1us) on rate changes (CDR/PCS restart at the new rate).
            rate_10g_d = Signal()
            reinit     = Signal(max=int(sys_clk_freq*1e-6) + 1)
            self.sync += [
                rate_10g_d.eq(self.rate_10g),
                If(self.rate_10g != rate_10g_d,
                    reinit.eq(int(sys_clk_freq*1e-6))
                ).Elif(reinit != 0,
                    reinit.eq(reinit - 1)
                )
            ]
            self.comb += self.rx_reinit.eq(reinit != 0)
            # Raw words (Gen 2).
            self.comb += [
                gty.tx_raw_enable.eq(self.gen2_tx_enable),
                gty.tx_raw.eq(self.gen2_tx_data),
                self.gen2_rx_data.eq(gty.rx_raw),
            ]
        else:
            self.pll = pll = GTYChannelPLL(refclk, refclk_freq, 5e9)
            gty = self.add_transceiver("gty", GTY, pll, tx_pads, rx_pads, sys_clk_freq)
        assert pll.config['d'] == 1 # 5Gbps (CPLL) / 10Gbps (QPLL).

        # Fabric reference clock: CPLL from GTGREFCLK (the QPLL is configured by GTYQuadPLL).
        if refclk_from_fabric and not gen2_capable:
            gty.gty_params.update(
                i_GTREFCLK0     = 0,
                i_GTGREFCLK     = refclk,
                i_CPLLREFCLKSEL = 0b111,
            )

        # Override GTY RX settings for USB3 --------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings from
        # the Xilinx Ultrascale+ Transceivers Wizard (SATA preset at 5Gbps, SSC) and PCIe Gen2
        # (pcie4_uscale_plus) GT wrappers:
        # - RXCDR_CFG2/3: SSC tolerant CDR settings.
        # - RX_SUM_*/RX_DEGEN_CTRL/RX_DFE_AGC_CFG1: 5Gbps settings (LiteICLink: high line rates ones).
        # - RX/TX_CLK25_DIV: from the reference clock (also clocks the OOB detector).
        # - RXCDRHOLD: the CDR keeps its state during Electrical Idle/LFPS (as pcie4_uscale_plus).
        # - LPM mode (as PCIe Gen1/Gen2, LiteICLink default).
        clk25_div = int((refclk_freq + 25e6 - 1)//25e6)
        rx_idle   = Signal()
        gty.gty_params.update(
            p_RXCDR_CFG2                = 0x0263,
            p_RXCDR_CFG2_GEN2           = 0x263,
            p_RXCDR_CFG3                = 0x001a,
            p_RXCDR_CFG3_GEN2           = 0b011010,
            p_RX_SUM_VCMTUNE            = 0b0011,
            p_RX_SUM_DEGEN_AVTT_OVERITE = 0b0,
            p_RX_DEGEN_CTRL             = 0b100,
            p_RX_DFE_AGC_CFG1           = 0b010,
            p_RX_CLK25_DIV              = clk25_div,
            p_TX_CLK25_DIV              = clk25_div,
            i_RXCDRHOLD                 = rx_idle,
        )

        # Override GTY parameters/signals for USB3 LFPS --------------------------------------------
        gty.gty_params.update(
            p_OOB_PWRUP        = 0b1,
            p_OOBDIVCTL        = 0b01,
            p_RXOOB_CFG        = 0b000000110,
            p_RXOOB_CLK_CFG    = "PMA",
            i_RXELECIDLEMODE   = 0b00,
            o_RXELECIDLE       = rx_idle,
            i_TXPDELECIDLEMODE = 0b1,
            i_TXELECIDLE       = self.tx_idle
        )
        self.add_lfps(gty, rx_idle)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gty)

# Gowin GW5AST USB3 Serializer/Deserializer --------------------------------------------------------

class GW5USB3SerDes(USB3SerDes):
    """Gowin GW5AST-138B GTR12 USB3 SerDes (Q1 lane 0/1: SFP0/SFP1 of the Tang Mega 138K Pro dock,
    100 MHz REFCLK1).

    5Gbps, raw 20-bit PMA with 40-bit fabric words (125MHz, 4 symbols per cycle), 8b/10b and polarity
    in the fabric (LiteICLink GTR12), comma alignment by the PCS. The register configuration is
    generated from usb3_pipe/gw5 TOML files with Gowin's tools.

    TX Electrical Idle: DC (constant raw words) between the LFPS bursts (no transitions on the AC
    coupled link). rx_idle_source: RX Electrical Idle indicator of the lane ("~astat5": analog status
    bit 5 inverted, set while a signal is detected (LFPS bursts toggle it, measured on hardware),
    "elecidle": RXELECIDLE, "[~]astat<n>"), with_upar: SerDes registers runtime access, upar_clk: UPAR
    (and quad AHB) clock, xcvr_cd: clock domain of the GTR12 control logic (free-running, see the
    Tang Mega 138K Pro target: SerDes datapath on the GTR12 TX clock).
    """
    def __init__(self, platform, sys_clk, sys_clk_freq, lane=0, rx_idle_source="~astat5",
        with_upar=True, upar_clk=None, xcvr_cd="sys", refclk="100M", csr_config=None, tx_cdc=True):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.gtr12_gw5 import GTR12
        from usb3_pipe.gw5 import gw5_usb3_csr_config

        # Transceiver ------------------------------------------------------------------------------
        gtr12 = GTR12(platform,
            line_rate   = 5e9,
            csr_config  = gw5_usb3_csr_config(lane, "5G", refclk) if csr_config is None else csr_config,
            data_width  = 40,
            position    = "Q1",
            lane        = lane,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity,
            csr_name    = f"usb3_5g_ln{lane}.csr",
            with_upar   = with_upar,
            upar_clk    = upar_clk)
        # GTR12 control logic (PLL lock resynchronization, resets, UPAR) in xcvr_cd: a free-running
        # clock domain when this SerDes "sys" clock domain is renamed to the GTR12 TX clock.
        if xcvr_cd != "sys":
            gtr12 = ClockDomainsRenamer({"sys": xcvr_cd})(gtr12)
        # No RX running disparity check (RX paths at 125MHz on the GW5A fabric).
        self.add_xcvr_datapath("gtr12", gtr12, data_width=40, rx_disparity_check=False,
            tx_cdc=tx_cdc)

        # LFPS -------------------------------------------------------------------------------------
        rx_idle = Signal()
        if rx_idle_source == "elecidle":
            self.comb += rx_idle.eq(gtr12.rx_elecidle)
        else:
            invert = rx_idle_source.startswith("~")
            source = rx_idle_source.lstrip("~")
            assert source.startswith("astat")
            bit = gtr12.astat[int(source[5:])]
            self.comb += rx_idle.eq(~bit if invert else bit)
        self.add_lfps(gtr12, rx_idle)
        # TX Electrical Idle: constant (zeros) raw words when idle and no LFPS burst.
        self.comb += If(self.tx_idle & ~self.tx_lfps & (self.tx_pattern == 0),
            gtr12.tx_produce_pattern.eq(1),
            gtr12.tx_pattern.eq(0),
        )

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtr12)
