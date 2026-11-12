#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 SerDes (USB 3.2 chapter 6): LiteICLink transceivers (Artix7 GTP/Kintex7 GTX/UltraScale+
GTY/Gowin GW5AST/GW5AT GTR12/Lattice ECP5-5G DCU) at 5Gbps, 10Gbps on the GTY and the GTR12 with
gen2_capable (Gen 2, raw transceiver words), with the TX/RX datapaths (usb3_pipe/datapath.py) and
LFPS (TX pattern/electrical idle, filtered RX electrical idle, RX decode errors).
"""

from migen import *
from migen.genlib.cdc import PulseSynchronizer, MultiReg

from litex.gen import *

from litex.soc.interconnect import stream

from usb3_pipe.common   import Buffer
from usb3_pipe.datapath import RXErrorSubstitution, RXErrorMonitor, RXIdleFilter
from usb3_pipe.datapath import TXDatapath, RXDatapath

# USB3 Serializer/Deserializer ---------------------------------------------------------------------

class USB3SerDes(LiteXModule):
    """USB3 SerDes on a LiteICLink transceiver.

    Common part of the transceivers: 5Gbps, 20-bit (or 40-bit) with 8b/10b, TX/RX buffers (no clock
    aligner, word alignment done in the RX datapath), TX/RX datapaths (SKP insertion/removal, CDC,
    word alignment), LFPS (TX pattern/electrical idle, filtered RX electrical idle), RX decode
    errors (Warm Reset LFPS detection) and timing constraints. Transceivers specific parts (PLL,
    USB3 parameters overrides, electrical idle/OOB signals) are done by the subclasses.
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
        self.rx_reinit     = Signal()        # RX re-initialization request of the SerDes itself
                                             # (ex: rate change).
        self.tx_reinit     = Signal()        # TX re-initialization request of the SerDes itself
                                             # (ex: lane reconfiguration).

        self.rx_idle_stretch = 0    # rx_idle stretch (cycles) of the RX idle filter (add_lfps).
        self.xcvr_name       = None # Transceiver submodule name (add_transceiver).

        # Gen 2 (gen2_capable SerDes): 10Gbps line rate and raw transceiver words (Gen2PIPE).
        self.gen2_capable   = False
        self.rate_10g       = Signal()   # i: 10Gbps line rate.
        self.gen2_tx_enable = Signal()   # i: Transmit gen2_tx_data (raw words, 8b/10b bypassed).
        self.gen2_tx_data   = Signal(40) # i: Raw TX words (tx clock domain, bit 0 first).
        self.gen2_rx_data   = Signal(40) # o: Raw RX words (rx clock domain, bit 0 first).
        self.gen2_rx_errors = Signal()   # i: Gen 2 RX errors (block alignment lost, sys clock
                                         #    domain), rx_errors at 10Gbps (no 8b/10b decode
                                         #    errors).

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

    def add_xcvr_datapath(self, name, xcvr, data_width, rx_disparity_check=True, tx_cdc=True,
        tx_input_register=False, rx_rom_style="distributed"):
        """TX/RX datapaths of a LiteICLink transceiver (added as name): 8b/10b stream endpoints,
        SKP insertion/removal, CDC, word alignment, RX errors substitution (rx_rom_style: 8b/10b
        checker ROM style)."""
        self.add_module(name=name, module=xcvr)
        self.xcvr_name = name
        xcvr.add_stream_endpoints()
        phy_dw = 8*(data_width//10)
        self.tx_datapath     = tx_datapath     = TXDatapath("tx", phy_dw=phy_dw, with_cdc=tx_cdc,
            with_input_register=tx_input_register)
        self.rx_substitution = rx_substitution = RXErrorSubstitution(xcvr, "rx",
            with_disparity=rx_disparity_check, rom_style=rx_rom_style)
        self.rx_buffer       = rx_buffer       = ClockDomainsRenamer("rx")(
            Buffer(rx_substitution.source.description, pipe_ready=True)) # Timings.
        self.rx_datapath     = rx_datapath     = RXDatapath("rx", phy_dw=phy_dw)
        self.comb += [
            xcvr.tx_enable.eq(self.enable & ~self.tx_reinit),
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

    def add_lfps(self, xcvr, rx_idle, rx_idle_burst_cycles=1):
        """LFPS: TX pattern, filtered RX electrical idle (from the transceiver's OOB detector, non-idle
        spikes shorter than rx_idle_burst_cycles rejected) and RX decode errors (Warm Reset LFPS
        detection)."""
        self.comb += [
            xcvr.tx_produce_pattern.eq(self.tx_lfps | (self.tx_pattern != 0)),
            # Constant words.
            xcvr.tx_pattern.eq(Replicate(self.tx_pattern[0], len(xcvr.tx_pattern))),
        ]
        self.rx_idle_filter  = RXIdleFilter(rx_idle, burst_cycles=rx_idle_burst_cycles)
        self.rx_idle_stretch = self.rx_idle_filter.stretch_cycles
        self.comb += self.rx_idle.eq(self.rx_idle_filter.idle)
        self.rx_error_monitor = RXErrorMonitor(xcvr.decoders, clock_domain="rx")
        # At 10Gbps (Gen 2, raw 128b/132b words), the 8b/10b decode errors are always set: Gen 2 RX
        # errors (block alignment lost) instead (a Warm Reset LFPS on a Gen 2 link would otherwise
        # be detected after tResetDelay in U0).
        self.comb += self.rx_errors.eq(Mux(self.rate_10g, self.gen2_rx_errors,
            self.rx_error_monitor.errors))

    def add_timing_constraints(self, platform, sys_clk, xcvr, with_tx=True):
        """Transceiver TX/RX clocks constraints (with_tx: False when the TX clock is shared with
        another transceiver, already constrained)."""
        if with_tx:
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
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings
        # from the Xilinx 7-Series Transceivers Wizard (SATA, SSC) and PCIe Gen2 (pcie_7x) GT
        # wrappers:
        # - RXCDR_CFG: SSC tolerant CDR settings (RXOUT_DIV=1).
        # - RXCDR_HOLD_DURING_EIDLE/RX_DFE_LPM_HOLD_DURING_EIDLE: the CDR/LPM keep their state
        #   during Electrical Idle/LFPS.
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

    tx_clk: optional TX clock shared with another transceiver of the quad on the same PLL (ex: the
    TX clock of Lane 0 for Lane 1 in Gen 1x2: the lanes TX run in lockstep), else the TX clock is
    generated from the transceiver TXOUTCLK.

    rx_idle_min_burst: optional minimum RX non-idle period (s, ex: 40e-9, the shortest LFPS burst):
    shorter non-idle spikes from the RX electrical idle detector are rejected (RXIdleFilter).
    """
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads,
        rxcdr_cfg=None, pll=None, tx_clk=None, rx_idle_min_burst=None):
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
            qpll_reset = not external_pll,
            tx_clk     = tx_clk)

        # Override GTP RX settings for USB3 --------------------------------------------------------
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz):
        # - RXCDR_CFG: UG482 Table 4-14 (CDR Recommended Settings for Protocols with SSC,
        #   RXOUT_DIV=1). The 8B/10B (no SSC) settings (Table 4-15, +-1250ppm) saturate the CDR
        #   frequency tracking (recovered clock pinned at ~-1465ppm) and slip a bit every few
        #   hundred bits on hardware.
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
        rx_idle_burst_cycles = 1
        if rx_idle_min_burst is not None:
            rx_idle_burst_cycles = max(1, int(rx_idle_min_burst*sys_clk_freq))
        self.add_lfps(gtp, rx_idle, rx_idle_burst_cycles=rx_idle_burst_cycles)

        # RX status (debug: comma realignment, elastic buffer), synchronized to sys ----------------
        rx_realign       = Signal()
        rx_bufstatus     = Signal(3)
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
        self.add_timing_constraints(platform, sys_clk, gtp, with_tx=tx_clk is None)

# Xilinx UltraScale+ USB3 Serializer/Deserializer --------------------------------------------------

class USPUSB3SerDes(USB3SerDes):
    """UltraScale+ GTY USB3 SerDes (reference clock from dedicated pads or from the fabric).

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
            self.pll = pll = GTYQuadPLL(refclk, refclk_freq, 10e9,
                refclk_from_fabric = refclk_from_fabric)
            gty = self.add_transceiver("gty", GTY, pll, tx_pads, rx_pads, sys_clk_freq,
                data_width = 40)
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
        # USB3 links have independent clocks and the host uses SSC (0/-5000ppm, 33kHz), settings
        # from the Xilinx UltraScale+ Transceivers Wizard (SATA preset at 5Gbps, SSC) and PCIe Gen2
        # (pcie4_uscale_plus) GT wrappers:
        # - RXCDR_CFG2/3: SSC tolerant CDR settings.
        # - RX_SUM_*/RX_DEGEN_CTRL/RX_DFE_AGC_CFG1: 5Gbps settings (LiteICLink: high line rates
        #   ones).
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

class GW5RateSequencer(LiteXModule):
    """GTR12 lane rate change (5Gbps <-> 10Gbps): on a rate_10g change, the lane is held in reset
    (busy), the rate dependent registers are written through UPAR (writes: [(addr, value_5g,
    value_10g)] or [(addr, value_5g, value_10g, wait_cycles)]: wait after the write, ex: PLL
    recalibration), then the lane is released after a settling delay. rate_10g is resynchronized
    (asynchronous input). Values: constants or Signals (runtime values, ex: TX FFE)."""
    def __init__(self, upar, writes, settle_cycles=256):
        writes = [tuple(w) + (0,)*(4 - len(w)) for w in writes]
        self.rate_10g = Signal() # i: Asynchronous.
        self.busy     = Signal() # o: Lane held in reset.

        # # #

        rate_10g = Signal()
        current  = Signal()
        self.specials += MultiReg(self.rate_10g, rate_10g)

        index = Signal(max=max(len(writes), 2))
        timer = Signal(max=max([settle_cycles] + [w for a, v5, v10, w in writes]) + 1)
        value = lambda v: C(v, 32) if isinstance(v, int) else v
        self.comb += [
            upar.addr.eq(Array([C(a, 24) for a, v5, v10, w in writes])[index]),
            upar.wrdata.eq(Mux(current,
                Array([value(v10) for a, v5, v10, w in writes])[index],
                Array([value(v5)  for a, v5, v10, w in writes])[index])),
        ]
        waits = [w for a, v5, v10, w in writes]
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(rate_10g != current,
                NextValue(current, rate_10g),
                NextValue(index, 0),
                NextValue(timer, settle_cycles),
                NextState("RESET")
            )
        )
        fsm.act("RESET",
            # Lane in reset before the registers writes.
            self.busy.eq(1),
            NextValue(timer, timer - 1),
            If(timer == 0, NextState("WRITE"))
        )
        fsm.act("WRITE",
            self.busy.eq(1),
            upar.we.eq(1),
            If(upar.done,
                NextValue(index, index + 1),
                If(index == (len(writes) - 1),
                    NextValue(timer, settle_cycles),
                    NextState("SETTLE")
                ).Elif(Array([C(w > 0) for w in waits])[index],
                    NextValue(timer, Array([C(max(w - 1, 0), len(timer)) for w in waits])[index]),
                    NextState("WAIT")
                )
            )
        )
        fsm.act("WAIT",
            self.busy.eq(1),
            NextValue(timer, timer - 1),
            If(timer == 0, NextState("WRITE"))
        )
        fsm.act("SETTLE",
            self.busy.eq(1),
            NextValue(timer, timer - 1),
            If(timer == 0, NextState("IDLE"))
        )

class GW5UPARPort:
    """UPAR user requests (addr/wrdata/we, done) of a GW5RateSequencer/GW5TXElectricalIdle."""
    def __init__(self):
        self.addr   = Signal(24)
        self.wrdata = Signal(32)
        self.we     = Signal()
        self.done   = Signal()

class GW5TXElectricalIdle(LiteXModule):
    """GTR12 lane TX Electrical Idle: the TX driver control register (addr) is written with
    idle_value/active_value through UPAR on each change of idle (asynchronous input), the last
    requested value written again when changed during a write, and on refresh (ex: after a lane
    reconfiguration that reset the register)."""
    def __init__(self, upar, addr, idle_value, active_value):
        self.idle    = Signal() # i: Asynchronous.
        self.refresh = Signal() # i: Write the current value again (pulse).

        # # #

        idle    = Signal(reset=1)
        written = Signal()
        current = Signal()
        self.specials += MultiReg(self.idle, idle, reset=1)
        self.comb += [
            upar.addr.eq(addr),
            upar.wrdata.eq(Mux(current, idle_value, active_value)),
        ]
        # The register is written at start (written != idle at reset).
        self.fsm = fsm = FSM(reset_state="IDLE")
        refresh = Signal()
        self.sync += If(self.refresh, refresh.eq(1)).Elif(fsm.ongoing("WRITE"), refresh.eq(0))
        fsm.act("IDLE",
            If((idle != written) | refresh,
                NextValue(current, idle),
                NextState("WRITE")
            )
        )
        fsm.act("WRITE",
            upar.we.eq(1),
            If(upar.done,
                NextValue(written, current),
                NextState("IDLE")
            )
        )

class GW5USB3SerDes(USB3SerDes):
    """Gowin GW5AST-138B/GW5AT-15 GTR12 USB3 SerDes (GW5AST-138B: Q1 lane 0/1, SFP0/SFP1 of the Tang
    Mega 138K Pro dock, 100 MHz REFCLK1 or 125 MHz REFCLK0; GW5AT-15: Q0 lane 0, 125 MHz REFCLK0,
    single quad devices).

    5Gbps, raw 20-bit PMA with 40-bit fabric words (125MHz, 4 symbols per cycle), 8b/10b and
    polarity in the fabric (LiteICLink GTR12), comma alignment by the PCS. The register
    configuration is generated from usb3_pipe/gw5 TOML files with Gowin's tools.

    gen2_capable: 10Gbps selected with `rate_10g` (Gen 2, GW5AST-138B Q1 lanes): the rate dependent
    registers (CDR, QPLL1 output divider, TX lanes clock tree, TX/RX gear FIFOs: 1:2 -> 1:4) are
    written through UPAR with the lane held in reset (GW5RateSequencer), 80-bit raw words at 125MHz
    (Gen 2 raw 128b/132b blocks), same fabric clocks at both rates.

    TX Electrical Idle: lane TX driver control register written through UPAR (GW5TXElectricalIdle),
    LFPS square wave in the raw TX words while idle (bursts made by the driver control).
    rx_idle_source: RX Electrical Idle indicator of the lane ("~astat5": analog status bit 5
    inverted, set while a signal is detected (LFPS bursts toggle it, measured on hardware),
    "elecidle": RXELECIDLE, "[~]astat<n>"), with_upar: SerDes registers runtime access (required),
    upar_clk: UPAR (and quad AHB) clock, xcvr_cd: clock domain of the GTR12 control logic
    (xcvr_clk_freq: its frequency, default sys_clk_freq)
    (free-running, see the Tang Mega 138K Pro target: SerDes datapath on the GTR12 TX clock), pll:
    lane PLL of the register configuration (default: QPLL1 on the GW5AST-138B, QPLL0 on the GW5AT-15
    as Gowin's USB 3.0 PHY IP).
    """
    def __init__(self, platform, sys_clk, sys_clk_freq, lane=0, rx_idle_source="~astat5",
        with_upar=True, upar_clk=None, xcvr_cd="sys", refclk="100M", csr_config=None, tx_cdc=True,
        position=None, pll=None, gen2_capable=False, xcvr_clk_freq=None):
        USB3SerDes.__init__(self)
        assert with_upar # TX Electrical Idle through UPAR.
        self.gen2_capable = gen2_capable
        if gen2_capable:
            self.gen2_data_width = 80
            self.gen2_tx_data    = Signal(80) # i: Raw TX words (tx clock domain, bit 0 first).
            self.gen2_rx_data    = Signal(80) # o: Raw RX words (rx clock domain, bit 0 first).

        # # #

        from liteiclink.serdes.gtr12_gw5 import GTR12, gtr12_primitives
        from usb3_pipe.gw5 import gw5_usb3_csr_config, gw5_usb3_rate_writes
        from usb3_pipe.gw5 import gw5_tx_idle_reg, GW5_TX_IDLE_VALUE, GW5_TX_ACTIVE_VALUE
        from usb3_pipe.gw5 import GW5_TX_LEVEL_REG, GW5_TX_LEVEL_DEFAULT, GW5_TX_CURSORS_REG
        from usb3_pipe.gw5 import GW5_TX_CURSORS_DEFAULT, GW5_TX_FFE_MANUAL_REG
        from usb3_pipe.gw5 import GW5_TX_FFE_MANUAL_VALUE, GW5_RX_CTLE_BOOST_REG, GW5_RX_CTLE_ATT_REG

        # Quad position: Q0 on single quad devices (GW5AT-15), Q1 on the GW5AST-138B.
        if position is None:
            quad, _  = gtr12_primitives(platform.devicename)
            position = "Q0" if quad == "GTR12_QUADB" else "Q1"
        if pll is None:
            pll = "QPLL0" if position == "Q0" else "QPLL1"
        assert (not gen2_capable) or (position == "Q1") # 10G rate writes: Q1 lanes.

        # Transceiver ------------------------------------------------------------------------------
        if csr_config is None:
            csr_config = gw5_usb3_csr_config(lane, "5G", refclk, position=position, pll=pll)
        gtr12 = GTR12(platform,
            line_rate   = 5e9,
            csr_config  = csr_config,
            data_width  = 40,
            raw_width   = 80 if gen2_capable else 40,
            position    = position,
            lane        = lane,
            pll         = pll,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity,
            csr_name    = f"usb3_5g_ln{lane}.csr",
            with_upar   = with_upar,
            upar_clk    = upar_clk)
        # GTR12 control logic (PLL lock resynchronization, resets, UPAR) in xcvr_cd: a free-running
        # clock domain when this SerDes "sys" clock domain is renamed to the GTR12 TX clock.
        if xcvr_cd != "sys":
            gtr12 = ClockDomainsRenamer({"sys": xcvr_cd})(gtr12)
        # No RX running disparity check (RX paths at 125MHz on the GW5A fabric), TX datapath input
        # register (PIPE TX multiplexers at 125MHz on the GW5A fabric).
        self.add_xcvr_datapath("gtr12", gtr12, data_width=40, rx_disparity_check=False,
            tx_cdc=tx_cdc, tx_input_register=True)

        # TX Electrical Idle (UPAR writes of the lane TX driver control register): no TX Electrical
        # Idle from the fabric data (constant words: DC level on the AC coupled link, seen on
        # hardware as LFPS bursts lengthened/merged by the link partner, LBPMs not decoded).
        idle_port = GW5UPARPort()
        self.tx_electrical_idle = tx_electrical_idle = ClockDomainsRenamer({"sys": xcvr_cd})(
            GW5TXElectricalIdle(idle_port, gw5_tx_idle_reg(lane, position), GW5_TX_IDLE_VALUE,
                GW5_TX_ACTIVE_VALUE))
        self.comb += tx_electrical_idle.idle.eq(self.tx_idle & ~self.tx_lfps)

        # Gen 2: rate changes (UPAR registers writes, lane held in reset), raw words ---------------
        if gen2_capable:
            rate_port = GW5UPARPort()
            writes = gw5_usb3_rate_writes(lane, refclk, pll=pll,
                clk_freq=sys_clk_freq if xcvr_clk_freq is None else xcvr_clk_freq)
            # TX FFE at 10Gbps (runtime, bring-up: applied at the next rate change, lane 0
            # registers): tx_ffe_manual: amplitude (txlev), post-cursor (c1) and pre-cursor (cm)
            # values, else the configuration defaults (5G values, as read on hardware).
            self.tx_ffe_manual = Signal()  # i
            self.tx_ffe_txlev  = Signal(4) # i
            self.tx_ffe_c1     = Signal(4) # i
            self.tx_ffe_cm     = Signal(4) # i
            if lane == 0:
                txlev = Signal(32)
                c1cm  = Signal(32)
                ffe   = Signal(32)
                self.comb += [
                    txlev.eq(Mux(self.tx_ffe_manual, Cat(C(0, 12), self.tx_ffe_txlev),
                        GW5_TX_LEVEL_DEFAULT)),
                    c1cm.eq(Mux(self.tx_ffe_manual,
                        Cat(self.tx_ffe_cm, C(0, 4), self.tx_ffe_c1),
                        GW5_TX_CURSORS_DEFAULT)),
                    ffe.eq(Mux(self.tx_ffe_manual, GW5_TX_FFE_MANUAL_VALUE, 0)),
                ]
                writes = writes[:1] + [
                    (GW5_TX_LEVEL_REG,      GW5_TX_LEVEL_DEFAULT,   txlev),
                    (GW5_TX_CURSORS_REG,    GW5_TX_CURSORS_DEFAULT, c1cm),
                    (GW5_TX_FFE_MANUAL_REG, 0,                      ffe),
                ] + writes[1:]
            # RX CTLE at 10Gbps (runtime, bring-up: applied at the next rate change, lane 0
            # registers): rx_eq_manual: attenuation (att) and boost values, else the configuration
            # values (gw5_usb3_rate_writes).
            self.rx_eq_manual = Signal()  # i
            self.rx_eq_att    = Signal(4) # i
            self.rx_eq_boost  = Signal(4) # i
            if lane == 0:
                def eq_value(default, value):
                    v = Signal(32)
                    self.comb += v.eq(Mux(self.rx_eq_manual,
                        Cat(C(1, 4), Replicate(value, 4), C(0, 12)), default))
                    return v
                rx_eq = {GW5_RX_CTLE_ATT_REG: self.rx_eq_att, GW5_RX_CTLE_BOOST_REG: self.rx_eq_boost}
                writes = [(w[0], w[1], eq_value(w[2], rx_eq[w[0]]) if w[0] in rx_eq else w[2]) +
                    tuple(w[3:]) for w in writes]
            self.rate_sequencer = rate_sequencer = ClockDomainsRenamer({"sys": xcvr_cd})(
                GW5RateSequencer(rate_port, writes))
            # TX driver control register reset by the lane reconfiguration: written again after.
            busy_d = Signal()
            sync   = getattr(self.sync, xcvr_cd)
            sync  += busy_d.eq(rate_sequencer.busy)
            self.comb += tx_electrical_idle.refresh.eq(busy_d & ~rate_sequencer.busy)
            # TX re-initialization request (bring-up: TX PCS reset without a lane reconfiguration).
            self.tx_reinit_req = Signal() # i
            self.comb += [
                rate_sequencer.rate_10g.eq(self.rate_10g),
                self.tx_reinit.eq(rate_sequencer.busy | self.tx_reinit_req),
                self.rx_reinit.eq(rate_sequencer.busy),
                self.gen2_rx_data.eq(gtr12.rx_raw),
            ]
            # UPAR user requests: rate sequencer while busy (its writes start after the lane reset
            # delay: a TX Electrical Idle write in progress ends before, rewritten if cut).
            ports = [(rate_sequencer.busy, rate_port), (~rate_sequencer.busy, idle_port)]
        else:
            ports = [(1, idle_port)]
        for sel, port in ports:
            self.comb += [
                If(sel,
                    gtr12.upar.addr.eq(port.addr),
                    gtr12.upar.wrdata.eq(port.wrdata),
                    gtr12.upar.we.eq(port.we),
                ),
                port.done.eq(sel & gtr12.upar.done),
            ]

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
        # TX LFPS: bit-level square wave with a 24ns period (41.7MHz, Table 6-29: 20-80ns for
        # SuperSpeedPlus) through the raw TX words whenever the transmitter is idle, the bursts made
        # by the TX Electrical Idle register (the square wave is already running when the driver is
        # enabled and still running when disabled: same UPAR write latency on both burst edges).
        # Measured on hardware (SFP2USB, xHCI Gen 2 host): Polling LFPS/SCDs with the PIPE LFPS
        # pattern (constant words: 48ns period) not received, received with 20-28ns periods.
        self.add_gw5_lfps_tx(gtr12, raw_width=80 if gen2_capable else 40)

        # Timing constraints -----------------------------------------------------------------------
        self.add_timing_constraints(platform, sys_clk, gtr12)

    def add_gw5_lfps_tx(self, gtr12, raw_width, half_period=12e-9):
        """LFPS square wave as raw TX words while idle or in LFPS bursts (tx clock domain):
        half_period in bits at 5Gbps (40-bit words) or 10Gbps (80-bit words, rate_10g)."""
        def lfps_words(bit_rate, word_bits):
            half   = int(half_period*bit_rate)
            period = 2*half
            n      = 1
            while (word_bits*n) % period:
                n += 1
            bits = [int((i % period) < half) for i in range(word_bits*n)]
            return [sum(b << j for j, b in enumerate(bits[word_bits*k:word_bits*(k + 1)]))
                for k in range(n)]
        words_5g = lfps_words(5e9, 40)
        words    = [words_5g]
        if raw_width == 80:
            words_10g = lfps_words(10e9, 80)
            assert len(words_10g) == len(words_5g)
            words.append(words_10g)

        lfps_tx   = Signal()
        rate_10g  = Signal()
        index     = Signal(max=len(words_5g))
        lfps_word = Signal(raw_width)
        self.specials += [
            MultiReg(self.tx_lfps | self.tx_idle, lfps_tx, "tx"),
            MultiReg(self.rate_10g, rate_10g, "tx"),
        ]
        self.sync.tx += [
            If(index == len(words_5g) - 1,
                index.eq(0)
            ).Else(
                index.eq(index + 1)
            ),
            lfps_word.eq(Array(Constant(w, raw_width) for w in words_5g)[index]),
        ]
        if raw_width == 80:
            self.sync.tx += If(rate_10g,
                lfps_word.eq(Array(Constant(w, raw_width) for w in words[1])[index])
            )
        self.comb += [
            gtr12.tx_raw_enable.eq(lfps_tx | self.gen2_tx_enable),
            gtr12.tx_raw.eq(Mux(lfps_tx, lfps_word, self.gen2_tx_data if raw_width == 80 else 0)),
        ]

# Lattice ECP5 USB3 Serializer/Deserializer --------------------------------------------------------

class ECP5USB3SerDes(USB3SerDes):
    """Lattice ECP5-5G DCU USB3 SerDes (LiteICLink SerDesECP5: raw 10BSER DCU channel, 40-bit
    125MHz fabric words, fabric 8b/10b/comma alignment).

    refclk/refclk_freq: DCU reference clock (fabric clock: 250MHz x20 for 5Gbps, ex: ECPIX-5, the
    100MHz EXTREF0 input only reaches 2.5Gbps with the x25 multiplier). TX Electrical Idle from the
    DCU FFC_EI_EN input, RX Electrical Idle from the RX loss of signal detector, DCU RX settings for
    USB3 (rx_settings).

    xcvr_cd: clock domain of the DCU control logic (initialization, resets, SCI): a free-running
    clock domain when this SerDes "sys" clock domain is renamed to the DCU TX clock (PIPE on the TX
    clock, LTSSM/Link Layer/Core at a lower frequency: USB3PHY pipe_cd), tx_cdc: TX datapath clock
    domain crossing (no CDC with the SerDes logic on the DCU TX clock).
    """
    # DCU RX settings (over LiteICLink's SerDesECP5 ones), from LUNA's ECP5 SerDes PHY (Great Scott
    # Gadgets, BSD-3-Clause, luna/gateware/interface/serdes_phy/ecp5.py) and the ECPIX-5 bring-up:
    # the CDR/DCO calibration and lock window are the RX fix with the host SSC (host TSEQ word
    # errors 1.4e-1 -> ~1e-4), the equalizer settings are within the run-to-run spread.
    rx_settings = dict(
        p_CHX_RXTERM_CM         = "0b10",  # RX termination to GND (USB3 receiver detection).
        p_CHX_REQ_EN            = "0b1",   # Equalizer enabled.
        p_CHX_REQ_LVL_SET       = "0b00",  # Equalizer attenuation.
        p_CHX_RX_RATE_SEL       = "0d06",  # Equalizer pole position.
        p_CHX_PDEN_SEL          = "0b0",   # CDR phase detector always enabled (not disabled on LOS).
        p_CHX_DCOCALDIV         = "0b010", # DCO calibration.
        p_CHX_DCODISBDAVOID     = "0b1",
        p_CHX_DCONUOFLSB        = "0b101",
        p_CHX_BAND_THRESHOLD    = "0d0",
        p_CHX_CALIB_CK_MODE     = "0b1",
        p_D_DCO_CALIB_TIME_SEL  = "0b00",
        p_D_CDR_LOL_SET         = "0b10",  # +-4000ppm lock, +-7000ppm unlock (SSC).
        p_CHX_RX_LOS_HYST_EN    = "0b0",   # No loss of signal hysteresis.
        i_CHX_FFC_SIGNAL_DETECT = 0,       # DCU word alignment unused (fabric alignment).
        p_CHX_ENABLE_CG_ALIGN   = "0b0",
    )

    def __init__(self, platform, sys_clk, sys_clk_freq, refclk, refclk_freq, tx_pads, rx_pads,
        dual=0, channel=0, xcvr_cd="sys", tx_cdc=True):
        USB3SerDes.__init__(self)

        # # #

        from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

        # Transceiver ------------------------------------------------------------------------------
        self.pll = pll = SerDesECP5PLL(refclk, refclk_freq, linerate=5e9)
        dcu = SerDesECP5(pll, tx_pads, rx_pads,
            dual        = dual,
            channel     = channel,
            data_width  = 40,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity,
            with_oob    = True) # TX up without RX signal (LFPS), fabric alignment, FFC_EI_EN.
        dcu.serdes_params.update(self.rx_settings)
        if xcvr_cd != "sys":
            dcu = ClockDomainsRenamer({"sys": xcvr_cd})(dcu)
        # No RX running disparity check, TX datapath input register (125MHz on the ECP5 fabric),
        # 8b/10b checker lookups in logic (no 1024 entries distributed RAM on ECP5).
        self.add_xcvr_datapath("dcu", dcu, data_width=40, rx_disparity_check=False,
            tx_cdc=tx_cdc, tx_input_register=True, rx_rom_style="logic")

        # LFPS -------------------------------------------------------------------------------------
        self.add_lfps(dcu, dcu.rx_idle)
        self.comb += dcu.tx_idle.eq(self.tx_idle & ~self.tx_lfps)

        # Timing constraints (DCU clocks at linerate/20, CLKDIVF /2 for the 40-bit words) ----------
        platform.add_period_constraint(dcu.txoutclk, 1e9/(2*dcu.tx_clk_freq))
        platform.add_period_constraint(dcu.rxoutclk, 1e9/(2*dcu.rx_clk_freq))
        platform.add_false_path_constraints(sys_clk, dcu.txoutclk, dcu.rxoutclk)
