#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2019-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Lattice ECP5-5G DCU transceiver for USB3 (5Gbps) with 40-bit/125MHz fabric words.

The DCU channel runs as a raw 10BSER interface (8b/10b, comma alignment and polarity in the fabric)
with its 1:2 gearbox: 20-bit words at 250MHz on the DCU clocks. Only the TX/RX gearboxes run at
250MHz ("tx_full"/"rx_full", reset-less), everything else (8b/10b, comma alignment, the USB3 TX/RX
datapaths) runs at 125MHz ("tx"/"rx", CLKDIVF /2 of the DCU clocks): the ECP5 fabric does not
reach 250MHz with the 8b/10b and alignment logic.

The DCU parameters are LiteICLink's SerDesECP5 ones (documented by whitequark as part of the
Yumewatari project: https://github.com/whitequark/Yumewatari) with the 5Gbps bring-up changes
(ECPIX-5): CDR phase detector always enabled (CHX_PDEN_SEL: disabled on LOS otherwise, the CDR then
doesn't track the data in internal loopback), separate TX/RX initialization (TX up without RX
signal), TX Electrical Idle from the FFC_EI_EN input, and the RX settings of LUNA's ECP5 SerDes
PHY (Great Scott Gadgets, BSD-3-Clause, luna/gateware/interface/serdes_phy/ecp5.py): RX termination
to GND, CDR/DCO calibration and lock window (the main RX fix with the host SSC on hardware: host
TSEQ word errors 1.4e-1 -> ~1e-4), equalizer enabled (level/pole: sweep results within the
run-to-run spread, 4e-5 to 8e-4).

Interface: LiteICLink transceiver (tx_enable/tx_ready, rx_enable/rx_ready, rx_align, encoder/
decoders, tx_produce_pattern/tx_pattern, cd_tx/cd_rx, tx_clk_freq/rx_clk_freq, add_stream_endpoints)
used by USB3SerDes.add_xcvr_datapath/add_lfps, plus tx_idle/rx_los (LFPS) and SCI access.
"""

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

# ECP5 DCU PLL -------------------------------------------------------------------------------------

class ECP5DCUPLL:
    """DCU TX PLL configuration: refclk x m (D_REFCK_MODE) / d (D_TX_VCO_CK_DIV/CHX_RX_DCO_CK_DIV).

    The line rate is refclk x m / d (measured on hardware): 5Gbps from a 250MHz refclk x20 (fabric
    PLL on the ECPIX-5), 2.5Gbps from 100MHz x25.
    """
    multipliers = {25: "0b100", 20: "0b000", 16: "0b010", 10: "0b001", 8: "0b011"}
    dividers    = {32: "0b111", 16: "0b110", 8: "0b101", 4: "0b100", 2: "0b010", 1: "0b000"}

    def __init__(self, refclk, refclk_freq, linerate=5e9):
        self.refclk      = refclk
        self.refclk_freq = refclk_freq
        self.linerate    = linerate
        for d in self.dividers:
            for m in self.multipliers:
                if refclk_freq*m/d == linerate:
                    self.m, self.d = m, d
                    return
        raise ValueError(f"No DCU PLL config for {refclk_freq/1e6:.3f}MHz refclk / {linerate/1e9:.3f}Gbps.")

# ECP5 TX Gearbox ----------------------------------------------------------------------------------

class ECP5TXGearbox(LiteXModule):
    """40-bit "half" words (bits 0-19 first) to 20-bit "full" words (full: 2x half clock, frequency/
    phase related). The half clock domain phase is recovered from a toggle sampled in full: the first
    full cycle with a new half word transmits its LSBs."""
    def __init__(self, cd_full="full", cd_half="half"):
        self.sink   = Signal(40) # i (half)
        self.source = Signal(20) # o (full)

        # # #

        toggle = Signal()
        sync_half = getattr(self.sync, cd_half)
        sync_full = getattr(self.sync, cd_full)
        sync_half += toggle.eq(~toggle)

        word     = Signal(40)
        toggle_r = Signal()
        toggle_d = Signal()
        sync_full += [
            word.eq(self.sink),
            toggle_r.eq(toggle),
            toggle_d.eq(toggle_r),
            If(toggle_r != toggle_d,
                self.source.eq(word[:20])
            ).Else(
                self.source.eq(word[20:])
            )
        ]

# ECP5 RX Gearbox ----------------------------------------------------------------------------------

class ECP5RXGearbox(LiteXModule):
    """20-bit "full" words to 40-bit "half" words (first full word in bits 0-19). The 40-bit word is
    updated in the full cycle following a half clock edge (stable for the next half clock edge)."""
    def __init__(self, cd_full="full", cd_half="half"):
        self.sink   = Signal(20) # i (full)
        self.source = Signal(40) # o (half)

        # # #

        toggle = Signal()
        sync_half = getattr(self.sync, cd_half)
        sync_full = getattr(self.sync, cd_full)
        sync_half += toggle.eq(~toggle)

        data     = Signal(20)
        data_d   = Signal(20)
        word     = Signal(40)
        toggle_r = Signal()
        toggle_d = Signal()
        sync_full += [
            data.eq(self.sink),
            data_d.eq(data),
            toggle_r.eq(toggle),
            toggle_d.eq(toggle_r),
            If(toggle_r != toggle_d,
                word.eq(Cat(data_d, data))
            )
        ]
        sync_half += self.source.eq(word)

# ECP5 RX Comma Aligner ----------------------------------------------------------------------------

class ECP5RXCommaAligner(LiteXModule):
    """10-bit symbols alignment of the 40-bit raw RX words (LSB first) on the commas (K28.1/K28.5/K28.7,
    bits a-g 0011111/1100000).

    The bit offset (0-9) moves when commas are found at another offset in 2 consecutive words with a
    comma (enable: alignment allowed). The symbols rotation within the 40-bit words is done by the
    USB3 RX datapath (RXWordAligner).
    """
    def __init__(self):
        self.enable  = Signal(reset=1) # i
        self.sink    = Signal(40)      # i
        self.source  = Signal(40)      # o (2 cycles latency)
        self.offset  = Signal(4)       # o
        self.aligned = Signal()        # o: comma(s) received at the current offset.

        # # #

        data   = Signal(40)
        data_d = Signal(40)
        window = Signal(80)
        self.sync += [
            data.eq(self.sink),
            data_d.eq(data),
        ]
        self.comb += window.eq(Cat(data_d, data))

        # Commas detection at each bit offset (4 symbols per word).
        found = Signal(10)
        for o in range(10):
            matches = []
            for k in range(4):
                bits = window[o + 10*k:o + 10*k + 7]
                matches.append((bits == 0b1111100) | (bits == 0b0000011))
            self.sync += found[o].eq(Reduce("OR", matches))

        # Offset selection.
        window_d  = Signal(80)
        candidate = Signal(4)
        pending   = Signal()
        current   = Signal()
        self.sync += window_d.eq(window)
        self.comb += current.eq(Array(found)[self.offset])
        others = Signal(4)
        self.comb += others.eq(0)
        for o in reversed(range(10)):
            self.comb += If(found[o], others.eq(o))
        self.sync += [
            If(found != 0,
                self.aligned.eq(current),
                If(~current & self.enable,
                    If(pending & (others == candidate),
                        self.offset.eq(candidate),
                        pending.eq(0),
                    ).Else(
                        candidate.eq(others),
                        pending.eq(1),
                    )
                ).Else(
                    pending.eq(0),
                )
            )
        ]

        # Shift.
        self.sync += Case(self.offset, {o: self.source.eq(window_d[o:o + 40]) for o in range(10)})

# ECP5 DCU -----------------------------------------------------------------------------------------

class ECP5DCU(LiteXModule):
    """ECP5-5G DCU channel, raw 10BSER, 40-bit/125MHz fabric words with fabric 8b/10b (see module
    docstring). DCU initialization: LiteICLink SerdesInitOOB (TX up first, RX after signal/CDR lock).
    rx_ldr: DCU low data rate input (raw RX input value, LFPS/activity observations), rx_rst_release:
    RX resets release without the initialization (bring-up, ex: internal loopback without RX signal)."""
    def __init__(self, pll, tx_pads, rx_pads, dual=0, channel=0, tx_polarity=0, rx_polarity=0,
        data_width=40):
        from liteiclink.serdes.serdes_ecp5 import SerdesInitOOB

        assert data_width == 40
        assert dual    in [0, 1]
        assert channel in [0, 1]
        self.dual       = dual
        self.channel    = channel
        self.nwords     = nwords = data_width//10
        self.data_width = data_width

        # TX controls.
        self.tx_enable          = Signal(reset=1) # i
        self.tx_ready           = Signal()        # o
        self.tx_idle            = Signal()        # i: TX Electrical Idle.
        self.tx_produce_pattern = Signal()        # i
        self.tx_pattern         = Signal(data_width)

        # RX controls.
        self.rx_enable  = Signal(reset=1) # i
        self.rx_ready   = Signal()        # o
        self.rx_align   = Signal(reset=1) # i
        self.rx_los     = Signal()        # o: RX loss of signal (async).
        self.rx_ldr     = Signal()        # o: RX low data rate input (raw RX input value, async).
        self.rx_raw     = Signal(data_width) # o: Aligned raw RX words (rx clock domain).
        self.rx_aligned = Signal()        # o: Commas at the current alignment (rx clock domain).
        self.rx_rst_release = Signal()    # i: Release the RX PCS reset (bring-up, bypass the init).

        # Loopback (TX to RX near-end serial loopback, SCI lb_ctl, see add_sci).
        self.loopback = Signal()

        # Status (sys).
        self.tx_lol = Signal()
        self.rx_lol = Signal()

        self.tx_clk_freq = self.rx_clk_freq = pll.linerate/data_width

        # # #

        self.encoder  = ClockDomainsRenamer("tx")(Encoder(nwords, True))
        self.decoders = [ClockDomainsRenamer("rx")(Decoder(True)) for _ in range(nwords)]
        self.submodules += self.decoders

        # Internal signals -------------------------------------------------------------------------
        self.txoutclk = txoutclk = Signal()
        self.rxoutclk = rxoutclk = Signal()
        tx_lol = Signal()
        rx_lol = Signal()
        rx_los = Signal()
        rx_bus = Signal(24)
        tx_bus = Signal(24)
        self.comb += self.rx_los.eq(rx_los)

        # Init -------------------------------------------------------------------------------------
        self.init = init = SerdesInitOOB(tx_lol, rx_lol, rx_los)
        self.comb += [
            self.tx_ready.eq(init.tx_ready),
            self.rx_ready.eq(init.rx_ready),
            self.tx_lol.eq(init.tx_lol),
            self.rx_lol.eq(init.rx_lol),
        ]

        # Clocking ---------------------------------------------------------------------------------
        # DCU clocks (250MHz) for the gearboxes, CLKDIVF /2 (125MHz) for the fabric words.
        self.cd_tx_full = ClockDomain(reset_less=True)
        self.cd_rx_full = ClockDomain(reset_less=True)
        self.cd_tx      = ClockDomain()
        self.cd_rx      = ClockDomain()
        self.comb += [
            self.cd_tx_full.clk.eq(txoutclk),
            self.cd_rx_full.clk.eq(rxoutclk),
        ]
        for name, clki, clko in [("tx", txoutclk, self.cd_tx.clk), ("rx", rxoutclk, self.cd_rx.clk)]:
            self.specials += Instance("CLKDIVF",
                name      = f"{name}_clkdivf",
                p_DIV     = "2.0",
                i_CLKI    = clki,
                i_RST     = 0,
                i_ALIGNWD = 0,
                o_CDIVX   = clko,
            )
        self.specials += [
            AsyncResetSynchronizer(self.cd_tx, ~init.tx_ready),
            AsyncResetSynchronizer(self.cd_rx, ~init.rx_ready & ~self.rx_rst_release),
        ]

        # TX Data ----------------------------------------------------------------------------------
        tx_polarity_tx        = Signal()
        tx_produce_pattern_tx = Signal()
        tx_pattern_tx         = Signal(data_width)
        self.specials += [
            MultiReg(tx_polarity,             tx_polarity_tx,        "tx"),
            MultiReg(self.tx_produce_pattern, tx_produce_pattern_tx, "tx"),
            MultiReg(self.tx_pattern,         tx_pattern_tx,         "tx"),
        ]
        tx_data = Signal(data_width)
        self.sync.tx += [
            If(tx_produce_pattern_tx,
                tx_data.eq(tx_pattern_tx)
            ).Else(
                tx_data.eq(Cat(*[self.encoder.output[i] for i in range(nwords)]))
            ),
        ]
        tx_data_pol = Signal(data_width)
        self.sync.tx += tx_data_pol.eq(tx_data ^ Replicate(tx_polarity_tx, data_width))
        self.tx_gearbox = tx_gearbox = ECP5TXGearbox(cd_full="tx_full", cd_half="tx")
        self.comb += [
            tx_gearbox.sink.eq(tx_data_pol),
            tx_bus[ 0:10].eq(tx_gearbox.source[ 0:10]),
            tx_bus[12:22].eq(tx_gearbox.source[10:20]),
        ]

        # RX Data ----------------------------------------------------------------------------------
        rx_polarity_rx = Signal()
        rx_align_rx    = Signal()
        self.specials += [
            MultiReg(rx_polarity, rx_polarity_rx, "rx"),
            MultiReg(self.rx_align, rx_align_rx, "rx"),
        ]
        self.rx_gearbox = rx_gearbox = ECP5RXGearbox(cd_full="rx_full", cd_half="rx")
        self.rx_aligner = rx_aligner = ClockDomainsRenamer("rx")(ECP5RXCommaAligner())
        rx_data = Signal(data_width)
        self.comb += rx_gearbox.sink.eq(Cat(rx_bus[0:10], rx_bus[12:22]))
        self.sync.rx += rx_data.eq(rx_gearbox.source ^ Replicate(rx_polarity_rx, data_width))
        self.comb += [
            rx_aligner.enable.eq(rx_align_rx),
            rx_aligner.sink.eq(rx_data),
            self.rx_raw.eq(rx_aligner.source),
            self.rx_aligned.eq(rx_aligner.aligned),
        ]
        for i in range(nwords):
            self.comb += self.decoders[i].input.eq(rx_aligner.source[10*i:10*(i+1)])

        # TX Electrical Idle -----------------------------------------------------------------------
        tx_idle = Signal()
        self.specials += MultiReg(self.tx_idle, tx_idle, "tx")

        # DCU instance -----------------------------------------------------------------------------
        self.serdes_params = dict(
            # DCU — power management
            p_D_MACROPDB            = "0b1",
            p_D_IB_PWDNB            = "0b1",    # undocumented (required for RX)
            p_D_TXPLL_PWDNB         = "0b1",
            i_D_FFC_MACROPDB        = 1,

            # DCU — reset
            i_D_FFC_MACRO_RST       = ResetSignal("sys"),
            i_D_FFC_DUAL_RST        = ResetSignal("sys"),

            # DCU — clocking
            i_D_REFCLKI             = pll.refclk,
            o_D_FFS_PLOL            = tx_lol,
            p_D_REFCK_MODE          = ECP5DCUPLL.multipliers[pll.m],
            p_D_TX_MAX_RATE         = "5.0",    # 5.0 Gbps
            p_D_TX_VCO_CK_DIV       = ECP5DCUPLL.dividers[pll.d],
            p_D_BITCLK_LOCAL_EN     = "0b1",    # Use clock from local PLL
            p_D_SYNC_LOCAL_EN       = "0b1",    # Enable the local TX gearbox clock.

            # DCU — unknown
            p_D_CMUSETBIASI         = "0b00",   # begin undocumented (10BSER sample code used)
            p_D_CMUSETI4CPP         = "0d3",
            p_D_CMUSETI4CPZ         = "0d3",
            p_D_CMUSETI4VCO         = "0b00",
            p_D_CMUSETICP4P         = "0b01",
            p_D_CMUSETICP4Z         = "0b101",
            p_D_CMUSETINITVCT       = "0b00",
            p_D_CMUSETISCL4VCO      = "0b000",
            p_D_CMUSETP1GM          = "0b000",
            p_D_CMUSETP2AGM         = "0b000",
            p_D_CMUSETZGM           = "0b000",
            p_D_SETIRPOLY_AUX       = "0b01",
            p_D_SETICONST_AUX       = "0b01",
            p_D_SETIRPOLY_CH        = "0b01",
            p_D_SETICONST_CH        = "0b10",
            p_D_SETPLLRC            = "0d1",
            p_D_RG_EN               = "0b0",
            p_D_RG_SET              = "0b00",
            p_D_REQ_ISET            = "0b011",
            p_D_PD_ISET             = "0b11",   # end undocumented

            # DCU — FIFOs
            p_D_LOW_MARK            = "0d4",    # Clock compensation FIFO low  water mark (mean=8)
            p_D_HIGH_MARK           = "0d12",   # Clock compensation FIFO high water mark (mean=8)

            # CHX common ---------------------------------------------------------------------------
            # CHX — protocol
            p_CHX_PROTOCOL          = "10BSER",
            p_CHX_UC_MODE           = "0b1",
            p_CHX_ENC_BYPASS        = "0b1",    # Bypass 8b10b encoder
            p_CHX_DEC_BYPASS        = "0b1",    # Bypass 8b10b decoder

            # CHX receive --------------------------------------------------------------------------
            # CHX RX — power management
            p_CHX_RPWDNB            = "0b1",
            i_CHX_FFC_RXPWDNB       = 1,

            # CHX RX — reset
            i_CHX_FFC_RRST          = ~self.rx_enable | init.rx_cdr_rst,
            i_CHX_FFC_LANE_RX_RST   = (~self.rx_enable | init.rx_pcs_rst) & ~self.rx_rst_release,

            # CHX RX — input
            i_CHX_HDINP             = rx_pads.p,
            i_CHX_HDINN             = rx_pads.n,

            p_CHX_REQ_EN            = "0b1",    # Enable equalizer
            p_CHX_REQ_LVL_SET       = "0b00",   # Equalizer attenuation
            p_CHX_RX_RATE_SEL       = "0d06",   # Equalizer pole position
            p_CHX_RTERM_RX          = "0d19",   # 50 Ohms
            p_CHX_RXIN_CM           = "0b11",   # CMFB (wizard value used)
            p_CHX_RXTERM_CM         = "0b10",   # RX termination to GND (USB3 receiver detection)

            # CHX RX ­— clocking
            i_CHX_RX_REFCLK         = pll.refclk,
            o_CHX_FF_RX_PCLK        = rxoutclk,
            i_CHX_FF_RXI_CLK        = ClockSignal("rx_full"),

            p_CHX_CDR_MAX_RATE      = "5.0",    # 5.0 Gbps
            p_CHX_RX_DCO_CK_DIV     = ECP5DCUPLL.dividers[pll.d],
            p_CHX_RX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_RX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_RX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock
            p_CHX_SEL_SD_RX_CLK     = "0b1",    # FIFO driven by recovered clock

            p_CHX_AUTO_FACQ_EN      = "0b1",    # undocumented (wizard value used)
            p_CHX_AUTO_CALIB_EN     = "0b1",    # undocumented (wizard value used)
            p_CHX_PDEN_SEL          = "0b0",    # phase detector always enabled (not disabled on LOS)

            p_CHX_DCOATDCFG         = "0b00",   # begin undocumented (sample code used)
            p_CHX_DCOATDDLY         = "0b00",
            p_CHX_DCOBYPSATD        = "0b1",
            p_CHX_DCOCALDIV         = "0b010",
            p_CHX_DCOCTLGI          = "0b011",
            p_CHX_DCODISBDAVOID     = "0b1",
            p_CHX_DCOFLTDAC         = "0b00",
            p_CHX_DCOFTNRG          = "0b001",
            p_CHX_DCOIOSTUNE        = "0b010",
            p_CHX_DCOITUNE          = "0b00",
            p_CHX_DCOITUNE4LSB      = "0b010",
            p_CHX_DCOIUPDNX2        = "0b1",
            p_CHX_DCONUOFLSB        = "0b101",
            p_CHX_DCOSCALEI         = "0b01",
            p_CHX_DCOSTARTVAL       = "0b010",
            p_CHX_DCOSTEP           = "0b11",   # end undocumented
            p_CHX_BAND_THRESHOLD    = "0d0",
            p_CHX_CALIB_CK_MODE     = "0b1",
            p_D_DCO_CALIB_TIME_SEL  = "0b00",
            p_D_CDR_LOL_SET         = "0b10",   # +-4000ppm lock, +-7000ppm unlock (SSC)

            # CHX RX — loss of signal
            o_CHX_FFS_RLOS          = rx_los,
            p_CHX_RLOS_SEL          = "0b1",
            p_CHX_RX_LOS_EN         = "0b1",
            p_CHX_RX_LOS_LVL        = "0b100",  # Lattice "TBD" (wizard value used)
            p_CHX_RX_LOS_CEQ        = "0b11",   # Lattice "TBD" (wizard value used)
            p_CHX_RX_LOS_HYST_EN    = "0b0",

            # CHX RX — low data rate input (raw RX input value to the fabric)
            p_CHX_LDR_RX2CORE_SEL   = "0b1",
            o_CHX_LDR_RX2CORE       = self.rx_ldr,

            # CHX RX — loss of lock
            o_CHX_FFS_RLOL          = rx_lol,

            # CHX RX — link state machine (disabled: alignment in the fabric)
            i_CHX_FFC_SIGNAL_DETECT = 0,
            p_CHX_ENABLE_CG_ALIGN   = "0b0",
            p_CHX_LSM_DISABLE       = "0b1",
            p_CHX_UDF_COMMA_MASK    = "0x3ff",  # compare all 10 bits
            p_CHX_UDF_COMMA_A       = "0x283",  # K28.5 inverted
            p_CHX_UDF_COMMA_B       = "0x17C",  # K28.5

            p_CHX_CTC_BYPASS        = "0b1",    # bypass CTC FIFO
            p_CHX_MIN_IPG_CNT       = "0b11",   # minimum interpacket gap of 4
            p_CHX_MATCH_2_ENABLE    = "0b0",    # 2 character skip matching
            p_CHX_MATCH_4_ENABLE    = "0b0",    # 4 character skip matching
            p_CHX_CC_MATCH_1        = "0x000",
            p_CHX_CC_MATCH_2        = "0x000",
            p_CHX_CC_MATCH_3        = "0x000",
            p_CHX_CC_MATCH_4        = "0x000",

            # CHX RX — data
            **{"o_CHX_FF_RX_D_%d" % n: rx_bus[n] for n in range(rx_bus.nbits)},

            # CHX transmit -------------------------------------------------------------------------
            # CHX TX — power management
            p_CHX_TPWDNB            = "0b1",
            i_CHX_FFC_TXPWDNB       = 1,

            # CHX TX — reset
            i_D_FFC_TRST            = ~self.tx_enable | init.tx_pll_rst,
            i_CHX_FFC_LANE_TX_RST   = ~self.tx_enable | init.tx_pcs_rst,

            # CHX TX — electrical idle
            i_CHX_FFC_EI_EN         = tx_idle,

            # CHX TX - output
            o_CHX_HDOUTP            = tx_pads.p,
            o_CHX_HDOUTN            = tx_pads.n,

            p_CHX_TXAMPLITUDE       = "0d1000", # 1000 mV
            p_CHX_RTERM_TX          = "0d19",   # 50 Ohms

            p_CHX_TDRV_SLICE0_CUR   = "0b011",  # 400 uA
            p_CHX_TDRV_SLICE0_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE1_CUR   = "0b000",  # 100 uA
            p_CHX_TDRV_SLICE1_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE2_CUR   = "0b11",   # 3200 uA
            p_CHX_TDRV_SLICE2_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE3_CUR   = "0b10",   # 2400 uA
            p_CHX_TDRV_SLICE3_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE4_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE4_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE5_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE5_SEL   = "0b00",   # power down

            # CHX TX — clocking
            o_CHX_FF_TX_PCLK        = txoutclk,
            i_CHX_FF_TXI_CLK        = ClockSignal("tx_full"),

            p_CHX_TX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_TX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_TX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock

            # CHX TX — data
            **{"i_CHX_FF_TX_D_%d" % n: tx_bus[n] for n in range(tx_bus.nbits)}
        )

    def add_sci(self):
        """SCI access (CSRs) and loopback (lb_ctl) from LiteICLink's SerDesECP5SCIReconfig."""
        from liteiclink.serdes.serdes_ecp5 import SerDesECP5SCIReconfig
        self.sci_reconfig = sci_reconfig = SerDesECP5SCIReconfig(self, tx_idle_via_sci=False)
        self.comb += [
            sci_reconfig.reset.eq(~self.tx_ready),
            sci_reconfig.sci.dual_sel.eq(self.dual),
            sci_reconfig.loopback.eq(self.loopback),
        ]

    def add_stream_endpoints(self):
        self.sink   = sink   = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.source = source = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.comb += [
            sink.ready.eq(1),
            source.valid.eq(1),
        ]
        for i in range(self.nwords):
            self.comb += [
                self.encoder.k[i].eq(sink.ctrl[i]),
                self.encoder.d[i].eq(sink.data[8*i:8*(i+1)]),
                source.ctrl[i].eq(self.decoders[i].k),
                source.data[8*i:8*(i+1)].eq(self.decoders[i].d),
            ]

    def do_finalize(self):
        serdes_params = {k.replace("CHX", f"CH{self.channel}"): v for k, v in self.serdes_params.items()}
        self.specials.dcu = Instance("DCUA", **serdes_params)
        self.dcu.attr.add(("LOC",  f"DCU{self.dual}"))
        self.dcu.attr.add(("CHAN", f"CH{self.channel}"))
