# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

from usb3_pipe.common import K, COM, SKP

# RX Skip Remover ----------------------------------------------------------------------------------

class RXSkipRemover(Module):
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.skip   = Signal()

        # # #

        # Find SKP symbols -------------------------------------------------------------------------
        skp = Signal(4)
        for i in range(4):
            self.comb += skp[i].eq(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == SKP.value))
        self.comb += self.skip.eq(self.sink.valid & self.sink.ready & (skp != 0))

        # Select valid Data/Ctrl fragments ---------------------------------------------------------
        frag_data  = Signal(32)
        frag_ctrl  = Signal(4)
        frag_bytes = Signal(3)
        cases = {}
        for i in range(2**4):
            datas = []
            ctrls = []
            for j in range(4):
                if (i & 2**j) == 0:
                    datas.append(sink.data[8*j:8*(j+1)])
                    ctrls.append(sink.ctrl[1*j:1*(j+1)])
            cases[i] = [
                frag_data.eq(Cat(*datas) if len(datas) else 0),
                frag_ctrl.eq(Cat(*ctrls) if len(ctrls) else 0),
                frag_bytes.eq(len(ctrls)),
            ]
        self.comb += Case(skp, cases)

        # Store Data/Ctrl in a 64/8-bit Shift Register ---------------------------------------------
        sr_data  = Signal(64)
        sr_ctrl  = Signal(8)
        sr_bytes = Signal(4)
        cases = {}
        cases[0] = [
            sr_data.eq(sr_data),
            sr_ctrl.eq(sr_ctrl),
        ]
        for i in range(1, 5):
            cases[i] = [
                sr_data.eq(Cat(sr_data[8*i:], frag_data[0:8*i])),
                sr_ctrl.eq(Cat(sr_ctrl[1*i:], frag_ctrl[0:1*i])),
            ]
        self.comb += sink.ready.eq(sr_bytes <= 7)
        self.sync += [
            If(sink.valid & sink.ready,
                If(source.valid & source.ready,
                    sr_bytes.eq(sr_bytes + frag_bytes - 4)
                ).Else(
                    sr_bytes.eq(sr_bytes + frag_bytes)
                ),
                Case(frag_bytes, cases)
            ).Elif(source.valid & source.ready,
                sr_bytes.eq(sr_bytes - 4)
            )
        ]

        # Output Data/Ctrl when there is a full 32/4-bit word --------------------------------------
        self.comb += source.valid.eq(sr_bytes >= 4)
        cases = {}
        for i in range(4, 8):
            cases[i] = [
                source.data.eq(sr_data[8*(8-i):8*(8-i+4)]),
                source.ctrl.eq(sr_ctrl[1*(8-i):1*(8-i+4)]),
            ]
        self.comb += Case(sr_bytes, cases)


# RX Aligner ---------------------------------------------------------------------------------------

class RXWordAligner(Module):
    def __init__(self, check_ctrl_only=False):
        self.enable = Signal(reset=1)
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        alignment   = Signal(2)
        alignment_d = Signal(2)

        buf = stream.Buffer([("data", 32), ("ctrl", 4)])
        self.submodules += buf
        self.comb += [
            sink.connect(buf.sink),
            source.valid.eq(sink.valid & buf.source.valid),
            buf.source.ready.eq(sink.valid & source.ready),
        ]

        # Alignment detection
        for i in reversed(range(4)):
            self.comb += [
                If(sink.valid & sink.ready,
                    If(sink.ctrl[i] & (check_ctrl_only | (sink.data[8*i:8*(i+1)] == COM.value)),
                        alignment.eq(i)
                    )
                )
            ]
        self.sync += [
            If(sink.valid & sink.ready,
                If(self.enable,
                    If((sink.ctrl != 0) & (buf.source.ctrl == 0),
                        alignment_d.eq(alignment),
                    )
                )
            ),
        ]

        # Data selection
        data = Cat(buf.source.data, sink.data)
        ctrl = Cat(buf.source.ctrl, sink.ctrl)
        cases = {}
        for i in range(4):
            cases[i] = [
                source.data.eq(data[8*i:]),
                source.ctrl.eq(ctrl[i:]),
            ]
        self.comb += Case(alignment_d, cases)

# RXSubstitution -----------------------------------------------------------------------------------

class RXSubstitution(Module):
    def __init__(self, serdes, clock_domain):
        self.sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        self.comb += self.sink.connect(self.source)
        for i in range(2):
            self.comb += [
                If(serdes.decoders[i].invalid,
                    self.source.ctrl[i].eq(1),
                    self.source.data[8*i:8*(i+1)].eq(K(28, 4)),
                )
            ]

# TX Skip Inserter ---------------------------------------------------------------------------------

class TXSkipInserter(Module):
    def __init__(self):
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        data_count   = Signal(8)
        skip_grant   = Signal(reset=1)
        skip_queue   = Signal()
        skip_dequeue = Signal()
        skip_count   = Signal(16)

        # Queue one 2 SKP Ordered Set every 166 Data/Ctrl words (FIXME: should be 1 SKP every 88 words)
        self.sync += [
            skip_queue.eq(0),
            If(sink.valid & sink.ready,
                data_count.eq(data_count + 1),
                If(data_count == 165,
                    data_count.eq(0),
                    skip_queue.eq(1)
                )
            )
        ]

        # SKP grant: SKP should not be inserted inside packets
        self.sync += [
            If(sink.valid & sink.ready,
                If(sink.last,
                    skip_grant.eq(1)
                ).Elif(sink.first,
                    skip_grant.eq(0)
                )
            )
        ]

        # SKP counter
        self.sync += [
            If(skip_queue & ~skip_dequeue,
                skip_count.eq(skip_count + 1)
            ),
            If(~skip_queue &  skip_dequeue,
                skip_count.eq(skip_count - 1)
            )
        ]

        # SKP insertion
        self.comb += [
            If(skip_grant & (skip_count != 0),
                source.valid.eq(1),
                source.data.eq(Replicate(Signal(8, reset=SKP.value), 4)),
                source.ctrl.eq(Replicate(Signal(1, reset=1)        , 4)),
                skip_dequeue.eq(source.ready)
            ).Else(
                sink.connect(source)
            )
        ]

# Datapath (Clock Domain Crossing & Converter) -----------------------------------------------------

class SerdesTXDatapath(Module):
    def __init__(self, clock_domain="sys", phy_dw=16):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])

        # # #

        skip_inserter = TXSkipInserter()
        self.submodules += skip_inserter
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
        cdc = ClockDomainsRenamer({"write": "sys", "read": clock_domain})(cdc)
        self.submodules.cdc = cdc
        converter = stream.StrideConverter(
            [("data", 32), ("ctrl", 4)],
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            reverse=False)
        converter = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules.converter = converter
        self.comb += [
            self.sink.connect(skip_inserter.sink),
            skip_inserter.source.connect(cdc.sink),
            cdc.source.connect(converter.sink),
            converter.source.connect(self.source)
        ]

class SerdesRXDatapath(Module):
    def __init__(self, clock_domain="sys", phy_dw=16):
        self.sink   = stream.Endpoint([("data", phy_dw), ("ctrl", phy_dw//8)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        converter = stream.StrideConverter(
            [("data", phy_dw), ("ctrl", phy_dw//8)],
            [("data", 32), ("ctrl", 4)],
            reverse=False)
        converter = stream.BufferizeEndpoints({"sink":   stream.DIR_SINK})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules.converter = converter
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
        cdc = ClockDomainsRenamer({"write": clock_domain, "read": "sys"})(cdc)
        self.submodules.cdc = cdc
        skip_remover = RXSkipRemover()
        self.submodules.skip_remover = skip_remover
        word_aligner = RXWordAligner()
        self.submodules.word_aligner = word_aligner
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(cdc.sink),
            cdc.source.connect(skip_remover.sink),
            skip_remover.source.connect(word_aligner.sink),
            word_aligner.source.connect(self.source),
        ]

# Xilinx Kintex7 USB3 Serializer/Deserializer ------------------------------------------------------

class K7USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_align    = Signal()   # i

        # # #

        from liteiclink.transceiver.gtx_7series import GTXChannelPLL, GTX

        # Clock ------------------------------------------------------------------------------------
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            refclk = refclk_pads
        else:
            refclk = Signal()
            self.specials += [
                Instance("IBUFDS_GTE2",
                    i_CEB = 0,
                    i_I   = refclk_pads.p,
                    i_IB  = refclk_pads.n,
                    o_O   = refclk
                )
            ]

        # PLL --------------------------------------------------------------------------------------
        pll = GTXChannelPLL(refclk, refclk_freq, 5e9)
        self.submodules += pll

        # Transceiver ------------------------------------------------------------------------------
        gtx = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 20,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            tx_polarity      = self.tx_polarity,
            rx_polarity      = self.rx_polarity)
        gtx.add_stream_endpoints()
        tx_datapath     = SerdesTXDatapath("tx")
        rx_substitution = RXSubstitution(gtx, "rx")
        rx_datapath     = SerdesRXDatapath("rx")
        self.submodules.gtx             = gtx
        self.submodules.tx_datapath     = tx_datapath
        self.submodules.rx_substitution = rx_substitution
        self.submodules.rx_datapath     = rx_datapath
        self.comb += [
            gtx.tx_enable.eq(self.enable),
            gtx.rx_enable.eq(self.enable),
            self.ready.eq(gtx.tx_ready & gtx.rx_ready),
            gtx.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtx.sink),
            gtx.source.connect(rx_substitution.sink),
            rx_substitution.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]

        # Override GTX RX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtx.gtx_params.update(
            p_RX_CM_SEL  = 0b11,
            p_RX_CM_TRIM = 0b1010,
            p_PMA_RSV2   = 0x2050,
        )

        # Override GTX parameters/signals to allow LFPS --------------------------------------------
        gtx.gtx_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100,
            p_RXOOB_CFG      = 0b0000110,
            i_CLKRSVD        = ClockSignal("usb3_oob"),
            i_RXELECIDLEMODE = 0b00,
            o_RXELECIDLE     = self.rx_idle,
            i_TXELECIDLE     = self.tx_idle)
        self.comb += [
            gtx.tx_produce_pattern.eq(self.tx_pattern != 0),
            gtx.tx_pattern.eq(self.tx_pattern)
        ]

        # Timing constraints -----------------------------------------------------------------------
        gtx.cd_tx.clk.attr.add("keep")
        gtx.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(gtx.cd_tx.clk, 1e9/gtx.tx_clk_freq)
        platform.add_period_constraint(gtx.cd_rx.clk, 1e9/gtx.rx_clk_freq)
        platform.add_false_path_constraints(
            sys_clk,
            gtx.cd_tx.clk,
            gtx.cd_rx.clk)

# Xilinx Artix7 USB3 Serializer/Deserializer -------------------------------------------------------

class A7USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_align    = Signal()   # i

        # # #

        from liteiclink.transceiver.gtp_7series import GTPQuadPLL, GTP

        # Clock ------------------------------------------------------------------------------------
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            refclk = refclk_pads
        else:
            refclk = Signal()
            self.specials += [
                Instance("IBUFDS_GTE2",
                    i_CEB=0,
                    i_I=refclk_pads.p,
                    i_IB=refclk_pads.n,
                    o_O=refclk
                )
            ]

        # PLL --------------------------------------------------------------------------------------
        pll = GTPQuadPLL(refclk, refclk_freq, 5e9)
        self.submodules += pll

        # Transceiver ------------------------------------------------------------------------------
        gtp = gtp = GTP(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width       = 20,
            clock_aligner    = False,
            tx_buffer_enable = True,
            rx_buffer_enable = True,
            tx_polarity      = self.tx_polarity,
            rx_polarity      = self.rx_polarity)
        gtp.add_stream_endpoints()
        tx_datapath     = SerdesTXDatapath("tx")
        rx_substitution = RXSubstitution(gtp, "rx")
        rx_datapath     = SerdesRXDatapath("rx")
        self.submodules.gtp             = gtp
        self.submodules.tx_datapath     = tx_datapath
        self.submodules.rx_substitution = rx_substitution
        self.submodules.rx_datapath     = rx_datapath
        self.comb += [
            gtp.tx_enable.eq(self.enable),
            gtp.rx_enable.eq(self.enable),
            self.ready.eq(gtp.tx_ready & gtp.rx_ready),
            gtp.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtp.sink),
            gtp.source.connect(rx_substitution.sink),
            rx_substitution.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]

        # Override GTP RX termination for USB3 (800 mV Term Voltage) -------------------------------
        gtp.gtp_params.update(
            p_RX_CM_SEL      = 0b11,
            p_RX_CM_TRIM     = 0b1010,
            p_RXLPM_INCM_CFG = 0b1,
            p_RXLPM_IPCM_CFG = 0b0
        )

        # Override GTP parameters/signals to allow LFPS --------------------------------------------
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100,
            p_RXOOB_CLK_CFG  = "FABRIC",
            i_SIGVALIDCLK    = ClockSignal("usb3_oob"),
            p_RXOOB_CFG      = 0b0000110,
            i_RXELECIDLEMODE = 0b00,
            o_RXELECIDLE     = self.rx_idle,
            i_TXELECIDLE     = self.tx_idle)
        self.comb += [
            gtp.tx_produce_pattern.eq(self.tx_pattern != 0),
            gtp.tx_pattern.eq(self.tx_pattern)
        ]

        # Timing constraints -----------------------------------------------------------------------
        gtp.cd_tx.clk.attr.add("keep")
        gtp.cd_rx.clk.attr.add("keep")
        platform.add_period_constraint(gtp.cd_tx.clk, 1e9/gtp.tx_clk_freq)
        platform.add_period_constraint(gtp.cd_rx.clk, 1e9/gtp.rx_clk_freq)
        platform.add_false_path_constraints(
            sys_clk,
            gtp.cd_tx.clk,
            gtp.cd_rx.clk)

# Lattice ECP5 USB3 Serializer/Deserializer --------------------------------------------------------

class ECP5USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads, channel):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_align    = Signal()   # i

        # # #

        from liteiclink.transceiver.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

        # Clock ------------------------------------------------------------------------------------
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            refclk = refclk_pads
        else:
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP     = refclk_pads.p,
                i_REFCLKN     = refclk_pads.n,
                o_REFCLKO     = refclk,
                p_REFCK_PWDNB = "0b1",
                p_REFCK_RTERM = "0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # PLL --------------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=5e9)
        self.submodules += serdes_pll

        # Transceiver ------------------------------------------------------------------------------
        serdes  = SerDesECP5(serdes_pll, tx_pads, rx_pads,
            channel     = channel,
            data_width  = 20,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity)
        serdes.add_stream_endpoints()
        tx_datapath     = SerdesTXDatapath("tx")
        rx_substitution = RXSubstitution(serdes, "rx")
        rx_datapath     = SerdesRXDatapath("rx")
        self.submodules.serdes          = serdes
        self.submodules.tx_datapath     = tx_datapath
        self.submodules.rx_substitution = rx_substitution
        self.submodules.rx_datapath     = rx_datapath
        self.comb += [
            serdes.tx_enable.eq(self.enable),
            serdes.rx_enable.eq(self.enable),
            self.ready.eq(serdes.tx_ready & serdes.rx_ready),
            serdes.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(serdes.sink),
            serdes.source.connect(rx_substitution.sink),
            rx_substitution.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]

        # Override SerDes parameters/signals to allow LFPS --------------------------------------------
        self.comb += [
            serdes.tx_produce_pattern.eq(self.tx_pattern != 0),
            serdes.tx_pattern.eq(self.tx_pattern)
        ]

        # Timing constraints -----------------------------------------------------------------------
        # FIXME: Add keep and false path?
        platform.add_period_constraint(serdes.txoutclk, 1e9/serdes.tx_clk_freq)
        platform.add_period_constraint(serdes.rxoutclk, 1e9/serdes.rx_clk_freq)
