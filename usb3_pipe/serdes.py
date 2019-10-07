from migen import *

from litex.soc.interconnect import stream

# Datapath (Clock Domain Crossing & Converter) -----------------------------------------------------

class SerdesTXDatapath(Module):
    def __init__(self, clock_domain):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 4)
        cdc = ClockDomainsRenamer({"write": "sys", "read": clock_domain})(cdc)
        self.submodules += cdc
        converter = stream.StrideConverter(
            [("data", 32), ("ctrl", 4)],
            [("data", 16), ("ctrl", 2)],
            reverse=False)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules += converter
        self.comb += [
            self.sink.connect(cdc.sink),
            cdc.source.connect(converter.sink),
            converter.source.connect(self.source)
        ]

class SerdesRXDatapath(Module):
    def __init__(self, clock_domain):
        self.sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        # # #

        converter = stream.StrideConverter(
            [("data", 16), ("ctrl", 2)],
            [("data", 32), ("ctrl", 4)],
            reverse=False)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules += converter
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 4)
        cdc = ClockDomainsRenamer({"write": clock_domain, "read": "sys"})(cdc)
        self.submodules += cdc
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(cdc.sink),
            cdc.source.connect(self.source)
        ]

# Kintex7 USB3 Serializer/Deserializer -------------------------------------------------------------

class K7USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal()

        self.tx_polarity = Signal()
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)

        self.rx_polarity = Signal()
        self.rx_idle     = Signal()
        self.rx_align    = Signal()

        # # #

        from liteiclink.transceiver.gtx_7series import GTXChannelPLL, GTX

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
        pll = GTXChannelPLL(refclk, refclk_freq, 5e9)
        self.submodules += pll

        # Transceiver ------------------------------------------------------------------------------
        self.submodules.gtx = gtx = GTX(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True,
            tx_polarity=self.tx_polarity,
            rx_polarity=self.rx_polarity)
        gtx.add_stream_endpoints()
        tx_datapath = SerdesTXDatapath("tx")
        rx_datapath = SerdesRXDatapath("rx")
        self.submodules += gtx, tx_datapath, rx_datapath
        self.comb += [
            gtx.tx_enable.eq(self.enable),
            gtx.rx_enable.eq(self.enable),
            gtx.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtx.sink),
            gtx.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]
        # Override GTX parameters/signals to allow LFPS --------------------------------------------
        gtx.gtx_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100,
            p_RXOOB_CFG      = 0b0000110,
            i_RXOOBRESET     = 0,
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

# Artix7 USB3 Serializer/Deserializer -------------------------------------------------------------

class A7USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal()

        self.tx_polarity = Signal()
        self.tx_idle     = Signal()
        self.tx_pattern  = Signal(20)

        self.rx_polarity = Signal()
        self.rx_idle     = Signal()
        self.rx_align    = Signal()

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
        self.submodules.gtp = gtp = GTP(pll, tx_pads, rx_pads, sys_clk_freq,
            data_width=20,
            clock_aligner=False,
            tx_buffer_enable=True,
            rx_buffer_enable=True,
            tx_polarity=self.tx_polarity,
            rx_polarity=self.rx_polarity)
        gtp.add_stream_endpoints()
        tx_datapath = SerdesTXDatapath("tx")
        rx_datapath = SerdesRXDatapath("rx")
        self.submodules += gtp, tx_datapath, rx_datapath
        self.comb += [
            gtp.tx_enable.eq(self.enable),
            gtp.rx_enable.eq(self.enable),
            gtp.rx_align.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtp.sink),
            gtp.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(self.source),
        ]
        # Override GTP parameters/signals to allow LFPS --------------------------------------------
        gtp.gtp_params.update(
            p_PCS_RSVD_ATTR  = 0x000000000100,
            #p_RXOOB_CLK_CFG  = "FABRIC",
            #i_SIGVALIDCLK    = ClockSignal("usb3_oob"),
            p_RXOOB_CLK_CFG  = "PMA",
            i_RXOOBRESET     = 0b0,
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
