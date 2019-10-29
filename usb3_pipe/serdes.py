# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen import *

from litex.soc.interconnect import stream
from litex.soc.cores.code_8b10b import Encoder, Decoder

from usb3_pipe.common import COM, SKP

# Datapath (Clock Domain Crossing & Converter) -----------------------------------------------------

class SerdesTXDatapath(Module):
    def __init__(self, clock_domain):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 16), ("ctrl", 2)])

        # # #

        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
        cdc = ClockDomainsRenamer({"write": "sys", "read": clock_domain})(cdc)
        self.submodules += cdc
        converter = stream.StrideConverter(
            [("data", 32), ("ctrl", 4)],
            [("data", 16), ("ctrl", 2)],
            reverse=False)
        converter = stream.BufferizeEndpoints({"source": stream.DIR_SOURCE})(converter)
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
        converter = stream.BufferizeEndpoints({"sink":   stream.DIR_SINK})(converter)
        converter = ClockDomainsRenamer(clock_domain)(converter)
        self.submodules += converter
        cdc = stream.AsyncFIFO([("data", 32), ("ctrl", 4)], 8)
        cdc = ClockDomainsRenamer({"write": clock_domain, "read": "sys"})(cdc)
        self.submodules += cdc
        self.comb += [
            self.sink.connect(converter.sink),
            converter.source.connect(cdc.sink),
            cdc.source.connect(self.source)
        ]

# RX Aligner ---------------------------------------------------------------------------------------

class SerdesRXWordAligner(stream.PipelinedActor):
    def __init__(self):
        self.enable = Signal(reset=1)
        self.sink   = sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        stream.PipelinedActor.__init__(self, 1)

        # # #

        alignment = Signal(2)

        last_data = Signal(32)
        last_ctrl = Signal(4)

        # Register last data/ctrl
        self.sync += [
            If(self.pipe_ce,
                last_data.eq(sink.data),
                last_ctrl.eq(sink.ctrl)
            )
        ]

        # Alignment detection
        for i in range(4):
            self.sync += [
                If(self.enable & sink.valid & self.pipe_ce,
                    If(sink.ctrl[i] & (sink.data[8*i:8*(i+1)] == COM.value),
                        alignment.eq(i)
                    )
                )
            ]

        # Do the alignment
        data = Cat(last_data, sink.data)
        ctrl = Cat(last_ctrl, sink.ctrl)
        cases = {}
        for i in range(4):
            cases[i] = [
                source.data.eq(data[8*i:]),
                source.ctrl.eq(ctrl[i:]),
            ]
        self.comb += Case(alignment, cases)

# RX Skip Remover ----------------------------------------------------------------------------------

class SerdesRXSkipRemover(Module):
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
        cases[0b0000] = [
            frag_data.eq(sink.data),
            frag_ctrl.eq(sink.ctrl),
            frag_bytes.eq(4)
        ]
        cases[0b0001] = [
            frag_data.eq(sink.data[8:]),
            frag_ctrl.eq(sink.ctrl[1:]),
            frag_bytes.eq(3)
        ]
        cases[0b0010] = [
            frag_data.eq(Cat(sink.data[0:8], sink.data[16:])),
            frag_ctrl.eq(Cat(sink.ctrl[0], sink.ctrl[2:])),
            frag_bytes.eq(3)
        ]
        cases[0b0011] = [
            frag_data.eq(Cat(sink.data[16:])),
            frag_ctrl.eq(Cat(sink.ctrl[2:])),
            frag_bytes.eq(2)
        ]
        cases[0b0100] = [
            frag_data.eq(Cat(sink.data[0:16], sink.data[24:])),
            frag_ctrl.eq(Cat(sink.ctrl[0:2], sink.ctrl[3])),
            frag_bytes.eq(3)
        ]
        cases[0b0101] = [
            frag_data.eq(Cat(sink.data[8:16], sink.data[24:])),
            frag_ctrl.eq(Cat(sink.ctrl[1:2], sink.ctrl[3])),
            frag_bytes.eq(2)
        ]
        cases[0b0111] = [
            frag_data.eq(sink.data[24:]),
            frag_ctrl.eq(sink.ctrl[3]),
            frag_bytes.eq(1)
        ]
        cases[0b1000] = [
            frag_data.eq(sink.data[:24]),
            frag_ctrl.eq(sink.ctrl[:3]),
            frag_bytes.eq(3)
        ]
        cases[0b1001] = [
            frag_data.eq(sink.data[8:24]),
            frag_ctrl.eq(sink.ctrl[1:3]),
            frag_bytes.eq(2)
        ]
        cases[0b1010] = [
            frag_data.eq(Cat(sink.data[:8], sink.data[16:24])),
            frag_ctrl.eq(Cat(sink.ctrl[:1], sink.data[2:3])),
            frag_bytes.eq(2)
        ]
        cases[0b1011] = [
            frag_data.eq(sink.data[16:24]),
            frag_ctrl.eq(sink.ctrl[2]),
            frag_bytes.eq(1)
        ]
        cases[0b1100] = [
            frag_data.eq(sink.data[:16]),
            frag_ctrl.eq(sink.ctrl[:2]),
            frag_bytes.eq(2)
        ]
        cases[0b1101] = [
            frag_data.eq(sink.data[8:16]),
            frag_ctrl.eq(sink.ctrl[1:2]),
            frag_bytes.eq(1)
        ]
        cases[0b1110] = [
            frag_data.eq(sink.data[:8]),
            frag_ctrl.eq(sink.ctrl[0]),
            frag_bytes.eq(1)
        ]
        cases[0b1111] = [
            frag_data.eq(0),
            frag_ctrl.eq(0),
            frag_bytes.eq(0),
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
        cases[1] = [
            sr_data.eq(Cat(frag_data[:8], sr_data)),
            sr_ctrl.eq(Cat(frag_ctrl[:1],  sr_ctrl)),
        ]
        cases[2] = [
            sr_data.eq(Cat(frag_data[:16], sr_data)),
            sr_ctrl.eq(Cat(frag_ctrl[:2],  sr_ctrl)),
        ]
        cases[3] = [
            sr_data.eq(Cat(frag_data[:24], sr_data)),
            sr_ctrl.eq(Cat(frag_ctrl[:3],  sr_ctrl)),
        ]
        cases[4] = [
            sr_data.eq(Cat(frag_data[:32], sr_data)),
            sr_ctrl.eq(Cat(frag_ctrl[:4],  sr_ctrl)),
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
        cases[4] = [
            source.data.eq(sr_data[0:32]),
            source.ctrl.eq(sr_ctrl[0:4]),
        ]
        cases[5] = [
            source.data.eq(sr_data[8:40]),
            source.ctrl.eq(sr_ctrl[1:5]),
        ]
        cases[6] = [
            source.data.eq(sr_data[16:48]),
            source.ctrl.eq(sr_ctrl[2:6]),
        ]
        cases[7] = [
            source.data.eq(sr_data[24:56]),
            source.ctrl.eq(sr_ctrl[3:7]),
        ]
        self.comb += Case(sr_bytes, cases)

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
        self.rx_skip     = Signal()   # o

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
        tx_datapath     = SerdesTXDatapath("tx")
        rx_datapath     = SerdesRXDatapath("rx")
        rx_aligner      = SerdesRXWordAligner()
        rx_skip_remover = SerdesRXSkipRemover()
        self.comb += self.rx_skip.eq(rx_skip_remover.skip)
        self.submodules += gtx, tx_datapath, rx_datapath, rx_aligner, rx_skip_remover
        self.comb += [
            gtx.tx_enable.eq(self.enable),
            gtx.rx_enable.eq(self.enable),
            self.ready.eq(gtx.tx_ready & gtx.rx_ready),
            gtx.rx_align.eq(self.rx_align),
            rx_aligner.enable.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtx.sink),
            gtx.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(rx_aligner.sink),
            rx_aligner.source.connect(rx_skip_remover.sink),
            rx_skip_remover.source.connect(self.source),
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
        self.rx_skip     = Signal()   # o

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
        tx_datapath     = SerdesTXDatapath("tx")
        rx_datapath     = SerdesRXDatapath("rx")
        rx_aligner      = SerdesRXWordAligner()
        rx_skip_remover = SerdesRXSkipRemover()
        self.comb += self.rx_skip.eq(rx_skip_remover.skip)
        self.submodules += gtp, tx_datapath, rx_datapath, rx_aligner, rx_skip_remover
        self.comb += [
            gtp.tx_enable.eq(self.enable),
            gtp.rx_enable.eq(self.enable),
            self.ready.eq(gtp.tx_ready & gtp.rx_ready),
            gtp.rx_align.eq(self.rx_align),
            rx_aligner.enable.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(gtp.sink),
            gtp.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(rx_aligner.sink),
            rx_aligner.source.connect(rx_skip_remover.sink),
            rx_skip_remover.source.connect(self.source),
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

# Lattice ECP5 USB3 Serializer/Deserializer --------------------------------------------------------

class ECP5USB3SerDes(Module):
    def __init__(self, platform, sys_clk, sys_clk_freq, refclk_pads, refclk_freq, tx_pads, rx_pads, channel):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i # FIXME: not used for now
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i

        self.rx_polarity = Signal()   # i # FIXME: not used for now
        self.rx_idle     = Signal()   # o
        self.rx_align    = Signal()   # i
        self.rx_skip     = Signal()   # o

        # # #

        from liteiclink.transceiver.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

        # Clock ------------------------------------------------------------------------------------
        if isinstance(refclk_pads, (Signal, ClockSignal)):
            refclk = refclk_pads
        else:
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP=refclk_pads.p,
                i_REFCLKN=refclk_pads.n,
                o_REFCLKO=refclk,
                p_REFCK_PWDNB="0b1",
                p_REFCK_RTERM="0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # PLL --------------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=5e9)
        self.submodules += serdes_pll

        # Transceiver ------------------------------------------------------------------------------
        serdes  = SerDesECP5(serdes_pll, tx_pads, rx_pads, channel=channel, data_width=20)
        serdes.add_stream_endpoints()
        tx_datapath     = SerdesTXDatapath("tx")
        rx_datapath     = SerdesRXDatapath("rx")
        rx_aligner      = SerdesRXWordAligner()
        rx_skip_remover = SerdesRXSkipRemover()
        self.comb += self.rx_skip.eq(rx_skip_remover.skip)
        self.submodules += serdes, tx_datapath, rx_datapath, rx_aligner, rx_skip_remover
        self.comb += [
            serdes.tx_enable.eq(self.enable),
            serdes.rx_enable.eq(self.enable),
            serdes.tx_idle.eq(self.tx_idle),
            self.rx_idle.eq(serdes.rx_idle),
            self.ready.eq(serdes.tx_ready & serdes.rx_ready),
            serdes.rx_align.eq(self.rx_align),
            rx_aligner.enable.eq(self.rx_align),
            self.sink.connect(tx_datapath.sink),
            tx_datapath.source.connect(serdes.sink),
            serdes.source.connect(rx_datapath.sink),
            rx_datapath.source.connect(rx_aligner.sink),
            rx_aligner.source.connect(rx_skip_remover.sink),
            rx_skip_remover.source.connect(self.source),
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

# Simulation Serializer/Deserializer Model ---------------------------------------------------------

class USB3SerDesModel(Module):
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.source = stream.Endpoint([("data", 32), ("ctrl", 4)])
        self.tx     = stream.Endpoint([("data", 40)])
        self.rx     = stream.Endpoint([("data", 40)])

        self.enable = Signal(reset=1) # i
        self.ready  = Signal()        # o

        self.tx_polarity = Signal()   # i
        self.tx_idle     = Signal()   # i
        self.tx_pattern  = Signal(20) # i

        self.rx_polarity = Signal()   # i
        self.rx_idle     = Signal()   # o
        self.rx_align    = Signal()   # i # not used

        # # #

        tx_data = Signal(40)
        rx_data = Signal(40)

        encoder  = Encoder(4, True)
        decoders = [Decoder(True) for _ in range(4)]
        self.submodules += encoder, decoders
        self.comb += self.sink.ready.eq(1)
        self.comb += self.source.valid.eq(1)
        for i in range(4):
            self.comb += [
                encoder.k[i].eq(self.sink.ctrl[i]),
                encoder.d[i].eq(self.sink.data[8*i:8*(i+1)]),
                self.source.ctrl[i].eq(decoders[i].k),
                self.source.data[8*i:8*(i+1)].eq(decoders[i].d),
            ]
        self.comb += [
            If(self.tx_pattern != 0,
                tx_data.eq(self.tx_pattern)
            ).Else(
                tx_data.eq(Cat(*[encoder.output[i] for i in range(4)])),
            ),
            If(self.tx_polarity,
                self.tx.data.eq(~tx_data)
            ).Else(
                self.tx.data.eq(tx_data)
            )
        ]
        self.comb += [
            If(self.rx_polarity,
                rx_data.eq(~self.rx.data)
            ).Else(
                rx_data.eq(self.rx.data)
            )
        ]
        for i in range(4):
            self.comb += decoders[i].input.eq(rx_data[10*i:10*(i+1)])

        # Ready when enabled
        self.comb += self.ready.eq(self.enable)

    def connect(self, serdes):
        self.comb += [
            self.tx.connect(serdes.rx),
            serdes.tx.connect(self.rx),
            self.rx_idle.eq(serdes.tx_idle),
            serdes.rx_idle.eq(self.tx_idle),
        ]
