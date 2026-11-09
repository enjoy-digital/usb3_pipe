#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# USB3-PIPE on the LambdaConcept ECPIX-5 (Lattice LFE5UM5G-85F, ECP5USB3SerDes on DCU0, USB-C
# connector through the board's SuperSpeed mux).
#
# Board notes (measured on hardware, see doc/ecpix5.md):
# - 5Gbps DCU reference clock: 250MHz from a fabric PLL (x20). The 100MHz EXTREF0 input
#   (serdes_clk100) only reaches 2.5Gbps (x25 max).
# - USB-C SuperSpeed mux: enable pin active high (usbc_mux en_n in the platform), amsel high, polarity
#   from the cable direction of the CC controller (TI TUSB322, I2C 0x47, read by TUSB322Reader: the
#   usbc_cfg dir pin is not usable for it).
# - USB3 PIPE at 125MHz on the DCU TX clock, LTSSM/Link Layer/Core/SoC at 83.33MHz on 64-bit words
#   (USB3PHY pipe_cd, asynchronous CDC, as on the Gowin GW5A at 62.5MHz: ECP5 fabric timings with
#   margin, ~Gen 1 bandwidth), without the PHY debug: USB3LinkStatus instead. --sys-clk-freq 125e6:
#   PIPE/LTSSM/Core at 125MHz (32-bit words, met on some nextpnr seeds only).
#
# Progressive bring-up designs (SerDes, DCU loopback, LFPS/RX only): lambdaconcept_ecpix5_bringup.py.

import argparse

import common # Repository root on the path.

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.build.generic_platform import *

from litex_boards.platforms import lambdaconcept_ecpix5

from litex.soc.interconnect.csr import *
from litex.soc.cores.clock      import ECP5PLL
from litex.soc.cores.bitbang    import I2CMaster
from litex.soc.cores.i2c        import I2CMasterMachine

from usb3_pipe import ECP5USB3SerDes

from usb3_soc import USB3BaseSoC, add_usb3_device
from usb3_soc import usb3_soc_args, usb3_soc_argdict, usb3_soc_build

# USB3 IOs -----------------------------------------------------------------------------------------

# USB-C SuperSpeed lanes (DCU0 CH0/CH1) and mux controls (platform's usbc_mux without the lanes,
# requested as LVCMOS18D there).
_usb3_io = [
    ("usb3_tx", 0, Subsignal("p", Pins("AD7")),  Subsignal("n", Pins("AD8"))),
    ("usb3_rx", 0, Subsignal("p", Pins("AF6")),  Subsignal("n", Pins("AF7"))),
    ("usb3_tx", 1, Subsignal("p", Pins("AD10")), Subsignal("n", Pins("AD11"))),
    ("usb3_rx", 1, Subsignal("p", Pins("AF9")),  Subsignal("n", Pins("AF10"))),
    ("usb3_mux", 0,
        Subsignal("en",    Pins("C23")), # Active high (en_n in the platform).
        Subsignal("amsel", Pins("B26")),
        Subsignal("pol",   Pins("D26")),
        IOStandard("LVCMOS33"),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, refclk_freq=250e6):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_refclk = ClockDomain(reset_less=True)

        # # #

        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")

        # PLL: sys and the DCU fabric reference clock (5Gbps: 250MHz x20).
        self.pll = pll = ECP5PLL()
        self.comb += pll.reset.eq(~rst_n | self.rst)
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys,    sys_clk_freq)
        pll.create_clkout(self.cd_refclk, refclk_freq)
        platform.add_false_path_constraints(self.cd_sys.clk, clk100)

# TUSB322 Reader -----------------------------------------------------------------------------------

class TUSB322Reader(LiteXModule):
    """TI TUSB322 USB-C CC controller register reader (I2C master, periodic polling of the
    ATTACHED_STATE/CABLE_DIR register 0x09): attached, cable direction (CC1: 0, CC2: 1), valid (last
    read acknowledged)."""
    def __init__(self, pads, sys_clk_freq, address=0x47, i2c_freq=100e3, period=10e-3):
        self.valid     = Signal() # o
        self.attached  = Signal() # o
        self.cable_dir = Signal() # o
        self.reg       = Signal(8) # o: Register 0x09.

        # # #

        # I2C Master (LiteX I2CMasterMachine: one operation per command, from idle).
        self.i2c = i2c = I2CMasterMachine(clock_width=16)
        self.comb += i2c.cg.load.eq(int(sys_clk_freq/(2*i2c_freq)) - 1)

        # I/Os (open drain, SDA changed when SCL is stable: as LiteX I2CMaster).
        self.scl_t = scl_t = TSTriple()
        self.sda_t = sda_t = TSTriple()
        self.specials += scl_t.get_tristate(pads.scl), sda_t.get_tristate(pads.sda)
        scl_i_d  = Signal()
        sda_oe_d = Signal()
        self.sync += scl_i_d.eq(scl_t.i), sda_oe_d.eq(sda_t.oe)
        self.comb += [
            scl_t.oe.eq(~i2c.scl_o),
            scl_t.o.eq(0),
            sda_t.oe.eq(sda_oe_d),
            If(scl_i_d == i2c.scl_o, sda_t.oe.eq(~i2c.sda_o)),
            sda_t.o.eq(0),
            i2c.sda_i.eq(sda_t.i),
        ]

        # Register read sequence: START, address + W, register, RESTART, address + R, read (NACK),
        # STOP.
        ops = [
            ("start", None),
            ("write", address << 1),
            ("write", 0x09),
            ("start", None),
            ("write", (address << 1) | 1),
            ("read",  None),
            ("stop",  None),
        ]
        step  = Signal(max=len(ops))
        ok    = Signal()
        timer = WaitTimer(int(period*sys_clk_freq))
        self.timer = timer
        self.fsm = fsm = FSM(reset_state="WAIT")
        fsm.act("WAIT",
            timer.wait.eq(1),
            If(timer.done,
                NextValue(step, 0),
                NextValue(ok, 1),
                NextState("ISSUE")
            )
        )
        cases = {}
        for n, (op, data) in enumerate(ops):
            cases[n] = [getattr(i2c, op).eq(1)]
        fsm.act("ISSUE",
            Case(step, cases),
            NextState("BUSY")
        )
        for n, (op, data) in enumerate(ops):
            if data is not None:
                self.sync += If(fsm.ongoing("ISSUE") & (step == n), i2c.data.eq(data))
            if op == "read":
                self.sync += If(fsm.ongoing("ISSUE") & (step == n), i2c.ack.eq(0)) # NACK.
        is_write = Signal()
        self.comb += is_write.eq(Array([op == "write" for op, _ in ops])[step])
        fsm.act("BUSY",
            If(i2c.idle,
                If(is_write & ~i2c.ack, NextValue(ok, 0)),
                If(step == (len(ops) - 2),
                    NextValue(self.reg, i2c.data)
                ),
                NextValue(step, step + 1),
                If(step == (len(ops) - 1),
                    NextState("DONE")
                ).Else(
                    NextState("ISSUE")
                )
            )
        )
        fsm.act("DONE",
            NextValue(self.valid, ok),
            NextState("WAIT")
        )
        self.comb += [
            self.attached.eq(self.valid & (self.reg[6:8] != 0b00)),
            self.cable_dir.eq(self.reg[5]),
        ]

# USB-C Control ------------------------------------------------------------------------------------

class USBCControl(LiteXModule):
    """USB-C SuperSpeed mux control, TUSB322 cable direction and VBUS output (kept off: device).

    pol: mux polarity from the TUSB322 cable direction ("cc", held when not attached) or fixed ("0",
    "1", also used when the manual CSR field is set). with_i2c_bitbang: I2C bitbang CSRs instead of
    the TUSB322Reader (bring-up, fixed polarity).
    """
    def __init__(self, platform, sys_clk_freq, pol="cc", with_i2c_bitbang=False):
        assert pol in ["cc", "0", "1"]
        assert not (with_i2c_bitbang and pol == "cc")
        mux = platform.request("usb3_mux")
        cfg = platform.request("usbc_cfg")
        pwr = platform.request("usb_c_power")

        self.mux = CSRStorage(fields=[
            CSRField("en",     size=1, reset=1, description="Mux enable."),
            CSRField("amsel",  size=1, reset=1, description="Mux mode select (SuperSpeed only)."),
            CSRField("pol",    size=1, reset=int(pol == "1"), description="Mux polarity (manual)."),
            CSRField("manual", size=1, reset=int(pol != "cc"), description="Manual mux polarity (else from the TUSB322 cable direction)."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("pol",       size=1, description="Mux polarity."),
            CSRField("cc_valid",  size=1, description="TUSB322 read acknowledged."),
            CSRField("attached",  size=1, description="TUSB322 attached."),
            CSRField("cable_dir", size=1, description="TUSB322 cable direction."),
            CSRField("cc_reg",    size=8, offset=8, description="TUSB322 register 0x09."),
        ])

        # # #

        mux_pol = Signal()
        self.comb += [
            pwr.en_n.eq(1),
            mux.en.eq(self.mux.fields.en),
            mux.amsel.eq(self.mux.fields.amsel),
            mux.pol.eq(mux_pol),
            self.status.fields.pol.eq(mux_pol),
        ]
        if with_i2c_bitbang:
            self.i2c = I2CMaster(pads=cfg)
            self.comb += mux_pol.eq(self.mux.fields.pol)
        else:
            self.tusb322 = tusb322 = TUSB322Reader(cfg, sys_clk_freq)
            cc_pol = Signal()
            self.sync += If(tusb322.attached, cc_pol.eq(tusb322.cable_dir))
            self.comb += [
                mux_pol.eq(Mux(self.mux.fields.manual, self.mux.fields.pol, cc_pol)),
                self.status.fields.cc_valid.eq(tusb322.valid),
                self.status.fields.attached.eq(tusb322.attached),
                self.status.fields.cable_dir.eq(tusb322.cable_dir),
                self.status.fields.cc_reg.eq(tusb322.reg),
            ]

# USB3 Link Status ---------------------------------------------------------------------------------

class USB3LinkStatus(LiteXModule):
    """Minimal USB3 link status (no PHY debug on the ECP5: sys timings): LTSSM state, U0/Recovery/
    SerDes ready/RX Electrical Idle, counters: U0 entries, Recovery entries, RX decode errors and RX
    Electrical Idle assertions while in U0, U0 cycles (cleared by clear). serdes_cd: SerDes signals
    clock domain (resynchronized to sys when not sys)."""
    def __init__(self, phy, serdes, serdes_cd="sys"):
        self.clear  = CSR()
        self.status = CSRStatus(fields=[
            CSRField("ltssm_state", size=len(phy.ltssm_state)),
            CSRField("u0",          size=1, offset=8),
            CSRField("recovery",    size=1),
            CSRField("ready",       size=1),
            CSRField("rx_idle",     size=1),
        ])
        names = ["u0_entries", "recovery_entries", "u0_rx_errors", "u0_rx_idles", "u0_cycles"]
        for name in names:
            setattr(self, name, CSRStatus(32, name=name))

        # # #

        u0        = phy.u0
        recovery  = phy.ltssm.recovery
        ready     = Signal()
        rx_idle   = Signal()
        rx_errors = Signal()
        if serdes_cd == "sys":
            self.comb += [
                ready.eq(serdes.ready),
                rx_idle.eq(serdes.rx_idle),
                rx_errors.eq(serdes.rx_errors),
            ]
        else:
            self.specials += [
                MultiReg(serdes.ready,     ready),
                MultiReg(serdes.rx_idle,   rx_idle),
                MultiReg(serdes.rx_errors, rx_errors),
            ]
        self.comb += [
            self.status.fields.ltssm_state.eq(phy.ltssm_state),
            self.status.fields.u0.eq(u0),
            self.status.fields.recovery.eq(recovery),
            self.status.fields.ready.eq(ready),
            self.status.fields.rx_idle.eq(rx_idle),
        ]
        u0_d       = Signal()
        recovery_d = Signal()
        rx_idle_d  = Signal()
        self.sync += [
            u0_d.eq(u0),
            recovery_d.eq(recovery),
            rx_idle_d.eq(rx_idle),
        ]
        events = {
            "u0_entries"       : u0 & ~u0_d,
            "recovery_entries" : recovery & ~recovery_d,
            "u0_rx_errors"     : u0 & rx_errors,
            "u0_rx_idles"      : u0 & rx_idle & ~rx_idle_d,
            "u0_cycles"        : u0,
        }
        for name in names:
            csr = getattr(self, name).status
            self.sync += If(self.clear.re, csr.eq(0)).Elif(events[name], csr.eq(csr + 1))

# ECPIX5BaseSoC ------------------------------------------------------------------------------------

class ECPIX5BaseSoC(USB3BaseSoC):
    """ECPIX-5 base: CRG (sys, DCU fabric reference clock), UARTBone, USB-C control, LEDs (all off,
    led() for dimmed single color indicators)."""
    def __init__(self, platform, sys_clk_freq=125e6, refclk_freq=250e6, usbc_pol="cc",
        with_i2c_bitbang=False, **kwargs):
        platform.add_extension(_usb3_io)
        self.refclk_freq = refclk_freq

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, refclk_freq)

        # USB3BaseSoC ------------------------------------------------------------------------------
        USB3BaseSoC.__init__(self, platform, sys_clk_freq, **kwargs)

        # USB-C ------------------------------------------------------------------------------------
        self.usbc = USBCControl(platform, sys_clk_freq, pol=usbc_pol,
            with_i2c_bitbang=with_i2c_bitbang)

        # Leds -------------------------------------------------------------------------------------
        # Active low, all driven (floating ones light up), single color and dimmed (1/32).
        self.leds = [platform.request("rgb_led", i) for i in range(4)]
        self.leds_dim = Signal(5)
        self.sync += self.leds_dim.eq(self.leds_dim + 1)
        for led in self.leds:
            self.comb += [led.r.eq(1), led.g.eq(1), led.b.eq(1)]

    def led(self, color, signal):
        """Dimmed single color LED (overrides the default off)."""
        self.comb += color.eq(~(signal & (self.leds_dim == 0)))

# USB3SoC ------------------------------------------------------------------------------------------

class USB3SoC(ECPIX5BaseSoC):
    def __init__(self, platform, sys_clk_freq=int(500e6/6), channel=0, serdes_params={}, usb3_kwargs={},
        **kwargs):
        # USB3 PIPE/SerDes at 125MHz (Gen 1 32-bit words); with a lower sys_clk_freq, the PIPE runs on
        # the DCU TX clock ("tx" clock domain: frequency-locked to the transmitted words, no TX CDC)
        # and the LTSSM/Link Layer/Core/SoC in sys with 64-bit Link Layer words.
        usb3_clk_freq = 125e6
        phy_cd        = None if sys_clk_freq == usb3_clk_freq else "tx"

        ECPIX5BaseSoC.__init__(self, platform, sys_clk_freq, **kwargs)

        # USB3 SerDes ------------------------------------------------------------------------------
        self.usb3_serdes = usb3_serdes = ECP5USB3SerDes(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = usb3_clk_freq,
            refclk       = ClockSignal("refclk"),
            refclk_freq  = self.refclk_freq,
            tx_pads      = platform.request("usb3_tx", channel),
            rx_pads      = platform.request("usb3_rx", channel),
            dual         = 0,
            channel      = channel,
            xcvr_cd      = "sys" if phy_cd is None else "dcu_ctrl",
            tx_cdc       = phy_cd != "tx")
        usb3_serdes.dcu.serdes_params.update(serdes_params) # DCU parameters overrides (bring-up).
        if phy_cd is not None:
            # DCU control logic on sys (free-running), SerDes datapath on the DCU TX clock.
            self.cd_dcu_ctrl = ClockDomain()
            self.comb += [
                self.cd_dcu_ctrl.clk.eq(self.crg.cd_sys.clk),
                self.cd_dcu_ctrl.rst.eq(self.crg.cd_sys.rst),
            ]
            usb3_serdes = ClockDomainsRenamer({"sys": phy_cd})(usb3_serdes)
            usb3_kwargs = dict(usb3_kwargs, phy_cd=phy_cd, phy_clk_freq=usb3_clk_freq, link_words=2)

        # USB3 (PHY, Core, Leds: SerDes ready/U0 green, Recovery red) ------------------------------
        leds = [Signal() for _ in range(3)]
        add_usb3_device(self, usb3_serdes, leds=leds, **usb3_kwargs)
        self.usb3_link = USB3LinkStatus(self.usb3_phy, usb3_serdes,
            serdes_cd="sys" if phy_cd is None else phy_cd)
        self.led(self.leds[0].g, leds[0])
        self.led(self.leds[1].g, leds[1])
        self.led(self.leds[2].r, leds[2])

# Build --------------------------------------------------------------------------------------------

def ecpix5_args(parser):
    """ECPIX-5 target arguments (shared with lambdaconcept_ecpix5_bringup.py)."""
    parser.add_argument("--sys-clk-freq", default=500e6/6, type=float, help="System clock frequency (125MHz: PHY/Core/SoC in sys).")
    parser.add_argument("--channel",      default=0,     type=int,   help="DCU0 channel (USB-C lanes).", choices=[0, 1])
    parser.add_argument("--usbc-pol",     default="cc",              help="USB-C mux polarity (cc: TUSB322 cable direction).", choices=["cc", "0", "1"])
    parser.add_argument("--serdes-param", default=[], action="append", help="DCU parameter override NAME=VALUE (ex: p_CHX_REQ_LVL_SET=0b01).")
    parser.add_argument("--seed",         default=1,     type=int,   help="nextpnr seed.")

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE on the LambdaConcept ECPIX-5 (ECP5 DCU).")
    ecpix5_args(parser)
    usb3_soc_args(parser, output_dir="build/lambdaconcept_ecpix5")
    parser.set_defaults(max_burst=16) # Bulk IN bandwidth with the 64-bit Link Layer words.
    args = parser.parse_args()

    # No PHY debug, throughput profile and core trace on the ECP5 (125MHz timings, --lean).
    soc_args = usb3_soc_argdict(args)
    soc_args["usb3_kwargs"].update(with_debug=False, with_profile=False, with_trace=False)

    platform = lambdaconcept_ecpix5.Platform(toolchain="trellis")
    soc = USB3SoC(platform,
        sys_clk_freq  = int(args.sys_clk_freq),
        channel       = args.channel,
        usbc_pol      = args.usbc_pol,
        serdes_params = dict(p.split("=", 1) for p in args.serdes_param),
        with_uartbone = soc_args["with_uartbone"],
        with_jtagbone = soc_args["with_jtagbone"],
        cpu_type      = soc_args["cpu_type"],
        usb3_kwargs   = soc_args["usb3_kwargs"],
    )
    usb3_soc_build(parser, args, soc, seed=args.seed)

if __name__ == "__main__":
    main()
