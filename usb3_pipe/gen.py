#!/usr/bin/env python3

#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
USB3-PIPE standalone core generator.

Generates a standalone USB3.0 SuperSpeed PHY (FPGA transceiver + USB3PIPE + LTSSM) as a Verilog
module with an AXI-Stream symbol interface, its timing constraints, an instantiation template and
an interface documentation, from a YAML configuration file:

    usb3_pipe_gen examples/a7_device.yml

Current support: Xilinx 7-Series (Artix7 GTP / Kintex7 GTX), Ultrascale+ (GTY).
"""

import os
import re
import argparse
from collections import namedtuple

import yaml

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, XilinxUSPPlatform

from litex.soc.cores.clock import S7PLL, USPMMCM
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from usb3_pipe.serdes import A7USB3SerDes, K7USB3SerDes, USPUSB3SerDes
from usb3_pipe.ltssm  import USB3LTSSM
from usb3_pipe.phy    import USB3PHY, USB3PHY_STATUS, USB3PHY_LINK_CTRL

# Configuration ------------------------------------------------------------------------------------

# Supported PHYs: SerDes class, default device (family), platform class, OOB clock PLL class.
PHYS = {
    "A7USB3SerDes"  : (A7USB3SerDes,  "xc7a",   XilinxPlatform,    S7PLL),
    "K7USB3SerDes"  : (K7USB3SerDes,  "xc7k",   XilinxPlatform,    S7PLL),
    "USPUSB3SerDes" : (USPUSB3SerDes, "xcku3p", XilinxUSPPlatform, USPMMCM),
}

# Configuration keys and default values.
CONFIG_DEFAULTS = {
    "phy"                  : None,     # A7USB3SerDes / K7USB3SerDes / USPUSB3SerDes (required).
    "device"               : None,     # FPGA device, defaults to the PHY's family.
    "speedgrade"           : -1,       # FPGA speedgrade (used for the OOB clock PLL).
    "sys_clk_freq"         : 125e6,    # sys clock frequency (>= 125MHz).
    "refclk"               : "pads",   # Transceiver refclk: "pads" (refclk_p/n ports) or "sys" (clk).
    "refclk_freq"          : 125e6,    # Transceiver refclk frequency.
    "role"                 : "device", # "device" (Upstream Port) or "host" (Downstream Port).
    "with_endianness_swap" : True,     # First symbol on the wire in tdata[31:24] (else tdata[7:0]).
    "with_link_ctrl"       : False,    # Expose the LTSSM Link-Layer control interface.
    "timer_scale"          : 1,        # LTSSM/LFPS timers scaling (simulation only, 1 for hardware).
    # USB3 Device Core (usb3_core): Link/Protocol Layers with Bulk IN/OUT AXI-Stream endpoints.
    "usb_core"             : False,    # Include the USB3 Device Core (device role).
    "usb_vid"              : 0x1d50,   # USB Vendor ID.
    "usb_pid"              : 0x605a,   # USB Product ID.
    "usb_manufacturer"     : "Enjoy-Digital",
    "usb_product"          : "USB3-PIPE",
    "usb_serial"           : "0001",
    "usb_max_burst"        : 8,        # Bulk endpoints MaxBurst (2/4/8/16 packets of 1024 bytes).
}

class ConfigError(ValueError):
    pass

def get_config(core_config):
    """Check a core configuration and fill it with default values."""
    unknown = set(core_config) - set(CONFIG_DEFAULTS)
    if unknown:
        raise ConfigError(f"Unknown configuration key(s): {', '.join(sorted(unknown))}.")
    config = {**CONFIG_DEFAULTS, **core_config}

    # Convert YAML elements to Python.
    replaces = {"False": False, "True": True, "None": None}
    for k, v in config.items():
        if isinstance(v, str) and v in replaces:
            config[k] = replaces[v]
    for k in ["sys_clk_freq", "refclk_freq"]:
        config[k] = int(float(config[k]))
    for k in ["speedgrade", "timer_scale", "usb_vid", "usb_pid", "usb_max_burst"]:
        config[k] = int(config[k])

    # Checks.
    if config["phy"] not in PHYS:
        raise ConfigError(f"Unsupported phy: {config['phy']} (supported: {', '.join(PHYS)}).")
    if config["device"] is None:
        config["device"] = PHYS[config["phy"]][1]
    if config["sys_clk_freq"] < 125e6:
        raise ConfigError(f"sys_clk_freq must be >= 125MHz (got {config['sys_clk_freq']/1e6}MHz).")
    if config["refclk"] not in ["pads", "sys"]:
        raise ConfigError(f"Unsupported refclk: {config['refclk']} (supported: pads, sys).")
    if config["refclk"] == "sys" and config["refclk_freq"] != config["sys_clk_freq"]:
        raise ConfigError("refclk_freq must be equal to sys_clk_freq with refclk: sys.")
    if config["role"] not in ["device", "host"]:
        raise ConfigError(f"Unsupported role: {config['role']} (supported: device, host).")
    if config["timer_scale"] < 1:
        raise ConfigError("timer_scale must be >= 1.")
    if config["usb_core"]:
        if config["role"] != "device":
            raise ConfigError("usb_core requires role: device.")
        if config["with_link_ctrl"]:
            raise ConfigError("usb_core and with_link_ctrl are exclusive (the core drives the Link-Layer control).")
        if not config["with_endianness_swap"]:
            raise ConfigError("usb_core requires with_endianness_swap: True.")
        if config["usb_max_burst"] not in [2, 4, 8, 16]:
            raise ConfigError(f"Unsupported usb_max_burst: {config['usb_max_burst']} (supported: 2, 4, 8, 16).")
    return config

# LTSSM States -------------------------------------------------------------------------------------

def get_ltssm_states():
    """Return the LTSSM states encoding of the ltssm_state port ({state name: value})."""
    ltssm = USB3LTSSM(sys_clk_freq=125e6)
    ltssm.finalize()
    return ltssm.fsm.encoding

def get_ltssm_state_localparam(name, state):
    """Return the Verilog localparam name of an LTSSM state (ex: USB3_PIPE_CORE_LTSSM_POLLING_LFPS)."""
    return re.sub(r"[^A-Z0-9]+", "_", f"{name}_LTSSM_{state}".upper())

# Ports --------------------------------------------------------------------------------------------

# Top-level port: name, direction (i/o), width, clock domain, description. Ports with a group are
# grouped in a platform resource (ex: tx_p/tx_n in "tx") for the SerDes pads.
Port = namedtuple("Port", ["name", "direction", "width", "domain", "description", "group"])

_STATUS_DESCRIPTIONS = {
    "u0"           : "Link in U0 (trained, ready for Link-Layer traffic).",
    "recovery"     : "LTSSM in Recovery.",
    "rx_ready"     : "RX symbol stream valid (source_* active).",
    "tx_ready"     : "TX symbol stream accepted (sink_* active).",
    "u1"           : "Link in U1 low-power state.",
    "u2"           : "Link in U2 low-power state.",
    "u3"           : "Link in U3 (suspend) state.",
    "compliance"   : "LTSSM in Compliance mode.",
    "loopback"     : "LTSSM in Loopback.",
    "hot_reset"    : "LTSSM in Hot Reset.",
    "ess_inactive" : "LTSSM in eSS.Inactive.",
    "polling_idle" : "LTSSM in Polling.Idle.",
}

_LINK_CTRL_DESCRIPTIONS = {
    "lgo_u1"            : "Partner LGO_U1 received (pulse).",
    "lgo_u2"            : "Partner LGO_U2 received (pulse).",
    "lgo_u3"            : "Partner LGO_U3 received (pulse).",
    "lau"               : "Our LGO_Ux accepted: LAU received (pulse).",
    "lxu"               : "Our LGO_Ux rejected: LXU received (pulse).",
    "lpma"              : "Partner LPMA received (pulse, accepted U1/U2 entered without PM_ENTRY_TIMER).",
    "req_u1"            : "Request entry to U1.",
    "req_u2"            : "Request entry to U2.",
    "req_u3"            : "Request entry to U3 (host only).",
    "tx_lgo_u1"         : "Link Layer must send LGO_U1 (pulse).",
    "tx_lgo_u2"         : "Link Layer must send LGO_U2 (pulse).",
    "tx_lgo_u3"         : "Link Layer must send LGO_U3 (pulse).",
    "tx_lau"            : "Link Layer must send LAU (pulse).",
    "tx_lxu"            : "Link Layer must send LXU (pulse).",
    "tx_lpma"           : "Link Layer must send LPMA (pulse).",
    "link_activity"     : "Link activity in the current 1ms window (tie to 1 if unused).",
    "directed_recovery" : "Direct the LTSSM to Recovery.",
    "hot_reset_req"     : "Request a Hot Reset (host only).",
    "warm_reset_req"    : "Request a Warm Reset (host only).",
    "exit_u1_req"       : "Request U1 exit.",
    "exit_u2_req"       : "Request U2 exit.",
    "wakeup_u3_req"     : "Request U3 wakeup (host only).",
}

def get_ports(config):
    """Return the top-level ports of the core for a configuration."""
    ports = []
    def add(name, direction, width, domain, description, group=None):
        ports.append(Port(name, direction, width, domain, description, group))

    # Clock/Reset.
    add("clk", "i", 1, "-",   f"sys clock ({config['sys_clk_freq']/1e6:g}MHz), clocks all user ports.")
    add("rst", "i", 1, "async", "Core reset (active high).")

    # Transceiver.
    if config["refclk"] == "pads":
        add("refclk_p", "i", 1, "-", f"Transceiver reference clock ({config['refclk_freq']/1e6:g}MHz) +.", "refclk")
        add("refclk_n", "i", 1, "-", f"Transceiver reference clock ({config['refclk_freq']/1e6:g}MHz) -.", "refclk")
    add("tx_p", "o", 1, "-", "SuperSpeed TX serial +.", "tx")
    add("tx_n", "o", 1, "-", "SuperSpeed TX serial -.", "tx")
    add("rx_p", "i", 1, "-", "SuperSpeed RX serial +.", "rx")
    add("rx_n", "i", 1, "-", "SuperSpeed RX serial -.", "rx")

    if config["usb_core"]:
        # AXI-Stream Bulk IN/OUT endpoints (USB3 Device Core).
        add("bulk_in_tvalid",  "i",  1, "sys", "Bulk IN (EP1, device -> host) data valid.")
        add("bulk_in_tready",  "o",  1, "sys", "Bulk IN data accepted.")
        add("bulk_in_tlast",   "i",  1, "sys", "Bulk IN end of packet (packets are also split at 1024 bytes).")
        add("bulk_in_tdata",   "i", 32, "sys", "Bulk IN data (first byte on tdata[7:0]).")
        add("bulk_in_tkeep",   "i",  4, "sys", "Bulk IN valid bytes (contiguous from tdata[7:0], partial with tlast only).")
        add("bulk_out_tvalid", "o",  1, "sys", "Bulk OUT (EP2, host -> device) data valid.")
        add("bulk_out_tready", "i",  1, "sys", "Bulk OUT data accepted.")
        add("bulk_out_tlast",  "o",  1, "sys", "Bulk OUT end of packet.")
        add("bulk_out_tdata",  "o", 32, "sys", "Bulk OUT data (first byte on tdata[7:0]).")
        add("bulk_out_tkeep",  "o",  4, "sys", "Bulk OUT valid bytes (contiguous from tdata[7:0], partial with tlast only).")
        add("usb_ready",       "o",  1, "sys", "USB3 link initialized (U0).")
        add("usb_configured",  "o",  1, "sys", "Device configured by the host (SET_CONFIGURATION).")
    else:
        # AXI-Stream Sink (TX symbols).
        add("sink_tvalid", "i",  1, "sys", "TX symbols valid.")
        add("sink_tready", "o",  1, "sys", "TX symbols accepted.")
        add("sink_tlast",  "i",  1, "sys", "Unused (ignored).")
        add("sink_tdata",  "i", 32, "sys", "TX symbols (4 x 8-bit).")
        add("sink_tuser",  "i",  4, "sys", "TX K-symbol flags (1 bit per symbol).")

        # AXI-Stream Source (RX symbols).
        add("source_tvalid", "o",  1, "sys", "RX symbols valid.")
        add("source_tready", "i",  1, "sys", "RX symbols accepted (must be kept high).")
        add("source_tlast",  "o",  1, "sys", "Unused (always 0).")
        add("source_tdata",  "o", 32, "sys", "RX symbols (4 x 8-bit).")
        add("source_tuser",  "o",  4, "sys", "RX K-symbol flags (1 bit per symbol).")

    # Status.
    add("ready", "o", 1, "sys", "Transceiver ready (PLL locked, reset done).")
    for name in USB3PHY_STATUS:
        add(name, "o", 1, "sys", _STATUS_DESCRIPTIONS[name])
    ltssm_states = get_ltssm_states()
    add("ltssm_state", "o", bits_for(max(ltssm_states.values())), "sys",
        "LTSSM state (see LTSSM states).")

    # Link-Layer control.
    if config["with_link_ctrl"]:
        for name, direction in USB3PHY_LINK_CTRL:
            add(name, direction, 1, "sys", _LINK_CTRL_DESCRIPTIONS[name])

    return ports

def get_ios(ports):
    """Convert ports to platform IOs (resources)."""
    ios    = []
    groups = {}
    for port in ports:
        if port.group is None:
            ios.append([port.name, 0, Pins(port.width)])
        else:
            if port.group not in groups:
                groups[port.group] = [port.group, 0]
                ios.append(groups[port.group])
            groups[port.group].append(Subsignal(port.name[len(port.group) + 1:], Pins(port.width)))
    return [tuple(io) for io in ios]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, speedgrade, pll_cls=S7PLL):
        self.cd_sys = ClockDomain()
        self.cd_oob = ClockDomain()

        # # #

        clk = platform.request("clk")
        rst = platform.request("rst")

        # sys: Directly from clk input (constrained by the user project).
        self.comb += self.cd_sys.clk.eq(clk)

        # oob: sys/8 for the transceiver OOB/SIGVALID clock.
        self.pll = pll = pll_cls(speedgrade=speedgrade)
        self.comb += pll.reset.eq(rst)
        pll.register_clkin(clk, sys_clk_freq)
        pll.create_clkout(self.cd_oob, sys_clk_freq/8, margin=0)

        # sys reset: rst or OOB PLL not locked.
        self.specials += AsyncResetSynchronizer(self.cd_sys, rst | ~pll.locked)

# Core ---------------------------------------------------------------------------------------------

class USB3PIPECore(SoCMini):
    def __init__(self, platform, config):
        self.ports = ports = get_ports(config)
        platform.add_extension(get_ios(ports))

        sys_clk_freq = config["sys_clk_freq"]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, config["speedgrade"], pll_cls=PHYS[config["phy"]][3])

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq,
            ident         = "USB3-PIPE standalone core",
            ident_version = True,
        )

        # SerDes -----------------------------------------------------------------------------------
        if config["refclk"] == "pads":
            refclk_pads = platform.request("refclk") # Constrained by the user project.
        else:
            refclk_pads = ClockSignal("sys")
        serdes_cls = PHYS[config["phy"]][0]
        self.serdes = serdes = serdes_cls(platform,
            sys_clk      = self.crg.cd_sys.clk,
            sys_clk_freq = sys_clk_freq,
            refclk_pads  = refclk_pads,
            refclk_freq  = config["refclk_freq"],
            tx_pads      = platform.request("tx"),
            rx_pads      = platform.request("rx"),
        )

        # PHY (PIPE + LTSSM) -----------------------------------------------------------------------
        self.phy = phy = USB3PHY(serdes,
            sys_clk_freq         = sys_clk_freq,
            role                 = config["role"],
            with_endianness_swap = config["with_endianness_swap"],
            timer_scale          = config["timer_scale"],
        )

        if config["usb_core"]:
            self.add_usb_core(platform, config, phy, sys_clk_freq)
        else:
            # AXI-Stream Sink (TX) -----------------------------------------------------------------
            self.comb += [
                phy.sink.valid.eq(platform.request("sink_tvalid")),
                platform.request("sink_tready").eq(phy.sink.ready),
                phy.sink.last.eq(platform.request("sink_tlast")),
                phy.sink.data.eq(platform.request("sink_tdata")),
                phy.sink.ctrl.eq(platform.request("sink_tuser")),
            ]

            # AXI-Stream Source (RX) ---------------------------------------------------------------
            self.comb += [
                platform.request("source_tvalid").eq(phy.source.valid),
                phy.source.ready.eq(platform.request("source_tready")),
                platform.request("source_tlast").eq(0),
                platform.request("source_tdata").eq(phy.source.data),
                platform.request("source_tuser").eq(phy.source.ctrl),
            ]

        # Status -----------------------------------------------------------------------------------
        self.comb += platform.request("ready").eq(phy.ready)
        for name in USB3PHY_STATUS:
            self.comb += platform.request(name).eq(getattr(phy, name))
        assert phy.ltssm_states == get_ltssm_states()
        self.comb += platform.request("ltssm_state").eq(phy.ltssm_state)

        # Link-Layer control (when disabled, LTSSM inputs keep their reset values) -----------------
        if config["with_link_ctrl"]:
            for name, direction in USB3PHY_LINK_CTRL:
                if direction == "i":
                    self.comb += getattr(phy, name).eq(platform.request(name))
                else:
                    self.comb += platform.request(name).eq(getattr(phy, name))

    def add_usb_core(self, platform, config, phy, sys_clk_freq):
        # Imported here: the PHY package does not depend on the USB3 Device Core (usb3_core).
        from usb3_core.device      import USB3DeviceCore
        from usb3_core.endpoints   import USB3BulkIN, USB3BulkOUT
        from usb3_core.descriptors import bulk_descriptors

        # USB3 Device Core -------------------------------------------------------------------------
        max_burst   = config["usb_max_burst"]
        descriptors = bulk_descriptors(
            vid          = config["usb_vid"],
            pid          = config["usb_pid"],
            manufacturer = config["usb_manufacturer"],
            product      = config["usb_product"],
            serial       = config["usb_serial"],
            max_burst    = max_burst)
        self.usb_core = usb_core = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
            USB3BulkIN(ept=1,  max_burst=max_burst),
            USB3BulkOUT(ept=2, max_burst=max_burst),
        ])
        bulk_in, bulk_out = usb_core.endpoints

        # Bulk IN (tkeep considered on the last word only, full words otherwise).
        in_tkeep = platform.request("bulk_in_tkeep")
        self.comb += [
            bulk_in.sink.valid.eq(platform.request("bulk_in_tvalid")),
            platform.request("bulk_in_tready").eq(bulk_in.sink.ready),
            bulk_in.sink.last.eq(platform.request("bulk_in_tlast")),
            bulk_in.sink.data.eq(platform.request("bulk_in_tdata")),
            bulk_in.sink.be.eq(Mux(bulk_in.sink.last, in_tkeep, 0b1111)),
        ]

        # Bulk OUT.
        out_tkeep = platform.request("bulk_out_tkeep")
        self.comb += [
            platform.request("bulk_out_tvalid").eq(bulk_out.source.valid),
            bulk_out.source.ready.eq(platform.request("bulk_out_tready")),
            platform.request("bulk_out_tlast").eq(bulk_out.source.last),
            platform.request("bulk_out_tdata").eq(bulk_out.source.data),
            out_tkeep.eq(bulk_out.source.be),
            platform.request("usb_ready").eq(usb_core.ready),
            platform.request("usb_configured").eq(usb_core.configured),
        ]

# Documentation ------------------------------------------------------------------------------------

def generate_instance_template(name, ports):
    """Generate a Verilog instantiation template of the core."""
    width = max(len(port.name) for port in ports)
    lines = []
    lines.append(f"// {name} instantiation template (generated by usb3_pipe_gen).")
    lines.append(f"{name} {name}_inst (")
    for i, port in enumerate(ports):
        sep = "," if i < len(ports) - 1 else ""
        lines.append(f"    .{port.name:<{width}} ({port.name:<{width}}){sep} // {port.direction}")
    lines.append(");")
    return "\n".join(lines) + "\n"

def generate_ltssm_header(name, ports):
    """Generate a Verilog header with the LTSSM states encoding (ltssm_state port)."""
    width  = {port.name: port.width for port in ports}["ltssm_state"]
    states = get_ltssm_states()
    pwidth = max(len(get_ltssm_state_localparam(name, state)) for state in states)
    lines  = []
    lines.append(f"// {name} LTSSM states (ltssm_state port encoding, generated by usb3_pipe_gen).")
    for state, value in states.items():
        lines.append(f"localparam {get_ltssm_state_localparam(name, state):<{pwidth}} = {width}'d{value};")
    return "\n".join(lines) + "\n"

def generate_documentation(name, config, ports):
    """Generate the Markdown interface documentation of the core."""
    first_symbol = "tdata[31:24]/tuser[3]" if config["with_endianness_swap"] else "tdata[7:0]/tuser[0]"
    refclk = {
        "pads" : f"`refclk_p/n` ports ({config['refclk_freq']/1e6:g}MHz, dedicated GT clock pins)",
        "sys"  : f"`clk` ({config['sys_clk_freq']/1e6:g}MHz, routed from the fabric)",
    }[config["refclk"]]

    d = []
    d.append(f"# {name}")
    d.append("")
    d.append("USB3.0 SuperSpeed (5Gbps) PHY generated by `usb3_pipe_gen`: FPGA transceiver, USB3 PIPE")
    d.append("(LFPS, Training Sequences, scrambling, clock compensation) and LTSSM. It provides a")
    d.append("link-trained 32-bit symbol stream for a USB3.0 Link Layer.")
    if config["usb_core"]:
        d.append("")
        d.append("With `usb_core`, the core also includes the USB3 Device Core: a complete USB3.0 device")
        d.append("with Bulk IN/OUT AXI-Stream endpoints (see USB3 Device Core).")
    d.append("")
    d.append("## Configuration")
    d.append("")
    d.append("| Parameter | Value |")
    d.append("|---|---|")
    for k in CONFIG_DEFAULTS:
        d.append(f"| `{k}` | `{config[k]}` |")
    d.append("")
    d.append("## Files")
    d.append("")
    d.append(f"- `gateware/{name}.v`: Verilog of the core (top-level module `{name}`).")
    d.append(f"- `gateware/{name}.xdc`: Timing constraints of the core internal clocks/CDCs.")
    d.append("- `gateware/*.init`: Memories initialization files (loaded by `$readmemh`).")
    d.append(f"- `{name}_inst.v`: Instantiation template.")
    d.append(f"- `{name}_ltssm.vh`: LTSSM states encoding (`localparam`s for `ltssm_state`).")
    d.append("")
    d.append("## Ports")
    d.append("")
    d.append("| Port | Dir | Width | Domain | Description |")
    d.append("|---|---|---|---|---|")
    for port in ports:
        direction = {"i": "in", "o": "out"}[port.direction]
        d.append(f"| `{port.name}` | {direction} | {port.width} | {port.domain} | {port.description} |")
    d.append("")
    d.append("## Clocking and reset")
    d.append("")
    d.append(f"- `clk` ({config['sys_clk_freq']/1e6:g}MHz) clocks the core and all user ports.")
    d.append("- An internal PLL generates the transceiver OOB clock (`clk`/8) from `clk`.")
    d.append(f"- Transceiver reference clock: {refclk}.")
    d.append("- `rst` is asynchronous and active high. The core stays in reset while `rst` is")
    d.append("  asserted or the internal PLL is not locked.")
    d.append("")
    if config["usb_core"]:
        d.append("## USB3 Device Core")
        d.append("")
        d.append("The core includes the USB3 Device Core (`usb3_core`): Link and Protocol Layers, Control")
        d.append(f"Endpoint (descriptors: VID/PID {config['usb_vid']:04x}:{config['usb_pid']:04x}, vendor class")
        d.append(f"interface) and two Bulk endpoints (MaxBurst {config['usb_max_burst']}) exposed as AXI-Stream:")
        d.append("")
        d.append("- `bulk_in_*` (EP1 IN, device -> host): packets of up to 1024 bytes, ended by `tlast`")
        d.append("  (short packet) or split at 1024 bytes. `tkeep` gives the valid bytes of the last word.")
        d.append("- `bulk_out_*` (EP2 OUT, host -> device): received packets, `tlast` on the last word.")
        d.append("- First byte of the data on `tdata[7:0]`.")
        d.append("- `usb_ready`: link initialized, `usb_configured`: device configured by the host.")
        d.append("- Host side: any libusb application (bulk transfers on endpoints 0x81/0x02).")
        d.append("")
    else:
        d.append("## Data format")
        d.append("")
        d.append("- `tdata` carries 4 USB3 symbols (bytes) per `clk` cycle, `tuser` the matching K-symbol")
        d.append("  flags (`tuser[i]` = 1: `tdata[8*i+7:8*i]` is a K-symbol).")
        d.append(f"- First symbol on the wire: `{first_symbol}`.")
        d.append("- Symbols are unscrambled: scrambling/descrambling, 8b/10b, SKP ordered sets")
        d.append("  insertion/removal, LFPS and Training Sequences are handled by the core.")
        d.append("")
        d.append("## Link bring-up and flow control")
        d.append("")
        d.append("- `ready` goes high when the transceiver is ready. The LTSSM then trains the link")
        d.append(f"  (role: {config['role']}) and `u0` goes high when the link is in U0.")
        d.append("- Only send/consume Link-Layer traffic when `u0` is high.")
        d.append("- TX: `sink_tready` provides back-pressure (low when `tx_ready` is low or during SKP")
        d.append("  insertion).")
        d.append("- RX: `source_tvalid` is only active when `rx_ready` is high. `source_tready` must be kept")
        d.append("  high: RX symbols arrive at the line rate and back-pressure is not supported.")
        if config["with_link_ctrl"]:
            d.append("- The Link-Layer control ports implement the PM handshake (LGO_Ux/LAU/LXU/LPMA, decoded")
            d.append("  or sent by the user Link Layer) and directed LTSSM transitions.")
        else:
            d.append("- Link-Layer control (PM handshake, directed recovery/resets) is not exposed")
            d.append("  (`with_link_ctrl: False`): the core never requests or accepts U1/U2/U3 entry.")
        d.append("")
    d.append("## LTSSM states")
    d.append("")
    d.append(f"`ltssm_state` encoding (also available as `localparam`s in `{name}_ltssm.vh`):")
    d.append("")
    d.append("| Value | State |")
    d.append("|---|---|")
    for state, value in get_ltssm_states().items():
        d.append(f"| {value} | {state} |")
    d.append("")
    d.append("## Integration notes")
    d.append("")
    d.append(f"- Add `gateware/{name}.v` and the `gateware/*.init` files to the project, ex in Vivado:")
    d.append(f"  `read_verilog {name}.v` and `add_files [glob *.init]`.")
    d.append(f"- Add `gateware/{name}.xdc` scoped to the core: `read_xdc -ref {name} {name}.xdc`")
    d.append(f"  (or `set_property SCOPED_TO_REF {name}` on the file in project mode).")
    d.append("- `clk` (and `refclk_p/n`) must be constrained by the user project (`create_clock` on the")
    d.append("  top-level pins or generated by a MMCM/PLL); the core only constrains its internal clocks.")
    d.append("- Transceiver/pin placement (LOC) constraints for `tx_p/n`, `rx_p/n` (and `refclk_p/n`) are")
    d.append("  to be provided by the user project.")
    d.append("- `timer_scale` must be 1 for hardware (values > 1 are only for simulation).")
    d.append("")
    return "\n".join(d)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="USB3-PIPE standalone core generator")
    builder_args(parser)
    parser.set_defaults(output_dir="build")
    parser.add_argument("config", help="YAML config file")
    parser.add_argument("--name", default="usb3_pipe_core", help="Standalone core/module name")
    args = parser.parse_args()

    # Configuration.
    with open(args.config, "r", encoding="utf-8") as f:
        core_config = yaml.load(f, Loader=yaml.SafeLoader) or {}
    try:
        config = get_config(core_config)
    except ConfigError as e:
        parser.error(f"{args.config}: {e}")

    # Core.
    platform = PHYS[config["phy"]][2](config["device"], io=[], toolchain="vivado")
    soc      = USB3PIPECore(platform, config)

    # Build (Verilog/Constraints only).
    builder_arguments = builder_argdict(args)
    builder_arguments["compile_gateware"] = False
    if builder_arguments["csr_csv"] is None:
        builder_arguments["csr_csv"] = os.path.join(builder_arguments["output_dir"], "csr.csv")
    builder = Builder(soc, **builder_arguments)
    builder.build(build_name=args.name)

    # Instantiation template / LTSSM states header / Documentation.
    with open(os.path.join(builder.output_dir, f"{args.name}_inst.v"), "w") as f:
        f.write(generate_instance_template(args.name, soc.ports))
    with open(os.path.join(builder.output_dir, f"{args.name}_ltssm.vh"), "w") as f:
        f.write(generate_ltssm_header(args.name, soc.ports))
    with open(os.path.join(builder.output_dir, f"{args.name}.md"), "w") as f:
        f.write(generate_documentation(args.name, config, soc.ports))

if __name__ == "__main__":
    main()
