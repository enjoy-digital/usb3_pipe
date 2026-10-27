#
# This file is part of USB3-PIPE project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""USB3 Device Core (SuperSpeed Gen1 Link and Protocol Layers, Control Endpoint, Bulk/Isochronous
Endpoints).

Connects to a USB3 PHY through its Link interface (see usb3_pipe.phy.USB3PHY): any PHY providing the
same interface can be used.

Example:
    descriptors = bulk_descriptors(max_burst=8)
    usb3 = USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints=[
        USB3BulkIN(ept=1,  max_burst=8),
        USB3BulkOUT(ept=2, max_burst=8),
    ])
    # usb3.endpoints[0].sink  : Device -> Host packets.
    # usb3.endpoints[1].source: Host -> Device packets.

with_lpm: U1/U2 Link Power Management (USB 3.2 7.2.4.2): non-zero U1/U2 exit latencies advertised in
the BOS, LGO_U1/LGO_U2 accepted by the Link Layer when idle (else rejected with LXU), U1/U2 exited
when packets are to be transmitted. Device initiated U1/U2 entry is not supported (U1/U2 Enable only
tracked/reported). Without, LGO_U1/LGO_U2 are rejected (U1/U2 not used by the host).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from usb3_core.common    import TT_CONTROL, TT_ISOCHRONOUS, TT_BULK
from usb3_core.link      import USB3LinkLayer
from usb3_core.iso       import USB3IsoIN
from usb3_core.protocol  import USB3ProtocolLayer
from usb3_core.control   import USB3EndpointControl
from usb3_core.trace     import USB3CoreTrace

# USB3 Device Core ---------------------------------------------------------------------------------

class USB3DeviceCore(LiteXModule):
    def __init__(self, phy, sys_clk_freq, descriptors, endpoints=None, timer_scale=1,
        with_ep0_handler=False, with_lpm=False):
        endpoints      = [] if endpoints is None else endpoints
        self.endpoints = endpoints

        # # #

        # Link Power Management: U1/U2 exit latencies in the BOS.
        if with_lpm:
            descriptors.enable_lpm()

        # Link Layer (Transfer Types of the endpoints for SuperSpeedPlus operation).
        ssp_tt = {0: TT_CONTROL}
        for ep in endpoints:
            ssp_tt[ep.ept] = TT_ISOCHRONOUS if isinstance(ep, USB3IsoIN) else TT_BULK
        self.link = link = USB3LinkLayer(phy, sys_clk_freq=sys_clk_freq, timer_scale=timer_scale,
            ssp_tt=ssp_tt)
        self.comb += link.lpm_enable.eq(with_lpm)

        # Control Endpoint.
        self.ep0 = ep0 = ResetInserter()(USB3EndpointControl(descriptors,
            with_handler = with_ep0_handler))
        self.comb += ep0.reset.eq(link.reset)

        # Endpoints (Sequence Numbers reset on link reset, SET_CONFIGURATION, CLEAR_FEATURE(HALT)).
        for ep in endpoints:
            self.add_module(name=f"ep{ep.ept}_{['out', 'in'][ep.direction]}", module=ep)
            self.comb += ep.reset_seq.eq(link.reset | ep0.reset_seqs |
                (ep0.clear_halt & (ep0.clear_ept == ep.ept) & (ep0.clear_dir == ep.direction)))

        # Protocol Layer.
        self.protocol = ResetInserter()(USB3ProtocolLayer(link, [ep0] + endpoints, ep0.dev_addr))
        self.comb += [
            self.protocol.reset.eq(link.reset),
            link.pm_busy.eq(self.protocol.pm_busy),
        ]

        # Status.
        self.ready      = link.ready
        self.configured = ep0.configured
        self.u1_enable  = ep0.u1_enable
        self.u2_enable  = ep0.u2_enable

    def add_csr(self, with_trace=False, trace_depth=512):
        """Status/statistics CSRs (bench/usb3_debug.py), optional Header Packets trace
        (bench/usb3_trace.py)."""
        if with_trace:
            self.trace = USB3CoreTrace(self.link, depth=trace_depth)
        self._status = CSRStatus(fields=[
            CSRField("ready",      size=1, description="U0, link initialized."),
            CSRField("configured", size=1, description="Configured (SET_CONFIGURATION)."),
            CSRField("address",    size=7, offset=8, description="Device Address."),
        ])
        self._rx_lbad      = CSRStatus(16, description="LBAD sent (Header Packets RX errors).")
        self._tx_retry     = CSRStatus(16, description="LBAD received (Header Packets resent).")
        self._recovery_req = CSRStatus(16, description="Recovery requests (timeouts, errors).")
        self.comb += [
            self._status.fields.ready.eq(self.link.ready),
            self._status.fields.configured.eq(self.ep0.configured),
            self._status.fields.address.eq(self.ep0.dev_addr),
            self._rx_lbad.status.eq(self.link.rx_lbad),
            self._tx_retry.status.eq(self.link.tx_retry),
            self._recovery_req.status.eq(self.link.recovery_req),
        ]
