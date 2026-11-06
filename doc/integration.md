# Integration in a LiteX SoC

Using the USB3 PHY (`usb3_pipe`) and the USB3 Device Core (`usb3_core`) in your own LiteX SoC. The
bench targets ([targets.md](targets.md)) do this through `bench/usb3_soc.py` (`add_usb3_device`:
PHY + debug CSRs + an example design of `usb3_core/examples.py`): the closest target and example
are the reference to copy. Designs not built with LiteX use the standalone Verilog core
([standalone-core.md](standalone-core.md), Gen 1x1 on Xilinx only).

## SerDes

One `USB3SerDes` subclass per transceiver (`usb3_pipe/serdes.py`), on the LiteICLink transceiver
wrappers with the USB3 settings (SSC tolerant CDR, LFPS/electrical idle, 8b/10b, polarity):

| Class | Transceiver | Line rate | Reference clock | Notes |
|---|---|---|---|---|
| `A7USB3SerDes` | Artix7 GTP | 5Gbps | Dedicated pads, or a fabric clock (`refclk_pads=ClockSignal(...)`, driven to GTGREFCLK0) | `rxcdr_cfg`: RXCDR_CFG override. Needs an `oob` clock domain at sys/8 (SIGVALIDCLK). |
| `K7USB3SerDes` | Kintex7 GTX | 5Gbps | Dedicated pads (ex: 125MHz) | Needs an `oob` clock domain at sys/8 (CLKRSVD). |
| `USPUSB3SerDes` | UltraScale+ GTY | 5Gbps, 10Gbps with `gen2_capable=True` | Dedicated pads (ex: 156.25MHz) or a fabric clock | Gen 2: QPLL at 10Gbps, 40-bit raw words, Gen 1 at the /2 rate. |
| `GW5USB3SerDes` | Gowin GW5AST-138B (Q1 lanes 0/1) / GW5AT-15 (Q0 lane 0) GTR12 | 5Gbps, 10Gbps with `gen2_capable=True` (Q1 lane 0, CPLL) | Quad REFCLK1 100MHz or REFCLK0 125MHz (`refclk`, `pll`) | Register configuration from `usb3_pipe/gw5`, runtime register access (UPAR, `upar_clk`), 40-bit words at 125MHz, 80-bit raw words at Gen 2, `xcvr_cd`/`tx_cdc` for the `pipe_cd` designs. |

The SerDes adds the period constraints of its TX/RX clocks and the false paths with `sys`; the user
design constrains `sys` (and the reference clock pads) as usual. The SerDes is not a submodule of
the PHY: add both to the SoC.

```python
from usb3_pipe     import A7USB3SerDes
from usb3_pipe.phy import USB3PHY

self.usb3_serdes = A7USB3SerDes(platform,
    sys_clk      = self.crg.cd_sys.clk,
    sys_clk_freq = sys_clk_freq,
    refclk_pads  = platform.request("refclk"), # Or a fabric clock (ClockSignal).
    refclk_freq  = 125e6,
    tx_pads      = platform.request("sfp_tx"),
    rx_pads      = platform.request("sfp_rx"))
self.usb3_phy = USB3PHY(serdes=self.usb3_serdes, sys_clk_freq=sys_clk_freq, role="device")
```

Gen 2x1 (UltraScale+ GTY, as `bench/alibaba_xcku3p.py --gen2-capable`):

```python
from usb3_pipe     import USPUSB3SerDes
from usb3_pipe.phy import USB3PHY

self.usb3_serdes = USPUSB3SerDes(platform,
    sys_clk      = self.crg.cd_sys.clk,
    sys_clk_freq = sys_clk_freq,
    refclk_pads  = platform.request("sfp_mgt_clk"),
    refclk_freq  = 156.25e6,
    tx_pads      = platform.request("sfp_tx"),
    rx_pads      = platform.request("sfp_rx"),
    gen2_capable = True)  # QPLL at 10Gbps, 40-bit raw words, Gen 1 at the /2 rate.
self.usb3_phy = USB3PHY(serdes=self.usb3_serdes, sys_clk_freq=sys_clk_freq, role="device",
    gen2_capable = True,  # Gen 2x1 advertised in the SuperSpeedPlus Polling.
    link_words   = 2)     # Optional: 64-bit Link interface (Gen 2 bandwidth, 64-bit endpoints).
```

## PHY (`USB3PHY`)

`USB3PHY(serdes, sys_clk_freq, ...)` ([usb3-pipe.md](usb3-pipe.md)), sys clock domain:

| Parameter | Default | Description |
|---|---|---|
| `serdes` | | The SerDes (a list of two with `lanes=2`). |
| `sys_clk_freq` | | Link interface clock frequency: >= 125MHz, or the Link Layer clock with `pipe_cd`. |
| `role` | `device` | `device` (Upstream Port) or `host` (Downstream Port). |
| `gen2_capable` | `False` | Gen 2x1 advertised in the SuperSpeedPlus Polling; 10Gbps operation on the `Gen2PIPE` with a `gen2_capable` SerDes (else the training continues at Gen 1 speed). |
| `link_words` | `1` | Link interface words per cycle: 1 (32-bit), 2 (64-bit pairs), 4 (128-bit quads: `pipe_cd` and a Gen 2 capable SerDes with raw words wider than 40 bits), [usb3-pipe.md](usb3-pipe.md) "Link Layer words". |
| `pipe_cd`, `pipe_clk_freq`, `pipe_rx_depth` | `None` | PIPE (and SerDes, renamed by the caller) in the `pipe_cd` clock domain at `pipe_clk_freq`, LTSSM and Link interface in `sys` (below); RX CDC FIFO depth override. |
| `with_u1u2` | `True` | `False`: no U1/U2 in the LTSSM (less logic, LGO_U1/LGO_U2 rejected by the Link Layer). |
| `with_endianness_swap` | `True` | First symbol on `data[31:24]` (`False`: on `data[7:0]`). |
| `timer_scale` | `1` | LTSSM/LFPS timers scaling (simulation only). |
| `lanes`, `x2_capable`, `deskew_depth` | `1`, `True`, `8` | Gen 1x2 (simulation only, [gen2-x2-roadmap.md](gen2-x2-roadmap.md)). |

Other signals: `reset` (PHY reset, `sys`: the link trains again from Polling; with `pipe_cd` the
reset is resynchronized and also resets the PIPE/Gen2PIPE/CDC), `serdes_rx_reset` (transceiver RX
re-initialization request), `ready` (SerDes ready), `ltssm_state` with the `ltssm_states` encoding
dict (`usb3_pipe.gen.get_ltssm_states()` for the host tools), `u2_timeout` (U2 Inactivity Timeout
from the Link Layer).

## Link interface

What a USB3 Link Layer needs from the PHY (the USB3 Device Core only uses these; the signals are
the LTSSM ones, undriven inputs keep their reset values):

- `sink`/`source`: TX/RX symbol streams. `link_words=1`: `data` 32, `ctrl` 4 (K-symbol flags,
  first symbol on `data[31:24]`); `link_words=2|4`: `data` 64/128, `ctrl` 8/16, `words` (1 to
  `link_words` valid words, word 0 first). Logical Idle is sent when `sink.valid` is low,
  `sink.ready` is low during the SKP insertions; `source` has no back-pressure (framing words
  aligned). The standalone core and the `pipe_cd` TX path need `sink.last` (end of packet: SKP
  Ordered Sets only between the packets).
- `source_lc`, `source_lc_enable`: Gen 2 only (LGOOD_n/LCRD_x received ahead of the buffered RX
  words when the Link Layer sets `source_lc_enable`; `source_lc` is `None` without `gen2_capable`).

Status (`USB3PHY_STATUS`, PHY -> Link):

| Signal | Description |
|---|---|
| `u0`, `recovery`, `u1`, `u2`, `u3`, `compliance`, `loopback`, `hot_reset`, `ess_inactive`, `polling_idle` | LTSSM in that state/substate (`polling_idle`/`hot_reset`: link reset for the Link Layer). |
| `rx_ready`, `tx_ready` | RX words delivered to / TX words accepted from the Link Layer (U0). |
| `rx_keep` | RX words kept for the Link Layer until U0 (Recovery.Idle: the partner may already send its Advertisement). |

Link-Layer control (`USB3PHY_LINK_CTRL`, PM handshake and directed transitions, 7.2.4.2):

| Signal | Direction | Description |
|---|---|---|
| `lgo_u1`, `lgo_u2`, `lgo_u3` | Link -> PHY | Partner LGO_Ux decoded (pulse). |
| `lau`, `lxu`, `lpma` | Link -> PHY | LAU/LXU received for our LGO_Ux, partner LPMA received (pulses). |
| `req_u1`, `req_u2`, `req_u3` | Link -> PHY | Upper layer requests a Ux entry (device initiated entry, not used by the Device Core). |
| `tx_lgo_u1`, `tx_lgo_u2`, `tx_lgo_u3`, `tx_lau`, `tx_lxu`, `tx_lpma` | PHY -> Link | Link Command to send (pulses). |
| `link_activity` | Link -> PHY | Link active in the current window (U0 inactivity, reset value 1). |
| `directed_recovery`, `hot_reset_req`, `warm_reset_req` | Link -> PHY | Directed Recovery, Hot Reset, Warm Reset (host role). |
| `exit_u1_req`, `exit_u2_req`, `wakeup_u3_req` | Link -> PHY | U1/U2 exit, U3 wakeup (device remote wakeup). |
| `u2_timeout` (8) | Link -> PHY | U2 Inactivity Timeout LMP value (256us units, 00h/FFh: disabled): U1 -> U2 on timeout. |

Capabilities and flow control (`USB3PHY_LINK_INTERFACE`, always defined, constants when the
feature is absent):

| Attribute | Description |
|---|---|
| `link_words` | Link words per cycle: 1, 2 or 4. |
| `with_u1u2` | U1/U2 Link Power Management supported (LGO_U1/LGO_U2 accepted). |
| `ssp_capable` | Gen 2 operation possible: the Link Layer builds the SuperSpeedPlus rules (4-bit Header Sequence Numbers, Type 1/Type 2 credits). |
| `gen2_active` | Gen 2 (10Gbps) operation (`Constant(0)` when not `ssp_capable`). |
| `source_lc` | Fast Link Commands stream, `None` without Gen 2. |
| `rx_throttle` | RX words buffered (`pipe_cd`): the Link Layer holds its Rx Header Buffer credits (`Constant(0)` without). |
| `rx_t2_credits` | Type 2 credits the partner may hold: 4 (no limit) or a Signal (runtime, `pipe_cd` Gen 2). |

## USB3 Device Core (`USB3DeviceCore`)

```python
from usb3_core.device      import USB3DeviceCore
from usb3_core.bulk        import USB3BulkIN, USB3BulkOUT
from usb3_core.descriptors import bulk_descriptors

self.usb3_core = USB3DeviceCore(usb3_phy, sys_clk_freq,
    descriptors = bulk_descriptors(vid=0x1d50, pid=0x605a, max_burst=8),
    endpoints   = [USB3BulkIN(ept=1, max_burst=8), USB3BulkOUT(ept=2, max_burst=8)])
self.usb3_core.add_csr(with_trace=True) # Status/statistics CSRs, Header Packets trace.
# endpoints[0].sink: Device -> Host packets, endpoints[1].source: Host -> Device packets.
```

`USB3DeviceCore(phy, sys_clk_freq, descriptors, endpoints, with_lpm, with_ep0_handler,
crc32_cascade, timer_scale)` ([usb3-core.md](usb3-core.md)):

- `descriptors`: `USB3Descriptors` (device/configuration/BOS/strings builder), from the helpers
  `bulk_descriptors(in_ept, out_ept, max_burst, vid, pid, ...)`, `iso_in_descriptors(ept,
  max_burst, interval, ...)` or a list of `USB3InterfaceDescriptor` (composite devices, device
  classes: `uvc_interfaces()`, `cdc_acm_interfaces()`). The BOS advertises the SuperSpeedPlus
  capability when the PHY is `ssp_capable`, non-zero U1/U2 exit latencies with `with_lpm`.
- `endpoints`: `USB3BulkIN(ept, max_burst, data_width)`, `USB3BulkOUT(ept, max_burst,
  data_width)` (`max_burst` 2/4/8/16 packet slots of 1024 bytes), `USB3IsoIN(ept, max_burst)`.
  EP0 is built in. `data_width` is 32, or the Link Layer width (`32*link_words`: 64/128-bit Bulk
  endpoints), 32-bit endpoints on a wider Link Layer are adapted by the Protocol Layer.
- Endpoint user streams (`endpoint_layout(data_width)`: `data`, `be`, with `valid`/`ready`/`last`):
  IN `sink` (packets to send: first byte on `data[7:0]`, `be` valid bytes, `last` end of packet,
  longer packets split at 1024 bytes, `flush`), OUT `source` (received packets, `be` full except on
  the last word, `last` end of packet).
- Status: `ready` (U0 and link initialized), `configured`, `u1_enable`/`u2_enable`; `ep0`:
  `dev_addr`, `set_interface`/`interface`/`alt_setting`, and the class/vendor requests handler
  with `with_ep0_handler=True` (`req_*`, `USB3EP0Wishbone` for a Wishbone master over USB).
- Device classes (`usb3_core/cdc_acm.py`, `uvc.py`, `msc.py`): complete devices
  (`USB3CDCACM(phy, sys_clk_freq)`, `USB3UVC`, `USB3MSC`) or functions attached to a core's EP0 and
  endpoints (`USB3CDCACMFunction`, `USB3UVCFunction`), see the `composite` example.

## Debug CSRs and host tools

```python
from usb3_pipe.debug import USB3PHYDebug, USB3SerDesDebug, usb3_analyzer_groups

self.usb3_debug        = USB3PHYDebug(self.usb3_phy, self.usb3_serdes, sys_clk_freq) # usb3_debug_*
self.usb3_serdes_debug = USB3SerDesDebug(self.usb3_serdes, sys_clk_freq,           # Optional.
    ltssm_state = self.usb3_phy.ltssm_state)
self.usb3_core.add_csr(with_trace=True)                                            # usb3_core_*
```

`bench/usb3_debug.py` expects the `usb3_debug` CSR prefix (status, history, counters, retrain),
`bench/usb3_trace.py` autodetects the core prefix (`usb3_core`, or `usb3_<class>_core` in the
single class examples, `--prefix`). LiteScope: `LiteScopeAnalyzer(usb3_analyzer_groups(phy, serdes,
serdes_debug)[group], ...)` with `bench/usb3_analyzer.py`. The `--lean` option of the targets
leaves out the PHY debug CSRs, the throughput profile and the core trace. Everything is described
in [debugging-guide.md](debugging-guide.md).

## Slower fabrics (`pipe_cd`)

When the LTSSM/Link Layer/Core cannot run at 125MHz (Gowin GW5A), the PIPE and the SerDes
datapath run in their own clock domain and the Link interface at a lower frequency on 64-bit
words ([usb3-pipe.md](usb3-pipe.md) "Slower fabrics"), as in `bench/sipeed_tang_mega_138k_pro.py`:

```python
# sys at 62.5MHz. SerDes control logic on sys ("gtr12"), SerDes datapath renamed to the GTR12 TX
# clock ("tx", frequency-locked to the transmitted words: no TX CDC in the SerDes).
usb3_serdes = GW5USB3SerDes(platform,
    sys_clk       = self.crg.cd_sys.clk,
    sys_clk_freq  = 125e6,          # PIPE/SerDes datapath frequency.
    upar_clk      = self.crg.cd_sys.clk,
    xcvr_cd       = "gtr12",        # cd_gtr12 = sys (free-running), created by the SoC.
    xcvr_clk_freq = 62.5e6,
    tx_cdc        = False)
usb3_serdes = ClockDomainsRenamer({"sys": "tx"})(usb3_serdes)
self.usb3_serdes = usb3_serdes
self.usb3_phy    = USB3PHY(usb3_serdes, sys_clk_freq=62.5e6, role="device",
    link_words    = 2,              # 64-bit Link interface (4 with a Gen 2 capable SerDes).
    pipe_cd       = "tx",
    pipe_clk_freq = 125e6,
    with_u1u2     = False)          # No U1/U2: less logic (U3 kept).
```

The Device Core then uses 64-bit Bulk endpoints (`data_width=64`); the PHY `rx_throttle` holds the
Link Layer credits when the RX FIFO fills (no change in the core). `USB3PHYDebug` works with
`pipe_cd` (the SerDes levels/pulses are resynchronized); `USB3SerDesDebug` and the throughput
profile of the bench are not used with `pipe_cd` by the targets.
