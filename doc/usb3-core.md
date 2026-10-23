# USB3 Device Core (`usb3_core`)

Migen/LiteX USB3.0 SuperSpeed (Gen 1x1, 5Gbps) device core: Link Layer, Protocol Layer, Control
Endpoint and Bulk Endpoints. It runs at the PIPE rate (32-bit words at 125MHz) in the `sys` clock
domain of the PHY.

```
            USB3PHY (usb3_pipe)                         USB3DeviceCore (usb3_core)
 SerDes -> PIPE -> LTSSM                       link.py        protocol.py           endpoints
   source (data/ctrl) ------------------>  RX parser  --> Header dispatch ---> EP0 (control.py)
   sink   (data/ctrl) <------------------  TX framer  <-- TX arbitration <--- Bulk IN/OUT
   u0/polling_idle/hot_reset ----------->  Header flow control (seq/credits/retry)
   lgo_u1/2/3, tx_lau/tx_lxu <---------->  Link PM (U3 accepted, U1/U2 rejected)
   directed_recovery, link_activity <---   Timers / Recovery requests
```

## PHY interface

The core only uses the `USB3PHY` Link interface (see `usb3_pipe/phy.py`), any PHY providing it can be
used:
- `sink`/`source`: 32-bit data + 4-bit ctrl (K-symbols) words, first symbol on `data[31:24]`. The PHY
  sends Logical Idle when `sink.valid` is low, `sink.ready` is low during SKP insertions; `source`
  can't be back-pressured.
- Status: `u0`, `polling_idle`, `hot_reset` (link reset: Header Sequence Numbers to 0).
- Link-Layer control: `lgo_u1/2/3` (LGO_Ux received), `tx_lau`/`tx_lxu` (LTSSM decision),
  `directed_recovery` (Link Layer errors/timeouts), `link_activity` (LTSSM U0 recovery timeout).

## Link Layer (`link.py`, USB 3.2 chapter 7)

- RX: framing parser (SHP/SLC/SDP framing words aligned by the PHY), CRC-16/CRC-5 checks of Header
  Packets/Link Control Words, CRC-32 of Data Packet Payloads (length from the DPH, any byte count).
- Header Packets flow control: 3-bit Header Sequence Numbers, 4 Rx/Tx Header Buffers, LGOOD_n/LCRD_x,
  LBAD on errors (packets ignored until LRTY), LRTY + retransmission on LBAD (DL bit set, DPP not
  resent: the Protocol Layer retries), Advertisements on each U0 entry (state kept across Recovery).
- Recovery requested on: Header Sequence Number/LGOOD/LCRD order errors, PENDING_HP_TIMER (3us),
  CREDIT_HP_TIMER (5ms).
- U0 keep-alive (LUP after 10us without transmission), LMPs (Port Capability, Port Configuration
  Response, Set Link Function), LGO_U3 accepted through the LTSSM, LGO_U1/U2 rejected (LXU) unless
  `u1_u2_enable` or Force_LinkPM_Accept.

## Protocol Layer and endpoints

- `protocol.py`: received headers dispatched in order to all endpoints (broadcast + strobe), DPHs
  processed with the status of their DPP, TP responses queued (with the Device Address of their
  generation), PING_RESPONSE, TX arbitration (responses first), DPPs sent in DPH order.
- `endpoints.py`:
  - `USB3BulkIN(ept, max_burst)`: `sink` stream (data, last, last_be), packets split at 1024 bytes,
    rings of `max_burst` slots, sent in bursts within the host window (ACK TP NumP), kept until
    acknowledged (retries), flow control: EOB/NRDY -> ERDY -> new host request. The last max size
    packet is held while the user is still writing (bursts continued instead of EOB round trips).
  - `USB3BulkOUT(ept, max_burst)`: `source` stream, DPPs landed as received, acknowledged in order
    (NumP: free slots, retry on CRC errors), ERDY after a NumP of 0.
- `control.py`: EP0 requests from a table built by `descriptors.py` (standard requests, class/vendor
  requests with constant IN responses or accepted OUT data), unsupported requests stalled.
- `descriptors.py`: `bulk_descriptors()` (vendor interface, Bulk IN/OUT), `cdc_acm_descriptors()`
  (CDC-ACM: no driver needed on Linux/macOS/Windows 10+).

## Examples

| Example (`acorn.py --example`) | Host side                                                      |
|--------------------------------|----------------------------------------------------------------|
| `loopback`                     | `scripts/usb3_bulk.py loopback --size 1024 --count 1000`       |
| `bench`                        | `scripts/usb3_bench.py in\|out\|both [--csr-csv ...]`            |
| `cdc_uartbone`                 | `litex_server --uart --uart-port /dev/ttyACM0`, `litex_cli --regs` |

`usb3_pipe_gen` can also generate the PHY + core as a standalone Verilog core with AXI-Stream Bulk
endpoints (`usb_core: True`).

## Performance (Acorn CLE-215+, SFP2USB, xHCI host, Linux)

| Test                                    | Result                                     |
|-----------------------------------------|--------------------------------------------|
| Bulk IN (`usb3_bench.py in --no-check`) | 446 MB/s                                   |
| Bulk OUT (`usb3_bench.py out`)          | 449 MB/s                                   |
| Full duplex (`usb3_bench.py both`)      | IN 376 MB/s + OUT 400 MB/s                 |
| 3 min full duplex soak, data checked    | 139 GB, 0 errors, no LBAD/retry/Recovery   |
| CDC-ACM UARTBone                        | ~6000 CSR reads/s                          |

(5Gbps with 8b/10b: 500 MB/s of symbols, ~90% reached with 1024 bytes packets.)

## Limitations

- Gen 1x1 only (see `doc/gen2-x2-roadmap.md` for Gen 2x1/Gen 1x2).
- Bulk and Control endpoints (no Isochronous/Interrupt data, Streams). An Interrupt IN endpoint can be
  declared (descriptors) with a Bulk IN endpoint that never has data (CDC-ACM notifications).
- U1/U2 rejected (descriptors advertise 0 exit latencies: hosts disable LPM), U3 entry/exit through
  the LTSSM (not validated on hardware yet: the test host never suspends the device).
- Function remote wakeup, LTM, PTM not supported.
