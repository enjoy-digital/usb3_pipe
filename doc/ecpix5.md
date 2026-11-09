# LambdaConcept ECPIX-5 (Lattice ECP5-5G)

USB3 Gen 1x1 (5Gbps) device on the [ECPIX-5](https://shop.lambdaconcept.com/home/46-ecpix-5.html)
(LFE5UM5G-85F) USB-C connector, with `ECP5USB3SerDes` on DCU0 and the open source toolchain
(Yosys/nextpnr-ecp5/Trellis). Needs [LiteICLink](https://github.com/enjoy-digital/liteiclink)
`SerDesECP5` 40-bit words support.

```sh
$ ./bench/lambdaconcept_ecpix5.py --build --load --example bench
$ litex_server --uart --uart-port /dev/ttyUSB1 --bind-port 1241
$ ./bench/usb3_bench.py both --transfer-size 65536 --port 1241 \
    --csr-csv build/lambdaconcept_ecpix5/csr.csv
```

## Results

Linux xHCI host (motherboard USB 3.2 root port), direct USB-C cable:

| | |
|---|---|
| Enumeration | SuperSpeed (5000M) |
| Bulk IN/OUT | 434/449 MB/s |
| Full duplex | 342 + 349 MB/s |
| Stability | 3 min of full duplex (~110GB), data checked: 0 errors, 0 Recovery, no RX errors/Electrical Idle in U0 |

Clocking: USB3 PIPE at 125MHz on the DCU TX clock, LTSSM/Link Layer/Core/SoC at 83.33MHz on 64-bit
Link Layer words (USB3PHY `pipe_cd`, asynchronous CDC) and Bulk `max_burst=16`. nextpnr margins
(seeds 1-3): sys 119-128MHz (83.33MHz), PIPE 156-166MHz (125MHz), RX 187-196MHz (125MHz), DCU
gearboxes >430MHz (250MHz). Other configurations measured (MB/s, IN/OUT, full duplex):

| | IN/OUT | Full duplex | Timings |
|---|---|---|---|
| PIPE/Core at 125MHz (`--sys-clk-freq 125e6`, 32-bit words) | 440/445 | 336 + 406 | sys 122-136MHz: some seeds only |
| Core at 62.5MHz (as GW5A), `max_burst=16` | 403/436 | 272 + 385 | sys ~2x |
| Core at 100MHz, `max_burst=16` | 421/448 | 372 + 306 | sys +24% |

No PHY debug on the ECP5 (timings): `USB3LinkStatus` instead (LTSSM state, U0/Recovery entries, U0
RX errors/Electrical Idle).

## ECP5 DCU

- LiteICLink `SerDesECP5` (`data_width=40`, `with_oob`): raw 10BSER channel, 8b/10b and comma
  alignment in the fabric, 20-bit words at 250MHz on the DCU clocks (1:2 gearbox) converted to
  40-bit words at 125MHz (`CLKDIVF` /2 clocks): only the gearboxes run at 250MHz (the ECP5 fabric
  doesn't reach 250MHz with the 8b/10b/alignment logic).
- 5Gbps needs a 250MHz reference clock (x20, fabric PLL from the 100MHz oscillator): the 100MHz
  EXTREF0 input (`serdes_clk100`) only reaches 2.5Gbps (x25 max, measured on the DCU TX clock).
- CDR phase detector always enabled (`CHX_PDEN_SEL`): disabled on LOS otherwise.
- Separate TX/RX initialization (LiteICLink `SerdesInitOOB`): TX up without RX signal (LFPS).
- USB3 RX settings (`ECP5USB3SerDes.rx_settings`) from [LUNA](https://github.com/greatscottgadgets/luna)'s ECP5 SerDes PHY: RX termination
  to GND, CDR/DCO calibration and lock window (`D_CDR_LOL_SET`), equalizer. The CDR/DCO settings are
  the RX fix with the host SSC (host TSEQ word errors: 1.4e-1 -> 4e-5..8e-4 per word in training).
- Avoid LUNA's TX driver/CMU loop settings without checks: after a test with them, the board DDR3 failed
  until a power cycle (not seen with the RX settings only).

## LFPS

TX: constant words through the SerDes (LFPS clock square wave) and Electrical Idle from the DCU
`FFC_EI_EN` input, as on the other transceivers. RX: the DCU loss of signal detector, filtered
(`RXIdleFilter`) and checked by the generic LFPS checkers. Measured: host Polling LFPS bursts of
1.11us every 9.43us detected, Polling LFPS handshake with the host, and no false Electrical Idle in
U0 with data. The DCU low data rate input (`rx_ldr`, raw single-ended RX input value) is available
as an alternative LFPS/activity source (bring-up captures). Note: the loss of signal detector also
reports the host Rx.Detect common mode steps as ~1ms signal periods (seen with the mux disconnected).

## USB-C

- SuperSpeed mux: enable pin active high (`usbc_mux en_n` in the platform), `amsel` high, polarity
  from the cable direction.
- CC controller: TI TUSB322 (I2C 0x47). Its ATTACHED_STATE/CABLE_DIR register (0x09) is polled by
  `TUSB322Reader` and drives the mux polarity (`--usbc-pol 0/1`: fixed). The `usbc_cfg` `dir` pin is
  not usable for the polarity.
- VBUS output (`usb_c_power`) kept off (device).

## Bring-up

`bench/lambdaconcept_ecpix5_bringup.py` builds small designs (seconds to build with nextpnr) used for
the progressive bring-up, still useful to check/tune the DCU:

1. `serdes`: LiteICLink `SerDesECP5` with PRBS, internal loopback (SCI `lb_ctl`, channel register
   0x15 bit 0), clocks frequency meters, raw RX words: PLL/reference clock checks, 0 errors over
   1.5e11 bits (PRBS7/15/31) at 5Gbps in loopback.
2. `dcu`: `SerDesECP5` 40-bit path in loopback (incrementing bytes, decode errors/sequence checks):
   0 errors over 1.3e10 symbols.
3. `lfps`: `ECP5USB3SerDes` with the LFPS Unit only (no LTSSM/Link Layer: no link training, safe for
   the host): host LFPS measurements, LOS/LDR captures, host TSEQ/TS1 RX monitor (data window
   decode errors, COM spacing, recovered clock frequency) used for the RX settings.
