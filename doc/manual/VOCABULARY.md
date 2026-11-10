# Vocabulary of the USB3-PIPE manual

Project-specific writing rules, completing the generic `STYLE.md` of the manual kit.

## Names of the parts

- *the PHY* is `usb3_pipe` (transceiver wrapper, PIPE datapath, LTSSM); *the device core* is
  `usb3_core` (link layer, protocol layer, endpoints, classes); *the project* is both.
- *the host* is the USB 3 host controller (xHCI) at the other end of the cable; *the link
  partner* is the port the PHY is connected to (the host or a hub); *the device* is the FPGA
  design.
- *transceiver* is the FPGA high-speed serial block (GTP, GTX, GTY, GTR12, DCU); "SerDes" only
  in class names (`USB3SerDes`, `A7USB3SerDes`).
- *target* is a script in `bench/`; *example* is a device design of `usb3_core/examples.py`.
- *link layer*, *protocol layer*, *physical layer* in lower case in prose; "Link Layer" only in
  a title or a class docstring.

## Protocol names

- USB 3 names follow the USB 3.2 specification: *Gen 1* and *Gen 2* (with a space), *Gen 1x1*,
  *Gen 2x1*, *Gen 1x2*; *SuperSpeed* and *SuperSpeedPlus* for the operating modes; link states
  as U0 to U3 in prose without code font; LTSSM states as *Polling.Active*, *Recovery.Idle*
  (dotted, capitalised, no code font); packet names in capitals (TS1, TS2, TSEQ, SKP, LFPS,
  LGOOD, LCRD, LUP, LDN, ACK, NRDY, ERDY, DP, ITP).
- Cite the specification as "USB 3.2 section 7.5.4".

## Placeholders

- `<target>` is a target script name without `.py` (`acorn`, `alibaba_xcku3p`); `<example>` is an
  example name (`loopback`, `bench`, `uvc`). Both are defined once in the Conventions of the
  front matter.

## Hardware names

- Boards: Acorn CLE-215+, KC705, Alibaba XCKU3P (then "the XCKU3P board"), Tang Mega 138K Pro,
  ECPIX-5, GW5AT-15.
- FPGA families: Artix-7, Kintex-7, Kintex UltraScale+, Gowin GW5AST and GW5AT, Lattice ECP5-5G.
