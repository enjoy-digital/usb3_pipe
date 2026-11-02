# AGENTS.md: USB3-PIPE

USB 3.2 Gen 1/Gen 2 PHY (PIPE, LTSSM, framing, scrambling) on FPGA transceivers with no external USB
chip, plus a native USB3 device core (link and protocol layers, endpoints, CDC-ACM/UVC/MSC classes).

Read the LiteX agent page first: https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions, how to
verify a change, contribute back). Index of the whole wiki for agents: https://github.com/enjoy-digital/litex/wiki/llms.txt.

## Where things are

- `usb3_pipe/`: PHY: `ltssm.py`, `lfps.py`, `training.py`, `datapath.py`, `framing.py`, `coding.py`, `scrambling.py`, `serdes.py`, `phy.py`, `gw5/`, `debug.py` (debug CSRs and LiteScope groups).
- `usb3_core/`: device core: link, protocol, endpoints, classes, Header Packet/Link traces; `host_model.py` (simulation host).
- `bench/`: hardware benches (Acorn, KC705, XCKU3P, Tang Mega 138K Pro) and host scripts; `doc/`: `usb3-pipe.md`, `usb3-core.md`, `examples.md`, `debugging-guide.md`, `gen2-x2-roadmap.md`.

## Verify a change

```sh
python3 -m pytest -v -n auto -m "not slow and not sim"            # fast suite (CI)
python3 -m pytest -v                                                # full suite (simulations, slower)
```

- Read `doc/debugging-guide.md` before hardware work: debug CSRs, LiteScope groups, traces.
- Never retrain a link from the debug CSRs while the device is in U3 (suspended).
- Slow fabrics (Gowin GW5): Fmax is the limiting factor; keep new logic registered and check timing
  on the GW5 target, not only on Xilinx.
- Gowin synthesis drops the init contents of memories with an asynchronous read port: use logic
  lookups or synchronous reads.

## Rules

- Style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md (match the
  surrounding file; `module: Description of the change.` commit titles; one change per commit).
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, the Verilog): change the Python.
- One topic per pull request, from the default development branch, with what was tested and on
  which hardware; say that an agent wrote it and who supervised. Do not rename options or
  reformat unrelated code.
- Hardware: never flash a board or write its SPI flash unless the user asked; `--load` (volatile)
  is the default for experiments.

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and `--ident` first, registers before
traces, narrow LiteScope captures). `litex_mcp` gives the same loop as MCP tools (CSRs by name,
LiteScope captures, builds with parsed timing, simulation): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.
