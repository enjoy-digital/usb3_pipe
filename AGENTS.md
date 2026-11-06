# AGENTS.md: USB3-PIPE

USB 3.2 Gen 1x1/Gen 2x1 PHY (PIPE, LTSSM, framing, scrambling) on FPGA transceivers with no external
USB chip, plus a native USB3 Device Core (Link and Protocol Layers, endpoints, CDC-ACM/UVC/MSC
classes).

Read the LiteX agent page first: https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions, how to
verify a change, contribute back). Index of the whole wiki for agents: https://github.com/enjoy-digital/litex/wiki/llms.txt.
Contributor rules (tests and markers, CI, coding style, commits/pull requests, hardware safety,
documentation of new options): [CONTRIBUTING.md](CONTRIBUTING.md).

## Where things are

- `usb3_pipe/`: PHY: `serdes.py`, `datapath.py`, `coding.py`, `scrambling.py`, `lfps.py`, `training.py`, `core.py` (USB3PIPE/Gen2PIPE), `framing.py` (Gen 2 packet framing, word pairs/quads), `ltssm.py`, `lanes.py`, `phy.py` (USB3PHY, Link interface), `crc.py`, `common.py`, `debug.py` (debug CSRs and LiteScope groups), `gen.py` (standalone core), `gw5/` (GTR12 configurations).
- `usb3_core/`: Device Core: `link.py`, `protocol.py`, `endpoints.py`/`bulk.py`/`iso.py`/`control.py`, device classes (`cdc_acm.py`, `uvc.py`, `msc.py`), `trace.py`, `examples.py` (example designs); `host_model.py` (simulation host).
- `bench/`: targets (Acorn, KC705, XCKU3P, Tang Mega 138K Pro), shared SoC `usb3_soc.py`, host scripts; `sim.py`: Verilator simulation; `test/`: pytest suite.
- `doc/`: `usb3-pipe.md`, `usb3-core.md`, `integration.md`, `targets.md`, `examples.md`, `standalone-core.md`, `debugging-guide.md`, `gen2-x2-roadmap.md`.

## Verify a change

```sh
python3 -m pytest -v -n auto -m "not slow and not sim"            # Migen tests (first CI step, no Verilator)
python3 -m pytest -v                                                # default suite with the Verilator simulations
python3 -m pytest -v -m slow                                        # long simulations (nightly CI)
```

Read `doc/debugging-guide.md` before hardware work (debug CSRs, LiteScope groups, traces); never
flash a board unless asked, never retrain a link in U3 ([CONTRIBUTING.md](CONTRIBUTING.md),
"Hardware").

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and `--ident` first, registers before
traces, narrow LiteScope captures). `litex_mcp` gives the same loop as MCP tools (CSRs by name,
LiteScope captures, builds with parsed timing, simulation): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.
